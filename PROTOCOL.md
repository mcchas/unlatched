# The 2.4 GHz variant of io-homecontrol

How the wire format differs from the 868 MHz FSK band that other open-source
io-homecontrol work targets.

## The short version

**io-homecontrol is one link layer carried on two unrelated PHYs.** Everything
from the frame header upwards — addressing, the command set, the AES
challenge/response, the position encoding — is byte-for-byte the same on both.
Everything below it is different: different chip, different modulation,
different framing, and a small Velux-specific wrapper that does not exist on
868 at all.

That is why an 868 MHz tool cannot be pointed at a 2.4 GHz installation and
made to work by changing a frequency constant. The port is entirely beneath the
protocol.

---

## What is identical

Handed a decoded frame, code written for 868 MHz works unmodified:

```
CtrlByte1  CtrlByte2  target[3]  source[3]  cmd  payload[...]
```

- **CtrlByte1** — `MsgLen:5`, `Protocol:1` (the 1W/2W bit), `StartFrame:1`,
  `EndFrame:1`.
- **CtrlByte2** — `Version:2`, `Prio`, `Unk2`, `Unk3`, `LPM`, `Routed`,
  `Beacon`.
- **Addressing** — 3-byte node addresses, target before source.
- **The command set** — `0x00` execute, `0x03` private/status, `0x04` status
  reply, `0x28`–`0x33` discovery and key transfer, `0x3C`/`0x3D` challenge and
  answer, `0xFE` error.
- **The crypto** — same AES construction, same challenge/response, same key
  hierarchy.
- **Position encoding** — `0x0000` fully open, `0xC800` fully closed, with
  values above full scale as markers (`0xD200` stop, `0xD803` vent).

The driver hands the layer above it a bare io-homecontrol frame with no CRC and
`buffer_length = CtrlByte1.MsgLen + 1`, which is exactly what the 868 stack
produces. Everything above `Radio::` is shared between the two builds.

---

## What differs

| | 868 MHz (SX1276) | 2.4 GHz (AT86RF231) |
| --- | --- | --- |
| Modulation | FSK | IEEE 802.15.4 O-QPSK, 250 kb/s, standard spreading |
| Band | 868.95 MHz and neighbours | 802.15.4 channels 15 / 20 / 25 — 2425 / 2450 / 2475 MHz |
| Sync | FSK sync word | 802.15.4 SHR: four zero octets then an SFD |
| SFD | n/a | **`0x56`**, not the standard `0xA7` |
| Preamble | length is settable, and a long one is the wake-up | fixed by the standard; the wake-up is a train of frames instead |
| Length | io-home variable-length, from `CtrlByte1.MsgLen` | PHR octet, managed by the chip |
| Link wrapper | none | **3-byte `<type> 'i' 'o'` prefix** |
| io-home CRC | generated in hardware | appended and checked in software |
| 802.15.4 FCS | n/a | present in the standard, **never valid** on these frames |
| Channel use | one channel per command | the remote hops; replies must return on the channel they arrived on |

---

## 1. The start-of-frame delimiter

**Velux uses SFD `0x56`.** The 802.15.4 standard specifies `0xA7`, and with
`0xA7` the receiver never locks onto a KLR 200 at all.

On the RF231 this is one register, `SFD_VALUE`, and it can be written while
receiving — the correlator samples it per frame.

---

## 2. The link wrapper

Every 2.4 GHz frame carries three extra bytes in front of the io-homecontrol
frame and two behind it:

```
<type> 'i' 'o' <io-homecontrol frame> <CRC-16, LSB first>
  0x01  0x69 0x6F                        covers the wrapper too
```

The ASCII marker `io` is literal. Nothing equivalent exists on 868, where the
io-homecontrol frame sits directly on the PHY.

Reception gates on **the marker and the CRC only**. The type byte is reported
rather than required, because `0x01` is the only value ever observed — if a
second type exists, a frame carrying it will be surfaced instead of silently
dropped.

Consequences worth knowing:

- The shortest possible real frame is **14 bytes** of PSDU: 3 wrapper + 9 header
  + 2 CRC. Anything shorter cannot be a frame, which is what makes wake-up
  frames separable by length alone.
- The CRC covers the wrapper as well as the frame, so it cannot be computed
  before wrapping.

---

## 3. The CRC

Two different CRCs are in play, and only one of them matters.

**The io-homecontrol CRC** is the same 16-bit function as on 868, over the same
bytes plus the wrapper. The SX1276 generated it in hardware; the RF231 has no
equivalent, so the driver computes it on both paths. `TX_AUTO_CRC` is left off
deliberately: the chip would compute the same function over the same bytes, but
only ever the trailing two, and doing it in software keeps one implementation
that transmit and receive are checked against.

Byte order on the wire is **LSB first**.

**The 802.15.4 FCS** is never valid on these frames — Velux is not producing
one. That has a practical consequence on the RF231: `RX_SAFE_MODE` must stay
off, since in Basic mode it only protects frames with a valid FCS, and none of
ours qualify.

---

## 4. The wake-up train

Actuators are low-power and duty-cycle their receivers, so a bare command frame
is simply not heard. Both bands solve this, differently.

On 868 the wake-up is **a very long preamble** — the upstream stack sends
roughly a kilobyte of it — which works because FSK preamble length is a
settable radio parameter.

On 2.4 GHz the 802.15.4 SHR is fixed by the standard, so there is no preamble to
lengthen. Velux instead sends **a train of tiny frames**, one every
millisecond:

```
00 hh ll <CRC>          5-byte PSDU, no wrapper
   ^^^^^ milliseconds remaining, counting down
```

When `hh:ll` reaches zero, the command goes out. The KLR 200 sends **510 ms** of
this — 511 frames.

Two details that matter if you are implementing it:

- **These carry no link wrapper.** They are bare, which is why the transmit path
  needs a raw entry point alongside the wrapped one.
- **The cadence cannot be produced with a 1 ms RTOS delay.** A `vTaskDelay(1)`
  quantises to the tick and stretches the train to roughly double its length,
  past what the actuator will sit through. It has to be a busy-wait against a
  microsecond clock.

The countdown value also makes a receiver useful as a passive observer: it says
exactly how long until the command lands.

---

## 5. Channels, and which one to answer on

The 868 stack pins commands to a single channel. **The 2.4 GHz remote does
not.** An energy scan caught a KLR 200 alternating channel 25 with 15 or 20 at
roughly 500 ms per channel — one full wake-up train each — with channel 25
carrying about half of all airtime.

So a transmitter has to try more than one channel, and a receiver that parks on
one will hear a fraction of what is going on.

**A unicast reply must return on the channel its request arrived on.** This is
not a preference. An authentication answered on the wrong channel cannot be
heard however correct its contents, and the symptom — the actuator challenges
you, you answer, and nothing happens — looks exactly like a wrong key.

That makes the channel a property of the received frame rather than of the
receiver's configuration, so it has to be read back from the hardware at frame
time, not inferred from a scan list.

---

## 6. Smaller differences

- **ACEI byte.** `0x63` as the 2.4 GHz remote sends it, against `0x43`/`0x67`
  in the 868 stack. It is the third payload byte of an execute command.
- **Frame delivery.** The SX1276 streams bytes through a FIFO; the RF231 stages
  a whole frame in a buffer read in one burst. Anything written against a
  byte-at-a-time FIFO needs an adapter.
- **Interrupts.** The SX1276 offers DIO0 for payload/sent and DIO2 for sync
  detect. The RF231 has one IRQ line with the cause read back from
  `IRQ_STATUS`, so a single interrupt can carry several causes at once.
- **Link quality.** The RF231 reports LQI and a per-frame energy level latched
  at frame start; the SX1276 reports AFC correction, which has no analogue here.

---

## Where this lives in the code

The whole difference is absorbed in two files:

| File | Contains |
| --- | --- |
| `include/board_config.h` | pin map, channel frequencies, `IOHC_SFD`, the link wrapper constants, PHY parameters |
| `src/helpers/rf231_helpers.cpp` | the driver: SFD, wrapping and unwrapping, software CRC, the wake-up train |

`include/radio_if.h` defines the PHY-neutral contract they implement —
`readFrame`, `writeFrame`, `irqCause` — and everything above it, including the
entire io-homecontrol implementation, is shared.

---

## How this was established

Nearly all of it was read off the air rather than taken from documentation,
because no public documentation of the 2.4 GHz variant appears to exist.
SPI captures were not needed (or attempted as the traces were too small).

The method was an energy scan to find the channels in use, then an SFD sweep to
find the delimiter, then captures of a KLR 200 driving a known device — a
close command whose payload could be predicted from what the 868 stack builds
for the same button. That prediction is what fixed the frame alignment and
confirmed the wrapper: `CtrlByte2 = 0x20` with LPM set matched the wake-up
train actually on air, and the payload was byte-for-byte what the upstream
implementation produces.

**Confidence varies by claim**, and it is worth being explicit:

| Claim | Basis |
| --- | --- |
| SFD `0x56`, channels, O-QPSK at 250 kb/s | measured, reproducible |
| The wrapper's shape, and the CRC covering it | measured, and confirmed by transmitting frames that actuators accept |
| Wake-up train shape and cadence | decoded from the remote, and reproduced well enough to drive real devices |
| CRC byte order LSB first | measured; earlier builds accepted both while it was open |
| Link type `0x01` | only value ever seen. Others may exist |
| Reply-channel rule | measured against real actuators |

Where a finding informed the implementation, the source file says so at the
point of use, including the captures it came from.

---

## Disclaimer

This is an independent open-source project, not affiliated with, endorsed by, or
connected to VELUX, Somfy, or io-homecontrol. Trademarks are used only to
describe the hardware this software interoperates with.
