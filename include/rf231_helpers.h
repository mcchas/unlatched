#ifndef RF231HELPERS_H
#define RF231HELPERS_H

#include <cstdint>
#include <cstddef>
#include <radio_if.h>

#if defined(ESP32)
#include "mbedtls/aes.h"
#endif

/*
    AT86RF231 driver presenting the same Radio:: surface as SX1276Helpers, so
    the io-homecontrol stack above it is unchanged.

    The RF231 runs the IEEE 802.15.4 O-QPSK PHY in Basic Operating Mode: the
    chip does the SHR, the PHR and the modulation, and hands over whole frames
    at TRX_END "even if the third level filter rules do not match or the FCS is
    not valid". That last part is the reason this chip was chosen -- Velux's
    frames never satisfy the 802.15.4 FCS (io-homecontrol brings its own CRC),
    so any receiver that enforces the FCS in hardware discards all of them.
*/

// ---- SPI command byte encoding (datasheet Table 6-2) ----
#define RF231_CMD_REG_READ(a) (0x80 | ((a) & 0x3F))
#define RF231_CMD_REG_WRITE(a) (0xC0 | ((a) & 0x3F))
#define RF231_CMD_FB_READ 0x20
#define RF231_CMD_FB_WRITE 0x60

// ---- Registers ----
#define RG_TRX_STATUS 0x01
#define RG_TRX_STATE 0x02
#define RG_TRX_CTRL_0 0x03
#define RG_TRX_CTRL_1 0x04
#define RG_PHY_TX_PWR 0x05
#define RG_PHY_RSSI 0x06
#define RG_PHY_ED_LEVEL 0x07
#define RG_PHY_CC_CCA 0x08
#define RG_CCA_THRES 0x09
#define RG_RX_CTRL 0x0A
#define RG_SFD_VALUE 0x0B
#define RG_TRX_CTRL_2 0x0C
#define RG_ANT_DIV 0x0D
#define RG_IRQ_MASK 0x0E
#define RG_IRQ_STATUS 0x0F
#define RG_VREG_CTRL 0x10
#define RG_BATMON 0x11
#define RG_XOSC_CTRL 0x12
#define RG_RX_SYN 0x15
#define RG_XAH_CTRL_1 0x17
#define RG_FTN_CTRL 0x18
#define RG_PLL_CF 0x1A
#define RG_PLL_DCU 0x1B
#define RG_PART_NUM 0x1C
#define RG_VERSION_NUM 0x1D
#define RG_MAN_ID_0 0x1E
#define RG_MAN_ID_1 0x1F
#define RG_CSMA_SEED_1 0x2E

// ---- TRX_CMD values written to RG_TRX_STATE ----
#define TRX_CMD_NOP 0x00
#define TRX_CMD_TX_START 0x02
#define TRX_CMD_FORCE_TRX_OFF 0x03
#define TRX_CMD_FORCE_PLL_ON 0x04
#define TRX_CMD_RX_ON 0x06
#define TRX_CMD_TRX_OFF 0x08
#define TRX_CMD_PLL_ON 0x09

// ---- TRX_STATUS values ----
#define TRX_STATUS_P_ON 0x00
#define TRX_STATUS_BUSY_RX 0x01
#define TRX_STATUS_BUSY_TX 0x02
#define TRX_STATUS_RX_ON 0x06
#define TRX_STATUS_TRX_OFF 0x08
#define TRX_STATUS_PLL_ON 0x09
#define TRX_STATUS_SLEEP 0x0F
#define TRX_STATUS_STATE_TRANSITION 0x1F
#define TRX_STATUS_MASK 0x1F

// ---- IRQ_STATUS / IRQ_MASK bits ----
#define RF231_IRQ_PLL_LOCK (1 << 0)
#define RF231_IRQ_PLL_UNLOCK (1 << 1)
#define RF231_IRQ_RX_START (1 << 2)
#define RF231_IRQ_TRX_END (1 << 3)
#define RF231_IRQ_CCA_ED_DONE (1 << 4)
#define RF231_IRQ_AMI (1 << 5)
#define RF231_IRQ_TRX_UR (1 << 6)
#define RF231_IRQ_BAT_LOW (1 << 7)

// TRX_CTRL_1 bit 5. Left OFF: with automatic CRC the chip would overwrite our
// last two frame-buffer bytes with an 802.15.4 FCS, and io-homecontrol carries
// its own CRC there instead.
#define RF231_TX_AUTO_CRC_ON (1 << 5)

// PHY_RSSI bit 7. The 802.15.4 FCS verdict -- meaningless for io-homecontrol
// traffic, kept only so the diagnostics can show it.
#define RF231_RX_CRC_VALID (1 << 7)

// Frame buffer read length = 3 + frame_length (PHY_STATUS, PHR, PSDU, LQI).
// NOTE: the RF233 appends three trailing bytes instead (5 + len); using that
// layout here reads past the frame and yields garbage.
#define RF231_FB_MAX (3 + 127)
#define RF231_PSDU_MAX 127

// RSSI_BASE_VAL (datasheet s8.4): P[dBm] = -91 + 3*(RSSI-1), RSSI 1..28.
#define RF231_RSSI_BASE_VAL (-91)

// 802.15.4 channel 11 sits at 2405 MHz, spaced 5 MHz apart.
#define RF231_CH_BASE_HZ 2405000000U
#define RF231_CH_SPACING_HZ 5000000U
#define RF231_CH_MIN 11
#define RF231_CH_MAX 26

namespace Radio {
/// Kept identical to the SX1276 driver's enum so callers are portable.
/// On this PHY only Frequency and Bitrate are real: O-QPSK deviation and
/// channel bandwidth are fixed by the standard and those calls are ignored.
enum class Carrier { Frequency, Deviation, Bandwidth, Bitrate, Modulation };

enum Modulation : uint32_t { OOK = 0x00, FSK, LoRa };

struct regBandWidth {
    uint8_t Mant;
    uint8_t Exp;
};

void initHardware();
void initRegisters(uint8_t maxPayloadLength);
void calibrate();
void setStandby();
void setTx();
void setRx();
void setPreambleLength(uint16_t preambleLen);
void clearBuffer();
void clearFlags();
bool preambleDetected();
bool syncedAddress();
bool dataAvail();
bool crcOk();

uint8_t readByte(uint8_t regAddr);
void readBytes(uint8_t regAddr, uint8_t *out, uint8_t len);
void writeByte(uint8_t regAddr, uint8_t data);
void writeBurst(uint8_t regAddr, uint8_t *in, uint8_t len);
uint16_t readWord(uint8_t regAddr);
void writeWord(uint8_t regAddr, uint16_t value);

bool inStdbyOrSleep();
bool setCarrier(Carrier param, uint32_t value);
void dump();
void dumpReal();

int32_t getFrequencyError();
int16_t getAFCError();

// ---- RF231-specific extras, used by the diagnostics commands ----

/// 802.15.4 channel currently tuned (11..26).
uint8_t channel();

/// OQPSK_DATA_RATE currently set (0 = 250k, 1 = 500k, 2 = 1M, 3 = 2M).
uint8_t dataRate();

/// Peak RSSI in dBm sampled over @p ms milliseconds on the current channel.
int peakRssiDbm(uint32_t ms);

/// True once PART_NUM identified a genuine RF231/RF233 on the SPI bus.
bool present();

/// Receiver desensitisation (RX_SYN.RX_PDT_LEVEL, reg 0x15 bits 3:0).
/// 0 = full sensitivity; each step raises the detection threshold by 3 dB,
/// to -48 dB at 15. The band here is shared with WiFi, Thread, Zigbee and
/// BLE, and 802.15.4 channels 15 and 20 sit under WiFi 6 and 11 — ambient
/// alone pegs the RSSI ceiling. Turning the receiver deaf to everything but
/// a transmitter held against the antenna is the only way to attribute a
/// frame to the remote with confidence.
void setSensitivity(uint8_t pdtLevel);
uint8_t sensitivity();

/// Start-of-frame delimiter the receiver hunts for (RG_SFD_VALUE, 0x0B).
/// 0xA7 is the IEEE 802.15.4 standard value. It is programmable on both the
/// RF231 and the RF233 in the remote, which makes it the one PHY parameter a
/// vendor can change while still using stock silicon -- the cheapest way to
/// run a private network that standard receivers ignore. Velux uses 0x56
/// (IOHC_SFD in board_config.h, read off the air 2026-09-14).
void setSfd(uint8_t sfd);
uint8_t sfd();

/// Velux precedes every command with a ~510 ms train of 5-byte wake-up
/// frames, one per millisecond, each carrying a millisecond countdown to the
/// command. The driver recognises them (len 5, first byte 0x00, valid FCS),
/// never hands them up, and with tracing on prints ONE summary line per
/// burst -- printing each would blind the UART for the very 20 ms in which
/// the command arrives. traceWake(true) prints them individually anyway.
void traceWake(bool on);
uint32_t wakeCount();

/// Sweep every SFD value 0x00-0xFF on-chip, advancing only while the band
/// is actually busy.
///
/// Host-driven sweeping cannot do this job: the firmware polls serial on a
/// 500 ms ticker so rapid `rfsfd` writes get coalesced and silently dropped,
/// and the operator transmits only intermittently, so a wall-clock sweep
/// spends most of its dwells on silence. Measured: one 256-value host sweep
/// landed only 204 distinct values and gave just 61 of them any live
/// traffic.
///
/// Here each value is held until it has accumulated @p liveMs of energy
/// above the detection threshold, so the sweep pauses by itself between
/// button presses and every value gets equal real exposure.
/// @p first/@p last bound the range, so values that timed out for lack of
/// traffic can be re-run without repeating the whole space.
/// @p rssiBusy is the raw PHY_RSSI step counted as "transmitting"
/// (dBm = -91 + 3*(raw-1)); 17 = -43 dBm, measured to capture ~75% of this
/// remote's output while staying above the deafened ambient floor.
void sfdScan(uint16_t liveMs, uint16_t timeoutMs, uint8_t first = 0x00, uint8_t last = 0xFF,
             uint8_t rssiBusy = 17);

/// Round-robin energy scan over Velux's three channels plus two it never
/// uses, sampling RSSI instead of trying to demodulate.
///
/// Answers what frame capture cannot: whether a transmitter is on air AT
/// ALL. Listening confuses "nothing was sent" with "sent in a framing we
/// cannot sync to" -- on 2026-09-14 the KLR 200's "send a copy" procedure
/// produced zero frames on ch15, ch20 AND ch25 at SFD 0x56, which is equally
/// consistent with a silent donor and with one using a different SFD or
/// rate. Energy detection needs no SFD, no rate and no framing, so it
/// distinguishes them.
///
/// The channel register re-locks the PLL in ~150 us, so every channel is
/// revisited within a millisecond or so -- ample for the 510 ms wake-up
/// train, and good odds on a lone frame. The unused channels are the
/// control: broadband interference lifts all five together, while a
/// transmission lifts only Velux's, which is how tools/rf231_offchan.py
/// ruled out touchscreen EMI. @p rssiBusy is the raw PHY_RSSI step counted
/// as busy (dBm = -91 + 3*(raw-1)). Any serial input aborts.
void channelScan(uint32_t ms, uint8_t rssiBusy = 17);

/// Transmit Velux's wake-up train: a 5-byte frame every millisecond for
/// @p ms milliseconds, each carrying a countdown to the command that follows
/// it. Required before any command to a low-power target -- the skylight
/// sleeps, and a bare command frame is not heard. Blocks for @p ms.
void sendWakeupTrain(uint16_t ms);

/// Print every frame the PHY hands over, in the same format the standalone
/// rf231-sniffer used, so captures stay directly comparable. Traced before
/// the length clamp, so frames too long for an io-homecontrol packet (the
/// neighbouring Thread network is full of them) still show up.
void traceFrames(bool on);

/// Emission-free check that the IRQ line reaches the ESP32: a TRX_OFF ->
/// PLL_ON transition pulses IRQ via PLL_LOCK with the PA still off.
bool irqSelfTest();

/// One-line health report: chip state, channel, rate, live RSSI, and the
/// interrupt/frame counters. Distinguishes "nothing on air" from "frames
/// arriving but discarded".
void status();
}  // namespace Radio

#endif  // RF231HELPERS_H
