#ifndef IOHC_BOARD_H
#define IOHC_BOARD_H

#define INTERRUPT_ATTR IRAM_ATTR

// platformio.ini also passes -DXIAO_C6_RF231, so define it only if the build
// has not already. Defining it unconditionally warns on every translation unit.
// Kept here as well so the header stands alone if compiled without the flag.
#ifndef XIAO_C6_RF231
#define XIAO_C6_RF231
#endif

/*
 * Radio selection.
 *
 * io-homecontrol is one link layer carried on two different PHYs. The upstream
 * stack targets the EU 868 MHz FSK band via an SX1276. VELUX INTEGRA units sold
 * with 2.4 GHz remotes (KLR 200) put the same link layer on an IEEE 802.15.4
 * O-QPSK PHY instead, which is what the AT86RF231 build below talks to.
 *
 * Everything above Radio:: is shared; only this file and the *Helpers driver
 * differ between the two.
 */
#if defined(XIAO_C6_RF231)
#define RADIO_AT86RF231
#endif

#if defined(RADIO_AT86RF231)
/* ------------------------------------------------------------------------
 * Seeed XIAO ESP32-C6 + AT86RF231 module -- io-homecontrol over 2.4 GHz
 * ------------------------------------------------------------------------
 * Pinout MEASURED on the rig (see rf231-sniffer/src/main.c). The module's
 * pads 3/4 are SLP_TR//RST, i.e. the REVERSE of what the ascending chip-pin
 * order predicts -- do not "fix" these back.
 *
 *   module pad -> XIAO pad -> GPIO
 *   1/2 VCC    -> 3V3   (NEVER 5V: 3.6 V absolute max)
 *   1/2 GND    -> GND
 *   3   SLP_TR -> D2  -> GPIO2
 *   4   /RST   -> D1  -> GPIO1
 *   5   SCLK   -> D8  -> GPIO19
 *   6   MISO   -> D9  -> GPIO20
 *   7   MOSI   -> D10 -> GPIO18
 *   8   /SEL   -> D3  -> GPIO21
 *   9   IRQ    -> D0  -> GPIO0
 */
#define RADIO_SCLK_PIN 19
#define RADIO_MISO_PIN 20
#define RADIO_MOSI_PIN 18
#define RADIO_CS_PIN 21
#define RADIO_RST_PIN 1
#define RADIO_SLP_TR_PIN 2
#define RADIO_IRQ_PIN 0
#define BOARD_LED_PIN 15  // XIAO ESP32-C6 user LED, active low

#define RADIO_MOSI RADIO_MOSI_PIN
#define RADIO_MISO RADIO_MISO_PIN
#define RADIO_SCLK RADIO_SCLK_PIN
#define RADIO_NSS RADIO_CS_PIN
#define RADIO_RESET RADIO_RST_PIN

/* The RF231 has a single IRQ line; causes are separated by reading IRQ_STATUS.
 * The names below keep iohcRadio's SX1276-era vocabulary: DIO0 is the one real
 * pin, DIO2 is absent and its ISR is not attached. */
#define RADIO_DIO0_PIN RADIO_IRQ_PIN
#define RADIO_DIO2_PIN (-1)
#define RADIO_PACKET_AVAIL RADIO_DIO0_PIN
#define RADIO_SYNCHRO_DETECTED RADIO_DIO0_PIN

/* Dupont jumpers get flaky well below the RF231's 7.5 MHz ceiling, and a
 * 127-byte frame at 2 MHz is still only ~0.5 ms of bus time. */
#define SPI_CLK_FRQ 2000000

#define BOARD_TCXO_WAKEUP_TIME 0
#define BOARD_READY_AFTER_POR 10000

/* Velux hops these three 802.15.4 channels (15/20/25). Frequencies are carried
 * through the stack in Hz exactly as on 868; the driver maps Hz -> channel. */
#define CHANNEL1 2425000000U  // 802.15.4 ch15
#define CHANNEL2 2450000000U  // 802.15.4 ch20
#define CHANNEL3 2475000000U  // 802.15.4 ch25

/* Park on ch20 (index 0): the remote's wake-up train and, per the protocol
 * source, every command go out on the middle channel. Measured 2026-09-14.
 * Widen to MAX_FREQS 3 once RX is validated and hopping is wanted. */
#define FREQS2SCAN {CHANNEL2, CHANNEL1, CHANNEL3}
#define MAX_FREQS 1

/* Velux's start-of-frame delimiter. Read directly off the air on 2026-09-14:
 * with SFD 0x00 the receiver syncs on the preamble and reports the transmitter's
 * real SFD as the frame length (0x60 / 0x56 depending on where it latched), and
 * with 0x56 every wake-up frame decodes with a valid FCS at LQI 255. The
 * standard 0xA7 never locks on this remote. */
#define IOHC_SFD 0x56

/* PHY parameters handed to Radio::setCarrier() by iohcRadio's constructor.
 * Deviation/bandwidth are meaningless for O-QPSK and ignored by the driver.
 * 250 kb/s = OQPSK_DATA_RATE 0, standard 802.15.4 spreading: Velux's wake-up
 * frames despread at LQI 255 with valid FCS at this rate (2026-09-14). The
 * earlier "500k" reading came from misframed captures. */
#define IOHC_BITRATE 250000
#define IOHC_DEVIATION 0
#define IOHC_BANDWIDTH 0

/* Velux's 2.4 GHz link wrapper. Every frame on air is
 *
 *     <type> 'i' 'o' <io-homecontrol frame> <CRC-16, LSB first>
 *
 * i.e. a type byte then the ASCII marker "io" (0x69 0x6F), then the ordinary
 * io-homecontrol frame the 868 MHz stack already speaks, then the CRC -- which
 * covers the wrapper as well. Decoded from a captured "close" command on
 * 2026-09-14 (captures/20260914-142712-cmdcap-sfd56-ch20-250k.log); the
 * alignment is fixed by CtrlByte2 = 0x20 (LPM set, matching the wake-up train
 * actually on air) and by a payload that is byte-for-byte what
 * iohcRemote1W's own RemoteButton::Close builds.
 *
 * Only 0x01 has ever been observed as the type byte, so RX matches on the "io"
 * marker alone and reports the type rather than requiring it. */
#define IOHC_LINK_TYPE 0x01
#define IOHC_LINK_MARK0 0x69
#define IOHC_LINK_MARK1 0x6F
#define IOHC_LINK_PREFIX_LEN 3

/* The SX1276 generates the io-homecontrol CRC in hardware; the RF231 cannot,
 * so the driver appends it in software from iohcCrypto::radioPacketComputeCrc.
 * Byte order on the wire is LSB first. RX used to accept either order while
 * that was still an open question; it is pinned now, because accepting both
 * doubles the false-accept rate on a band this crowded. See iohcCrcCheck(). */
#define IOHC_SW_CRC 1

#define SCAN_LED BOARD_LED_PIN
#define RX_LED SCAN_LED

#endif /* RADIO_AT86RF231 */
#endif /* IOHC_BOARD_H */
