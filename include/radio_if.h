#ifndef IOHC_RADIO_IF_H
#define IOHC_RADIO_IF_H

#include <cstdint>

/*
    Radio-neutral part of the Radio:: contract.

    io-homecontrol runs over two very different PHYs, and the two transceivers
    disagree about almost everything below the frame:

                        SX1276 (868 FSK)          AT86RF231 (2.4 O-QPSK)
      frame delivery    streaming FIFO, byte      whole frame in a frame
                        at a time                 buffer, read in one burst
      interrupts        DIO0 payload/sent,        one IRQ line, causes read
                        DIO2 sync detect          back from IRQ_STATUS
      length            io-home variable-length   PHR octet, chip-managed
      io-home CRC       generated in hardware     must be appended in software

    The three calls below are where those differences are absorbed, so that
    iohcRadio and everything above it stay PHY-agnostic. Register-level helpers
    (readByte/writeBurst/...) still exist per driver for diagnostics, but the
    packet path only needs these.
*/
namespace Radio {
/// Why an interrupt fired. One event can carry several causes -- the RF231
/// reports RX_START and TRX_END on the same line, and a reader that arrives
/// late sees both at once -- so these are flags, not an enumeration.
enum IrqCause : uint8_t {
    IRQ_NONE = 0,
    IRQ_RX_DONE = 1 << 0,   ///< whole frame available (PayloadReady / TRX_END in RX)
    IRQ_TX_DONE = 1 << 1,   ///< transmission finished (PacketSent / TRX_END in TX)
    IRQ_SYNC_ON = 1 << 2,   ///< sync locked (SyncAddress rising / RX_START)
    IRQ_SYNC_OFF = 1 << 3,  ///< sync window closed (SyncAddress falling; SX1276 only)
};

/// Link stats for the frame most recently handed over by readFrame().
struct FrameStats {
    uint8_t airLength;  ///< length on air, BEFORE clamping to the caller's buffer
    float rssiDbm;
    uint8_t lqi;   ///< RF231 only; 0 on the SX1276
    double afcHz;  ///< SX1276 only; 0 on the RF231
    uint8_t snrDb;
    int8_t edDbm;  ///< RF231 only: energy at frame start (PHY_ED_LEVEL), latched
    ///< per frame, unlike rssiDbm which is live at read time
    uint8_t linkType;  ///< RF231 only: the type byte ahead of Velux's "io" marker
    ///< on 2.4 GHz. 0x01 is the only value seen so far.
    bool phyCrcOk;  ///< the PHY's own verdict. On 2.4 GHz that is the 802.15.4
    ///< FCS -- and Velux does use it: every frame validates (2026-09-14).
    bool iohcCrcOk;  ///< io-homecontrol's own CRC over the frame, when the
    ///< driver checks it (RF231). False on the SX1276, which
    ///< strips the CRC in hardware before we ever see it.
    ///< NOTE on 2.4 GHz this is NOT independent of phyCrcOk: the
    ///< 802.15.4 FCS and io-homecontrol's CRC are the same
    ///< function (reflected CCITT, poly 0x8408, init 0, LSB
    ///< first) over the same bytes, so the two always agree.
    ///< Velux did not add a CRC here -- it reused the PHY's.
};

/// Decode one interrupt into a set of IrqCause flags. @p source and @p edge
/// describe which pin fired and at what level; the RF231 has a single line
/// and ignores both.
uint8_t irqCause(uint8_t source, uint8_t edge);

/// Copy the received frame out. Returns the number of bytes copied, clamped
/// to @p maxLen. lastFrameStats().airLength carries the true on-air length,
/// so an over-long frame (the neighbouring Thread network is full of them)
/// is distinguishable from a genuinely short one and can be dropped.
uint8_t readFrame(uint8_t *out, uint8_t maxLen);

/// Hand one complete io-homecontrol frame to the PHY and start sending.
/// @p len excludes the io-homecontrol CRC: the driver appends it where the
/// PHY cannot generate it itself (see IOHC_SW_CRC in board_config.h).
bool writeFrame(const uint8_t *in, uint8_t len);

const FrameStats &lastFrameStats();
}  // namespace Radio

#endif  // IOHC_RADIO_IF_H
