#include <board_config.h>

#if defined(RADIO_AT86RF231)

#include <Arduino.h>
#include <rf231_helpers.h>
#include <iohc_crypto_helpers.h>

#include <atomic>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

namespace Radio {
static spi_device_handle_t s_spi = nullptr;

// Every SPI transaction below runs under this. The interrupt task reads
// IRQ_STATUS and the frame buffer while the command task may be doing
// rfstat/rfch/dump, and spi_device_polling_transmit() is not safe for
// concurrent use on one device -- two overlapping transactions corrupt each
// other. That is not merely a bad read: losing the IRQ_STATUS read leaves
// the RF231's IRQ line asserted forever, so the POSEDGE interrupt never
// fires again and the receiver goes permanently deaf.
// Recursive because writeFrame() -> setTx() -> readByte() nests.
static SemaphoreHandle_t s_spiMux = nullptr;

struct SpiLock {
    SpiLock() {
        if (s_spiMux) xSemaphoreTakeRecursive(s_spiMux, portMAX_DELAY);
    }
    ~SpiLock() {
        if (s_spiMux) xSemaphoreGiveRecursive(s_spiMux);
    }
};

// ESP-IDF SPI DMA needs word-aligned, DMA-capable buffers. Stack arrays are
// neither, which silently corrupts long transfers -- allocate once instead.
static uint8_t *s_fbTx = nullptr;
static uint8_t *s_fbRx = nullptr;

// iohcRadio configures registers before it sets a carrier, so these have to
// start at something the chip will accept -- channel 0 is not a valid
// 802.15.4 channel. iohcRadio::start() overwrites both from scan_freqs[0]
// and IOHC_BITRATE a moment later.
static uint8_t s_channel = 20;  // 802.15.4 channel 11..26; 20 = 2450 MHz, Velux's command channel
static uint8_t s_rate =
    0;  // OQPSK_DATA_RATE; 0 = 250 kb/s -- Velux wake-ups despread here at LQI 255
static bool s_present = false;

// TRX_END means "frame received" in the RX states and "frame sent" in the
// TX ones, but by the time the IRQ is serviced the chip is already back in
// PLL_ON either way -- so remember which one we asked for.
static volatile bool s_txPending = false;

static FrameStats s_stats = {};

// Observability. Without these there is no way to tell a quiet band from a
// receive path that is firing but discarding everything. Bumped from the
// interrupt task and read from the command task, hence atomic -- relaxed is
// enough, nothing is ordered against them.
static std::atomic<uint32_t> s_irqCount{0};
static std::atomic<uint32_t> s_rxStartCount{0};
static std::atomic<uint32_t> s_frameCount{0};
static std::atomic<uint32_t> s_badPhrCount{0};
// Frames that synced and passed the PHR check but were not handed up: no
// "io" marker, or a failing CRC. Counted separately because the two causes
// look identical here but are not: a neighbour's traffic, or one of Velux's
// own frames with a symbol error. Both have been seen -- in the 2026-09-14
// capture, 4 of 46 command frames were the latter, arriving at the remote's
// own -10 dBm with a single byte wrong inside the "io" marker. So a rising
// count is not by itself evidence of foreign traffic; compare with rfstat's
// wake-up counters and the trace, which still shows the raw bytes.
static std::atomic<uint32_t> s_notOursCount{0};
static bool s_trace = false;

// How much of the last frame the caller has taken, so that dataAvail() can
// answer the way the SX1276's streaming FIFO would.
static uint8_t s_rxLen = 0;
static uint8_t s_rxPos = 0;

// Velux's 2.4 GHz wake-up train, measured 2026-09-14 (captures/20260914-
// 113351-sfdread-60.log, 546/546 FCS-valid): before every command the remote
// sends a 5-byte frame every 1.0 ms for ~510 ms -- 00 hh ll + FCS, hh:ll
// counting down in milliseconds to the command frame. It is the RF233's
// stand-in for the 1024-byte wake-up preamble the 868 MHz stack sends to
// duty-cycled receivers. Printing each one takes ~20 ms of UART time, which
// is exactly the blind window that hid the command frame in all 22 recorded
// bursts. So they are counted and summarised per burst, never handed up, and
// printed individually only on request (rfwu 1).
static std::atomic<uint32_t> s_wakeCount{0};
static std::atomic<uint32_t> s_wakeBadCount{0};  // wake-up slots lost to chip errors
// TX has never been verified over the air, so count both ends of it: frames
// we handed to the PHY, and TRX_END interrupts saying the chip finished
// keying. txDone lagging tx means the state machine is not completing --
// a different fault from "transmitted but nobody answered", and the two are
// otherwise indistinguishable from this side of the radio.
static std::atomic<uint32_t> s_txCount{0};
static std::atomic<uint32_t> s_txDoneCount{0};
static bool s_traceWake = false;
static portMUX_TYPE s_wuMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_wuBurstN = 0;                   // wake-ups in the current burst
static uint16_t s_wuFirstCd = 0, s_wuLastCd = 0;  // countdown at its first/last frame
static int64_t s_wuFirstUs = 0, s_wuLastUs = 0;
static constexpr int64_t WU_BURST_GAP_US =
    50 * 1000;  // frames are 1 ms apart; 50 ms of silence ends a burst

// Trace lines are queued and printed by a low-priority task so the radio
// task never blocks on the UART. A 200-character line is ~20 ms at 115200,
// and anything on air in that time is lost -- see above for what that cost.
struct TraceEntry {
    bool summary;     // a burst summary rather than a frame
    int64_t tUs;      // receive time (summary: first wake-up of the burst)
    int64_t tLastUs;  // summary: last wake-up of the burst
    uint32_t wuN;     // wake-ups in the burst this frame follows (0 = none recent)
    uint16_t wuFirstCd, wuLastCd;
    int32_t wuDtMs;  // frame: ms since the last wake-up, -1 if none recent
    uint8_t len, lqi;
    int8_t rssi, ed;
    bool fcsOk, iohcOk;
    uint8_t psdu[RF231_PSDU_MAX];
};
static QueueHandle_t s_traceQ = nullptr;

static void traceEmit(const TraceEntry &e) {
    if (!s_traceQ) return;
    (void)xQueueSend(s_traceQ, &e, 0);  // never wait; dropping a line beats missing a frame
}

/// Caller holds s_wuMux. Packages the current burst and clears it.
static void burstSummaryLocked(TraceEntry &e) {
    e = {};
    e.summary = true;
    e.tUs = s_wuFirstUs;
    e.tLastUs = s_wuLastUs;
    e.wuN = s_wuBurstN;
    e.wuFirstCd = s_wuFirstCd;
    e.wuLastCd = s_wuLastCd;
    s_wuBurstN = 0;
}

static void noteWakeup(const uint16_t cd, const int64_t nowUs) {
    s_wakeCount.fetch_add(1, std::memory_order_relaxed);
    TraceEntry prev;
    bool havePrev = false;
    taskENTER_CRITICAL(&s_wuMux);
    if (s_wuBurstN && nowUs - s_wuLastUs > WU_BURST_GAP_US) {
        burstSummaryLocked(prev);  // a new burst started; report the one before it
        havePrev = true;
    }
    if (!s_wuBurstN) {
        s_wuFirstCd = cd;
        s_wuFirstUs = nowUs;
    }
    s_wuBurstN++;
    s_wuLastCd = cd;
    s_wuLastUs = nowUs;
    taskEXIT_CRITICAL(&s_wuMux);
    if (havePrev && s_trace) traceEmit(prev);
}

static void traceFrame(const uint8_t *psdu, const uint8_t len, const int64_t nowUs) {
    TraceEntry e = {};
    TraceEntry burst;
    bool haveBurst = false;
    taskENTER_CRITICAL(&s_wuMux);
    if (s_wuLastUs && nowUs - s_wuLastUs < 5 * 1000 * 1000) {
        e.wuDtMs = static_cast<int32_t>((nowUs - s_wuLastUs) / 1000);
        e.wuLastCd = s_wuLastCd;
        e.wuN = s_wuBurstN;
    } else {
        e.wuDtMs = -1;
    }
    if (s_wuBurstN) {
        burstSummaryLocked(burst);
        haveBurst = true;
    }  // the burst is over: this is what it was for
    taskEXIT_CRITICAL(&s_wuMux);
    e.summary = false;
    e.tUs = nowUs;
    e.len = len;
    e.lqi = s_stats.lqi;
    e.rssi = static_cast<int8_t>(s_stats.rssiDbm);
    e.ed = s_stats.edDbm;
    e.fcsOk = s_stats.phyCrcOk;
    e.iohcOk = s_stats.iohcCrcOk;
    memcpy(e.psdu, psdu, len);
    if (haveBurst) traceEmit(burst);
    traceEmit(e);
}

static void tracePrint(const TraceEntry &e) {
    if (e.summary) {
        ets_printf(
            "WAKEUP burst n=%lu cd=0x%04X->0x%04X span=%lldms t_first=%lld t_last=%lld "
            "cmd_due=+%ums\n",
            static_cast<unsigned long>(e.wuN), e.wuFirstCd, e.wuLastCd, (e.tLastUs - e.tUs) / 1000,
            e.tUs / 1000, e.tLastUs / 1000, e.wuLastCd);
        return;
    }
    static const char D[] = "0123456789ABCDEF";
    char hex[2 * RF231_PSDU_MAX + 1];
    for (uint8_t i = 0; i < e.len; i++) {
        hex[2 * i] = D[e.psdu[i] >> 4];
        hex[2 * i + 1] = D[e.psdu[i] & 0x0F];
    }
    hex[2 * e.len] = '\0';
    // Field order is what tools/*.py parse; the wake-up context is appended
    // so existing captures and parsers stay valid.
    if (e.wuDtMs >= 0)
        ets_printf(
            "FRAME t=%lld len=%u lqi=%u rssi=%ddBm ed=%ddBm fcs=%s iohc=%s %s wu=%lu cd_last=%u "
            "wu_dt=%ldms\n",
            e.tUs / 1000, e.len, e.lqi, e.rssi, e.ed, e.fcsOk ? "OK" : "BAD",
            e.iohcOk ? "OK" : "BAD", hex, static_cast<unsigned long>(e.wuN), e.wuLastCd,
            static_cast<long>(e.wuDtMs));
    else
        ets_printf("FRAME t=%lld len=%u lqi=%u rssi=%ddBm ed=%ddBm fcs=%s iohc=%s %s\n",
                   e.tUs / 1000, e.len, e.lqi, e.rssi, e.ed, e.fcsOk ? "OK" : "BAD",
                   e.iohcOk ? "OK" : "BAD", hex);
}

static void tracePump(void *) {
    TraceEntry e;
    for (;;) {
        if (xQueueReceive(s_traceQ, &e, pdMS_TO_TICKS(200)) == pdTRUE) {
            tracePrint(e);
            continue;
        }
        // Quiet for 200 ms. A burst whose command frame we did not catch would
        // otherwise only be reported when the next burst begins -- or never.
        bool have = false;
        taskENTER_CRITICAL(&s_wuMux);
        if (s_wuBurstN && esp_timer_get_time() - s_wuLastUs > 500 * 1000) {
            burstSummaryLocked(e);
            have = true;
        }
        taskEXIT_CRITICAL(&s_wuMux);
        if (have && s_trace) tracePrint(e);
    }
}

// ---------------------------------------------------------------- SPI access

uint8_t IRAM_ATTR readByte(const uint8_t regAddr) {
    SpiLock lock;
    uint8_t tx[2] = {static_cast<uint8_t>(RF231_CMD_REG_READ(regAddr)), 0x00};
    uint8_t rx[2] = {0, 0};
    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    if (spi_device_polling_transmit(s_spi, &t) != ESP_OK) return 0;
    return rx[1];  // rx[0] is PHY_STATUS
}

void IRAM_ATTR writeByte(const uint8_t regAddr, const uint8_t data) {
    SpiLock lock;
    uint8_t tx[2] = {static_cast<uint8_t>(RF231_CMD_REG_WRITE(regAddr)), data};
    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = nullptr;
    spi_device_polling_transmit(s_spi, &t);
}

/// Register-block read. The RF231 has no burst-read mode for registers, so
/// this walks them one at a time; only diagnostics use it.
void IRAM_ATTR readBytes(const uint8_t regAddr, uint8_t *out, const uint8_t len) {
    for (uint8_t i = 0; i < len; ++i) out[i] = readByte(regAddr + i);
}

void IRAM_ATTR writeBurst(const uint8_t regAddr, uint8_t *in, const uint8_t len) {
    for (uint8_t i = 0; i < len; ++i) writeByte(regAddr + i, in[i]);
}

uint16_t IRAM_ATTR readWord(const uint8_t regAddr) {
    return static_cast<uint16_t>(readByte(regAddr + 1) << 8) | readByte(regAddr);
}

void IRAM_ATTR writeWord(const uint8_t regAddr, const uint16_t value) {
    writeByte(regAddr, value & 0xFF);
    writeByte(regAddr + 1, value >> 8);
}

/// Pulls the whole frame buffer into s_fbRx in one transaction, laid out as
/// PHY_STATUS, PHR, PSDU..., LQI. Overreading past the frame is harmless --
/// the address counter just advances -- and the PHR gives the real length.
/// Reads into the DMA buffer directly; callers index s_fbRx.
static bool IRAM_ATTR frameBufferRead(const size_t n) {
    SpiLock lock;
    memset(s_fbTx, 0x00, n);
    s_fbTx[0] = RF231_CMD_FB_READ;
    spi_transaction_t t = {};
    t.length = n * 8;
    t.tx_buffer = s_fbTx;
    t.rx_buffer = s_fbRx;
    if (spi_device_polling_transmit(s_spi, &t) != ESP_OK) {
        memset(s_fbRx, 0, n);
        return false;
    }
    return true;
}

/// Frame buffer write: command byte, PHR (PSDU length), then the PSDU.
static void IRAM_ATTR frameBufferWrite(const uint8_t *psdu, const uint8_t psduLen) {
    SpiLock lock;
    s_fbTx[0] = RF231_CMD_FB_WRITE;
    s_fbTx[1] = psduLen;
    memcpy(s_fbTx + 2, psdu, psduLen);
    spi_transaction_t t = {};
    t.length = static_cast<size_t>(psduLen + 2) * 8;
    t.tx_buffer = s_fbTx;
    t.rx_buffer = nullptr;
    spi_device_polling_transmit(s_spi, &t);
}

// ------------------------------------------------------------ state control

static const char *stateName(const uint8_t s) {
    switch (s & TRX_STATUS_MASK) {
        case TRX_STATUS_P_ON: return "P_ON";
        case TRX_STATUS_BUSY_RX: return "BUSY_RX";
        case TRX_STATUS_BUSY_TX: return "BUSY_TX";
        case TRX_STATUS_RX_ON: return "RX_ON";
        case TRX_STATUS_TRX_OFF: return "TRX_OFF";
        case TRX_STATUS_PLL_ON: return "PLL_ON";
        case TRX_STATUS_SLEEP: return "SLEEP";
        case TRX_STATUS_STATE_TRANSITION: return "STATE_TRANSITION";
        default: return "?";
    }
}

/// Drives TRX_STATE and waits for the chip to actually get there. Returns
/// false on timeout rather than letting a stuck PLL look like success.
static bool IRAM_ATTR forceState(const uint8_t cmd, const uint8_t wantStatus) {
    writeByte(RG_TRX_STATE, cmd);
    for (int i = 0; i < 200; ++i) {  // 200 x 50us = 10ms, PLL settles in ~180us
        if ((readByte(RG_TRX_STATUS) & TRX_STATUS_MASK) == wantStatus) return true;
        ets_delay_us(50);
    }
    return false;
}

void IRAM_ATTR setStandby() {
    forceState(TRX_CMD_FORCE_TRX_OFF, TRX_STATUS_TRX_OFF);
}

/// Prepare to transmit. The frame itself goes out from writeFrame(), which
/// re-checks the state, so calling this first is an optimisation not a
/// requirement.
void IRAM_ATTR setTx() {
    const uint8_t st = readByte(RG_TRX_STATUS) & TRX_STATUS_MASK;
    if (st == TRX_STATUS_PLL_ON) return;
    // FORCE_PLL_ON is the only transition allowed straight out of RX_ON and
    // the BUSY_* states; TRX_OFF needs the plain PLL_ON command.
    if (st == TRX_STATUS_TRX_OFF)
        forceState(TRX_CMD_PLL_ON, TRX_STATUS_PLL_ON);
    else
        forceState(TRX_CMD_FORCE_PLL_ON, TRX_STATUS_PLL_ON);
}

void IRAM_ATTR setRx() {
    s_txPending = false;
    const uint8_t st = readByte(RG_TRX_STATUS) & TRX_STATUS_MASK;
    if (st == TRX_STATUS_RX_ON || st == TRX_STATUS_BUSY_RX) return;
    forceState(TRX_CMD_RX_ON, TRX_STATUS_RX_ON);
}

bool IRAM_ATTR inStdbyOrSleep() {
    const uint8_t st = readByte(RG_TRX_STATUS) & TRX_STATUS_MASK;
    return st == TRX_STATUS_TRX_OFF || st == TRX_STATUS_SLEEP || st == TRX_STATUS_P_ON;
}

// ------------------------------------------------------- init and calibration

void initHardware() {
    ets_printf("\nAT86RF231 SPI init\n");

    gpio_config_t out = {};
    out.pin_bit_mask = (1ULL << RADIO_RST_PIN) | (1ULL << RADIO_SLP_TR_PIN);
    out.mode = GPIO_MODE_OUTPUT;
    out.pull_up_en = GPIO_PULLUP_DISABLE;
    out.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&out);

    // The IRQ line idles low and pulses high; pull down so a floating pin
    // cannot look like a permanent interrupt.
    gpio_config_t in = {};
    in.pin_bit_mask = (1ULL << RADIO_IRQ_PIN);
    in.mode = GPIO_MODE_INPUT;
    in.pull_up_en = GPIO_PULLUP_DISABLE;
    in.pull_down_en = GPIO_PULLDOWN_ENABLE;
    in.intr_type = GPIO_INTR_DISABLE;  // iohcRadio attaches the handler later
    gpio_config(&in);

    spi_bus_config_t bus = {};
    bus.mosi_io_num = RADIO_MOSI;
    bus.miso_io_num = RADIO_MISO;
    bus.sclk_io_num = RADIO_SCLK;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = RF231_FB_MAX + 8;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = SPI_CLK_FRQ;
    dev.mode = 0;  // AT86RF231 is SPI mode 0, MSB first
    dev.spics_io_num = RADIO_NSS;
    dev.queue_size = 4;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &s_spi));

    s_spiMux = xSemaphoreCreateRecursiveMutex();
    if (!s_spiMux) {
        ets_printf("RF231: cannot create SPI mutex\n");
        return;
    }

    s_fbTx = static_cast<uint8_t *>(heap_caps_malloc(RF231_FB_MAX, MALLOC_CAP_DMA));
    s_fbRx = static_cast<uint8_t *>(heap_caps_malloc(RF231_FB_MAX, MALLOC_CAP_DMA));
    if (!s_fbTx || !s_fbRx) {
        ets_printf("RF231: cannot allocate DMA frame buffers\n");
        return;
    }

    // Reset pulse, then out of P_ON into TRX_OFF.
    gpio_set_level(static_cast<gpio_num_t>(RADIO_SLP_TR_PIN), 0);
    gpio_set_level(static_cast<gpio_num_t>(RADIO_RST_PIN), 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(static_cast<gpio_num_t>(RADIO_RST_PIN), 1);
    vTaskDelay(pdMS_TO_TICKS(10));  // crystal settle
    writeByte(RG_TRX_STATE, TRX_CMD_FORCE_TRX_OFF);
    vTaskDelay(pdMS_TO_TICKS(5));

    const uint8_t part = readByte(RG_PART_NUM);
    const uint8_t ver = readByte(RG_VERSION_NUM);
    const uint8_t m0 = readByte(RG_MAN_ID_0);
    const uint8_t m1 = readByte(RG_MAN_ID_1);
    const uint8_t st = readByte(RG_TRX_STATUS);

    ets_printf("RF231: PART_NUM=0x%02X VERSION=0x%02X MAN_ID=0x%02X%02X state=%s(0x%02X)\n", part,
               ver, m1, m0, stateName(st), st);

    s_present = (part == 0x03 || part == 0x0B);
    if (part == 0x03)
        ets_printf("  -> AT86RF231. Correct part.\n");
    else if (part == 0x0B)
        ets_printf("  -> AT86RF233. Also fine.\n");
    else if (part == 0x02)
        ets_printf("  !! AT86RF230 -- no high-data-rate modes. Wrong part.\n");
    else if (part == 0x00 || part == 0xFF)
        ets_printf(
            "  !! No response (0x%02X). Check VCC/GND, /SEL, MOSI/MISO/SCLK,\n"
            "     and that /RST is GPIO%d and SLP_TR GPIO%d (they are the REVERSE\n"
            "     of the ascending chip-pin order on this module).\n",
            part, RADIO_RST_PIN, RADIO_SLP_TR_PIN);
    else
        ets_printf("  !! Unexpected PART_NUM 0x%02X\n", part);

    pinMode(SCAN_LED, OUTPUT);
    digitalWrite(SCAN_LED, HIGH);  // XIAO C6 LED is active low
}

/// Deliberately does nothing.
///
/// The SX1276 needs an explicit RC/image calibration pass, so iohcRadio's
/// constructor calls this. The RF231 does not: it runs filter tuning and
/// both PLL calibrations automatically on every TRX_OFF -> PLL_ON
/// transition (datasheet s9.7), which happens on the way into RX_ON anyway.
///
/// Forcing them by hand is worse than useless here. PLL_CF and PLL_DCU each
/// pack their start bit alongside the calibration value, so writing a bare
/// 0x80 starts the calibration AND zeroes the value it is meant to keep --
/// which leaves the synthesiser mistuned, the receiver deaf (peak RSSI
/// pinned at the -100 dBm floor) and PLL_LOCK never firing. Anything added
/// here must be a read-modify-write of the start bit alone.
void calibrate() {}

void initRegisters(uint8_t maxPayloadLength) {
    (void)maxPayloadLength;  // the PHR carries the length; nothing to clamp
    if (!s_present) return;

    forceState(TRX_CMD_FORCE_TRX_OFF, TRX_STATUS_TRX_OFF);

    // TX_AUTO_CRC_ON deliberately OFF. With it on, the chip overwrites the
    // last two frame-buffer bytes with an 802.15.4 FCS -- but the bytes we
    // put there are io-homecontrol's own CRC, which is what the motor
    // checks. Leaving it off transmits them verbatim.
    writeByte(RG_TRX_CTRL_1, readByte(RG_TRX_CTRL_1) & ~RF231_TX_AUTO_CRC_ON);

    // TRX_CTRL_2 bits 2:0 = OQPSK_DATA_RATE. RX_SAFE_MODE (bit 7) stays OFF:
    // in Basic mode it only protects frames with a VALID FCS, which ours
    // never are, so it would buy nothing.
    writeByte(RG_TRX_CTRL_2, s_rate & 0x07);

    // Velux's start-of-frame delimiter, read off the air (board_config.h
    // IOHC_SFD). With the 802.15.4 default 0xA7 this remote never locks.
    writeByte(RG_SFD_VALUE, IOHC_SFD);

    // PHY_CC_CCA bits 4:0 = CHANNEL, bits 6:5 = CCA_MODE (1 = energy above
    // threshold).
    writeByte(RG_PHY_CC_CCA, 0x20 | (s_channel & 0x1F));

    // Maximum output: TX_PWR 0 = +3 dBm, with the default PA buffer/ramp.
    writeByte(RG_PHY_TX_PWR, 0xC0);

    writeByte(RG_IRQ_MASK, RF231_IRQ_TRX_END | RF231_IRQ_RX_START);
    (void)readByte(RG_IRQ_STATUS);  // reading clears

    ets_printf(
        "RF231: ch%u (%lu MHz) rate=%u (%u kb/s) SFD=0x%02X TX_AUTO_CRC=off\n", s_channel,
        static_cast<unsigned long>(
            (RF231_CH_BASE_HZ + (s_channel - RF231_CH_MIN) * RF231_CH_SPACING_HZ) / 1000000U),
        s_rate, static_cast<unsigned>(250 << (s_rate & 0x03)), IOHC_SFD);
}

/// The 802.15.4 SHR is fixed by the standard (4 zero octets then the SFD),
/// so there is no preamble length to set. Kept so callers stay portable.
void setPreambleLength(uint16_t preambleLen) {
    (void)preambleLen;
}

void clearBuffer() {
    s_rxLen = 0;
    s_rxPos = 0;
}

void IRAM_ATTR clearFlags() {
    (void)readByte(RG_IRQ_STATUS);  // reading clears every pending cause
}

// -------------------------------------------------------------- carrier setup

bool IRAM_ATTR setCarrier(const Carrier param, const uint32_t value) {
    switch (param) {
        case Carrier::Frequency: {
            // The stack carries frequencies in Hz on both bands; map to the
            // 802.15.4 channel grid, 2405 MHz + 5 MHz per channel.
            if (value < RF231_CH_BASE_HZ) return false;
            const uint32_t ch = RF231_CH_MIN + (value - RF231_CH_BASE_HZ) / RF231_CH_SPACING_HZ;
            if (ch > RF231_CH_MAX) return false;
            if (ch == s_channel) return true;
            s_channel = static_cast<uint8_t>(ch);
            // Channel can be changed on the fly; the PLL re-locks in ~150us.
            writeByte(RG_PHY_CC_CCA, 0x20 | (s_channel & 0x1F));
            return true;
        }
        case Carrier::Bitrate: {
            // 250k / 500k / 1M / 2M -> OQPSK_DATA_RATE 0..3.
            uint8_t rate;
            if (value <= 250000)
                rate = 0;
            else if (value <= 500000)
                rate = 1;
            else if (value <= 1000000)
                rate = 2;
            else
                rate = 3;
            if (rate == s_rate) return true;
            s_rate = rate;
            // Unlike the channel, OQPSK_DATA_RATE is not safe to change on
            // the fly -- TRX_CTRL_2 is sampled when the receiver starts, so
            // writing it in RX_ON leaves the old rate running. Drop to
            // TRX_OFF for the write; the caller's setRx() restarts RX.
            const bool wasRx = (readByte(RG_TRX_STATUS) & TRX_STATUS_MASK) != TRX_STATUS_TRX_OFF;
            forceState(TRX_CMD_FORCE_TRX_OFF, TRX_STATUS_TRX_OFF);
            writeByte(RG_TRX_CTRL_2, s_rate & 0x07);
            if (wasRx) setRx();
            return true;
        }
        case Carrier::Deviation:
        case Carrier::Bandwidth:
        case Carrier::Modulation:
            // Fixed by the O-QPSK PHY. Accepted and ignored so that shared
            // code can configure both radios with one sequence of calls.
            return true;
    }
    return false;
}

uint8_t channel() {
    return s_channel;
}
uint8_t dataRate() {
    return s_rate;
}
bool present() {
    return s_present;
}

// No AFC on this PHY -- the O-QPSK receiver tracks the carrier itself.
int32_t getFrequencyError() {
    return 0;
}
int16_t getAFCError() {
    return 0;
}

// ------------------------------------------------------------- the frame path

uint8_t IRAM_ATTR irqCause(uint8_t source, uint8_t edge) {
    (void)source;  // one IRQ line on this chip
    (void)edge;
    const uint8_t irq = readByte(RG_IRQ_STATUS);  // reading clears
    if (irq) s_irqCount++;
    uint8_t causes = IRQ_NONE;
    if (irq & RF231_IRQ_RX_START) {
        causes |= IRQ_SYNC_ON;
        s_rxStartCount++;
    }
    if (irq & RF231_IRQ_TRX_END) {
        if (s_txPending) s_txDoneCount++;  // the chip finished keying, i.e. the PA actually ran
        causes |= s_txPending ? IRQ_TX_DONE : IRQ_RX_DONE;
    }
    return causes;
}

/// io-homecontrol's own CRC, the one the SX1276 generates in hardware.
/// Reflected CCITT, poly 0x8408, init 0 -- see iohcCrypto::computeCrc.
static uint16_t iohcCrc(const uint8_t *buf, const uint8_t len) {
    return iohcCrypto::radioPacketComputeCrc(const_cast<uint8_t *>(buf), len);
}

/// Checks the trailing two octets of a received frame against the io-home
/// CRC over everything before them -- the whole on-air frame, Velux's "io"
/// wrapper included.
///
/// Byte order is LSB first, settled on real traffic 2026-09-14: an earlier
/// version accepted either order because neither had been seen. Accepting
/// both doubles the false-accept rate on a band this crowded, so now that
/// the answer is known it is pinned.
static bool iohcCrcCheck(const uint8_t *frame, const uint8_t len) {
    if (len < 3) return false;
    const uint8_t bodyLen = len - 2;
    const uint16_t want = iohcCrc(frame, bodyLen);
    return want == (static_cast<uint16_t>(frame[bodyLen]) | (frame[bodyLen + 1] << 8));
}

uint8_t IRAM_ATTR readFrame(uint8_t *out, const uint8_t maxLen) {
    SpiLock lock;  // status regs + frame buffer must be read as one unit
    // FCS validity and signal strength live in REGISTERS on this chip, not
    // in trailing frame-buffer bytes, and RX_CRC_VALID is only guaranteed
    // between this TRX_END and the next -- so read them before the buffer.
    const uint8_t phyRssi = readByte(RG_PHY_RSSI);
    const uint8_t ed = readByte(RG_PHY_ED_LEVEL);
    const uint8_t rssiRaw = phyRssi & 0x1F;

    frameBufferRead(RF231_FB_MAX);

    const uint8_t len = s_fbRx[1] & 0x7F;  // PHR
    s_stats = {};
    s_stats.airLength = len;
    s_stats.phyCrcOk = (phyRssi & RF231_RX_CRC_VALID) != 0;
    s_stats.rssiDbm =
        rssiRaw ? static_cast<float>(RF231_RSSI_BASE_VAL + 3 * (rssiRaw - 1)) : -100.0f;
    s_stats.afcHz = 0;

    if (len < 2 || len > RF231_PSDU_MAX) {
        s_badPhrCount++;
        if (s_trace) ets_printf("!! PHR=0x%02X -> implausible length %u\n", s_fbRx[1], len);
        s_rxLen = s_rxPos = 0;
        return 0;
    }
    s_stats.lqi = s_fbRx[2 + len];     // LQI trails the PSDU
    s_stats.snrDb = s_stats.lqi >> 3;  // 0..255 LQI as a coarse quality figure
    s_stats.edDbm = static_cast<int8_t>(RF231_RSSI_BASE_VAL + ed);

    const uint8_t *psdu = &s_fbRx[2];
    s_stats.iohcCrcOk = iohcCrcCheck(psdu, len);
    const int64_t nowUs = esp_timer_get_time();

    // Velux wake-up frame: 00 hh ll + CRC, hh:ll = ms until the command.
    // Counted and summarised; nothing for the io-home layer. The trace line
    // is queued, never printed here -- see TraceEntry.
    //
    // Classify on LENGTH alone, not on a valid CRC: a 5-byte PSDU can never
    // be a real frame, because the shortest one is the 3-byte wrapper + a
    // 9-byte io-home header + 2-byte CRC = 14. Requiring the CRC let the
    // corrupted ones fall through to be traced as ordinary frames, which
    // printed a line each and -- far worse -- ended the burst they belonged
    // to, fragmenting one 510-frame train into dozens of stubs (measured:
    // 95 of them in a 90 s capture).
    //
    // These are NOT noise or a neighbour's traffic. Checked on that capture:
    // all 95 arrived at -8..-12 dBm, the remote's own level in the cage and
    // far above anything else there; 94 of 95 carried byte 0 = 0x00; and
    // their countdown in bytes 1..2 continues the train, 47 of them landing
    // exactly 1 ms after the previous good one. They are genuine wake-ups
    // the demodulator got a symbol wrong in -- so drop them for being
    // untrustworthy, never for being foreign. A rising s_wakeBadCount means
    // the link is degrading, which is worth seeing; at -8 dBm the far more
    // likely cause is receiver compression from a transmitter held too
    // close than any weakness in the signal.
    if (len == 5) {
        if (s_stats.iohcCrcOk) {
            noteWakeup(static_cast<uint16_t>((psdu[1] << 8) | psdu[2]), nowUs);
            if (s_trace && s_traceWake) traceFrame(psdu, len, nowUs);
        } else {
            s_wakeBadCount++;
        }
        s_rxLen = s_rxPos = 0;
        return 0;
    }

    s_frameCount++;
    if (s_trace)
        traceFrame(psdu, len, nowUs);  // raw, wrapper included, so a new wrapper is visible

    // Unwrap Velux's 2.4 GHz link layer:
    //     <type> 'i' 'o' <io-homecontrol frame> <CRC-16, LSB first>
    // Only the marker and the CRC gate acceptance -- the type byte is
    // reported rather than required, because 0x01 is the only value seen.
    // Everything above this line then sees exactly what the 868 MHz stack
    // sees: a bare io-homecontrol frame with no CRC, buffer_length =
    // CtrlByte1.MsgLen + 1.
    if (len < IOHC_LINK_PREFIX_LEN + 2 || psdu[1] != IOHC_LINK_MARK0 ||
        psdu[2] != IOHC_LINK_MARK1 || !s_stats.iohcCrcOk) {
        s_notOursCount++;
        s_rxLen = s_rxPos = 0;
        return 0;
    }
    s_stats.linkType = psdu[0];

    const uint8_t bodyLen = len - IOHC_LINK_PREFIX_LEN - 2;
    s_stats.airLength = bodyLen;  // the io-home frame, which is what upstream length checks mean
    s_rxLen = bodyLen;

    const uint8_t copied = bodyLen > maxLen ? maxLen : bodyLen;
    memcpy(out, psdu + IOHC_LINK_PREFIX_LEN, copied);
    s_rxPos = copied;
    return copied;
}

/// Key one PSDU exactly as given -- no "io" wrapper, no CRC appended.
/// writeFrame() wraps and CRCs before calling this; the wake-up train needs
/// the unwrapped path, because a wake-up frame is a bare 00 hh ll + CRC and
/// carries no wrapper at all (measured: they arrive as 5-byte PSDUs).
static bool IRAM_ATTR txPsduRaw(const uint8_t *psdu, const uint8_t len) {
    setTx();
    if ((readByte(RG_TRX_STATUS) & TRX_STATUS_MASK) != TRX_STATUS_PLL_ON) return false;
    frameBufferWrite(psdu, len);
    s_txPending = true;
    writeByte(RG_TRX_STATE, TRX_CMD_TX_START);
    s_txCount++;
    return true;
}

void sendWakeupTrain(const uint16_t ms) {
    // Velux precedes every command to a low-power target with this, and the
    // skylight is exactly that: CtrlByte2.LPM is set in the KLR 200's own
    // close command. A sleeping actuator duty-cycles its receiver, so a bare
    // command frame is simply not heard -- the train is what holds it awake
    // until the command lands.
    //
    // Shape decoded from the remote 2026-09-14: a 5-byte frame every
    // millisecond, 00 hh ll + CRC, hh:ll counting milliseconds down to zero,
    // at which point the command goes out. 510 ms is what the remote sends.
    const bool wasTracing = s_trace;
    s_trace = false;  // ~510 trace lines would take longer than the train itself

    for (int32_t cd = ms; cd >= 0; cd--) {
        const int64_t due = esp_timer_get_time() + 1000;  // 1 ms cadence
        uint8_t wu[5];
        wu[0] = 0x00;
        wu[1] = static_cast<uint8_t>(cd >> 8);
        wu[2] = static_cast<uint8_t>(cd & 0xFF);
        const uint16_t crc = iohcCrc(wu, 3);
        wu[3] = crc & 0xFF;
        wu[4] = crc >> 8;
        txPsduRaw(wu, sizeof(wu));
        // Busy-wait rather than vTaskDelay: the tick is 1 ms, so a delay
        // would quantise to roughly double the cadence and stretch a 510 ms
        // train past a second, which the target would not sit through.
        while (esp_timer_get_time() < due) { /* spin */
        }
    }
    s_trace = wasTracing;
    ets_printf("wake-up train sent: %u frames over ~%u ms\n", ms + 1, ms);
}

bool IRAM_ATTR writeFrame(const uint8_t *in, const uint8_t len) {
    if (!s_present || !in || len == 0) return false;

    uint8_t psdu[RF231_PSDU_MAX];
    if (static_cast<uint16_t>(len) + IOHC_LINK_PREFIX_LEN + 2 > RF231_PSDU_MAX) return false;

    // Velux's 2.4 GHz link wrapper, the mirror of readFrame(). Callers hand
    // down a bare io-homecontrol frame exactly as they do on 868.
    uint8_t psduLen = 0;
    psdu[psduLen++] = IOHC_LINK_TYPE;
    psdu[psduLen++] = IOHC_LINK_MARK0;
    psdu[psduLen++] = IOHC_LINK_MARK1;
    memcpy(psdu + psduLen, in, len);
    psduLen += len;

    // CRC over the wrapper AND the frame, LSB first. The SX1276 appended it
    // in hardware, so buffer_length upstream still excludes it. TX_AUTO_CRC
    // stays off: the chip would compute the same function over the same
    // bytes, but only ever the last two, and writing it here keeps the one
    // code path that RX checks against.
    const uint16_t crc = iohcCrc(psdu, psduLen);
    psdu[psduLen++] = crc & 0xFF;
    psdu[psduLen++] = crc >> 8;

    setTx();  // ensure PLL_ON; safe if the caller already did it
    if ((readByte(RG_TRX_STATUS) & TRX_STATUS_MASK) != TRX_STATUS_PLL_ON) return false;

    if (!txPsduRaw(psdu, psduLen)) return false;
    if (s_trace) {
        char hex[2 * RF231_PSDU_MAX + 1];
        static const char D[] = "0123456789ABCDEF";
        for (uint8_t i = 0; i < psduLen; i++) {
            hex[2 * i] = D[psdu[i] >> 4];
            hex[2 * i + 1] = D[psdu[i] & 0x0F];
        }
        hex[2 * psduLen] = '\0';
        // Printed straight from the radio task rather than queued: a TX is
        // rare and deliberate, and seeing exactly what went on air matters
        // more here than the few ms of UART time.
        ets_printf("TX t=%lld ch%u len=%u %s\n", esp_timer_get_time() / 1000, s_channel, psduLen,
                   hex);
    }
    return true;
}

const FrameStats &lastFrameStats() {
    return s_stats;
}

// Byte-at-a-time drain of the staged frame, for callers written against the
// SX1276's streaming FIFO.
bool IRAM_ATTR dataAvail() {
    return s_rxPos < s_rxLen;
}
bool IRAM_ATTR crcOk() {
    return s_stats.iohcCrcOk;
}

// The RF231 reports RX_START rather than a preamble/sync pair, and it is
// already surfaced through irqCause(); these exist for API compatibility.
bool IRAM_ATTR preambleDetected() {
    return (readByte(RG_TRX_STATUS) & TRX_STATUS_MASK) == TRX_STATUS_BUSY_RX;
}
bool IRAM_ATTR syncedAddress() {
    return preambleDetected();
}

// -------------------------------------------------------------- diagnostics

int peakRssiDbm(const uint32_t ms) {
    uint8_t peak = 0;
    const int64_t until = esp_timer_get_time() + static_cast<int64_t>(ms) * 1000;
    while (esp_timer_get_time() < until) {
        const uint8_t v = readByte(RG_PHY_RSSI) & 0x1F;
        if (v > peak) peak = v;
        vTaskDelay(1);
    }
    return peak ? RF231_RSSI_BASE_VAL + 3 * (peak - 1) : -100;
}

/// Emission-free check that the IRQ line and the ESP32's edge interrupt are
/// actually wired up. A TRX_OFF -> PLL_ON transition raises PLL_LOCK, which
/// pulses IRQ exactly as a received frame would, but the PA stays off so
/// nothing goes on air. Returns the raw pin level seen while asserted.
bool irqSelfTest() {
    const uint8_t savedMask = readByte(RG_IRQ_MASK);

    forceState(TRX_CMD_FORCE_TRX_OFF, TRX_STATUS_TRX_OFF);
    writeByte(RG_IRQ_MASK, RF231_IRQ_PLL_LOCK);
    (void)readByte(RG_IRQ_STATUS);  // start from a cleared line

    // Count the interrupt rather than polling the pin: the handler task
    // reads IRQ_STATUS as soon as the edge lands, so a poll here races it
    // and reports a dead line on a perfectly working one.
    const uint32_t before = s_irqCount.load(std::memory_order_relaxed);
    writeByte(RG_TRX_STATE, TRX_CMD_PLL_ON);
    vTaskDelay(pdMS_TO_TICKS(5));  // PLL locks in ~110us; give the task a tick

    const uint32_t seen = s_irqCount.load(std::memory_order_relaxed) - before;
    ets_printf("RF231 IRQ self-test: GPIO%d, PLL_LOCK raised %lu interrupt(s) -> %s\n",
               RADIO_IRQ_PIN, static_cast<unsigned long>(seen),
               seen ? "line works; zero frames just means a quiet channel"
                    : "IRQ line NOT reaching the ESP32 -- check the pad 9 wire");

    writeByte(RG_IRQ_MASK, savedMask);
    (void)readByte(RG_IRQ_STATUS);
    setRx();
    return seen != 0;
}

void setSensitivity(const uint8_t pdtLevel) {
    const uint8_t v = readByte(RG_RX_SYN);
    // Preserve RX_PDT_DIS (bit 7) and the reserved bits; only 3:0 are ours.
    writeByte(RG_RX_SYN, static_cast<uint8_t>((v & 0xF0) | (pdtLevel & 0x0F)));
}

uint8_t sensitivity() {
    return readByte(RG_RX_SYN) & 0x0F;
}

void setSfd(const uint8_t v) {
    // Safe to write while receiving: the SFD correlator samples it per frame.
    writeByte(RG_SFD_VALUE, v);
}

uint8_t sfd() {
    return readByte(RG_SFD_VALUE);
}

void sfdScan(const uint16_t liveMs, const uint16_t timeoutMs, const uint8_t first,
             const uint8_t last, const uint8_t rssiBusy) {
    if (!s_present) return;
    const uint8_t savedSfd = readByte(RG_SFD_VALUE);
    const bool savedTrace = s_trace;
    s_trace = false;

    // Energy threshold for "the remote is transmitting". PHY_RSSI is a raw
    // 0..28 step where dBm = RSSI_BASE_VAL + 3*(raw-1).
    //
    // MEASURED, do not raise this blindly: across 206 captured frames the
    // remote's median is -39 dBm and its 25th percentile -43. An earlier
    // threshold of raw 21 (-31 dBm) counted only 26% of actual transmission
    // as live, so the scan crawled and badly overran its estimate. Raw 17 is
    // -43 dBm, which counts ~75% of it while staying above the deafened
    // ambient floor (peaks around -46 dBm here).

    ets_printf("# SFD scan: %u ms live per value, %u ms timeout, busy>=%d dBm\n", liveMs, timeoutMs,
               RF231_RSSI_BASE_VAL + 3 * (rssiBusy - 1));
    ets_printf(
        "# Press the remote continuously; the scan pauses by itself\n"
        "# whenever the band goes quiet, so gaps cost nothing.\n");
    ets_printf("# Send any character to abort.\n");
    ets_printf("SFD,frames,shortFrames,liveMs,timedOut\n");

    // The scan blocks the command handler for as long as it runs, so without
    // this the only way out of a 256-value sweep is a board reset -- which
    // is exactly what happened the first time it was launched early.
    while (Serial.available()) Serial.read();
    bool aborted = false;

    for (uint16_t v = first; v <= last && !aborted; ++v) {
        writeByte(RG_SFD_VALUE, static_cast<uint8_t>(v));
        (void)readByte(RG_IRQ_STATUS);
        const uint32_t before = s_frameCount.load(std::memory_order_relaxed);
        uint32_t live = 0;
        uint16_t shortFrames = 0;
        const int64_t deadline = esp_timer_get_time() + timeoutMs * 1000LL;
        uint32_t seen = 0;

        while (live < liveMs && esp_timer_get_time() < deadline) {
            if (Serial.available()) {
                aborted = true;
                break;
            }
            // 5 ms slices: long enough not to thrash the SPI bus, short
            // enough to track a burst.
            const uint32_t now = s_frameCount.load(std::memory_order_relaxed);
            const bool gotFrame = (now != seen);
            if (gotFrame) {
                // s_stats belongs to the most recent frame; count the ones
                // the size of a real io-homecontrol frame.
                if (s_stats.airLength >= 9 && s_stats.airLength <= 32) shortFrames++;
                seen = now;
            }

            // A received frame is PROOF of transmission and must credit live
            // time. Polling RSSI every 5 ms misses most bursts -- measured, an
            // earlier RSSI-only gate credited just 8% of wall clock as live
            // while frames were pouring in, turning a 256-value scan into 31
            // minutes of the operator's button-pressing. Credit a frame
            // generously: it proves the whole surrounding burst was live.
            const uint8_t rssi = readByte(RG_PHY_RSSI) & 0x1F;
            if (gotFrame)
                live += 25;
            else if (rssi >= rssiBusy)
                live += 5;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        const uint32_t got = s_frameCount.load(std::memory_order_relaxed) - before;
        ets_printf("%02X,%lu,%u,%lu,%d\n", v, static_cast<unsigned long>(got), shortFrames,
                   static_cast<unsigned long>(live), live < liveMs ? 1 : 0);
    }
    ets_printf(aborted ? "# SFD scan ABORTED\n" : "# SFD scan complete\n");
    while (Serial.available()) Serial.read();
    writeByte(RG_SFD_VALUE, savedSfd);
    s_trace = savedTrace;
}

void channelScan(const uint32_t ms, const uint8_t rssiBusy) {
    if (!s_present) return;

    // Velux's three, then two it has never been seen on. Without the
    // controls a "busy" reading means nothing: this band carries WiFi,
    // Thread, Zigbee and BLE, and 802.15.4 ch15/ch20 sit under WiFi 6/11.
    static const uint8_t chans[] = {15, 20, 25, 11, 26};
    constexpr size_t N = sizeof(chans) / sizeof(chans[0]);
    uint32_t samples[N] = {};
    uint32_t busy[N] = {};
    uint8_t peak[N] = {};
    int64_t lastReport[N] = {};

    const uint8_t savedCh = s_channel;
    const uint8_t savedPdt = readByte(RG_RX_SYN) & 0x0F;
    // Measure with the front end wide open. RX_PDT_LEVEL gates the frame
    // detector rather than the RSSI reading, but leaving the receiver
    // deafened while asking "is anything on air" invites a false negative.
    setSensitivity(0);
    setRx();

    ets_printf("# channel scan: %lu ms, busy >= %d dBm\n", static_cast<unsigned long>(ms),
               RF231_RSSI_BASE_VAL + 3 * (static_cast<int>(rssiBusy) - 1));
    ets_printf(
        "# ch15/20/25 are Velux's, ch11/26 the control: interference lifts\n"
        "# all five together, a transmission lifts only Velux's.\n"
        "# Send any character to abort.\n");

    const int64_t t0 = esp_timer_get_time();
    const int64_t until = t0 + static_cast<int64_t>(ms) * 1000;
    bool aborted = false;
    uint32_t round = 0;

    while (esp_timer_get_time() < until) {
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            aborted = true;
            break;
        }
        for (size_t i = 0; i < N; i++) {
            writeByte(RG_PHY_CC_CCA, 0x20 | (chans[i] & 0x1F));
            delayMicroseconds(200);  // PLL re-lock is ~150 us
            for (uint8_t k = 0; k < 3; k++) {
                const uint8_t v = readByte(RG_PHY_RSSI) & 0x1F;
                samples[i]++;
                if (v > peak[i]) peak[i] = v;
                if (v < rssiBusy) continue;
                busy[i]++;
                const int64_t now = esp_timer_get_time();
                if (now - lastReport[i] < 100000) continue;  // one line per 100 ms per channel
                lastReport[i] = now;
                ets_printf("  t=%6lldms ch%-2u %d dBm\n", (now - t0) / 1000, chans[i],
                           RF231_RSSI_BASE_VAL + 3 * (static_cast<int>(v) - 1));
            }
        }
        if ((++round & 0x07) == 0) vTaskDelay(1);  // feed the watchdog
    }

    s_channel = savedCh;
    writeByte(RG_PHY_CC_CCA, 0x20 | (s_channel & 0x1F));
    setSensitivity(savedPdt);
    setRx();

    ets_printf(aborted ? "# channel scan ABORTED\n" : "# channel scan complete\n");
    // Per-mille rather than a percentage: ets_printf has no float support.
    ets_printf("ch,samples,busy,busy_permille,peak_dBm\n");
    for (size_t i = 0; i < N; i++)
        ets_printf("%u,%lu,%lu,%lu,%d\n", chans[i], static_cast<unsigned long>(samples[i]),
                   static_cast<unsigned long>(busy[i]),
                   static_cast<unsigned long>(samples[i] ? 1000ULL * busy[i] / samples[i] : 0),
                   peak[i] ? RF231_RSSI_BASE_VAL + 3 * (static_cast<int>(peak[i]) - 1) : -100);
}

void traceFrames(const bool on) {
    if (on && !s_traceQ) {
        s_traceQ = xQueueCreate(16, sizeof(TraceEntry));
        if (!s_traceQ ||
            xTaskCreate(tracePump, "rf231trace", 4096, nullptr, 1, nullptr) != pdPASS) {
            ets_printf("RF231: cannot start trace printer\n");
            s_traceQ = nullptr;
            return;
        }
    }
    s_trace = on;
    ets_printf("RF231: frame trace %s (wake-up frames %s)\n", on ? "ON" : "off",
               s_traceWake ? "printed individually" : "one summary line per burst");
}

void traceWake(const bool on) {
    s_traceWake = on;
    ets_printf("RF231: wake-up frames %s\n", on ? "printed individually" : "summarised per burst");
}

uint32_t wakeCount() {
    return s_wakeCount.load(std::memory_order_relaxed);
}

void status() {
    const uint8_t st = readByte(RG_TRX_STATUS);
    // A single RSSI sample reads -100 most of the time even on a busy
    // channel; peak over 300ms is what tells you the front end is alive.
    const int peak = peakRssiDbm(300);
    ets_printf(
        "RF231 %s(0x%02X) ch%u (%lu MHz) rate=%u (%u kb/s) peakRssi=%ddBm irqPin=%d\n",
        stateName(st), st, s_channel,
        static_cast<unsigned long>(
            (RF231_CH_BASE_HZ + (s_channel - RF231_CH_MIN) * RF231_CH_SPACING_HZ) / 1000000U),
        s_rate, static_cast<unsigned>(250 << (s_rate & 0x03)), peak,
        gpio_get_level(static_cast<gpio_num_t>(RADIO_IRQ_PIN)));
    const uint8_t pdt = readByte(RG_RX_SYN) & 0x0F;
    const uint8_t sfdNow = readByte(RG_SFD_VALUE);
    ets_printf("  SFD=0x%02X%s  RX_PDT_LEVEL=%u (-%u dB)\n", sfdNow,
               sfdNow == IOHC_SFD ? " (Velux)"
               : sfdNow == 0xA7   ? " (802.15.4 std)"
                                  : " (non-standard)",
               pdt, pdt * 3u);
    ets_printf(
        "  irq=%lu rx_start=%lu frames=%lu wakeups=%lu (bad %lu) not_ours=%lu bad_phr=%lu "
        "trace=%s\n",
        static_cast<unsigned long>(s_irqCount.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(s_rxStartCount.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(s_frameCount.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(s_wakeCount.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(s_wakeBadCount.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(s_notOursCount.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(s_badPhrCount.load(std::memory_order_relaxed)),
        s_trace ? "on" : "off");
    ets_printf("  tx=%lu tx_done=%lu\n",
               static_cast<unsigned long>(s_txCount.load(std::memory_order_relaxed)),
               static_cast<unsigned long>(s_txDoneCount.load(std::memory_order_relaxed)));
}

void dump() {
    const uint8_t st = readByte(RG_TRX_STATUS);
    ets_printf("# AT86RF231 state=%s(0x%02X) ch=%u rate=%u (%u kb/s) SFD=0x%02X\n", stateName(st),
               st, s_channel, s_rate, static_cast<unsigned>(250 << (s_rate & 0x03)),
               readByte(RG_SFD_VALUE));
    ets_printf("#Type\tRegister\tAddress[Hex]\tValue[Hex]\n");
    for (uint8_t a = 0x00; a <= 0x2F; ++a)
        ets_printf("REG\tname\t0x%2.2x\t0x%2.2x\n", a, readByte(a));
}

void dumpReal() {
    for (uint8_t a = 0x00; a <= 0x2F; ++a) {
        if (a) ets_printf(",");
        ets_printf("0x%2.2x", readByte(a));
    }
    ets_printf("\n");
}
}  // namespace Radio

#endif  // RADIO_AT86RF231
