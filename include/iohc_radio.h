#ifndef IOHC_RADIO_H
#define IOHC_RADIO_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cstdint>
#include <functional>

#include <board_config.h>
#include <iohc_crypto_helpers.h>
#include <iohc_packet.h>
#include <queue>

// Only one radio now: the SX1276 branch pointed at a header deleted with the
// 868 MHz support.
#include <rf231_helpers.h>
#if defined(ESP32)
#include <ticker_us_esp32.h>
#endif

#define SM_GRANULARITY_US 130ULL  // Ticker function frequency in uS (100 minimum) 4 x 26µs = 104
#define SM_GRANULARITY_MS 2       // Ticker function frequency in mS
#define SM_PREAMBLE_RECOVERY_TIMEOUT_US \
    1378  // 12500   // SM_GRANULARITY_US * PREAMBLE_LSB //12500   // Maximum duration in uS of Preamble before reset of receiver
// Frequency Hopping
#define DEFAULT_SCAN_INTERVAL_US 13520  // Default uS between frequency changes

struct TxPacketWrapper;

/*
    Singleton class to implement an IOHC Radio abstraction layer for controllers.
    Implements all needed functionalities to receive and send packets from/to the air, masking complexities related to frequency hopping
    IOHC timings, async sending and receiving through callbacks, ...
*/
namespace IOHC {
struct RadioIrqEvent;

using CallbackFunction = std::function<bool(iohcPacket *iohc)>;

class iohcRadio {
  public:
    static iohcRadio *getInstance();

    virtual ~iohcRadio() {
        // Nettoyer la queue
        clearTxQueue();
        if (tx_mutex) vSemaphoreDelete(tx_mutex);
        if (txQueue_binary_sem) vSemaphoreDelete(txQueue_binary_sem);
    };

    enum class RadioState : uint8_t {
        IDLE,      ///< Default state: nothing happening
        RX,        ///< Receiving mode
        TX,        ///< Transmitting mode
        PREAMBLE,  ///< Preamble detected
        PAYLOAD,   ///< Payload available
        LOCKED,    ///< Frequency locked
        ERROR      ///< Error or unknown state
    };

    static const char *radioStateToString(RadioState state) {
        switch (state) {
            case RadioState::IDLE: return "IDLE";
            case RadioState::RX: return "RX";
            case RadioState::TX: return "TX";
            case RadioState::PREAMBLE: return "PREAMBLE";
            case RadioState::PAYLOAD: return "PAYLOAD";
            case RadioState::LOCKED: return "LOCKED";
            case RadioState::ERROR: return "ERROR";
            default: return "UNKNOWN";
        }
    }

    volatile bool expectingResponse = false;
    volatile int64_t lastActivityTime = 0;
    volatile uint32_t responseTimeoutMs = 0;
    volatile bool txQueue_busy = false;  // Simple spinlock

    void start(uint8_t num_freqs, uint32_t *scan_freqs, uint32_t scanTimeUs,
               CallbackFunction rxCallback, CallbackFunction txCallback);

    bool send(std::vector<iohcPacket *> &TxPackets);
    static void setRadioState(RadioState newState);
    static void processNextPacketCallback(iohcRadio *radio);
    void startTransmission();
    bool processNextPacket();
    void stopTransmission();
    void clearTxQueue();
    bool isTransmitting() const;
    void cancelTransmissions();

    volatile static RadioState radioState;
    volatile static bool f_lock_hop;

    static void tickerCounter(iohcRadio *radio, const RadioIrqEvent &evt);

    bool sendSingle(iohcPacket *packet);
    bool sendPriority(iohcPacket *packet);

    static TaskHandle_t txTaskHandle;  // TX Task handle

    uint8_t num_freqs;  // = 0;
    /// Frequency in Hz to transmit on when a packet does not name one,
    /// overriding scan_freqs[currentFreqIdx]. 0 = no override.
    ///
    /// Needed because the 868 MHz stack pins every command to CHANNEL2 and
    /// the 2.4 GHz remote demonstrably does not: an energy scan on
    /// 2026-09-14 caught it alternating ch25 with ch15 or ch20, ~500 ms per
    /// channel (one full wake-up train each), with ch25 carrying about half
    /// of all airtime. `rfch` cannot express this, because packetSender
    /// re-sets the carrier from the packet on every transmission.
    static uint32_t txFreqOverride;

    uint32_t *scan_freqs;    //{};
    uint32_t scanTimeUs;     //{};
    uint8_t currentFreqIdx;  // = 0;

    std::deque<TxPacketWrapper *> txQueue;
    SemaphoreHandle_t txQueue_binary_sem; 
    TxPacketWrapper *currentTxPacket = nullptr;
    SemaphoreHandle_t tx_mutex = nullptr;
    bool isSending = false;

    iohcRadio();

    void startPacketProcessor();

    static void packetProcessorTask(void *parameter);

    static constexpr size_t PACKET_QUEUE_SIZE = 10;
    QueueHandle_t packetQueue;
    TaskHandle_t packetProcessorTaskHandle;

    /// Bytes of stack the packet decoder has never used. This task runs
    /// msgRcvd(), which is the deepest call chain in the firmware, and a
    /// canary trip here presents as an unexplained panic rather than as a
    /// stack overflow -- so the margin is worth being able to read.
    uint32_t decoderStackHeadroom() const {
        return packetProcessorTaskHandle ? uxTaskGetStackHighWaterMark(packetProcessorTaskHandle)
                                         : 0;
    }

    iohcPacket tempRxPacket;

    SemaphoreHandle_t lastPacketMutex;
    bool receive(bool stats, const RadioIrqEvent &evt);

    bool isResponsePacket(iohcPacket *packet);

    bool sent(iohcPacket *packet);

    static iohcRadio *_iohcRadio;
    static uint8_t _flags[2];

    volatile static bool send_lock;
    volatile static bool txMode;

    volatile uint32_t tickCounter = 0;
    volatile uint8_t txCounter = 0;

    TimersUS::TickerUsESP32 TickTimer;
    TimersUS::TickerUsESP32 Ticker;

    iohcPacket *new_packet{};

    CallbackFunction rxCB = nullptr;
    CallbackFunction txCB = nullptr;
    std::vector<iohcPacket *> packets2send{};

    static void maybeCheckHeap(const char *tag) {
        if (!heap_caps_check_integrity_all(true)) {
            ets_printf("HEAP CORRUPTED at %s\n", tag);
        } else {
            ets_printf("heap ok at %s free:%u\n", tag, heap_caps_get_free_size(MALLOC_CAP_8BIT));
        }
    }
    void printStats() const {
        if (xSemaphoreTake(statsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            ets_printf(
                "Radio Stats - Received: %lu, Processed: %lu, Dropped: %lu, Max Queue: %lu, Avg "
                "Delay: %luµs",
                packetsReceived, packetsProcessed, packetsDropped, maxQueueDepth,
                totalQueueDelay / (packetsProcessed ? packetsProcessed : 1));
            xSemaphoreGive(statsMutex);
        }
    }
    void debugRadioFlags() {
        // The RF231 keeps a single IRQ_STATUS register rather than the
        // SX1276's two flag words, and reading it clears every cause -- so
        // this reports the persistent state instead and leaves IRQ_STATUS
        // to the interrupt path. Radio::dump() shows the full register set.
        const uint8_t trx = Radio::readByte(RG_TRX_STATUS);
        const uint8_t rssi = Radio::readByte(RG_PHY_RSSI);
        ets_printf("TRX_STATUS: 0x%02X  RSSI: %ddBm  RX_CRC_VALID(802.15.4): %d\n", trx,
                   (rssi & 0x1F) ? RF231_RSSI_BASE_VAL + 3 * ((rssi & 0x1F) - 1) : -100,
                   (rssi & RF231_RX_CRC_VALID) ? 1 : 0);
    }

  private:
    static void packetSender(iohcRadio *radio);

    uint32_t packetsReceived = 0;
    uint32_t packetsProcessed = 0;
    uint32_t packetsDropped = 0;
    uint32_t maxQueueDepth = 0;
    uint32_t totalQueueDelay = 0;
    SemaphoreHandle_t statsMutex;
};
}  // namespace IOHC

#endif
