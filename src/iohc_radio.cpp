#include <esp32-hal-gpio.h>
#include <map>
#include "esp_log.h"

#include <iohc_last_command.h>
#include <iohc_radio.h>
#include <utility>

// Timestamps for precise measurements
// max airTime µs: (preamble (0x3D) + sync (3 B) + header (2 B) + payload (32 B) + CRC (2 B)) / 38.4 kbps.
volatile uint64_t timestamp_sync_started = 0;
volatile uint64_t timestamp_sync_stopped = 0;
volatile uint64_t timestamp_payload_ready = 0;
volatile uint64_t timestamp_packet_sent = 0;

TaskHandle_t IOHC::iohcRadio::txTaskHandle = nullptr;

// Structure to manage a packet in transmission
struct TxPacketWrapper {
    IOHC::iohcPacket *packet;
    uint8_t repeatsRemaining;
    uint32_t repeatTime;

    explicit TxPacketWrapper(IOHC::iohcPacket *pkt)
        : packet(pkt),
          repeatsRemaining(pkt ? pkt->repeat : 0),
          repeatTime(pkt ? pkt->repeatTime : 0) {}

    // Owns `packet`, so copying it would double-free. Nothing copies one today
    // -- they live in a deque<TxPacketWrapper*> and are always held by pointer --
    // but cppcheck was right that nothing stopped it. Now the compiler does.
    TxPacketWrapper(const TxPacketWrapper &) = delete;
    TxPacketWrapper &operator=(const TxPacketWrapper &) = delete;

    ~TxPacketWrapper() {
        if (packet) {
            delete packet;
            packet = nullptr;
        }
    }
};

namespace IOHC {
iohcRadio *iohcRadio::_iohcRadio = nullptr;

uint8_t iohcRadio::_flags[2] = {0, 0};
volatile bool iohcRadio::f_lock_hop = false;
volatile bool iohcRadio::send_lock = false;
uint32_t iohcRadio::txFreqOverride = 0;
volatile bool iohcRadio::txMode = false;
volatile iohcRadio::RadioState iohcRadio::radioState = iohcRadio::RadioState::IDLE;

static IOHC::iohcPacket last1wPacket;

TaskHandle_t handle_interrupt;
struct RadioIrqEvent {
    uint8_t source;  // 0 = DIO0 (payload), 2 = DIO2 (synchro)
    uint8_t edge;    // 0 = falling, 1 = rising
    mutable uint64_t timestamp_us;
};

static QueueHandle_t radioIrqQueue = nullptr;
static constexpr size_t RADIO_IRQ_QUEUE_LEN = 128;

/**
    * No semaphore here.
    * esp_timer_get_time() is ISR-safe
    */
void IRAM_ATTR handle_interrupt_fromDIO0(void *arg) {
    // Only GPIOS short ops, non blocking
    RadioIrqEvent ev;
    ev.source = 0;  // DIO0 - PayloadReady / PacketSent
    ev.edge = gpio_get_level(static_cast<gpio_num_t>(RADIO_PACKET_AVAIL));
    ev.timestamp_us = esp_timer_get_time();

    // Send it to queue (FromISR)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(radioIrqQueue, &ev, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/**
    * No semaphore here.
    * esp_timer_get_time() is ISR-safe
    */
void IRAM_ATTR handle_interrupt_fromDIO1(void *arg) {
    // Only GPIOS short ops, non blocking
    RadioIrqEvent ev;
    ev.source = 1;  // DIO1 - PayloadReady / PacketSent
    ev.edge = gpio_get_level(static_cast<gpio_num_t>(RADIO_PACKET_AVAIL));
    ev.timestamp_us = esp_timer_get_time();

    // Send it to queue (FromISR)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(radioIrqQueue, &ev, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/**
    * No semaphore here.
    * esp_timer_get_time() is ISR-safe
    */

void IRAM_ATTR handle_interrupt_fromDIO2(void *arg) {
    // Only GPIOS short ops, non blocking
    RadioIrqEvent ev;
    ev.source = 2;  // DIO2 - SyncAddress
    ev.edge = gpio_get_level(static_cast<gpio_num_t>(RADIO_SYNCHRO_DETECTED));
    ev.timestamp_us = esp_timer_get_time();

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(radioIrqQueue, &ev, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR handleSynchroStart(iohcRadio *radio, const RadioIrqEvent &evt) {
    timestamp_sync_started = evt.timestamp_us;
    radio->setRadioState(iohcRadio::RadioState::PREAMBLE);
}

void IRAM_ATTR handleSynchroStop(iohcRadio *radio, const RadioIrqEvent &evt) {
    timestamp_sync_stopped = evt.timestamp_us;
    radio->setRadioState(iohcRadio::RadioState::RX);
    // SyncLen ~= nByte * 1250 µs
}

void IRAM_ATTR handlePayloadReady(iohcRadio *radio, const RadioIrqEvent &evt) {
    timestamp_payload_ready = evt.timestamp_us;
    if (iohcRadio::radioState == iohcRadio::RadioState::PREAMBLE ||
        iohcRadio::radioState == iohcRadio::RadioState::RX) {
        radio->setRadioState(iohcRadio::RadioState::PAYLOAD);
        radio->tickerCounter(radio, evt);
    }
}

void IRAM_ATTR handlePacketSent(iohcRadio *radio, const RadioIrqEvent &evt) {
    timestamp_packet_sent = evt.timestamp_us;
    Radio::clearFlags();
    // Return the radio to RX mode
    Radio::setRx();
    radio->setRadioState(iohcRadio::RadioState::RX);
}

/**
    * Sequential : Read queue — no concurrencies possible.
    * tickerCounter isn't called by the ISR.
    */
void handle_interrupt_task(void *pvParameters) {
    auto radio = static_cast<iohcRadio *>(pvParameters);
    RadioIrqEvent evt;
    // RF_IRQFLAGS1_0xDB -> MODEREADY RXREADY PLLLOCK RSSI PREAMBLEDETECT SYNCADDRESSMATCH
    // RF_IRQFLAGS2_0x26 -> FIFOLEVEL PAYLOADREADY CRCOK
    // RF_IRQFLAGS1_0xB0 -> MODEREADY TXREADY PLLLOCK
    // RF_IRQFLAGS2_0x48 -> FIFOEMPTY PACKETSENT

    while (true) {
        // Bounded wait, not portMAX_DELAY. The RF231 holds its IRQ line
        // asserted until IRQ_STATUS is read, so an edge-triggered interrupt
        // has no margin for error: miss one read and no further rising edge
        // is ever generated, leaving the receiver silently deaf. On timeout,
        // if the line is still high there is an unserviced cause -- treat it
        // as an event and let irqCause() read (and so clear) the register.
        if (xQueueReceive(radioIrqQueue, &evt, pdMS_TO_TICKS(50)) != pdTRUE) {
            if (gpio_get_level(static_cast<gpio_num_t>(RADIO_DIO0_PIN))) {
                evt.source = 0;
                evt.edge = 1;
                evt.timestamp_us = esp_timer_get_time();
            } else
                continue;
        }
        {
            // Ask the driver what this interrupt meant. The SX1276 answers
            // from IRQFLAGS2 plus which DIO line fired; the RF231 answers
            // from its single IRQ_STATUS register. One event can carry more
            // than one cause, so these are flags and all of them are acted on.
            const uint8_t causes = Radio::irqCause(evt.source, evt.edge);

            if (causes & Radio::IRQ_SYNC_ON) handleSynchroStart(radio, evt);
            if (causes & Radio::IRQ_SYNC_OFF) handleSynchroStop(radio, evt);
            if (causes & Radio::IRQ_RX_DONE) handlePayloadReady(radio, evt);
            if (causes & Radio::IRQ_TX_DONE) handlePacketSent(radio, evt);
        }
    }
}

iohcRadio::iohcRadio() {
    Radio::initHardware();
    Radio::calibrate();

    Radio::initRegisters(MAX_FRAME_LEN);
    // PHY parameters live in board_config.h: 868 FSK wants all four, the
    // 2.4 GHz O-QPSK PHY fixes deviation/bandwidth/modulation by standard
    // and ignores those calls.
    Radio::setCarrier(Radio::Carrier::Deviation, IOHC_DEVIATION);
    Radio::setCarrier(Radio::Carrier::Bitrate, IOHC_BITRATE);
    Radio::setCarrier(Radio::Carrier::Bandwidth, IOHC_BANDWIDTH);
    Radio::setCarrier(Radio::Carrier::Modulation, Radio::Modulation::FSK);

    // Configure interrupts with pull-down to avoid false triggers
    // Attach interrupts with hardware debounce
    // ESP_ERR_INVALID_STATE just means someone (the Arduino core, via
    // attachInterrupt) already installed the service -- not a failure.
    // Anything else means gpio_isr_handler_add below will silently do
    // nothing, which looks exactly like a dead radio, so say so.
    const esp_err_t isrSvc = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (isrSvc != ESP_OK && isrSvc != ESP_ERR_INVALID_STATE)
        ets_printf("gpio_install_isr_service failed: %s\n", esp_err_to_name(isrSvc));

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << RADIO_DIO0_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    const esp_err_t isrAdd = gpio_isr_handler_add(static_cast<gpio_num_t>(RADIO_DIO0_PIN),
                                                  handle_interrupt_fromDIO0, nullptr);
    if (isrAdd != ESP_OK)
        ets_printf("gpio_isr_handler_add(GPIO%d) failed: %s\n", RADIO_DIO0_PIN,
                   esp_err_to_name(isrAdd));

#if RADIO_DIO2_PIN >= 0
    // The SX1276 signals SyncAddress on a second line. The RF231 has one
    // IRQ pin and reports the equivalent (RX_START) through IRQ_STATUS, so
    // there is nothing to attach here.
    io_conf.pin_bit_mask = (1ULL << RADIO_DIO2_PIN);
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&io_conf);
    gpio_isr_handler_add(static_cast<gpio_num_t>(RADIO_DIO2_PIN), handle_interrupt_fromDIO2,
                         nullptr);
#else
    (void)handle_interrupt_fromDIO2;
#endif

    // Create all mutex
    tx_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(tx_mutex);
    txQueue_binary_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(txQueue_binary_sem);  // Initialement libre
    lastPacketMutex = xSemaphoreCreateMutex();

    // Create ISR Queue
    radioIrqQueue = xQueueCreate(RADIO_IRQ_QUEUE_LEN, sizeof(RadioIrqEvent));

    // Create Packet Queue
    packetQueue = xQueueCreate(PACKET_QUEUE_SIZE, sizeof(iohcPacket));

    // Start Packet processor
    startPacketProcessor();

    // start state machine
    ets_printf("Starting Interrupt Handler...\n");

    BaseType_t task_code =
        xTaskCreatePinnedToCore(handle_interrupt_task, "handle_interrupt_task", 8192, this,
                                configMAX_PRIORITIES - 1,  // Before FHSS
                                &handle_interrupt, xPortGetCoreID());
    if (task_code != pdPASS) {
        ets_printf("ERROR STATEMACHINE Can't create task %d\n", task_code);
        return;
    }
}

/**
     * @brief Returns a pointer to a single instance of the `iohcRadio`
     * class, creating it if it doesn't already exist.
     *
     * @return An instance of the `iohcRadio` class.
     */
iohcRadio *iohcRadio::getInstance() {
    if (!_iohcRadio) _iohcRadio = new iohcRadio();
    return _iohcRadio;
}
/**
     * Initializes the radio with specified parameters and sets it to receive mode.
     */
void iohcRadio::start(uint8_t num_freqs, uint32_t *scan_freqs, uint32_t scanTimeUs,
                      CallbackFunction rxCallback = nullptr,
                      CallbackFunction txCallback = nullptr) {
    this->num_freqs = num_freqs;
    this->scan_freqs = scan_freqs;
    this->rxCB = std::move(rxCallback);
    this->txCB = std::move(txCallback);
    this->currentFreqIdx = 0;
    this->expectingResponse = false;

    // Default
    if (scanTimeUs == 0) {
        this->scanTimeUs = 30 * 1000;  // 15ms par fréquence
    } else {
        this->scanTimeUs = scanTimeUs;
    }

    Radio::clearBuffer();
    Radio::clearFlags();

    /* We always start at freq[0], the 1W/2W channel*/
    Radio::setCarrier(Radio::Carrier::Frequency, scan_freqs[0]);  // CHANNEL2); // 868950000);

    if (this->num_freqs > 1) {
        this->currentFreqIdx = 0;

        // start adaptative

        // Start Frequency Hopping Timer
        // adaptiveFHSS->switchToFastScan(17); // ...ms par fréquence
        ets_printf("FHSSTimer Handler Started...\n");
    }
    Radio::setRx();
    // handlePayloadReady() only acts from PREAMBLE or RX. On the SX1276 the
    // DIO2 sync interrupt supplies PREAMBLE; the RF231 has no such line, so
    // declare RX here rather than rely on RX_START arriving first.
    setRadioState(RadioState::RX);
}

/**
    */
void IRAM_ATTR iohcRadio::tickerCounter(iohcRadio *radio, const RadioIrqEvent &evt) {
    // Sync = preamble end
    // else
    if (radioState == RadioState::PAYLOAD) {
        // Payload ready
        radio->receive(false, evt);
        Radio::clearFlags();  // After receive
        if (!txMode) {
            // radio->unlockFHSS(FHSSLockReason::RECEIVING);
        }
        radio->tickCounter = 0;
    }
}

/**
    */
static TxPacketWrapper *createPacketWrapper(iohcPacket *sourcePacket) {
    if (!sourcePacket) {
        return nullptr;
    }

    // Try to allocate copy of package first
    iohcPacket *pktToSend = nullptr;

    pktToSend = new (std::nothrow) iohcPacket(*sourcePacket);
    if (!pktToSend) {
        return nullptr;  // Zero leakage - nothing has been allocated
    }

    // Then allocate the wrapper
    auto wrapper = new (std::nothrow) TxPacketWrapper(pktToSend);
    if (!wrapper) {
        // Release the package
        delete pktToSend;
        return nullptr;  // Zero leak
    }

    return wrapper;
}
/**
     * Sends packets stored in a vector with a specified repeat time.
     *
     * @param TxPackets `Vector of pointers to `iohcPacket` objects.
    *
* Sends one or more packets
     * Packages are COPIED internally, we retain ownership of the originals
     */
bool IRAM_ATTR iohcRadio::send(std::vector<iohcPacket *> &TxPackets) {
    if (TxPackets.empty()) {
        return false;
    }
    if (xSemaphoreTake(txQueue_binary_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    txCounter = 0;
    size_t packetsAdded = 0;
    // Add each packet to the queue (making a COPY)
    for (auto *pkt : TxPackets) {
        if (pkt) {
            // Use la factory
            if (TxPacketWrapper *wrapper = createPacketWrapper(pkt)) {
                txQueue.push_back(wrapper);
                packetsAdded++;
            }
        }
    }
    TxPackets.clear();
    xSemaphoreGive(txQueue_binary_sem);

    if (packetsAdded == 0) {
        ets_printf("send(): no packets added");
        return false;
    }

    startTransmission();
    return true;
}

/**
     * Send a single packet, to help reveiced the answer in another frequency
     */
bool IRAM_ATTR iohcRadio::sendSingle(iohcPacket *packet) {
    if (!packet) return false;
    if (TxPacketWrapper *wrapper = createPacketWrapper(packet))
        txQueue.push_back(wrapper);
    else {
        return false;
    }
    startTransmission();
    return true;
}

/**
    * Insère un paquet en priorité (au début de la queue)
    * Pour les réponses urgentes
    */
bool IRAM_ATTR iohcRadio::sendPriority(iohcPacket *packet) {
    if (!packet) return false;
    // O(1)
    if (TxPacketWrapper *wrapper = createPacketWrapper(packet))
        txQueue.push_front(wrapper);
    else {
        return false;
    }
    startTransmission();
    return true;
}

/**

    */
void IRAM_ATTR iohcRadio::packetSender(iohcRadio *radio) {
    if (!radio || !radio->currentTxPacket) {
        ets_printf("packetSender called with null pointer\n");
        return;
    }

    // Lock for transmission
    txMode = true;

    iohcPacket *pkt = radio->currentTxPacket->packet;

    // Use the frequency of the packet if specified. 1W is always set at CHANNEL2
    if (pkt->frequency == 0) {
        pkt->frequency = radio->scan_freqs[radio->currentFreqIdx];
    }
    // ...unless a TX channel has been forced. The 2.4 GHz remote does not
    // keep to one channel the way the 868 stack assumes -- see txFreqOverride.
    if (iohcRadio::txFreqOverride) {
        pkt->frequency = iohcRadio::txFreqOverride;
    }
    // Only change frequency if necessary
    // This assumes a function Radio::getCurrentFrequency() exists or can be implemented
    // to avoid redundant SPI writes. For now, we set it.
    Radio::setCarrier(Radio::Carrier::Frequency, pkt->frequency);

    radio->setRadioState(RadioState::TX);
    // buffer_length excludes the io-homecontrol CRC -- the SX1276 generates
    // it in hardware, and the RF231 driver appends it in software before
    // handing the frame to the PHY.
    Radio::writeFrame(pkt->payload.buffer, pkt->buffer_length);
    // There is no need to maintain radio locked between packets transmission unless clearly asked
    txMode = pkt->lock;

    packetStamp = esp_timer_get_time();

    pkt->decode(true);
    // Memorize last command sent. This runs on the esp_timer task, while the
    // packet decoder task reads it to answer a challenge -- see
    // iohc_last_command.h for why that cannot be two bare globals.
    const std::vector<uint8_t> sent = pkt->data();
    iohcLastCommandSet(pkt->cmd(), sent.data(), sent.size());

    if (pkt->repeat > 0) {
        // Only the first frame is LPM (1W)
        pkt->payload.packet.header.CtrlByte2.asStruct.LPM = 0;
        pkt->repeat--;
    }
    if (pkt->repeat == 0) {
        radio->Ticker.detach();

        // Check if there is a delay before the next packet
        bool hasNextPacket = false;
        uint32_t nextDelay = 0;

        // QUICK CHECK without modifying the queue
        if (!radio->txQueue_busy && !radio->txQueue.empty()) {
            hasNextPacket = true;
            nextDelay = radio->txQueue.front()->repeatTime;
        }

        if (hasNextPacket && nextDelay > 0) {
            // Delay before next packet
            radio->Ticker.attach_ms(nextDelay, processNextPacketCallback, radio);
        } else if (hasNextPacket) {
            // No delay, process immediately
            radio->processNextPacket();
        } else {
            // No more packets to send
            txMode = false;

            radio->stopTransmission();
        }
    }
}

/**

    */
bool IRAM_ATTR iohcRadio::sent(iohcPacket *packet) {
    bool ret = false;
    if (txCB) ret = txCB(packet);
    return ret;
}

void iohcRadio::startPacketProcessor() {
    /* 8 KB, was 4 KB.
     *
     * This task runs msgRcvd(), and msgRcvd() grew. It already did AES over a
     * challenge and built several std::vectors; it now also looks devices up in
     * two mutex-protected tables on every frame, and the front end made those
     * paths run constantly rather than once when somebody typed a command.
     *
     * A FreeRTOS stack overflow here does not announce itself as one -- the
     * canary watchpoint fires and the chip reports an ordinary panic, which is
     * precisely the unexplained crash this rig kept hitting after the web UI
     * landed. `bootstat` now prints this task's high-water mark so the margin
     * is a measurement rather than a hope. */
    BaseType_t result = xTaskCreatePinnedToCore(packetProcessorTask,  // Function
                                                "PacketProcessor",    // Name
                                                8192,                 // Stack size
                                                this,                 // Parameter
                                                5,                    // Priority
                                                &packetProcessorTaskHandle, xPortGetCoreID());

    if (result != pdPASS) {
        ets_printf("Failed to create packet processor task\n");
    }
}

void IRAM_ATTR iohcRadio::packetProcessorTask(void *parameter) {
    auto radio = static_cast<iohcRadio *>(parameter);
    iohcPacket receivedPacket;

    ets_printf("Packet processor task started\n");

    while (true) {
        if (xQueueReceive(radio->packetQueue, &receivedPacket, portMAX_DELAY) == pdTRUE) {

            // **VALIDATION WITHOUT INTERRUPTION OF THE FLOW**
            bool shouldDecode = true;
            std::string rejectReason;

            // Rejection criteria (but we maintain the flow)
            if (receivedPacket.buffer_length == 0) {
                shouldDecode = false;
                rejectReason = "Empty packet";
            } else if (receivedPacket.buffer_length < 9 /*MIN_PACKET_LENGTH*/) {
                shouldDecode = false;
                rejectReason = "Too short";
            }

            if (!shouldDecode)
                ets_printf("RX dropped: %s (%u bytes)\n", rejectReason.c_str(),
                           receivedPacket.buffer_length);

            if (shouldDecode)
                if (xSemaphoreTake(radio->lastPacketMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    last1wPacket = receivedPacket;
                    xSemaphoreGive(radio->lastPacketMutex);
                }

            // Process the package
            // **ALWAYS log, even rejections**
            if (!shouldDecode) {
            } else {
                // Call the callback if defined
                if (radio->rxCB) {
                    radio->rxCB(&receivedPacket);
                }

                // Decode the packet
                receivedPacket.decode(true);
            }
        }
    }
}

bool IRAM_ATTR iohcRadio::receive(bool stats, const RadioIrqEvent &evt) {

    // Use a local TEMPORARY buffer
    tempRxPacket.reset();
    // The channel the frame ACTUALLY arrived on, not the one the scan list
    // says we ought to be on. Five handlers in main.cpp answer with
    // `response.frequency = receivedPacket->frequency`, i.e. "reply on the
    // channel I heard you on" -- and with scan_freqs[currentFreqIdx] that
    // always evaluated to CHANNEL2, so every reply would go out on ch20
    // however the frame reached us. On 2.4 GHz that is simply wrong: the
    // remote hops, and TX leaves the shared channel register wherever it
    // last pointed, so RX genuinely does move between channels.
    tempRxPacket.frequency =
        RF231_CH_BASE_HZ +
        static_cast<uint32_t>(Radio::channel() - RF231_CH_MIN) * RF231_CH_SPACING_HZ;

    packetStamp = evt.timestamp_us;  //_g_payload_millis;

    // One call, whatever the radio underneath: the SX1276 drains its FIFO,
    // the RF231 bursts its frame buffer out. Link stats come back with it.
    tempRxPacket.buffer_length = Radio::readFrame(tempRxPacket.payload.buffer, MAX_FRAME_LEN);
    const Radio::FrameStats &fs = Radio::lastFrameStats();

    // Nothing to hand up: a bad PHR, or one of Velux's 5-byte wake-up frames,
    // which the driver counts and summarises itself (see RF231Helpers.cpp).
    // Queueing an empty packet only costs the processor a reject.
    if (tempRxPacket.buffer_length == 0) {
        setRadioState(RadioState::RX);
        return false;
    }

    if (stats) {
        tempRxPacket.rssi = fs.rssiDbm;
        tempRxPacket.snr = fs.snrDb;
        tempRxPacket.afc = fs.afcHz;
    }

    // MAX_FRAME_LEN is 32 -- an io-homecontrol frame never exceeds it. On
    // 2.4 GHz the band is shared with a neighbouring Thread network whose
    // frames run to 119 bytes, and a truncated one of those is not a short
    // io-home frame. Drop rather than hand up something misleading.
    if (fs.airLength > MAX_FRAME_LEN) {
        packetsDropped++;
        setRadioState(RadioState::RX);
        return false;
    }

    // **ALWAYS send to queue, even "invalid" packets**
    // Rejection decisions are made in the processor, not here
    // Send a COPY to the queue (thread-safe)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t queueResult =
        xQueueSendFromISR(packetQueue, &tempRxPacket, &xHigherPriorityTaskWoken);

    setRadioState(RadioState::RX);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    return (queueResult == pdTRUE);
}

void IRAM_ATTR iohcRadio::setRadioState(const RadioState newState) {
    radioState = newState;
}

/**
    * Static callback for inter-packet delay
    */
void IRAM_ATTR iohcRadio::processNextPacketCallback(iohcRadio *radio) {
    if (radio) {
        radio->processNextPacket();
    }
}

/**
    * Starts transmission if not already in progress
    */
void IRAM_ATTR iohcRadio::startTransmission() {
    if (xSemaphoreTake(tx_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    if (!isSending) {
        isSending = true;
        xSemaphoreGive(tx_mutex);

        // Launch the first packet
        processNextPacket();
    } else {
        xSemaphoreGive(tx_mutex);
    }
}

/**
     * Process Next Packet in Queue
     */
bool IRAM_ATTR iohcRadio::processNextPacket() {
    // Clean current packet
    if (currentTxPacket) {
        delete currentTxPacket;
        currentTxPacket = nullptr;
    }

    if (txQueue.empty()) {
        stopTransmission();
        return false;
    }

    currentTxPacket = txQueue.front();
    txQueue.pop_front();
    // Check validity
    if (!currentTxPacket || !currentTxPacket->packet) {
        ets_printf("processNextPacket(): Invalid packet!\n");
        stopTransmission();
        return false;
    }

    // Start packet timer
    uint32_t repeatTime = currentTxPacket->repeatTime;
    if (repeatTime == 0) repeatTime = 50;  // Default if not specified

    // Reattach the Ticker
    Ticker.attach_ms(repeatTime, packetSender, this);

    return true;
}

/**
     * Stop Transmission
     */
void IRAM_ATTR iohcRadio::stopTransmission() {
    if (xSemaphoreTake(tx_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        isSending = false;
        Ticker.detach();

        // Clean current packet
        if (currentTxPacket) {
            delete currentTxPacket;
            currentTxPacket = nullptr;
        }
        // Mark as not sending
        isSending = false;

        txMode = false;

        xSemaphoreGive(tx_mutex);
    }
}

/**
     * Clean TX Queue
     */
void IRAM_ATTR iohcRadio::clearTxQueue() {
    while (!txQueue.empty()) {
        const TxPacketWrapper *wrapper = txQueue.front();
        txQueue.pop_front();
        delete wrapper;  // The destructor frees the memory
    }
}

/**
    * Check if currently transmitting
    */
bool IRAM_ATTR iohcRadio::isTransmitting() const {
    bool transmitting = false;
    if (xSemaphoreTake(tx_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        transmitting = isSending;
        xSemaphoreGive(tx_mutex);
    }
    return transmitting;
}

/**
     * Clean TX Queue and stop transmission
     */
void IRAM_ATTR iohcRadio::cancelTransmissions() {
    Ticker.detach();
    clearTxQueue();
    stopTransmission();
}
}  // namespace IOHC
