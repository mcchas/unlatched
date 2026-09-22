/*
   Network console for the io-homecontrol rig. See net_console.h for why the
   output path is hooked at the ROM rather than at the call sites.
 */

#include <net_console.h>

#include <ArduinoOTA.h>
#include <rf231_helpers.h>
#include <WiFi.h>
#include <atomic>
#include <cstring>

#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "esp_rom_uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace {

// ------------------------------------------------------------------ output

// 16 KB. The frame trace can emit a 250-character line per frame and the band
// produces bursts of them, so a small ring drops exactly the traffic spike you
// most wanted to see. This is BSS, not heap: the allocation must never fail.
constexpr size_t RING_SIZE = 16384;
char s_ring[RING_SIZE];

/* --- console output that survives a crash ----------------------------------
 *
 * The ring above is useless for the one thing you most want it for. A panic
 * prints its message and backtrace through this same putc hook, but nothing
 * runs afterwards to push the ring out of the socket, and the next boot clears
 * it. So the rig can crash, reboot and leave no account of itself -- which is
 * exactly what happened twice while bringing the front end up.
 *
 * RTC slow memory is not cleared by a software reset or a panic reset, only by
 * a power cycle. A small ring there costs a kilobyte and turns "it panicked"
 * into the fault, the address and the backtrace. RTC_NOINIT_ATTR means it is
 * genuinely not initialised, including on first power-on, so a magic word
 * decides whether what is in there is ours or leftover noise. */
constexpr size_t PANIC_SIZE = 1536;
constexpr uint32_t PANIC_MAGIC = 0x56454C58;  // 'VELX'

RTC_NOINIT_ATTR char s_panicRing[PANIC_SIZE];
RTC_NOINIT_ATTR uint32_t s_panicPos;
RTC_NOINIT_ATTR uint32_t s_panicMagic;

/// The previous boot's tail, copied out of RTC memory before the new boot
/// starts overwriting it. Ordinary BSS: once it is here it is safe.
char s_prevTail[PANIC_SIZE + 1];
bool s_havePrevTail = false;

/// Characters ever emitted, not an index. Readers hold an absolute position in
/// this count and work out for themselves whether it is still in the window,
/// which is what lets the telnet client and any number of browsers read the
/// same output independently. The previous design had the single reader consume
/// the ring, so a second one was impossible without stealing the first's bytes.
std::atomic<uint32_t> s_written{0};

/// Characters that fell out of the window before the telnet client read them.
std::atomic<uint32_t> s_dropped{0};

WiFiServer *s_server = nullptr;
WiFiClient s_client;
uint32_t s_clientCursor = 0;
uint16_t s_port = 23;
bool s_otaReady = false;

// Input typed by a console client, consumed by Cmd::cmdReceived().
constexpr size_t IN_SIZE = 512;
char s_in[IN_SIZE];
std::atomic<size_t> s_inHead{0};
std::atomic<size_t> s_inTail{0};
/// The input ring has one reader but now two writers -- the telnet task and the
/// web server's -- so producing into it is serialised. The output ring is not
/// locked: it is written from the receive path and must never block there.
SemaphoreHandle_t s_inLock = nullptr;

/// Every character ets_printf() produces passes through here. Runs in the
/// caller's context -- radio task, trace pump, possibly an ISR -- so it does the
/// minimum: hand the character to the UART as before, then copy it into the
/// ring.
///
/// The reservation is a single fetch_add, so concurrent callers cannot corrupt
/// the position even though several tasks print. What they can do is interleave
/// their characters, which was already true of the UART output and is why the
/// frame trace is emitted as one atomic printf.
void IRAM_ATTR dualPutc(char c) {
    // esp_rom_output_tx_one_char, not the deprecated esp_rom_uart_tx_one_char
    // alias: this is the symbol the crash capture wraps, so routing ordinary
    // console output through it too means the RTC ring holds what the rig was
    // doing as well as how it died. (It also silences a deprecation warning
    // that has been in every build of this file.)
    esp_rom_output_tx_one_char(static_cast<uint8_t>(c));

    const uint32_t pos = s_written.fetch_add(1, std::memory_order_relaxed);
    s_ring[pos % RING_SIZE] = c;
    // The RTC copy is NOT made here. It is made in the UART wrapper below,
    // which sees this character too -- and also sees the panic handler's, which
    // never come through this hook at all.
}

}  // namespace

/* The last thing the rig says before it dies.
 *
 * The first attempt at this copied characters into RTC memory from dualPutc,
 * the esp_rom_printf channel hook. It captured everything except the one thing
 * it was built for: the panic handler does not use that channel, it calls
 * esp_rom_output_tx_one_char() directly, precisely so that a broken hook
 * cannot suppress a crash report. So the capture caught the console output
 * leading up to the fault and none of the fault itself.
 *
 * Wrapping that symbol at link time (-Wl,--wrap in platformio.ini) puts this
 * function underneath every character the chip prints, the panic handler's
 * included. It runs in panic context -- no scheduler, interrupts off -- so it
 * does nothing but store a byte and hand it on.
 */
extern "C" int __real_esp_rom_output_tx_one_char(uint8_t c);

extern "C" int IRAM_ATTR __wrap_esp_rom_output_tx_one_char(uint8_t c) {
    s_panicRing[s_panicPos % PANIC_SIZE] = static_cast<char>(c);
    s_panicPos++;
    return __real_esp_rom_output_tx_one_char(c);
}

namespace {

/// Read from an absolute cursor. Shared by the telnet client and the web
/// console; see net_console.h.
size_t logRead(uint32_t *cursor, char *dst, const size_t maxLen, uint32_t *lost) {
    if (lost) *lost = 0;
    const uint32_t head = s_written.load(std::memory_order_acquire);
    uint32_t pos = *cursor;

    // Ahead of the writer: only possible after a reboot, when a browser comes
    // back holding a cursor from the previous boot's counter. Start it over
    // rather than handing back 4 GB of nothing.
    if (static_cast<int32_t>(head - pos) < 0) pos = head;

    const uint32_t oldest = head > RING_SIZE ? head - RING_SIZE : 0;
    if (static_cast<int32_t>(pos - oldest) < 0) {
        if (lost) *lost = oldest - pos;
        pos = oldest;
    }

    size_t n = 0;
    while (n < maxLen && pos != head) {
        dst[n++] = s_ring[pos % RING_SIZE];
        pos++;
    }
    *cursor = pos;
    return n;
}

/// Append one character to the input ring. Returns false when it is full.
/// Every caller holds s_inLock: there are two producers now, and a line typed
/// in the browser interleaving character-by-character with one typed over
/// telnet would hand the parser a word neither of them wrote.
bool pushInput(const char c) {
    const size_t head = s_inHead.load(std::memory_order_relaxed);
    const size_t next = (head + 1) % IN_SIZE;
    if (next == s_inTail.load(std::memory_order_acquire)) return false;
    s_in[head] = c;
    s_inHead.store(next, std::memory_order_release);
    return true;
}

// ------------------------------------------------------------------- task

void netTask(void *) {
    bool announced = false;
    uint8_t chunk[512];

    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            if (announced) {
                announced = false;
                s_otaReady = false;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (!announced) {
            announced = true;
            if (!s_server) {
                s_server = new WiFiServer(s_port);
                s_server->begin();
                s_server->setNoDelay(true);  // a console is latency-bound, not throughput-bound
            }
            if (!s_otaReady) {
                // The ESP8266's historic OTA port rather than the ESP32 default
                // of 3232. OTA needs the DEVICE to open a fresh TCP connection
                // back to the host, and on this network that is what gets
                // dropped -- the rig sits on a different subnet from the
                // workstation, and only the UDP reply (return traffic a stateful
                // firewall already permits) gets through. These ports are open
                // here, so pin both ends to them: device 8266, host 8267.
                ArduinoOTA.setPort(8266);
                ArduinoOTA.setHostname("velux-rig");
                // No mDNS. ArduinoOTA advertises itself over it by default,
                // which costs 37 KiB of image for a service nothing here uses:
                // mDNS does not cross subnets, and the rig sits on a different
                // one from the workstation, so both the OTA upload and rig.py
                // address it by IP. tools/rig.py says the same in its header.
                ArduinoOTA.setMdnsEnabled(false);
                // Deliberately no OTA password: this is a lab rig on the user's
                // own network, and a forgotten password on a device that lives
                // out of reach of a USB cable is worse than the exposure.
                ArduinoOTA.onStart([]() {
                    // Quiesce the radio for the duration. The ISR itself is
                    // IRAM-resident so it survives the cache being disabled for
                    // flash writes, but the decoder task behind it is not, and a
                    // busy band keeps it and the trace running for the whole
                    // ~1.8MB write. An upload dying at 83% is what that looks like.
                    Radio::traceFrames(false);
                    Radio::setStandby();
                    ets_printf("\nOTA starting -- radio parked, console paused\n");
                });
                ArduinoOTA.onEnd([]() { ets_printf("\nOTA done, rebooting\n"); });
                ArduinoOTA.onError([](ota_error_t e) { ets_printf("\nOTA error %u\n", e); });
                ArduinoOTA.begin();
                s_otaReady = true;
            }
            ets_printf("\nnet console up: telnet %s %u   OTA host 'velux-rig'\n",
                       WiFi.localIP().toString().c_str(), s_port);
        }

        ArduinoOTA.handle();

        if (s_server && s_server->hasClient()) {
            WiFiClient incoming = s_server->available();
            if (s_client && s_client.connected()) {
                // One console at a time: two clients interleaving a byte-stream
                // trace would corrupt both views of it.
                incoming.println("velux-rig: console already in use");
                incoming.stop();
            } else {
                s_client = incoming;
                s_client.setNoDelay(true);
                // Start at the live end rather than replaying the whole buffer:
                // a telnet session is for watching what happens next. The web
                // console asks for history explicitly instead.
                s_clientCursor = s_written.load(std::memory_order_acquire);
                s_client.printf(
                    "velux-rig console. Type 'help'. %lu chars dropped so far.\r\n",
                    static_cast<unsigned long>(s_dropped.load(std::memory_order_relaxed)));
            }
        }

        if (s_client && s_client.connected()) {
            // Client -> command parser.
            if (s_client.available() && xSemaphoreTake(s_inLock, pdMS_TO_TICKS(20)) == pdTRUE) {
                while (s_client.available()) {
                    const int c = s_client.read();
                    if (c < 0) break;
                    if (!pushInput(static_cast<char>(c))) break;  // full; drop
                }
                xSemaphoreGive(s_inLock);
            }
            // Ring -> client. Bounded: under a frame trace on a busy band the
            // writer can outpace the socket indefinitely, and an unbounded
            // drain would never reach the delay below -- starving OTA, which
            // is handled by this same task, exactly when someone is trying to
            // flash a fix for whatever is producing the traffic.
            size_t n;
            uint32_t lost = 0;
            for (uint8_t pass = 0; pass < 8; pass++) {
                n = logRead(&s_clientCursor, reinterpret_cast<char *>(chunk), sizeof(chunk), &lost);
                if (!n) break;
                if (lost) s_dropped.fetch_add(lost, std::memory_order_relaxed);
                if (s_client.write(chunk, n) != n) break;  // client gone or backed up
            }
        } else if (s_client) {
            s_client.stop();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

}  // namespace

// ------------------------------------------------------------------- API

/// Lift the previous boot's output out of RTC memory and start a fresh one.
/// Must run before the putc hook is installed, or this boot's own first
/// characters overwrite the thing we came to read.
void capturePreviousTail() {
    if (s_panicMagic == PANIC_MAGIC && s_panicPos) {
        const uint32_t total = s_panicPos;
        const uint32_t have = total < PANIC_SIZE ? total : PANIC_SIZE;
        const uint32_t start = total - have;
        for (uint32_t i = 0; i < have; i++) s_prevTail[i] = s_panicRing[(start + i) % PANIC_SIZE];
        s_prevTail[have] = '\0';
        s_havePrevTail = true;
    }
    s_panicMagic = PANIC_MAGIC;
    s_panicPos = 0;
}

void netConsoleBegin(const uint16_t port) {
    s_port = port;
    capturePreviousTail();
    s_inLock = xSemaphoreCreateMutex();
    // Installed before WiFi is up so boot output is already buffered for the
    // first client to connect. Channel 1 is the ROM's default output channel.
    esp_rom_install_channel_putc(1, dualPutc);
    // 16KB, not 8: ArduinoOTA's update path runs on this task and carries
    // Update plus an MD5 context plus a WiFiClient on the same stack. Normally
    // it runs on loop()'s, which is larger; an overflow here presents as the
    // device resetting the connection partway through an upload.
    xTaskCreate(netTask, "netconsole", 16384, nullptr, 1, nullptr);
}

size_t netConsoleRead(uint8_t *dst, const size_t maxLen) {
    size_t n = 0;
    size_t tail = s_inTail.load(std::memory_order_relaxed);
    const size_t head = s_inHead.load(std::memory_order_acquire);
    while (n < maxLen && tail != head) {
        dst[n++] = static_cast<uint8_t>(s_in[tail]);
        tail = (tail + 1) % IN_SIZE;
    }
    s_inTail.store(tail, std::memory_order_release);
    return n;
}

bool netConsoleInject(const char *line) {
    if (!line || !s_inLock) return false;
    if (xSemaphoreTake(s_inLock, pdMS_TO_TICKS(200)) != pdTRUE) return false;
    // Echo into the ring, so every viewer sees what was run. A line typed over
    // telnet is echoed back down that socket by the parser, which reaches the
    // person who typed it and nobody else -- so a command sent from the browser
    // would otherwise produce output with nothing above it saying what asked
    // for it.
    ets_printf("> %s\n", line);
    bool ok = true;
    const size_t len = strlen(line);
    for (size_t i = 0; i < len && ok; i++) {
        if (line[i] == '\r' || line[i] == '\n') continue;  // terminator is added below
        ok = pushInput(line[i]);
    }
    // The parser recognises a line by its terminator, so an injected command
    // that does not carry one would sit in the buffer until somebody typed.
    if (ok) ok = pushInput('\n');
    xSemaphoreGive(s_inLock);
    return ok;
}

bool netConsoleHavePreviousTail() {
    return s_havePrevTail;
}

void netConsoleDumpPreviousTail() {
    if (!s_havePrevTail) {
        ets_printf("no console output survives from the previous boot\n");
        return;
    }
    ets_printf("---- last %u characters before the previous reset ----\n",
               static_cast<unsigned>(strlen(s_prevTail)));
    // One character at a time through the same hook, so it lands in this boot's
    // ring and reaches whoever is watching, however long it is.
    for (const char *p = s_prevTail; *p; p++) ets_printf("%c", *p);
    ets_printf("\n---- end ----\n");
}

uint32_t netConsoleLogPos() {
    return s_written.load(std::memory_order_acquire);
}

size_t netConsoleLogRead(uint32_t *cursor, char *dst, const size_t maxLen, uint32_t *lost) {
    if (!cursor || !dst || !maxLen) return 0;
    return logRead(cursor, dst, maxLen, lost);
}

void netConsoleEcho(const char *s) {
    if (s_client && s_client.connected()) s_client.print(s);
}

bool netConsoleConnected() {
    return s_client && s_client.connected();
}

IPAddress netConsoleIp() {
    return WiFi.localIP();
}

void netConsoleStatus() {
    ets_printf("net: wifi=%s ip=%s telnet=%u client=%s ota=%s emitted=%lu dropped=%lu\n",
               WiFi.status() == WL_CONNECTED ? "up" : "down", WiFi.localIP().toString().c_str(),
               s_port, netConsoleConnected() ? "connected" : "none", s_otaReady ? "ready" : "off",
               static_cast<unsigned long>(s_written.load(std::memory_order_relaxed)),
               static_cast<unsigned long>(s_dropped.load(std::memory_order_relaxed)));
}
