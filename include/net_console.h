/*
 * Network console: monitor, control and update the rig without serial.
 *
 * Needed because the rig has to sit within radio range of a skylight, which is
 * nowhere near a USB cable. Three things have to work over WiFi for that to be
 * usable: seeing the output, issuing commands, and flashing new firmware.
 *
 * OUTPUT is the interesting part. Every diagnostic in this firmware -- including
 * the radio frame trace, which is the whole point of watching it remotely --
 * goes through ets_printf(), the ROM printf, which writes straight to the UART
 * and cannot be redirected by esp_log_set_vprintf() or by reopening stdout.
 * Rather than edit several hundred call sites, we install our own ROM character
 * handler with esp_rom_install_channel_putc(): every character ets_printf emits
 * then passes through one function, which forwards it to the UART exactly as
 * before AND copies it into a ring buffer that a task drains to the TCP client.
 * Nothing above this file changes, and output that already existed is captured.
 *
 * That handler runs in whatever context called ets_printf -- the radio task, the
 * trace pump, potentially an interrupt -- so it must never block, allocate, or
 * touch the network stack. It only writes to the ring, and drops characters when
 * the ring is full rather than stalling the caller: losing a trace line matters
 * far less than stalling the receive path that produced it.
 *
 * A note on interference: the ESP32-C6's own WiFi is 2.4 GHz, the same band as
 * the RF231, sitting centimetres away. WiFi transmissions will desensitise the
 * receiver while they happen. Console and OTA traffic is light and bursty, so
 * this is a real but intermittent cost -- worth knowing if frames seem to vanish
 * exactly when you are typing.
 */

#ifndef NET_CONSOLE_H
#define NET_CONSOLE_H

#include <Arduino.h>
#include <IPAddress.h>

/// Start the telnet console and OTA updates. Safe to call before WiFi is up:
/// the server binds when the interface comes up, and the output hook is
/// installed immediately so early boot messages are buffered for whoever
/// connects first. @p port is the telnet port (23 by default).
void netConsoleBegin(uint16_t port = 23);

/// Pull pending bytes typed by the network client. Returns 0 when none.
/// Cmd::cmdReceived() calls this so a telnet client drives exactly the same
/// command parser as the serial port, rather than a second, diverging one.
size_t netConsoleRead(uint8_t *dst, size_t maxLen);

/// Feed a command line in as if it had been typed. The web console's input box
/// goes through here, so it drives the same parser as telnet and the UART --
/// a third command path would have drifted from the other two the first time
/// any of them changed. A newline is appended if @p line does not end with one.
/// False when the input buffer is full.
bool netConsoleInject(const char *line);

/* The output ring has several readers: the telnet client, and every browser
 * with the console tab open. Each holds its own cursor -- an absolute count of
 * characters ever emitted -- rather than the ring consuming what it hands out,
 * which is what the single-reader version did. A reader that falls more than
 * the ring behind loses the oldest output rather than stalling the writer: the
 * writer can be the radio's receive path, and dropping a log line matters far
 * less than stalling the path that produced it. */

/* A second, much smaller copy of the output lives in RTC memory, which a
 * software reset and a panic reset both leave alone. It exists because the ring
 * above cannot answer the question that matters most: a panic prints its
 * message and backtrace through the same hook, but nothing runs afterwards to
 * send them anywhere and the next boot clears the ring. The RTC copy is lifted
 * out at startup, before this boot overwrites it. */

/// True when the previous boot left output behind -- i.e. it did not simply
/// power on cold.
bool netConsoleHavePreviousTail();

/// Print what the previous boot said just before it stopped. On a panic that
/// includes the fault and the backtrace, which is the whole reason this exists.
void netConsoleDumpPreviousTail();

/// Characters emitted since boot. A new reader starts here to see only what
/// happens from now on, or at 0 to be given the whole buffered history.
uint32_t netConsoleLogPos();

/// Copy output from @p cursor, advancing it. Returns the number of bytes
/// written to @p dst. When the cursor had fallen out of the buffer, @p lost
/// receives how many characters were skipped and the cursor jumps to the
/// oldest character still held.
size_t netConsoleLogRead(uint32_t *cursor, char *dst, size_t maxLen, uint32_t *lost);

/// Echo back to the network client only (not the UART), for command echo.
void netConsoleEcho(const char *s);

bool netConsoleConnected();
IPAddress netConsoleIp();

/// One-line status: IP, client, OTA, and how many output characters have been
/// dropped for a full ring -- a non-zero drop count means the trace you are
/// reading over the network has holes in it, which is worth knowing before
/// concluding anything from it.
void netConsoleStatus();

#endif  // NET_CONSOLE_H
