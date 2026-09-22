/*
 * The phone-shaped front end: a small HTTP server and a JSON API over the
 * things the console already does.
 *
 * Why a synchronous WebServer and not an async one: the async stack this
 * project used to carry (ESPAsyncWebServer / AsyncTCP) predates arduino-esp32
 * 3.x and no longer compiles -- platformio.ini says so at length, and the
 * lib_ignore list exists to keep the linker's dependency finder from dragging
 * it back in. WebServer ships with the framework, needs no new dependency, and
 * the load here is one phone polling a few kilobytes a second.
 *
 * It runs on its own task rather than out of loop(). Handlers build JSON and
 * touch NVS, and loop() is also where an Arduino sketch's stack is smallest;
 * more to the point, a task can wait for WiFi on its own instead of every
 * handler having to cope with the interface being down.
 *
 * Nothing here transmits. Every actuator command goes on the queue in
 * iohc_control.h, so a tap on a phone returns immediately and cannot interleave
 * with a command the console issued half a second earlier.
 *
 * The page is served from flash, gzipped, generated from web/ at build time by
 * scripts/gen_web_assets.py. There is no filesystem involved: LittleFS is not
 * mounted (main.cpp explains why), and an asset in the image is one artifact to
 * flash rather than two, which matters on a rig that is updated over the air.
 */

#ifndef WEB_UI_H
#define WEB_UI_H

#include <cstdint>

/// Start the server task. Safe to call before WiFi is up -- it binds when the
/// interface comes up, like the network console does.
void webUiBegin(uint16_t port = 80);

/// One line for the console's `net`-style status.
void webUiStatus();

/// Bytes of stack the server task has never used. The state document is built
/// on this stack as an Arduino String, so it is worth being able to measure
/// rather than assume.
uint32_t webUiStackHeadroom();

#endif  // WEB_UI_H
