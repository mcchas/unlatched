#ifndef INTERACT_H
#define INTERACT_H
/*
  MQTT & Command Line interaction
*/
#include <board_config.h>
#include <user_config.h>

#include <tokens.h>
#include <utils.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

enum class ConnState { Connecting, Connected, Disconnected };

namespace IOHC {
class iohcRemote1W;
}

#if defined(ESP32)
#include <ticker_us_esp32.h>
// 60 handlers are registered today. addHandler() fills the table in source
// order and returns false once it is full, and nothing checked that return --
// so commands defined near the end of createCommands() silently did not exist.
// That cost a real experiment: three new commands pushed discover2A off the
// end, and a capture run transmitted nothing while reporting success.
#define MAXCMDS 80
#endif

void tokenize(std::string const &str, const char delim, Tokens &out);

struct _cmdEntry {
    char cmd[15];
    char description[61];
    void (*handler)(Tokens *);
};
extern _cmdEntry *_cmdHandler[MAXCMDS];
extern uint8_t lastEntry;

#if defined(DEBUG)
#ifndef DEBUG_PORT
#define DEBUG_PORT Serial
#endif
#define LOG(func, ...) DEBUG_PORT.func(__VA_ARGS__)
#else
#define LOG(func, ...)
#endif

namespace Cmd {

extern bool verbosity;
extern bool pairMode;
extern bool scanMode;

extern TimersUS::TickerUsESP32 kbd_tick;

extern TimerHandle_t consoleTimer;

bool addHandler(const char *cmd, const char *description, void (*handler)(Tokens *));

const char *cmdReceived(bool echo = false);
void cmdFuncHandler();
void createCommands();
void init();

}  // namespace Cmd

#endif
