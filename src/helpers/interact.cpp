#include <interact.h>
#include <iohc_radio.h>  // was reaching us via the deleted iohcOther2W.h
#include <net_console.h>
#include <nvs_helpers.h>
#include <keystore.h>
#include <boot_guard.h>
#include <app_settings.h>
#include <iohc_control.h>
#include <mqtt_bridge.h>
#include <pairing.h>
#include <web_ui.h>

/// Defined in main.cpp, at global scope. Declared here rather than inside the
/// command lambda: that lambda lives in namespace Cmd, so an extern there names
/// Cmd::keyTransferMode and fails to link.
extern uint8_t pairReplyMode;
extern uint16_t advertisedType;
extern uint8_t advertisedManu;
extern bool answer2A;
extern uint8_t systemKey[16];
extern bool haveSystemKey;
extern uint8_t ourAddress[3];
void skyCommand(Tokens *cmd);
extern uint8_t replyCmd29;
extern uint8_t replyData29[16];
extern uint8_t replyData29Len;

/* A `ConnState mqttStatus` used to live here, written by nothing and read by
 * nothing since the original MQTT handler was deleted. The bridge keeps its own
 * state and reports it through mqttGetStatus(); a second variable claiming to
 * hold the same thing is the one that would be believed after it went stale. */

_cmdEntry *_cmdHandler[MAXCMDS];
uint8_t lastEntry = 0;

/* Was std::getline over a std::stringstream. Replaced to keep C++ iostreams --
 * and the 57 KiB of libstdc++ locale tables they drag in -- out of the image.
 *
 * The loop condition reproduces getline's treatment of a trailing delimiter:
 * "sky close " yields ONE token, not two, because getline fails at eof rather
 * than returning a final empty field. Every `cmd->size()` check in this file
 * depends on that, so a console line typed with a trailing space must not
 * suddenly parse differently. Interior empty fields ("a  b" -> "a", "", "b")
 * are preserved, as getline preserved them. */
void tokenize(std::string const &str, const char delim, Tokens &out) {
    size_t start = 0;
    while (start < str.size()) {
        const size_t pos = str.find(delim, start);
        if (pos == std::string::npos) {
            out.push_back(str.substr(start));
            return;
        }
        out.push_back(str.substr(start, pos - start));
        start = pos + 1;
    }
}

namespace Cmd {
bool verbosity = true;
bool pairMode = false;
bool scanMode = false;

TimersUS::TickerUsESP32 kbd_tick;
TimerHandle_t consoleTimer;

static char _rxbuffer[512];
static size_t _len = 0;
static size_t _avail = 0;

/**
     * The function `createCommands()` initializes and adds various command handlers for controlling
     * different devices and functionalities.
     */
void createCommands() {
    Cmd::addHandler("verbose", "Toggle verbose output on packets list",
                    [](Tokens *cmd) -> void { verbosity = !verbosity; });
    Cmd::addHandler("pairMode", "Pair mode: pairMode [0|1], no arg reports",
                    [](Tokens *cmd) -> void {
                        // Was a silent blind toggle. Arming for a key extraction and getting
                        // it backwards costs a press cycle, and a press cycle costs a trip to
                        // the remote -- so take an explicit argument and always say where it
                        // ended up. No argument now reports rather than flipping.
                        //
                        // Routed through pairingArm() rather than setting the flag directly,
                        // so that arming from the console shows up on the pairing screen too.
                        if (cmd->size() >= 2) pairingArm(atoi(cmd->at(1).c_str()) != 0);
                        ets_printf("pairMode %s\n", pairMode ? "ON" : "off");
                    });

    // 2.4 GHz bring-up. The band is shared and Velux only transmits while a
    // remote is being pressed, so these exist to tell a quiet channel apart
    // from a receive path that is not working.
    Cmd::addHandler("rfstat", "RF231 state + RX counters",
                    [](Tokens *cmd) -> void { Radio::status(); });
    Cmd::addHandler("rftrace", "Raw frame trace: rftrace [0|1], no arg toggles",
                    [](Tokens *cmd) -> void {
                        static bool on = false;
                        // An explicit argument matters: a blind toggle cannot be driven by a
                        // capture script, which has no way to know the current state.
                        on = (cmd->size() >= 2) ? (atoi(cmd->at(1).c_str()) != 0) : !on;
                        Radio::traceFrames(on);
                    });
    Cmd::addHandler("keymode", "Reply to 0x29: 0=0x2C 1=0x38 key transfer 2=0x2B announce",
                    [](Tokens *cmd) -> void {
                        // No argument reports. It used to reset to 0, which silently disarmed
                        // a rig someone had just armed for a key transfer.
                        if (cmd->size() >= 2)
                            ::pairReplyMode = static_cast<uint8_t>(atoi(cmd->at(1).c_str()));
                        static const char *names[] = {"DISCOVER_ACTUATOR_0x2C",
                                                      "LAUNCH_KEY_TRANSFERT_0x38",
                                                      "DISCOVER_REMOTE_ANSWER_0x2B"};
                        if (::pairReplyMode > 2) ::pairReplyMode = 0;
                        ets_printf("0x29 will be answered with %s\n", names[::pairReplyMode]);
                    });
    Cmd::addHandler("sky", "open|close|stop|vent|pos N|status|info [tgt] [silent]",
                    [](Tokens *cmd) -> void { skyCommand(cmd); });
    Cmd::addHandler(
        "keys", "System keys: list|add|bind|name|remote|del|wipe", [](Tokens *cmd) -> void {
            const std::string sub = cmd->size() >= 2 ? cmd->at(1) : "list";
            uint8_t node[3];
            if (sub == "list") {
                keystoreList();
            } else if (sub == "add") {
                if (cmd->size() < 3 || cmd->at(2).size() < 32) {
                    ets_printf("keys add <32 hex key> [remoteid]\n");
                    return;
                }
                uint8_t k[16];
                for (uint8_t i = 0; i < 16; i++)
                    k[i] = strtol(cmd->at(2).substr(2 * i, 2).c_str(), nullptr, 16);
                uint8_t rem[3] = {};
                const bool haveRem = cmd->size() >= 4 && keystoreParseNode(cmd->at(3).c_str(), rem);
                const int slot = keystoreAdd(k, haveRem ? rem : nullptr);
                if (slot < 0)
                    ets_printf("keystore full\n");
                else
                    ets_printf("key in slot %d -- now: keys bind %d <deviceid>\n", slot, slot);
            } else if (sub == "bind") {
                if (cmd->size() < 4 || !keystoreParseNode(cmd->at(3).c_str(), node)) {
                    ets_printf("keys bind <slot> <deviceid, 6 hex>\n");
                    return;
                }
                const long slot = strtol(cmd->at(2).c_str(), nullptr, 10);
                if (keystoreBind(static_cast<uint8_t>(slot), node))
                    ets_printf("%s bound to slot %ld\n", cmd->at(3).c_str(), slot);
                else
                    ets_printf("could not bind -- empty slot, or the slot is full\n");
            } else if (sub == "name") {
                if (cmd->size() < 4) {
                    ets_printf("keys name <slot> <room>\n");
                    return;
                }
                const long slot = strtol(cmd->at(2).c_str(), nullptr, 10);
                // Join the rest, so a two-word room name works.
                std::string room = cmd->at(3);
                for (size_t i = 4; i < cmd->size(); i++) room += " " + cmd->at(i);
                ets_printf(keystoreSetName(static_cast<uint8_t>(slot), room.c_str())
                               ? "slot named\n"
                               : "no such slot\n");
            } else if (sub == "remote") {
                if (cmd->size() < 4 || !keystoreParseNode(cmd->at(3).c_str(), node)) {
                    ets_printf("keys remote <slot> <remoteid, 6 hex>\n");
                    return;
                }
                const long slot = strtol(cmd->at(2).c_str(), nullptr, 10);
                ets_printf(keystoreSetRemote(static_cast<uint8_t>(slot), node) ? "slot labelled\n"
                                                                               : "no such slot\n");
            } else if (sub == "unbind") {
                if (cmd->size() < 3 || !keystoreParseNode(cmd->at(2).c_str(), node)) {
                    ets_printf("keys unbind <deviceid, 6 hex>\n");
                    return;
                }
                ets_printf(keystoreUnbind(node) ? "unbound\n" : "that device was not bound\n");
            } else if (sub == "del") {
                if (cmd->size() < 3) {
                    ets_printf("keys del <slot>\n");
                    return;
                }
                const long slot = strtol(cmd->at(2).c_str(), nullptr, 10);
                ets_printf(keystoreErase(static_cast<uint8_t>(slot)) ? "slot cleared\n"
                                                                     : "no such slot\n");
            } else if (sub == "wipe") {
                // Spelt out in full: this destroys credentials that cost a trip
                // to each remote to obtain again.
                if (cmd->size() < 3 || cmd->at(2) != "confirm") {
                    ets_printf("this erases every key. `keys wipe confirm` if you mean it\n");
                    return;
                }
                keystoreEraseAll();
                ets_printf("keystore erased\n");
            } else {
                ets_printf(
                    "keys list|add <hex> [remoteid]|bind <slot> <id>|unbind <id>|"
                    "name <slot> <room>|remote <slot> <id>|del <slot>|wipe confirm\n");
            }
        });
    Cmd::addHandler("bootstat", "Boot health: image, last reset, stage, heap, stacks",
                    [](Tokens *cmd) -> void {
                        bootGuardStatus();
                        // High-water marks, in bytes never used. Both tasks were
                        // sized by judgement; a figure in the low hundreds means
                        // the guess was too tight and is about to cost a crash
                        // that looks like something else entirely.
                        ets_printf(
                            "stack headroom: iohc-cmd %lu B, web-ui %lu B, "
                            "PacketProcessor %lu B, mqtt %lu B\n",
                            static_cast<unsigned long>(controlStackHeadroom()),
                            static_cast<unsigned long>(webUiStackHeadroom()),
                            static_cast<unsigned long>(
                                IOHC::iohcRadio::getInstance()->decoderStackHeadroom()),
                            static_cast<unsigned long>(mqttStackHeadroom()));
                    });
    Cmd::addHandler("crashlog", "What the console said before the last reset",
                    [](Tokens *cmd) -> void { netConsoleDumpPreviousTail(); });
    Cmd::addHandler("net", "Network status: ip, client, OTA, drops", [](Tokens *cmd) -> void {
        netConsoleStatus();
        webUiStatus();
        mqttStatusPrint();
        ets_printf("listening on ch%u (rotates between commands)\n", controlListenChannel());
    });
    Cmd::addHandler(
        "mqtt", "MQTT bridge: mqtt | on | off | host <h> [port] | base <prefix> | publish",
        [](Tokens *cmd) -> void {
            // Everything here is also on the settings page. It is repeated on
            // the console because the console is what is reachable when the
            // page is not -- and a broker misconfigured badly enough to matter
            // is exactly the sort of thing that wants fixing from telnet.
            if (cmd->size() < 2) {
                mqttStatusPrint();
                ets_printf(
                    "mqtt on|off | host <host> [port] | user <name> | pass <secret> | "
                    "base <prefix> | id <client-id> | publish\n");
                return;
            }
            AppSettings next = settings();
            const std::string &verb = cmd->at(1);

            if (verb == "on" || verb == "off") {
                next.mqttEnabled = verb == "on";
            } else if (verb == "host" && cmd->size() >= 3) {
                strncpy(next.mqttHost, cmd->at(2).c_str(), sizeof(next.mqttHost) - 1);
                next.mqttHost[sizeof(next.mqttHost) - 1] = '\0';
                if (cmd->size() >= 4)
                    next.mqttPort = static_cast<uint16_t>(atoi(cmd->at(3).c_str()));
            } else if (verb == "user" && cmd->size() >= 3) {
                strncpy(next.mqttUser, cmd->at(2).c_str(), sizeof(next.mqttUser) - 1);
                next.mqttUser[sizeof(next.mqttUser) - 1] = '\0';
            } else if (verb == "pass" && cmd->size() >= 3) {
                strncpy(next.mqttPass, cmd->at(2).c_str(), sizeof(next.mqttPass) - 1);
                next.mqttPass[sizeof(next.mqttPass) - 1] = '\0';
            } else if (verb == "base" && cmd->size() >= 3) {
                strncpy(next.mqttBase, cmd->at(2).c_str(), sizeof(next.mqttBase) - 1);
                next.mqttBase[sizeof(next.mqttBase) - 1] = '\0';
            } else if (verb == "id" && cmd->size() >= 3) {
                strncpy(next.mqttClientId, cmd->at(2).c_str(), sizeof(next.mqttClientId) - 1);
                next.mqttClientId[sizeof(next.mqttClientId) - 1] = '\0';
            } else if (verb == "publish") {
                mqttRepublish();
                ets_printf("mqtt: everything will be republished on the next pass\n");
                return;
            } else {
                ets_printf("mqtt: don't know '%s'\n", verb.c_str());
                return;
            }

            settingsSave(next);
            mqttApplySettings();
            mqttStatusPrint();
        });
    Cmd::addHandler("reboot", "Restart the board", [](Tokens *cmd) -> void {
        ets_printf("rebooting\n");
        delay(200);
        ESP.restart();
    });
    Cmd::addHandler("myaddr", "Our address: myaddr <6 hex> | new", [](Tokens *cmd) -> void {
        // The KLR 200 is looking for a "new or reset control pad". We have
        // presented as ba11ad (the upstream author's hardcoded gateway)
        // across a dozen failed handshakes -- and it answered our discovery
        // the first few times, then stopped. A device that remembers a
        // candidate it already rejected would behave exactly like that, so
        // being able to appear genuinely new is worth a test.
        // iohcOther2W::gateway, and a file-static fake_gateway -- so setting
        // only one left us transmitting a mix of two identities. The address
        // is now owned by main.cpp's ourAddress and that is the only copy
        // anything outside IOHC_LEGACY reads; the legacy modules keep theirs
        // in step below only so their own commands stay coherent.
        uint8_t a[3];
        memcpy(a, ::ourAddress, 3);
        if (cmd->size() >= 2 && cmd->at(1) == "new") {
            for (uint8_t i = 0; i < 3; i++) a[i] = static_cast<uint8_t>(esp_random() & 0xFF);
        } else if (cmd->size() >= 2 && cmd->at(1).size() >= 6) {
            for (uint8_t i = 0; i < 3; i++)
                a[i] =
                    static_cast<uint8_t>(strtol(cmd->at(1).substr(2 * i, 2).c_str(), nullptr, 16));
        }
        memcpy(::ourAddress, a, 3);
        // Persist it. This was RAM-only, so every OTA silently put the rig back
        // to the compiled-in default -- ba11ad at the time, the MAC-derived
        // address now -- and the address is what the installation paired with,
        // so a rig that quietly reverts it stops being recognised by the
        // devices it was enrolled with.
        AppSettings next = settings();
        memcpy(next.ourAddress, a, 3);
        settingsSave(next);
        ets_printf("our address is %02X%02X%02X (saved)\n", a[0], a[1], a[2]);
    });
    Cmd::addHandler(
        "rfreg", "Read/write any RF231 register: rfreg 0C [val]", [](Tokens *cmd) -> void {
            if (cmd->size() < 2) {
                ets_printf("rfreg <hex addr> [hex val]\n");
                return;
            }
            const uint8_t a = static_cast<uint8_t>(strtol(cmd->at(1).c_str(), nullptr, 16));
            if (cmd->size() >= 3) {
                Radio::writeByte(a, static_cast<uint8_t>(strtol(cmd->at(2).c_str(), nullptr, 16)));
            }
            ets_printf("reg 0x%02X = 0x%02X\n", a, Radio::readByte(a));
        });
    Cmd::addHandler("rfsens", "Desensitise RX 0-15 (x3 dB), 0 = full", [](Tokens *cmd) -> void {
        if (cmd->size() < 2) {
            Radio::status();
            return;
        }
        const int v = atoi(cmd->at(1).c_str());
        if (v < 0 || v > 15) {
            ets_printf("rfsens: 0..15\n");
            return;
        }
        Radio::setSensitivity(static_cast<uint8_t>(v));
        ets_printf("RX_PDT_LEVEL=%u (-%u dB)\n", Radio::sensitivity(), Radio::sensitivity() * 3u);
    });
    Cmd::addHandler(
        "rfch", "Set 802.15.4 channel 11-26 (Velux: 15/20/25)", [](Tokens *cmd) -> void {
            if (cmd->size() < 2) {
                Radio::status();
                return;
            }
            const int ch = atoi(cmd->at(1).c_str());
            if (ch < RF231_CH_MIN || ch > RF231_CH_MAX) {
                ets_printf("rfch: channel must be %d..%d\n", RF231_CH_MIN, RF231_CH_MAX);
                return;
            }
            Radio::setCarrier(Radio::Carrier::Frequency,
                              RF231_CH_BASE_HZ + (ch - RF231_CH_MIN) * RF231_CH_SPACING_HZ);
            Radio::setRx();
            Radio::status();
        });

    // Utils
    Cmd::addHandler("dump", "Dump Transceiver registers",
                    [](Tokens *cmd) -> void { Radio::dump(); });
}

bool addHandler(const char *cmd, const char *description, void (*handler)(Tokens *)) {
    for (uint8_t idx = 0; idx < MAXCMDS; ++idx) {
        if (_cmdHandler[idx] != nullptr) {
            //
        } else {
            void *alloc = malloc(sizeof(struct _cmdEntry));
            if (!alloc) return false;

            _cmdHandler[idx] = static_cast<_cmdEntry *>(alloc);
            memset(alloc, 0, sizeof(_cmdEntry));
            // Both copies used to bound themselves by comparing strlen(cmd) --
            // the COMMAND's length -- against the DESTINATION buffer's size, then
            // copy strlen(description) bytes anyway. For any description longer
            // than the 61-byte field that writes past the struct. It corrupted
            // memory at startup and boot-looped the rig with a Store access fault,
            // unreachable over the network; a 71-character help string was all it
            // took. An over-long description now truncates and says so.
            strncpy(_cmdHandler[idx]->cmd, cmd, sizeof(_cmdHandler[idx]->cmd) - 1);
            _cmdHandler[idx]->cmd[sizeof(_cmdHandler[idx]->cmd) - 1] = '\0';
            strncpy(_cmdHandler[idx]->description, description,
                    sizeof(_cmdHandler[idx]->description) - 1);
            _cmdHandler[idx]->description[sizeof(_cmdHandler[idx]->description) - 1] = '\0';
            if (strlen(description) >= sizeof(_cmdHandler[idx]->description))
                ets_printf("!! '%s' help text truncated to %u chars\n", cmd,
                           static_cast<unsigned>(sizeof(_cmdHandler[idx]->description) - 1));
            _cmdHandler[idx]->handler = handler;

            if (idx > lastEntry) lastEntry = idx;
            return true;
        }
    }
    // Silent failure here once made three commands vanish without a trace.
    ets_printf("!! command table full (MAXCMDS=%d): '%s' NOT registered\n", MAXCMDS, cmd);
    return false;
}

const char *cmdReceived(bool echo) {
    // Serial and the network console feed the SAME parser. A separate command
    // path for telnet would drift from this one the first time either changed.
    size_t room = sizeof(_rxbuffer) - _len - 1;

    _avail = Serial.available();
    if (_avail) {
        if (_avail > room) _avail = room;
        _len += Serial.readBytes(&_rxbuffer[_len], _avail);
        room = sizeof(_rxbuffer) - _len - 1;
        if (echo) {
            _rxbuffer[_len] = '\0';
            Serial.printf("%s", &_rxbuffer[_len - _avail]);
        }
    }

    if (room) {
        const size_t n = netConsoleRead(reinterpret_cast<uint8_t *>(&_rxbuffer[_len]), room);
        if (n) {
            _len += n;
            _rxbuffer[_len] = '\0';
            // Echo to the network client only: the UART already shows its own.
            if (echo) netConsoleEcho(&_rxbuffer[_len - n]);
        }
    }

    if (_len == 0) return nullptr;

    if (_rxbuffer[_len - 1] == 0x0a || _rxbuffer[_len - 1] == 0x0d) {
        // Trim CR, LF or CRLF. The old code assumed exactly CRLF and blindly
        // cut two bytes, which ate the last character of a bare-LF line.
        while (_len && (_rxbuffer[_len - 1] == 0x0a || _rxbuffer[_len - 1] == 0x0d))
            _rxbuffer[--_len] = '\0';
        _len = 0;
        return _rxbuffer;
    }

    if (_len >= sizeof(_rxbuffer) - 1) _len = 0;  // overlong line: discard, do not overflow
    return nullptr;
}

void cmdFuncHandler() {
    constexpr char delim = ' ';
    Tokens segments;

    const auto cmd = cmdReceived(true);
    if (!cmd) return;
    if (!strlen(cmd)) return;

    tokenize(cmd, delim, segments);
    if (strcmp("help", segments[0].c_str()) == 0) {
        ets_printf("\nRegistered commands:\n");
        for (uint8_t idx = 0; idx <= lastEntry; ++idx) {
            if (_cmdHandler[idx] == nullptr) continue;
            ets_printf("- %s\t%s\n", _cmdHandler[idx]->cmd, _cmdHandler[idx]->description);
        }
        ets_printf("- %s\t%s\n\n", "help", "This command");
        ets_printf("\n");
        return;
    }
    for (uint8_t idx = 0; idx <= lastEntry; ++idx) {
        if (_cmdHandler[idx] == nullptr) continue;
        if (strcmp(_cmdHandler[idx]->cmd, segments[0].c_str()) == 0) {
            _cmdHandler[idx]->handler(&segments);
            return;
        }
    }
    ets_printf("*> Unknown <*\n");
}

void init() {
    kbd_tick.attach_ms(500, cmdFuncHandler);
}
}  // namespace Cmd
