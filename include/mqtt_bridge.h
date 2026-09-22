/*
 * The MQTT bridge: the rig's state on a broker, and commands back from one.
 *
 * WHICH CLIENT, AND WHY NOT THE VENDORED ONE
 *
 * This uses ESP-IDF's own esp-mqtt (mqtt_client.h), which arduino-esp32 3.x
 * ships precompiled. The repository already carries AsyncMqttClient-esphome
 * under lib/, and platformio.ini goes out of its way to `lib_ignore` it along
 * with AsyncTCP: those predate arduino-esp32 3.x and no longer compile, which
 * is the same wall the web UI hit. esp-mqtt has no such problem, runs its own
 * task with reconnect and backoff already written, and adds no dependency to
 * a build whose library resolution is already delicate.
 *
 * WHAT IT IS NOT
 *
 * It is not a bridge to any particular home automation product, and there is no
 * vendor discovery format here. What it publishes is this installation
 * described plainly: devices, the rooms they are in, where they are, and what
 * happened when something asked one to move. Anything that speaks MQTT can read
 * it; nothing has to be taught a schema belonging to a product the user does
 * not run.
 *
 * ---------------------------------------------------------------------------
 * TOPICS.  <base> is the configurable prefix, "velux" unless changed, and
 * <node> is a six-hex-digit address exactly as every other interface prints it.
 * <room> is a slug: lower case, non-alphanumerics folded to '-', so the room
 * named "Kids' room" is addressed as kids-room.
 *
 * PUBLISHED (retained, unless noted):
 *
 *   <base>/status              "online" / "offline". The offline copy is the
 *                              broker's last will, so it appears when the rig
 *                              drops off without saying goodbye. This is the
 *                              only availability signal there is -- see the
 *                              note on availability below.
 *   <base>/bridge              rig health: address, ip, uptime, heap, queue
 *                              depth, device and room counts.
 *   <base>/discovery           the whole inventory in one document: every
 *                              device with its name, room, type, whether it can
 *                              be driven, and its state if known, plus the room
 *                              list and the topic shapes above. Published on
 *                              connect, whenever the inventory changes, and
 *                              every mqttDiscoveryS seconds.
 *   <base>/devices             {"devices":[ ... ]} -- every device state in one
 *                              read, for a subscriber that wants the lot
 *                              without a wildcard subscription.
 *   <base>/rooms               {"rooms":[ ... ]} -- likewise for rooms.
 *   <base>/device/<node>/state one device.
 *   <base>/room/<room>/state   one room: the aggregate position of its members.
 *   <base>/event               NOT retained. One message per command outcome.
 *   <base>/device/<node>/event NOT retained. The same message, on the device's
 *                              own topic, so a subscriber interested in one
 *                              window does not have to filter the stream.
 *
 * Events are deliberately not retained. A retained event is a past action
 * presented to every future subscriber as news, and the action it describes --
 * a window opening -- is one where that matters.
 *
 * SUBSCRIBED:
 *
 *   <base>/device/<node>/set   one device.
 *   <base>/room/<room>/set     every device in that room.
 *   <base>/all/set             every device.
 *   <base>/set                 several devices, named in the payload.
 *   <base>/get                 republish everything from cache. Costs no radio
 *                              time; it does not go and ask the devices.
 *
 * COMMAND PAYLOADS.  JSON, or a bare word for the common case:
 *
 *   OPEN | CLOSE | STOP | VENT      as plain text, any case
 *   45                              a bare number is a position, percent OPEN
 *   {"action":"open"}
 *   {"position":45}
 *   {"action":"position","position":45,"silent":true}
 *   {"action":"status"}             go and read the real position over the air
 *
 * On <base>/set, the same fields plus the targets to apply them to:
 *
 *   {"devices":["2055CB","0EE9A6"],"action":"close"}
 *   {"rooms":["kitchen"],"position":50}
 *   {"all":true,"action":"stop"}
 *   {"commands":[{"device":"2055CB","position":0},
 *                {"room":"kitchen","action":"close"}]}
 *
 * POSITIONS are percent OPEN throughout: 100 is fully open, 0 fully closed,
 * and -1 means not known. The wire protocol counts the other way (0x0000 open,
 * 0xC800 closed) and that inversion is confined to iohc_control.cpp; nothing
 * outside it should ever see a raw value.
 *
 * ON AVAILABILITY.  There is no per-device availability topic, and that is a
 * decision rather than an omission. These actuators are low-power and say
 * nothing at all unless asked -- silence from one carries no information about
 * whether it is reachable. The rig could invent a figure from how long ago it
 * last heard something, but that would be a guess presented as a reading, and a
 * window reported unavailable because nobody polled it for an hour is worse
 * than no signal. <base>/status answers the question that can actually be
 * answered: whether the rig itself is up.
 */

#ifndef MQTT_BRIDGE_H
#define MQTT_BRIDGE_H

#include <Arduino.h>
#include <cstdint>

/// Start the bridge if the settings enable it. Call after registryBegin(),
/// controlBegin() and WiFi -- it does not need the link to be up yet, the
/// client connects when it comes.
void mqttBegin();

/// Re-read the settings: connect, disconnect or reconnect as they now say.
/// Called when the settings are written, so a broker typed into the front end
/// takes effect without a reboot.
void mqttApplySettings();

/// Publish everything from cache on the next pass, as though a subscriber had
/// asked on <base>/get.
void mqttRepublish();

enum class MqttState : uint8_t {
    Disabled,      ///< switched off, or no broker host configured
    WaitingWifi,
    Connecting,
    Connected,
    Failed,        ///< the last connection attempt was refused or dropped
};

struct MqttStatus {
    MqttState state;
    char broker[64];      ///< host:port, for display
    char base[24];        ///< topic prefix in use
    char clientId[32];
    uint32_t published;   ///< messages published since boot
    uint32_t received;    ///< command messages accepted
    uint32_t rejected;    ///< command messages that could not be understood
    uint32_t connects;    ///< successful connections, so a flapping link shows
    uint32_t connectedAgoMs;  ///< since the state last changed, 0 = never
    char lastError[64];   ///< why the last attempt failed, empty when none
};

void mqttGetStatus(MqttStatus *out);
const char *mqttStateName(MqttState state);

/// One line for the console's `net` and `mqtt` commands.
void mqttStatusPrint();

/// Bytes of stack the bridge task has never used. 0 when it is not running.
uint32_t mqttStackHeadroom();

#endif  // MQTT_BRIDGE_H
