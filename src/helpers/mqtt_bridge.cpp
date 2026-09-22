/*
   The MQTT bridge. See mqtt_bridge.h for the topic map and the payload shapes;
   this file is how they are produced and consumed.

   THE SHAPE OF IT

   Two tasks are involved and neither of them is the radio's.

   esp-mqtt owns one: it connects, reconnects, and delivers inbound messages to
   mqttEvent() below. That callback parses a command and puts it on the existing
   control queue, which is a queue send and nothing more -- no radio work
   happens on a task belonging to the network stack.

   The bridge owns the other. It polls the registry once a second, publishes
   what has changed, and drains the event queue. Polling rather than being
   called from the receive path is deliberate: the packet decoder is the task
   that must stay cheap -- an NVS write in it was already one of this
   firmware's bugs -- and the network stack is the last thing that belongs
   there. A second of latency on a window that takes twenty-five seconds to
   travel is not worth the risk.

   WHAT COUNTS AS A CHANGE

   Every device state carries fields that tick on their own: how long ago the
   node was last heard, the energy of that frame. Comparing whole payloads would
   therefore find a difference every single pass and publish six retained
   messages a second forever. So the comparison is against a signature built
   from the fields that MEAN something changed -- position, target, movement,
   error, whether a key is bound, and the name and room -- while the payload
   still carries the rest for whoever wants it.

   The front end learned the same lesson from the other end: there the data was
   stable and the rendered markup was not, so its signature is the markup. Same
   bug, opposite direction.
 */

#include <mqtt_bridge.h>

#include <ArduinoJson.h>
#include <Arduino.h>
#include <WiFi.h>
#include <app_settings.h>
#include <device_registry.h>
#include <esp_mac.h>
#include <iohc_control.h>
#include <json_out.h>
#include <keystore.h>
#include <mqtt_client.h>
#include <cctype>
#include <cstring>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
}

/// Our node address, owned by main.cpp. Reported in the bridge document so that
/// an installation with two rigs can tell which one is talking.
extern uint8_t ourAddress[3];

namespace {

constexpr uint8_t SLUG_LEN = 24;
constexpr uint16_t TOPIC_LEN = 160;

/// Big enough for the discovery document: sixteen devices at roughly 300 bytes
/// each, plus the room list and the topic map. A publish larger than this is
/// refused by esp-mqtt rather than fragmented, so buildDiscovery() checks the
/// length it produced and says so instead of failing quietly.
constexpr int OUT_BUFFER = 8192;
constexpr int IN_BUFFER = 2048;

/// Outcomes waiting to be published. Deep enough for "close the house" -- six
/// devices, queued and settled -- with room to spare.
constexpr uint8_t EVENT_QUEUE_DEPTH = 24;

/// How often the loop looks at the registry. Events are drained on the same
/// pass, so this is also the worst-case delay on an event reaching the broker.
constexpr uint32_t POLL_MS = 500;

/// The bridge document is health, not state; nothing waits on it.
constexpr uint32_t BRIDGE_EVERY_MS = 30000;

esp_mqtt_client_handle_t s_client = nullptr;
TaskHandle_t s_task = nullptr;
QueueHandle_t s_events = nullptr;

volatile MqttState s_state = MqttState::Disabled;
uint32_t s_stateAtMs = 0;
uint32_t s_published = 0;
uint32_t s_received = 0;
uint32_t s_rejected = 0;
uint32_t s_connects = 0;
char s_lastError[64] = {};

/// Set when everything must go out again regardless of the signature cache: a
/// fresh connection (the broker may have been restarted without persistence),
/// a settings change, or an explicit <base>/get.
volatile bool s_forceAll = true;

/* Set by mqttApplySettings(), acted on by the bridge task.
 *
 * It is a flag rather than a direct stopClient() because the settings are saved
 * from the web-ui task and from the console, and tearing the client down from
 * either of those would free it underneath the bridge task in the middle of a
 * publish. The client is created and destroyed on exactly one task; everybody
 * else asks. */
volatile bool s_restartWanted = false;

/* esp-mqtt keeps the pointers it is given in its config for the lifetime of the
 * client rather than copying the strings, so these have to outlive the call.
 * They are rebuilt from the settings each time the client starts. */
char s_uri[96] = {};
char s_clientId[32] = {};
char s_user[32] = {};
char s_pass[32] = {};
char s_lwtTopic[TOPIC_LEN] = {};
/// A private copy of the prefix. The live settings can change under us between
/// a subscribe and an unsubscribe, and half a rename is worse than either end
/// of it.
char s_base[24] = {};

// ------------------------------------------------------------------ utilities

void setState(const MqttState st) {
    if (s_state == st) return;
    s_state = st;
    s_stateAtMs = millis();
}

/// A room name as a topic level. Lower case, every run of non-alphanumerics
/// folded to a single '-', trimmed. "Kids' room" -> "kids-room".
///
/// This has to be lossy -- '/', '+' and '#' have meaning in a topic and a space
/// in one is a nuisance to every command-line tool -- so the display name is
/// published alongside the slug rather than being recoverable from it.
void slugify(const char *in, char *out, const size_t outLen) {
    size_t o = 0;
    bool pendingDash = false;
    for (const char *p = in; p && *p && o + 1 < outLen; ++p) {
        const auto c = static_cast<unsigned char>(*p);
        if (isalnum(c)) {
            if (pendingDash && o) out[o++] = '-';
            pendingDash = false;
            if (o + 1 < outLen) out[o++] = static_cast<char>(tolower(c));
        } else if (o) {
            pendingDash = true;
        }
    }
    out[o] = '\0';
    if (!o) snprintf(out, outLen, "%s", "unassigned");
}

uint32_t hashMix(uint32_t h, const uint32_t v) {
    h ^= v;
    return h * 16777619u;
}

uint32_t hashStr(uint32_t h, const char *s) {
    for (const char *p = s; p && *p; ++p) h = hashMix(h, static_cast<uint8_t>(*p));
    return h;
}

/// Percent open -> the word for it. "moving" without a position is the honest
/// answer for a device that has been told to move and never been read.
const char *stateWord(const RegistryDevice &d) {
    if (d.position < 0) return d.moving ? "moving" : "unknown";
    if (d.moving && d.target >= 0) {
        if (d.target > d.position) return "opening";
        if (d.target < d.position) return "closing";
    }
    if (d.position >= 99) return "open";
    if (d.position <= 1) return "closed";
    return "partial";
}

/// The CMD_ERROR_RESP reasons, worded as main.cpp words them. Repeated here
/// rather than shared because main.cpp's copy is inside a switch in the receive
/// path and lifting it out would touch the one function in this firmware that
/// most wants leaving alone.
const char *errorText(const uint8_t code) {
    switch (code) {
        case 0x00: return "";
        case 0x03: return "manually operated";
        case 0x07: return "reached wrong position";
        case 0x08: return "error during execution / opcode unsupported";
        case 0x18: return "automatic cycle engaged";
        case 0x21: return "wrong position";
        case 0x23: return "intermediate position not set";
        case 0x58: return "request rejected (malformed or unsupported index)";
        default: return "unmapped";
    }
}

// ------------------------------------------------------------------ publishing

bool publish(const char *topic, const String &payload, const bool retain) {
    if (!s_client || s_state != MqttState::Connected) return false;
    const AppSettings &cfg = settings();
    if (static_cast<int>(payload.length()) > OUT_BUFFER) {
        ets_printf("mqtt: %s is %u bytes, over the %d byte buffer -- not sent\n", topic,
                   static_cast<unsigned>(payload.length()), OUT_BUFFER);
        return false;
    }
    const int id = esp_mqtt_client_publish(s_client, topic, payload.c_str(),
                                           static_cast<int>(payload.length()), cfg.mqttQos,
                                           retain ? (cfg.mqttRetain ? 1 : 0) : 0);
    if (id < 0) return false;
    s_published++;
    return true;
}

/// Clear a retained topic. A zero-length retained message is how MQTT says a
/// topic no longer exists; without it a device that is removed from the rig
/// stays on the broker forever, and every subscriber keeps believing in a
/// window that is not there any more.
void clearRetained(const char *topic) {
    if (!s_client || s_state != MqttState::Connected) return;
    esp_mqtt_client_publish(s_client, topic, "", 0, settings().mqttQos, 1);
    s_published++;
}

void topicFor(char *out, const size_t outLen, const char *fmt, const char *a = nullptr) {
    if (a)
        snprintf(out, outLen, fmt, s_base, a);
    else
        snprintf(out, outLen, fmt, s_base);
}

// --------------------------------------------------------------- device state

void appendDeviceState(String &out, const RegistryDevice &d) {
    char group[REGISTRY_GROUP_LEN];
    char label[REGISTRY_NAME_LEN + 16];
    char slug[SLUG_LEN];
    registryGroupOf(d.node, group, sizeof(group));
    registryLabelOf(d.node, label, sizeof(label));
    slugify(group, slug, sizeof(slug));

    out += '{';
    kvNode(out, "node", d.node);
    out += ',';
    kvStr(out, "name", label);
    out += ',';
    kvStr(out, "room", group);
    out += ',';
    kvStr(out, "roomSlug", slug);
    out += ',';
    kvStr(out, "type", registryTypeName(d.type));
    out += ',';
    kvNum(out, "typeId", d.type);
    out += ',';
    // Percent OPEN, -1 when it has never been read. See the note in the header:
    // the wire scale is inverted and never leaves iohc_control.cpp.
    kvNum(out, "position", d.position);
    out += ',';
    kvNum(out, "target", d.target);
    out += ',';
    kvStr(out, "state", stateWord(d));
    out += ',';
    kvBool(out, "moving", d.moving);
    out += ',';
    // Whether a movement command can actually be authenticated. A device
    // without a key can be read but not driven, and a consumer that shows a
    // slider for one is promising something the rig cannot deliver.
    kvBool(out, "controllable", d.hasKey);
    out += ',';
    kvNum(out, "slot", d.slot);
    out += ',';
    kvNum(out, "error", d.lastError);
    out += ',';
    kvStr(out, "errorText", errorText(d.lastError));
    out += ',';
    // Diagnostics. These tick on their own and are deliberately NOT part of the
    // change signature -- see the header comment -- so they are as of the last
    // publish rather than as of now.
    kvNum(out, "rssi", d.edDbm);
    out += ',';
    kvNum(out, "lastSeenS", d.seenAgoMs ? (d.seenAgoMs + 500) / 1000 : -1);
    out += ',';
    kvNum(out, "lastReadS", d.posAgoMs ? (d.posAgoMs + 500) / 1000 : -1);
    out += '}';
}

/// Everything that means "this device changed", and nothing that does not.
uint32_t deviceSig(const RegistryDevice &d) {
    char group[REGISTRY_GROUP_LEN];
    char label[REGISTRY_NAME_LEN + 16];
    registryGroupOf(d.node, group, sizeof(group));
    registryLabelOf(d.node, label, sizeof(label));

    uint32_t h = 2166136261u;
    h = hashMix(h, static_cast<uint32_t>(d.position) & 0xFF);
    h = hashMix(h, static_cast<uint32_t>(d.target) & 0xFF);
    h = hashMix(h, d.moving ? 1u : 0u);
    h = hashMix(h, d.lastError);
    h = hashMix(h, d.hasKey ? 1u : 0u);
    h = hashMix(h, static_cast<uint32_t>(d.slot) & 0xFF);
    h = hashMix(h, d.type);
    h = hashStr(h, label);
    h = hashStr(h, group);
    return h;
}

/// Only the parts that describe what a device IS, for deciding whether the
/// discovery document needs reissuing. A window moving is not a change to the
/// inventory, and republishing six kilobytes every time one does would be.
uint32_t inventorySig(const RegistryDevice &d) {
    char group[REGISTRY_GROUP_LEN];
    char label[REGISTRY_NAME_LEN + 16];
    registryGroupOf(d.node, group, sizeof(group));
    registryLabelOf(d.node, label, sizeof(label));
    uint32_t h = 2166136261u;
    for (uint8_t i = 0; i < 3; i++) h = hashMix(h, d.node[i]);
    h = hashMix(h, d.type);
    h = hashMix(h, d.hasKey ? 1u : 0u);
    h = hashStr(h, label);
    h = hashStr(h, group);
    return h;
}

// ----------------------------------------------------------------- room state

struct RoomAgg {
    char name[REGISTRY_GROUP_LEN];
    char slug[SLUG_LEN];
    uint8_t count;
    uint8_t known;   ///< members whose position is known
    int16_t sumPos;  ///< of the known ones
    bool moving;
    bool controllable;  ///< every member has a key
    uint8_t node[REGISTRY_MAX_DEVICES][3];
    uint32_t sig;
};

/* File scope rather than on the stack: at roughly 1.4 KB this is a significant
 * fraction of the bridge task's stack, and the bridge task is the only thing
 * that ever touches it. */
RoomAgg s_roomAgg[REGISTRY_MAX_DEVICES];
uint8_t s_roomCount = 0;

/// Rebuild the room table from the registry. Rooms are derived, not stored --
/// a device with no room of its own reports the name of the key slot it is
/// bound to, so naming a slot after a room groups its devices immediately.
void buildRooms() {
    s_roomCount = 0;
    const uint8_t count = registryCount();
    for (uint8_t i = 0; i < count; i++) {
        RegistryDevice d{};
        if (!registryGet(i, &d)) continue;

        char group[REGISTRY_GROUP_LEN];
        registryGroupOf(d.node, group, sizeof(group));

        RoomAgg *room = nullptr;
        for (uint8_t r = 0; r < s_roomCount; r++)
            if (strcmp(s_roomAgg[r].name, group) == 0) {
                room = &s_roomAgg[r];
                break;
            }
        if (!room) {
            if (s_roomCount >= REGISTRY_MAX_DEVICES) continue;
            room = &s_roomAgg[s_roomCount++];
            memset(room, 0, sizeof(*room));
            snprintf(room->name, sizeof(room->name), "%s", group);
            slugify(group, room->slug, sizeof(room->slug));
            room->controllable = true;
            room->sig = 2166136261u;
        }
        if (room->count < REGISTRY_MAX_DEVICES) memcpy(room->node[room->count], d.node, 3);
        room->count++;
        if (d.position >= 0) {
            room->known++;
            room->sumPos = static_cast<int16_t>(room->sumPos + d.position);
        }
        if (d.moving) room->moving = true;
        if (!d.hasKey) room->controllable = false;
        room->sig = hashMix(room->sig, deviceSig(d));
    }
}

/// The room's position: the mean of the members that have one. A room whose
/// blinds are at 0 and 100 reports 50, which is not a position any of them is
/// in -- so the member count and how many are known are published with it,
/// rather than presenting an average as a reading.
int16_t roomPosition(const RoomAgg &r) {
    if (!r.known) return -1;
    return static_cast<int16_t>((r.sumPos + r.known / 2) / r.known);
}

void appendRoomState(String &out, const RoomAgg &r) {
    const int16_t pos = roomPosition(r);
    out += '{';
    kvStr(out, "room", r.name);
    out += ',';
    kvStr(out, "slug", r.slug);
    out += ',';
    kvNum(out, "devices", r.count);
    out += ',';
    kvNum(out, "known", r.known);
    out += ',';
    kvNum(out, "position", pos);
    out += ',';
    kvStr(out, "state", pos < 0    ? (r.moving ? "moving" : "unknown")
                        : r.moving ? "moving"
                        : pos >= 99 ? "open"
                        : pos <= 1  ? "closed"
                                    : "partial");
    out += ',';
    kvBool(out, "moving", r.moving);
    out += ',';
    kvBool(out, "controllable", r.controllable);
    out += ",\"members\":[";
    const uint8_t n = r.count < REGISTRY_MAX_DEVICES ? r.count : REGISTRY_MAX_DEVICES;
    for (uint8_t i = 0; i < n; i++) {
        if (i) out += ',';
        jsonNode(out, r.node[i]);
    }
    out += "]}";
}

// ------------------------------------------------------------------ documents

void buildBridge(String &out) {
    out.reserve(400);
    out += '{';
    kvNode(out, "node", ourAddress);
    out += ',';
    kvStr(out, "ip", WiFi.localIP().toString().c_str());
    out += ',';
    kvStr(out, "ssid", WiFi.SSID().c_str());
    out += ',';
    kvNum(out, "rssi", WiFi.RSSI());
    out += ',';
    kvNum(out, "uptimeS", millis() / 1000);
    out += ',';
    kvNum(out, "heap", ESP.getFreeHeap());
    out += ',';
    kvNum(out, "devices", registryCount());
    out += ',';
    kvNum(out, "rooms", s_roomCount);
    out += ',';
    kvNum(out, "keys", keystoreCount());
    out += ',';
    // A tap that will not be acted on for half a minute should not look
    // ignored, so the depth of the radio queue is part of the health document.
    kvNum(out, "queued", controlPending());
    out += ',';
    kvBool(out, "busy", controlBusy());
    out += ',';
    kvNum(out, "published", s_published);
    out += ',';
    kvNum(out, "received", s_received);
    out += '}';
}

/// The inventory: what exists, what it is called, where it is, and what it was
/// last known to be doing. Published on connect, on an inventory change, and on
/// a timer -- so a subscriber that arrives at any moment can describe the whole
/// installation from one retained message without having to know the topic
/// scheme in advance, which is why the topic shapes are in here too.
void buildDiscovery(String &out) {
    out.reserve(7000);
    out += "{\"schema\":1,\"bridge\":";
    String bridge;
    buildBridge(bridge);
    out += bridge;

    out += ",\"topics\":{";
    char t[TOPIC_LEN];
    topicFor(t, sizeof(t), "%s/status");
    kvStr(out, "status", t);
    out += ',';
    topicFor(t, sizeof(t), "%s/discovery");
    kvStr(out, "discovery", t);
    out += ',';
    topicFor(t, sizeof(t), "%s/devices");
    kvStr(out, "devices", t);
    out += ',';
    topicFor(t, sizeof(t), "%s/rooms");
    kvStr(out, "rooms", t);
    out += ',';
    topicFor(t, sizeof(t), "%s/event");
    kvStr(out, "event", t);
    out += ',';
    topicFor(t, sizeof(t), "%s/set");
    kvStr(out, "set", t);
    out += ',';
    topicFor(t, sizeof(t), "%s/get");
    kvStr(out, "get", t);
    out += ',';
    topicFor(t, sizeof(t), "%s/all/set");
    kvStr(out, "allSet", t);
    out += ',';
    topicFor(t, sizeof(t), "%s/device/<node>/state");
    kvStr(out, "deviceState", t);
    out += ',';
    topicFor(t, sizeof(t), "%s/device/<node>/set");
    kvStr(out, "deviceSet", t);
    out += ',';
    topicFor(t, sizeof(t), "%s/device/<node>/event");
    kvStr(out, "deviceEvent", t);
    out += ',';
    topicFor(t, sizeof(t), "%s/room/<room>/state");
    kvStr(out, "roomState", t);
    out += ',';
    topicFor(t, sizeof(t), "%s/room/<room>/set");
    kvStr(out, "roomSet", t);
    out += '}';

    out += ",\"actions\":[\"open\",\"close\",\"stop\",\"vent\",\"position\",\"status\"]";
    // Said explicitly so nothing has to infer it from the numbers, and because
    // it is the opposite of the wire protocol underneath.
    out += ",\"positionScale\":\"percent open: 100 fully open, 0 fully closed, -1 unknown\"";

    out += ",\"rooms\":[";
    for (uint8_t r = 0; r < s_roomCount; r++) {
        if (r) out += ',';
        appendRoomState(out, s_roomAgg[r]);
    }
    out += "],\"devices\":[";
    bool first = true;
    const uint8_t count = registryCount();
    for (uint8_t i = 0; i < count; i++) {
        RegistryDevice d{};
        if (!registryGet(i, &d)) continue;
        if (!first) out += ',';
        first = false;
        appendDeviceState(out, d);
    }
    out += "]}";
}

// --------------------------------------------------------------------- events

void appendEvent(String &out, const IohcResult &r) {
    char label[REGISTRY_NAME_LEN + 16];
    char group[REGISTRY_GROUP_LEN];
    registryLabelOf(r.node, label, sizeof(label));
    registryGroupOf(r.node, group, sizeof(group));

    /* Two words, because they answer different questions. `result` is the one
     * an automation branches on; `phase` is the one a person reads when they
     * want to know why. A queued command is neither a success nor a failure
     * yet, hence "pending" -- reporting it as either would be a lie for however
     * many seconds of radio work are ahead of it. */
    const char *result;
    switch (r.outcome) {
        case IohcOutcome::Queued: result = "pending"; break;
        case IohcOutcome::Moving: result = "pending"; break;
        case IohcOutcome::Acked: result = "ok"; break;
        case IohcOutcome::Settled: result = r.reachedTarget ? "ok" : "failed"; break;
        default: result = "failed"; break;
    }

    const char *detail;
    switch (r.outcome) {
        case IohcOutcome::Queued: detail = "accepted, waiting for the radio"; break;
        case IohcOutcome::Acked:
            detail = r.challenged ? "the device answered the challenge" : "the device answered";
            break;
        case IohcOutcome::NoResponse:
            detail = "nothing answered on any channel tried";
            break;
        case IohcOutcome::Rejected: detail = errorText(r.errorCode); break;
        case IohcOutcome::NoKey:
            detail = "no key is bound to this device, so the challenge cannot be answered";
            break;
        case IohcOutcome::Moving: detail = "on its way"; break;
        case IohcOutcome::Settled:
            detail = r.reachedTarget ? "reached the requested position"
                                     : "stopped without reaching the requested position";
            break;
        default: detail = ""; break;
    }

    out.reserve(420);
    out += '{';
    kvNode(out, "node", r.node);
    out += ',';
    kvStr(out, "name", label);
    out += ',';
    kvStr(out, "room", group);
    out += ',';
    kvStr(out, "action", controlActionName(r.action));
    out += ',';
    kvStr(out, "result", result);
    out += ',';
    kvStr(out, "phase", controlOutcomeName(r.outcome));
    out += ',';
    kvStr(out, "detail", detail);
    out += ',';
    kvStr(out, "source", controlSourceName(r.source));
    out += ',';
    if (r.action == IohcAction::Position) {
        kvNum(out, "requested", r.percent);
        out += ',';
    }
    kvNum(out, "position", r.position);
    out += ',';
    kvNum(out, "target", r.target);
    out += ',';
    kvNum(out, "channel", r.channel);
    out += ',';
    kvBool(out, "authenticated", r.challenged);
    out += ',';
    kvNum(out, "error", r.errorCode);
    out += ',';
    kvNum(out, "uptimeS", millis() / 1000);
    out += '}';
}

/// Registered with iohc_control. Runs on the command worker -- the task that
/// keys the radio, above the packet decoder in priority -- so it does the least
/// it possibly can: copy the result and return. Publishing happens on the
/// bridge task, where blocking on the network costs nothing.
void onResult(const IohcResult &r) {
    // Nothing drains this queue when the bridge is switched off, so do not fill
    // it: the handler stays registered either way, because the bridge can be
    // enabled from the settings page at any time and re-registering it from
    // there would be a second place that has to remember to.
    if (!s_events || !s_task) return;
    xQueueSend(s_events, &r, 0);
}

void drainEvents() {
    IohcResult r{};
    char topic[TOPIC_LEN];
    char node[8];
    while (s_events && xQueueReceive(s_events, &r, 0) == pdTRUE) {
        String payload;
        appendEvent(payload, r);
        topicFor(topic, sizeof(topic), "%s/event");
        publish(topic, payload, false);
        snprintf(node, sizeof(node), "%02X%02X%02X", r.node[0], r.node[1], r.node[2]);
        snprintf(topic, sizeof(topic), "%s/device/%s/event", s_base, node);
        publish(topic, payload, false);

        // A command that finished is a change to the device, and the poll below
        // would find it -- but up to half a second later. Push the state out
        // with the event so the two arrive together.
        if (r.outcome != IohcOutcome::Queued) s_forceAll = true;
    }
}

// ------------------------------------------------------------ inbound commands

struct ParsedCommand {
    bool haveAction;
    IohcAction action;
    uint8_t percent;
    bool silent;
    uint8_t channel;
};

/// A command with nothing filled in yet, carrying the settings' defaults. Used
/// everywhere a ParsedCommand is started, so that a payload which says nothing
/// about travel speed gets the configured one rather than a hard-coded false.
ParsedCommand freshCommand() {
    ParsedCommand pc{};
    pc.silent = settings().silentDefault;
    return pc;
}

/// A bare word or number, which is what anyone reaching for mosquitto_pub will
/// send first. Accepted alongside JSON because a command line that needs a JSON
/// document to close a blind is a command line nobody will use to debug this.
bool parsePlain(const char *text, ParsedCommand *out) {
    char word[24] = {};
    size_t w = 0;
    for (const char *p = text; *p && w + 1 < sizeof(word); ++p) {
        if (isspace(static_cast<unsigned char>(*p))) continue;
        word[w++] = static_cast<char>(tolower(static_cast<unsigned char>(*p)));
    }
    if (!w) return false;

    if (isdigit(static_cast<unsigned char>(word[0]))) {
        const long pct = strtol(word, nullptr, 10);
        if (pct < 0 || pct > 100) return false;
        out->action = IohcAction::Position;
        out->percent = static_cast<uint8_t>(pct);
        out->haveAction = true;
        return true;
    }
    if (controlParseAction(word, &out->action)) {
        out->haveAction = true;
        return true;
    }
    return false;
}

/// The action fields of a command object, which are the same wherever they
/// appear: on a device topic, a room topic, or inside a `commands` entry.
/// @p out arrives carrying any inherited values, so an entry can override just
/// the position and keep the action from the document around it.
bool parseFields(const JsonObjectConst obj, ParsedCommand *out) {
    if (const JsonVariantConst a = obj["action"]; !a.isNull()) {
        char word[24] = {};
        const char *s = a.as<const char *>();
        if (!s) return false;
        size_t w = 0;
        for (const char *p = s; *p && w + 1 < sizeof(word); ++p)
            word[w++] = static_cast<char>(tolower(static_cast<unsigned char>(*p)));
        if (!controlParseAction(word, &out->action)) return false;
        out->haveAction = true;
    }
    if (const JsonVariantConst p = obj["position"]; !p.isNull()) {
        const long pct = p.as<long>();
        if (pct < 0 || pct > 100) return false;
        out->percent = static_cast<uint8_t>(pct);
        // A position without an action is a request to go there; with an
        // explicit action it is that action's parameter.
        if (!out->haveAction) {
            out->action = IohcAction::Position;
            out->haveAction = true;
        }
    }
    if (const JsonVariantConst s = obj["silent"]; !s.isNull()) out->silent = s.as<bool>();
    if (const JsonVariantConst c = obj["channel"]; !c.isNull())
        out->channel = static_cast<uint8_t>(c.as<long>());
    return out->haveAction;
}

bool enqueue(const uint8_t node[3], const ParsedCommand &pc) {
    IohcCommand cmd{};
    memcpy(cmd.node, node, 3);
    cmd.action = pc.action;
    cmd.percent = pc.percent;
    cmd.silent = pc.silent;
    cmd.channel = pc.channel ? pc.channel : settings().commandChannel;
    cmd.source = IohcSource::Mqtt;
    return controlEnqueue(cmd);
}

uint8_t applyToDevice(const char *nodeHex, const ParsedCommand &pc) {
    // A JSON array can hold anything, so an element that is not a string
    // arrives here as null rather than as text.
    if (!nodeHex || !*nodeHex) return 0;
    uint8_t node[3];
    if (!keystoreParseNode(nodeHex, node)) return 0;
    // Only devices the rig knows about. An address typed into a topic by
    // mistake would otherwise put the radio to work on a node that does not
    // exist, for the four seconds it takes to find that out.
    RegistryDevice d{};
    if (!registryGetByNode(node, &d)) {
        ets_printf("mqtt: %s is not a known device -- ignored\n", nodeHex);
        return 0;
    }
    return enqueue(node, pc) ? 1 : 0;
}

/// Match by slug, so `velux/room/Kitchen/set` and `velux/room/kitchen/set` both
/// reach the kitchen -- the reference is slugified before comparing, which
/// folds the case difference away along with everything else.
///
/// This walks the registry rather than the room table above, deliberately.
/// s_roomAgg is rebuilt by the bridge task on every pass, and this runs on
/// esp-mqtt's task: reading it here would be two tasks on a 1.4 KB structure
/// with no lock between them, to save a scan of at most sixteen rows. The
/// registry has a lock of its own and is the authority anyway.
uint8_t applyToRoom(const char *roomRef, const ParsedCommand &pc) {
    if (!roomRef || !*roomRef) return 0;
    char want[SLUG_LEN];
    slugify(roomRef, want, sizeof(want));

    uint8_t sent = 0;
    const uint8_t count = registryCount();
    for (uint8_t i = 0; i < count; i++) {
        RegistryDevice d{};
        if (!registryGet(i, &d)) continue;
        char group[REGISTRY_GROUP_LEN];
        char slug[SLUG_LEN];
        registryGroupOf(d.node, group, sizeof(group));
        slugify(group, slug, sizeof(slug));
        if (strcmp(slug, want) != 0) continue;
        if (enqueue(d.node, pc)) sent++;
    }
    if (!sent) ets_printf("mqtt: no room matched '%s'\n", roomRef);
    return sent;
}

uint8_t applyToAll(const ParsedCommand &pc) {
    uint8_t sent = 0;
    const uint8_t count = registryCount();
    for (uint8_t i = 0; i < count; i++) {
        RegistryDevice d{};
        if (!registryGet(i, &d)) continue;
        if (enqueue(d.node, pc)) sent++;
    }
    return sent;
}

/// One entry of a multi-target document: its own target, its own fields, with
/// anything it does not say inherited from the document around it.
uint8_t applyEntry(const JsonObjectConst obj, const ParsedCommand &inherited) {
    ParsedCommand pc = inherited;
    if (!parseFields(obj, &pc)) return 0;

    if (const JsonVariantConst v = obj["device"]; !v.isNull())
        return applyToDevice(v.as<const char *>(), pc);
    if (const JsonVariantConst v = obj["node"]; !v.isNull())
        return applyToDevice(v.as<const char *>(), pc);
    if (const JsonVariantConst v = obj["room"]; !v.isNull())
        return applyToRoom(v.as<const char *>(), pc);
    if (const JsonVariantConst v = obj["all"]; v.as<bool>()) return applyToAll(pc);
    return 0;
}

/// <base>/set: several devices, several rooms, or an explicit list of commands,
/// in one message. This is the topic that answers "set multiple devices in a
/// single command" -- one publish, one radio queue, in the order given.
uint8_t applyTargeted(const JsonObjectConst root) {
    ParsedCommand pc = freshCommand();
    // Not an error yet: a document that only carries `commands` has its fields
    // on the entries rather than at the top.
    parseFields(root, &pc);

    uint8_t sent = 0;
    if (const JsonArrayConst a = root["devices"]; !a.isNull())
        for (const JsonVariantConst v : a) sent += applyToDevice(v.as<const char *>(), pc);
    if (const JsonArrayConst a = root["rooms"]; !a.isNull())
        for (const JsonVariantConst v : a) sent += applyToRoom(v.as<const char *>(), pc);
    if (const JsonVariantConst v = root["all"]; v.as<bool>()) sent += applyToAll(pc);
    if (const JsonArrayConst a = root["commands"]; !a.isNull())
        for (const JsonVariantConst v : a)
            if (v.is<JsonObjectConst>()) sent += applyEntry(v.as<JsonObjectConst>(), pc);
    return sent;
}

/// Turn a payload into a ParsedCommand, accepting either form.
bool parsePayload(const char *payload, ParsedCommand *pc) {
    if (!payload || !*payload) return false;
    // Cheap discrimination: a JSON object starts with '{' after any space, and
    // nothing in the plain vocabulary does.
    const char *p = payload;
    while (*p && isspace(static_cast<unsigned char>(*p))) p++;
    if (*p != '{') return parsePlain(payload, pc);

    JsonDocument doc;
    if (deserializeJson(doc, payload) != DeserializationError::Ok) return false;
    return parseFields(doc.as<JsonObjectConst>(), pc);
}

void handleMessage(const char *topic, const char *payload) {
    const size_t baseLen = strlen(s_base);
    if (strncmp(topic, s_base, baseLen) != 0 || topic[baseLen] != '/') return;
    const char *rest = topic + baseLen + 1;

    if (strcmp(rest, "get") == 0) {
        s_forceAll = true;
        s_received++;
        return;
    }

    if (strcmp(rest, "set") == 0) {
        JsonDocument doc;
        if (deserializeJson(doc, payload) != DeserializationError::Ok ||
            !doc.is<JsonObjectConst>()) {
            ets_printf("mqtt: %s needs a JSON object naming what to act on\n", topic);
            s_rejected++;
            return;
        }
        const uint8_t sent = applyTargeted(doc.as<JsonObjectConst>());
        if (sent) {
            s_received++;
            ets_printf("mqtt: %s -> %u command(s)\n", topic, sent);
        } else {
            s_rejected++;
            ets_printf("mqtt: %s named nothing this rig could act on\n", topic);
        }
        return;
    }

    ParsedCommand pc = freshCommand();
    uint8_t sent = 0;

    if (strcmp(rest, "all/set") == 0) {
        if (!parsePayload(payload, &pc)) {
            s_rejected++;
            ets_printf("mqtt: could not read '%s' as a command\n", payload);
            return;
        }
        sent = applyToAll(pc);
    } else if (strncmp(rest, "device/", 7) == 0) {
        const char *id = rest + 7;
        const char *slash = strchr(id, '/');
        if (!slash || strcmp(slash, "/set") != 0) return;
        char nodeHex[16] = {};
        const size_t n = static_cast<size_t>(slash - id);
        if (n >= sizeof(nodeHex)) return;
        memcpy(nodeHex, id, n);
        if (!parsePayload(payload, &pc)) {
            s_rejected++;
            ets_printf("mqtt: could not read '%s' as a command\n", payload);
            return;
        }
        sent = applyToDevice(nodeHex, pc);
    } else if (strncmp(rest, "room/", 5) == 0) {
        const char *id = rest + 5;
        const char *slash = strchr(id, '/');
        if (!slash || strcmp(slash, "/set") != 0) return;
        char room[SLUG_LEN * 2] = {};
        const size_t n = static_cast<size_t>(slash - id);
        if (n >= sizeof(room)) return;
        memcpy(room, id, n);
        if (!parsePayload(payload, &pc)) {
            s_rejected++;
            ets_printf("mqtt: could not read '%s' as a command\n", payload);
            return;
        }
        sent = applyToRoom(room, pc);
    } else {
        return;  // not a topic we subscribed to; nothing to say about it
    }

    if (sent) {
        s_received++;
        ets_printf("mqtt: %s %s -> %u command(s)\n", topic, controlActionName(pc.action), sent);
    } else {
        s_rejected++;
    }
}

// ----------------------------------------------------------------- the client

void subscribeAll() {
    char t[TOPIC_LEN];
    const AppSettings &cfg = settings();
    const int qos = cfg.mqttQos;
    snprintf(t, sizeof(t), "%s/device/+/set", s_base);
    esp_mqtt_client_subscribe(s_client, t, qos);
    snprintf(t, sizeof(t), "%s/room/+/set", s_base);
    esp_mqtt_client_subscribe(s_client, t, qos);
    snprintf(t, sizeof(t), "%s/all/set", s_base);
    esp_mqtt_client_subscribe(s_client, t, qos);
    snprintf(t, sizeof(t), "%s/set", s_base);
    esp_mqtt_client_subscribe(s_client, t, qos);
    snprintf(t, sizeof(t), "%s/get", s_base);
    esp_mqtt_client_subscribe(s_client, t, qos);
}

void mqttEvent(void *, esp_event_base_t, const int32_t id, void *data) {
    const auto *e = static_cast<esp_mqtt_event_handle_t>(data);

    switch (static_cast<esp_mqtt_event_id_t>(id)) {
        case MQTT_EVENT_CONNECTED: {
            setState(MqttState::Connected);
            s_connects++;
            s_lastError[0] = '\0';
            ets_printf("mqtt: connected to %s as %s, prefix %s\n", s_uri, s_clientId, s_base);
            char t[TOPIC_LEN];
            topicFor(t, sizeof(t), "%s/status");
            esp_mqtt_client_publish(s_client, t, "online", 0, settings().mqttQos, 1);
            s_published++;
            subscribeAll();
            /* Everything again, from scratch. The broker may have restarted
             * without persistence, or another client may have cleared a
             * retained topic; either way our idea of what it already holds is
             * only trustworthy for the life of a connection. */
            s_forceAll = true;
            break;
        }
        case MQTT_EVENT_DISCONNECTED:
            // Not an error on its own: esp-mqtt reconnects on its own schedule
            // and this fires on every retry while a broker is down.
            if (s_state == MqttState::Connected) ets_printf("mqtt: disconnected\n");
            setState(MqttState::Connecting);
            break;

        case MQTT_EVENT_DATA: {
            /* esp-mqtt splits a payload larger than the input buffer across
             * several events. Commands are tens of bytes, so a fragmented one
             * is a malformed message rather than a big one -- say so instead of
             * acting on the first fragment, which would be a command with its
             * tail missing. */
            if (e->data_len != e->total_data_len) {
                ets_printf("mqtt: %d byte payload exceeds the %d byte buffer -- ignored\n",
                           e->total_data_len, IN_BUFFER);
                s_rejected++;
                break;
            }
            // Neither topic nor data arrives null-terminated.
            char topic[TOPIC_LEN];
            const size_t tn = static_cast<size_t>(e->topic_len) < sizeof(topic) - 1
                                  ? static_cast<size_t>(e->topic_len)
                                  : sizeof(topic) - 1;
            memcpy(topic, e->topic, tn);
            topic[tn] = '\0';

            char payload[512];
            const size_t pn = static_cast<size_t>(e->data_len) < sizeof(payload) - 1
                                  ? static_cast<size_t>(e->data_len)
                                  : sizeof(payload) - 1;
            memcpy(payload, e->data, pn);
            payload[pn] = '\0';

            handleMessage(topic, payload);
            break;
        }

        case MQTT_EVENT_ERROR: {
            setState(MqttState::Failed);
            if (e->error_handle &&
                e->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                snprintf(s_lastError, sizeof(s_lastError), "broker refused the connection (0x%02X)",
                         e->error_handle->connect_return_code);
            } else if (e->error_handle) {
                snprintf(s_lastError, sizeof(s_lastError), "transport error %d (errno %d)",
                         e->error_handle->esp_tls_last_esp_err, e->error_handle->esp_transport_sock_errno);
            }
            ets_printf("mqtt: %s\n", s_lastError);
            break;
        }
        default: break;
    }
}

void stopClient() {
    if (!s_client) return;
    if (s_state == MqttState::Connected) {
        // Say goodbye properly. The last will covers the case where we cannot,
        // but a clean stop should not leave subscribers waiting for a keepalive
        // to expire before they learn the rig went away on purpose.
        char t[TOPIC_LEN];
        topicFor(t, sizeof(t), "%s/status");
        esp_mqtt_client_publish(s_client, t, "offline", 0, 1, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    esp_mqtt_client_stop(s_client);
    esp_mqtt_client_destroy(s_client);
    s_client = nullptr;
    setState(MqttState::Disabled);
}

bool startClient() {
    const AppSettings &cfg = settings();
    if (!cfg.mqttEnabled || !cfg.mqttHost[0]) return false;

    snprintf(s_base, sizeof(s_base), "%s", cfg.mqttBase[0] ? cfg.mqttBase : "velux");
    snprintf(s_uri, sizeof(s_uri), "mqtt://%s:%u", cfg.mqttHost,
             cfg.mqttPort ? cfg.mqttPort : 1883);
    snprintf(s_user, sizeof(s_user), "%s", cfg.mqttUser);
    snprintf(s_pass, sizeof(s_pass), "%s", cfg.mqttPass);
    snprintf(s_lwtTopic, sizeof(s_lwtTopic), "%s/status", s_base);

    if (cfg.mqttClientId[0]) {
        snprintf(s_clientId, sizeof(s_clientId), "%s", cfg.mqttClientId);
    } else {
        // Derived from the MAC rather than fixed: two rigs sharing a broker with
        // one client id disconnect each other in a loop, and the symptom looks
        // like a flapping network rather than a name collision.
        uint8_t mac[6] = {};
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        snprintf(s_clientId, sizeof(s_clientId), "velux-%02X%02X%02X", mac[3], mac[4], mac[5]);
    }

    esp_mqtt_client_config_t c = {};
    c.broker.address.uri = s_uri;
    c.credentials.client_id = s_clientId;
    if (s_user[0]) c.credentials.username = s_user;
    if (s_pass[0]) c.credentials.authentication.password = s_pass;
    c.session.last_will.topic = s_lwtTopic;
    c.session.last_will.msg = "offline";
    c.session.last_will.msg_len = 7;
    c.session.last_will.qos = 1;
    c.session.last_will.retain = 1;
    c.session.keepalive = 60;
    c.network.reconnect_timeout_ms = 10000;
    c.network.timeout_ms = 10000;
    // Below the packet decoder (5) and well below the command worker (6). The
    // bridge is never the thing that must run next: the radio is.
    c.task.priority = 4;
    c.task.stack_size = 6144;
    c.buffer.size = IN_BUFFER;
    c.buffer.out_size = OUT_BUFFER;

    s_client = esp_mqtt_client_init(&c);
    if (!s_client) {
        snprintf(s_lastError, sizeof(s_lastError), "client could not be created");
        setState(MqttState::Failed);
        return false;
    }
    esp_mqtt_client_register_event(s_client, MQTT_EVENT_ANY, mqttEvent, nullptr);
    if (esp_mqtt_client_start(s_client) != ESP_OK) {
        snprintf(s_lastError, sizeof(s_lastError), "client could not be started");
        setState(MqttState::Failed);
        esp_mqtt_client_destroy(s_client);
        s_client = nullptr;
        return false;
    }
    setState(MqttState::Connecting);
    ets_printf("mqtt: connecting to %s\n", s_uri);
    return true;
}

// ------------------------------------------------------------------ the loop

struct PubDevice {
    uint8_t node[3];
    bool used;
    uint32_t sig;
};
PubDevice s_pubDev[REGISTRY_MAX_DEVICES];

struct PubRoom {
    char slug[SLUG_LEN];
    bool used;
    uint32_t sig;
};
PubRoom s_pubRoom[REGISTRY_MAX_DEVICES];

uint32_t s_inventorySig = 0;
uint32_t s_lastDiscoveryMs = 0;
uint32_t s_lastBridgeMs = 0;

void forgetPublished() {
    memset(s_pubDev, 0, sizeof(s_pubDev));
    memset(s_pubRoom, 0, sizeof(s_pubRoom));
    s_inventorySig = 0;
    s_lastDiscoveryMs = 0;
    s_lastBridgeMs = 0;
}

void publishPass() {
    const bool force = s_forceAll;
    s_forceAll = false;

    buildRooms();

    char topic[TOPIC_LEN];
    char node[8];

    // ---- devices, and the aggregate ----
    bool anyDeviceChanged = false;
    uint32_t inventory = 2166136261u;
    String all;
    all.reserve(6000);  // sixteen devices at roughly 350 bytes, without regrowing
    all += "{\"devices\":[";

    bool seen[REGISTRY_MAX_DEVICES] = {};
    bool first = true;
    const uint8_t count = registryCount();
    for (uint8_t i = 0; i < count; i++) {
        RegistryDevice d{};
        if (!registryGet(i, &d)) continue;

        if (!first) all += ',';
        first = false;
        appendDeviceState(all, d);
        inventory = hashMix(inventory, inventorySig(d));

        const uint32_t sig = deviceSig(d);
        PubDevice *slot = nullptr;
        for (uint8_t p = 0; p < REGISTRY_MAX_DEVICES; p++) {
            if (!s_pubDev[p].used || memcmp(s_pubDev[p].node, d.node, 3) != 0) continue;
            slot = &s_pubDev[p];
            seen[p] = true;
            break;
        }
        if (!slot)
            for (uint8_t p = 0; p < REGISTRY_MAX_DEVICES; p++) {
                if (s_pubDev[p].used) continue;
                slot = &s_pubDev[p];
                memcpy(slot->node, d.node, 3);
                slot->used = true;
                slot->sig = sig + 1;  // anything but sig, so the first pass publishes
                seen[p] = true;
                break;
            }
        if (!slot) continue;
        if (!force && slot->sig == sig) continue;

        String payload;
        payload.reserve(500);
        appendDeviceState(payload, d);
        snprintf(node, sizeof(node), "%02X%02X%02X", d.node[0], d.node[1], d.node[2]);
        snprintf(topic, sizeof(topic), "%s/device/%s/state", s_base, node);
        if (publish(topic, payload, true)) {
            slot->sig = sig;
            anyDeviceChanged = true;
        }
    }
    all += "]}";

    // A device that has been removed from the rig leaves a retained topic that
    // would otherwise outlive it on the broker.
    for (uint8_t p = 0; p < REGISTRY_MAX_DEVICES; p++) {
        if (!s_pubDev[p].used || seen[p]) continue;
        snprintf(node, sizeof(node), "%02X%02X%02X", s_pubDev[p].node[0], s_pubDev[p].node[1],
                 s_pubDev[p].node[2]);
        snprintf(topic, sizeof(topic), "%s/device/%s/state", s_base, node);
        clearRetained(topic);
        s_pubDev[p].used = false;
        anyDeviceChanged = true;
    }

    if (anyDeviceChanged || force) {
        topicFor(topic, sizeof(topic), "%s/devices");
        publish(topic, all, true);
    }

    // ---- rooms, and their aggregate ----
    bool anyRoomChanged = false;
    String rooms;
    rooms.reserve(2000);
    rooms += "{\"rooms\":[";
    bool roomSeen[REGISTRY_MAX_DEVICES] = {};
    for (uint8_t r = 0; r < s_roomCount; r++) {
        if (r) rooms += ',';
        appendRoomState(rooms, s_roomAgg[r]);

        PubRoom *slot = nullptr;
        for (uint8_t p = 0; p < REGISTRY_MAX_DEVICES; p++) {
            if (!s_pubRoom[p].used || strcmp(s_pubRoom[p].slug, s_roomAgg[r].slug) != 0) continue;
            slot = &s_pubRoom[p];
            roomSeen[p] = true;
            break;
        }
        if (!slot)
            for (uint8_t p = 0; p < REGISTRY_MAX_DEVICES; p++) {
                if (s_pubRoom[p].used) continue;
                slot = &s_pubRoom[p];
                snprintf(slot->slug, sizeof(slot->slug), "%s", s_roomAgg[r].slug);
                slot->used = true;
                slot->sig = s_roomAgg[r].sig + 1;
                roomSeen[p] = true;
                break;
            }
        if (!slot) continue;
        if (!force && slot->sig == s_roomAgg[r].sig) continue;

        String payload;
        payload.reserve(400);
        appendRoomState(payload, s_roomAgg[r]);
        snprintf(topic, sizeof(topic), "%s/room/%s/state", s_base, s_roomAgg[r].slug);
        if (publish(topic, payload, true)) {
            slot->sig = s_roomAgg[r].sig;
            anyRoomChanged = true;
        }
    }
    rooms += "]}";

    for (uint8_t p = 0; p < REGISTRY_MAX_DEVICES; p++) {
        if (!s_pubRoom[p].used || roomSeen[p]) continue;
        snprintf(topic, sizeof(topic), "%s/room/%s/state", s_base, s_pubRoom[p].slug);
        clearRetained(topic);
        s_pubRoom[p].used = false;
        anyRoomChanged = true;
    }

    if (anyRoomChanged || force) {
        topicFor(topic, sizeof(topic), "%s/rooms");
        publish(topic, rooms, true);
    }

    // ---- discovery ----
    const uint32_t now = millis();
    const uint32_t every = static_cast<uint32_t>(settings().mqttDiscoveryS) * 1000UL;
    const bool inventoryChanged = inventory != s_inventorySig;
    const bool due = !s_lastDiscoveryMs || (every && now - s_lastDiscoveryMs >= every);
    if (force || inventoryChanged || due) {
        String doc;
        buildDiscovery(doc);
        topicFor(topic, sizeof(topic), "%s/discovery");
        if (publish(topic, doc, true)) {
            s_inventorySig = inventory;
            s_lastDiscoveryMs = now ? now : 1;
        }
    }

    // ---- bridge health ----
    if (force || !s_lastBridgeMs || now - s_lastBridgeMs >= BRIDGE_EVERY_MS) {
        String doc;
        buildBridge(doc);
        topicFor(topic, sizeof(topic), "%s/bridge");
        if (publish(topic, doc, true)) s_lastBridgeMs = now ? now : 1;
    }
}

void bridgeTask(void *) {
    for (;;) {
        const AppSettings &cfg = settings();
        const bool want = cfg.mqttEnabled && cfg.mqttHost[0];

        if (!want) {
            if (s_client) {
                ets_printf("mqtt: disabled\n");
                stopClient();
                forgetPublished();
            }
            setState(MqttState::Disabled);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // A broker, prefix or credential changed under us. Everything is
        // rebuilt from the settings on the next lap.
        if (s_restartWanted) {
            s_restartWanted = false;
            if (s_client) {
                ets_printf("mqtt: settings changed -- reconnecting\n");
                stopClient();
            }
            forgetPublished();
            s_forceAll = true;
        }

        if (WiFi.status() != WL_CONNECTED) {
            if (!s_client) setState(MqttState::WaitingWifi);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (!s_client && !startClient()) {
            // startClient() has already recorded why. Waiting here rather than
            // spinning: a broker that refuses us will refuse us again, and a
            // tight retry loop on a rig that cannot be reached with a cable is
            // a good way to make the console unusable.
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        drainEvents();
        if (s_state == MqttState::Connected) publishPass();

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

}  // namespace

// --------------------------------------------------------------------- public

void mqttBegin() {
    if (!s_events) s_events = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(IohcResult));
    if (!s_events) {
        ets_printf("mqtt: event queue could not be created\n");
        return;
    }
    controlSetResultHandler(onResult);

    const AppSettings &cfg = settings();
    if (!cfg.mqttEnabled || !cfg.mqttHost[0]) {
        // No task at all while it is switched off. It can be turned on from the
        // settings page, which calls mqttApplySettings() and starts it then --
        // an idle task costs eight kilobytes of a heap that has 180 to spend.
        setState(MqttState::Disabled);
        ets_printf("mqtt: not enabled\n");
        return;
    }

    /* 8 KB. It builds the discovery document -- several kilobytes of String
     * appends -- and reaches the registry and keystore underneath. `bootstat`
     * prints the measured headroom, which is how the other tasks here were
     * sized after the fact. */
    if (!s_task) xTaskCreate(bridgeTask, "mqtt", 8192, nullptr, 3, &s_task);
}

void mqttApplySettings() {
    // Called from the web UI and the console, neither of which owns the client.
    // Both only ever raise the flag; the bridge task does the work. See the
    // comment on s_restartWanted.
    s_restartWanted = true;
    s_forceAll = true;

    const AppSettings &cfg = settings();
    // Switched on for the first time: there is no task yet to notice the flag.
    if (cfg.mqttEnabled && cfg.mqttHost[0] && !s_task) mqttBegin();
}

void mqttRepublish() {
    s_forceAll = true;
}

const char *mqttStateName(const MqttState state) {
    switch (state) {
        case MqttState::Disabled: return "disabled";
        case MqttState::WaitingWifi: return "waiting for wifi";
        case MqttState::Connecting: return "connecting";
        case MqttState::Connected: return "connected";
        case MqttState::Failed: return "failed";
        default: return "?";
    }
}

void mqttGetStatus(MqttStatus *out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    const AppSettings &cfg = settings();
    out->state = s_state;
    snprintf(out->broker, sizeof(out->broker), "%s:%u", cfg.mqttHost,
             cfg.mqttPort ? cfg.mqttPort : 1883);
    snprintf(out->base, sizeof(out->base), "%s", s_base[0] ? s_base : cfg.mqttBase);
    snprintf(out->clientId, sizeof(out->clientId), "%s", s_clientId);
    out->published = s_published;
    out->received = s_received;
    out->rejected = s_rejected;
    out->connects = s_connects;
    out->connectedAgoMs = s_stateAtMs ? millis() - s_stateAtMs : 0;
    snprintf(out->lastError, sizeof(out->lastError), "%s", s_lastError);
}

void mqttStatusPrint() {
    MqttStatus st{};
    mqttGetStatus(&st);
    ets_printf("mqtt: %s", mqttStateName(st.state));
    if (st.state != MqttState::Disabled) {
        ets_printf(" broker=%s prefix=%s id=%s", st.broker, st.base, st.clientId);
        ets_printf(" pub=%lu rx=%lu bad=%lu connects=%lu",
                   static_cast<unsigned long>(st.published), static_cast<unsigned long>(st.received),
                   static_cast<unsigned long>(st.rejected), static_cast<unsigned long>(st.connects));
    }
    if (st.lastError[0]) ets_printf(" last error: %s", st.lastError);
    ets_printf("\n");
}

uint32_t mqttStackHeadroom() {
    return s_task ? uxTaskGetStackHighWaterMark(s_task) : 0;
}
