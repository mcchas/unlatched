/*
   HTTP server and JSON API behind the front end. See web_ui.h.

   Shape of the API, so it can be read without following every handler:

     GET  /                    the page itself, gzipped, from flash
     GET  /api/state           everything the page renders, in one poll
     POST /api/command         move a device: node, action, position
     POST /api/device          name / group / type / travel for a node
     POST /api/device/forget   drop a node from the table and its key slot
     POST /api/pair            arm or disarm the key handover
     POST /api/pair/commit     store the captured key under a room name
     POST /api/pair/discard    throw the captured key away
     POST /api/slot            rename a slot, or set the remote it came from
     POST /api/slot/bind       bind or unbind a node to a slot
     POST /api/slot/forget     erase a slot and the key in it
     POST /api/settings        write the settings block
     POST /api/wifi/forget     clear WiFi credentials and restart the portal
     POST /api/reboot          restart
     GET  /api/console         output since a cursor, as text/plain
     POST /api/console         type a line into the shared command parser

   Requests carry form-encoded parameters, which WebServer parses for us, and
   answers are JSON -- so nothing here has to parse JSON on a microcontroller.
   Every POST must also carry an `X-Velux` header; see sameApp() for why.

   Documents are assembled with the appenders in json_out.h, which the MQTT
   bridge shares. They used to be private to this file; the escaping is the part
   that must not exist twice.
 */

#include <web_ui.h>

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <app_settings.h>
#include <board_config.h>
#include <boot_guard.h>
#include <device_registry.h>
#include <interact.h>
#include <iohc_control.h>
#include <json_out.h>
#include <keystore.h>
#include <mqtt_bridge.h>
#include <net_console.h>
#include <pairing.h>
#include <web_assets.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

/// The live copy of our node address, owned by main.cpp -- see the comment
/// there on why it is owned in exactly one place. Settings persist it; this is
/// what the transmit paths actually read, so a change has to reach both.
extern uint8_t ourAddress[3];

namespace {

WebServer *s_server = nullptr;
TaskHandle_t s_task = nullptr;
uint16_t s_port = 80;
bool s_running = false;
uint32_t s_requests = 0;

// ------------------------------------------------------------------ requests

/// Milliseconds -> whole seconds, which is all the page displays. Zero means
/// "never" throughout the registry, and stays zero here.
uint32_t toSeconds(const uint32_t ms) {
    return ms ? (ms + 500) / 1000 : 0;
}

bool argNode(const char *name, uint8_t out[3]) {
    if (!s_server->hasArg(name)) return false;
    return keystoreParseNode(s_server->arg(name).c_str(), out);
}

void sendJson(const int code, const String &body) {
    s_requests++;
    s_server->sendHeader("Cache-Control", "no-store");
    s_server->send(code, "application/json", body);
}

void sendOk() {
    sendJson(200, "{\"ok\":true}");
}

void sendError(const int code, const char *why) {
    String body = "{\"ok\":false,";
    kvStr(body, "error", why);
    body += '}';
    sendJson(code, body);
}

/// True when the request may proceed. An empty password means no
/// authentication, which is the default: this is a rig on the user's own
/// network, and locking someone out of a device that lives out of reach of a
/// cable is the worse failure of the two.
bool authorised() {
    const char *pass = settings().uiPass;
    if (!pass[0]) return true;
    if (s_server->authenticate("velux", pass)) return true;
    s_server->requestAuthentication();
    return false;
}

/// Guards every request that changes something.
///
/// The password above is optional and off by default, so on its own it is not
/// what stops a stranger. What this adds is protection from a page on another
/// site: that page cannot read anything back from here, because no CORS
/// headers are sent, but nothing stops it making the browser submit a form to
/// this address -- and a form post is all it takes to open someone's roof
/// windows from a site they happened to be visiting. The rig is reachable by
/// IP on a home network, which is a small space to guess.
///
/// Requiring a header closes it: a form cannot set one, and a script that
/// tries has to pass a CORS preflight this server never answers. The front
/// end sends it on every POST, and so must anything else driving this API.
bool sameApp() {
    if (s_server->hasHeader("X-Velux")) return true;
    sendError(403, "requests that change something must carry the X-Velux header");
    return false;
}

/// Both checks, in the order the caller wants them reported.
bool mayWrite() {
    return authorised() && sameApp();
}

// -------------------------------------------------------------------- state

void appendDevice(String &out, const RegistryDevice &d) {
    char group[REGISTRY_GROUP_LEN];
    char label[REGISTRY_NAME_LEN + 16];
    registryGroupOf(d.node, group, sizeof(group));
    registryLabelOf(d.node, label, sizeof(label));

    out += '{';
    kvNode(out, "node", d.node);
    out += ',';
    kvStr(out, "label", label);
    out += ',';
    kvStr(out, "name", d.name);
    out += ',';
    kvStr(out, "group", group);
    out += ',';
    // The group the device carries in its own right, as opposed to the one it
    // inherits from its key slot. The edit form needs to show the override
    // field empty when there is no override.
    kvStr(out, "groupOwn", d.group);
    out += ',';
    kvStr(out, "typeName", registryTypeName(d.type));
    out += ',';
    kvNum(out, "type", d.type);
    out += ',';
    kvNum(out, "pos", d.position);
    out += ',';
    kvNum(out, "target", d.target);
    out += ',';
    kvBool(out, "moving", d.moving);
    out += ',';
    kvNum(out, "seenAgoS", toSeconds(d.seenAgoMs));
    out += ',';
    kvNum(out, "posAgoS", toSeconds(d.posAgoMs));
    out += ',';
    kvNum(out, "ed", d.edDbm);
    out += ',';
    kvNum(out, "err", d.lastError);
    out += ',';
    kvNum(out, "slot", d.slot);
    out += ',';
    kvBool(out, "hasKey", d.hasKey);
    out += ',';
    kvNum(out, "travelS", d.travelS);
    out += '}';
}

void appendSlots(String &out) {
    out += "\"slots\":[";
    bool first = true;
    for (uint8_t i = 0; i < KEYSTORE_MAX_SLOTS; i++) {
        KeystoreSlotInfo info{};
        if (!keystoreGetSlot(i, &info)) continue;
        if (!first) out += ',';
        first = false;
        out += '{';
        kvNum(out, "slot", i);
        out += ',';
        kvBool(out, "used", info.used);
        out += ',';
        kvStr(out, "name", info.name);
        out += ',';
        kvNode(out, "remote", info.remote);
        out += ',';
        char fp[8];
        snprintf(fp, sizeof(fp), "%06X", info.fingerprint);
        kvStr(out, "fp", fp);
        out += ",\"devices\":[";
        for (uint8_t d = 0; d < info.deviceCount; d++) {
            char node[8];
            snprintf(node, sizeof(node), "%02X%02X%02X", info.device[d][0], info.device[d][1],
                     info.device[d][2]);
            if (d) out += ',';
            out += '"';
            out += node;
            out += '"';
        }
        out += "]}";
    }
    out += ']';
}

void appendPairing(String &out) {
    PairingStatus st{};
    pairingStatus(&st);
    out += "\"pair\":{";
    kvBool(out, "armed", st.armed);
    out += ',';
    kvBool(out, "haveCapture", st.haveCapture);
    out += ',';
    kvNode(out, "remote", st.remote);
    out += ',';
    char fp[8];
    snprintf(fp, sizeof(fp), "%06X", st.fingerprint);
    kvStr(out, "fp", fp);
    out += ',';
    kvNum(out, "capturedAgoS", toSeconds(st.capturedAgoMs));
    out += ',';
    kvNum(out, "existingSlot", st.existingSlot);
    out += ",\"log\":[";
    const uint8_t n = pairingLogCount();
    for (uint8_t i = 0; i < n; i++) {
        PairingEvent e{};
        if (!pairingLogGet(i, &e)) break;
        if (i) out += ',';
        out += '{';
        kvStr(out, "text", e.text);
        out += ',';
        kvNum(out, "agoS", toSeconds(e.agoMs));
        out += '}';
    }
    out += "]}";
}

void appendSettings(String &out) {
    const AppSettings &s = settings();
    out += "\"settings\":{";
    kvStr(out, "mqttHost", s.mqttHost);
    out += ',';
    kvNum(out, "mqttPort", s.mqttPort);
    out += ',';
    kvStr(out, "mqttUser", s.mqttUser);
    out += ',';
    kvStr(out, "mqttBase", s.mqttBase);
    out += ',';
    kvBool(out, "mqttEnabled", s.mqttEnabled);
    out += ',';
    kvStr(out, "mqttClientId", s.mqttClientId);
    out += ',';
    kvBool(out, "mqttRetain", s.mqttRetain);
    out += ',';
    kvNum(out, "mqttQos", s.mqttQos);
    out += ',';
    kvNum(out, "mqttDiscoveryS", s.mqttDiscoveryS);
    out += ',';
    // The passwords themselves never leave the device; the page only needs to
    // know whether one is set, so it can offer "change" rather than "set".
    kvBool(out, "mqttPassSet", s.mqttPass[0] != '\0');
    out += ',';
    kvBool(out, "uiPassSet", s.uiPass[0] != '\0');
    out += ',';
    kvNum(out, "statusAfterMoveS", s.statusAfterMoveS);
    out += ',';
    kvNum(out, "trainMs", s.trainMs);
    out += ',';
    kvNum(out, "commandChannel", s.commandChannel);
    out += ',';
    kvBool(out, "silentDefault", s.silentDefault);
    out += ',';
    kvNum(out, "pollIntervalS", s.pollIntervalS);
    out += ',';
    kvNode(out, "ourAddress", s.ourAddress);
    out += '}';
}

/// What the bridge is actually doing, as opposed to what it is configured to
/// do. The settings block says a broker is enabled; this says whether it
/// connected -- which is the question someone looking at the page has.
void appendMqtt(String &out) {
    MqttStatus st{};
    mqttGetStatus(&st);
    out += "\"mqtt\":{";
    kvStr(out, "state", mqttStateName(st.state));
    out += ',';
    kvBool(out, "up", st.state == MqttState::Connected);
    out += ',';
    kvStr(out, "broker", st.broker);
    out += ',';
    kvStr(out, "base", st.base);
    out += ',';
    kvStr(out, "clientId", st.clientId);
    out += ',';
    kvNum(out, "published", st.published);
    out += ',';
    kvNum(out, "received", st.received);
    out += ',';
    kvNum(out, "rejected", st.rejected);
    out += ',';
    kvNum(out, "connects", st.connects);
    out += ',';
    kvNum(out, "sinceS", toSeconds(st.connectedAgoMs));
    out += ',';
    kvStr(out, "error", st.lastError);
    out += '}';
}

void handleState() {
    if (!authorised()) return;
    bootGuardMark(Mark::WebState);

    String out;
    // One allocation for the whole document. Six devices, eight slots and a
    // couple of dozen observed nodes come to roughly 4 KB; growing a String by
    // doubling would fragment the heap on every poll.
    out.reserve(7600);
    out += "{\"net\":{";
    kvStr(out, "ip", WiFi.localIP().toString().c_str());
    out += ',';
    kvStr(out, "ssid", WiFi.SSID().c_str());
    out += ',';
    kvNum(out, "rssi", WiFi.RSSI());
    out += ',';
    kvBool(out, "up", WiFi.status() == WL_CONNECTED);
    out += ',';
    kvNum(out, "uptimeS", millis() / 1000);
    out += ',';
    kvNum(out, "heap", ESP.getFreeHeap());
    out += "},\"radio\":{";
    kvNum(out, "pending", controlPending());
    out += ',';
    kvBool(out, "busy", controlBusy());
    out += ',';
    kvNum(out, "keys", keystoreCount());
    out += ',';
    // Which channel the receiver is parked on right now. It rotates while the
    // rig is idle so that a window operated from a handheld remote is noticed;
    // a figure that never changes means that has stopped.
    kvNum(out, "listenCh", controlListenChannel());
    out += "},\"devices\":[";

    // The separator is tracked rather than derived from the index: a device
    // removed from another task between the count and the loop makes one
    // iteration skip, and `if (i)` would then put a comma where there is no
    // preceding element -- a document the page cannot parse, which reads at
    // the far end as the rig having gone offline.
    bool first = true;
    const uint8_t count = registryCount();
    for (uint8_t i = 0; i < count; i++) {
        RegistryDevice d{};
        if (!registryGet(i, &d)) continue;
        if (!first) out += ',';
        first = false;
        appendDevice(out, d);
    }
    out += "],\"observed\":[";

    first = true;
    const uint8_t obs = registryObservedCount();
    for (uint8_t i = 0; i < obs; i++) {
        RegistryObserved o{};
        if (!registryObservedGet(i, &o)) continue;
        if (!first) out += ',';
        first = false;
        out += '{';
        kvNode(out, "node", o.node);
        out += ',';
        kvNum(out, "agoS", toSeconds(o.seenAgoMs));
        out += ',';
        kvNum(out, "frames", o.frames);
        out += ',';
        kvNum(out, "ed", o.edDbm);
        out += ',';
        char cmd[4];
        snprintf(cmd, sizeof(cmd), "%02X", o.lastCmd);
        kvStr(out, "cmd", cmd);
        out += '}';
    }
    out += "],";

    appendSlots(out);
    out += ',';
    appendPairing(out);
    out += ',';
    appendSettings(out);
    out += ',';
    appendMqtt(out);
    out += '}';

    sendJson(200, out);
}

// ------------------------------------------------------------------ commands

void handleCommand() {
    if (!mayWrite()) return;
    bootGuardMark(Mark::WebCommand);

    IohcCommand cmd{};
    if (!argNode("node", cmd.node)) return sendError(400, "node must be six hex digits");
    if (!controlParseAction(s_server->arg("action").c_str(), &cmd.action))
        return sendError(400, "unknown action");

    if (cmd.action == IohcAction::Position) {
        const long pct = s_server->arg("position").toInt();
        cmd.percent = static_cast<uint8_t>(pct < 0 ? 0 : (pct > 100 ? 100 : pct));
    }
    // Absent means "whatever the setting says", not "off" -- otherwise the
    // default could only ever be reached by a caller that knew to ask for it.
    cmd.silent = s_server->hasArg("silent") ? s_server->arg("silent") == "1"
                                            : settings().silentDefault;
    cmd.channel = s_server->hasArg("channel")
                      ? static_cast<uint8_t>(s_server->arg("channel").toInt())
                      : settings().commandChannel;
    cmd.source = IohcSource::Web;

    if (!controlEnqueue(cmd)) return sendError(503, "command queue full");
    String body = "{\"ok\":true,";
    kvNum(body, "pending", controlPending());
    body += '}';
    sendJson(200, body);
}

void handleDevice() {
    if (!mayWrite()) return;
    uint8_t node[3];
    if (!argNode("node", node)) return sendError(400, "node must be six hex digits");

    // A missing field leaves that value alone; an empty one clears it. That
    // distinction is why the page posts only what its form actually showed --
    // renaming a device must not blank a group the user set on another screen.
    String name, group;
    const char *namePtr = nullptr;
    const char *groupPtr = nullptr;
    if (s_server->hasArg("name")) {
        name = s_server->arg("name");
        namePtr = name.c_str();
    }
    if (s_server->hasArg("group")) {
        group = s_server->arg("group");
        groupPtr = group.c_str();
    }
    uint8_t type = 0;
    const uint8_t *typePtr = nullptr;
    if (s_server->hasArg("type")) {
        type = static_cast<uint8_t>(s_server->arg("type").toInt());
        typePtr = &type;
    }
    uint16_t travel = 0;
    const uint16_t *travelPtr = nullptr;
    if (s_server->hasArg("travelS")) {
        travel = static_cast<uint16_t>(s_server->arg("travelS").toInt());
        travelPtr = &travel;
    }

    if (!registrySet(node, namePtr, groupPtr, typePtr, travelPtr))
        return sendError(507, "device table full");

    // Adding a device is also the moment to bind it to a key, because without
    // one it can be read but never moved.
    if (s_server->hasArg("slot")) {
        const long slot = s_server->arg("slot").toInt();
        if (slot < 0) {
            keystoreUnbind(node);
        } else if (!keystoreBind(static_cast<uint8_t>(slot), node)) {
            return sendError(409, "could not bind -- empty slot, or the slot is full");
        }
    }
    sendOk();
}

void handleDeviceForget() {
    if (!mayWrite()) return;
    uint8_t node[3];
    if (!argNode("node", node)) return sendError(400, "node must be six hex digits");
    keystoreUnbind(node);
    registryRemove(node);
    sendOk();
}

// ------------------------------------------------------------------- pairing

void handlePair() {
    if (!mayWrite()) return;
    pairingArm(s_server->arg("on") == "1");
    sendOk();
}

void handlePairCommit() {
    if (!mayWrite()) return;
    const String room = s_server->arg("name");
    const int slot = pairingCommit(room.c_str());
    if (slot < 0) return sendError(409, "nothing captured, or the keystore is full");
    String body = "{\"ok\":true,";
    kvNum(body, "slot", slot);
    body += '}';
    sendJson(200, body);
}

void handlePairDiscard() {
    if (!mayWrite()) return;
    pairingDiscard();
    sendOk();
}

// --------------------------------------------------------------------- slots

void handleSlot() {
    if (!mayWrite()) return;
    if (!s_server->hasArg("slot")) return sendError(400, "slot required");
    const long slot = s_server->arg("slot").toInt();
    if (slot < 0 || slot >= KEYSTORE_MAX_SLOTS) return sendError(400, "no such slot");

    if (s_server->hasArg("name") &&
        !keystoreSetName(static_cast<uint8_t>(slot), s_server->arg("name").c_str()))
        return sendError(404, "that slot is empty");
    if (uint8_t remote[3]; argNode("remote", remote))
        keystoreSetRemote(static_cast<uint8_t>(slot), remote);
    sendOk();
}

void handleSlotBind() {
    if (!mayWrite()) return;
    uint8_t node[3];
    if (!argNode("node", node)) return sendError(400, "node must be six hex digits");
    const long slot = s_server->hasArg("slot") ? s_server->arg("slot").toInt() : -1;
    if (slot < 0) {
        keystoreUnbind(node);
        return sendOk();
    }
    if (!keystoreBind(static_cast<uint8_t>(slot), node))
        return sendError(409, "could not bind -- empty slot, or the slot is full");
    // A newly bound device should appear on the devices page straight away
    // rather than after the next reboot's adoption pass.
    registryEnsure(node);
    sendOk();
}

void handleSlotForget() {
    if (!mayWrite()) return;
    if (!s_server->hasArg("slot")) return sendError(400, "slot required");
    const long slot = s_server->arg("slot").toInt();
    // Deliberately destructive and deliberately explicit: this erases a key
    // that cost a physical trip to a remote to obtain, so the page asks first
    // and the request has to say so.
    if (s_server->arg("confirm") != "yes") return sendError(400, "confirmation required");
    if (slot < 0 || !keystoreErase(static_cast<uint8_t>(slot)))
        return sendError(404, "no such slot");
    sendOk();
}

// ------------------------------------------------------------------ settings

/// Copy a form field into a fixed buffer, leaving it alone when the field was
/// not submitted at all.
void copyArg(const char *arg, char *dst, const size_t len) {
    if (!s_server->hasArg(arg)) return;
    const String v = s_server->arg(arg);
    strncpy(dst, v.c_str(), len - 1);
    dst[len - 1] = '\0';
}

void handleSettings() {
    if (!mayWrite()) return;
    // Start from what is stored, so a form that posts four fields does not
    // blank the other six.
    AppSettings next = settings();

    copyArg("mqttHost", next.mqttHost, sizeof(next.mqttHost));
    copyArg("mqttUser", next.mqttUser, sizeof(next.mqttUser));
    copyArg("mqttBase", next.mqttBase, sizeof(next.mqttBase));
    copyArg("mqttClientId", next.mqttClientId, sizeof(next.mqttClientId));
    // Passwords are only written when the field was filled in: the page cannot
    // show what is stored, so an untouched field arrives empty and must not be
    // taken to mean "clear it". Clearing is a separate, explicit flag.
    if (s_server->arg("mqttPass").length())
        copyArg("mqttPass", next.mqttPass, sizeof(next.mqttPass));
    if (s_server->arg("mqttPassClear") == "1") next.mqttPass[0] = '\0';
    if (s_server->arg("uiPass").length()) copyArg("uiPass", next.uiPass, sizeof(next.uiPass));
    if (s_server->arg("uiPassClear") == "1") next.uiPass[0] = '\0';

    if (s_server->hasArg("mqttPort"))
        next.mqttPort = static_cast<uint16_t>(s_server->arg("mqttPort").toInt());
    if (s_server->hasArg("mqttEnabled")) next.mqttEnabled = s_server->arg("mqttEnabled") == "1";
    if (s_server->hasArg("mqttRetain")) next.mqttRetain = s_server->arg("mqttRetain") == "1";
    if (s_server->hasArg("mqttQos"))
        next.mqttQos = static_cast<uint8_t>(s_server->arg("mqttQos").toInt());
    if (s_server->hasArg("mqttDiscoveryS"))
        next.mqttDiscoveryS = static_cast<uint16_t>(s_server->arg("mqttDiscoveryS").toInt());
    if (s_server->hasArg("statusAfterMoveS"))
        next.statusAfterMoveS = static_cast<uint16_t>(s_server->arg("statusAfterMoveS").toInt());
    if (s_server->hasArg("trainMs"))
        next.trainMs = static_cast<uint16_t>(s_server->arg("trainMs").toInt());
    if (s_server->hasArg("commandChannel"))
        next.commandChannel = static_cast<uint8_t>(s_server->arg("commandChannel").toInt());
    if (s_server->hasArg("silentDefault"))
        next.silentDefault = s_server->arg("silentDefault") == "1";
    if (s_server->hasArg("pollIntervalS"))
        next.pollIntervalS = static_cast<uint16_t>(s_server->arg("pollIntervalS").toInt());
    if (uint8_t addr[3]; argNode("ourAddress", addr)) {
        memcpy(next.ourAddress, addr, 3);
        // main.cpp holds the live copy every transmit path reads. Changing the
        // stored value without this would take effect only after a reboot, and
        // a rig that says one thing and transmits another is a bad thing to
        // debug over the air.
        memcpy(ourAddress, addr, 3);
    }

    settingsSave(next);
    // A broker typed in here should connect now, not after a reboot -- this rig
    // is reflashed over the air and a settings page that needs a restart to
    // mean anything is a settings page nobody trusts.
    mqttApplySettings();
    sendOk();
}

void handleWifiForget() {
    if (!mayWrite()) return;
    if (s_server->arg("confirm") != "yes") return sendError(400, "confirmation required");
    sendOk();
    // Answer first: clearing the credentials takes the interface down, and the
    // browser would otherwise show a failed request for something that worked.
    delay(250);
    WiFiManager wm;
    wm.resetSettings();
    ESP.restart();
}

void handleReboot() {
    if (!mayWrite()) return;
    sendOk();
    delay(250);
    ESP.restart();
}

// ------------------------------------------------------------------- console

void handleConsoleRead() {
    if (!authorised()) return;
    s_requests++;
    // Cursors are absolute character counts since boot. A page that has just
    // opened asks for the tail of the buffer by passing nothing.
    uint32_t cursor = 0;
    if (s_server->hasArg("from")) {
        cursor = static_cast<uint32_t>(strtoul(s_server->arg("from").c_str(), nullptr, 10));
    } else {
        const uint32_t head = netConsoleLogPos();
        cursor = head > 4096 ? head - 4096 : 0;  // enough context to see what just happened
    }

    // Answered as text rather than JSON: the log is the body, so there is
    // nothing to escape and nothing to allocate twice.
    static char chunk[2048];
    uint32_t lost = 0;
    const size_t n = netConsoleLogRead(&cursor, chunk, sizeof(chunk), &lost);

    s_server->sendHeader("Cache-Control", "no-store");
    s_server->sendHeader("X-Log-Next", String(cursor));
    s_server->sendHeader("X-Log-Lost", String(lost));
    s_server->setContentLength(n);
    s_server->send(200, "text/plain", "");
    if (n) s_server->sendContent(chunk, n);
}

void handleConsoleWrite() {
    if (!mayWrite()) return;
    const String line = s_server->arg("line");
    if (!line.length()) return sendError(400, "nothing to run");
    if (!netConsoleInject(line.c_str())) return sendError(503, "console input buffer full");
    String body = "{\"ok\":true,";
    kvNum(body, "at", netConsoleLogPos());
    body += '}';
    sendJson(200, body);
}

// -------------------------------------------------------------------- assets

void sendAsset(const char *type, const uint8_t *data, const size_t len) {
    s_requests++;
    s_server->sendHeader("Content-Encoding", "gzip");
    // The page is regenerated on every build and the device is small; caching
    // it would only make an OTA appear not to have changed.
    s_server->sendHeader("Cache-Control", "no-cache");
    s_server->send_P(200, type, reinterpret_cast<PGM_P>(data), len);
}

void handleIndex() {
    if (!authorised()) return;
    sendAsset("text/html", WEB_INDEX_GZ, WEB_INDEX_GZ_LEN);
}

void handleManifest() {
    // No authentication: a phone fetches this before it has anywhere to prompt
    // for a password, and it carries nothing but the app's name and colours.
    sendAsset("application/manifest+json", WEB_MANIFEST_GZ, WEB_MANIFEST_GZ_LEN);
}

void handleNotFound() {
    // Anything unrecognised gets the page. A single-page front end keeps its
    // tab in the URL fragment, but a phone that has added it to the home screen
    // can still come back to a path we never registered.
    if (s_server->uri().startsWith("/api/")) return sendError(404, "no such endpoint");
    handleIndex();
}

// ---------------------------------------------------------------------- task

void webTask(void *) {
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (!s_running) {
            s_server = new WebServer(s_port);
            // Request headers are discarded unless named, and sameApp() needs
            // this one.
            static const char *kHeaders[] = {"X-Velux"};
            s_server->collectHeaders(kHeaders, 1);
            s_server->on("/", HTTP_GET, handleIndex);
            s_server->on("/index.html", HTTP_GET, handleIndex);
            s_server->on("/manifest.webmanifest", HTTP_GET, handleManifest);
            s_server->on("/api/state", HTTP_GET, handleState);
            s_server->on("/api/command", HTTP_POST, handleCommand);
            s_server->on("/api/device", HTTP_POST, handleDevice);
            s_server->on("/api/device/forget", HTTP_POST, handleDeviceForget);
            s_server->on("/api/pair", HTTP_POST, handlePair);
            s_server->on("/api/pair/commit", HTTP_POST, handlePairCommit);
            s_server->on("/api/pair/discard", HTTP_POST, handlePairDiscard);
            s_server->on("/api/slot", HTTP_POST, handleSlot);
            s_server->on("/api/slot/bind", HTTP_POST, handleSlotBind);
            s_server->on("/api/slot/forget", HTTP_POST, handleSlotForget);
            s_server->on("/api/settings", HTTP_POST, handleSettings);
            s_server->on("/api/wifi/forget", HTTP_POST, handleWifiForget);
            s_server->on("/api/reboot", HTTP_POST, handleReboot);
            s_server->on("/api/console", HTTP_GET, handleConsoleRead);
            s_server->on("/api/console", HTTP_POST, handleConsoleWrite);
            s_server->onNotFound(handleNotFound);
            s_server->begin();
            s_running = true;
            ets_printf("web ui up: http://%s/\n", WiFi.localIP().toString().c_str());
        }

        s_server->handleClient();
        // handleClient() returns immediately when no client is waiting, so
        // without this the task would spin a core. 5 ms keeps the page feeling
        // immediate while leaving the radio's tasks the CPU.
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

}  // namespace

void webUiBegin(const uint16_t port) {
    s_port = port;
    // 12 KB: the state document is built on this stack as an Arduino String,
    // and WiFiManager's constructor in the forget-WiFi path is not small.
    xTaskCreate(webTask, "web-ui", 12288, nullptr, 1, &s_task);
}

uint32_t webUiStackHeadroom() {
    return s_task ? uxTaskGetStackHighWaterMark(s_task) : 0;
}

void webUiStatus() {
    ets_printf("web: %s port=%u requests=%lu\n", s_running ? "up" : "waiting for wifi", s_port,
               static_cast<unsigned long>(s_requests));
}
