/*
   Names, types, groups and live position state per node. See device_registry.h.
 */

#include <device_registry.h>

#include <Arduino.h>
#include <Preferences.h>
#include <keystore.h>
#include <cstdio>
#include <cstring>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
}

namespace {

/// The persisted half. Kept a plain POD blob rather than one NVS key per field:
/// sixteen devices is one 700-byte write, and a partial update cannot leave the
/// table describing a device that half exists.
struct StoredDevice {
    uint8_t node[3];
    uint8_t used;
    uint8_t type;
    uint16_t travelS;
    char name[REGISTRY_NAME_LEN];
    char group[REGISTRY_GROUP_LEN];
};

constexpr uint32_t BLOB_MAGIC = 0x494F4456;  // 'IODV'
constexpr uint8_t BLOB_VERSION = 1;

struct Blob {
    uint32_t magic;
    uint8_t version;
    uint8_t count;
    StoredDevice dev[REGISTRY_MAX_DEVICES];
};

/// The live half, index-parallel to Blob::dev. Deliberately not persisted: a
/// position read from before a reboot is a guess about a device someone may
/// have operated from the wall in the meantime, and a guess presented as a
/// reading is worse than "unknown".
struct LiveDevice {
    int8_t position;
    int8_t target;
    bool moving;
    uint32_t seenMs;
    uint32_t posMs;
    int8_t edDbm;
    uint8_t lastError;
};

struct Observed {
    uint8_t node[3];
    uint8_t used;
    uint32_t seenMs;
    uint16_t frames;
    int8_t edDbm;
    uint8_t lastCmd;
};

Blob table{};
/// Set when something learned in the receive path needs persisting. The write
/// itself happens elsewhere; see registryNoteType().
bool dirty = false;
LiveDevice live[REGISTRY_MAX_DEVICES]{};
Observed observed[REGISTRY_MAX_OBSERVED]{};

constexpr const char *NS = "iohcdev";
constexpr const char *BLOB_KEY = "devices";

Preferences prefs;
bool opened = false;
SemaphoreHandle_t s_lock = nullptr;

struct Guard {
    Guard() {
        if (s_lock) xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    }
    ~Guard() {
        if (s_lock) xSemaphoreGiveRecursive(s_lock);
    }
    Guard(const Guard &) = delete;
    Guard &operator=(const Guard &) = delete;
};

bool open() {
    if (!opened) opened = prefs.begin(NS, false);
    return opened;
}

void save() {
    if (!open()) {
        ets_printf("devices: NVS unavailable -- change is RAM-only\n");
        return;
    }
    table.magic = BLOB_MAGIC;
    table.version = BLOB_VERSION;
    table.count = REGISTRY_MAX_DEVICES;
    prefs.putBytes(BLOB_KEY, &table, sizeof(table));
}

int findIndex(const uint8_t node[3]) {
    for (uint8_t i = 0; i < REGISTRY_MAX_DEVICES; i++)
        if (table.dev[i].used && memcmp(table.dev[i].node, node, 3) == 0) return i;
    return -1;
}

/// Milliseconds since @p then, or 0 when it never happened. Callers treat 0 as
/// "never", which costs one millisecond of resolution at boot and saves every
/// one of them from carrying a separate validity flag.
uint32_t ago(const uint32_t then) {
    if (!then) return 0;
    const uint32_t now = millis();
    const uint32_t d = now - then;  // unsigned: correct across the 49-day wrap
    return d ? d : 1;
}

/// Wire position -> percent OPEN. 0x0000 is fully open and 0xC800 fully closed,
/// for blinds as well as skylights (checked visually against both). Anything
/// above full scale is a marker -- 0xD200 stop, 0xD400 keep -- not a position.
constexpr uint16_t POS_CLOSED = 0xC800;

int8_t rawToPercentOpen(const uint16_t raw) {
    if (raw > POS_CLOSED) return -1;
    return static_cast<int8_t>(100u - (static_cast<uint32_t>(raw) * 100u) / POS_CLOSED);
}

}  // namespace

void registryBegin() {
    if (!s_lock) s_lock = xSemaphoreCreateRecursiveMutex();
    Guard g;
    memset(&table, 0, sizeof(table));
    memset(live, 0, sizeof(live));
    memset(observed, 0, sizeof(observed));
    for (auto &l : live) {
        l.position = -1;
        l.target = -1;
    }

    if (open()) {
        Blob loaded{};
        if (prefs.getBytes(BLOB_KEY, &loaded, sizeof(loaded)) == sizeof(loaded) &&
            loaded.magic == BLOB_MAGIC && loaded.version == BLOB_VERSION) {
            table = loaded;
        }
    } else {
        ets_printf("devices: NVS unavailable -- nothing will persist\n");
    }

    // Adopt whatever the keystore already knows about. Four remotes were paired
    // from the console long before there was a front end, so without this the
    // first page load would show an empty house.
    uint8_t adopted = 0;
    for (uint8_t s = 0; s < KEYSTORE_MAX_SLOTS; s++) {
        KeystoreSlotInfo info{};
        if (!keystoreGetSlot(s, &info) || !info.used) continue;
        for (uint8_t d = 0; d < info.deviceCount; d++)
            if (findIndex(info.device[d]) < 0 && registryEnsure(info.device[d]) >= 0) adopted++;
    }
    ets_printf("devices: %u known (%u adopted from the keystore)\n", registryCount(), adopted);
}

uint8_t registryCount() {
    Guard g;
    uint8_t n = 0;
    for (const auto &d : table.dev)
        if (d.used) n++;
    return n;
}

namespace {

/// Fill @p out from slot @p i. The caller holds the lock.
void fill(const uint8_t i, RegistryDevice *out) {
    const StoredDevice &s = table.dev[i];
    const LiveDevice &l = live[i];
    memset(out, 0, sizeof(*out));
    memcpy(out->node, s.node, 3);
    memcpy(out->name, s.name, REGISTRY_NAME_LEN);
    out->name[REGISTRY_NAME_LEN - 1] = '\0';
    memcpy(out->group, s.group, REGISTRY_GROUP_LEN);
    out->group[REGISTRY_GROUP_LEN - 1] = '\0';
    out->type = s.type;
    out->travelS = s.travelS;
    out->position = l.position;
    out->target = l.target;
    out->moving = l.moving;
    out->seenAgoMs = ago(l.seenMs);
    out->posAgoMs = ago(l.posMs);
    out->edDbm = l.edDbm;
    out->lastError = l.lastError;
    out->slot = static_cast<int8_t>(keystoreFindByDevice(s.node));
    out->hasKey = keystoreKeyForDevice(s.node) != nullptr;
}

/// The index of the Nth used entry, or -1. The table is sparse after a removal
/// and the front end iterates 0..count-1.
int nthUsed(const uint8_t index) {
    uint8_t seen = 0;
    for (uint8_t i = 0; i < REGISTRY_MAX_DEVICES; i++) {
        if (!table.dev[i].used) continue;
        if (seen == index) return i;
        seen++;
    }
    return -1;
}

}  // namespace

bool registryGet(const uint8_t index, RegistryDevice *out) {
    Guard g;
    if (!out) return false;
    const int i = nthUsed(index);
    if (i < 0) return false;
    fill(static_cast<uint8_t>(i), out);
    return true;
}

bool registryGetByNode(const uint8_t node[3], RegistryDevice *out) {
    Guard g;
    if (!out) return false;
    const int i = findIndex(node);
    if (i < 0) return false;
    fill(static_cast<uint8_t>(i), out);
    return true;
}

int registryEnsure(const uint8_t node[3]) {
    Guard g;
    if (const int existing = findIndex(node); existing >= 0) return existing;
    for (uint8_t i = 0; i < REGISTRY_MAX_DEVICES; i++) {
        if (table.dev[i].used) continue;
        memset(&table.dev[i], 0, sizeof(table.dev[i]));
        memcpy(table.dev[i].node, node, 3);
        table.dev[i].used = 1;
        live[i] = LiveDevice{};
        live[i].position = -1;
        live[i].target = -1;
        save();
        return i;
    }
    return -1;
}

bool registrySet(const uint8_t node[3], const char *name, const char *group, const uint8_t *type,
                 const uint16_t *travelS) {
    Guard g;
    const int i = registryEnsure(node);
    if (i < 0) return false;
    StoredDevice &s = table.dev[i];
    if (name) {
        strncpy(s.name, name, REGISTRY_NAME_LEN - 1);
        s.name[REGISTRY_NAME_LEN - 1] = '\0';
    }
    if (group) {
        strncpy(s.group, group, REGISTRY_GROUP_LEN - 1);
        s.group[REGISTRY_GROUP_LEN - 1] = '\0';
    }
    if (type) s.type = *type;
    if (travelS) s.travelS = *travelS;
    save();
    return true;
}

bool registryRemove(const uint8_t node[3]) {
    Guard g;
    const int i = findIndex(node);
    if (i < 0) return false;
    memset(&table.dev[i], 0, sizeof(table.dev[i]));
    live[i] = LiveDevice{};
    live[i].position = -1;
    live[i].target = -1;
    save();
    return true;
}

void registryGroupOf(const uint8_t node[3], char *out, const size_t outLen) {
    Guard g;
    if (!out || !outLen) return;
    if (const int i = findIndex(node); i >= 0 && table.dev[i].group[0]) {
        snprintf(out, outLen, "%s", table.dev[i].group);
        return;
    }
    // No group of its own: fall back to the room the key was named after, which
    // is where the four systems already carry a room name.
    if (const int slot = keystoreFindByDevice(node); slot >= 0) {
        KeystoreSlotInfo info{};
        if (keystoreGetSlot(static_cast<uint8_t>(slot), &info) && info.used && info.name[0]) {
            snprintf(out, outLen, "%s", info.name);
            return;
        }
    }
    snprintf(out, outLen, "%s", "Unassigned");
}

const char *registryTypeName(const uint8_t type) {
    switch (type) {
        case DEV_TYPE_VENETIAN: return "Venetian blind";
        case DEV_TYPE_ROLLER: return "Roller shutter";
        case DEV_TYPE_AWNING: return "Awning";
        case DEV_TYPE_WINDOW: return "Skylight";
        case DEV_TYPE_LIGHT: return "Light";
        case DEV_TYPE_BLIND: return "Blind";
        case DEV_TYPE_SCREEN: return "Screen";
        default: return "Device";
    }
}

void registryLabelOf(const uint8_t node[3], char *out, const size_t outLen) {
    Guard g;
    if (!out || !outLen) return;
    const int i = findIndex(node);
    if (i >= 0 && table.dev[i].name[0]) {
        snprintf(out, outLen, "%s", table.dev[i].name);
        return;
    }
    const uint8_t type = i >= 0 ? table.dev[i].type : DEV_TYPE_UNKNOWN;
    snprintf(out, outLen, "%s %02X%02X%02X", registryTypeName(type), node[0], node[1], node[2]);
}

// ---------------------------------------------------------------- receive path

void registryNoteSeen(const uint8_t node[3], const uint8_t cmd, const int8_t edDbm) {
    Guard g;
    if (const int i = findIndex(node); i >= 0) {
        live[i].seenMs = millis();
        live[i].edDbm = edDbm;
        return;
    }

    for (auto &o : observed) {
        if (!o.used || memcmp(o.node, node, 3) != 0) continue;
        o.seenMs = millis();
        o.edDbm = edDbm;
        o.lastCmd = cmd;
        if (o.frames < 0xFFFF) o.frames++;
        return;
    }
    // New address. Take a free row, or the least recently heard one -- the band
    // carries a neighbour's Thread network and a neighbour's Velux gear, so this
    // table fills with traffic that is not the user's and must not crowd out
    // what is.
    Observed *victim = nullptr;
    for (auto &o : observed) {
        if (!o.used) {
            victim = &o;
            break;
        }
        if (!victim || o.seenMs < victim->seenMs) victim = &o;
    }
    if (!victim) return;
    memset(victim, 0, sizeof(*victim));
    memcpy(victim->node, node, 3);
    victim->used = 1;
    victim->seenMs = millis();
    victim->edDbm = edDbm;
    victim->lastCmd = cmd;
    victim->frames = 1;
}

void registryNotePosition(const uint8_t node[3], const uint16_t rawCurrent,
                          const uint16_t rawTarget) {
    Guard g;
    const int i = findIndex(node);
    if (i < 0) return;
    const int8_t cur = rawToPercentOpen(rawCurrent);
    const int8_t tgt = rawToPercentOpen(rawTarget);
    live[i].position = cur;
    live[i].posMs = millis();
    live[i].seenMs = millis();
    live[i].lastError = 0;
    if (tgt >= 0) live[i].target = tgt;
    // The device reports both, so it can say for itself whether it has arrived
    // -- better than timing the travel and hoping.
    live[i].moving = cur >= 0 && tgt >= 0 && cur != tgt;
}

void registryNoteType(const uint8_t node[3], const uint8_t type) {
    Guard g;
    const int i = findIndex(node);
    if (i < 0 || !type || table.dev[i].type == type) return;
    table.dev[i].type = type;
    // Deliberately NOT save() -- this runs on the packet decoder task, and an
    // 840-byte NVS write wants far more stack than a receive path should be
    // spending. registryFlush() does it from somewhere that can afford it.
    dirty = true;
}

void registryFlush() {
    Guard g;
    if (!dirty) return;
    dirty = false;
    save();
}

void registryNoteError(const uint8_t node[3], const uint8_t code) {
    Guard g;
    const int i = findIndex(node);
    if (i < 0) return;
    live[i].lastError = code;
    live[i].moving = false;
    live[i].seenMs = millis();
}

// --------------------------------------------------------------- transmit path

void registryNoteCommand(const uint8_t node[3], const int8_t targetPct) {
    Guard g;
    const int i = findIndex(node);
    if (i < 0) return;
    live[i].lastError = 0;
    live[i].target = targetPct;
    // A stop has no target percentage and ends the move rather than starting
    // one; anything else is in flight until a position read says otherwise.
    live[i].moving = targetPct >= 0;
}

void registryNoteSettled(const uint8_t node[3]) {
    Guard g;
    const int i = findIndex(node);
    if (i < 0) return;
    live[i].moving = false;
}

// ------------------------------------------------------------ observed nodes

uint8_t registryObservedCount() {
    Guard g;
    uint8_t n = 0;
    for (const auto &o : observed)
        if (o.used) n++;
    return n;
}

bool registryObservedGet(const uint8_t index, RegistryObserved *out) {
    Guard g;
    if (!out) return false;
    uint8_t seen = 0;
    for (const auto &o : observed) {
        if (!o.used) continue;
        if (seen++ != index) continue;
        memcpy(out->node, o.node, 3);
        out->seenAgoMs = ago(o.seenMs);
        out->frames = o.frames;
        out->edDbm = o.edDbm;
        out->lastCmd = o.lastCmd;
        return true;
    }
    return false;
}

void registryObservedClear() {
    Guard g;
    memset(observed, 0, sizeof(observed));
}
