/*
   Several system keys, each bound to the devices it opens. See keystore.h.
 */

#include <keystore.h>

#include <Arduino.h>
#include <Preferences.h>
#include <cstring>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
}

namespace {

/// Recursive because the public calls compose: keystoreBind() reaches
/// keystoreUnbind(), keystoreAdd() reaches keystoreFindByKey(). A plain mutex
/// would deadlock on the first rebind.
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

struct Slot {
    uint8_t key[16];
    uint8_t remote[3];  ///< 000000 when unknown.
    uint8_t device[KEYSTORE_MAX_DEVICES][3];
    uint8_t deviceCount;
    uint8_t used;
    char name[KEYSTORE_NAME_LEN];  ///< Room, e.g. "stairs". May be empty.
};

/// Magic and version so a future layout change reads as "no table" rather than
/// as garbage keys that would fail authentication in a confusing way.
constexpr uint32_t BLOB_MAGIC = 0x494F4B53;  // 'IOKS'
/// v2 added Slot::name. A stored v1 table is discarded rather than reinterpreted
/// -- main() re-runs its one-time migration and the only thing lost is a label.
constexpr uint8_t BLOB_VERSION = 2;

struct Blob {
    uint32_t magic;
    uint8_t version;
    uint8_t slotCount;
    Slot slot[KEYSTORE_MAX_SLOTS];
};

Blob table{};

/// Its own namespace: the legacy single `syskey` lives in "iohc" and has to stay
/// readable there until it has been migrated.
constexpr const char *NS = "iohckeys";
constexpr const char *BLOB_KEY = "table";

Preferences prefs;
bool opened = false;

bool open() {
    if (!opened) opened = prefs.begin(NS, false);
    return opened;
}

void save() {
    if (!open()) {
        ets_printf("keystore: NVS unavailable -- change is RAM-only\n");
        return;
    }
    table.magic = BLOB_MAGIC;
    table.version = BLOB_VERSION;
    table.slotCount = KEYSTORE_MAX_SLOTS;
    prefs.putBytes(BLOB_KEY, &table, sizeof(table));
}

bool isZero(const uint8_t *p, const size_t n) {
    for (size_t i = 0; i < n; i++)
        if (p[i]) return false;
    return true;
}

/// A short label for a key, so two slots can be told apart in the console
/// without the key itself ever being printed. FNV-1a truncated to 24 bits --
/// not a checksum and not cryptographic, just a name.
uint32_t fingerprint(const uint8_t key[16]) {
    uint32_t h = 2166136261u;
    for (uint8_t i = 0; i < 16; i++) {
        h ^= key[i];
        h *= 16777619u;
    }
    return h & 0xFFFFFF;
}

void printNode(const uint8_t n[3]) {
    ets_printf("%02X%02X%02X", n[0], n[1], n[2]);
}

}  // namespace

void keystoreBegin() {
    if (!s_lock) s_lock = xSemaphoreCreateRecursiveMutex();
    Guard g;
    memset(&table, 0, sizeof(table));
    if (!open()) {
        ets_printf("keystore: NVS unavailable -- no keys will persist\n");
        return;
    }
    Blob loaded{};
    const size_t got = prefs.getBytes(BLOB_KEY, &loaded, sizeof(loaded));
    if (got != sizeof(loaded)) {
        ets_printf("keystore: empty (%u keys)\n", 0);
        return;
    }
    if (loaded.magic != BLOB_MAGIC || loaded.version != BLOB_VERSION) {
        ets_printf("keystore: stored table is v%u, this build reads v%u -- ignoring it\n",
                   loaded.version, BLOB_VERSION);
        return;
    }
    table = loaded;
    ets_printf("keystore: %u key(s) loaded\n", keystoreCount());
}

uint8_t keystoreCount() {
    Guard g;
    uint8_t n = 0;
    for (const auto &s : table.slot)
        if (s.used) n++;
    return n;
}

int keystoreFindByKey(const uint8_t key[16]) {
    Guard g;
    for (uint8_t i = 0; i < KEYSTORE_MAX_SLOTS; i++)
        if (table.slot[i].used && memcmp(table.slot[i].key, key, 16) == 0) return i;
    return -1;
}

int keystoreFindByDevice(const uint8_t node[3]) {
    Guard g;
    for (uint8_t i = 0; i < KEYSTORE_MAX_SLOTS; i++) {
        if (!table.slot[i].used) continue;
        for (uint8_t d = 0; d < table.slot[i].deviceCount; d++)
            if (memcmp(table.slot[i].device[d], node, 3) == 0) return i;
    }
    return -1;
}

const uint8_t *keystoreKeyForDevice(const uint8_t node[3]) {
    Guard g;
    const int i = keystoreFindByDevice(node);
    return i < 0 ? nullptr : table.slot[i].key;
}

int keystoreAdd(const uint8_t key[16], const uint8_t remote[3]) {
    Guard g;
    if (const int existing = keystoreFindByKey(key); existing >= 0) {
        // Same key offered again -- typically a re-run of the extraction. Take
        // the chance to fill in the remote if it was not known the first time.
        if (remote && !isZero(remote, 3) && isZero(table.slot[existing].remote, 3)) {
            memcpy(table.slot[existing].remote, remote, 3);
            save();
        }
        return existing;
    }
    for (uint8_t i = 0; i < KEYSTORE_MAX_SLOTS; i++) {
        if (table.slot[i].used) continue;
        memset(&table.slot[i], 0, sizeof(table.slot[i]));
        memcpy(table.slot[i].key, key, 16);
        if (remote) memcpy(table.slot[i].remote, remote, 3);
        table.slot[i].used = 1;
        save();
        return i;
    }
    return -1;
}

bool keystoreBind(const uint8_t slot, const uint8_t node[3]) {
    Guard g;
    if (slot >= KEYSTORE_MAX_SLOTS || !table.slot[slot].used) return false;
    if (isZero(node, 3)) return false;

    // A device belongs to one system. Binding it somewhere new has to remove the
    // old binding, or a lookup would find whichever slot came first and
    // authenticate with a key the device has never agreed to.
    if (const int old = keystoreFindByDevice(node); old >= 0) {
        if (old == slot) return true;
        keystoreUnbind(node);
    }
    Slot &s = table.slot[slot];
    if (s.deviceCount >= KEYSTORE_MAX_DEVICES) return false;
    memcpy(s.device[s.deviceCount], node, 3);
    s.deviceCount++;
    save();
    return true;
}

bool keystoreSetRemote(const uint8_t slot, const uint8_t remote[3]) {
    Guard g;
    if (slot >= KEYSTORE_MAX_SLOTS || !table.slot[slot].used) return false;
    memcpy(table.slot[slot].remote, remote, 3);
    save();
    return true;
}

bool keystoreSetName(const uint8_t slot, const char *name) {
    Guard g;
    if (slot >= KEYSTORE_MAX_SLOTS || !table.slot[slot].used || !name) return false;
    strncpy(table.slot[slot].name, name, KEYSTORE_NAME_LEN - 1);
    table.slot[slot].name[KEYSTORE_NAME_LEN - 1] = '\0';
    save();
    return true;
}

bool keystoreUnbind(const uint8_t node[3]) {
    Guard g;
    const int i = keystoreFindByDevice(node);
    if (i < 0) return false;
    Slot &s = table.slot[i];
    for (uint8_t d = 0; d < s.deviceCount; d++) {
        if (memcmp(s.device[d], node, 3) != 0) continue;
        for (uint8_t m = d; m + 1 < s.deviceCount; m++) memcpy(s.device[m], s.device[m + 1], 3);
        s.deviceCount--;
        memset(s.device[s.deviceCount], 0, 3);
        save();
        return true;
    }
    return false;
}

bool keystoreErase(const uint8_t slot) {
    Guard g;
    if (slot >= KEYSTORE_MAX_SLOTS || !table.slot[slot].used) return false;
    memset(&table.slot[slot], 0, sizeof(table.slot[slot]));
    save();
    return true;
}

void keystoreEraseAll() {
    Guard g;
    memset(&table, 0, sizeof(table));
    save();
}

void keystoreList() {
    Guard g;
    ets_printf("keystore: %u/%u slots used\n", keystoreCount(), KEYSTORE_MAX_SLOTS);
    for (uint8_t i = 0; i < KEYSTORE_MAX_SLOTS; i++) {
        const Slot &s = table.slot[i];
        if (!s.used) continue;
        ets_printf("  [%u] %-12s key#%06X  remote ", i, s.name[0] ? s.name : "(unnamed)",
                   fingerprint(s.key));
        if (isZero(s.remote, 3))
            ets_printf("unknown");
        else
            printNode(s.remote);
        ets_printf("  devices:");
        if (!s.deviceCount) ets_printf(" none");
        for (uint8_t d = 0; d < s.deviceCount; d++) {
            ets_printf(" ");
            printNode(s.device[d]);
        }
        ets_printf("\n");
    }
}

bool keystoreGetSlot(const uint8_t slot, KeystoreSlotInfo *out) {
    Guard g;
    if (slot >= KEYSTORE_MAX_SLOTS || !out) return false;
    const Slot &s = table.slot[slot];
    memset(out, 0, sizeof(*out));
    out->slot = slot;
    out->used = s.used != 0;
    if (!out->used) return true;
    memcpy(out->name, s.name, KEYSTORE_NAME_LEN);
    out->name[KEYSTORE_NAME_LEN - 1] = '\0';
    memcpy(out->remote, s.remote, 3);
    out->deviceCount = s.deviceCount;
    memcpy(out->device, s.device, sizeof(out->device));
    out->fingerprint = fingerprint(s.key);
    return true;
}

bool keystoreParseNode(const char *hex, uint8_t out[3]) {
    if (!hex) return false;
    size_t n = strlen(hex);
    if (n != 6) return false;
    for (uint8_t i = 0; i < 6; i++)
        if (!isxdigit(static_cast<unsigned char>(hex[i]))) return false;
    for (uint8_t i = 0; i < 3; i++) {
        char b[3] = {hex[2 * i], hex[2 * i + 1], '\0'};
        out[i] = static_cast<uint8_t>(strtol(b, nullptr, 16));
    }
    return true;
}
