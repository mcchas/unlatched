/*
   Front-end settings, in NVS. See app_settings.h for what is deliberately not
   stored here.
 */

#include <app_settings.h>

#include <Arduino.h>
#include <Preferences.h>
#include <esp_mac.h>
#include <cstddef>
#include <cstring>

namespace {

constexpr uint32_t BLOB_MAGIC = 0x494F5347;  // 'IOSG'

/* 2 added the MQTT client id, retain, QoS and discovery interval.
 *
 * A version bump used to mean the stored blob was discarded and every setting
 * reverted -- including ourAddress, which is the identity this installation is
 * paired with, on a rig that is reflashed over the air and is nowhere near a
 * cable. So the loader below accepts a SHORTER blob from an older version and
 * leaves the fields it does not cover at their defaults. That works only while
 * AppSettings is append-only; see the note in app_settings.h.
 *
 * 3 added silentDefault.
 * 4 added pollIntervalS. */
constexpr uint8_t BLOB_VERSION = 4;

struct Blob {
    uint32_t magic;
    uint8_t version;
    AppSettings s;
};

/// Bytes of Blob before the settings themselves. Taken from the member rather
/// than assumed, because the padding between `version` and `s` is the
/// compiler's business.
constexpr size_t HEADER_LEN = offsetof(Blob, s);

constexpr const char *NS = "iohccfg";
constexpr const char *BLOB_KEY = "settings";

AppSettings current{};
Preferences prefs;
bool opened = false;

void defaults(AppSettings &s) {
    memset(&s, 0, sizeof(s));
    s.mqttPort = 1883;
    strncpy(s.mqttBase, "velux", sizeof(s.mqttBase) - 1);
    s.mqttRetain = true;
    s.mqttQos = 0;
    s.mqttDiscoveryS = 300;
    // These are bedrooms; quiet is the right default for a window that may be
    // driven at night. See the caveat in app_settings.h on why it is a setting.
    s.silentDefault = true;
    // Fifteen minutes. Six devices spread across it is one read every 150 s,
    // which is about three percent of the radio's time -- enough to keep the
    // published state honest without competing with listening for the remote.
    s.pollIntervalS = 900;
    s.statusAfterMoveS = 25;
    s.trainMs = 510;
    s.commandChannel = 0;  // all three: we do not know which one an actuator naps on
    /* Our node address: the last three octets of the WiFi MAC.
     *
     * It used to be a hardcoded BA11AD, the upstream author's gateway
     * constant. That is fine for one rig and wrong for two -- both would
     * transmit under the same identity onto the same installation, and an
     * actuator cannot tell them apart, because every holder of a system key
     * shares the same secret and the source address is the only thing that
     * distinguishes them. Two rigs answering each other's challenges is a
     * failure this firmware has already had one expensive version of.
     *
     * The MAC costs nothing, needs no configuration, is stable across
     * reflashes, and is the same derivation the MQTT client id uses -- so a
     * rig's address and its broker identity read as obviously one device.
     *
     * This decides only what a board with NO saved settings comes up as. A
     * stored address always wins, which is why the rig this was developed on
     * stays BA11AD: it is paired under it, and re-pairing costs a trip to
     * every remote.
     */
    uint8_t mac[6] = {};
    const bool haveMac = esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK;
    const bool usable = haveMac && (mac[3] || mac[4] || mac[5]) &&
                        !(mac[3] == 0xFF && mac[4] == 0xFF && mac[5] == 0xFF);
    if (usable) {
        s.ourAddress[0] = mac[3];
        s.ourAddress[1] = mac[4];
        s.ourAddress[2] = mac[5];
    } else {
        // 000000 is what a zeroed blob looks like and FFFFFF reads as a
        // broadcast, so neither is safe to transmit from. Fall back to the
        // historical constant rather than to something unusable.
        s.ourAddress[0] = 0xBA;
        s.ourAddress[1] = 0x11;
        s.ourAddress[2] = 0xAD;
    }
}

bool open() {
    if (!opened) opened = prefs.begin(NS, false);
    return opened;
}

}  // namespace

void settingsBegin() {
    defaults(current);
    if (!open()) {
        ets_printf("settings: NVS unavailable -- defaults only\n");
        return;
    }
    Blob loaded{};
    // Read whatever is there, up to the size this build expects. A blob written
    // by an older version is shorter; the fields it does carry land at the same
    // offsets, and the rest keep the defaults set above.
    const size_t got = prefs.getBytes(BLOB_KEY, &loaded, sizeof(loaded));
    if (got > HEADER_LEN && loaded.magic == BLOB_MAGIC && loaded.version <= BLOB_VERSION) {
        const size_t have = got - HEADER_LEN;
        memcpy(&current, &loaded.s, have < sizeof(AppSettings) ? have : sizeof(AppSettings));
        if (loaded.version < BLOB_VERSION)
            ets_printf("settings: migrated from blob version %u (%u of %u bytes)\n", loaded.version,
                       static_cast<unsigned>(have), static_cast<unsigned>(sizeof(AppSettings)));

        /* Put back the defaults for everything added since the version that
         * wrote this blob.
         *
         * This is not belt-and-braces, it is required, and the reason is nasty
         * enough to be worth stating. `have` is the STORED length, and what was
         * stored is sizeof(Blob) -- which carries the struct's trailing padding.
         * So a blob one field behind is not simply short: the copy above runs a
         * byte or two past the end of the old struct and lands on the start of
         * the new field, which then holds a padding byte rather than anything.
         *
         * Caught on hardware: silentDefault, a bool defaulting to true, came
         * back false on the first boot that read a version 2 blob. A setting
         * that silently inverts itself across an upgrade is exactly the kind of
         * thing nobody thinks to check.
         *
         * So: when you append a field, bump BLOB_VERSION and add a line here. */
        AppSettings d{};
        defaults(d);
        if (loaded.version < 2) {
            memcpy(current.mqttClientId, d.mqttClientId, sizeof(current.mqttClientId));
            current.mqttRetain = d.mqttRetain;
            current.mqttQos = d.mqttQos;
            current.mqttDiscoveryS = d.mqttDiscoveryS;
        }
        if (loaded.version < 3) current.silentDefault = d.silentDefault;
        if (loaded.version < 4) current.pollIntervalS = d.pollIntervalS;

        // A stored address of 000000 is not a valid identity -- it is what a
        // zeroed blob looks like. Fall back rather than transmitting from it.
        if (!current.ourAddress[0] && !current.ourAddress[1] && !current.ourAddress[2]) {
            AppSettings d{};
            defaults(d);
            memcpy(current.ourAddress, d.ourAddress, 3);
        }
        if (!current.trainMs) current.trainMs = 510;
        // Zero means "never" for an interval, which is not what an older blob
        // that simply predates the field is saying.
        if (!current.mqttDiscoveryS) current.mqttDiscoveryS = 300;
        if (!current.mqttPort) current.mqttPort = 1883;
        if (!current.mqttBase[0]) strncpy(current.mqttBase, "velux", sizeof(current.mqttBase) - 1);
    }
    ets_printf("settings: address %02X%02X%02X, train %ums, channel %s, mqtt %s\n",
               current.ourAddress[0], current.ourAddress[1], current.ourAddress[2], current.trainMs,
               current.commandChannel ? "fixed" : "all", current.mqttEnabled ? "on" : "off");
}

const AppSettings &settings() {
    return current;
}

void settingsSave(const AppSettings &next) {
    current = next;
    if (!current.trainMs) current.trainMs = 510;
    if (!current.ourAddress[0] && !current.ourAddress[1] && !current.ourAddress[2]) {
        AppSettings d{};
        defaults(d);
        memcpy(current.ourAddress, d.ourAddress, 3);
    }
    if (!open()) {
        ets_printf("settings: NVS unavailable -- change is RAM-only\n");
        return;
    }
    Blob blob{};
    blob.magic = BLOB_MAGIC;
    blob.version = BLOB_VERSION;
    blob.s = current;
    prefs.putBytes(BLOB_KEY, &blob, sizeof(blob));
}

void settingsReset() {
    AppSettings d{};
    defaults(d);
    settingsSave(d);
}
