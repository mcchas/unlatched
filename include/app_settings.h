/*
 * Settings the front end can change, held in NVS.
 *
 * What is NOT here, and why:
 *
 *   WiFi credentials belong to WiFiManager, which owns its own NVS namespace
 *   and its own captive portal. Keeping a second copy here would give the rig
 *   two answers to "which network" and no way to decide between them. The
 *   settings page therefore reports the connection and offers to forget it,
 *   which restarts the portal -- it does not store an SSID.
 *
 *   The system keys are in the keystore, which never hands them out. A settings
 *   page that could read a key back would put it on the wire in plaintext over
 *   the user's network every time the page loaded.
 *
 * ADDING A FIELD, in three steps, all of them needed:
 *
 *   1. Append it to the END of AppSettings. The loader reads a shorter blob
 *      from an older build by offset, which works only while the existing
 *      fields keep theirs. Reordering or removing one silently reinterprets
 *      whatever is in NVS -- and one of the fields down there is the node
 *      address the whole installation is paired with.
 *   2. Bump BLOB_VERSION in app_settings.cpp.
 *   3. Add a line to the restore block in settingsBegin() putting your field
 *      back to its default for blobs older than that version. Skipping this
 *      does not leave the field at its default: it leaves it holding a byte of
 *      the old struct's trailing padding. See the comment there.
 */

#ifndef APP_SETTINGS_H
#define APP_SETTINGS_H

#include <cstdint>

struct AppSettings {
    // -- MQTT ------------------------------------------------------------------
    char mqttHost[48];  ///< host name or IP; empty disables the bridge
    uint16_t mqttPort;
    char mqttUser[32];
    char mqttPass[32];
    char mqttBase[24];  ///< topic prefix, e.g. "velux"
    bool mqttEnabled;

    // -- the front end itself -------------------------------------------------
    /// HTTP basic-auth password, empty for none. Empty is the default: this is
    /// a rig on the user's own network, and locking someone out of a device
    /// that lives out of reach of a cable is the worse failure.
    char uiPass[32];

    // -- radio and command behaviour ------------------------------------------
    /// Seconds after a move before the rig reads the position back. Full travel
    /// on these units is around 25 s; reading too early just reports a device
    /// still on its way.
    uint16_t statusAfterMoveS;
    /// Length of the wake-up train ahead of a command, milliseconds. These
    /// actuators are low-power and hear nothing without it; the remote sends
    /// 510 ms.
    uint16_t trainMs;
    /// 802.15.4 channel to command on: 15, 20, 25, or 0 to try all three.
    uint8_t commandChannel;
    /// The address every frame is transmitted from. Persisted so that a rig
    /// paired under one identity keeps it across a reflash -- `myaddr` used to
    /// be RAM-only, so an OTA silently reverted it.
    ///
    /// Defaults to the last three octets of the WiFi MAC, so two rigs are never
    /// born with the same identity; see defaults() for why that matters. A rig
    /// already paired under an older address keeps it, because what is stored
    /// here wins over the default.
    uint8_t ourAddress[3];

    // -- appended in blob version 2; see the note at the top of this file -------

    /// MQTT client id. Empty means one derived from the MAC, which is what you
    /// want unless two rigs share a broker.
    char mqttClientId[32];
    /// Retain state topics. On by default: it is what makes a subscriber that
    /// connects later see the current position instead of waiting for the next
    /// change, and the whole point of the state topics is to be readable.
    /// Events are never retained regardless -- see mqtt_bridge.h.
    bool mqttRetain;
    /// QoS for everything the bridge publishes. 0 by default; 1 costs an
    /// acknowledgement round trip per message and the state topics are
    /// self-correcting on the next change anyway.
    uint8_t mqttQos;
    /// How often to republish the discovery document, seconds. It also goes out
    /// on every connect and whenever the inventory changes, so this is the
    /// backstop for a subscriber that missed those, not the primary path.
    uint16_t mqttDiscoveryS;

    // -- appended in blob version 3 --------------------------------------------

    /// Use the slow, quiet travel profile when the caller does not say either
    /// way. On by default: these are bedrooms, and a window that may be driven
    /// at night should not be loud about it.
    ///
    /// The silent frame is a longer payload than the ordinary one and was
    /// confirmed against these actuators when it was added (see the comment in
    /// iohc_control.cpp where it is built). It stays a setting rather than
    /// becoming the only behaviour because "try it at ordinary speed" is the
    /// first thing worth doing if a device ever refuses one, and this rig is
    /// nowhere near a cable. `sky <action> <node> loud` overrides it per
    /// command.
    bool silentDefault;

    // -- appended in blob version 4 --------------------------------------------

    /// How often to refresh every device's position, seconds. 0 disables it.
    ///
    /// Needed because these actuators volunteer nothing: a position is only
    /// known if something asked, and live state is deliberately not persisted
    /// across a reboot. Without a sweep the rig comes back knowing where
    /// nothing is, and publishes "unknown" for the whole house until somebody
    /// operates each device in turn.
    ///
    /// The reads are spread across the interval rather than fired together --
    /// one device every intervalS/count -- because each costs a wake-up train
    /// and up to three channel attempts, and a burst of six would tie up the
    /// radio for half a minute and stop it listening while it did.
    uint16_t pollIntervalS;
};

/// Load from NVS, filling anything unset with the defaults above. Call once in
/// setup(), before the radio starts.
void settingsBegin();

/// The current settings. Read-only for callers; use settingsUpdate() to change.
const AppSettings &settings();

/// Replace the settings and persist them. The caller edits a copy taken from
/// settings(), so a partial form post cannot blank the fields it left out.
void settingsSave(const AppSettings &next);

/// Restore every field to its compiled-in default and persist that.
void settingsReset();

#endif  // APP_SETTINGS_H
