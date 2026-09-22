#include "nvs_helpers.h"
#include <Preferences.h>

static Preferences prefs;
static bool initialized = false;

bool nvs_init() {
    if (!initialized) {
        initialized = prefs.begin("seq", false);
    }
    return initialized;
}

// Its own namespace, kept distinct from the sequence counters that used to live
// alongside: those were the 1W rolling counters, and the 1W code is gone.
static Preferences keyPrefs;
static bool keyInitialized = false;

static bool key_init() {
    if (!keyInitialized) keyInitialized = keyPrefs.begin("iohc", false);
    return keyInitialized;
}

bool nvs_read_system_key(uint8_t key[16]) {
    if (!key_init() || !keyPrefs.isKey("syskey")) return false;
    return keyPrefs.getBytes("syskey", key, 16) == 16;
}
