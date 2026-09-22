/*
   Holding a captured key until someone says where it belongs. See pairing.h.
 */

#include <pairing.h>

#include <Arduino.h>
#include <interact.h>
#include <keystore.h>
#include <cstring>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
}

namespace {

struct Capture {
    uint8_t key[16];
    uint8_t remote[3];
    uint32_t atMs;
    bool held;
};

Capture s_capture{};
PairingEvent s_log[PAIRING_LOG_LEN]{};
uint8_t s_logCount = 0;  ///< entries in use; s_log[0] is the most recent
uint32_t s_logAt[PAIRING_LOG_LEN]{};
SemaphoreHandle_t s_lock = nullptr;

struct Guard {
    Guard() {
        if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    ~Guard() {
        if (s_lock) xSemaphoreGive(s_lock);
    }
    Guard(const Guard &) = delete;
    Guard &operator=(const Guard &) = delete;
};

/// Same FNV-1a truncation the keystore prints, so a capture and the slot it
/// becomes carry the same label and can be matched by eye.
uint32_t fingerprint(const uint8_t key[16]) {
    uint32_t h = 2166136261u;
    for (uint8_t i = 0; i < 16; i++) {
        h ^= key[i];
        h *= 16777619u;
    }
    return h & 0xFFFFFF;
}

uint32_t ago(const uint32_t then) {
    if (!then) return 0;
    const uint32_t d = millis() - then;
    return d ? d : 1;
}

}  // namespace

void pairingBegin() {
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
}

void pairingArm(const bool on) {
    Cmd::pairMode = on;
    pairingNote(on ? "armed -- run Add device on the remote" : "disarmed");
}

bool pairingArmed() {
    return Cmd::pairMode;
}

void pairingNote(const char *text) {
    if (!text) return;
    Guard g;
    // Newest first: the front end shows the top few and a phone should not have
    // to scroll to find what just happened.
    for (uint8_t i = PAIRING_LOG_LEN - 1; i > 0; i--) {
        s_log[i] = s_log[i - 1];
        s_logAt[i] = s_logAt[i - 1];
    }
    strncpy(s_log[0].text, text, PAIRING_MSG_LEN - 1);
    s_log[0].text[PAIRING_MSG_LEN - 1] = '\0';
    s_logAt[0] = millis();
    if (s_logCount < PAIRING_LOG_LEN) s_logCount++;
}

void pairingNoteKey(const uint8_t remote[3], const uint8_t keyA[16]) {
    {
        Guard g;
        memcpy(s_capture.key, keyA, 16);
        memcpy(s_capture.remote, remote, 3);
        s_capture.atMs = millis();
        s_capture.held = true;
    }
    char msg[PAIRING_MSG_LEN];
    snprintf(msg, sizeof(msg), "key#%06X captured from %02X%02X%02X", fingerprint(keyA), remote[0],
             remote[1], remote[2]);
    pairingNote(msg);
}

bool pairingStatus(PairingStatus *out) {
    if (!out) return false;
    Guard g;
    memset(out, 0, sizeof(*out));
    out->armed = Cmd::pairMode;
    out->haveCapture = s_capture.held;
    out->existingSlot = -1;
    if (!s_capture.held) return true;
    memcpy(out->remote, s_capture.remote, 3);
    out->capturedAgoMs = ago(s_capture.atMs);
    out->fingerprint = fingerprint(s_capture.key);
    // Re-running an extraction on a remote that is already stored is common --
    // say so, rather than letting the user believe they are about to fill a new
    // slot and then wondering why the count did not change.
    out->existingSlot = static_cast<int8_t>(keystoreFindByKey(s_capture.key));
    return true;
}

uint8_t pairingLogCount() {
    Guard g;
    return s_logCount;
}

bool pairingLogGet(const uint8_t index, PairingEvent *out) {
    Guard g;
    if (!out || index >= s_logCount) return false;
    memcpy(out->text, s_log[index].text, PAIRING_MSG_LEN);
    out->agoMs = ago(s_logAt[index]);
    return true;
}

int pairingCommit(const char *roomName) {
    uint8_t key[16];
    uint8_t remote[3];
    {
        Guard g;
        if (!s_capture.held) return -1;
        memcpy(key, s_capture.key, 16);
        memcpy(remote, s_capture.remote, 3);
    }

    const int slot = keystoreAdd(key, remote);
    if (slot < 0) {
        pairingNote("keystore full -- delete a slot first");
        return -1;
    }
    if (roomName && *roomName) keystoreSetName(static_cast<uint8_t>(slot), roomName);

    {
        Guard g;
        // Only now: a failed commit should leave the capture where it was, so
        // the user can free a slot and try again without re-running the whole
        // procedure on the remote.
        memset(&s_capture, 0, sizeof(s_capture));
    }
    char msg[PAIRING_MSG_LEN];
    snprintf(msg, sizeof(msg), "saved to slot %d (%s)", slot,
             roomName && *roomName ? roomName : "unnamed");
    pairingNote(msg);
    return slot;
}

void pairingDiscard() {
    {
        Guard g;
        memset(&s_capture, 0, sizeof(s_capture));
    }
    pairingNote("capture discarded");
}
