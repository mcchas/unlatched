/*
 * The key handover, as something a front end can drive.
 *
 * The exchange itself lives in the receive path in main.cpp: a remote's 0x28
 * discovery draws our 0x29, its 0x31 draws a fresh challenge in a 0x3C, and its
 * 0x32 carries the system key wrapped against that challenge. That code is
 * unchanged. What it lacked was anywhere to put the result: the recovered key
 * was printed to the console as two candidates and then dropped on the floor,
 * so completing a pairing meant reading hex off a telnet session and typing it
 * back in as `keys add`.
 *
 * So the 0x32 handler now hands the key here instead. It is held in RAM,
 * unwritten, until someone names the room it belongs to and commits it -- which
 * is the one decision that genuinely needs a person: the firmware cannot know
 * whether a key that just arrived is the bathroom's or the stairs'.
 *
 * Candidate A -- the payload XORed with the keystream, without the second
 * transfert_key XOR -- is the one stored. That is not a guess: it has been the
 * correct derivation in every extraction performed on this installation, and
 * the other candidate is kept only for the console dump.
 *
 * Nothing here prints or returns key material. The front end sees a
 * fingerprint, which is enough to tell one capture from another and useless to
 * anyone who intercepts the page.
 */

#ifndef PAIRING_H
#define PAIRING_H

#include <cstdint>

/// Short notes on how far an exchange got, so the pairing screen can say
/// something more useful than "waiting". Eight is two full handshakes.
static constexpr uint8_t PAIRING_LOG_LEN = 8;
static constexpr uint8_t PAIRING_MSG_LEN = 56;

struct PairingEvent {
    char text[PAIRING_MSG_LEN];
    uint32_t agoMs;
};

struct PairingStatus {
    bool armed;         ///< answering discovery and willing to accept a key
    bool haveCapture;   ///< a key is held, waiting to be named and committed
    uint8_t remote[3];  ///< which remote sent it
    uint32_t capturedAgoMs;
    uint32_t fingerprint;  ///< of the held key; matches what `keys list` shows
    int8_t existingSlot;   ///< slot already holding this key, or -1
};

void pairingBegin();

/// Arm or disarm. Arming answers any remote's discovery and will accept a key,
/// so it disarms itself as soon as one arrives.
void pairingArm(bool on);
bool pairingArmed();

/// Record a step of the exchange for the front end. Safe from the receive path.
void pairingNote(const char *text);

/// A key arrived. @p remote is the node that sent it; @p keyA is candidate A.
void pairingNoteKey(const uint8_t remote[3], const uint8_t keyA[16]);

bool pairingStatus(PairingStatus *out);
uint8_t pairingLogCount();
bool pairingLogGet(uint8_t index, PairingEvent *out);

/// Write the held key into the keystore under @p roomName, labelled with the
/// remote it came from. Returns the slot, or -1 when nothing is held or the
/// keystore is full. The capture is cleared on success.
int pairingCommit(const char *roomName);

/// Throw the held key away.
void pairingDiscard();

#endif  // PAIRING_H
