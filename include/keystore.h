/*
 * Several io-homecontrol system keys, each bound to the devices it opens.
 *
 * The rig held exactly one system key, in one global, chosen for every
 * authentication regardless of who was asking. That works for a single system
 * and stops working here: each remote in this installation was set up on its
 * own, so each is its own system with its own key, and a skylight/blind combo
 * shares the key of the remote that drives it. Answering a challenge from one
 * system with another system's key authenticates nothing.
 *
 * So keys are stored in slots -- one slot per system, holding the key, the
 * remote it was taken from, and the node addresses known to belong to it -- and
 * looked up by the device we are actually talking to.
 *
 * Nothing here ever prints a key. A short non-cryptographic fingerprint stands
 * in for it so slots can be told apart in a console that is plaintext telnet
 * over the user's network.
 */

#ifndef KEYSTORE_H
#define KEYSTORE_H

#include <cstdint>

/* Thread safety: every entry point below takes the table's own recursive mutex.
 * It used to be reached only from the console, i.e. from one task; the web UI
 * adds a second writer, and two concurrent binds would interleave a read of
 * deviceCount with the other's increment and lose a device -- or worse, write a
 * half-updated table to NVS over four keys that each cost a trip to a remote. */

/// One slot per system. Four remotes in this installation; eight leaves room to
/// re-pair one without having to delete the old slot first.
static constexpr uint8_t KEYSTORE_MAX_SLOTS = 8;

/// Devices bound to one key. A skylight/blind combo is two; the headroom is for
/// a remote that drives several windows.
static constexpr uint8_t KEYSTORE_MAX_DEVICES = 8;

/// Room or location a slot covers ("stairs"), including the terminator. Node
/// addresses are unreadable and there will be four systems; this is also what
/// the MQTT topics will eventually be built from.
static constexpr uint8_t KEYSTORE_NAME_LEN = 20;

/// Load the table from NVS. Call once in setup(), before anything authenticates.
void keystoreBegin();

/// The key to authenticate with when talking to `node`, or nullptr if no slot
/// claims it. Deliberately no "use the only key we have" fallback: that guess is
/// right until a second key is stored and silently wrong afterwards, and a
/// failed authentication is far easier to diagnose than a wrong one.
const uint8_t *keystoreKeyForDevice(const uint8_t node[3]);

/// Slot index holding `key`, or -1. Used to make adding a key idempotent.
int keystoreFindByKey(const uint8_t key[16]);

/// Slot index that has `node` bound, or -1.
int keystoreFindByDevice(const uint8_t node[3]);

/// Store `key`, noting which remote it came from (`remote` may be nullptr).
/// Returns the slot index, the existing index if the key is already stored, or
/// -1 when every slot is taken.
int keystoreAdd(const uint8_t key[16], const uint8_t remote[3]);

/// Bind a device to a slot. A device belongs to one system, so this moves it if
/// it was bound elsewhere.
bool keystoreBind(uint8_t slot, const uint8_t node[3]);

/// Label a slot with the remote its key came from. Separate from keystoreAdd()
/// so a slot can be named without retyping the key -- the console is plaintext
/// telnet, and a key should cross it once at most.
bool keystoreSetRemote(uint8_t slot, const uint8_t remote[3]);

/// Name the room a slot covers. Truncated to KEYSTORE_NAME_LEN - 1.
bool keystoreSetName(uint8_t slot, const char *name);

/// Remove a device binding wherever it is. The key stays.
bool keystoreUnbind(const uint8_t node[3]);

/// Forget a slot: its key and every device bound to it.
bool keystoreErase(uint8_t slot);

/// Forget everything.
void keystoreEraseAll();

/// Print the table -- slots, fingerprints, remotes, bound devices. No keys.
void keystoreList();

/// Slots currently holding a key.
uint8_t keystoreCount();

/// Parse "6537B4" into three bytes. Returns false on anything else.
bool keystoreParseNode(const char *hex, uint8_t out[3]);

/// One slot, copied out for a caller that wants to render it rather than print
/// it. Carries the same fingerprint keystoreList() shows and, as there, never
/// the key itself -- the web UI is plain HTTP on the user's network.
struct KeystoreSlotInfo {
    uint8_t slot;
    bool used;
    char name[KEYSTORE_NAME_LEN];
    uint8_t remote[3];
    uint8_t deviceCount;
    uint8_t device[KEYSTORE_MAX_DEVICES][3];
    uint32_t fingerprint;
};

/// Copy slot @p slot out. Returns false for an out-of-range index; an unused
/// slot returns true with `used` false, so a caller can render the empty ones.
bool keystoreGetSlot(uint8_t slot, KeystoreSlotInfo *out);

#endif  // KEYSTORE_H
