/*
 * What the user calls each node, and what it is currently doing.
 *
 * The keystore answers "which key authenticates this node". It deliberately
 * says nothing about the node itself, because authentication does not need to:
 * a slot is a system, and a system is a remote plus the devices it drives.
 *
 * A front end needs the other half. A list of six-hex node addresses is not
 * something anyone can operate -- "6537B4" tells you nothing about which window
 * it opens -- so this table carries the human part: a name, the kind of device,
 * and which room it is grouped under. It also carries the live part, which is
 * not persisted: last known position, whether a move is in flight, when the
 * node was last heard.
 *
 * Kept separate from the keystore rather than bolted onto it, for one specific
 * reason: the keystore blob holds four extracted keys, each of which cost a
 * physical trip to a remote to obtain, and keystoreBegin() discards a table
 * whose version it does not recognise. Adding fields there means a version bump
 * means those keys are gone on the next OTA. This table can be extended freely
 * because everything in it can be retyped in a minute.
 *
 * Grouping is deliberately derived rather than copied. A device with no group
 * of its own reports the name of the keystore slot it is bound to, so naming a
 * slot "bathroom" groups its devices immediately and renaming it moves them.
 */

#ifndef DEVICE_REGISTRY_H
#define DEVICE_REGISTRY_H

#include <cstddef>
#include <cstdint>

/// Six devices are known in this installation; the headroom is for the
/// neighbours' nodes being adopted by mistake and then removed, and for a
/// second rig position.
static constexpr uint8_t REGISTRY_MAX_DEVICES = 16;
static constexpr uint8_t REGISTRY_NAME_LEN = 24;
static constexpr uint8_t REGISTRY_GROUP_LEN = 20;

/// Nodes heard on air but not adopted. Sized for a roll-call from a remote that
/// drives several windows, plus the neighbours' traffic.
static constexpr uint8_t REGISTRY_MAX_OBSERVED = 24;

/// io-homecontrol device type, as CMD_GET_INFO2_RESP (0x57) reports it.
/// 0 means "not asked yet"; `sky info` fills it in and it is then persisted.
enum : uint8_t {
    DEV_TYPE_UNKNOWN = 0x00,
    DEV_TYPE_VENETIAN = 0x01,
    DEV_TYPE_ROLLER = 0x02,
    DEV_TYPE_AWNING = 0x03,
    DEV_TYPE_WINDOW = 0x04,  ///< the skylight itself
    DEV_TYPE_LIGHT = 0x06,
    DEV_TYPE_BLIND = 0x0A,  ///< the blind over a skylight
    DEV_TYPE_SCREEN = 0x0B,
};

/// A device as the front end needs it: the stored half and the live half in one
/// structure, because every caller wants both.
struct RegistryDevice {
    uint8_t node[3];
    char name[REGISTRY_NAME_LEN];    ///< empty -> the caller shows type + address
    char group[REGISTRY_GROUP_LEN];  ///< empty -> the keystore slot's name
    uint8_t type;                    ///< DEV_TYPE_*
    uint16_t travelS;                ///< full-travel time; 0 -> the global default

    // ---- live, not persisted ----
    int8_t position;   ///< percent OPEN, -1 when never read
    int8_t target;     ///< percent OPEN we last asked for, -1 when none
    bool moving;       ///< a move was sent and has not been confirmed finished
    uint32_t seenAgoMs;  ///< since anything was heard from this node, 0 = never
    uint32_t posAgoMs;   ///< since the position was last read, 0 = never
    int8_t edDbm;        ///< energy of the last frame heard from it
    uint8_t lastError;   ///< last CMD_ERROR_RESP code, 0 = none

    // ---- derived ----
    int8_t slot;      ///< keystore slot, -1 when the node is bound to none
    bool hasKey;      ///< a key is bound, so movement can be authenticated
};

/// A node heard on air that is not in the table yet -- the raw material for
/// "add a device". Nodes announce themselves only during a remote's roll-call,
/// so this is what makes adding one possible without a console.
struct RegistryObserved {
    uint8_t node[3];
    uint32_t seenAgoMs;
    uint16_t frames;
    int8_t edDbm;
    uint8_t lastCmd;
};

/// Load from NVS and adopt every device the keystore already has bound. Call
/// once in setup(), after keystoreBegin().
void registryBegin();

/// Number of devices in the table.
uint8_t registryCount();

/// Copy device @p index out, live state included. False when out of range.
bool registryGet(uint8_t index, RegistryDevice *out);

/// Copy the device with this address out. False when it is not in the table.
bool registryGetByNode(const uint8_t node[3], RegistryDevice *out);

/// Add the node if it is not known, leaving any existing entry alone. Returns
/// the index, or -1 when the table is full.
int registryEnsure(const uint8_t node[3]);

/// Create or update the editable fields. A null pointer leaves that field as it
/// was, so a rename does not have to restate the group. Returns false only when
/// the node is unknown and the table is full.
bool registrySet(const uint8_t node[3], const char *name, const char *group, const uint8_t *type,
                 const uint16_t *travelS);

/// Forget a device. The keystore binding is the caller's to remove.
bool registryRemove(const uint8_t node[3]);

/// The group a device is shown under: its own, else its keystore slot's name,
/// else "Unassigned". Written into @p out rather than returned, so that no
/// caller ends up holding a pointer into a table another task can rewrite.
void registryGroupOf(const uint8_t node[3], char *out, size_t outLen);

/// The name a device is shown under: its own, else "<type> <address>".
/// Written into @p out, which must hold at least REGISTRY_NAME_LEN + 8 bytes.
void registryLabelOf(const uint8_t node[3], char *out, size_t outLen);

/// Human name for a DEV_TYPE_* value.
const char *registryTypeName(uint8_t type);

// -- fed from the receive path ---------------------------------------------
// All four are called from the packet decoder task and must stay cheap.

/// Any frame from this node. Updates last-seen for a known device, or the
/// observed table for one that is not.
void registryNoteSeen(const uint8_t node[3], uint8_t cmd, int8_t edDbm);

/// A 0x04 position reply: raw wire values, 0 = fully open, 0xC800 = closed.
void registryNotePosition(const uint8_t node[3], uint16_t rawCurrent, uint16_t rawTarget);

/// A 0x57 type reply. Remembered, and persisted later by registryFlush() --
/// this is called from the receive path, which is no place for an NVS write.
void registryNoteType(const uint8_t node[3], uint8_t type);

/// Persist anything the receive path learned. Call from a task that can afford
/// the stack and the milliseconds; does nothing when there is nothing to write.
void registryFlush();

/// A CMD_ERROR_RESP code, so the front end can say why a command did nothing.
void registryNoteError(const uint8_t node[3], uint8_t code);

// -- fed from the transmit path ---------------------------------------------

/// A move was just sent. @p targetPct is percent open, or -1 for a command
/// whose end position is not a percentage (stop, vent).
void registryNoteCommand(const uint8_t node[3], int8_t targetPct);

/// Clear the in-flight flag without a position read -- used when travel time
/// has elapsed and nothing answered.
void registryNoteSettled(const uint8_t node[3]);

// -- observed nodes ---------------------------------------------------------

uint8_t registryObservedCount();
bool registryObservedGet(uint8_t index, RegistryObserved *out);
void registryObservedClear();

#endif  // DEVICE_REGISTRY_H
