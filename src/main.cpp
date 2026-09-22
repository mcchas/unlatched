#include <board_config.h>
#include <interact.h>
#include <iohc_crypto_helpers.h>
#include <iohc_device.h>  // was reaching us via the deleted iohcCozyDevice2W.h
#include <iohc_radio.h>
#include <user_config.h>
#include <nvs_helpers.h>
#include <keystore.h>
#include <boot_guard.h>
#include <wifi_helper.h>
#include <net_console.h>
#include <app_settings.h>
#include <device_registry.h>
#include <iohc_control.h>
#include <iohc_last_command.h>
#include <mqtt_bridge.h>
#include <pairing.h>
#include <web_ui.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_random.h"
}

bool msgRcvd(IOHC::iohcPacket *receivedPacket);

uint8_t keyCap[16] = {};

/* --- 2W key handover state -------------------------------------------------
 * A KLR 200 answering our discovery replies 0x29 "waiting for 0x2C (or 0x38)".
 * 0x2C (DISCOVER_ACTUATOR) continues enumerating products; 0x38
 * (LAUNCH_KEY_TRANSFERT) is the branch that leads to the key. Which one we send
 * is a runtime choice so both can be tried without reflashing between attempts
 * -- each attempt costs the operator a physical trip to the remote.
 *
 * keyChallenge is the 6 bytes we put in our 0x38. The device's 0x32 reply is
 * its key under a keystream derived from that challenge and the well-known
 * transfert_key, so the challenge has to survive until the reply arrives or the
 * key cannot be recovered.
 *
 * It was a fixed constant -- deliberately, so attempts were reproducible while
 * the scheme was still being confirmed. Four successful extractions have
 * confirmed it, so the 0x31 handler now draws fresh bytes from the hardware RNG
 * on every exchange and this initialiser is only a placeholder. */
/* 0 = DISCOVER_ACTUATOR_0x2C, 1 = LAUNCH_KEY_TRANSFERT_0x38,
 * 2 = DISCOVER_REMOTE_ANSWER_0x2B ("2W Remote want to be paired").
 *
 * NOTE the decoder's "Waiting for 0x2C (or 0x38)" is an ets_printf in
 * iohc_packet_decoder.cpp, i.e. the upstream author's inference -- NOT something
 * the device says. All three are guesses; 0x2B is included because the role we
 * need to play is a control pad announcing itself, not a gateway enumerating
 * products, and only 0x2B carries that meaning. */
uint8_t pairReplyMode = 0;
uint8_t keyChallenge[6] = {0xab, 0x99, 0x45, 0x78, 0x38, 0x8d};

/* What we answer a device's 0x29 with, settable at runtime as a raw command
 * byte plus payload.
 *
 * Every candidate tried so far (0x2C, 0x38, 0x2B) left the KLR 200 simply
 * re-sending its 0x29, and each attempt costs a physical trip to the remote --
 * so the point of this is to sweep several candidates within ONE run instead of
 * reflashing between them. 0 disables the reply entirely, which is itself worth
 * testing: a real new control pad may say nothing at this point. */
uint8_t replyCmd29 = 0x2C;
uint8_t replyData29[16] = {};
uint8_t replyData29Len = 0;

/* What our 0x29 advertises us as when a hub's 0x28 discovery arrives. The
 * 2-byte field is (type << 6). Runtime-settable via `devtype`, because every
 * attempt costs a physical trip to the remote.
 *
 * Defaults are the literal bytes from the only known-good real KLR 200 key
 * extraction: home_io_control corpus issue_80_velux_klr200_key_extraction_success,
 * whose 0x29 is
 *     D1 00 <hub> <us> 29  00 80 <us> 02 DD 00 0E
 * i.e. type field 0x0080 (type 2, roller shutter), manufacturer 0x02 (Somfy),
 * flags 0xDD, timestamp 0x000E. That corpus entry calls the flags and timestamp
 * "the exact post-fix literal values", i.e. the hub checks them -- announcing a
 * window opener with manufacturer 0x01 and flags 0xCC, as we had been, got the
 * 0x29 accepted onto the air but never produced a 0x2C. */
uint16_t advertisedType = 0x0080;  // type 2, roller shutter
uint8_t advertisedManu = 0x02;     // Somfy
uint8_t advertisedFlags = 0xDD;    // ATT_CLASS_40S | POWER_SAVE_LOW_POWER
uint16_t advertisedTimestamp = 0x000E;
bool answer2A = false;  // also answer 0x2A discovery with 0x2B

/* The installation's system key, recovered by key extraction. 2W commands are
 * authenticated with it: the actuator challenges (0x3C) and the answer must be
 * computed under this key, not the well-known transfert_key the 0x3C handler
 * below used to use unconditionally ("seems to use the transfert_key, not the
 * system_key. This is unusual"). It is unusual because it is only correct for
 * the key-transfer exchange, where no system key exists yet.
 *
 * Kept in RAM only: it is the user's credential, and writing it into the
 * filesystem image would put it somewhere easy to leak. Set with `syskey`. */
uint8_t systemKey[16] = {};
bool haveSystemKey = false;

/* The skylight, from the KLR 200's own close command decoded 2026-09-14:
 *     4E 20 6537B4 92105F 00  01 63 C8 00 00 00
 * CtrlByte2 = 0x20 is LPM -- the actuator is a low-power device, so a command
 * has to be preceded by the wake-up train or it is simply not heard. */
uint8_t skyTarget[3] = {0x65, 0x37, 0xB4};

/* Frame building, the wake-up train and the channel hop now live in
 * src/helpers/iohc_control.cpp, behind a queue. They were inline here and ran
 * on whichever task called them -- in practice the esp_timer task, which also
 * drives iohcRadio's own timing, so one `sky close` stalled the radio's
 * timebase for up to four seconds. skyCommand() below is now only the console's
 * parser for that queue. */

IOHC::iohcRadio *radioInstance;
IOHC::iohcPacket *radioPackets[IOHC_INBOUND_MAX_PACKETS];

std::vector<IOHC::iohcPacket *> packets2send{};

uint8_t nextPacket = 0;

/* Our own node address -- the source of every frame we transmit.
 *
 * This used to be borrowed from iohcCozyDevice2W::gateway, a field on the Cozy
 * *thermostat* emulation, and the same three bytes were ALSO defined in
 * iohcOther2W::gateway and again as a file-static fake_gateway there. `myaddr`
 * had to write two of them at once to stop us transmitting under a mix of two
 * identities. Owning the value here ends that: the legacy modules keep their
 * own copies when they are compiled in, and nothing outside them reads those.
 *
 * This initialiser is only what the variable holds between C++ static init and
 * settingsBegin() a few lines into setup(), which overwrites it from NVS --
 * nothing transmits in that window. The value a rig actually uses is the stored
 * one, defaulting to the last three octets of its WiFi MAC; see defaults() in
 * app_settings.cpp for why it is no longer a constant shared by every build. */
uint8_t ourAddress[3] = {0xBA, 0x11, 0xAD};

// Longueur de préambule pour réveiller un appareil distant en veille (ex: gateway)
constexpr uint16_t PREAMBLE_LENGTH_WAKEUP = 0x0068;   // 104 symboles
constexpr uint16_t PREAMBLE_LENGTH_DEFAULT = 0x0034;  // 52 symboles

uint32_t frequencies[] = FREQS2SCAN;

using namespace IOHC;

void setup() {
    esp_log_level_set("*", ESP_LOG_VERBOSE);  // Or VERBOSE for ESP_LOGV
    Serial.begin(115200);                     //Start serial connection for debug and manual input

    // FIRST, before anything that can fault. An OTA once left this rig
    // boot-looping with no SSID and no ping -- recoverable only with a cable.
    bootGuardBegin();
    pinMode(RX_LED, OUTPUT);  // Blink this LED
    digitalWrite(RX_LED, digitalRead(RX_LED) ^ 1);
    Cmd::createCommands();
    Cmd::kbd_tick.attach_ms(500, Cmd::cmdFuncHandler);

    /* The network console, and with it the output hook, now comes BEFORE the
     * things that read NVS rather than after them.
     *
     * It used to sit below the keystore and the registry, which meant that a
     * fault in any of those printed to a UART nobody can reach and vanished.
     * That is exactly what happened on the first flash of the front end: an
     * image rolled back and left no account of itself. netConsoleBegin()
     * installs the hook as its first act and its task simply waits for WiFi,
     * so there is nothing to gain from doing it later.
     *
     * Even so, a panic does not drain the ring -- nothing runs afterwards --
     * which is why the breadcrumbs below exist as well. */
    bootGuardStage(BootStage::Console);
    netConsoleBegin();
    bootGuardReport();

    // Before anything can transmit or receive: both sides of the challenge
    // exchange go through it.
    iohcLastCommandBegin();

    bootGuardStage(BootStage::Nvs);
    nvs_init();

    bootGuardStage(BootStage::Settings);
    settingsBegin();
    // The address is persisted now. It used to be RAM-only, so every OTA
    // silently reverted a rig that had been given a fresh identity with
    // `myaddr` -- and the identity is what the installation paired with.
    memcpy(ourAddress, settings().ourAddress, 3);

    bootGuardStage(BootStage::Keystore);
    keystoreBegin();
    if (nvs_read_system_key(systemKey)) {
        haveSystemKey = true;
        ets_printf("system key loaded from NVS\n");  // value deliberately not printed
    }

    // Migrate the single stored key into the keystore, once. The old key is the
    // one that demonstrably drives skyTarget, so bind it there rather than
    // leaving it unbound -- an unbound key authenticates nothing, and this rig
    // is often out of reach when the firmware changes under it. Done here and
    // not inside the keystore because skyTarget is main's knowledge.
    if (haveSystemKey && keystoreCount() == 0) {
        const int slot = keystoreAdd(systemKey, nullptr);
        if (slot >= 0 && keystoreBind(static_cast<uint8_t>(slot), skyTarget))
            ets_printf(
                "keystore: migrated the stored key to slot %d, bound to "
                "%02X%02X%02X\n",
                slot, skyTarget[0], skyTarget[1], skyTarget[2]);
    }

    // After the keystore, because it adopts whatever the keystore already has
    // bound: four remotes were paired from the console long before there was a
    // front end, and without this the first page load would show an empty house.
    bootGuardStage(BootStage::Registry);
    registryBegin();

    bootGuardStage(BootStage::Pairing);
    pairingBegin();

    bootGuardStage(BootStage::Wifi);
    initWifi();

    // The HTTP server binds when the interface comes up, so it is safe to start
    // it here alongside the console rather than waiting on WiFi.
    bootGuardStage(BootStage::WebUi);
    webUiBegin();
    // Wait for WiFi to be connected before starting services

    // set WiFi power mode

    /* LittleFS used to be mounted here. Nothing reads it any more -- every
     * consumer was a JSON config for one of the deleted modules, and the one
     * piece of state worth keeping, the system keys, lives in NVS.
     *
     * Mounting it was also actively dangerous by this point. A failed mount
     * `return`ed out of setup(), skipping radioInstance = getInstance() and
     * everything after it, so loop() then ran against a null radio. An empty
     * filesystem guarding nothing should not be able to take out the radio. */

    bootGuardStage(BootStage::Radio);
    radioInstance = IOHC::iohcRadio::getInstance();
    // If MQTT is not defined, txCallback is null
    radioInstance->start(MAX_FREQS, frequencies, 0, msgRcvd, nullptr);

    // After the radio: the worker keys it directly.
    bootGuardStage(BootStage::Control);
    controlBegin();

    // Last, because it reads the registry and registers a handler on the
    // command worker -- both of which have to exist first. It does nothing at
    // all unless a broker is configured, and it does not need WiFi to be up:
    // its task waits for the link and connects when it arrives.
    bootGuardStage(BootStage::Mqtt);
    mqttBegin();

    // Anything that goes wrong from here is a runtime fault, not a boot one --
    // which is itself the answer to a different question, so it gets its own
    // breadcrumb rather than being left looking like the last stage.
    bootGuardStage(BootStage::Running);
    ets_printf("Startup completed. type help to see what you can do!\n");
    digitalWrite(RX_LED, 0);

    // Verification logic to match Python implementation for deriving system_key
    // challenge used from 0x3C received after sending 0x31
    // Challenge utilisé (CMD 31/3C): f4bf794c01bd
    // encrypted_key = bytes.fromhex("72c552ce5183f740c806e30ad8d3733c")
    // expected_key = bytes.fromhex("777cb0cf4fdc591d4c8173637e1b7013")
    std::vector<uint8_t> challengeAsked = {0xf4, 0xbf, 0x79, 0x4c, 0x01, 0xbd};
    std::vector<uint8_t> encrypted_key = {0x72, 0xc5, 0x52, 0xce, 0x51, 0x83, 0xf7, 0x40,
                                          0xc8, 0x06, 0xe3, 0x0a, 0xd8, 0xd3, 0x73, 0x3c};

    std::vector<uint8_t> IVdata = {IOHC::iohcDevice::ASK_CHALLENGE_0x31};

    // 1. Construct IV and encrypt it with transfer_key to get the keystream
    std::vector<uint8_t> keystream =
        iohcCrypto::encrypt_2W_payload(IVdata, challengeAsked, iohcCrypto::transfert_key);

    // 2. XOR the encrypted_key with the keystream to get the final system_key
    std::vector<uint8_t> calculated_system_key = encrypted_key;  // copy
    for (size_t i = 0; i < calculated_system_key.size(); i++) {
        calculated_system_key[i] ^= keystream[i];
    }

    ets_printf("Calculated 2W SYSTEM_KEY (KEEP IT PRIVATE): ");
    for (unsigned char idx : calculated_system_key) ets_printf("%2.2X", idx);
    ets_printf("\n");
    ets_printf("Expected 2W SYSTEM_KEY: 777CB0CF4FDC591D4C8173637E1B7013\n");
}

/**
* @brief Creates a common iohcPacket with the given data to send.
* @param packet * The packet you want to forge
* @param toSend The data that will be added to the packet
*/
/// The console's `sky` command: parse, then hand the result to the queue that
/// iohc_control.cpp owns. The parsing is unchanged; only the sending moved.
void skyCommand(Tokens *cmd) {
    if (!cmd || cmd->size() < 2) {
        ets_printf("sky <open|close|stop|vent|pos N|status|statusx|info> [targethex] [ch|0=all]\n");
        return;
    }
    const std::string &what = cmd->at(1);
    // `status` is the ordinary 3-byte CMD_PRIVATE every captured hub sends;
    // `statusx` is the 4-byte extended form real hubs send to real motors
    // (payload 03 80 <block> 00), whose reply carries a trailing 0x80-tagged
    // block. Kept as separate words so a single flash can try both against an
    // actuator that rejects one of them. `info` asks CMD_GET_INFO2 (0x56),
    // whose 0x57 reply carries the packed device type -- pairing hands us a
    // room's node addresses but not which is the skylight and which is the
    // blind, and both answer a position read identically.
    IohcCommand out{};
    if (!controlParseAction(what.c_str(), &out.action)) {
        ets_printf(
            "sky <open|close|stop|vent|pos N|status|statusx|info> [target] "
            "[ch] [silent|loud]\n");
        return;
    }
    // The configured travel profile unless this line overrides it.
    out.silent = settings().silentDefault;

    // Positional arguments after the verb, with the optional travel-profile
    // modifier lifted out so it can appear anywhere. Stripping it matters:
    // "silent" is itself six characters, and the target used to be recognised
    // as "the third token, if it is six long", which would have read it as a
    // node address.
    //
    // `loud` is the other half of that, and exists because silent is now the
    // default: without it there would be no way to ask for an ordinary-speed
    // move, which is the first thing to try when a silent one is refused.
    std::vector<std::string> args;
    for (size_t i = 2; i < cmd->size(); i++) {
        if (cmd->at(i) == "silent") {
            out.silent = true;
            continue;
        }
        if (cmd->at(i) == "loud" || cmd->at(i) == "normal") {
            out.silent = false;
            continue;
        }
        args.push_back(cmd->at(i));
    }
    size_t ai = 0;
    if (out.action == IohcAction::Position) {
        // `sky pos <percent open>`: 100 = fully open, 0 = closed, which is the
        // way a cover is spoken about.
        if (args.empty()) {
            ets_printf("sky pos <0-100 percent OPEN> [target] [ch]\n");
            return;
        }
        long pct = strtol(args[ai++].c_str(), nullptr, 10);
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        out.percent = static_cast<uint8_t>(pct);
    } else if (out.action == IohcAction::StatusExt && ai < args.size() && args[ai].size() < 6) {
        // `sky statusx 1` -- the block index, distinguished from a node address
        // the same way the target is: by not being six hex digits.
        out.percent = static_cast<uint8_t>(strtol(args[ai++].c_str(), nullptr, 0));
    }

    // Target, then optional channel. Recognising the target by "parses as six
    // hex digits" rather than by position lets `sky pos 10 2055CB` work -- the
    // old rule looked only at the third token, which `pos` had already spent on
    // the percentage, so a positional command silently went to the default
    // device. Harmless with one skylight; not with six devices in four rooms.
    memcpy(out.node, skyTarget, 3);
    if (uint8_t parsed[3]; ai < args.size() && keystoreParseNode(args[ai].c_str(), parsed)) {
        memcpy(out.node, parsed, 3);
        ai++;
    }
    out.channel = ai < args.size() ? static_cast<uint8_t>(atoi(args[ai].c_str()))
                                   : settings().commandChannel;
    out.source = IohcSource::Console;

    if (!controlEnqueue(out))
        ets_printf("command queue full -- %u already waiting\n", controlPending());
}

void forgePacket(iohcPacket *packet, const std::vector<uint8_t> &toSend) {
    IOHC::packetStamp = esp_timer_get_time();

    // Common Flags
    packet->payload.packet.header.CtrlByte1.asStruct.Protocol = 0;
    packet->payload.packet.header.CtrlByte1.asStruct.StartFrame = 1;
    packet->payload.packet.header.CtrlByte1.asStruct.EndFrame = 0;
    packet->payload.packet.header.CtrlByte1.asByte += toSend.size();
    memcpy(packet->payload.buffer + 9, toSend.data(), toSend.size());
    packet->buffer_length = toSend.size() + 9;
}

/// Framing for a device-role reply. Every command in the key-extraction
/// handshake carries its OWN start/end pair -- they are not all "single frame"
/// -- and CtrlByte2 is 0 throughout. Taken from the known-good real KLR 200
/// capture (home_io_control corpus issue_80_velux_klr200_key_extraction_success):
///     0x29 D1 -> start=1 end=1      0x2D 88 -> start=0 end=1
///     0x3C 0E -> start=0 end=0      0x33 88 -> start=0 end=1
///     0x37 0B -> start=0 end=0
/// True only when a frame is addressed to the identity we are advertising.
///
/// Every addressed handler below answers by swapping source/target, and none of
/// them used to check the target first -- so a 0x36 the hub sent to the user's
/// real skylight (6537B4) got answered by us, in six frames that claimed to BE
/// that skylight. We must never transmit as another device on the user's own
/// installation: answer only what is addressed to us.
bool addressedToUs(const iohcPacket *p) {
    return memcmp(p->payload.packet.header.target, ourAddress, 3) == 0;
}

void setFraming(iohcPacket *p, bool start, bool end) {
    p->payload.packet.header.CtrlByte1.asStruct.StartFrame = start;
    p->payload.packet.header.CtrlByte1.asStruct.EndFrame = end;
    p->payload.packet.header.CtrlByte2.asByte = 0x00;
}

/* A 0x00 -- activate / set position -- that this rig did not send.
 *
 * This is what a handheld remote transmits to move a window, so it is the
 * earliest notice we get that something in the house changed without us. The
 * actuator broadcasts its progress afterwards and the 0x04 handler picks those
 * up, but only on whichever channel we happen to be dwelling on; the command
 * frame arrives first and carries the TARGET, which the progress reports only
 * imply.
 *
 * Filed against the frame's TARGET -- the device being driven -- not its
 * source, which is the remote doing the driving. registryNoteSeen() has
 * already filed the source.
 */
void noteExternalCommand(const IOHC::iohcPacket *pkt) {
    // We never hear our own transmissions, but a second rig on the same
    // installation would, and one acting on its own moves would report every
    // one of them twice.
    if (memcmp(pkt->payload.packet.header.source, ourAddress, 3) == 0) return;

    const uint8_t *node = pkt->payload.packet.header.target;
    if (RegistryDevice dev{}; !registryGetByNode(node, &dev)) return;  // not one of ours

    const std::vector<uint8_t> d = pkt->data();
    if (d.size() < 4) return;
    // The layout this rig transmits: originator, ACEI, then the 16-bit main
    // parameter. 0x0000 fully open, 0xC800 fully closed, anything above that a
    // marker rather than a position.
    const auto raw = static_cast<uint16_t>((d[2] << 8) | d[3]);

    int8_t pct = -1;
    const char *what = "a command we do not decode";
    if (raw <= 0xC800) {
        pct = static_cast<int8_t>(100u - (static_cast<uint32_t>(raw) * 100u) / 0xC800u);
        what = "a move";
    } else if (raw == 0xD200) {
        what = "stop";
    } else if (raw == 0xD803) {
        what = "vent";
    }

    char label[REGISTRY_NAME_LEN + 16];
    registryLabelOf(node, label, sizeof(label));
    ets_printf("*** %s: %s from %02X%02X%02X (not us)", label, what,
               pkt->payload.packet.header.source[0], pkt->payload.packet.header.source[1],
               pkt->payload.packet.header.source[2]);
    if (pct >= 0) ets_printf(" -> %d%% open", pct);
    ets_printf("\n");

    // Believe the target immediately, then confirm where it actually stopped.
    registryNoteCommand(node, pct);
    controlNoteExternalMove(node);
}

bool msgRcvd(IOHC::iohcPacket *receivedPacket) {
    /* A JsonDocument used to be built here on EVERY received packet, written
     * to in six places and never serialised -- the MQTT publisher that read it
     * was deleted. That is a heap allocation per frame in the receive path for
     * nothing. Removing it also drops the ArduinoJson dependency entirely. */

    /* Note every source address. A device that is already known gets its
     * last-seen stamp refreshed; one that is not lands in the observed table,
     * which is what makes "add a device" possible from a phone -- nodes
     * announce themselves only during a remote's roll-call, so an address you
     * have not written down is otherwise unreachable.
     *
     * A linear scan of at most 16 + 24 rows, on the decoder task. The band
     * carries a neighbour's Thread network, so this runs often. */
    bootGuardMark(Mark::RxEnter);
    registryNoteSeen(receivedPacket->payload.packet.header.source, receivedPacket->cmd(),
                     static_cast<int8_t>(receivedPacket->rssi));

    /* Tell the command worker that the device it is currently talking to said
     * something. It only sets a flag, and only when the address matches the
     * command on air; it does not influence the channel hop or the hold timing,
     * which the 0x3C handler alone drives. What it buys is the difference
     * between "the actuator never answered" and "it answered and refused",
     * which is the difference between a range problem and a device problem. */
    controlNoteReply(receivedPacket->payload.packet.header.source, receivedPacket->cmd());

    switch (receivedPacket->cmd()) {
        case iohcDevice::DISCOVER_0x28: {
            if (!Cmd::pairMode) break;

            // Node type and subtype (2 bytes): type on 10 bits and subtype on the remainer
            // Node type = (field >> 6) & 1023
            // Node subtype = field & 63
            // Node address (3 bytes)
            // Manufacturer ID (1 byte)
            // Multiinfo (1 byte)
            // Timestamp (2 bytes)

            // 0x0b OverKiz 0x0c Atlantic
            // Mirrors the shape of the KLR 200's own 0x29 (ffc0 92105f 01 cc0000):
            // type(2) + our address(3) + manufacturer + info(3). Carrying OUR
            // address matters -- the previous hardcoded payload announced the
            // upstream author's sunblind at 344E1F, which is not us and which the
            // hub has no reason to pair with.
            // type(2) + our id(3) + manufacturer + flags + timestamp(2).
            std::vector<uint8_t> toSend = {static_cast<uint8_t>(advertisedType >> 8),
                                           static_cast<uint8_t>(advertisedType & 0xFF),
                                           ourAddress[0],
                                           ourAddress[1],
                                           ourAddress[2],
                                           advertisedManu,
                                           advertisedFlags,
                                           static_cast<uint8_t>(advertisedTimestamp >> 8),
                                           static_cast<uint8_t>(advertisedTimestamp & 0xFF)};
            pairingNote("discovery seen -- announced ourselves (0x29)");
            ets_printf(
                "0x28 -> 0x29 type 0x%04X manu 0x%02X flags 0x%02X ts 0x%04X addr %02X%02X%02X\n",
                advertisedType, advertisedManu, advertisedFlags, advertisedTimestamp, ourAddress[0],
                ourAddress[1], ourAddress[2]);

            iohcPacket response;

            forgePacket(&response, toSend);

            setFraming(&response, true, true);  // 0xD1 in the corpus

            response.payload.packet.header.cmd = IOHC::iohcDevice::DISCOVER_ANSWER_0x29;

            /* Swap */
            memcpy(response.payload.packet.header.source, ourAddress, 3);
            memcpy(response.payload.packet.header.target,
                   receivedPacket->payload.packet.header.source, 3);

            response.repeatTime = 50;

            // Send the 0x29 on EVERY channel, starting with the one the 0x28
            // arrived on. 0x28 is a broadcast, and a hub waiting for replies to
            // a broadcast rotates channels while it listens (home_io_control
            // ADR 0028: its wait_for_discovery_response_ uses ROTATE_ALL_CHANNELS,
            // and their roll-call measured just 1 reply of 149 arriving back on
            // the request channel). Answering once, on one channel, therefore has
            // at best a one-in-three chance of landing while the hub is there --
            // which matches what we see: the 0x29 goes out correctly formed and
            // no 0x2C ever follows.
            const uint32_t chans[3] = {CHANNEL1, CHANNEL2, CHANNEL3};
            response.frequency = receivedPacket->frequency;
            radioInstance->sendPriority(&response);
            for (uint8_t c = 0; c < 3; c++) {
                if (chans[c] == receivedPacket->frequency) continue;  // already sent there
                iohcPacket alt = response;
                alt.frequency = chans[c];
                radioInstance->sendPriority(&alt);
            }

            break;
        }
        case iohcDevice::DISCOVER_ANSWER_0x29: {
            //
            //
            if (!Cmd::pairMode) break;

            // The device offers two continuations. 0x2C enumerates products;
            // 0x38 launches the key transfer, which is the one that ends in a
            // key. See keyTransferMode above.
            if (!addressedToUs(receivedPacket)) break;
            if (replyCmd29 == 0) {
                ets_printf("0x29 seen; replying with nothing (reply29 0)\n");
                break;
            }
            std::vector<uint8_t> toSend(replyData29, replyData29 + replyData29Len);
            iohcPacket response;
            const uint8_t replyCmd = replyCmd29;
            ets_printf("0x29 -> replying 0x%02X", replyCmd);
            if (replyData29Len) {
                ets_printf(" data ");
                for (unsigned char c : toSend) ets_printf("%02X", c);
            }
            ets_printf("\n");

            forgePacket(&response, toSend);

            // forgePacket leaves EndFrame = 0, meaning "more frames follow", so
            // the device waits for a continuation that never comes and simply
            // re-sends its 0x29. Measured 2026-09-14: every frame it has ever
            // answered carries EndFrame = 1 -- our discovery (CB1 0xC8) and its
            // own 0x29 (CB1 0xD1) -- while both replies built here (0x48, 0x4E)
            // had it clear and were ignored nine times running. A single-frame
            // reply has to say it is complete.
            response.payload.packet.header.CtrlByte1.asStruct.EndFrame = 1;

            response.payload.packet.header.cmd = replyCmd;

            /* Swap */
            memcpy(response.payload.packet.header.source,
                   receivedPacket->payload.packet.header.target, 3);
            memcpy(response.payload.packet.header.target,
                   receivedPacket->payload.packet.header.source, 3);

            // Now that the swap has given us our own address, fill the hole left
            // in the 0x2B announcement (payload starts at buffer + 9).
            if (pairReplyMode == 2) {
                memcpy(response.payload.buffer + 9 + 2, response.payload.packet.header.source, 3);
            }

            response.frequency = receivedPacket->frequency;

            // Send it repeatedly rather than once. The device is duty-cycled: it
            // precedes its OWN frames to us with a ~510 ms wake-up train, so it
            // is only listening for a slice of each cycle. That also explains why
            // it answers our discovery -- which is transmitted continuously, so
            // one lands whenever it wakes -- while every single-shot reply here
            // went unanswered, whatever command it carried. Frames go out ~75 ms
            // apart, so 16 spans ~1.2 s and covers a 510 ms cycle twice over.
            // The proper fix is to generate the wake-up train ourselves; this
            // tests the hypothesis first, since it is one line.
            response.repeat = 16;
            radioInstance->sendPriority(&response);

            break;
        }
        case iohcDevice::DISCOVER_REMOTE_ANSWER_0x2B: {
            break;
        }
        case iohcDevice::DISCOVER_ACTUATOR_0x2C: {
            if (!Cmd::pairMode || !addressedToUs(receivedPacket)) break;

            std::vector<uint8_t> toSend = {};

            iohcPacket response;

            forgePacket(&response, toSend);

            setFraming(&response, false, true);  // 0x88 in the corpus
            response.payload.packet.header.cmd = IOHC::iohcDevice::DISCOVER_ACTUATOR_ACK_0x2D;

            /* Swap */
            memcpy(response.payload.packet.header.source,
                   receivedPacket->payload.packet.header.target, 3);
            memcpy(response.payload.packet.header.target,
                   receivedPacket->payload.packet.header.source, 3);

            response.repeatTime = 50;
            response.frequency = receivedPacket->frequency;

            radioInstance->sendPriority(&response);

            break;
        }
        case iohcDevice::LAUNCH_KEY_TRANSFERT_0x38: {
            std::vector<uint8_t> key_transfert;
            key_transfert.assign(receivedPacket->payload.buffer + 9,
                                 receivedPacket->payload.buffer + 15);

            for (unsigned char i : key_transfert) {
                ets_printf("%02X ", i);
            }
            ets_printf("\n");

            std::vector<uint8_t> data = {IOHC::iohcDevice::ASK_CHALLENGE_0x31};  //0x38
            std::vector<uint8_t> initial_value =
                iohcCrypto::constructInitialValue(data, key_transfert.data(), nullptr);
            ets_printf("2) Initial value used for key encryption: ");
            for (unsigned char i : initial_value) {
                ets_printf("%02X ", i);
            }
            ets_printf("\n");

            std::vector<uint8_t> encrypted_key =
                iohcCrypto::encrypt_2W_payload(data, key_transfert, iohcCrypto::transfert_key);
            //  XORing transfert_key
            for (int i = 0; i < 16; i++) {
                encrypted_key[i] = encrypted_key[i] ^ iohcCrypto::transfert_key[i];
            }
            ets_printf("2) Encrypted 2-way key to be sent with SEND_KEY_TRANSFERT_0x32: ");
            for (unsigned char i : encrypted_key) {
                ets_printf("%02X ", i);
            }
            ets_printf("\n");
            if (!Cmd::pairMode) break;

            iohcPacket response;
            forgePacket(&response, encrypted_key);

            response.payload.packet.header.cmd = IOHC::iohcDevice::KEY_TRANSFERT_0x32;

            /* Swap */
            memcpy(response.payload.packet.header.source,
                   receivedPacket->payload.packet.header.target, 3);
            memcpy(response.payload.packet.header.target,
                   receivedPacket->payload.packet.header.source, 3);

            radioInstance->sendPriority(&response);
            break;
        }
        case iohcDevice::DISCOVER_REMOTE_0x2A: {
            // The hub interleaves 0x2A (looking for remotes) with its 0x28
            // (looking for devices) -- measured: 3 of each in one add-product
            // run, plus 7 address requests to an already-paired skylight. We
            // answer as a DEVICE (0x29 to its 0x28), so this stays off unless
            // asked: claiming to be both a device and a control pad at once is
            // more likely to confuse the hub than to help.
            if (!Cmd::pairMode || !answer2A) break;

            std::vector<uint8_t> toSend = {static_cast<uint8_t>(advertisedType >> 8),
                                           static_cast<uint8_t>(advertisedType & 0xFF),
                                           ourAddress[0],
                                           ourAddress[1],
                                           ourAddress[2],
                                           advertisedManu,
                                           0xCC,
                                           0x00,
                                           0x00};
            ets_printf("0x2A -> answering 0x2B (announcing as a control pad)\n");

            iohcPacket response;
            forgePacket(&response, toSend);
            response.payload.packet.header.CtrlByte1.asStruct.EndFrame = 1;
            response.payload.packet.header.cmd = iohcDevice::DISCOVER_REMOTE_ANSWER_0x2B;
            memcpy(response.payload.packet.header.source, ourAddress, 3);
            memcpy(response.payload.packet.header.target,
                   receivedPacket->payload.packet.header.source, 3);
            response.frequency = receivedPacket->frequency;
            radioInstance->sendPriority(&response);
            break;
        }
        case iohcDevice::ADDRESS_REQUEST_0x36: {
            // The address-verification round that follows the key transfer. The
            // KLR 200 does this (the KIG 300 does not), and the responder must
            // stay listening ~60s past "key extracted" to catch it. The payload
            // echoes our own backbone address -- the same id advertised in 0x29.
            // We had been ignoring 0x36 entirely: it decoded as "Unknown command
            // 36" in every capture.
            if (!Cmd::pairMode || !addressedToUs(receivedPacket)) break;
            const uint8_t *me = receivedPacket->payload.packet.header.target;
            std::vector<uint8_t> toSend(me, me + 3);
            iohcPacket response;
            forgePacket(&response, toSend);
            setFraming(&response, false, false);        // 0x0B in the corpus
            response.payload.packet.header.cmd = 0x37;  // ADDRESS_RESP; no enum for it upstream
            memcpy(response.payload.packet.header.source,
                   receivedPacket->payload.packet.header.target, 3);
            memcpy(response.payload.packet.header.target,
                   receivedPacket->payload.packet.header.source, 3);
            response.frequency = receivedPacket->frequency;
            pairingNote("address check answered (0x37)");
            ets_printf("\n*** ADDRESS_REQ 0x36 -> replying 0x37 with %02X%02X%02X\n", toSend[0],
                       toSend[1], toSend[2]);
            radioInstance->sendPriority(&response);
            break;
        }
        case iohcDevice::ASK_CHALLENGE_0x31: {
            // Device role, step 2 of the key handover (home_io_control ADR 0012):
            //   hub 0x28 -> us 0x29 -> hub 0x31 -> us 0x3C challenge
            //   -> hub 0x32 (node_id + system_key) -> us 0x33 confirm
            // The hub encrypts the key against the challenge we send here, so
            // keyChallenge has to be the same bytes the 0x32 handler later uses.
            // Previously 0x31 fell into a catch-all `break`, so the challenge was
            // never sent and the handover could not progress past this point.
            if (!Cmd::pairMode || !addressedToUs(receivedPacket)) break;

            // Fresh bytes per exchange, from the hardware RNG. The old constant
            // went out on air identically in four extractions two days apart,
            // which is not a property to leave in a device that stays armed.
            for (unsigned char &c : keyChallenge) c = esp_random() & 0xFF;

            pairingNote("key transfer started -- challenge sent (0x3C)");
            ets_printf("\n*** ASK_CHALLENGE_0x31 -- replying 0x3C with challenge ");
            for (unsigned char c : keyChallenge) ets_printf("%02X", c);
            ets_printf("\n");

            std::vector<uint8_t> toSend(keyChallenge, keyChallenge + sizeof(keyChallenge));
            iohcPacket response;
            forgePacket(&response, toSend);
            setFraming(&response, false, false);  // 0x0E in the corpus, NOT end=1
            response.payload.packet.header.cmd = iohcDevice::CHALLENGE_REQUEST_0x3C;

            /* Swap */
            memcpy(response.payload.packet.header.source,
                   receivedPacket->payload.packet.header.target, 3);
            memcpy(response.payload.packet.header.target,
                   receivedPacket->payload.packet.header.source, 3);

            // A unicast reply returns on the channel its request arrived on
            // (home_io_control ADR 0028, measured with no exceptions), and
            // receivedPacket->frequency now records the real channel.
            response.frequency = receivedPacket->frequency;
            iohcLastCommandSetCmd(iohcDevice::ASK_CHALLENGE_0x31);  // the 0x3C path keys off this
            radioInstance->sendPriority(&response);
            break;
        }
        case 0x04: {
            // CMD_PRIVATE_RESP: the answer to our 0x03 status request. Layout
            // current at data[4], both big-endian, on the same 0..0xC800 scale
            // as the command's main parameter (0 = fully open).
            const std::vector<uint8_t> d = receivedPacket->data();
            // Dump the raw payload before interpreting any of it. The reply
            // framing is device-dependent -- some devices lead with the stopped
            // flag byte and an 0x80-tagged block, others with 0x2D and no block
            // -- so the bytes are worth more than a decode that assumes one
            // shape, and this is the first 0x04 we have ever drawn.
            ets_printf("\n*** 0x04 from %02X%02X%02X, %u data bytes: ",
                       receivedPacket->payload.packet.header.source[0],
                       receivedPacket->payload.packet.header.source[1],
                       receivedPacket->payload.packet.header.source[2], d.size());
            for (unsigned char c : d) ets_printf("%02X", c);
            ets_printf("\n");
            if (d.size() < 6) {
                ets_printf("0x04 status reply too short (%u bytes)\n", d.size());
                break;
            }
            const uint16_t tgt = (d[2] << 8) | d[3];
            const uint16_t cur = (d[4] << 8) | d[5];
            // The only unsolicited-looking state these actuators ever give us.
            // Everything the front end shows about where a window is comes from
            // here; without it a position is whatever the user last asked for.
            bootGuardMark(Mark::RxPosition);
            registryNotePosition(receivedPacket->payload.packet.header.source, cur, tgt);
            ets_printf("\n*** POSITION from %02X%02X%02X: raw target 0x%04X current 0x%04X\n",
                       receivedPacket->payload.packet.header.source[0],
                       receivedPacket->payload.packet.header.source[1],
                       receivedPacket->payload.packet.header.source[2], tgt, cur);
            // Values above full scale are markers (0xD2 stop, 0xD4 keep), not positions.
            if (cur <= 0xC800)
                ets_printf("    current: %u%% open\n", 100u - (cur * 100u) / 0xC800u);
            else
                ets_printf("    current: unknown (marker 0x%04X)\n", cur);
            if (tgt <= 0xC800)
                ets_printf("    target:  %u%% open\n\n", 100u - (tgt * 100u) / 0xC800u);
            else
                ets_printf("    target:  unknown (marker 0x%04X)\n\n", tgt);
            break;
        }
        case 0x57: {
            // CMD_GET_INFO2_RESP. data[0..9] are a printable manufacturer-internal
            // reference string; data[10..11] pack the device type and subtype;
            // data[12..15] have no published decode. Offsets and the unpacking
            // from home_io_control hub_status.cpp:55 and proto_device_model.cpp:61.
            const std::vector<uint8_t> d = receivedPacket->data();
            const uint8_t *src = receivedPacket->payload.packet.header.source;
            ets_printf("\n*** 0x57 from %02X%02X%02X (%u bytes): ", src[0], src[1], src[2],
                       d.size());
            for (unsigned char c : d) ets_printf("%02X", c);
            if (d.size() < 12) {
                ets_printf("\n    too short to carry a type\n");
                break;
            }

            ets_printf("\n    ref  ");
            for (uint8_t i = 0; i < 10; i++) ets_printf("%c", isprint(d[i]) ? d[i] : '.');

            const uint8_t type = (d[10] << 2) | (d[11] >> 6);
            const uint8_t subtype = d[11] & 0x3F;
            // Persisted, so a device only ever has to be asked once. This is
            // what tells a skylight from the blind over it -- pairing hands us
            // a room's node addresses and both answer a position read
            // identically.
            registryNoteType(receivedPacket->payload.packet.header.source, type);
            const char *name;
            switch (type) {
                case 0x01: name = "venetian blind"; break;
                case 0x02: name = "roller shutter"; break;  // the blind over a skylight
                case 0x03: name = "awning"; break;
                case 0x04: name = "window opener"; break;  // the skylight itself
                case 0x06: name = "light"; break;
                case 0x0A: name = "blind"; break;
                case 0x0B: name = "screen"; break;
                case 0x10: name = "horizontal awning"; break;
                default: name = "unmapped"; break;
            }
            ets_printf("\n    type 0x%02X (%s) subtype %u\n\n", type, name, subtype);
            break;
        }
        case iohcDevice::KEY_TRANSFERT_0x32: {
            // The answer to our 0x38: the device's key, carried under a keystream
            // derived from the challenge we sent and the well-known transfert_key.
            // Print the ciphertext and the challenge whatever happens -- if the
            // unwrapping below is wrong, those two are enough to recover the key
            // offline (tools/iohc_keydecrypt.py), and re-running the procedure
            // costs a trip to the remote.
            std::vector<uint8_t> payload = receivedPacket->data();
            ets_printf("\n*** KEY_TRANSFERT_0x32 from ");
            for (uint8_t i = 0; i < 3; i++)
                ets_printf("%02X", receivedPacket->payload.packet.header.source[i]);
            ets_printf(" (%u bytes)\n    ciphertext ", payload.size());
            for (unsigned char c : payload) ets_printf("%02X", c);
            ets_printf("\n    challenge  ");
            for (unsigned char c : keyChallenge) ets_printf("%02X", c);
            ets_printf("\n");

            if (!addressedToUs(receivedPacket)) break;
            if (payload.size() >= 16) {
                std::vector<uint8_t> iv = {iohcDevice::ASK_CHALLENGE_0x31};
                std::vector<uint8_t> ks = iohcCrypto::encrypt_2W_payload(
                    iv, std::vector<uint8_t>(keyChallenge, keyChallenge + sizeof(keyChallenge)),
                    iohcCrypto::transfert_key);
                // Two candidates, because the sender's construction is not
                // confirmed: upstream's own 0x38 responder XORs the keystream
                // with transfert_key before transmitting, so try both.
                uint8_t candidateA[16];
                for (uint8_t i = 0; i < 16; i++) candidateA[i] = payload[i] ^ ks[i];
                ets_printf("    candidate A ");
                for (unsigned char c : candidateA) ets_printf("%02X", c);
                ets_printf("\n    candidate B ");
                for (uint8_t i = 0; i < 16; i++)
                    ets_printf("%02X", payload[i] ^ ks[i] ^ iohcCrypto::transfert_key[i]);
                ets_printf("\n");
                memcpy(keyCap, payload.data(), 16);
                // Hold candidate A for the pairing screen to name and commit.
                // It has been the correct derivation in every extraction on
                // this installation; candidate B stays in the dump above for
                // the case where that stops being true.
                bootGuardMark(Mark::RxKeyTransfer);
                pairingNoteKey(receivedPacket->payload.packet.header.source, candidateA);
            }
            // Close the handshake: the hub waits for 0x33 before it considers the
            // pairing done, and a hub left waiting may roll the credentials back.
            if (Cmd::pairMode) {
                std::vector<uint8_t> ack;
                iohcPacket confirm;
                forgePacket(&confirm, ack);
                setFraming(&confirm, false, true);  // 0x88 in the corpus
                confirm.payload.packet.header.cmd = iohcDevice::KEY_TRANSFERT_ACK_0x33;
                memcpy(confirm.payload.packet.header.source,
                       receivedPacket->payload.packet.header.target, 3);
                memcpy(confirm.payload.packet.header.target,
                       receivedPacket->payload.packet.header.source, 3);
                confirm.frequency = receivedPacket->frequency;
                radioInstance->sendPriority(&confirm);
                ets_printf("*** sent KEY_TRANSFERT_ACK_0x33\n\n");

                // Disarm. We have what we came for, and an armed rig answers any
                // remote's 0x28 and will hand out a challenge and accept a key --
                // a pairing surface with no reason to stay open. It was left on
                // across three consecutive extractions today simply because
                // nothing turned it off. Re-arm with `pairMode 1`.
                Cmd::pairMode = false;
                pairingNote("handshake acknowledged, pairing disarmed");
                ets_printf("pairMode off (armed only until a key arrives)\n");
            }
            break;
        }
        case iohcDevice::WRITE_PRIVATE_0x20: {
            break;
        }
        case iohcDevice::PRIVATE_ACK_0x21: {
            break;
        }
        case iohcDevice::CHALLENGE_REQUEST_0x3C: {
            /* Only a challenge addressed to US. The comment here always said so
             * -- "answer only to our fake gateway, not to others real devices"
             * -- but the test underneath it was `if (true)` with the `break`
             * commented out, so the rig answered every challenge it could hear.
             *
             * That is not theoretical. Captured 2026-09-22: the stairs remote
             * 92105F told 6537B4 to close, the skylight challenged 92105F, and
             * this handler keyed out a 0x3D of its own within the 9-12 ms
             * window -- ahead of the remote's. Worse, the address swap below
             * makes source = the challenge's target, so that answer went out
             * UNDER THE REMOTE'S ADDRESS, signed over whatever this rig last
             * did (logged as "Last Command 00 (3)", a stale status read)
             * instead of the remote's actual command. The skylight saw 92105F
             * fail authentication and told the user to re-pair. Seven attempts
             * in a row, the same challenge db5c0de63013 re-issued each time,
             * the remote's correct answer 67dddc609ad2 rejected each time.
             *
             * It only began happening when the receiver started rotating
             * channels: before that the rig sat on ch20 and rarely heard a
             * challenge meant for a remote. The bug was always here; listening
             * properly is what exposed it.
             *
             * Pairing is unaffected -- see the note on the branch below. */
            if (addressedToUs(receivedPacket)) {
                bootGuardMark(Mark::RxChallenge);
                controlNoteChallenge();  // tell the command worker to stop hopping
                // IVdata is the challenge with commandId put on start
                std::vector<uint8_t> challengeAsked = receivedPacket->data();

                iohcPacket response;
                std::vector<uint8_t> toSend;

                // One snapshot of the command being answered, taken under the
                // lock. Read separately, as this used to, the command byte and
                // its payload could describe two different commands even when
                // nothing crashed -- and the payload was a std::vector another
                // task could be reallocating underneath this copy.
                std::vector<uint8_t> lastPayload;
                const uint8_t lastCommand = iohcLastCommandGet(&lastPayload);

                /* The pairing branch: we asked for a challenge (0x31) and this
                 * is the answer, so what goes back is our key under the
                 * transfer key, not an authentication.
                 *
                 * Still reachable after the guard above, because a challenge
                 * drawn by our own 0x31 is by definition addressed to us -- the
                 * device is replying to a frame we sent from ourAddress. The
                 * guard removes challenges meant for somebody else, which this
                 * branch never wanted either. */
                if (lastCommand == IOHC::iohcDevice::ASK_CHALLENGE_0x31) {
                    response.payload.packet.header.cmd = IOHC::iohcDevice::KEY_TRANSFERT_0x32;
                    std::vector<uint8_t> IVdata = {IOHC::iohcDevice::ASK_CHALLENGE_0x31};
                    toSend = iohcCrypto::encrypt_2W_payload(IVdata, challengeAsked,
                                                            iohcCrypto::transfert_key);
                    for (size_t i = 0; i < toSend.size(); i++)
                        toSend[i] ^= iohcCrypto::transfert_key[i];
                } else {
                    response.payload.packet.header.cmd = IOHC::iohcDevice::CHALLENGE_ANSWER_0x3D;
                    std::vector<uint8_t> IVdata = lastPayload;
                    IVdata.insert(IVdata.begin(), lastCommand);
                    // Authenticate with the system key belonging to whoever is
                    // challenging us -- each remote in this installation is its
                    // own system with its own key, so the right key is a
                    // property of the device, not of the rig.
                    //
                    // The transfert_key fallback is a well-known constant and is
                    // only correct for the key-transfer exchange, where no system
                    // key exists yet; using it for ordinary commands cannot
                    // authenticate anything, which is why the original comment
                    // here found it "unusual".
                    //
                    // Note what this does NOT do: fall back to "the one key we
                    // happen to hold". That guess is right while there is only
                    // one system and silently wrong from the moment there are
                    // two, and a wrong-key answer is indistinguishable on air
                    // from a rig that was never paired. Better to fail loudly
                    // and name the fix.
                    const uint8_t *src = receivedPacket->payload.packet.header.source;
                    const uint8_t *authKey = keystoreKeyForDevice(src);
                    if (!authKey) {
                        ets_printf(
                            "!! no key bound for %02X%02X%02X -- this answer cannot "
                            "authenticate. Bind one: keys bind <slot> %02X%02X%02X\n",
                            src[0], src[1], src[2], src[0], src[1], src[2]);
                        authKey = iohcCrypto::transfert_key;
                    }
                    toSend = iohcCrypto::encrypt_2W_payload(IVdata, challengeAsked, authKey);
                    toSend.resize(6);
                }

                forgePacket(&response, toSend);

                /* Swap */
                memcpy(response.payload.packet.header.source,
                       receivedPacket->payload.packet.header.target, 3);
                memcpy(response.payload.packet.header.target,
                       receivedPacket->payload.packet.header.source, 3);

                response.payload.packet.header.CtrlByte1.asStruct.StartFrame = 0;

                // For remote devices, a longer preamble is needed
                // to allow their AFC (Automatic Frequency Control) to stabilize.
                // Timing is critical. A specific delay is necessary for the gateway to accept the response.
                // The window appears to be between 9 and 12 ms.
                response.repeatTime = 12;

                // This handler never set a frequency, so the reply fell back to
                // scan_freqs[0] -- CHANNEL2 -- and went out on ch20 no matter
                // which channel the challenge arrived on. Measured: the skylight
                // challenged us on ch15 and both 0x3D replies were transmitted on
                // ch20. A unicast reply must return on its request's channel
                // (home_io_control ADR 0028), so an authentication answered on
                // the wrong one cannot be heard however correct its contents.
                response.frequency = receivedPacket->frequency;

                // Key it straight out rather than queueing. sendPriority() hands
                // the frame to the Ticker-driven sender, which measured ~1.5s
                // from challenge to transmission -- against the 9-12ms window
                // this handler's own comment describes. The skylight answered our
                // command, challenged us, and then heard nothing in time: no ack,
                // no error, silence. The queue is fine for frames we originate;
                // an authentication is a hard-real-time reply and cannot go
                // through it.
                {
                    bootGuardMark(Mark::RxChallengeTx);
                    const uint8_t dlen =
                        response.payload.packet.header.CtrlByte1.asStruct.MsgLen + 1 - 9;
                    const uint8_t total = 9 + dlen;
                    Radio::setCarrier(Radio::Carrier::Frequency, response.frequency);
                    Radio::writeFrame(response.payload.buffer, total);
                    ets_printf("0x3D sent immediately (%u bytes) on %luMHz\n", total,
                               static_cast<unsigned long>(response.frequency / 1000000U));
                }
                // Queued copy deliberately not sent as well: a duplicate
                // authentication arriving a second late is worse than none.

                // Restaurer la longueur de préambule par défaut pour les communications suivantes.
            } else {
                /* Somebody else's authentication, in progress. Logged rather
                 * than dropped silently: this is the line that says the rig is
                 * staying out of an exchange it used to break, and a remote
                 * that starts complaining again is answered by whether these
                 * appear. */
                ets_printf("0x3C for %02X%02X%02X from %02X%02X%02X -- not ours, ignored\n",
                           receivedPacket->payload.packet.header.target[0],
                           receivedPacket->payload.packet.header.target[1],
                           receivedPacket->payload.packet.header.target[2],
                           receivedPacket->payload.packet.header.source[0],
                           receivedPacket->payload.packet.header.source[1],
                           receivedPacket->payload.packet.header.source[2]);
            }
            break;
        }
        case iohcDevice::GET_NAME_0x50: {
            // Was cozyDevice2W->isFake(), which returned true when EITHER the
            // source or the target matched us -- so we would answer a frame
            // merely sent FROM our own address. addressedToUs() is the guard
            // every other addressed handler here uses, and it drops the last
            // dependency on the Cozy thermostat emulation outside IOHC_LEGACY.
            if (addressedToUs(receivedPacket)) {
                // MY_GATEWAY 4d595f47415445574159
                std::vector<uint8_t> toSend = {0x4d, 0x59, 0x5f, 0x47, 0x41,
                                               0x54, 0x45, 0x57, 0x41, 0x59};
                toSend.resize(16);

                iohcPacket response;

                forgePacket(&response, toSend);

                response.payload.packet.header.cmd = 0x51;

                /* Swap */
                memcpy(response.payload.packet.header.source, ourAddress, 3);
                memcpy(response.payload.packet.header.target,
                       receivedPacket->payload.packet.header.source, 3);

                response.repeatTime = 50;
                response.frequency = receivedPacket->frequency;

                radioInstance->sendPriority(&response);
            }
            break;
        }
        case iohcDevice::GET_NAME_ANSWER_0x51: {
            // in iohcPacket
            break;
        }
        /* The iohcLastCommandSetCmd calls below are gated on the frame being
         * addressed to us, and that gate is load-bearing.
         *
         * This record is what a 0x3C answer is signed over. Setting it from
         * traffic between two OTHER devices means the rig signs its next
         * authentication over somebody else's command -- and it is how the
         * spurious 0x3D described at CHALLENGE_REQUEST_0x3C came to be signed
         * over "command 00, 3 bytes" when the remote had actually sent six.
         *
         * It also corrupts the rig's own exchanges: a remote pressed during the
         * 1.5 s this rig holds a channel waiting for its own challenge would
         * overwrite the command byte between transmit and answer, and the rig's
         * legitimate authentication would fail for no visible reason. */
        case 0x01:
            if (!receivedPacket->is1W()) {
                if (addressedToUs(receivedPacket)) iohcLastCommandSetCmd(0x01);
                break;
            }
        case 0X00:
            if (!receivedPacket->is1W()) {
                if (addressedToUs(receivedPacket)) iohcLastCommandSetCmd(0x00);
                // Somebody else driving one of our devices -- the whole reason
                // the rig listens between commands. This one is deliberately
                // NOT gated: a command addressed elsewhere is exactly what it
                // exists to notice. It only reads the frame.
                noteExternalCommand(receivedPacket);
            }
        case 0x03: {
            /* A 1W branch here decoded the main parameter into "OPEN"/"CLOSE"/
             * "STOP"/"VENT"/"FORCE" purely to store it in the JsonDocument that
             * MQTT used to publish. Nothing has read that document since the
             * MQTT handler was deleted, so the decode was unreachable work on
             * every received frame. Found by clang-tidy (bugprone-branch-clone
             * flagged the document's own dead branches). */
            /* Same gate, and for a second reason as well. Unguarded, ANY
             * position request the rig overheard -- a remote asking a skylight
             * where it is -- drew this fabricated 0x04 saying 0xC800, sent with
             * the addresses swapped so it went out as the device being asked.
             * A remote would have been told "closed" by an impostor while the
             * real answer was still coming. Now it only answers when something
             * asks the rig itself, which nothing in this installation does. */
            if (!receivedPacket->is1W() && receivedPacket->cmd() == 0x03 &&
                addressedToUs(receivedPacket)) {
                const std::vector<uint8_t> seen = receivedPacket->data();
                iohcLastCommandSet(0x03, seen.data(), seen.size());
                // For the fun....
                iohcPacket response;

                response.payload.packet.header.cmd = 0x04;
                std::vector<uint8_t> toSend = {0x05, 0x00, 0xc8, 0x00, 0xc8, 0x00, 0x00,
                                               0x00, 0x32, 0x99, 0xd8, 0x01, 0x00, 0x00};

                memcpy(toSend.data() + 8, receivedPacket->payload.packet.header.source, 3);
                forgePacket(&response, toSend);

                /* Swap */
                memcpy(response.payload.packet.header.source,
                       receivedPacket->payload.packet.header.target, 3);
                memcpy(response.payload.packet.header.target,
                       receivedPacket->payload.packet.header.source, 3);

                response.payload.packet.header.CtrlByte1.asStruct.StartFrame = 0;
                response.payload.packet.header.CtrlByte1.asStruct.EndFrame = 1;

                response.repeatTime = 10;
                radioInstance->sendSingle(&response);
            }
            break;
        }
        // 0x04 is handled above: it carries the position, and was being
        // discarded here along with the genuinely uninteresting commands.
        case 0x0D:
        case iohcDevice::DISCOVER_ACTUATOR_ACK_0x2D:
        case 0x4B:
        case 0x55:
        // 0x57 is handled above: it carries the device type, which is what tells
        // a skylight from the blind over it. It was being discarded here.
        case 0x59: {
            break;
        }
        case iohcDevice::STATUS_0xFE: {
            // CMD_ERROR_RESP. The single data byte says why the request was
            // refused; naming the common ones saves a trip to the constants
            // table every time a probe comes back empty.
            if (const std::vector<uint8_t> d = receivedPacket->data(); !d.empty()) {
                const char *why;
                switch (d[0]) {
                    case 0x03: why = "manually operated"; break;
                    case 0x07: why = "reached wrong position"; break;
                    case 0x08: why = "error during execution / opcode unsupported"; break;
                    case 0x18: why = "automatic cycle engaged"; break;
                    case 0x21: why = "wrong position"; break;
                    case 0x23: why = "intermediate position not set"; break;
                    case 0x58: why = "request rejected (malformed or unsupported index)"; break;
                    default: why = "unmapped"; break;
                }
                // So the front end can say why a tap did nothing, instead of
                // leaving the device showing "moving" until it gives up.
                registryNoteError(receivedPacket->payload.packet.header.source, d[0]);
                // And so the command's outcome carries the reason rather than
                // reporting a refusal as silence.
                controlNoteError(receivedPacket->payload.packet.header.source, d[0]);
                ets_printf("ERROR_RESP from %02X%02X%02X: 0x%02X -- %s\n",
                           receivedPacket->payload.packet.header.source[0],
                           receivedPacket->payload.packet.header.source[1],
                           receivedPacket->payload.packet.header.source[2], d[0], why);
            }
            break;
        }
        /* 0x30, 0x2E and 0x39 were the 1W key-capture, learning-mode and HMAC
         * handlers. They exist to lift key material off 868 MHz one-way devices;
         * nothing in this installation speaks 1W. */
        case iohcDevice::CHALLENGE_ANSWER_0x3D:  // TODO save
        {
            // This is where we verify if a received challenge answer is valid.

            // 1. Get the encrypted data from the received packet.

            // 2. Decrypt it using the system key.

            // 3. Reconstruct the expected IV based on the last command we sent.
            // We can't know the challenge used by the other remote, so we only construct the first part of the IV.

            break;
        }
        // 0x2A is handled above (opt-in 0x2B answer).
        case 0x48:
        case 0x49:
        case 0x4A:
        case 0X05:
        case 0x19:
        case 0x0C: break;
        // 0x31 and 0x32 were silently ignored here. Both are handled above now:
        // they are steps 2 and 3 of the key handover, and dropping them was why
        // the exchange could never progress.
        case 0x33:
        case 0x46:
        case 0x47:
        case 0x54:
        case 0x56: break;
        default:
            ets_printf("Received Unknown command %02X ", receivedPacket->cmd());
            bootGuardMark(Mark::RxLeave);
            return false;
            break;
    }

    bootGuardMark(Mark::RxLeave);
    return true;
}

void loop() {
    // loopWebServer(); // For ESPAsyncWebServer, this is typically not needed.
}
