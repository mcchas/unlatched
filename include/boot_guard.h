/*
 * Automatic recovery from a firmware image that does not boot.
 *
 * Written after an OTA left the rig boot-looping with no SSID advertised and no
 * ping -- unrecoverable without a USB cable. That is an annoyance on a desk and
 * a ladder on a roof, which is where this rig is going.
 *
 * ESP-IDF has a bootloader-level rollback (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
 * plus esp_ota_mark_app_valid_cancel_rollback), but it is not confirmed enabled
 * in the prebuilt Arduino libraries this project builds against, and a safety
 * net that might not be armed is worse than none. So this does it in software,
 * which works regardless: count boots in NVS, clear the count once the firmware
 * has proved it can run, and switch back to the other OTA slot when the count
 * says we are looping.
 *
 * Health is deliberately "stayed up for a while", NOT "reached WiFi". A router
 * outage would otherwise look like a bad image and roll back a perfectly good
 * one -- repeatedly, since the older image cannot reach a down router either.
 * A boot loop never reaches the timeout; a WiFi problem always does.
 */

/*
 * A note on the breadcrumb below, added after the guard first had to do its job
 * for real.
 *
 * An image rolled back, correctly, and left nothing to say why. The rig is on a
 * different subnet from the workstation and has no cable within reach, so the
 * only channel is the network console -- and the network console's output hook
 * is installed partway through setup(). Anything that crashes before that point
 * is silent by construction, and a crash loop is over long before anyone can
 * connect anyway.
 *
 * So setup() now drops a breadcrumb in NVS before each stage, and the following
 * boot reports how far the previous one got, together with the chip's own reset
 * reason. That turns "it rolled back and we do not know why" into a line of
 * text, which is the difference between diagnosing this and guessing at it.
 */

#ifndef BOOT_GUARD_H
#define BOOT_GUARD_H

#include <cstdint>

/* How far setup() had got. Recorded BEFORE the stage runs, so a crash inside
 * one leaves that stage's name behind rather than the previous one's.
 *
 * APPEND ONLY, and never renumber. These values are written to NVS and read
 * back by the NEXT boot -- which, after an upgrade or a rollback, is a
 * different build of this firmware. Inserting a stage in the middle shifts
 * every value above it, so the previous boot's record is then decoded against
 * the wrong names.
 *
 * That is not hypothetical: adding Mqtt between Control and Running did exactly
 * this, and the upgraded boot announced "the previous boot died in 'mqtt
 * bridge' -- that stage is the suspect" about a firmware that had no such
 * stage and had in fact exited setup() cleanly. On a rig whose only crash
 * evidence is what it writes down about itself, a diagnostic that invents a
 * culprit is worse than one that says nothing.
 *
 * So new stages go at the END, and the declaration order no longer matches the
 * order they run in. stageName() below lists them in boot order instead, which
 * is where reading them in sequence actually helps. */
enum class BootStage : uint8_t {
    Unknown = 0,
    Console,   ///< network console and the output hook
    Nvs,       ///< nvs_init
    Settings,  ///< settingsBegin
    Keystore,  ///< keystoreBegin and the one-time key migration
    Registry,  ///< registryBegin, which reads its table and adopts from the keystore
    Pairing,   ///< pairingBegin
    Wifi,      ///< initWifi
    WebUi,     ///< webUiBegin
    Radio,     ///< iohcRadio::getInstance and start
    Control,   ///< controlBegin
    Running,   ///< setup() returned; anything after this is a runtime fault
    // -- appended after Running; see above. Runs between Control and Running.
    Mqtt,      ///< mqttBegin
};

/// Call FIRST in setup(), before anything that might crash.
///
/// Increments the boot-attempt counter, and if it has reached the limit,
/// switches the boot partition to the other OTA slot and restarts -- so a bad
/// flash undoes itself after a few power cycles instead of needing a cable.
/// Also picks up the previous boot's breadcrumb and the chip's reset reason,
/// for bootGuardReport() to print once there is somewhere to print to.
void bootGuardBegin();

/// Record that setup() is about to enter @p stage.
void bootGuardStage(BootStage stage);

/// Print how the PREVIOUS boot ended. Call as soon as the console output hook
/// is installed -- which is the whole point, since the interesting failures
/// happen before that and cannot report themselves.
void bootGuardReport();

/* Fine-grained breadcrumbs, for faults that happen long after setup().
 *
 * The stage breadcrumb above localises a crash to a phase of startup. It cannot
 * say anything about a fault at runtime, and the panic handler's own output
 * never reaches the network on this board -- the C6 console is USB-JTAG, so
 * wrapping the UART symbol catches the console but not the crash report.
 *
 * So the interesting code paths drop a one-byte marker into RTC memory as they
 * go. It survives the reset, the last few are printed on the boot after, and
 * the cost is a store to RTC RAM. Not a backtrace, but it names the function
 * the rig was in, which is most of what a backtrace would have told us. */
enum class Mark : uint8_t {
    None = 0,
    CmdExecEnter,     ///< the worker picked a command up
    CmdTrainStart,    ///< about to emit the 510 ms wake-up train
    CmdTrainDone,     ///< train finished, frame going out
    CmdWait,          ///< holding the channel for the actuator to answer
    CmdChallengeSeen, ///< the actuator challenged us; stopping the channel hop
    CmdExecLeave,     ///< back to receive, command finished
    RxEnter,          ///< a frame arrived and is being dispatched
    RxChallenge,      ///< inside the 0x3C handler
    RxChallengeTx,    ///< keying the 0x3D answer straight out
    RxPosition,       ///< inside the 0x04 position handler
    RxKeyTransfer,    ///< inside the 0x32 key handler
    RxLeave,          ///< handler returned
    WebState,         ///< building the state document
    WebCommand,       ///< queueing a command from the front end
};

/// Drop a marker. Cheap enough for the receive path.
void bootGuardMark(Mark m);

/// Report what the guard is doing, for the console, including the last few
/// markers from the previous boot.
void bootGuardStatus();

#endif  // BOOT_GUARD_H
