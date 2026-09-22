/*
   The outbound command queue and its worker. See iohc_control.h for why the
   radio is driven from one task rather than from whoever asked.

   The frame building below was skyCommand() in main.cpp, moved rather than
   rewritten: every comment in it records something that was measured against
   the user's own hardware, and each correction cost a trip to a remote.
 */

#include <iohc_control.h>

#include <Arduino.h>
#include <app_settings.h>
#include <board_config.h>
#include <boot_guard.h>
#include <device_registry.h>
#include <iohc_last_command.h>
#include <iohc_packet.h>
#include <iohc_radio.h>
#include <keystore.h>
#include <pairing.h>
#include <rf231_helpers.h>
#include <cstring>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
}

/// Our node address -- the source of every frame we transmit. main.cpp owns the
/// live copy (see the comment there on why it is owned in one place), loads it
/// from settings at boot and writes changes back; this module only reads it.
/// Declared here in the same way interact.cpp declares it, rather than adding a
/// header for one array.
extern uint8_t ourAddress[3];

namespace {

/// Deep enough for "close everything" on a six-device installation with a
/// position read queued behind each one, and no deeper: a longer queue only
/// stores taps the user made minutes ago and has forgotten about.
constexpr uint8_t QUEUE_DEPTH = 16;

/// Delayed entries, used for the position read that follows a move.
constexpr uint8_t PENDING_MAX = 8;

/* Tracking a move to its end.
 *
 * One read-back is not enough. statusAfterMoveS defaults to 25 s and a skylight
 * takes about that long to travel its full range, so a single read lands while
 * the motor is still running as often as not -- and a device reported as having
 * "stopped short" when it is merely half way there is a failure that did not
 * happen. Measured on this installation: a command to 25% read back 56% at the
 * first attempt, still closing.
 *
 * So a read that finds the device still moving queues another and says so. Only
 * a device that has stopped, or one that has run out of attempts, produces a
 * Settled verdict. */
constexpr uint8_t FOLLOWUP_MAX = 4;
constexpr uint32_t FOLLOWUP_RETRY_MS = 12000;

/* --- listening between commands -------------------------------------------
 *
 * The radio is started with MAX_FREQS 1, so its scan list is one entry --
 * CHANNEL2, ch20 -- and iohcRadio's frequency-hopping timer is commented out.
 * Left alone, the rig therefore hears only ch20, except after a command, which
 * leaves the carrier whereever it last transmitted: execute() below walks
 * ch15/20/25 and stops at whichever one answered.
 *
 * That matters because the reason to listen at all is to notice a window being
 * operated by something that is not us. The remote does not keep to one
 * channel -- an energy scan caught it alternating ch25 with ch15 or ch20, about
 * 500 ms per channel, ch25 carrying roughly half the airtime (see
 * txFreqOverride in iohc_radio.h) -- and an actuator answers on the channel the
 * command reached it on, so its progress reports follow the remote. A rig
 * parked on one channel hears whatever share of that happens to land there.
 *
 * So the listening channel rotates while the rig is idle. Two things about how:
 *
 * It is done HERE, from the command worker, and not from a timer or a task of
 * its own. Radio:: has no lock, and the number of tasks that can reach it is
 * the known hazard in this firmware -- the comment on xTaskCreate below is
 * about an evening lost to exactly that. This task already owns the radio for
 * transmission, already sits above the packet decoder, and is already awake
 * every 200 ms with nothing to do. A fourth caller would have been the change
 * that made the hazard real.
 *
 * And it never moves the carrier mid-frame. Changing channel while the
 * demodulator is holding a preamble or draining a payload loses that frame,
 * and the frames worth having here are precisely the ones that arrive
 * unannounced.
 */
constexpr uint32_t LISTEN_DWELL_MS = 400;

constexpr uint32_t LISTEN_CHANS[3] = {CHANNEL1, CHANNEL2, CHANNEL3};
uint8_t s_listenIdx = 1;  ///< start on CHANNEL2, which is where start() leaves it
uint32_t s_listenSinceMs = 0;

/* --- keeping the published state honest -----------------------------------
 *
 * These actuators volunteer nothing. A position is known only because
 * something asked for it, and the live half of the registry is deliberately
 * not persisted -- a stored position is a guess about a window somebody may
 * have operated by hand in the meantime, and a guess presented as a reading is
 * worse than "unknown".
 *
 * The consequence, which only showed up once there was a front end and a
 * broker to notice it: after every reboot the rig knows where nothing is, and
 * says so for the whole house until each device is individually operated. Five
 * of six sat at unknown.
 *
 * So every device gets read on a slow rotation. One at a time, spread across
 * the interval rather than swept together: each read is a 510 ms wake-up train
 * and up to three channel attempts, so six in a row would hold the radio for
 * half a minute and stop it listening for the remote while it did.
 *
 * The first pass after boot runs faster than the configured interval, because
 * the whole point is to stop the rig publishing "unknown" for a quarter of an
 * hour after it starts. */
constexpr uint32_t POLL_FIRST_DELAY_MS = 20000;  ///< let WiFi and MQTT settle
constexpr uint32_t POLL_SWEEP_GAP_MS = 9000;     ///< between devices, first pass

uint32_t s_pollNextMs = 0;
uint8_t s_pollIdx = 0;
bool s_pollSwept = false;  ///< the first, faster pass has finished

/// Defined below, once s_busy exists.
void listenTick();
void pollTick();

struct Pending {
    IohcCommand cmd;
    uint32_t dueMs;
    bool used;
};

QueueHandle_t s_queue = nullptr;
TaskHandle_t s_worker = nullptr;
Pending s_pending[PENDING_MAX]{};
SemaphoreHandle_t s_pendingLock = nullptr;
volatile bool s_busy = false;

/// Set by the 0x3C handler so the send loop can stop hopping the moment the
/// actuator answers. Without it the loop moved the radio to the next channel
/// while a challenge was still outstanding, and the 0x3D reply went out on the
/// wrong one -- a unicast reply has to return on the channel its request
/// arrived on (home_io_control ADR 0028).
volatile bool s_challengeSeen = false;

/* Outcome reporting. These three are written by the decoder task and read by
 * this one at the end of execute(); they are only ever cleared here, before a
 * command goes out, so a late frame from a previous exchange cannot be read as
 * an answer to the current one.
 *
 * Correlation is by time rather than by bookkeeping: execute() holds the
 * channel for up to 1.5 s after the challenge, which is where an actuator's
 * reply or refusal lands. Keeping a per-node record of outstanding commands
 * would buy accuracy in a case -- two commands to the same node overlapping --
 * that the single-worker queue makes impossible. */
volatile bool s_replySeen = false;
volatile bool s_errorSeen = false;
volatile uint8_t s_errorCode = 0;
/// The node execute() is currently talking to, so the decoder's notes can be
/// ignored when they are about somebody else's device.
volatile uint8_t s_inFlight[3] = {0, 0, 0};
volatile bool s_inFlightValid = false;

void listenTick() {
    if (s_busy) return;  // a command owns the radio

    /* Not while a key handover is armed. That exchange is a conversation with a
     * handheld remote, several frames each way, and every attempt at it costs
     * somebody a walk to the remote and a button press. It has been made to
     * work with the receiver pinned to CHANNEL2 -- four remotes were paired
     * that way -- so it keeps exactly the behaviour it was tuned against
     * rather than being asked to also survive a moving carrier. */
    if (PairingStatus ps{}; pairingStatus(&ps) && ps.armed) {
        // Park it there, rather than merely stopping wherever the rotation had
        // got to -- otherwise "pinned to CHANNEL2" would be true only by luck.
        if (s_listenIdx != 1) {
            s_listenIdx = 1;
            Radio::setCarrier(Radio::Carrier::Frequency, LISTEN_CHANS[1]);
        }
        return;
    }

    const uint32_t now = millis();
    if (now - s_listenSinceMs < LISTEN_DWELL_MS) return;

    using RS = IOHC::iohcRadio::RadioState;
    if (const RS st = IOHC::iohcRadio::radioState; st != RS::RX && st != RS::IDLE) return;

    s_listenSinceMs = now;
    s_listenIdx = static_cast<uint8_t>((s_listenIdx + 1) % 3);
    // One register write; the PLL re-locks in ~150 us and the receiver stays
    // on, so this does not need a setRx() behind it.
    Radio::setCarrier(Radio::Carrier::Frequency, LISTEN_CHANS[s_listenIdx]);
}

void pollTick() {
    const uint16_t interval = settings().pollIntervalS;
    if (!interval) return;
    if (s_busy) return;
    // Never on top of real work. A refresh is the least urgent thing this task
    // does, and queueing one behind somebody's tap would delay the tap.
    if (s_queue && uxQueueMessagesWaiting(s_queue)) return;

    const uint32_t now = millis();
    if (static_cast<int32_t>(now - s_pollNextMs) < 0) return;

    const uint8_t count = registryCount();
    if (!count) {
        s_pollNextMs = now + 10000;  // nothing to read yet; ask again shortly
        return;
    }
    if (s_pollIdx >= count) {
        s_pollIdx = 0;
        s_pollSwept = true;  // everything has been read at least once
    }

    RegistryDevice d{};
    if (!registryGet(s_pollIdx, &d)) {
        s_pollIdx++;
        return;
    }
    s_pollIdx++;

    IohcCommand poll{};
    memcpy(poll.node, d.node, 3);
    poll.action = IohcAction::Status;
    poll.source = IohcSource::Auto;
    poll.background = true;
    poll.channel = settings().commandChannel;
    controlEnqueue(poll);

    // Spread the reads evenly, so the radio is idle and listening between
    // them rather than busy for half a minute and then silent.
    const uint32_t spread = (static_cast<uint32_t>(interval) * 1000UL) / count;
    s_pollNextMs = now + (s_pollSwept ? (spread < 2000 ? 2000 : spread) : POLL_SWEEP_GAP_MS);
}

IohcResultFn s_resultFn = nullptr;

/* Devices somebody else has just told to move, noted by the decoder and acted
 * on by the worker.
 *
 * Handing the address over rather than queueing the read directly keeps the
 * receive path free of the pending-table mutex. The chip is single core, so
 * writing the address before raising the flag is enough for the worker to
 * never see one without the other. */
constexpr uint8_t EXTERNAL_MAX = 8;
struct ExternalMove {
    uint8_t node[3];
    volatile bool pending;
};
ExternalMove s_external[EXTERNAL_MAX]{};

void report(const IohcCommand &c, const IohcOutcome outcome, const uint8_t channel,
            const uint8_t errorCode, const bool challenged = false) {
    if (!s_resultFn) return;
    IohcResult r{};
    memcpy(r.node, c.node, 3);
    r.action = c.action;
    r.outcome = outcome;
    r.source = c.source;
    r.percent = c.percent;
    r.channel = channel;
    r.errorCode = errorCode;
    r.challenged = challenged;
    r.position = -1;
    r.target = -1;
    RegistryDevice d{};
    if (registryGetByNode(c.node, &d)) {
        r.position = d.position;
        r.target = d.target;
        // Within three percent counts as arrived. The actuators report their own
        // position to the same scale they accept one on, but a slat blind asked
        // for 50 will happily report 49, and calling that a failure would make
        // every second command look broken.
        r.reachedTarget = d.position >= 0 && d.target >= 0 && abs(d.position - d.target) <= 3;
    }
    s_resultFn(r);
}

/// As the remote sends it; 0x43/0x67 in the 868 MHz stack.
constexpr uint8_t ACEI = 0x63;

/// The main parameter is a 16-bit POSITION, not a set of flags: 0x0000 = fully
/// open, 0xC800 (51200) = fully closed. Values above full scale are commands
/// rather than positions.
constexpr uint16_t POS_CLOSED = 0xC800;

/// Percent open for an action, or -1 when it does not end at a percentage.
/// The registry uses this to show where a device is heading.
int8_t targetPercent(const IohcCommand &c) {
    switch (c.action) {
        case IohcAction::Open: return 100;
        case IohcAction::Close: return 0;
        case IohcAction::Vent: return -1;
        case IohcAction::Position: return static_cast<int8_t>(c.percent > 100 ? 100 : c.percent);
        default: return -1;
    }
}

bool isMove(const IohcAction a) {
    return a == IohcAction::Open || a == IohcAction::Close || a == IohcAction::Vent ||
           a == IohcAction::Position;
}

void execute(const IohcCommand &c) {
    bootGuardMark(Mark::CmdExecEnter);
    const AppSettings &cfg = settings();
    const bool statusExt = c.action == IohcAction::StatusExt;
    const bool statusOnly = c.action == IohcAction::Status || statusExt;
    const bool infoOnly = c.action == IohcAction::Info;

    uint8_t mainHi = 0, mainLo = 0;
    switch (c.action) {
        case IohcAction::Open:
            mainHi = 0x00;
            mainLo = 0x00;
            break;
        case IohcAction::Close:
            mainHi = 0xC8;
            mainLo = 0x00;
            break;
        case IohcAction::Stop:
            mainHi = 0xD2;
            mainLo = 0x00;
            break;
        // 0xD803, not 0xD800 -- iohcRemote1W.cpp:333. The earlier 0xD800 was a
        // guess and was never sent to a device.
        case IohcAction::Vent:
            mainHi = 0xD8;
            mainLo = 0x03;
            break;
        case IohcAction::Position: {
            // Percent OPEN, which is the way a cover is spoken about. The wire
            // value counts the other way, so invert.
            const uint8_t pct = c.percent > 100 ? 100 : c.percent;
            const uint16_t raw = static_cast<uint16_t>((POS_CLOSED * (100 - pct)) / 100);
            mainHi = raw >> 8;
            mainLo = raw & 0xFF;
            ets_printf("sky pos %u%% open -> raw 0x%04X (%u/%u closed)\n", pct, raw, raw,
                       POS_CLOSED);
            break;
        }
        default: break;
    }

    const uint8_t *me = ourAddress;
    uint8_t f[20];
    uint8_t dlen;
    if (infoOnly) {
        f[8] = 0x56;  // CMD_GET_INFO2 -- no payload
        dlen = 0;
    } else if (statusOnly) {
        // CMD_PRIVATE (0x03) at function id PRIVATE_GET_POSITION_STATUS (0x03).
        // Documented as needing no authentication, so unlike a movement command
        // this should complete without a challenge round.
        //
        // The payload is THREE bytes, not two. The first attempt sent only
        // {fn, sub} and the actuator answered CMD_ERROR_RESP with result 0x58 --
        // a code home_io_control records as "index rejected" and which no error
        // table maps. A short payload leaves the device reading the CRC as its
        // third data byte, which is exactly the kind of thing that draws a
        // rejection rather than an answer. proto_commands.cpp:458 builds it as
        // {function_id, sub_index, 0x00}.
        f[8] = 0x03;
        f[9] = 0x03;  // function id: get position status
        if (statusExt) {
            // Extended form: selector 0x80 then the block index. Field-observed
            // as 03 80 00 00 / 03 80 01 00 from real hubs to real motors.
            f[10] = 0x80;
            f[11] = c.percent;  // block index; see IohcCommand::percent
            f[12] = 0x00;
            dlen = 4;
        } else {
            f[10] = 0x00;  // sub-index
            f[11] = 0x00;  // pad -- required; see above
            dlen = 3;
        }
    } else {
        f[8] = 0x00;  // cmd: activate / set position
        f[9] = 0x01;  // originator: user
        f[10] = ACEI;
        f[11] = mainHi;  // main parameter: 0x0000 open, 0xC800 close, 0xD200 stop
        f[12] = mainLo;
        f[13] = 0x00;  // fp1
        f[14] = 0x00;  // fp2
        if (c.silent) {
            // Slow, quiet travel profile. home_io_control isolated this by A/B
            // capture (proto_commands_test.cpp:599): a hub with silent ON sends
            //     01 67 00 00 80 D8 05 00
            // and with it OFF the same bytes ending 80 D8 06 00 -- only the
            // profile byte differs, 0x05 vs 0x06. Lined up against our own
            // frames that payload is exactly this one with the two functional
            // parameters set to 80 D8 and the profile appended, which also fits
            // 0x80 being the "extended block follows" selector seen in the
            // status replies.
            //
            // That reference capture is from a Somfy hub, so the layout was
            // inference when it was first written -- but it was CONFIRMED on
            // this installation straight afterwards, and this comment outlived
            // the doubt by longer than it should have. The stairs skylight
            // accepted
            //     00  01 63 00 00 80 D8 05 00
            // challenged us, took the answer and opened at the slower speed,
            // with only the ACEI byte differing from the reference (0x63, our
            // remote's own value, against 0x67). The extended execute layout is
            // not Somfy-specific. See commit 0b34ddf.
            f[13] = 0x80;  // fp1: extended block follows
            f[14] = 0xD8;  // fp2: POS_FAVORITE, as the reference capture carries
            f[15] = 0x05;  // travel profile: silent (0x06 = normal)
            f[16] = 0x00;
        }
        dlen = c.silent ? 8 : 6;
    }
    f[0] = static_cast<uint8_t>(0x40 | (8 + dlen));  // 2W, Start=1, End=0, MsgLen
    f[1] = 0x20;                                     // CtrlByte2: LPM -- the actuator is low power
    memcpy(&f[2], c.node, 3);
    memcpy(&f[5], me, 3);
    const uint8_t flen = 9 + dlen;

    // The 0x3C handler signs whatever IOHC::lastCmd/lastData hold. This path
    // bypasses the packet queue that normally sets those, so without this it
    // authenticated the PREVIOUS command -- the first attempt logged "Challenge
    // asked after Last Command FF", i.e. it signed nothing at all. The IV is
    // [cmd] + payload.
    iohcLastCommandSet(f[8], &f[9], dlen);

    // A status read needs no key (CMD_PRIVATE is unauthenticated), but anything
    // that moves will draw a challenge we cannot answer without one. Say so
    // before transmitting rather than leaving a silent non-response to explain.
    // Deliberately still transmitted. Sending anyway is the existing behaviour
    // and worth keeping: a key can be present and merely unbound, and refusing
    // here on the strength of that lookup would break a working installation to
    // save four seconds. The flag instead sharpens the outcome below, so a
    // failure says "no key bound" rather than the unhelpfully generic
    // "nothing answered".
    const bool keyMissing =
        !statusOnly && !infoOnly && keystoreKeyForDevice(c.node) == nullptr;
    if (keyMissing)
        ets_printf(
            "!! no key bound for %02X%02X%02X -- the challenge will fail. "
            "`keys list` to see what is stored\n",
            c.node[0], c.node[1], c.node[2]);

    const uint32_t chans[3] = {CHANNEL1, CHANNEL2, CHANNEL3};
    const uint16_t trainMs = cfg.trainMs;
    ets_printf("sky %s -> %02X%02X%02X from %02X%02X%02X, train %ums\n",
               controlActionName(c.action), c.node[0], c.node[1], c.node[2], me[0], me[1], me[2],
               trainMs);

    if (isMove(c.action) || c.action == IohcAction::Stop)
        registryNoteCommand(c.node, targetPercent(c));

    // Open the window in which the decoder's notes count as answers to THIS
    // command. Cleared again before returning.
    s_replySeen = false;
    s_errorSeen = false;
    s_errorCode = 0;
    memcpy(const_cast<uint8_t *>(s_inFlight), c.node, 3);
    s_inFlightValid = true;

    uint8_t answeredOn = 0;
    for (uint8_t i = 0; i < 3; i++) {
        const uint8_t chNum = 15 + 5 * i;  // CHANNEL1/2/3 == ch15/20/25
        if (c.channel && c.channel != chNum) continue;
        s_challengeSeen = false;
        Radio::setCarrier(Radio::Carrier::Frequency, chans[i]);
        bootGuardMark(Mark::CmdTrainStart);
        Radio::sendWakeupTrain(trainMs);
        bootGuardMark(Mark::CmdTrainDone);
        Radio::writeFrame(f, flen);
        ets_printf("  sent on ch%u\n", chNum);
        bootGuardMark(Mark::CmdWait);

        // Hold this channel until the actuator has had its say. Moving on early
        // is what sent an earlier run's 0x3D replies out on the wrong channel.
        for (uint8_t t = 0; t < 30 && !s_challengeSeen; t++) vTaskDelay(pdMS_TO_TICKS(50));
        if (s_challengeSeen) {
            bootGuardMark(Mark::CmdChallengeSeen);
            ets_printf("  challenge answered on ch%u -- stopping here\n", chNum);
            answeredOn = chNum;
            vTaskDelay(pdMS_TO_TICKS(1500));  // let the exchange finish
            break;
        }
        // A status read draws a 0x04 rather than a challenge, so it never sets
        // s_challengeSeen and always walks all three channels. Note which one
        // actually answered so the outcome can say, but do not break the loop
        // on it: the hop and hold timings are load-bearing and are not being
        // changed to improve an event payload.
        if (s_replySeen && !answeredOn) answeredOn = chNum;
    }
    Radio::setRx();

    /* Stay where the conversation happened. The actuator sends its progress
     * reports back on the channel the command reached it on, and those 0x04
     * frames are the only account anyone gets of a window actually travelling.
     * Resuming the rotation from here means the next dwell is on the channel
     * most likely to carry them, rather than a third of the way round the
     * cycle from it. */
    if (answeredOn) {
        s_listenIdx = static_cast<uint8_t>((answeredOn - 15) / 5);
        s_listenSinceMs = millis();
    }

    /* The verdict, in order of how much it tells you.
     *
     * Rejected outranks Acked: an actuator that challenges us and then refuses
     * has done both, and "it said no" is the useful half. */
    IohcOutcome outcome;
    if (s_errorSeen) {
        outcome = IohcOutcome::Rejected;
    } else if (s_challengeSeen || s_replySeen) {
        outcome = IohcOutcome::Acked;
    } else {
        outcome = keyMissing ? IohcOutcome::NoKey : IohcOutcome::NoResponse;
    }
    const uint8_t code = s_errorCode;
    const bool challenged = s_challengeSeen;
    s_inFlightValid = false;

    // The follow-up read reports only its Settled outcome, which the worker
    // emits once it has the position; an Acked for a read nobody asked for is
    // noise on whatever is subscribed. A background refresh says nothing at
    // all -- see IohcCommand::background.
    if (!c.followUp && !c.background) report(c, outcome, answeredOn, code, challenged);

    bootGuardMark(Mark::CmdExecLeave);
}

void worker(void *) {
    for (;;) {
        // Anything whose delay has elapsed goes on the queue. Checked before the
        // blocking receive so a due read is not held back by an idle wait.
        const uint32_t now = millis();
        if (s_pendingLock && xSemaphoreTake(s_pendingLock, 0) == pdTRUE) {
            for (auto &p : s_pending) {
                if (!p.used) continue;
                // Unsigned compare so this still fires across the 49-day wrap.
                if (now - p.dueMs > 0x80000000UL) continue;
                if (xQueueSend(s_queue, &p.cmd, 0) == pdTRUE) p.used = false;
            }
            xSemaphoreGive(s_pendingLock);
        }

        // Somewhere unhurried to do the registry's deferred NVS writes: this
        // task has 6 KB of stack and nothing waiting on it between commands,
        // where the receive path has neither.
        registryFlush();

        /* Somebody operated one of our devices from a remote. The actuator
         * broadcasts its progress while it travels and we pick those up
         * passively, but only on whichever channel we happen to be dwelling
         * on, and there is no guarantee the last one lands while we are
         * listening. So confirm where it finished.
         *
         * The delay is the same read-back the rig gives its own moves, which
         * is sized for a full traverse. */
        for (auto &e : s_external) {
            if (!e.pending) continue;
            IohcCommand read{};
            memcpy(read.node, e.node, 3);
            e.pending = false;
            read.action = IohcAction::Status;
            read.source = IohcSource::Auto;
            read.followUp = 1;
            const uint16_t after = settings().statusAfterMoveS;
            controlEnqueueIn(read, static_cast<uint32_t>(after ? after : 25) * 1000UL);
        }

        IohcCommand c{};
        if (xQueueReceive(s_queue, &c, pdMS_TO_TICKS(200)) != pdTRUE) {
            // Nothing to send: move the ear along, and take the chance to
            // refresh a device nobody has asked about lately.
            listenTick();
            pollTick();
            continue;
        }

        s_busy = true;
        execute(c);
        s_busy = false;

        /* This was a read-back queued after a move, so it now knows the thing
         * the move itself could not: whether the device got there. An
         * acknowledgement says a command was heard; this says the window
         * moved.
         *
         * The device reports its own current and target, so it says for itself
         * whether it has arrived -- better than timing the travel and hoping.
         * While it is still going, keep asking. */
        if (c.followUp) {
            RegistryDevice d{};
            const bool known = registryGetByNode(c.node, &d);
            if (known && d.moving && c.followUp < FOLLOWUP_MAX) {
                report(c, IohcOutcome::Moving, 0, 0);
                IohcCommand again = c;
                again.followUp = static_cast<uint8_t>(c.followUp + 1);
                controlEnqueueIn(again, FOLLOWUP_RETRY_MS);
            } else {
                report(c, IohcOutcome::Settled, 0, 0);
            }
        }

        /* A routine refresh that caught a device on the move -- somebody at the
         * window, or a timer on the remote. Hand it to the tracking read, so
         * the state does not sit at "moving" until the next sweep comes round
         * a quarter of an hour later. That read reports its Settled outcome,
         * which is how an out-of-band move gets an event of its own. */
        if (c.background) {
            if (RegistryDevice d{}; registryGetByNode(c.node, &d) && d.moving) {
                IohcCommand track = c;
                track.background = false;
                track.followUp = 1;
                controlEnqueueIn(track, FOLLOWUP_RETRY_MS);
            }
        }

        // Read the position back once travel has had time to finish. These
        // actuators report nothing unless asked, so without this the front end
        // would show the position the user requested forever, whether or not
        // the window ever got there.
        if (isMove(c.action)) {
            const uint16_t after = settings().statusAfterMoveS;
            if (after) {
                IohcCommand read{};
                memcpy(read.node, c.node, 3);
                read.action = IohcAction::Status;
                read.channel = c.channel;
                read.source = c.source;  // carried through, so the event says who started this
                read.followUp = 1;
                controlEnqueueIn(read, static_cast<uint32_t>(after) * 1000UL);
            }
        }
    }
}

}  // namespace

void controlBegin() {
    if (s_queue) return;
    s_queue = xQueueCreate(QUEUE_DEPTH, sizeof(IohcCommand));
    s_pendingLock = xSemaphoreCreateMutex();
    if (!s_queue || !s_pendingLock) {
        ets_printf("!! control queue could not be created -- commands will not be sent\n");
        return;
    }
    /* Priority 6, ABOVE the packet decoder's 5. This is not a preference, it is
     * a correctness requirement, and getting it wrong cost most of an evening.
     *
     * sendWakeupTrain() busy-waits for 510 ms, emitting a frame every
     * millisecond, and it cannot yield: a vTaskDelay would quantise to the 1 ms
     * tick and stretch the train past what the actuator will sit through. For
     * that whole half second the radio is in back-to-back SPI transactions.
     *
     * This code used to live in skyCommand(), so it ran on the esp_timer task,
     * which sits near the top of the priority range -- nothing preempted the
     * train. Moving it to a task at priority 2 changed that: the decoder task
     * at priority 5 could interrupt the train to answer a 0x3C challenge, and
     * that handler deliberately bypasses the packet queue to key the radio
     * directly inside a 9-12 ms window. Two tasks in an SPI transaction on one
     * device, which the driver asserts on -- and an assert is an abort, which
     * surfaces as an unexplained panic during exactly the commands the front
     * end had just made easy to issue.
     *
     * Sitting above the decoder restores what was true before: the train runs
     * to completion, and the decoder gets its turn during the vTaskDelay loop
     * that follows, which is where the challenge actually needs answering.
     *
     * The underlying problem -- Radio:: has no lock of its own and three tasks
     * can reach it -- is contained by priority here rather than prevented.
     * Worth knowing before anything else is given a reason to transmit.
     *
     * 6 KB of stack: execute() builds a 20-byte frame of its own but also
     * reaches keystore, registry and NVS code. `bootstat` prints the measured
     * headroom. */
    xTaskCreate(worker, "iohc-cmd", 6144, nullptr, 6, &s_worker);
    // Not immediately: WiFi, the web server and the broker all come up in the
    // first few seconds, and a wake-up train competing with that is a poor
    // trade for a reading nobody is connected to see yet.
    s_pollNextMs = millis() + POLL_FIRST_DELAY_MS;
}

uint8_t controlListenChannel() {
    return static_cast<uint8_t>(15 + 5 * s_listenIdx);
}

uint32_t controlStackHeadroom() {
    return s_worker ? uxTaskGetStackHighWaterMark(s_worker) : 0;
}

void controlSetResultHandler(const IohcResultFn fn) {
    s_resultFn = fn;
}

bool controlEnqueue(const IohcCommand &cmd) {
    if (!s_queue) return false;
    if (xQueueSend(s_queue, &cmd, 0) != pdTRUE) return false;
    // Worth its own event: a command can sit behind four seconds of radio work
    // per entry ahead of it, so "accepted" and "done" can be most of a minute
    // apart. Anything driving this needs to be able to tell the difference
    // between a request that was dropped and one that is merely waiting.
    if (!cmd.followUp && !cmd.background) report(cmd, IohcOutcome::Queued, 0, 0);
    return true;
}

bool controlEnqueueIn(const IohcCommand &cmd, const uint32_t delayMs) {
    if (!s_pendingLock) return false;
    if (xSemaphoreTake(s_pendingLock, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    bool placed = false;
    for (auto &p : s_pending) {
        // One outstanding follow-up per device: a second move replaces the read
        // queued by the first, which would otherwise arrive mid-travel and
        // report a position nobody asked about.
        if (p.used && memcmp(p.cmd.node, cmd.node, 3) == 0 && p.cmd.action == cmd.action)
            p.used = false;
    }
    for (auto &p : s_pending) {
        if (p.used) continue;
        p.cmd = cmd;
        p.dueMs = millis() + delayMs;
        p.used = true;
        placed = true;
        break;
    }
    xSemaphoreGive(s_pendingLock);
    return placed;
}

uint8_t controlPending() {
    uint8_t n = s_busy ? 1 : 0;
    if (s_queue) n += static_cast<uint8_t>(uxQueueMessagesWaiting(s_queue));
    return n;
}

bool controlBusy() {
    return s_busy;
}

void controlNoteChallenge() {
    s_challengeSeen = true;
}

void controlNoteReply(const uint8_t node[3], uint8_t) {
    if (!s_inFlightValid) return;
    if (memcmp(const_cast<const uint8_t *>(s_inFlight), node, 3) != 0) return;
    s_replySeen = true;
}

void controlNoteExternalMove(const uint8_t node[3]) {
    // Already waiting for this one: a remote sends its command on more than one
    // channel, so the same move arrives two or three times.
    for (auto &e : s_external)
        if (e.pending && memcmp(e.node, node, 3) == 0) return;
    for (auto &e : s_external) {
        if (e.pending) continue;
        memcpy(e.node, node, 3);
        e.pending = true;  // after the address, never before
        return;
    }
}

void controlNoteError(const uint8_t node[3], const uint8_t code) {
    if (!s_inFlightValid) return;
    if (memcmp(const_cast<const uint8_t *>(s_inFlight), node, 3) != 0) return;
    s_errorSeen = true;
    s_errorCode = code;
}

const char *controlOutcomeName(const IohcOutcome outcome) {
    switch (outcome) {
        case IohcOutcome::Queued: return "queued";
        case IohcOutcome::Acked: return "acked";
        case IohcOutcome::NoResponse: return "no-response";
        case IohcOutcome::Rejected: return "rejected";
        case IohcOutcome::NoKey: return "no-key";
        case IohcOutcome::Settled: return "settled";
        case IohcOutcome::Moving: return "moving";
        default: return "?";
    }
}

const char *controlSourceName(const IohcSource source) {
    switch (source) {
        case IohcSource::Console: return "console";
        case IohcSource::Web: return "web";
        case IohcSource::Mqtt: return "mqtt";
        case IohcSource::Auto: return "auto";
        default: return "?";
    }
}

bool controlParseAction(const char *word, IohcAction *out) {
    if (!word || !out) return false;
    if (!strcmp(word, "open")) {
        *out = IohcAction::Open;
    } else if (!strcmp(word, "close")) {
        *out = IohcAction::Close;
    } else if (!strcmp(word, "stop")) {
        *out = IohcAction::Stop;
    } else if (!strcmp(word, "vent")) {
        *out = IohcAction::Vent;
    } else if (!strcmp(word, "pos") || !strcmp(word, "position")) {
        *out = IohcAction::Position;
    } else if (!strcmp(word, "status")) {
        *out = IohcAction::Status;
    } else if (!strcmp(word, "statusx")) {
        *out = IohcAction::StatusExt;
    } else if (!strcmp(word, "info")) {
        *out = IohcAction::Info;
    } else {
        return false;
    }
    return true;
}

const char *controlActionName(const IohcAction action) {
    switch (action) {
        case IohcAction::Open: return "open";
        case IohcAction::Close: return "close";
        case IohcAction::Stop: return "stop";
        case IohcAction::Vent: return "vent";
        case IohcAction::Position: return "pos";
        case IohcAction::Status: return "status";
        case IohcAction::StatusExt: return "statusx";
        case IohcAction::Info: return "info";
        default: return "?";
    }
}
