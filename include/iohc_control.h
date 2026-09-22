/*
 * One owner for outbound actuator commands, so that two things can ask for a
 * window to move without fighting over the radio.
 *
 * Sending a command is not quick. Each one is a 510 ms wake-up train followed
 * by the command frame, repeated on up to three channels because we do not know
 * which one the actuator naps on, and then a hold of up to 1.5 s while the
 * actuator challenges us and we authenticate. Worst case is over four seconds
 * of keying the radio.
 *
 * That used to run inline on whichever task called it -- in practice the
 * esp_timer task, because the console is driven from a Ticker. esp_timer's task
 * is also what drives iohcRadio's own scan and transmit timing, so a single
 * `sky close` stalled the radio's timebase for seconds. A front end that can
 * issue six commands with one tap makes that considerably worse.
 *
 * So commands go on a queue and a dedicated task executes them one at a time.
 * The console and the web UI are then just two producers, no command can
 * interleave with another halfway through its wake-up train, and neither caller
 * blocks: a tap on a phone returns immediately and the result appears in the
 * device's state a few seconds later.
 */

#ifndef IOHC_CONTROL_H
#define IOHC_CONTROL_H

#include <cstdint>

enum class IohcAction : uint8_t {
    Open,       ///< main parameter 0x0000
    Close,      ///< 0xC800
    Stop,       ///< 0xD200 -- a marker, not a position
    Vent,       ///< 0xD803
    Position,   ///< percent OPEN, inverted to the wire scale
    Status,     ///< CMD_PRIVATE get position; needs no key
    StatusExt,  ///< the 4-byte extended form real hubs send
    Info,       ///< CMD_GET_INFO2, whose reply carries the device type
};

/// Who asked for this. Carried only so the outcome can say -- an event on a
/// broker saying a window closed is a good deal less alarming when it also says
/// which interface asked for it.
enum class IohcSource : uint8_t {
    Console,
    Web,
    Mqtt,
    Auto,  ///< the rig's own follow-up read, not anybody's request
};

struct IohcCommand {
    uint8_t node[3];
    IohcAction action;
    /// Position: percent OPEN, 0..100. StatusExt: which block to ask for --
    /// real hubs send 00 and 01, and an actuator may answer one and not the
    /// other, so it stays selectable.
    uint8_t percent;
    uint8_t channel;  ///< 15/20/25, or 0 for all three
    bool silent;      ///< slow, quiet travel profile
    IohcSource source;
    /// A routine refresh, not anybody's request. Reports no outcome: a poll
    /// that finds nothing changed has nothing to say, and six devices'
    /// queued/acked pairs every refresh would drown the event stream in
    /// housekeeping. What it learns still reaches the state topics, because
    /// those are driven from the registry rather than from events.
    bool background;
    /// 0 for a command somebody asked for; otherwise the number of this
    /// read-back in the series this module queues itself after a move.
    ///
    /// A follow-up reports only how the move ended: the queued and acked phases
    /// of a read nobody asked for would double the event traffic and say
    /// nothing. It is a count rather than a flag because one read is not enough
    /// to tell "arrived" from "still on its way" -- see the worker.
    uint8_t followUp;
};

/// How a command turned out.
///
/// What is actually knowable here is worth being precise about, because none of
/// it is an acknowledgement in the sense a network protocol would mean:
///
///   Acked      the actuator challenged us (0x3C) and we answered. It heard the
///              command and is authenticating it. This is the strongest signal
///              available at transmit time and it arrives within ~1.5 s.
///   NoResponse nothing answered on any channel tried. Out of range, asleep,
///              wrong channel, or not listening -- indistinguishable from here.
///   Rejected   CMD_ERROR_RESP came back with a reason code. It heard us and
///              declined.
///   NoKey      refused before transmitting: a movement draws a challenge, and
///              without a bound key we cannot answer it. Reported rather than
///              transmitted-and-failed, since the outcome is already known.
///   Moving     a read-back found it part way there. Neither success nor
///              failure yet; another read is queued.
///   Settled    the read-back says it has stopped, and where. This, not Acked,
///              is what says the window actually moved.
enum class IohcOutcome : uint8_t {
    Queued,
    Acked,
    NoResponse,
    Rejected,
    NoKey,
    Settled,
    Moving,
};

struct IohcResult {
    uint8_t node[3];
    IohcAction action;
    IohcOutcome outcome;
    IohcSource source;
    uint8_t percent;    ///< what was asked for, when the action carries one
    uint8_t channel;    ///< the channel it answered on, 0 when none did
    uint8_t errorCode;  ///< CMD_ERROR_RESP code, for Rejected
    int8_t position;    ///< percent open at the time, -1 when unknown
    int8_t target;      ///< percent open it was heading for, -1 when none
    bool reachedTarget; ///< Settled only: it arrived within a few percent
    /// Acked only: whether the answer was a 0x3C challenge or a plain reply.
    /// A movement is authenticated and draws the former; a status read is
    /// unauthenticated and draws a 0x04 straight back. Both mean "it heard us",
    /// so both are Acked -- but describing a status read as having answered a
    /// challenge is the kind of small untruth that sends someone looking for a
    /// key problem that is not there.
    bool challenged;
};

/// Called once per outcome. Registered by the MQTT bridge; null by default, so
/// nothing is paid for when no one is listening.
///
/// IMPORTANT: this runs on the command worker, which is the task that keys the
/// radio and sits ABOVE the packet decoder in priority. It must not block, and
/// in particular must not touch the network stack -- stalling this task stalls
/// the wake-up train. The bridge's handler only copies the result onto its own
/// queue and returns.
using IohcResultFn = void (*)(const IohcResult &);
void controlSetResultHandler(IohcResultFn fn);

const char *controlOutcomeName(IohcOutcome outcome);
const char *controlSourceName(IohcSource source);

/// Start the worker task. Call after the radio is up.
void controlBegin();

/// Queue a command. Returns false only when the queue is full, which means the
/// rig already has more than a minute of radio work ahead of it.
bool controlEnqueue(const IohcCommand &cmd);

/// Queue a command to run later -- used to read a position back once travel
/// has had time to finish. Returns false when the pending table is full.
bool controlEnqueueIn(const IohcCommand &cmd, uint32_t delayMs);

/// Commands waiting, including one in flight. The front end shows this so a tap
/// that will not be acted on for thirty seconds does not look ignored.
uint8_t controlPending();

/// True while a command is on air.
bool controlBusy();

/// Tell the worker an actuator has answered, so it can stop hopping channels.
/// Called from the 0x3C challenge handler in the receive path.
void controlNoteChallenge();

/* The two below are additive: they record what happened for the outcome report
 * and deliberately do NOT influence the channel hop or the hold timing, which
 * controlNoteChallenge() alone drives. That path was arrived at by measurement
 * against the user's own actuators and is not worth perturbing to make an event
 * payload marginally better informed.
 *
 * Both are called from the packet decoder task and do nothing but set a flag. */

/// Any frame from @p node. Notes whether the command currently on air drew a
/// reply at all, which is what separates "not heard" from "heard and refused".
void controlNoteReply(const uint8_t node[3], uint8_t cmd);

/// A CMD_ERROR_RESP arrived: the actuator heard the command and declined it.
void controlNoteError(const uint8_t node[3], uint8_t code);

/// One of our devices was commanded by something that is not this rig -- a
/// handheld remote, a wall switch. Queues a read-back so the state we publish
/// catches up with what somebody just did in the room.
///
/// Also called from the decoder task, so it only writes down the address; the
/// worker does the queueing, which needs a mutex the receive path should not be
/// waiting on.
void controlNoteExternalMove(const uint8_t node[3]);

/// "open", "close", ... -> action. False for anything else.
bool controlParseAction(const char *word, IohcAction *out);

const char *controlActionName(IohcAction action);

/// The 802.15.4 channel the receiver is currently parked on -- 15, 20 or 25.
///
/// Between commands the worker rotates this, because a window operated from a
/// handheld remote is only heard if the rig happens to be listening where the
/// remote is talking. Reported so the rotation can be seen to be happening;
/// a figure that never moves means it has stopped.
uint8_t controlListenChannel();

/// Bytes of stack the worker has never used -- its lowest ever free headroom.
/// The wake-up train and the frame build run on this task, and it was sized by
/// judgement rather than measurement, so the console reports it. 0 means the
/// task does not exist.
uint32_t controlStackHeadroom();

#endif  // IOHC_CONTROL_H
