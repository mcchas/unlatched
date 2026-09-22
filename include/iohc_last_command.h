/*
 * The command a challenge gets answered against, shared safely between tasks.
 *
 * A 2W actuator answers a movement command with a challenge (0x3C), and the
 * reply has to be computed over the command that provoked it. So the sender
 * writes down what it just sent, and the receive path reads it back a few
 * milliseconds later. Two different tasks, one piece of state.
 *
 * That state used to be two globals in iohc_packet.h -- a size_t and a
 * std::vector<uint8_t> -- touched with no synchronisation whatsoever by three
 * tasks:
 *
 *   writes  iohc_control.cpp   the command worker, before transmitting
 *           iohc_radio.cpp     packetSender(), on the esp_timer task
 *           main.cpp           the 0x03 handler, on the packet decoder task
 *   reads   main.cpp           the 0x3C handler, on the packet decoder task
 *           iohc_packet_decoder.cpp   the trace, same task
 *
 * Assigning a std::vector frees its buffer and allocates a new one. A reader
 * copy-constructing from it at that moment walks a pointer that has just been
 * handed back to the heap. On this rig that presented as a panic during a
 * command exchange -- which is exactly when both sides are busiest, and which
 * a front end that can queue six commands with one tap hits far more often
 * than a person typing `sky close` into a console ever did.
 *
 * So the payload is a fixed buffer behind a mutex rather than a vector. No
 * allocation on either side, nothing to free under a reader, and the command
 * byte and its payload are now read as one consistent pair instead of
 * separately -- they could previously come from two different commands even
 * when nothing crashed.
 */

#ifndef IOHC_LAST_COMMAND_H
#define IOHC_LAST_COMMAND_H

#include <cstddef>
#include <cstdint>
#include <vector>

/// Create the lock. Call once in setup(), before the radio starts.
void iohcLastCommandBegin();

/// Record a command and its payload.
void iohcLastCommandSet(uint8_t cmd, const uint8_t *data, size_t len);

/// Record a command whose payload is not relevant, leaving the stored payload
/// alone -- which is what the call sites that only set a command id expect.
void iohcLastCommandSetCmd(uint8_t cmd);

/// The command byte, with its payload copied into @p data when non-null. Taken
/// together under the lock, so the two always describe the same command.
uint8_t iohcLastCommandGet(std::vector<uint8_t> *data);

/// Payload length, for callers that only want to print it.
size_t iohcLastCommandLen();

#endif  // IOHC_LAST_COMMAND_H
