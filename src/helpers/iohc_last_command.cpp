/*
   The last command sent, shared between the tasks that write it and the task
   that answers a challenge with it. See iohc_last_command.h for the race this
   replaces.
 */

#include <iohc_last_command.h>

#include <Arduino.h>
#include <iohc_packet.h>
#include <cstring>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
}

namespace {

/// A fixed buffer, not a vector. The point of this module is that the shared
/// state never allocates: a reader can then never be looking at memory a writer
/// has just handed back to the heap, whatever the two tasks do to each other.
/// MAX_FRAME_LEN bounds any io-homecontrol payload.
uint8_t s_data[MAX_FRAME_LEN];
size_t s_len = 0;
uint8_t s_cmd = 0xFF;

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

}  // namespace

void iohcLastCommandBegin() {
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
}

void iohcLastCommandSet(const uint8_t cmd, const uint8_t *data, size_t len) {
    if (len > sizeof(s_data)) len = sizeof(s_data);
    Guard g;
    s_cmd = cmd;
    s_len = len;
    if (data && len) memcpy(s_data, data, len);
}

void iohcLastCommandSetCmd(const uint8_t cmd) {
    Guard g;
    s_cmd = cmd;
}

uint8_t iohcLastCommandGet(std::vector<uint8_t> *data) {
    // Copied to the stack under the lock, and only turned into the caller's
    // vector afterwards: allocating while holding a lock that the receive path
    // waits on would put the heap in the middle of a hard real-time reply.
    uint8_t copy[MAX_FRAME_LEN];
    size_t len;
    uint8_t cmd;
    {
        Guard g;
        cmd = s_cmd;
        len = s_len;
        if (len) memcpy(copy, s_data, len);
    }
    if (data) data->assign(copy, copy + len);
    return cmd;
}

size_t iohcLastCommandLen() {
    Guard g;
    return s_len;
}
