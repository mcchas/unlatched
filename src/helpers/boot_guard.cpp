/*
   Software OTA rollback. See boot_guard.h for why this is not the IDF's
   bootloader rollback.
 */

#include <boot_guard.h>

#include <Arduino.h>
#include <Preferences.h>
#include <net_console.h>

#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

/// Consecutive boots that failed to reach the health timeout before we give up
/// on this image. Three rather than one: a single crash can come from something
/// transient (a brownout mid-write, a one-off), and rolling back on the first
/// one would undo a good flash for a bad reason.
constexpr uint8_t FAIL_LIMIT = 3;

/// How long the firmware must stay up to count as healthy. Comfortably longer
/// than boot plus WiFi association, and far longer than any crash loop, which
/// restarts in seconds.
constexpr uint32_t HEALTHY_AFTER_MS = 60000;

constexpr const char *NS = "bootguard";
constexpr const char *KEY_FAILS = "fails";
constexpr const char *KEY_STAGE = "stage";
constexpr const char *KEY_HIST = "hist";

/// How many previous boots to keep. A rollback takes three failures, so four
/// covers a whole rollback episode plus the boot that preceded it.
///
/// This exists because the live report is unreadable in exactly the case that
/// matters. A crash loop is over in seconds; WiFi takes longer than that to
/// associate, so by the time anything can connect and watch, the rig has given
/// up and rolled back to an image that knows nothing about any of it. Writing
/// the verdict down means the next flash can still answer the question.
constexpr uint8_t HIST_LEN = 4;

struct HistEntry {
    uint8_t stage;   ///< BootStage the boot was in when it stopped
    uint8_t reason;  ///< esp_reset_reason_t of the boot that FOLLOWED it
};

HistEntry hist[HIST_LEN]{};

/// The last few places the previous boot was, newest last. RTC memory, so a
/// panic does not take them with it.
constexpr uint8_t MARK_LEN = 12;
constexpr uint32_t MARK_MAGIC = 0x4D41524B;  // 'MARK'
RTC_NOINIT_ATTR uint8_t s_marks[MARK_LEN];
RTC_NOINIT_ATTR uint32_t s_markPos;
RTC_NOINIT_ATTR uint32_t s_markMagic;

/// Copied out before this boot starts overwriting them.
uint8_t prevMarks[MARK_LEN];
uint8_t prevMarkCount = 0;

const char *markName(const uint8_t m) {
    switch (static_cast<Mark>(m)) {
        case Mark::CmdExecEnter: return "cmd:enter";
        case Mark::CmdTrainStart: return "cmd:train-start";
        case Mark::CmdTrainDone: return "cmd:train-done";
        case Mark::CmdWait: return "cmd:waiting";
        case Mark::CmdChallengeSeen: return "cmd:challenged";
        case Mark::CmdExecLeave: return "cmd:leave";
        case Mark::RxEnter: return "rx:enter";
        case Mark::RxChallenge: return "rx:0x3C";
        case Mark::RxChallengeTx: return "rx:0x3D-tx";
        case Mark::RxPosition: return "rx:0x04";
        case Mark::RxKeyTransfer: return "rx:0x32";
        case Mark::RxLeave: return "rx:leave";
        case Mark::WebState: return "web:state";
        case Mark::WebCommand: return "web:command";
        default: return "-";
    }
}

Preferences prefs;
bool opened = false;
uint8_t failsAtBoot = 0;

/// The previous boot's last breadcrumb and the chip's reason for resetting,
/// both captured before anything overwrites them.
BootStage prevStage = BootStage::Unknown;
esp_reset_reason_t resetReason = ESP_RST_UNKNOWN;
uint32_t heapAtBoot = 0;

bool open() {
    if (!opened) opened = prefs.begin(NS, false);
    return opened;
}

/// Listed in the order the stages actually run, which is no longer the order
/// they are declared in -- see the append-only note in boot_guard.h.
const char *stageName(const BootStage s) {
    switch (s) {
        case BootStage::Console: return "console";
        case BootStage::Nvs: return "nvs";
        case BootStage::Settings: return "settings";
        case BootStage::Keystore: return "keystore";
        case BootStage::Registry: return "device registry";
        case BootStage::Pairing: return "pairing";
        case BootStage::Wifi: return "wifi";
        case BootStage::WebUi: return "web ui";
        case BootStage::Radio: return "radio";
        case BootStage::Control: return "command worker";
        case BootStage::Mqtt: return "mqtt bridge";  // between Control and Running
        case BootStage::Running: return "running (setup finished)";
        default: return "unknown";
    }
}

/// Why the chip reset. The distinction that matters here is whether this was a
/// clean restart, a panic, or a watchdog: a panic points at the code that ran
/// last, a watchdog at something that stopped yielding, and a brownout at the
/// power supply rather than at anything in this firmware.
const char *resetName(const esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON: return "power on";
        case ESP_RST_EXT: return "external reset";
        case ESP_RST_SW: return "software restart";
        case ESP_RST_PANIC: return "PANIC (exception or abort)";
        case ESP_RST_INT_WDT: return "interrupt watchdog";
        case ESP_RST_TASK_WDT: return "TASK WATCHDOG (something stopped yielding)";
        case ESP_RST_WDT: return "other watchdog";
        case ESP_RST_DEEPSLEEP: return "deep sleep wake";
        case ESP_RST_BROWNOUT: return "BROWNOUT (power supply)";
        case ESP_RST_SDIO: return "sdio";
        default: return "unknown";
    }
}

/// Clears the counter once the image has run long enough to be trusted.
void healthTask(void *) {
    vTaskDelay(pdMS_TO_TICKS(HEALTHY_AFTER_MS));
    if (open() && prefs.getUChar(KEY_FAILS, 0) != 0) {
        prefs.putUChar(KEY_FAILS, 0);
        ets_printf("boot guard: image healthy after %us, counter cleared\n",
                   HEALTHY_AFTER_MS / 1000);
    }
    vTaskDelete(nullptr);
}

}  // namespace

void bootGuardBegin() {
    // Captured whether or not NVS opens: the reset reason comes from the chip,
    // and it is the one clue that survives a crash this early.
    resetReason = esp_reset_reason();
    heapAtBoot = ESP.getFreeHeap();

    if (!open()) {
        ets_printf("boot guard: NVS unavailable -- NO rollback protection\n");
        return;
    }

    // Lift the markers out before anything overwrites them, then start fresh.
    if (s_markMagic == MARK_MAGIC) {
        const uint32_t have = s_markPos < MARK_LEN ? s_markPos : MARK_LEN;
        const uint32_t start = s_markPos - have;
        for (uint32_t i = 0; i < have; i++) prevMarks[i] = s_marks[(start + i) % MARK_LEN];
        prevMarkCount = static_cast<uint8_t>(have);
    }
    s_markMagic = MARK_MAGIC;
    s_markPos = 0;

    prevStage = static_cast<BootStage>(prefs.getUChar(KEY_STAGE, 0));
    failsAtBoot = prefs.getUChar(KEY_FAILS, 0);

    // Push the previous boot's ending onto the history, newest first. Done here
    // rather than at the point of failure, because a boot that fails does not
    // get to write anything down -- what killed it is only knowable from the
    // outside, on the boot after.
    prefs.getBytes(KEY_HIST, hist, sizeof(hist));
    for (uint8_t i = HIST_LEN - 1; i > 0; i--) hist[i] = hist[i - 1];
    hist[0].stage = static_cast<uint8_t>(prevStage);
    hist[0].reason = static_cast<uint8_t>(resetReason);
    prefs.putBytes(KEY_HIST, hist, sizeof(hist));

    if (failsAtBoot >= FAIL_LIMIT) {
        // Clear the counter BEFORE switching. If the other slot is also bad we
        // want it to get its own full allowance rather than ping-pong between
        // two broken images on every single boot.
        prefs.putUChar(KEY_FAILS, 0);

        const esp_partition_t *running = esp_ota_get_running_partition();
        const esp_partition_t *other = esp_ota_get_next_update_partition(running);
        esp_app_desc_t desc;
        if (other && esp_ota_get_partition_description(other, &desc) == ESP_OK) {
            ets_printf("\n!! boot guard: %u failed boots -- rolling back to '%s' (%s %s)\n",
                       failsAtBoot, other->label, desc.version, desc.date);
            if (esp_ota_set_boot_partition(other) == ESP_OK) {
                delay(200);
                esp_restart();
            }
            ets_printf("!! boot guard: could not set boot partition; continuing\n");
        } else {
            // Nothing to fall back to -- a freshly serial-flashed device has
            // only one valid slot. Say so rather than restarting into nothing.
            ets_printf("!! boot guard: %u failed boots but no valid alternate image\n",
                       failsAtBoot);
        }
    }

    prefs.putUChar(KEY_FAILS, static_cast<uint8_t>(failsAtBoot + 1));
    xTaskCreate(healthTask, "bootguard", 2560, nullptr, 1, nullptr);
    ets_printf("boot guard: attempt %u of %u, healthy at %us\n", failsAtBoot + 1, FAIL_LIMIT,
               HEALTHY_AFTER_MS / 1000);
}

void bootGuardMark(const Mark m) {
    s_marks[s_markPos % MARK_LEN] = static_cast<uint8_t>(m);
    s_markPos++;
}

void bootGuardStage(const BootStage stage) {
    if (!open()) return;
    // One small NVS write per stage, eleven per boot. NVS wear-levels and this
    // is nothing against its endurance; the alternative is a class of failure
    // that cannot report itself at all.
    prefs.putUChar(KEY_STAGE, static_cast<uint8_t>(stage));
}

void bootGuardReport() {
    ets_printf("boot: reset reason '%s', previous boot reached '%s', heap %lu free\n",
               resetName(resetReason), stageName(prevStage),
               static_cast<unsigned long>(heapAtBoot));
    // A previous boot that did not reach the end of setup() is the interesting
    // case, and the one the network console could never see before: say so
    // loudly rather than leaving it in a line someone has to notice.
    if (prevStage != BootStage::Running && prevStage != BootStage::Unknown)
        ets_printf("!! the previous boot died in '%s' -- that stage is the suspect\n",
                   stageName(prevStage));

    // A panic prints the fault and the backtrace on its way down, and those
    // survive in RTC memory. Nothing is more worth saying at this point, so it
    // is said without being asked for.
    if (resetReason == ESP_RST_PANIC || resetReason == ESP_RST_TASK_WDT ||
        resetReason == ESP_RST_INT_WDT)
        netConsoleDumpPreviousTail();
}

void bootGuardStatus() {
    bootGuardReport();
    ets_printf("boot: heap now %lu free, %lu least ever\n",
               static_cast<unsigned long>(ESP.getFreeHeap()),
               static_cast<unsigned long>(ESP.getMinFreeHeap()));
    if (prevMarkCount) {
        ets_printf("where the previous boot had been, oldest first:\n  ");
        for (uint8_t i = 0; i < prevMarkCount; i++)
            ets_printf("%s%s", i ? " -> " : "", markName(prevMarks[i]));
        ets_printf("\n");
    }
    ets_printf("boot history, newest first:\n");
    for (uint8_t i = 0; i < HIST_LEN; i++)
        ets_printf("  -%u: ended in '%s', next boot's reset reason '%s'\n", i + 1,
                   stageName(static_cast<BootStage>(hist[i].stage)),
                   resetName(static_cast<esp_reset_reason_t>(hist[i].reason)));
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t desc;
    const bool haveDesc = running && esp_ota_get_partition_description(running, &desc) == ESP_OK;
    ets_printf("boot guard: running '%s'%s%s, failed boots before this one: %u/%u\n",
               running ? running->label : "?", haveDesc ? ", built " : "",
               haveDesc ? desc.date : "", failsAtBoot, FAIL_LIMIT);
}
