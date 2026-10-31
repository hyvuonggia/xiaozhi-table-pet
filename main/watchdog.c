/* Supervisor implementation. See watchdog.h for the contract.
 * All waits finite (1 s tick + 100 ms sub-samples); all errors soft-fail;
 * no banned tokens (CI-checked).
 */

#include "watchdog.h"

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "audio_in.h"
#include "audio_out.h"
#include "buttons.h"
#include "voice_path.h"
#include "display.h"

static const char *TAG = "watchdog";

#define WD_PRIO          3
#define WD_STACK         3072
#define WD_TICK_MS       1000
#define WD_HEARTBEAT_MS  30000  /* SPEC §5.4#16 */
#define WD_MIC_MS        25000  /* SPEC §5.2#10 */
#define WD_DISP_DEAD_MS  90000  /* SPEC §5.1#7 */
#define WD_FORGIVE_MS    600000 /* E3: forgive after 10 min healthy */
#define WD_BOOT_SUB_MS   100    /* BOOT edge sub-sample inside the 1 s tick */
#define WD_BOOT_HOLD_MS  2000   /* >= 2 s hold = display-stub parity */

/* Reboot-cause codes retained across esp_restart (power loss clears). */
#define WD_REASON_HEARTBEAT  1u
#define WD_REASON_MIC        2u
#define WD_REASON_DISPLAY    3u

static RTC_NOINIT_ATTR uint32_t s_wd_last_reason;
static bool s_running;
static TickType_t s_heartbeat_tick;
static bool s_heartbeat_seen;
static TickType_t s_mic_listen_since; /* first tick of continuous listening */

/* Display progress state (SPEC §5.1#4: frame counters, never task liveness).
 * Unhealthy = pending frames with zero completions advancing, OR the
 * display lane itself reporting !display_healthy(). */
static uint32_t s_disp_last_comp;
static bool s_disp_baselined;
static TickType_t s_disp_progress_tick;
static TickType_t s_disp_unhealthy_since; /* 0 = currently healthy */
static TickType_t s_disp_healthy_since;
static bool s_disp_rescue_tried;

/* BOOT edge state (sub-sampled; buttons.c owns the real debounce). */
static int s_boot_last_level = 1;
static TickType_t s_boot_press_tick;
static bool s_boot_pressed;

void watchdog_heartbeat(void)
{
    s_heartbeat_tick = xTaskGetTickCount();
    s_heartbeat_seen = true;
}

/* Set retained cause, then reboot. Single choke point for owned restarts. */
static void wd_reboot(uint32_t reason, const char *why)
{
    s_wd_last_reason = reason;
    ESP_LOGE(TAG, "REBOOT reason=%lu: %s", (unsigned long)reason, why);
    vTaskDelay(pdMS_TO_TICKS(200)); /* let the log flush */
    esp_restart();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); /* unreachable, defensive */
    }
}

/* Unsigned tick subtraction is only safe when `now` was sampled after the
 * stamp. The mic task (prio 8) can stamp BETWEEN our snapshot and this
 * comparison, making `now - last` wrap to ~4 billion ticks and rebooting a
 * perfectly healthy robot (seen live 2026-10-31). Always compare signed. */
static inline bool wd_stale(TickType_t now, TickType_t stamp, uint32_t ms)
{
    return (int32_t)(now - stamp) > (int32_t)pdMS_TO_TICKS(ms);
}

static void wd_check_heartbeat(TickType_t now)
{
    if (!s_heartbeat_seen) {
        return; /* main loop not up yet; grace until first stamp */
    }
    if (wd_stale(now, s_heartbeat_tick, WD_HEARTBEAT_MS)) {
        wd_reboot(WD_REASON_HEARTBEAT, "main loop >30 s without heartbeat");
    }
}

static void wd_check_mic(TickType_t now)
{
    if (!voice_path_is_listening()) {
        s_mic_listen_since = 0;
        return; /* SPEC: only while a session is active */
    }
    TickType_t last = audio_in_last_ok_tick();
    if (last == 0) {
        /* Never succeeded: DMA starves from boot (loose mic clock/data).
         * Healthy mics deliver within milliseconds even in silence, so
         * allow a generous 90 s priming window, then reboot (SPEC §5.2#10).
         * Without this, a never-hearing mic gets grace forever. */
        if (s_mic_listen_since == 0) {
            s_mic_listen_since = now;
            return;
        }
        if (wd_stale(now, s_mic_listen_since, 90000)) {
            wd_reboot(WD_REASON_MIC, "mic never delivered in 90 s of session");
        }
        return;
    }
    s_mic_listen_since = 0;
    if (wd_stale(now, last, WD_MIC_MS)) {
        wd_reboot(WD_REASON_MIC, "mic >25 s without successful read in session");
    }
}

static void wd_check_display(TickType_t now)
{
    uint32_t req = display_requested_frames();
    uint32_t comp = display_completed_frames();
    bool healthy = display_healthy();
    if (!s_disp_baselined) {
        s_disp_baselined = true;
        s_disp_last_comp = comp;
        s_disp_progress_tick = now;
        s_disp_healthy_since = now;
        return;
    }
    if (comp != s_disp_last_comp) {
        /* Progress: healthy. Forgive stale history after 10 min healthy. */
        s_disp_last_comp = comp;
        s_disp_progress_tick = now;
        s_disp_unhealthy_since = 0;
        if (wd_stale(now, s_disp_healthy_since, WD_FORGIVE_MS)) {
            s_disp_rescue_tried = false;
            s_disp_healthy_since = now;
            ESP_LOGI(TAG, "display healthy 10 min: counters forgiven");
        }
        return;
    }
    if (req == comp && healthy) {
        /* Idle (render-on-change): nothing pending, lane agrees: healthy. */
        s_disp_progress_tick = now;
        s_disp_unhealthy_since = 0;
        return;
    }
    /* Pending frames with zero completions advancing, or the display lane
     * itself reporting unhealthy: stuck. */
    if (s_disp_unhealthy_since == 0) {
        s_disp_unhealthy_since = now;
        ESP_LOGW(TAG, "display stuck: req=%lu comp=%lu healthy=%d",
                 (unsigned long)req, (unsigned long)comp, (int)healthy);
        return;
    }
    if (!wd_stale(now, s_disp_unhealthy_since, WD_DISP_DEAD_MS)) {
        return;
    }
    if (!s_disp_rescue_tried) {
        s_disp_rescue_tried = true;
        s_disp_unhealthy_since = now; /* one more 90 s window after rescue */
        ESP_LOGW(TAG, "display dead >90 s: attempting bus rescue");
        bool ok = display_rescue_bus();
        ESP_LOGW(TAG, "display rescue %s", ok ? "OK" : "FAILED");
        return;
    }
    wd_reboot(WD_REASON_DISPLAY, "display still dead after rescue");
}

/* F6 volume: push on change only (log each change, soft-fail). */
static void wd_poll_volume(void)
{
    static int s_last_vol = -1;
    int v = buttons_get_volume();
    if (v == s_last_vol) {
        return;
    }
    s_last_vol = v;
    audio_out_set_volume(v);
    ESP_LOGI(TAG, "volume -> %d", v);
}

/* F6 BOOT short-press -> relisten; hold >= 2 s -> stub parity log.
 * Called every 100 ms sub-sample; press/release spans ticks. */
static void wd_poll_boot(TickType_t now)
{
    int lvl = gpio_get_level(GPIO_NUM_0); /* active-low, pull-up per buttons.c */
    if (lvl != 0 && lvl != 1) {
        return; /* bad read: skip slice */
    }
    if (lvl == s_boot_last_level) {
        if (s_boot_pressed
            && (now - s_boot_press_tick) >= pdMS_TO_TICKS(WD_BOOT_HOLD_MS)) {
            /* Hold threshold crossed while still down: parity log once. */
            s_boot_pressed = false; /* consume so release stays silent */
            ESP_LOGI(TAG, "BOOT hold >= 2s -> display-mode hook stub (no-op)");
        }
        return;
    }
    s_boot_last_level = lvl;
    if (lvl == 0) {
        s_boot_press_tick = now;
        s_boot_pressed = true;
    } else if (s_boot_pressed) {
        s_boot_pressed = false;
        if ((now - s_boot_press_tick) < pdMS_TO_TICKS(WD_BOOT_HOLD_MS)) {
            ESP_LOGI(TAG, "BOOT short-press -> relisten");
            voice_path_relisten();
        }
    }
}

static void watchdog_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "supervisor on (prio %d, 1 s tick)", WD_PRIO);
    while (1) {
        for (int i = 0; i < WD_TICK_MS / WD_BOOT_SUB_MS; i++) {
            vTaskDelay(pdMS_TO_TICKS(WD_BOOT_SUB_MS));
            wd_poll_boot(xTaskGetTickCount());
        }
        TickType_t now = xTaskGetTickCount();
        wd_poll_volume();
        wd_check_heartbeat(now);
        wd_check_mic(now);
        wd_check_display(now);
    }
}

void watchdog_init(void)
{
    esp_reset_reason_t rr = esp_reset_reason();
    ESP_LOGI(TAG, "boot: esp_reset_reason=%d last-wd-reason=%lu",
             (int)rr, (unsigned long)s_wd_last_reason);
    if (s_wd_last_reason == WD_REASON_HEARTBEAT) {
        ESP_LOGW(TAG, "last reboot: main-loop heartbeat >30 s");
    } else if (s_wd_last_reason == WD_REASON_MIC) {
        ESP_LOGW(TAG, "last reboot: mic >25 s stale in session");
    } else if (s_wd_last_reason == WD_REASON_DISPLAY) {
        ESP_LOGW(TAG, "last reboot: display dead after rescue");
    }
    if (s_running) {
        ESP_LOGI(TAG, "already init");
        return;
    }
    if (xTaskCreate(watchdog_task, "watchdog", WD_STACK, NULL,
                    WD_PRIO, NULL) != pdPASS) {
        ESP_LOGW(TAG, "supervisor task OOM (running without watchdog)");
        return;
    }
    s_running = true;
}
