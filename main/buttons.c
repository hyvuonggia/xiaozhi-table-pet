/* Buttons lane implementation. See buttons.h for pin/behavior contract. */

#include "buttons.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "buttons";

#define BTN_BOOT_PIN     GPIO_NUM_0
#define BTN_VOL_UP_PIN   GPIO_NUM_40
#define BTN_VOL_DOWN_PIN GPIO_NUM_39

#define BTN_POLL_MS      50
#define BTN_DEBOUNCE_N   2    /* 2 identical reads = ~100 ms stable */
#define BTN_HOLD_MS      2000 /* 2 s hold threshold */
#define BTN_COUNT        3

/* SPEC §2.3: shared volume, starts at 50. Single writer (poll task). */
static volatile int s_volume = 50;
static bool s_running;

typedef struct {
    gpio_num_t pin;
    const char *name;
    int last_raw;          /* last read level */
    int stable;            /* debounced level (1 = released, 0 = pressed) */
    int same_count;
    TickType_t press_tick; /* when stable became 0 */
    bool hold_fired;
} btn_state_t;

static btn_state_t s_btns[BTN_COUNT] = {
    { .pin = BTN_BOOT_PIN, .name = "BOOT", .last_raw = 1, .stable = 1 },
    { .pin = BTN_VOL_UP_PIN, .name = "VOL+", .last_raw = 1, .stable = 1 },
    { .pin = BTN_VOL_DOWN_PIN, .name = "VOL-", .last_raw = 1, .stable = 1 },
};

int buttons_get_volume(void)
{
    return s_volume;
}

void buttons_set_volume(int vol)
{
    if (vol < 0) {
        vol = 0;
    }
    if (vol > 100) {
        vol = 100;
    }
    s_volume = vol;
    ESP_LOGI(TAG, "volume set -> %d", s_volume);
}

static void btn_on_press(btn_state_t *b)
{
    b->press_tick = xTaskGetTickCount();
    b->hold_fired = false;
    if (b->pin == BTN_VOL_UP_PIN) {
        int v = s_volume + 10;
        if (v > 100) {
            v = 100;
        }
        s_volume = v;
        ESP_LOGI(TAG, "VOL+ press -> volume %d", s_volume);
    } else if (b->pin == BTN_VOL_DOWN_PIN) {
        int v = s_volume - 10;
        if (v < 0) {
            v = 0;
        }
        s_volume = v;
        ESP_LOGI(TAG, "VOL- press -> volume %d", s_volume);
    } else {
        ESP_LOGI(TAG, "BOOT press (hold >= 2s for display-mode)");
    }
}

static void btn_on_hold(btn_state_t *b)
{
    if (b->pin == BTN_VOL_UP_PIN) {
        s_volume = 100;
        ESP_LOGI(TAG, "VOL+ hold >= 2s -> volume 100");
    } else if (b->pin == BTN_VOL_DOWN_PIN) {
        s_volume = 0;
        ESP_LOGI(TAG, "VOL- hold >= 2s -> volume 0");
    } else {
        /* Display-mode hook stub: log only, never touch display state. */
        ESP_LOGI(TAG, "BOOT hold >= 2s -> display-mode hook stub (no-op)");
    }
}

static void btn_on_release(btn_state_t *b)
{
    ESP_LOGI(TAG, "%s released", b->name);
}

/* Prio 3, 50 ms poll, software debounce, finite waits only. */
static void buttons_poll_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "poll task start (50ms, prio 3), volume=%d", s_volume);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(BTN_POLL_MS));
        for (int i = 0; i < BTN_COUNT; i++) {
            btn_state_t *b = &s_btns[i];
            int raw = gpio_get_level(b->pin); /* active-low */
            if (raw != 0 && raw != 1) {
                continue; /* bad read: skip slice, never exit */
            }
            if (raw == b->last_raw) {
                if (b->same_count < BTN_DEBOUNCE_N) {
                    b->same_count++;
                }
            } else {
                b->last_raw = raw;
                b->same_count = 1;
            }
            if (b->same_count >= BTN_DEBOUNCE_N && raw != b->stable) {
                b->stable = raw;
                if (raw == 0) {
                    btn_on_press(b);
                } else {
                    btn_on_release(b);
                }
            }
            if (b->stable == 0 && !b->hold_fired
                && (xTaskGetTickCount() - b->press_tick)
                    >= pdMS_TO_TICKS(BTN_HOLD_MS)) {
                b->hold_fired = true;
                btn_on_hold(b);
            }
        }
    }
}

void buttons_init(void)
{
    if (s_running) {
        ESP_LOGI(TAG, "already init");
        return;
    }
    const gpio_num_t pins[BTN_COUNT] = {
        BTN_BOOT_PIN, BTN_VOL_UP_PIN, BTN_VOL_DOWN_PIN,
    };
    for (int i = 0; i < BTN_COUNT; i++) {
        if (gpio_reset_pin(pins[i]) != ESP_OK) {
            ESP_LOGW(TAG, "GPIO%d reset failed (continuing)", (int)pins[i]);
            continue;
        }
        if (gpio_set_direction(pins[i], GPIO_MODE_INPUT) != ESP_OK) {
            ESP_LOGW(TAG, "GPIO%d direction failed (continuing)", (int)pins[i]);
            continue;
        }
        if (gpio_pullup_en(pins[i]) != ESP_OK) {
            ESP_LOGW(TAG, "GPIO%d pullup failed (continuing)", (int)pins[i]);
        }
    }
    ESP_LOGI(TAG, "init OK: BOOT=GPIO0 VOL+=GPIO40 VOL-=GPIO39, volume=%d",
             s_volume);
    if (xTaskCreate(buttons_poll_task, "buttons", 2048, NULL, 3, NULL) != pdPASS) {
        ESP_LOGW(TAG, "poll task create failed");
        return;
    }
    s_running = true;
}
