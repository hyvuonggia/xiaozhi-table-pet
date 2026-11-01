/* Servo lane implementation. See servo.h for THE-servo-lock contract. */

#include "servo.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "servo";

/* SPEC §2.2: SG90/MG90, GPIO11 = horizontal (ch0), GPIO12 = vertical (ch1). */
#define SERVO_GPIO_H   11
#define SERVO_GPIO_V   12
#define SERVO_CH_H     LEDC_CHANNEL_0
#define SERVO_CH_V     LEDC_CHANNEL_1
#define SERVO_TIMER    LEDC_TIMER_0
#define SERVO_MODE     LEDC_LOW_SPEED_MODE
#define SERVO_FREQ_HZ  50
#define SERVO_RES      LEDC_TIMER_14_BIT /* 16383 steps */
#define SERVO_US_MIN   500
#define SERVO_US_MAX   2500
#define SERVO_US_PER_CYCLE (1000000 / SERVO_FREQ_HZ) /* 20000 us */
#define SERVO_DUTY_MAX ((1 << 14) - 1)               /* 16383 */

#define SERVO_LOCK_MS  2000 /* SPEC §5.4 #17: try-lock <= 2 s */
#define SERVO_DEBOUNCE_MS 3000 /* SPEC §5.5 #22 */
#define SERVO_STEP_MS  400
#define SERVO_DEMO_DELAY_MS 1000
/* The 3V3 rail is shared with the display/amp and powered from laptop USB
 * (SPEC §2.3): a 30-60 deg jump browns the SH1106 out and latches SDA low.
 * Move in 5 deg steps instead of one jump (same one nod/shake, lower peak
 * current). Live-tested: K-shake killed the panel on the jump version. */
#define SERVO_RAMP_STEP_DEG 5
#define SERVO_RAMP_STEP_MS  60

/* THE servo lock: the single mutex guarding all PWM writes. */
static SemaphoreHandle_t s_lock;
static bool s_ready;
static TickType_t s_last_motion;
static bool s_have_motion;
static int s_cur_h = 90; /* last successfully written angles (for ramping) */
static int s_cur_v = 90;

static int clamp_angle(int a)
{
    if (a < 0) {
        return 0;
    }
    if (a > 180) {
        return 180;
    }
    return a;
}

static uint32_t angle_to_duty(int angle)
{
    angle = clamp_angle(angle);
    uint32_t us = (uint32_t)SERVO_US_MIN
        + (uint32_t)(angle * (SERVO_US_MAX - SERVO_US_MIN)) / 180u;
    return (us * (uint32_t)SERVO_DUTY_MAX) / (uint32_t)SERVO_US_PER_CYCLE;
}

static uint32_t angle_to_us(int angle)
{
    angle = clamp_angle(angle);
    return (uint32_t)SERVO_US_MIN
        + (uint32_t)(angle * (SERVO_US_MAX - SERVO_US_MIN)) / 180u;
}

/* The one task-safe funnel: every PWM write in this file goes through here.
 * Takes THE lock (<= 2 s); on timeout the step is skipped, never forced.
 * Unlocks ONLY when the take succeeded. */
static void servo_move_both(int h_angle, int v_angle)
{
    h_angle = clamp_angle(h_angle);
    v_angle = clamp_angle(v_angle);
    if (!s_ready || !s_lock) {
        ESP_LOGW(TAG, "move H=%d V=%d skipped: not ready", h_angle, v_angle);
        return;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(SERVO_LOCK_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "move H=%d V=%d skipped: lock busy (>%dms)",
                 h_angle, v_angle, SERVO_LOCK_MS);
        return;
    }
    uint32_t dh = angle_to_duty(h_angle);
    uint32_t dv = angle_to_duty(v_angle);
    esp_err_t eh = ledc_set_duty(SERVO_MODE, SERVO_CH_H, dh);
    esp_err_t uh = (eh == ESP_OK) ? ledc_update_duty(SERVO_MODE, SERVO_CH_H) : eh;
    esp_err_t ev = ledc_set_duty(SERVO_MODE, SERVO_CH_V, dv);
    esp_err_t uv = (ev == ESP_OK) ? ledc_update_duty(SERVO_MODE, SERVO_CH_V) : ev;
    xSemaphoreGive(s_lock); /* locked above, so give is paired */
    if (uh != ESP_OK || uv != ESP_OK) {
        ESP_LOGW(TAG, "move H=%d V=%d driver err h=%d v=%d",
                 h_angle, v_angle, (int)uh, (int)uv);
        return;
    }
    s_cur_h = h_angle;
    s_cur_v = v_angle;
    ESP_LOGI(TAG, "angles H=%d (%luus) V=%d (%luus)",
             h_angle, (unsigned long)angle_to_us(h_angle),
             v_angle, (unsigned long)angle_to_us(v_angle));
}

/* Move to (h_to,v_to) in small steps: one logical nod/shake, low peak
 * current on the shared 3V3 rail (see SERVO_RAMP_* above). */
static void servo_ramp_both(int h_to, int v_to)
{
    h_to = clamp_angle(h_to);
    v_to = clamp_angle(v_to);
    int h = s_cur_h;
    int v = s_cur_v;
    while (h != h_to || v != v_to) {
        int dh = h_to - h;
        int dv = v_to - v;
        if (dh > SERVO_RAMP_STEP_DEG) {
            dh = SERVO_RAMP_STEP_DEG;
        } else if (dh < -SERVO_RAMP_STEP_DEG) {
            dh = -SERVO_RAMP_STEP_DEG;
        }
        if (dv > SERVO_RAMP_STEP_DEG) {
            dv = SERVO_RAMP_STEP_DEG;
        } else if (dv < -SERVO_RAMP_STEP_DEG) {
            dv = -SERVO_RAMP_STEP_DEG;
        }
        h += dh;
        v += dv;
        servo_move_both(h, v);
        vTaskDelay(pdMS_TO_TICKS(SERVO_RAMP_STEP_MS));
    }
}

/* 3 s debounce for motion commands. Returns true when the motion may run
 * (and stamps the time); false when it must be skipped. */
static bool servo_motion_allowed(const char *name)
{
    TickType_t now = xTaskGetTickCount();
    if (s_have_motion
        && (now - s_last_motion) < pdMS_TO_TICKS(SERVO_DEBOUNCE_MS)) {
        ESP_LOGI(TAG, "%s debounced (< %dms since last motion)",
                 name, SERVO_DEBOUNCE_MS);
        return false;
    }
    s_last_motion = now;
    s_have_motion = true;
    return true;
}

void servo_center(void)
{
    ESP_LOGI(TAG, "center 90/90");
    servo_move_both(90, 90);
}

void servo_nod_once(void)
{
    if (!servo_motion_allowed("nod")) {
        return;
    }
    /* EXACTLY one down-up (SPEC §5.5 #21): V 90 -> 60 -> 90, H stays 90. */
    ESP_LOGI(TAG, "nod step 1/2 V=60 (down)");
    servo_ramp_both(90, 60);
    vTaskDelay(pdMS_TO_TICKS(SERVO_STEP_MS));
    ESP_LOGI(TAG, "nod step 2/2 V=90 (up)");
    servo_ramp_both(90, 90);
    ESP_LOGI(TAG, "nod done (one down-up)");
}

void servo_shake_once(void)
{
    if (!servo_motion_allowed("shake")) {
        return;
    }
    /* EXACTLY one left-right: H 90 -> 60 -> 120 -> 90, V stays 90. */
    ESP_LOGI(TAG, "shake step 1/3 H=60 (left)");
    servo_ramp_both(60, 90);
    vTaskDelay(pdMS_TO_TICKS(SERVO_STEP_MS));
    ESP_LOGI(TAG, "shake step 2/3 H=120 (right)");
    servo_ramp_both(120, 90);
    vTaskDelay(pdMS_TO_TICKS(SERVO_STEP_MS));
    ESP_LOGI(TAG, "shake step 3/3 H=90 (center)");
    servo_ramp_both(90, 90);
    ESP_LOGI(TAG, "shake done (one left-right)");
}

/* Boot demo task: visible proof in the monitor log (center + single nod). */
static void servo_demo_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(SERVO_DEMO_DELAY_MS));
    ESP_LOGI(TAG, "boot demo: single nod");
    servo_nod_once();
    ESP_LOGI(TAG, "boot demo done");
    vTaskDelete(NULL);
}

void servo_init(void)
{
    if (s_ready) {
        ESP_LOGI(TAG, "already init");
        return;
    }
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) {
        ESP_LOGE(TAG, "mutex create failed, servo disabled");
        return;
    }

    ledc_timer_config_t timer = {
        .speed_mode = SERVO_MODE,
        .timer_num = SERVO_TIMER,
        .duty_resolution = SERVO_RES,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&timer) != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed, servo disabled");
        return;
    }
    ledc_channel_config_t ch_h = {
        .gpio_num = SERVO_GPIO_H,
        .speed_mode = SERVO_MODE,
        .channel = SERVO_CH_H,
        .timer_sel = SERVO_TIMER,
        .duty = angle_to_duty(90),
        .hpoint = 0,
    };
    ledc_channel_config_t ch_v = {
        .gpio_num = SERVO_GPIO_V,
        .speed_mode = SERVO_MODE,
        .channel = SERVO_CH_V,
        .timer_sel = SERVO_TIMER,
        .duty = angle_to_duty(90),
        .hpoint = 0,
    };
    if (ledc_channel_config(&ch_h) != ESP_OK
        || ledc_channel_config(&ch_v) != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed, servo disabled");
        return;
    }

    s_ready = true;
    s_have_motion = false;
    ESP_LOGI(TAG, "init OK: GPIO%d ch0 H + GPIO%d ch1 V, 50Hz 500-2500us",
             SERVO_GPIO_H, SERVO_GPIO_V);
    servo_center();
    if (xTaskCreateWithCaps(servo_demo_task, "servo_demo", 3072, NULL, 3, NULL, MALLOC_CAP_SPIRAM) != pdPASS) {
        ESP_LOGW(TAG, "demo task create failed (servo still usable)");
    }
}
