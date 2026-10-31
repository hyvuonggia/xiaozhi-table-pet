/* Wake word gating (WAKE lane): esp-sr WakeNet on 16 kHz mono int16.
 * See wakeword.h for the contract. Model + phrase are logged at init;
 * every failure path logs and degrades to "no wake" (never panics).
 */

#include "wakeword.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "freertos/semphr.h"

#include "model_path.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"

static void wakeword_detect_task(void *arg);

static const char *TAG = "wake";

/* Preferred model (SPEC section 9.5: wake word must be an existing model). */
#define WAKE_MODEL_WANT "wn9_alexa"

#define WAKE_TASK_PRIO   4
#define WAKE_TASK_STACK  8192
#define WAKE_RING_SAMP   32768   /* ~2 s @ 16 kHz; single malloc -> PSRAM */
#define WAKE_COOLDOWN_MS 3000    /* silence detect right after a trigger */

static const esp_wn_iface_t *s_wn;
static model_iface_data_t *s_data;
static int s_chunksize;           /* samples per detect() call */
static char s_model_name[MODEL_NAME_MAX_LENGTH];

static int16_t *s_ring;
static volatile uint32_t s_head;  /* written by mic task (feed) */
static volatile uint32_t s_tail;  /* advanced by detect task */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_drops;

static volatile bool s_flag;      /* latched detection, cleared by consume */
static volatile TickType_t s_last_tick;
static bool s_inited;

static uint32_t ring_avail(void)
{
    uint32_t h = s_head, t = s_tail;
    return (h >= t) ? (h - t) : (WAKE_RING_SAMP - (t - h));
}

void wakeword_init(void)
{
    if (s_inited) {
        return;
    }
    size_t psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (psram == 0) {
        /* Boot already panics without PSRAM (IGNORE_NOTFOUND=n), so this
         * is belt-and-braces: log loudly, stay silent, never crash here. */
        ESP_LOGE(TAG, "no PSRAM found — wake word disabled (needs octal PSRAM)");
        return;
    }
    ESP_LOGI(TAG, "PSRAM ok (%u bytes), loading wakenet", (unsigned)psram);

    srmodel_list_t *models = esp_srmodel_init("model");
    if (!models) {
        ESP_LOGE(TAG, "esp_srmodel_init(model) failed — wake word disabled");
        return;
    }
    char *name = esp_srmodel_filter(models, ESP_WN_PREFIX, "alexa");
    if (!name) {
        ESP_LOGW(TAG, "no alexa wakenet in model partition, trying any wn model");
        name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
    }
    if (!name) {
        ESP_LOGE(TAG, "no wakenet model in model partition — wake word disabled");
        esp_srmodel_deinit(models);
        return;
    }
    snprintf(s_model_name, sizeof(s_model_name), "%s", name);

    s_wn = esp_wn_handle_from_name(s_model_name);
    if (!s_wn) {
        ESP_LOGE(TAG, "no wakenet iface for %s — wake word disabled", s_model_name);
        esp_srmodel_deinit(models);
        return;
    }
    /* NOTE: models stay mmap'd from the model partition; deinit would
     * unmap them, so the list intentionally lives for the whole boot. */
    (void)models;

    /* DET_MODE_95 (aggressive) per the esp-sr wakenet test-app pattern
     * for wn9 models; Normal (90) under-triggers on this board's mic. */
    s_data = s_wn->create(s_model_name, DET_MODE_95);
    if (!s_data) {
        ESP_LOGE(TAG, "wakenet create(%s) failed — wake word disabled", s_model_name);
        s_wn = NULL;
        return;
    }
    int rate = s_wn->get_samp_rate(s_data);
    s_chunksize = s_wn->get_samp_chunksize(s_data);
    if (rate != 16000 || s_chunksize <= 0
        || s_chunksize > (int)(WAKE_RING_SAMP / 2)) {
        ESP_LOGE(TAG, "wakenet bad params rate=%d chunk=%d — disabled",
                 rate, s_chunksize);
        s_wn->destroy(s_data);
        s_data = NULL;
        s_wn = NULL;
        return;
    }

    s_ring = heap_caps_malloc(WAKE_RING_SAMP * sizeof(int16_t),
                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_ring) {
        s_ring = malloc(WAKE_RING_SAMP * sizeof(int16_t));
    }
    if (!s_ring) {
        ESP_LOGE(TAG, "wake ring OOM — wake word disabled");
        s_wn->destroy(s_data);
        s_data = NULL;
        s_wn = NULL;
        return;
    }

    /* Log the human phrase (e.g. "Alexa") alongside the model name. */
    int nwords = s_wn->get_word_num(s_data);
    char phrase[64] = "?";
    if (nwords >= 1) {
        char *w0 = s_wn->get_word_name(s_data, 1);
        if (w0) {
            snprintf(phrase, sizeof(phrase), "%s", w0);
        }
    }
    ESP_LOGI(TAG, "wakenet loaded: model=%s phrase=\"%s\" rate=%d chunk=%d",
             s_model_name, phrase, rate, s_chunksize);

    s_inited = true; /* set before task start so feed() accepts samples */
    if (xTaskCreate(wakeword_detect_task, "wake_det", WAKE_TASK_STACK,
                    NULL, WAKE_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "detect task OOM — wake word disabled");
        s_inited = false;
        return;
    }
    ESP_LOGI(TAG, "wake word ready (say \"%s\")", phrase);
}

void wakeword_feed_pcm16(const int16_t *pcm, int n)
{
    if (!s_inited || !s_ring || !pcm || n <= 0) {
        return;
    }
    portENTER_CRITICAL(&s_mux);
    uint32_t avail = ring_avail();
    if ((uint32_t)n > WAKE_RING_SAMP - 1) {
        /* Absurd burst: keep only the newest ring-full. */
        pcm += n - (WAKE_RING_SAMP - 1);
        n = WAKE_RING_SAMP - 1;
        avail = ring_avail();
    }
    uint32_t free = (WAKE_RING_SAMP - 1) - avail;
    if ((uint32_t)n > free) {
        uint32_t drop = (uint32_t)n - free;
        s_tail = (s_tail + drop) % WAKE_RING_SAMP;
        s_drops += drop;
    }
    uint32_t h = s_head;
    uint32_t first = WAKE_RING_SAMP - h;
    if ((uint32_t)n <= first) {
        memcpy(s_ring + h, pcm, (size_t)n * sizeof(int16_t));
    } else {
        memcpy(s_ring + h, pcm, first * sizeof(int16_t));
        memcpy(s_ring, pcm + first, ((size_t)n - first) * sizeof(int16_t));
    }
    s_head = (h + (uint32_t)n) % WAKE_RING_SAMP;
    portEXIT_CRITICAL(&s_mux);
}

bool wakeword_consume(void)
{
    if (!s_flag) {
        return false;
    }
    portENTER_CRITICAL(&s_mux);
    bool hit = s_flag;
    s_flag = false;
    portEXIT_CRITICAL(&s_mux);
    return hit;
}

TickType_t wakeword_last_wake_tick(void)
{
    return s_last_tick;
}

/* Detect task (prio 4): drain the ring in chunksize blocks, run WakeNet.
 * All waits finite; detect runs outside the lock; post-trigger cooldown
 * avoids retriggering on the utterance tail. */
void wakeword_detect_task(void *arg)
{
    (void)arg;
    int16_t *frame = malloc((size_t)s_chunksize * sizeof(int16_t));
    if (!frame) {
        ESP_LOGE(TAG, "detect task frame OOM, exiting (wake disabled)");
        vTaskDelete(NULL);
        return;
    }
    TickType_t cooldown_until = 0;
    ESP_LOGI(TAG, "detect task on (chunk=%d)", s_chunksize);
    while (1) {
        uint32_t avail;
        portENTER_CRITICAL(&s_mux);
        avail = ring_avail();
        portEXIT_CRITICAL(&s_mux);
        if (avail < (uint32_t)s_chunksize) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        /* Copy one chunk out, advance tail, detect outside the lock. */
        portENTER_CRITICAL(&s_mux);
        uint32_t t = s_tail;
        uint32_t first = WAKE_RING_SAMP - t;
        if ((uint32_t)s_chunksize <= first) {
            memcpy(frame, s_ring + t, (size_t)s_chunksize * sizeof(int16_t));
        } else {
            memcpy(frame, s_ring + t, first * sizeof(int16_t));
            memcpy(frame + first, s_ring,
                   ((size_t)s_chunksize - first) * sizeof(int16_t));
        }
        s_tail = (t + (uint32_t)s_chunksize) % WAKE_RING_SAMP;
        if (s_drops > 0) {
            ESP_LOGW(TAG, "ring overflow, dropped %lu samples",
                     (unsigned long)s_drops);
            s_drops = 0;
        }
        portEXIT_CRITICAL(&s_mux);

        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(now - cooldown_until) < 0) {
            continue; /* cooldown: drain only */
        }
        wakenet_state_t st = s_wn->detect(s_data, frame);
        if (st > 0) {
            s_last_tick = xTaskGetTickCount();
            portENTER_CRITICAL(&s_mux);
            s_flag = true;
            portEXIT_CRITICAL(&s_mux);
            cooldown_until = s_last_tick + pdMS_TO_TICKS(WAKE_COOLDOWN_MS);
            ESP_LOGI(TAG, "WAKE \"%s\" (word=%d)", s_model_name, (int)st);
        }
    }
}
