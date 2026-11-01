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
#include "esp_vad.h"
#include "esp_vadn_iface.h"
#include "esp_vadn_models.h"

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

/* Neural VAD state (vadnet1_medium, 16 kHz mono). Same feed as WakeNet:
 * wakeword_feed_pcm16() copies into both rings; the detect task drains the
 * VAD ring in vad-chunksize blocks (typically 30 ms) and tracks the speech
 * streak voice_path.c gates streaming on. */
#define VAD_RING_SAMP 16384   /* ~1 s @ 16 kHz; single malloc -> PSRAM */
#define VAD_WANT      "vadnet1_medium"
static const esp_vadn_iface_t *s_vadn;
static model_iface_data_t *s_vad_data;
static int s_vad_chunksize;
static int16_t *s_vad_ring;
static volatile uint32_t s_vad_head;  /* written by mic task (feed) */
static volatile uint32_t s_vad_tail;  /* advanced by detect task */
static volatile uint32_t s_vad_streak;      /* consecutive VAD_SPEECH frames */
static volatile TickType_t s_vad_last_speech;
static volatile bool s_vad_now;             /* most recent frame was speech */
static volatile bool s_vad_ready;

static uint32_t vad_avail(void)
{
    uint32_t h = s_vad_head, t = s_vad_tail;
    return (h >= t) ? (h - t) : (VAD_RING_SAMP - (t - h));
}

/* Fail-closed init: any failure logs loudly and leaves s_vad_ready false,
 * in which case the conversation window never opens on speech. Never panics. */
static void vad_init(srmodel_list_t *models)
{
    char *name = esp_srmodel_filter(models, ESP_VADN_PREFIX, VAD_WANT);
    if (!name) {
        ESP_LOGW(TAG, "no " VAD_WANT " vadnet in model partition, trying any vadnet");
        name = esp_srmodel_filter(models, ESP_VADN_PREFIX, NULL);
    }
    if (!name) {
        ESP_LOGE(TAG, "no vadnet model in model partition — speech confirm disabled (window stays shut)");
        return;
    }
    s_vadn = esp_vadn_handle_from_name(name);
    if (!s_vadn) {
        ESP_LOGE(TAG, "no vadn iface for %s — speech confirm disabled", name);
        return;
    }
    /* Minimal internal smoothing (one 30 ms frame each way): all timing —
     * 3-frame confirm, 2 s silence close — lives in voice_path.c on top. */
    s_vad_data = s_vadn->create(name, VAD_MODE_3, 1, 30, 30);
    if (!s_vad_data) {
        ESP_LOGE(TAG, "vadn create(%s) failed — speech confirm disabled", name);
        s_vadn = NULL;
        return;
    }
    int rate = s_vadn->get_samp_rate(s_vad_data);
    s_vad_chunksize = s_vadn->get_samp_chunksize(s_vad_data);
    if (rate != 16000 || s_vad_chunksize <= 0
        || s_vad_chunksize > (int)(VAD_RING_SAMP / 2)) {
        ESP_LOGE(TAG, "vadn bad params rate=%d chunk=%d — disabled",
                 rate, s_vad_chunksize);
        s_vadn->destroy(s_vad_data);
        s_vad_data = NULL;
        s_vadn = NULL;
        return;
    }
    s_vad_ring = heap_caps_malloc(VAD_RING_SAMP * sizeof(int16_t),
                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_vad_ring) {
        s_vad_ring = malloc(VAD_RING_SAMP * sizeof(int16_t));
    }
    if (!s_vad_ring) {
        ESP_LOGE(TAG, "vad ring OOM — speech confirm disabled");
        s_vadn->destroy(s_vad_data);
        s_vad_data = NULL;
        s_vadn = NULL;
        return;
    }
    s_vad_ready = true;
    ESP_LOGI(TAG, "vadnet loaded: model=%s rate=%d chunk=%d",
             name, rate, s_vad_chunksize);
}

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

    /* VAD shares the mic feed and the model partition (no extra flash cost
     * beyond the packed vadnet blob). Fail-closed: speech confirm simply
     * stays off if the model is absent — the window then never streams. */
    vad_init(models);

    s_inited = true; /* set before task start so feed() accepts samples */
    /* Internal RAM on purpose: WakeNet/VADNet inference is realtime and a
     * PSRAM stack made detection miss the wake word entirely (live-tested). */
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
    /* Same 16 k PCM also drives the neural VAD (independent ring/chunk).
     * pcm/n here are the (possibly burst-trimmed) samples just queued above. */
    if (s_vad_ring) {
        const int16_t *src = pcm;
        int m = n;
        if ((uint32_t)m > VAD_RING_SAMP - 1) {
            src += (size_t)m - (VAD_RING_SAMP - 1);
            m = VAD_RING_SAMP - 1;
        }
        uint32_t vfree = (VAD_RING_SAMP - 1) - vad_avail();
        if ((uint32_t)m > vfree) {
            s_vad_tail = (s_vad_tail + ((uint32_t)m - vfree)) % VAD_RING_SAMP;
        }
        uint32_t vh = s_vad_head;
        uint32_t vfirst = VAD_RING_SAMP - vh;
        if ((uint32_t)m <= vfirst) {
            memcpy(s_vad_ring + vh, src, (size_t)m * sizeof(int16_t));
        } else {
            memcpy(s_vad_ring + vh, src, vfirst * sizeof(int16_t));
            memcpy(s_vad_ring, src + vfirst,
                   ((size_t)m - vfirst) * sizeof(int16_t));
        }
        s_vad_head = (vh + (uint32_t)m) % VAD_RING_SAMP;
    }
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
 * avoids retriggering on the utterance tail. The same loop also drains the
 * VAD ring through VADNet (independent chunksize) and tracks the speech
 * streak — music/noise yields SILENCE, only real speech streaks. */
void wakeword_detect_task(void *arg)
{
    (void)arg;
    int16_t *frame = malloc((size_t)s_chunksize * sizeof(int16_t));
    if (!frame) {
        ESP_LOGE(TAG, "detect task frame OOM, exiting (wake disabled)");
        vTaskDelete(NULL);
        return;
    }
    int vad_chunk = (s_vad_chunksize > 0) ? s_vad_chunksize : 0;
    int16_t *vframe = NULL;
    if (s_vad_ready && vad_chunk > 0) {
        vframe = malloc((size_t)vad_chunk * sizeof(int16_t));
        if (!vframe) {
            ESP_LOGE(TAG, "vad frame OOM — speech confirm disabled");
            vad_chunk = 0;
        }
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
        /* Cooldown suppresses WakeNet RETRIGGERS only. It must NOT skip the
         * VAD block below: doing so made speech confirmation arrive ~3 s
         * late (exactly the cooldown), losing the words said right after
         * "Alexa". */
        if ((int32_t)(now - cooldown_until) >= 0) {
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

        /* VADNet drain: every available frame, outside the lock. */
        if (vframe) {
            while (1) {
                uint32_t va;
                portENTER_CRITICAL(&s_mux);
                va = vad_avail();
                portEXIT_CRITICAL(&s_mux);
                if (va < (uint32_t)vad_chunk) {
                    break;
                }
                portENTER_CRITICAL(&s_mux);
                uint32_t vt = s_vad_tail;
                uint32_t vfirst = VAD_RING_SAMP - vt;
                if ((uint32_t)vad_chunk <= vfirst) {
                    memcpy(vframe, s_vad_ring + vt,
                           (size_t)vad_chunk * sizeof(int16_t));
                } else {
                    memcpy(vframe, s_vad_ring + vt, vfirst * sizeof(int16_t));
                    memcpy(vframe + vfirst, s_vad_ring,
                           ((size_t)vad_chunk - vfirst) * sizeof(int16_t));
                }
                s_vad_tail = (vt + (uint32_t)vad_chunk) % VAD_RING_SAMP;
                portEXIT_CRITICAL(&s_mux);
                vad_state_t vs = s_vadn->detect(s_vad_data, vframe);
                portENTER_CRITICAL(&s_mux);
                if (vs == VAD_SPEECH) {
                    s_vad_streak++;
                    s_vad_last_speech = xTaskGetTickCount();
                    s_vad_now = true;
                } else {
                    s_vad_streak = 0;
                    s_vad_now = false;
                }
                portEXIT_CRITICAL(&s_mux);
            }
        }
    }
}

bool wakeword_vad_ready(void)
{
    return s_vad_ready;
}

uint32_t wakeword_vad_streak(void)
{
    return s_vad_streak;
}

TickType_t wakeword_vad_last_speech_tick(void)
{
    return s_vad_last_speech;
}

bool wakeword_vad_speech_now(void)
{
    return s_vad_now;
}

void wakeword_vad_reset(void)
{
    portENTER_CRITICAL(&s_mux);
    s_vad_streak = 0;
    s_vad_now = false;
    portEXIT_CRITICAL(&s_mux);
}

void wakeword_vad_flush(void)
{
    portENTER_CRITICAL(&s_mux);
    s_vad_tail = s_vad_head; /* discard buffered (echo) audio */
    s_vad_streak = 0;
    s_vad_now = false;
    s_vad_last_speech = 0;
    portEXIT_CRITICAL(&s_mux);
}
