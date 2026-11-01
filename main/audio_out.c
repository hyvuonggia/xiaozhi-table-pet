/* Speaker path: Opus decode at 16 kHz mono (== server rate, NO resample)
 * -> I2S1 STD TX mono 16 kHz to MAX98357A.
 *
 * Opus choice (recorded): espressif/esp_audio_codec direct impl API
 * (esp_opus_dec_*). Queue depth 16 frames (~960 ms, SPEC asks >= 8).
 * After enable, ~120 ms of silence is pushed before any real frame.
 */

#include "audio_out.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/i2s_std.h"

#include "decoder/impl/esp_opus_dec.h"

static const char *TAG = "audio_out";

/* ---- pins / format (SPEC section 2.2, exact) ---- */
#define SPK_I2S_NUM      I2S_NUM_1
#define SPK_GPIO_BCLK    15
#define SPK_GPIO_WS      16
#define SPK_GPIO_DOUT    7

#define SPK_RATE         16000
#define FRAME_SAMPLES    960          /* 60 ms @ 16 kHz */
#define OPUS_BUF_MAX     512
#define QUEUE_DEPTH      16           /* >= 8 frames required, ~960 ms here */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    uint8_t data[OPUS_BUF_MAX];
    uint16_t len;
} opus_item_t;

static i2s_chan_handle_t s_tx;
static TaskHandle_t s_task;
static QueueHandle_t s_q;
static volatile bool s_running;
static void *s_dec;
static int s_volume = 50;            /* SPEC 2.3: default <= 50 */
static uint32_t s_underruns;
static uint32_t s_drops;
/* Utterance-tail diagnostics (sentence-end distortion hunt): peak per played
 * frame, summarized when the queue goes idle. */
static uint32_t s_utt_frames;
static int s_utt_peak;
static UBaseType_t s_utt_qmin = 99;
static uint32_t s_utt_starves; /* played-frame gaps >120 ms (DMA starvation) */
static TickType_t s_utt_last_play;
/* Per-frame PCM fingerprints (FNV-1a over every 4th sample) for duplicate-tail
 * analysis; ring of last 48, dumped at utterance end. */
#define UTT_HASH_N 48
static uint32_t s_utt_hash[UTT_HASH_N];
static int s_utt_hash_i;

/* ---- helpers ---- */

static void apply_volume(int16_t *pcm, int n)
{
    if (s_volume >= 100) {
        return;
    }
    if (s_volume <= 0) {
        memset(pcm, 0, (size_t)n * sizeof(int16_t));
        return;
    }
    for (int i = 0; i < n; i++) {
        pcm[i] = (int16_t)(((int32_t)pcm[i] * s_volume) / 100);
    }
}

/* Finite-timeout write; on a miss the chunk is dropped after `retries`
 * bounded extra attempts (cold amp/DMA needs a nudge, SPEC section 5.3). */
static void write_all(const uint8_t *buf, size_t len, TickType_t per_try,
                      int retries)
{
    for (int attempt = 0; attempt <= retries && s_running && s_tx; attempt++) {
        size_t off = 0;
        bool chunk_ok = true;
        while (off < len && s_running && s_tx) {
            size_t bw = 0;
            if (i2s_channel_write(s_tx, buf + off, len - off, &bw, per_try)
                    != ESP_OK || bw == 0) {
                chunk_ok = false;
                break;
            }
            off += bw;
        }
        if (chunk_ok) {
            return;
        }
        s_underruns++;
        ESP_LOGW(TAG, "i2s write miss %u/%u try %d/%d (underruns=%lu)",
                 (unsigned)off, (unsigned)len, attempt + 1, retries + 1,
                 (unsigned long)s_underruns);
        if (attempt < retries) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

static void write_silence_ms(int ms)
{
    int total = (SPK_RATE * ms) / 1000;
    static uint8_t zeros[FRAME_SAMPLES * 2];
    memset(zeros, 0, sizeof(zeros));
    int left = total;
    while (left > 0 && s_running && s_tx) {
        int n = left > FRAME_SAMPLES ? FRAME_SAMPLES : left;
        write_all(zeros, (size_t)n * 2, pdMS_TO_TICKS(200), 3);
        left -= n;
    }
}

/* ---- speaker task (prio 5) ---- */

static void spk_task(void *arg)
{
    (void)arg;
    opus_item_t it;
    uint8_t *pcm_raw = malloc(FRAME_SAMPLES * 2 + 128);
    if (!pcm_raw) {
        ESP_LOGE(TAG, "spk task OOM, exiting (restart via audio_out_start)");
        s_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "speaker task running (prio 5, 16k mono, no resample)");
    while (s_running) {
        if (xQueueReceive(s_q, &it, pdMS_TO_TICKS(200)) != pdTRUE) {
            /* Idle: keep DMA fed with silence so the next frame starts clean. */
            if (s_utt_frames >= 5) {
                ESP_LOGI(TAG, "utterance end: frames=%lu maxpeak=%d qmin=%u starves=%lu underruns=%lu drops=%lu",
                         (unsigned long)s_utt_frames, s_utt_peak,
                         (unsigned)s_utt_qmin, (unsigned long)s_utt_starves,
                         (unsigned long)s_underruns, (unsigned long)s_drops);
                int n = s_utt_frames < UTT_HASH_N ? (int)s_utt_frames : UTT_HASH_N;
                int start = (s_utt_hash_i - n + UTT_HASH_N * 2) % UTT_HASH_N;
                char hline[UTT_HASH_N * 9 + 8];
                int p = 0;
                p += snprintf(hline + p, sizeof(hline) - (size_t)p, "pcmhash:");
                for (int i = 0; i < n && p < (int)sizeof(hline) - 10; i++) {
                    p += snprintf(hline + p, sizeof(hline) - (size_t)p, " %08lx",
                                  (unsigned long)s_utt_hash[(start + i) % UTT_HASH_N]);
                }
                ESP_LOGI(TAG, "%s", hline);
            }
            s_utt_frames = 0;
            s_utt_peak = 0;
            s_utt_qmin = 99;
            s_utt_starves = 0;
            s_utt_last_play = 0;
            s_utt_hash_i = 0;
            /* Pre-buffer next utterance (~300 ms) to ride out downlink
             * jitter: starting playback on a near-empty queue starves DMA
             * whenever a frame arrives >60 ms late (audible tail stutter).
             * Feed silence WHILE waiting: an unfed DMA holds its last sample
             * as DC, which drones through the speaker as an "echo" of the
             * final syllable. */
            if (s_running && s_q) {
                TickType_t t0 = xTaskGetTickCount();
                while (s_running && s_q
                       && uxQueueMessagesWaiting(s_q) < 5
                       && (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(500)) {
                    write_silence_ms(20);
                    vTaskDelay(pdMS_TO_TICKS(20)); /* realtime pacing */
                }
            }
            write_silence_ms(60);
            continue;
        }
        if (s_q) {
            UBaseType_t qn = uxQueueMessagesWaiting(s_q);
            if (qn < s_utt_qmin) {
                s_utt_qmin = qn;
            }
        }
        TickType_t nowplay = xTaskGetTickCount();
        if (s_utt_last_play != 0
            && nowplay - s_utt_last_play > pdMS_TO_TICKS(120)) {
            s_utt_starves++;
        }
        s_utt_last_play = nowplay;
        if (!s_dec) {
            s_drops++;
            continue;
        }
        esp_audio_dec_in_raw_t raw = {
            .buffer = it.data,
            .len = it.len,
            .consumed = 0,
            .frame_recover = ESP_AUDIO_DEC_RECOVERY_NONE,
        };
        esp_audio_dec_out_frame_t frame = {
            .buffer = pcm_raw,
            .len = FRAME_SAMPLES * 2 + 128,
            .needed_size = 0,
            .decoded_size = 0,
        };
        esp_audio_dec_info_t info;
        memset(&info, 0, sizeof(info));
        esp_audio_err_t r = esp_opus_dec_decode(s_dec, &raw, &frame, &info);
        if (r != ESP_AUDIO_ERR_OK || frame.decoded_size == 0) {
            s_drops++;
            ESP_LOGW(TAG, "opus decode rc=%d decoded=%lu (drops=%lu)", (int)r,
                     (unsigned long)frame.decoded_size, (unsigned long)s_drops);
            continue;
        }
        size_t samples = frame.decoded_size / 2;
        if (samples > FRAME_SAMPLES + 64) {
            samples = FRAME_SAMPLES + 64;
        }
        apply_volume((int16_t *)pcm_raw, (int)samples);
        int16_t *pcm16 = (int16_t *)pcm_raw;
        int peak = 0;
        uint32_t h = 2166136261u;
        for (size_t i = 0; i < samples; i += 7) {
            int v = pcm16[i] >= 0 ? pcm16[i] : -pcm16[i];
            if (v > peak) {
                peak = v;
            }
        }
        for (size_t i = 0; i < samples; i += 4) {
            h ^= (uint32_t)(uint16_t)pcm16[i];
            h *= 16777619u;
        }
        s_utt_hash[s_utt_hash_i % UTT_HASH_N] = h;
        s_utt_hash_i++;
        s_utt_frames++;
        if (peak > s_utt_peak) {
            s_utt_peak = peak;
        }
        if (peak > 30000) {
            ESP_LOGW(TAG, "near-clip frame: peak=%d samples=%u", peak, (unsigned)samples);
        }
        write_all(pcm_raw, samples * 2, pdMS_TO_TICKS(300), 1);
    }
    free(pcm_raw);
    s_task = NULL;
    vTaskDelete(NULL);
}

/* ---- public API ---- */

void audio_out_start(void)
{
    if (s_running) {
        return;
    }
    s_running = true;

    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(SPK_I2S_NUM, I2S_ROLE_MASTER);
    cc.dma_desc_num = 16; /* deep DMA: ride out preemption without write-miss */
    cc.dma_frame_num = 240;
    i2s_chan_handle_t tx = NULL;
    if (i2s_new_channel(&cc, &tx, NULL) != ESP_OK || !tx) {
        ESP_LOGE(TAG, "i2s_new_channel TX failed (speaker unavailable, continuing)");
        tx = NULL;
    } else {
        i2s_std_config_t sc = {
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SPK_RATE),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = SPK_GPIO_BCLK,
                .ws = SPK_GPIO_WS,
                .dout = SPK_GPIO_DOUT,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv = false,
                },
            },
        };
        if (i2s_channel_init_std_mode(tx, &sc) != ESP_OK) {
            ESP_LOGE(TAG, "i2s TX std init failed (speaker unavailable, continuing)");
            i2s_del_channel(tx);
            tx = NULL;
        } else if (i2s_channel_enable(tx) != ESP_OK) {
            ESP_LOGE(TAG, "i2s TX enable failed (speaker unavailable, continuing)");
            i2s_del_channel(tx);
            tx = NULL;
        }
    }
    s_tx = tx;

    esp_opus_dec_cfg_t dc = {
        .sample_rate = SPK_RATE,
        .channel = 1,
        .frame_duration = ESP_OPUS_DEC_FRAME_DURATION_60_MS,
        .self_delimited = false,
    };
    s_dec = NULL;
    if (esp_opus_dec_open(&dc, sizeof(dc), &s_dec) != ESP_AUDIO_ERR_OK || !s_dec) {
        ESP_LOGE(TAG, "opus decoder open failed (tone still works, continuing)");
        s_dec = NULL;
    } else {
        ESP_LOGI(TAG, "opus decoder ready (16k mono 60ms)");
    }

    if (!s_q) {
        s_q = xQueueCreate(QUEUE_DEPTH, sizeof(opus_item_t));
        if (!s_q) {
            ESP_LOGE(TAG, "playback queue OOM");
        }
    } else {
        opus_item_t drain;
        while (xQueueReceive(s_q, &drain, 0) == pdTRUE) {
        }
    }

    if (tx) {
        /* SPEC 5.3 rule 14: ~120 ms silence first (cold amp/DMA). */
        write_silence_ms(120);
        ESP_LOGI(TAG, "startup silence (120ms) pushed");
    }

    /* Opus decoder needs a large stack (same family as the encoder). */
    /* 24 KB stack in internal RAM starved the rest of the firmware (int free
     * fell to ~1.3 KB, WS writes failed, other tasks OOM'd). The speaker task
     * only decodes + writes DMA, so a PSRAM stack is fine here; the mic task
     * (realtime) stays internal. */
    if (tx && s_q && xTaskCreateWithCaps(spk_task, "spk", 24576, NULL, 5,
                                         &s_task, MALLOC_CAP_SPIRAM) != pdPASS) {
        /* No feeder task -> DMA underruns repeat the last buffer forever:
         * that is the loud continuous drone. Silence the amp instead. */
        ESP_LOGE(TAG, "speaker task create failed -> disabling I2S TX");
        s_task = NULL;
        i2s_channel_disable(tx);
        i2s_del_channel(tx);
        s_tx = NULL;
    }
    ESP_LOGI(TAG, "audio_out started (tx=%s dec=%s vol=%d)",
             tx ? "ok" : "DOWN", s_dec ? "ok" : "DOWN", s_volume);
}

void audio_out_stop(void)
{
    s_running = false;
    for (int i = 0; i < 50 && s_task; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (s_tx) {
        i2s_channel_disable(s_tx);
        i2s_del_channel(s_tx);
        s_tx = NULL;
    }
    if (s_dec) {
        esp_opus_dec_close(s_dec);
        s_dec = NULL;
    }
    ESP_LOGI(TAG, "audio_out stopped");
}

int audio_out_push_opus(const uint8_t *data, size_t len)
{
    if (!data || len == 0 || len > OPUS_BUF_MAX) {
        return -1;
    }
    if (!s_running || !s_q) {
        return -2;
    }
    opus_item_t it;
    it.len = (uint16_t)len;
    memcpy(it.data, data, len);
    if (xQueueSend(s_q, &it, 0) != pdTRUE) {
        s_drops++;
        return -3;
    }
    return 0;
}

void audio_out_tone(int freq_hz, int ms)
{
    if (freq_hz < 50) {
        freq_hz = 50;
    } else if (freq_hz > 4000) {
        freq_hz = 4000;
    }
    if (ms < 50) {
        ms = 50;
    } else if (ms > 5000) {
        ms = 5000;
    }
    if (!s_running || !s_tx) {
        ESP_LOGW(TAG, "tone skipped (speaker not running)");
        return;
    }
    int total = (SPK_RATE * ms) / 1000;
    /* Static: keeps large buffers off the caller's task stack. */
    static int16_t chunk[FRAME_SAMPLES];
    double phase = 0.0;
    double step = 2.0 * M_PI * (double)freq_hz / (double)SPK_RATE;
    int32_t amp = (28000 * s_volume) / 100;
    int left = total;
    while (left > 0 && s_running) {
        int n = left > FRAME_SAMPLES ? FRAME_SAMPLES : left;
        for (int i = 0; i < n; i++) {
            chunk[i] = (int16_t)(sin(phase) * (double)amp);
            phase += step;
            if (phase >= 2.0 * M_PI) {
                phase -= 2.0 * M_PI;
            }
        }
        write_all((const uint8_t *)chunk, (size_t)n * 2, pdMS_TO_TICKS(200), 1);
        left -= n;
    }
    ESP_LOGI(TAG, "tone %dHz %dms done (vol=%d underruns=%lu)", freq_hz, ms,
             s_volume, (unsigned long)s_underruns);
}

void audio_out_set_volume(int vol_0_100)
{
    if (vol_0_100 < 0) {
        vol_0_100 = 0;
    } else if (vol_0_100 > 100) {
        vol_0_100 = 100;
    }
    s_volume = vol_0_100;
    ESP_LOGI(TAG, "volume set to %d/100", s_volume);
}

uint32_t audio_out_underruns(void)
{
    return s_underruns + s_drops;
}
