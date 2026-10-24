/* Mic path: INMP441 on I2S0 STD RX stereo 24 kHz (LEFT slot picked;
 * the mic drives one slot) -> esp-dsp FIR anti-alias filter (dsps_fir_f32,
 * 32-tap lowpass fc~7 kHz) + integer 3:2 linear decimation -> 16 kHz
 * 60 ms frames (960 samples) -> Opus encode (espressif/esp_audio_codec,
 * 16 kHz mono) for uplink.
 *
 * Resampler choice (recorded): espressif/esp-dsp via component manager.
 * esp-dsp's dsps_resampler_ph/mr_* enforce samplerate_factor >= 1, i.e.
 * upsampling only, so the 24 k -> 16 k downsample is done as an esp-dsp
 * FIR anti-alias stage plus an exact integer 3:2 linear decimation.
 * Opus choice (recorded): espressif/esp_audio_codec direct impl API
 * (esp_opus_enc_* / esp_opus_dec_*), 16 kHz mono, 60 ms, CBR 32 kbps.
 *
 * SPEC section 5.2: an I2S read that yields zero bytes skips a 32 ms slice
 * and the task continues; the task never breaks/exits on read errors.
 * Partial reads are accepted as success (data is data). Later reads use a
 * short finite timeout. No infinite waits anywhere.
 */

#include "audio_in.h"
#include "audio_out.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/i2s_std.h"

#include "dsps_fir.h"

#include "encoder/impl/esp_opus_enc.h"

static const char *TAG = "audio_in";

/* ---- pins / format (SPEC section 2.2, exact) ---- */
#define MIC_I2S_NUM      I2S_NUM_0
#define MIC_GPIO_SCK     5
#define MIC_GPIO_WS      4
#define MIC_GPIO_SD      6

#define MIC_RATE         24000
#define OUT_RATE         16000
#define FRAME16_SAMPLES  960          /* 60 ms @ 16 kHz */
#define CHUNK24_SAMPLES  1440         /* 60 ms @ 24 kHz */

/* One 32 ms slice @ 24 kHz stereo 32-bit = 2*768 samples = 6144 bytes. */
#define SLICE_FRAMES     768
#define UPLINK_DEPTH     8
#define OPUS_BUF_MAX     512
#define OPUS_ENC_OUT_MAX 2048

typedef struct {
    uint8_t data[OPUS_BUF_MAX];
    uint16_t len;
} opus_item_t;

static i2s_chan_handle_t s_rx;
static TaskHandle_t s_task;
static QueueHandle_t s_uplink;
static volatile bool s_running;
static volatile TickType_t s_last_ok;
static volatile uint32_t s_rms_recent;
static void *s_enc;
static int s_enc_in_size;
static int s_enc_out_size;
static uint8_t s_enc_out[OPUS_ENC_OUT_MAX];
static uint32_t s_enc_frames;
static uint32_t s_enc_bytes;
static uint32_t s_last_enc_bytes;
static bool s_selftest_live;

/* 24 kHz anti-alias FIR (esp-dsp): 32-tap Hamming-windowed sinc,
 * fc ~7 kHz, DC gain 1. Persistent state keeps continuity across chunks. */
#define FIR_TAPS 32
static fir_f32_t s_fir;
/* S3 esp-dsp FIR requires 16-byte aligned coeffs AND delay. */
static float s_fir_coefs[FIR_TAPS] __attribute__((aligned(16))) = {
    -0.00021424f, 0.00194431f, -0.00105725f, -0.00327886f,
    0.00489406f, 0.00343861f, -0.01261977f, 0.00229690f,
    0.02206693f, -0.01951746f, -0.02613832f, 0.05364089f,
    0.01054374f, -0.11882851f, 0.07942144f, 0.50340751f,
    0.50340751f, 0.07942144f, -0.11882851f, 0.01054374f,
    0.05364089f, -0.02613832f, -0.01951746f, 0.02206693f,
    0.00229690f, -0.01261977f, 0.00343861f, 0.00489406f,
    -0.00327886f, -0.00105725f, 0.00194431f, -0.00021424f,
};
static float s_fir_delay[FIR_TAPS + 4] __attribute__((aligned(16)));
static bool s_fir_ready;

/* ---- helpers ---- */

static int16_t s32_to_s16(int32_t s)
{
    /* INMP441 ships 24-bit data left-justified in the 32-bit slot.
     * >>14: +12 dB vs plain >>16. (Was >>13/+18 dB: clipped in loud rooms —
     * server reported "mic rè", selftest RMS hit 11k with flat tops.) */
    int32_t v = s >> 14;
    if (v > 32767) {
        v = 32767;
    } else if (v < -32768) {
        v = -32768;
    }
    return (int16_t)v;
}

static void update_rms(const int16_t *pcm, int n)
{
    int64_t sum = 0;
    for (int i = 0; i < n; i++) {
        sum += pcm[i];
    }
    int32_t mean = (int32_t)(sum / n);
    uint64_t sq = 0;
    for (int i = 0; i < n; i++) {
        int32_t d = (int32_t)pcm[i] - mean;
        sq += (uint64_t)(d * d);
    }
    s_rms_recent = (uint32_t)sqrt((double)(sq / (uint64_t)n));
}

static void encode_frame(const int16_t *pcm16)
{
    update_rms(pcm16, FRAME16_SAMPLES);
    if (!s_enc) {
        return;
    }
    if (s_enc_in_size != FRAME16_SAMPLES * 2) {
        ESP_LOGW(TAG, "opus in_size %d != 1920, frame skipped",
                 s_enc_in_size);
        return;
    }
    esp_audio_enc_in_frame_t in = {
        .buffer = (uint8_t *)pcm16,
        .len = FRAME16_SAMPLES * 2,
    };
    esp_audio_enc_out_frame_t enc_out = {
        .buffer = s_enc_out,
        .len = sizeof(s_enc_out),
        .encoded_bytes = 0,
        .pts = (uint64_t)s_enc_frames * 60,
    };
    esp_audio_err_t r = esp_opus_enc_process(s_enc, &in, &enc_out);
    if (r != ESP_AUDIO_ERR_OK || enc_out.encoded_bytes == 0) {
        ESP_LOGW(TAG, "opus encode rc=%d bytes=%lu", (int)r,
                 (unsigned long)enc_out.encoded_bytes);
        return;
    }
    if (enc_out.encoded_bytes > OPUS_BUF_MAX) {
        ESP_LOGW(TAG, "opus frame too big (%lu), dropped",
                 (unsigned long)enc_out.encoded_bytes);
        return;
    }
    s_enc_frames++;
    s_enc_bytes += enc_out.encoded_bytes;
    s_last_enc_bytes = enc_out.encoded_bytes;
    if (s_uplink) {
        opus_item_t it;
        it.len = (uint16_t)enc_out.encoded_bytes;
        memcpy(it.data, s_enc_out, it.len);
        if (xQueueSend(s_uplink, &it, 0) != pdTRUE) {
            /* Uplink cares about the newest speech: drop oldest, retry. */
            opus_item_t drop;
            if (xQueueReceive(s_uplink, &drop, 0) == pdTRUE) {
                if (xQueueSend(s_uplink, &it, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "uplink queue stuck, frame dropped");
                }
            }
        }
    }
}

/* ---- mic task (prio 8, never exits on read errors) ---- */

static void mic_task(void *arg)
{
    (void)arg;
    bool first = true;
    bool dumped = false;
    uint32_t consec_fail = 0;
    uint32_t bytes_total = 0;
    uint32_t reads_ok = 0;
    TickType_t stat_tick = xTaskGetTickCount();

    int16_t *acc24 = malloc(CHUNK24_SAMPLES * 2 * sizeof(int16_t));
    int16_t *acc16 = malloc((FRAME16_SAMPLES + 128) * sizeof(int16_t));
    float *f_buf = malloc(CHUNK24_SAMPLES * sizeof(float));
    float *f_filt = malloc(CHUNK24_SAMPLES * sizeof(float));
    uint8_t *raw = malloc(SLICE_FRAMES * 8);
    if (!acc24 || !acc16 || !f_buf || !f_filt || !raw) {
        ESP_LOGE(TAG, "mic task OOM, exiting (restart via audio_in_start)");
        free(acc24);
        free(acc16);
        free(f_buf);
        free(f_filt);
        free(raw);
        s_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    size_t n24 = 0, n16 = 0;

    ESP_LOGI(TAG, "mic task running (prio 8, 24k->16k FIR+3:2, opus 60ms)");
    while (s_running) {
        size_t br = 0;
        /* First read waits >= 300 ms (DMA is empty at boot); all finite. */
        TickType_t to = first ? pdMS_TO_TICKS(500) : pdMS_TO_TICKS(120);
        if (!dumped) {
            memset(raw, 0xA5, SLICE_FRAMES * 8); /* DMA canary (diagnostic) */
        }
        esp_err_t r = i2s_channel_read(s_rx, raw, SLICE_FRAMES * 8, &br, to);
        first = false;
        if (br == 0) {
            /* SPEC 5.2 rule 8: skip a 32 ms slice, never break/exit. */
            consec_fail++;
            if (consec_fail == 1 || consec_fail % 50 == 0) {
                ESP_LOGW(TAG, "i2s read fail #%lu (%s), skipping 32ms",
                         (unsigned long)consec_fail, esp_err_to_name(r));
            }
            vTaskDelay(pdMS_TO_TICKS(32));
            continue;
        }
        /* Partial reads are success: data is data, process what arrived. */
        consec_fail = 0;
        s_last_ok = xTaskGetTickCount();
        bytes_total += br;
        reads_ok++;
        if (!dumped && br >= 32) {
            dumped = true;
            /* Canary check: 0xA5-filled words prove DMA never wrote (clock /
             * DMA issue); real data proves the mic path; all zeros with no
             * canary proves DMA delivers zeros (mic/wiring issue). */
            int32_t w[8];
            memcpy(w, raw, sizeof(w));
            ESP_LOGI(TAG, "raw dump rc=%s br=%u: %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx",
                     esp_err_to_name(r), (unsigned)br,
                     (unsigned long)(uint32_t)w[0],
                     (unsigned long)(uint32_t)w[1], (unsigned long)(uint32_t)w[2],
                     (unsigned long)(uint32_t)w[3], (unsigned long)(uint32_t)w[4],
                     (unsigned long)(uint32_t)w[5], (unsigned long)(uint32_t)w[6],
                     (unsigned long)(uint32_t)w[7]);
        }
        if (s_last_ok - stat_tick >= pdMS_TO_TICKS(5000)) {
            ESP_LOGI(TAG, "mic flow: %lu B in %lu reads over 5s",
                     (unsigned long)bytes_total, (unsigned long)reads_ok);
            bytes_total = 0;
            reads_ok = 0;
            stat_tick = s_last_ok;
        }

        size_t got = br / 8; /* stereo frames: [L0][R0][L1][R1]... */
        for (size_t i = 0; i < got; i++) {
            int32_t s;
            memcpy(&s, raw + i * 8, 4); /* LEFT slot only (INMP441 L/R=GND) */
            if (n24 < CHUNK24_SAMPLES * 2) {
                acc24[n24++] = s32_to_s16(s);
            }
            if (n24 >= CHUNK24_SAMPLES && s_fir_ready) {
                for (int k = 0; k < CHUNK24_SAMPLES; k++) {
                    f_buf[k] = (float)acc24[k] / 32768.0f;
                }
                if (dsps_fir_f32_ansi(&s_fir, f_buf, f_filt,
                                      CHUNK24_SAMPLES) != ESP_OK) {
                    ESP_LOGW(TAG, "fir failed, chunk dropped");
                } else {
                    /* Exact integer 3:2 linear decimation on the
                     * anti-aliased signal: out[2m]=in[3m],
                     * out[2m+1]=(in[3m+1]+in[3m+2])/2. */
                    for (int m = 0; m < CHUNK24_SAMPLES / 3; m++) {
                        float a = f_filt[3 * m];
                        float b = (f_filt[3 * m + 1] + f_filt[3 * m + 2]) * 0.5f;
                        float fa = a * 32767.0f;
                        float fb = b * 32767.0f;
                        if (fa > 32767.0f) {
                            fa = 32767.0f;
                        } else if (fa < -32768.0f) {
                            fa = -32768.0f;
                        }
                        if (fb > 32767.0f) {
                            fb = 32767.0f;
                        } else if (fb < -32768.0f) {
                            fb = -32768.0f;
                        }
                        if (n16 < FRAME16_SAMPLES + 128 - 1) {
                            acc16[n16++] = (int16_t)fa;
                            acc16[n16++] = (int16_t)fb;
                        }
                    }
                }
                size_t left = n24 - CHUNK24_SAMPLES;
                memmove(acc24, acc24 + CHUNK24_SAMPLES,
                        left * sizeof(int16_t));
                n24 = left;
                while (n16 >= FRAME16_SAMPLES) {
                    encode_frame(acc16);
                    size_t rem = n16 - FRAME16_SAMPLES;
                    memmove(acc16, acc16 + FRAME16_SAMPLES,
                            rem * sizeof(int16_t));
                    n16 = rem;
                }
            }
        }
    }

    free(acc24);
    free(acc16);
    free(f_buf);
    free(f_filt);
    free(raw);
    s_task = NULL;
    vTaskDelete(NULL);
}

/* ---- public API ---- */

void audio_in_start(void)
{
    if (s_running) {
        return;
    }
    s_running = true;
    s_last_ok = 0;

    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(MIC_I2S_NUM, I2S_ROLE_MASTER);
    cc.dma_desc_num = 8;
    cc.dma_frame_num = 512;
    i2s_chan_handle_t rx = NULL;
    if (i2s_new_channel(&cc, NULL, &rx) != ESP_OK || !rx) {
        ESP_LOGE(TAG, "i2s_new_channel RX failed (mic unavailable, continuing)");
        rx = NULL;
    } else {
        /* Stereo capture: the INMP441 drives one slot; the dump + RMS tell
         * us which. LEFT (even indices) is used per SPEC L/R=GND. */
        i2s_std_config_t sc = {
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MIC_RATE),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = MIC_GPIO_SCK,
                .ws = MIC_GPIO_WS,
                .dout = I2S_GPIO_UNUSED,
                .din = MIC_GPIO_SD,
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv = false,
                },
            },
        };
        if (i2s_channel_init_std_mode(rx, &sc) != ESP_OK) {
            ESP_LOGE(TAG, "i2s RX std init failed (mic unavailable, continuing)");
            i2s_del_channel(rx);
            rx = NULL;
        } else if (i2s_channel_enable(rx) != ESP_OK) {
            ESP_LOGE(TAG, "i2s RX enable failed (mic unavailable, continuing)");
            i2s_del_channel(rx);
            rx = NULL;
        }
    }
    s_rx = rx;

    if (dsps_fir_init_f32(&s_fir, s_fir_coefs, s_fir_delay,
                          FIR_TAPS) == ESP_OK) {
        s_fir_ready = true;
    } else {
        s_fir_ready = false;
        ESP_LOGE(TAG, "anti-alias FIR init failed (rms only, continuing)");
    }

    esp_opus_enc_config_t ec = {
        .sample_rate = OUT_RATE,
        .channel = 1,
        .bits_per_sample = 16,
        .bitrate = 32000,
        .frame_duration = ESP_OPUS_ENC_FRAME_DURATION_60_MS,
        .application_mode = ESP_OPUS_ENC_APPLICATION_VOIP,
        .complexity = 0,
        .enable_fec = false,
        .enable_dtx = false,
        .enable_vbr = false,
    };
    s_enc = NULL;
    s_enc_in_size = 0;
    s_enc_out_size = 0;
    if (esp_opus_enc_open(&ec, sizeof(ec), &s_enc) != ESP_AUDIO_ERR_OK || !s_enc) {
        ESP_LOGE(TAG, "opus encoder open failed (rms only, continuing)");
        s_enc = NULL;
    } else if (esp_opus_enc_get_frame_size(s_enc, &s_enc_in_size,
                                           &s_enc_out_size) != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "opus get_frame_size failed (rms only, continuing)");
        esp_opus_enc_close(s_enc);
        s_enc = NULL;
    } else {
        ESP_LOGI(TAG, "opus encoder ready (16k mono 60ms, 32kbps, in=%d out=%d)",
                 s_enc_in_size, s_enc_out_size);
        if (s_enc_in_size != FRAME16_SAMPLES * 2
            || s_enc_out_size > (int)sizeof(s_enc_out)) {
            ESP_LOGE(TAG, "opus frame size mismatch, encoder disabled");
            esp_opus_enc_close(s_enc);
            s_enc = NULL;
        }
    }

    if (!s_uplink) {
        s_uplink = xQueueCreate(UPLINK_DEPTH, sizeof(opus_item_t));
        if (!s_uplink) {
            ESP_LOGE(TAG, "uplink queue OOM");
        }
    }

    /* Opus encoder needs a large stack (measured: 16 KB overflows). */
    if (rx && xTaskCreate(mic_task, "mic", 32768, NULL, 8, &s_task) != pdPASS) {
        ESP_LOGE(TAG, "mic task create failed");
        s_task = NULL;
    }
    ESP_LOGI(TAG, "audio_in started (rx=%s enc=%s)",
             rx ? "ok" : "DOWN", s_enc ? "ok" : "DOWN");
}

void audio_in_stop(void)
{
    s_running = false;
    for (int i = 0; i < 50 && s_task; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (s_rx) {
        i2s_channel_disable(s_rx);
        i2s_del_channel(s_rx);
        s_rx = NULL;
    }
    if (s_enc) {
        esp_opus_enc_close(s_enc);
        s_enc = NULL;
    }
    s_fir_ready = false;
    ESP_LOGI(TAG, "audio_in stopped");
}

TickType_t audio_in_last_ok_tick(void)
{
    return s_last_ok;
}

int audio_in_pop_opus(uint8_t *buf, size_t cap, uint32_t timeout_ms)
{
    if (!buf || cap == 0) {
        return -1;
    }
    if (!s_running || !s_uplink) {
        return -1;
    }
    opus_item_t it;
    if (xQueueReceive(s_uplink, &it, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return 0;
    }
    if (it.len > cap) {
        return -2;
    }
    memcpy(buf, it.data, it.len);
    return (int)it.len;
}

uint32_t audio_in_recent_rms(void)
{
    return s_rms_recent;
}

void audio_in_opus_stats(uint32_t *frames, uint32_t *bytes, uint32_t *last_bytes)
{
    if (frames) {
        *frames = s_enc_frames;
    }
    if (bytes) {
        *bytes = s_enc_bytes;
    }
    if (last_bytes) {
        *last_bytes = s_last_enc_bytes;
    }
}

/* ---- self-test (no server needed, log-only, never panics) ---- */

static void selftest_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "selftest begin (server-independent)");

    audio_in_start();
    audio_out_start();

    uint32_t rms_peak = 0;
    for (int i = 0; i < 3; i++) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t r = audio_in_recent_rms();
        ESP_LOGI(TAG, "selftest mic rms t=%ds: %lu", i + 1, (unsigned long)r);
        if (r > rms_peak) {
            rms_peak = r;
        }
    }
    TickType_t last = audio_in_last_ok_tick();
    ESP_LOGI(TAG, "selftest mic: peak_rms=%lu last_ok_tick=%lu reads=%s",
             (unsigned long)rms_peak, (unsigned long)last,
             last ? "YES (hearing)" : "NONE (check mic wiring)");

    uint32_t u0 = audio_out_underruns();
    audio_out_tone(440, 500);
    uint32_t u1 = audio_out_underruns();
    ESP_LOGI(TAG, "selftest speaker: 440Hz 0.5s played, underruns %lu->%lu",
             (unsigned long)u0, (unsigned long)u1);

    uint32_t f, b, lb;
    audio_in_opus_stats(&f, &b, &lb);
    ESP_LOGI(TAG, "selftest opus: frames=%lu total_bytes=%lu avg_B_per_frame=%lu last=%lu",
             (unsigned long)f, (unsigned long)b,
             f ? (unsigned long)(b / f) : 0, (unsigned long)lb);

    ESP_LOGI(TAG, "selftest done (log-only, no panic)");
    s_selftest_live = false;
    vTaskDelete(NULL);
}

void audio_selftest(void)
{
    if (s_selftest_live) {
        return;
    }
    s_selftest_live = true;
    /* 8 KB: audio_out_tone + i2s driver call chain need headroom. */
    if (xTaskCreate(selftest_task, "aud_self", 8192, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "selftest task create failed (continuing)");
        s_selftest_live = false;
    }
}
