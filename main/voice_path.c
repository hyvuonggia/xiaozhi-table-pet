/* Voice path P2b: Opus uplink task + WS downlink dispatch.
 * See voice_path.h for the contract. All parses NULL-safe/cJSON,
 * soft-fail; every wait/timeout finite; no banned tokens (CI-checked).
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_websocket_client.h"
#include "cJSON.h"

#include "audio_in.h"
#include "audio_out.h"
#include "msg_filter.h"
#include "display.h"
#include "servo.h"
#include "wakeword.h"
#include "voice_path.h"

static const char *TAG = "voice";

#define VP_UPLINK_PRIO      4
#define VP_UPLINK_STACK     3072
#define VP_OPUS_BUF         512   /* 240 B typical 60 ms frame; headroom */
#define VP_POP_TIMEOUT_MS   50
#define VP_SEND_TIMEOUT_MS  200
#define VP_TEXT_TIMEOUT_MS  2000
#define VP_TEXT_MAX         4096  /* parse window for control JSON */

#define VP_LISTEN_START \
    "{\"type\":\"listen\",\"state\":\"start\",\"mode\":\"auto\"}"
#define VP_LISTEN_STOP \
    "{\"type\":\"listen\",\"state\":\"stop\",\"mode\":\"auto\"}"

static esp_websocket_client_handle_t s_ws;
static void (*s_open_cb)(void);  /* main.c: open audio channel on wake */
static void (*s_close_cb)(void); /* main.c: drop audio channel when idle */
static bool s_task_on;
static bool s_session;    /* hello with session_id seen */
static bool s_listening;  /* listen start sent for this session */
static bool s_speaking;   /* tts start seen, no stop yet: half-duplex echo
                             guard (SPEC: no AEC on device; server hears its
                             own TTS via mic otherwise). Uplink pauses while
                             true; barge-in deferred (user waits ~2 s). */
static uint32_t s_tx_frames, s_tx_bytes, s_tx_drop, s_tx_gated;
static uint32_t s_rx_muted;
static TickType_t s_mute_until; /* server-error utterance in flight: drop its
                                   audio (4 s window from the text frame). */

/* Server-side error leaks that must never reach the speaker: leaked tool
 * identifiers ("self_head_...") and the bridge's Chinese "不存在" notices.
 * Legit Vietnamese replies never contain these byte sequences. */
static bool vp_tts_is_server_error(const char *text)
{
    if (!text) {
        return false;
    }
    return strstr(text, "self_head") != NULL || strstr(text, "不存在") != NULL;
}
static uint32_t s_rx_frames, s_rx_drop;
static size_t s_rx_len_min = 9999, s_rx_len_max = 0;
static uint32_t s_rx_len_sum;

/* Best-effort text send, finite timeout, log-only on failure.
 * Returns the driver rc (>= 0 sent, < 0 failed) so callers can gate flags. */
static int vp_send_text(const char *json)
{
    if (!s_ws || !json) {
        return -1;
    }
    int rc = esp_websocket_client_send_text(s_ws, json, strlen(json),
                                            pdMS_TO_TICKS(VP_TEXT_TIMEOUT_MS));
    if (rc < 0) {
        ESP_LOGW(TAG, "text send failed: %.80s", json);
    }
    return rc;
}

static void vp_send_listen_start(void)
{
    /* F5: flag ONLY on rc >= 0. On failure the flag stays clear so the
     * uplink loop retries next iteration (server-side VAD segments). */
    if (vp_send_text(VP_LISTEN_START) >= 0) {
        s_listening = true;
        ESP_LOGI(TAG, "listen start sent (mode=auto)");
    }
}

/* Explicit stop on window close: makes the server finalize the utterance and
 * answer immediately instead of waiting for its own VAD timeout. */
static void vp_send_listen_stop(void)
{
    if (vp_send_text(VP_LISTEN_STOP) >= 0) {
        s_listening = false;
        ESP_LOGI(TAG, "listen stop sent (window closed)");
    }
}

/* Wake-word conversation window (WAKE lane, neural-VAD gated).
 *
 * Two-stage design (live-tested edge cases 2026-10-31):
 *  1. WAKE arms the window locally (screen: DANG NGHE) but sends NOTHING.
 *     If the user stays silent for VP_ARM_MS the window disarms silently:
 *     no listen start/stop, so the server never hears an empty utterance and
 *     never says goodbye ("tạm biệt sếp") after a bare "Alexa".
 *  2. The first real SPEECH — VADNet neural VAD streak, never raw energy —
 *     sends listen{start} and streams. The window then stays open while
 *     VAD reports speech, and through TTS playback (echo of our own speaker
 *     must NOT hold it open forever), closing VP_WINDOW_QUIET_MS after the
 *     last speech/answer.
 *
 * Energy alone can NEVER open or extend the stream: in a noisy room
 * (TV/music, ambient RMS 2000-3500) music trips any RMS threshold, the
 * server ASR hallucinates ("đây"), and the user hears nonsense. VADNet was
 * trained to separate speech from music — only its SPEECH streak counts. */
static bool s_armed;              /* wake seen, waiting for first speech */
static bool s_streaming;          /* listen start sent, uplink live */
static TickType_t s_arm_tick;     /* wake moment */
static TickType_t s_mute_until;   /* drop audio until here (wake word tail) */
static TickType_t s_quiet_since;  /* last tick with real (non-echo) speech */
static TickType_t s_window_open_tick;
static bool s_screen_listen;      /* last display_set_listening() value */
static bool s_vad_unready_warned; /* throttled "VAD missing" log */
static bool s_listen_sent;        /* listen start sent for this window */
static bool s_vad_seen;           /* VAD confirmed real speech this window */
static uint32_t s_speech_ms;      /* real VAD speech accumulated this window */
static bool s_closing;            /* window closed, waiting for the reply */
static bool s_followup;           /* after a reply: listen for a follow-up */
static TickType_t s_followup_until;
static TickType_t s_last_tts_tick; /* last tts text/audio seen (reply end) */
static bool s_reply_done;          /* tts stop seen for this turn */
static TickType_t s_close_at;     /* when it is safe to drop the channel */
#define VP_ARM_MS          5000u /* wait this long for the user to start */
/* The wake word itself is still in the mic/VAD path right after WAKE.
 * Ignore that tail, then require neural-VAD speech (not room noise) to
 * confirm the user is actually talking. */
#define VP_ARM_IGNORE_MS    900u /* whole wake word, not just its tail */
/* VADNet frames are ~30 ms: 3 consecutive SPEECH frames (~90 ms) confirm.
 * Isolated music blips never streak; real speech always does. */
#define VP_VAD_CONFIRM_FRAMES 2u
/* While streaming, VAD speech within the last 500 ms counts as activity;
 * longer gaps let the 2 s quiet timer below run out and close the window. */
#define VP_VAD_HOLD_MS      500u
#define VP_WINDOW_QUIET_MS 2000u /* server VAD needs the stream through the
                                     end of the sentence */
#define VP_WINDOW_MIN_MS   6000u /* short utterances are not cut off */
/* After a reply the channel stays open briefly so the user can answer without
 * repeating the wake word; kept below the server's 10 s no-voice timeout. */
#define VP_FOLLOWUP_MS     30000u /* stay in the conversation this long */

/* Small PSRAM ring used only while a channel is (re)opening: keeps the first
 * words of a follow-up from being clipped. Flushed paced (server drops on a
 * multi-second flood). */
#define VP_PRE_N 20 /* 20 x 60 ms = 1.2 s */
static uint8_t *s_pre_buf;
static uint16_t *s_pre_len;
static int s_pre_head, s_pre_count;

static void vp_pre_reset(void)
{
    s_pre_head = 0;
    s_pre_count = 0;
}

static void vp_pre_push(const uint8_t *data, size_t len)
{
    if (!s_pre_buf || !s_pre_len || len == 0 || len > VP_OPUS_BUF) {
        return;
    }
    int idx = (s_pre_head + s_pre_count) % VP_PRE_N;
    memcpy(s_pre_buf + (size_t)idx * VP_OPUS_BUF, data, len);
    s_pre_len[idx] = (uint16_t)len;
    if (s_pre_count < VP_PRE_N) {
        s_pre_count++;
    } else {
        s_pre_head = (s_pre_head + 1) % VP_PRE_N;
    }
}

static void vp_pre_flush(void)
{
    if (!s_pre_buf || !s_ws || s_pre_count == 0) {
        vp_pre_reset();
        return;
    }
    for (int i = 0; i < s_pre_count; i++) {
        int idx = (s_pre_head + i) % VP_PRE_N;
        if (s_pre_len[idx] == 0) {
            continue;
        }
        if (esp_websocket_client_send_bin(s_ws,
                (const char *)(s_pre_buf + (size_t)idx * VP_OPUS_BUF),
                s_pre_len[idx], pdMS_TO_TICKS(VP_SEND_TIMEOUT_MS)) >= 0) {
            s_tx_frames++;
        } else {
            s_tx_drop++;
        }
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    ESP_LOGI(TAG, "pre-stream flush: %d frames", s_pre_count);
    vp_pre_reset();
}

static void vp_set_screen_listen(bool on)
{
    if (on != s_screen_listen) {
        s_screen_listen = on;
        display_set_listening(on);
    }
}

/* Uplink: drain audio_in_pop_opus() -> WS BINARY. Never blocks the mic task
 * (pop has a short timeout); WS congestion drops the frame and counts it. */
static void vp_uplink_task(void *arg)
{
    (void)arg;
    uint8_t buf[VP_OPUS_BUF];
    ESP_LOGI(TAG, "uplink task on (prio %d)", VP_UPLINK_PRIO);
    while (1) {
        /* Wake detection must run even while the audio channel is CLOSED
         * (channel opens on wake), so this check comes before the session
         * guard. Missing it made "Alexa" dead until a session existed. */
        uint32_t rms = audio_in_recent_rms(); /* log-only; gating is VAD */
        TickType_t now = xTaskGetTickCount();

        /* Stage 1: wake arms the window locally (no protocol traffic). */
        if (wakeword_consume()) {
            if (s_speaking) {
                /* Barge-in not supported: ignore wake while the robot talks. */
                ESP_LOGI(TAG, "WAKE ignored (robot is speaking)");
            } else {
                /* Stream IMMEDIATELY from the wake word: this is what keeps a
                 * fast "Alexa, mấy giờ rồi" intact. VAD is used to decide when
                 * to close / whether any real speech happened, never as a
                 * gate that would clip the first words (a catch-up burst made
                 * the server drop the socket: transport_poll_write(0)). */
                s_armed = true;
                s_streaming = true;
                s_closing = false;
                s_followup = false;
                s_listen_sent = false;
                s_vad_seen = false;
                s_speech_ms = 0;
                s_arm_tick = now;
                s_quiet_since = now;
                s_window_open_tick = now;
                wakeword_vad_reset(); /* confirm streak starts AFTER wake */
                s_mute_until = now + pdMS_TO_TICKS(VP_ARM_IGNORE_MS);
                ESP_LOGI(TAG, "WAKE -> streaming (rms=%lu vad=%d)",
                         (unsigned long)rms,
                         wakeword_vad_ready() ? 1 : 0);
                if (s_open_cb) {
                    s_open_cb(); /* audio channel exists only during talk */
                }
            }
        }

        if (s_streaming) {
            if (wakeword_vad_streak() >= VP_VAD_CONFIRM_FRAMES) {
                s_vad_seen = true;
            }
            if (wakeword_vad_speech_now()
                && (int32_t)(now - s_arm_tick) >= (int32_t)pdMS_TO_TICKS(VP_ARM_IGNORE_MS)) {
                s_speech_ms += 60; /* real user speech, wake word excluded */
            }
            /* No real speech after wake: abort silently. Closing the socket
             * (instead of listen stop) means the server cannot deliver its
             * "tạm biệt sếp" end_prompt. */
            if (!s_vad_seen
                && (int32_t)(now - s_arm_tick) > (int32_t)pdMS_TO_TICKS(VP_ARM_MS)) {
                s_streaming = false;
                s_armed = false;
                vp_set_screen_listen(false);
                if (s_speaking) {
                    display_set_speaking(false);
                    s_speaking = false;
                }
                display_set_thinking(false);
                ESP_LOGI(TAG, "no speech after wake: silent abort");
                if (s_close_cb) {
                    s_close_cb();
                }
                continue;
            }
            if (!wakeword_vad_ready() && !s_vad_unready_warned) {
                s_vad_unready_warned = true;
                ESP_LOGW(TAG, "VAD model missing — window cannot confirm speech");
            }
        }
        if (s_streaming) {
            if (s_speaking) {
                /* Our own speaker is audible to the mic: never let the echo
                 * extend the window, but keep it open for the reply. */
                s_quiet_since = now;
            } else if (wakeword_vad_speech_now()
                       || (int32_t)(now - wakeword_vad_last_speech_tick())
                          < (int32_t)pdMS_TO_TICKS(VP_VAD_HOLD_MS)) {
                /* Neural speech holds the window; music/TV silence lets the
                 * quiet timer below run out and close it. */
                s_quiet_since = now;
            } else if ((int32_t)(now - s_window_open_tick) >= (int32_t)pdMS_TO_TICKS(VP_WINDOW_MIN_MS)
                       && (int32_t)(now - s_quiet_since) >= (int32_t)pdMS_TO_TICKS(VP_WINDOW_QUIET_MS)) {
                s_streaming = false;
                s_armed = false;
                if (s_speech_ms < 600) {
                    /* Only a blip (click/TV/noise): never ask the server to
                     * answer it. Dropping the socket is silent by design. */
                    ESP_LOGI(TAG, "window closed: only %lums speech, silent abort",
                             (unsigned long)s_speech_ms);
                    vp_set_screen_listen(false);
                    if (s_speaking) {
                        display_set_speaking(false);
                        s_speaking = false;
                    }
                    display_set_thinking(false);
                    s_closing = false;
                    if (s_close_cb) {
                        s_close_cb();
                    }
                    continue;
                }
                vp_send_listen_stop();
                display_set_thinking(true); /* DANG NGHI until the answer */
                /* Do NOT drop the channel yet: the server answers right after
                 * listen stop, and closing here cut the reply off (live-tested
                 * fast_speech: STT ok, 0 TTS). Wait for the answer to finish. */
                s_closing = true;
                s_reply_done = false;
                s_last_tts_tick = 0;
                s_close_at = now + pdMS_TO_TICKS(15000);
                ESP_LOGI(TAG, "window closed (vad silence %ums), awaiting reply",
                         (unsigned)VP_WINDOW_QUIET_MS);
            }
        }

        vp_set_screen_listen(s_armed || s_streaming || s_followup);

        if (s_closing) {
            if (s_speaking) {
                s_close_at = now + pdMS_TO_TICKS(1000); /* reply still playing */
            } else if (s_reply_done || (int32_t)(now - s_close_at) >= 0) {
                /* Multi-turn UX: keep the channel open for a follow-up so the
                 * user can answer WITHOUT saying "Alexa" again (real-user
                 * report). Close after the window, before the server's 10 s
                 * no-voice timeout can fire its goodbye. */
                s_closing = false;
                if (s_speaking) {
                    display_set_speaking(false);
                    s_speaking = false;
                }
                s_followup = true;
                display_set_thinking(false);
                vp_pre_reset();
                s_followup_until = now + pdMS_TO_TICKS(VP_FOLLOWUP_MS);
                s_vad_seen = false;
                s_speech_ms = 0;
                s_mute_until = 0;
                wakeword_vad_reset();
                if (s_close_cb) {
                    s_close_cb(); /* close before the server's 10 s goodbye */
                }
                ESP_LOGI(TAG, "follow-up window %u ms, channel closed (no wake needed)",
                         (unsigned)VP_FOLLOWUP_MS);
            }
        }

        if (s_followup) {
            if (wakeword_vad_streak() >= VP_VAD_CONFIRM_FRAMES) {
                /* User answered: reopen the channel and continue, no wake word.
                 * Audio is buffered until the session is live (no clipping). */
                s_followup = false;
                s_streaming = true;
                s_armed = false;
                s_listen_sent = false;
                s_vad_seen = true;
                s_speech_ms = 600; /* already proven speech */
                s_quiet_since = now;
                s_window_open_tick = now;
                s_mute_until = 0;
                if (s_open_cb) {
                    s_open_cb();
                }
                ESP_LOGI(TAG, "follow-up speech -> reopen + streaming (no wake)");
            } else if ((int32_t)(now - s_followup_until) >= 0) {
                s_followup = false;
                if (s_close_cb) {
                    s_close_cb();
                }
                ESP_LOGI(TAG, "follow-up window over: channel closed");
            }
        }

        /* Do NOT drain the mic queue while we are not sending: the queue
         * keeps the newest ~480 ms, so when streaming starts the words said
         * immediately after "Alexa" are still there (fast speech no longer
         * loses its first half). */
        if (s_streaming && !s_speaking && (!s_session || !s_ws)) {
            /* Channel is still connecting: buffer the first words. */
            int pn = audio_in_pop_opus(buf, sizeof(buf), 20);
            if (pn > 0) {
                vp_pre_push(buf, (size_t)pn);
            }
            continue;
        }
        if (!s_streaming || s_speaking || !s_session || !s_ws) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (!s_listen_sent) {
            vp_send_listen_start(); /* session is live now */
            s_listen_sent = s_listening;
            vp_pre_flush(); /* words said while the channel was opening */
        }
        if (s_mute_until != 0
            && (int32_t)(now - s_mute_until) < 0) {
            /* Do not send the wake word itself: the server would transcribe
             * "Alexa" and answer it ("Chào sếp..."). Keep the queue fresh. */
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* Once real speech has been heard, stop transmitting during silence:
         * trailing room noise was transcribed as "đây" and the server answered
         * it ("Dạ em đây..."), so every question got a second reply. */
        if (s_vad_seen) {
            TickType_t last_sp = wakeword_vad_last_speech_tick();
            if ((int32_t)(now - last_sp) > (int32_t)pdMS_TO_TICKS(400)) {
                audio_in_pop_opus(buf, sizeof(buf), 0); /* keep queue fresh */
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
        }

        int n = audio_in_pop_opus(buf, sizeof(buf), VP_POP_TIMEOUT_MS);
        if (n <= 0) {
            continue; /* timeout/empty/not-running: no frame, no send */
        }
        int rc = esp_websocket_client_send_bin(s_ws, (const char *)buf, n,
                                               pdMS_TO_TICKS(VP_SEND_TIMEOUT_MS));
        if (rc < 0) {
            s_tx_drop++;
        } else {
            s_tx_frames++;
            s_tx_bytes += (uint32_t)n;
            if (s_tx_frames == 1 || s_tx_frames % 50 == 0) {
                ESP_LOGI(TAG, "up opus: frames=%lu bytes=%lu dropped=%lu gated=%lu",
                         (unsigned long)s_tx_frames,
                         (unsigned long)s_tx_bytes,
                         (unsigned long)s_tx_drop,
                         (unsigned long)s_tx_gated);
            }
        }
    }
}

void voice_path_set_ws(esp_websocket_client_handle_t ws)
{
    s_ws = ws;
    if (!ws) {
        /* Only the handle/session die here. Do NOT clear armed/streaming:
         * ws_open_for_wake() is called from inside the wake branch, and
         * wiping the state there made the whole window a no-op. */
        s_session = false;
        s_listen_sent = false;
    }
}

void voice_path_start(esp_websocket_client_handle_t ws,
                      void (*open_cb)(void), void (*close_cb)(void))
{
    s_ws = ws;
    s_open_cb = open_cb;
    s_close_cb = close_cb;
    if (!s_pre_buf) {
        s_pre_buf = heap_caps_malloc((size_t)VP_PRE_N * VP_OPUS_BUF,
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_pre_len = calloc(VP_PRE_N, sizeof(uint16_t));
        if (!s_pre_buf || !s_pre_len) {
            free(s_pre_buf);
            free(s_pre_len);
            s_pre_buf = NULL;
            s_pre_len = NULL;
            ESP_LOGW(TAG, "no pre-stream ring (follow-up may clip)");
        }
    }
    wakeword_init(); /* idempotent; soft-fails to "no wake" if PSRAM/model bad */
    if (s_task_on) {
        return;
    }
    if (xTaskCreateWithCaps(vp_uplink_task, "vp_up", VP_UPLINK_STACK,
                        NULL, VP_UPLINK_PRIO, NULL, MALLOC_CAP_SPIRAM) != pdPASS) {
        ESP_LOGW(TAG, "uplink task OOM (downlink still live)");
        return;
    }
    s_task_on = true;
}

void voice_path_on_ws_connected(void)
{
    ESP_LOGI(TAG, "ws connected (session re-arms on hello)");
}

void voice_path_on_ws_disconnected(void)
{
    /* Stop streaming; uplink task idles until the next hello.
     * Re-listen is automatic on session re-establish. */
    s_session = false;
    s_listening = false;
    if (s_speaking) {
        display_set_speaking(false); /* else the screen sticks on DANG NOI */
    }
    display_set_thinking(false);
    s_speaking = false; /* never stay half-duplex-gated across sessions */
    s_armed = false;
    s_streaming = false; /* fresh WAKE required next session */
    s_closing = false;
    s_followup = false;
    s_vad_unready_warned = false;
    if (s_screen_listen) {
        s_screen_listen = false;
        display_set_listening(false);
    }
    ESP_LOGI(TAG, "ws down: streaming stopped (up=%lu drop=%lu down=%lu drop=%lu)",
             (unsigned long)s_tx_frames, (unsigned long)s_tx_drop,
             (unsigned long)s_rx_frames, (unsigned long)s_rx_drop);
}

void voice_path_relisten(void)
{
    if (!s_ws || !s_session) {
        ESP_LOGW(TAG, "relisten: no session (ignored)");
        return;
    }
    vp_send_listen_start();
}

bool voice_path_is_listening(void)
{
    /* Mic must be alive whenever a session is live: with wake-word gating a
     * dead mic means the robot can never hear "Alexa" again (deaf forever),
     * so the watchdog needs this true outside the conversation window too. */
    return s_session;
}

/* --- MCP (JSON-RPC over WS "mcp" frames, E1/E2) --- */

/* method/id may sit at top level or inside "payload" (server variants);
 * NULL-safe, returns NULL when absent. */
static cJSON *vp_mcp_field(cJSON *root, const char *key)
{
    cJSON *v = cJSON_GetObjectItemCaseSensitive(root, key);
    if (v) {
        return v;
    }
    cJSON *p = cJSON_GetObjectItemCaseSensitive(root, "payload");
    if (cJSON_IsObject(p)) {
        return cJSON_GetObjectItemCaseSensitive(p, key);
    }
    return NULL;
}

/* Reply carries BOTH envelopes ({"jsonrpc","id","result"} and
 * {"type":"mcp","id",...}) so either server parser accepts it. */
static void vp_mcp_reply(cJSON *id, cJSON *result, cJSON *error)
{
    cJSON *resp = cJSON_CreateObject();
    if (!resp) {
        return;
    }
    cJSON_AddStringToObject(resp, "jsonrpc", "2.0");
    cJSON_AddStringToObject(resp, "type", "mcp");
    if (cJSON_IsNumber(id) || cJSON_IsString(id)) {
        cJSON_AddItemToObject(resp, "id", cJSON_Duplicate(id, 1));
    }
    if (result) {
        cJSON_AddItemToObject(resp, "result", result);
    } else if (error) {
        cJSON_AddItemToObject(resp, "error", error);
    }
    char *s = cJSON_PrintUnformatted(resp);
    cJSON_Delete(resp);
    if (!s) {
        if (result) {
            cJSON_Delete(result);
        }
        if (error) {
            cJSON_Delete(error);
        }
        return;
    }
    vp_send_text(s);
    free(s);
}

static void vp_on_mcp(cJSON *root)
{
    cJSON *method = vp_mcp_field(root, "method");
    cJSON *id = vp_mcp_field(root, "id");
    const char *m = cJSON_IsString(method) ? method->valuestring : NULL;
    if (!m) {
        ESP_LOGW(TAG, "mcp: no method (ignored)");
        return;
    }
    ESP_LOGI(TAG, "mcp method=%s", m);
    if (strcmp(m, "initialize") == 0) {
        cJSON *r = cJSON_CreateObject();
        if (!r) {
            return;
        }
        cJSON_AddStringToObject(r, "protocolVersion", "2024-11-05");
        cJSON *cap = cJSON_CreateObject();
        cJSON *tools = cJSON_CreateObject();
        if (!cap || !tools) {
            cJSON_Delete(r);
            if (cap) {
                cJSON_Delete(cap);
            }
            if (tools) {
                cJSON_Delete(tools);
            }
            return;
        }
        cJSON_AddItemToObject(cap, "tools", tools);
        cJSON_AddItemToObject(r, "capabilities", cap);
        cJSON *info = cJSON_CreateObject();
        if (info) {
            cJSON_AddStringToObject(info, "name", "xiaozhi-table-pet");
            cJSON_AddStringToObject(info, "version", "0.0.1");
            cJSON_AddItemToObject(r, "serverInfo", info);
        }
        vp_mcp_reply(id, r, NULL);
    } else if (strcmp(m, "tools/list") == 0) {
        cJSON *r = cJSON_CreateObject();
        cJSON *arr = cJSON_CreateArray();
        if (!r || !arr) {
            if (r) {
                cJSON_Delete(r);
            }
            if (arr) {
                cJSON_Delete(arr);
            }
            return;
        }
        const char *names[] = { "self_head_nod", "self_head_shake" };
        const char *descs[] = {
            "Nod the robot head exactly once (vertical down-up)",
            "Shake the robot head exactly once (horizontal left-right)",
        };
        for (int i = 0; i < 2; i++) {
            cJSON *t = cJSON_CreateObject();
            cJSON *schema = cJSON_CreateObject();
            if (!t || !schema) {
                if (t) {
                    cJSON_Delete(t);
                }
                if (schema) {
                    cJSON_Delete(schema);
                }
                continue;
            }
            cJSON_AddStringToObject(t, "name", names[i]);
            cJSON_AddStringToObject(t, "description", descs[i]);
            cJSON_AddStringToObject(schema, "type", "object");
            cJSON_AddItemToObject(t, "inputSchema", schema);
            cJSON_AddItemToArray(arr, t);
        }
        cJSON_AddItemToObject(r, "tools", arr);
        vp_mcp_reply(id, r, NULL);
    } else if (strcmp(m, "tools/call") == 0) {
        cJSON *params = vp_mcp_field(root, "params");
        const char *name = NULL;
        if (cJSON_IsObject(params)) {
            cJSON *nm = cJSON_GetObjectItemCaseSensitive(params, "name");
            name = cJSON_IsString(nm) ? nm->valuestring : NULL;
        }
        bool handled = false;
        if (name && strcmp(name, "self_head_nod") == 0) {
            servo_nod_once(); /* exactly once; 3 s debounce in servo layer */
            handled = true;
        } else if (name && strcmp(name, "self_head_shake") == 0) {
            servo_shake_once();
            handled = true;
        }
        ESP_LOGI(TAG, "tools/call %s: %s", name ? name : "(no name)",
                 handled ? "executed once" : "unknown tool");
        cJSON *r = cJSON_CreateObject();
        cJSON *content = cJSON_CreateArray();
        cJSON *item = cJSON_CreateObject();
        if (!r || !content || !item) {
            if (r) {
                cJSON_Delete(r);
            }
            if (content) {
                cJSON_Delete(content);
            }
            if (item) {
                cJSON_Delete(item);
            }
            return;
        }
        cJSON_AddStringToObject(item, "type", "text");
        cJSON_AddStringToObject(item, "text", handled ? "done" : "unknown tool");
        cJSON_AddItemToArray(content, item);
        cJSON_AddItemToObject(r, "content", content);
        cJSON_AddBoolToObject(r, "isError", handled ? 0 : 1);
        vp_mcp_reply(id, r, NULL);
    } else {
        cJSON *e = cJSON_CreateObject();
        if (!e) {
            return;
        }
        cJSON_AddNumberToObject(e, "code", -32601);
        cJSON_AddStringToObject(e, "message", "method not found (P2b)");
        vp_mcp_reply(id, NULL, e);
    }
}

/* --- text frame dispatch --- */

void voice_path_on_ws_text(const char *data, size_t len)
{
    if (!data || len == 0) {
        return;
    }
    size_t n = len < VP_TEXT_MAX ? len : VP_TEXT_MAX;
    char *msg = malloc(n + 1);
    if (!msg) {
        return;
    }
    memcpy(msg, data, n);
    msg[n] = 0;
    cJSON *root = cJSON_Parse(msg);
    free(msg);
    if (!root) {
        ESP_LOGW(TAG, "rx text: non-JSON (%u bytes)", (unsigned)len);
        return;
    }
    cJSON *jt = cJSON_GetObjectItemCaseSensitive(root, "type");
    const char *type = cJSON_IsString(jt) ? jt->valuestring : NULL;
    if (!type) {
        /* Bare JSON-RPC (no "type"): treat like mcp when it has method. */
        if (vp_mcp_field(root, "method")) {
            vp_on_mcp(root);
        } else {
            ESP_LOGW(TAG, "rx text: no type/method (ignored)");
        }
        cJSON_Delete(root);
        return;
    }

    if (strcmp(type, "hello") == 0) {
        cJSON *sid = cJSON_GetObjectItemCaseSensitive(root, "session_id");
        if (cJSON_IsString(sid)) {
            s_session = true;
            s_listening = false; /* uplink task sends listen start */
            ESP_LOGI(TAG, "session live (voice path armed)");
        }
        cJSON *ap = cJSON_GetObjectItemCaseSensitive(root, "audio_params");
        if (cJSON_IsObject(ap)) {
            char *echo = cJSON_PrintUnformatted(ap);
            ESP_LOGI(TAG, "audio_params: %s", echo ? echo : "?");
            free(echo);
        }
    } else if (strcmp(type, "tts") == 0) {
        cJSON *tx = cJSON_GetObjectItemCaseSensitive(root, "text");
        const char *text = cJSON_IsString(tx) ? tx->valuestring : NULL;
        cJSON *st = cJSON_GetObjectItemCaseSensitive(root, "state");
        const char *state = cJSON_IsString(st) ? st->valuestring : "?";
        s_last_tts_tick = xTaskGetTickCount();
        if (text && msg_filter_is_tool_json(text)) {
            /* E2 rule: tool-call JSON must NEVER reach the speaker. */
            ESP_LOGW(TAG, "tts DROP tool-JSON (state=%s): %.120s", state, text);
        } else if (text && vp_tts_is_server_error(text)) {
            /* Bridge error notice (e.g. Chinese "tool missing"): the intent
             * was already acted on via stt marker; mute its audio too. */
            ESP_LOGW(TAG, "tts DROP server-error (state=%s): %.120s", state, text);
            s_mute_until = xTaskGetTickCount() + pdMS_TO_TICKS(4000);
            display_set_speaking(false);
            s_speaking = false;
        } else {
            ESP_LOGI(TAG, "tts [%s]: %.120s", state, text ? text : "(no text)");
            if (strcmp(state, "start") == 0) {
                /* Keep DANG NGHI: the server sends tts start before the audio
                 * is ready. Speaking starts on the first sentence/audio. */
                ESP_LOGI(TAG, "tts start (still thinking)");
            } else if (strcmp(state, "sentence_start") == 0) {
                display_set_thinking(false);
                if (!s_speaking) {
                    display_set_speaking(true);
                    s_speaking = true;
                }
            } else if (strcmp(state, "stop") == 0) {
                s_reply_done = true;
                s_close_at = xTaskGetTickCount() + pdMS_TO_TICKS(1500);
                display_set_thinking(false);
                display_set_speaking(false);
                if (s_speaking && s_tx_gated > 0) {
                    ESP_LOGI(TAG, "tts stop: resumed uplink (%lu gated while speaking)",
                             (unsigned long)s_tx_gated);
                }
                s_speaking = false;
            }
            cJSON *em = cJSON_GetObjectItemCaseSensitive(root, "emotion");
            if (cJSON_IsString(em)) {
                display_set_emotion(em->valuestring);
            }
        }
    } else if (strcmp(type, "stt") == 0) {
        cJSON *tx = cJSON_GetObjectItemCaseSensitive(root, "text");
        const char *stt = (cJSON_IsString(tx) && tx->valuestring) ? tx->valuestring : "(no text)";
        ESP_LOGI(TAG, "stt: %.120s", stt);
        /* Bridge device-intent markers (e.g. "% self_head_nod") arrive here;
         * the server may ALSO send tools/call later — the 3 s servo debounce
         * collapses any double-delivery into one motion (SPEC §5.5#21/22). */
        if (strstr(stt, "self_head_nod") != NULL) {
            ESP_LOGI(TAG, "intent marker -> nod once");
            servo_nod_once();
        } else if (strstr(stt, "self_head_shake") != NULL) {
            ESP_LOGI(TAG, "intent marker -> shake once");
            servo_shake_once();
        }
    } else if (strcmp(type, "llm") == 0) {
        cJSON *em = cJSON_GetObjectItemCaseSensitive(root, "emotion");
        const char *emo = (cJSON_IsString(em) && em->valuestring) ? em->valuestring : "(none)";
        ESP_LOGI(TAG, "llm emotion=%s", emo);
        if (cJSON_IsString(em)) {
            display_set_emotion(em->valuestring);
        }
    } else if (strcmp(type, "mcp") == 0) {
        vp_on_mcp(root);
    } else if (strcmp(type, "system") == 0) {
        cJSON *cmd = cJSON_GetObjectItemCaseSensitive(root, "command");
        if (!cJSON_IsString(cmd)) {
            cmd = cJSON_GetObjectItemCaseSensitive(root, "state");
        }
        const char *cs = cJSON_IsString(cmd) ? cmd->valuestring : "?";
        /* P2b NEVER reboots on server request; P3 owns power policy. */
        ESP_LOGW(TAG, "system (%s): logged only, NOT rebooting (P3 owns)", cs);
    } else {
        ESP_LOGI(TAG, "rx type=%s (ignored)", type);
    }
    cJSON_Delete(root);
}

/* --- binary frame: straight to the speaker queue --- */

void voice_path_on_ws_binary(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return;
    }
    /* NOTE: no barge-in yet — loud mic input while the speaker plays does
     * not abort. Mic VAD gates the window; server VAD owns segmentation. */
    s_last_tts_tick = xTaskGetTickCount();
    if (!s_speaking) {
        display_set_thinking(false);
        display_set_speaking(true);
        s_speaking = true;
    }
    s_rx_frames++;
    if (len < s_rx_len_min) {
        s_rx_len_min = len;
    }
    if (len > s_rx_len_max) {
        s_rx_len_max = len;
    }
    s_rx_len_sum += (uint32_t)len;
    if (s_rx_frames % 200 == 0) {
        ESP_LOGI(TAG, "rx opus len: min=%u max=%u avg=%lu (n=%lu)",
                 (unsigned)s_rx_len_min, (unsigned)s_rx_len_max,
                 (unsigned long)(s_rx_len_sum / 200), (unsigned long)s_rx_frames);
        s_rx_len_min = 9999;
        s_rx_len_max = 0;
        s_rx_len_sum = 0;
    }
    if (s_mute_until != 0
        && (int32_t)(xTaskGetTickCount() - s_mute_until) < 0) {
        s_rx_muted++; /* server-error utterance: silenced by text filter */
        return;
    }
    if (audio_out_push_opus(data, len) != 0) {
        s_rx_drop++; /* queue full: drop-on-full, never block */
    }
    if (s_rx_frames == 1 || s_rx_frames % 500 == 0) {
        ESP_LOGI(TAG, "rx opus: frames=%lu dropped=%lu",
                 (unsigned long)s_rx_frames, (unsigned long)s_rx_drop);
    }
}
