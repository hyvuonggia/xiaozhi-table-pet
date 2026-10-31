/* Voice path P2b: Opus uplink task + WS downlink dispatch.
 * See voice_path.h for the contract. All parses NULL-safe/cJSON,
 * soft-fail; every wait/timeout finite; no banned tokens (CI-checked).
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

/* Wake-word conversation window (WAKE lane, replaces energy auto-open).
 * No WakeNet trigger -> no uplink: room noise can never open the stream.
 * Window OPENS on wakeword_consume() ("WAKE -> window open"), stays open
 * while mic RMS > floor*3+800 (speech continuing), and CLOSES after
 * 1500 ms below that line ("window closed"). Server VAD still segments
 * inside the window. Quiet room -> IDLE screen, nothing sent.
 * TTS playing -> SPEAKING screen, uplink paused (half-duplex echo guard). */
static uint32_t s_floor;          /* adaptive noise floor (RMS units) */
static bool s_window;             /* conversation window open (wake latched) */
static TickType_t s_quiet_since;  /* last tick with RMS above keep threshold */
static TickType_t s_window_open_tick; /* wake moment (minimum window guard) */
static bool s_screen_listen;      /* last display_set_listening() value */
#define VP_KEEP_MUL 3u
#define VP_KEEP_ADD 800u
/* Inside an open window the wake word already proved the user is talking, so
 * be MORE sensitive (the laptop/TV can be quiet while the user speaks). */
#define VP_INWIN_MUL 2u
#define VP_INWIN_ADD 500u
#define VP_WINDOW_QUIET_MS 2000u /* server VAD needs the stream through the
                                    end of the sentence; 1.5 s cut it off */
#define VP_WINDOW_MIN_MS   6000u /* user needs time to start talking */
#define VP_FLOOR_SLOW_DIV  64u

/* Uplink: drain audio_in_pop_opus() -> WS BINARY while session active
 * AND listening. Never blocks the mic task (pop has a short timeout);
 * WS congestion drops the frame and counts it. */
static void vp_uplink_task(void *arg)
{
    (void)arg;
    uint8_t buf[VP_OPUS_BUF];
    ESP_LOGI(TAG, "uplink task on (prio %d)", VP_UPLINK_PRIO);
    while (1) {
        if (!s_session) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (!s_ws) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        /* Adaptive floor: drops fast to quieter rooms, rises slowly. */
        uint32_t rms = audio_in_recent_rms();
        if (s_floor == 0) {
            s_floor = rms;
        } else if (rms < s_floor) {
            s_floor = rms; /* quieter: floor drops fast */
        } else {
            s_floor += (rms - s_floor) / VP_FLOOR_SLOW_DIV; /* rises slowly */
        }
        uint32_t keep = s_floor * VP_INWIN_MUL + VP_INWIN_ADD;
        TickType_t now = xTaskGetTickCount();
        /* Window opens ONLY on a WakeNet trigger — never on energy. */
        if (wakeword_consume()) {
            if (!s_window) {
                ESP_LOGI(TAG, "WAKE -> window open (rms=%lu floor=%lu)",
                         (unsigned long)rms, (unsigned long)s_floor);
            }
            s_window = true;
            s_window_open_tick = now;
            s_quiet_since = now;
            vp_send_listen_start(); /* explicit: server starts segmenting now */
        } else if (s_window) {
            if (rms > keep) {
                s_quiet_since = now; /* speech continues: hold window */
            } else if ((now - s_window_open_tick) >= pdMS_TO_TICKS(VP_WINDOW_MIN_MS)
                       && (now - s_quiet_since) >= pdMS_TO_TICKS(VP_WINDOW_QUIET_MS)) {
                s_window = false;
                vp_send_listen_stop();
                ESP_LOGI(TAG, "window closed (rms=%lu floor=%lu quiet %ums)",
                         (unsigned long)rms, (unsigned long)s_floor,
                         (unsigned)VP_WINDOW_QUIET_MS);
            }
        }
        if (s_window != s_screen_listen) {
            s_screen_listen = s_window;
            display_set_listening(s_window);
        }
        int n = audio_in_pop_opus(buf, sizeof(buf), VP_POP_TIMEOUT_MS);
        if (n <= 0) {
            continue; /* timeout/empty/not-running: no frame, no send */
        }
        if (s_speaking) {
            /* Half-duplex: pop-and-drop so the mic queue stays fresh. */
            s_tx_gated++;
            continue;
        }
        if (!s_window) {
            /* Outside the wake window: pop-and-drop. Nothing sent. */
            s_tx_gated++;
            continue;
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

void voice_path_start(esp_websocket_client_handle_t ws)
{
    s_ws = ws;
    wakeword_init(); /* idempotent; soft-fails to "no wake" if PSRAM/model bad */
    if (s_task_on) {
        return;
    }
    if (xTaskCreate(vp_uplink_task, "vp_up", VP_UPLINK_STACK,
                    NULL, VP_UPLINK_PRIO, NULL) != pdPASS) {
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
    s_speaking = false; /* never stay half-duplex-gated across sessions */
    s_window = false; /* wake window needs a fresh WAKE next session */
    s_floor = 0; /* re-learn the room on next session */
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
                display_set_speaking(true);
                s_speaking = true;
            } else if (strcmp(state, "stop") == 0) {
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
    /* NOTE (P3): no barge-in yet — no VAD, so loud mic input while the
     * speaker plays does not abort. Mic keeps streaming; server VAD owns
     * segmentation. */
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
