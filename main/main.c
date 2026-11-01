/* P1 sketch: boot -> NVS -> WiFi -> OTA POST -> WS hello -> heartbeat.
 * SPEC FIRMWARE-REWRITE-SPEC.md §2/§3. No display/servo/audio yet (P2/P3).
 *
 * Power discipline (SPEC §2.3, Oracle binding, enforced from day one):
 *  - Volume default <= 50/100 (VOLUME_DEFAULT). No audio path yet; this
 *    constant is the single source of truth when P2 adds the speaker.
 *  - WiFi TX power <= 10 dBm: sdkconfig CONFIG_ESP_PHY_MAX_TX_POWER=40
 *    AND runtime esp_wifi_set_max_tx_power(40) after WiFi start (belt and
 *    suspenders: shared 3V3 rail with speaker+servo browns out otherwise).
 *
 * Stability contract (SPEC §5, enforced here):
 *  - No infinite waits, no abort-on-error on any runtime path.
 *  - WiFi reconnect: retry forever, supervisor ticks every 30 s (test #7).
 *  - OTA POST: retry forever. NVS missing/corrupt -> portal AP, never panic.
 *
 * Lock contract (SPEC §5.4#17, E3 lesson — binding for P2/P3):
 *  - The display task (P3) shares NO mutex with audio/state/servo tasks.
 *    A wedged I2C frame must never wedge the state machine (old-fw
 *    DisplayLockGuard held the lock 30 s across a flush -> full wedge).
 *  - Every runtime mutex uses try-lock with timeout <= 2 s, and unlocks
 *    ONLY if the lock was actually acquired (unlock-on-failure crashes).
 *  - Display violations of this contract yield to P0: the frame is skipped
 *    (lv_disp_flush_ready + dirty-repaint <= 3 s), never the audio path.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_http_client.h"
#include "esp_websocket_client.h"
#include "esp_http_server.h"
#include "cJSON.h"

#include "msg_filter.h"
#include "audio_in.h"
#include "audio_out.h"
#include "servo.h"
#include "buttons.h"
#include "voice_path.h"
#include "watchdog.h"
/* Display lane (D1) owns display.h at reconcile; guarded so this file builds
 * before it lands (watchdog's display check soft-disables until then). */
#if __has_include("display.h")
#include "display.h"
#endif

static const char *TAG = "p1";

/* ---- Config ---- */
#define WIFI_NVS_NS      "wifi"
#define NVS_KEY_SSID     "ssid"
#define NVS_KEY_PASS     "password"
#define NVS_KEY_OTA_URL  "ota_url"
#define NVS_KEY_CLIENT_ID "client_id" /* WS Client-Id UUID, generated once */

#define OTA_URL_FALLBACK "http://192.168.1.106:8000/xiaozhi/ota/"
#define WS_URL_FALLBACK  "ws://192.168.1.106:8000/xiaozhi/v1/"

/* SPEC §2.3: default volume <= 50. Single source of truth for P2 audio. */
#define VOLUME_DEFAULT   50
/* SPEC §2.3: 10 dBm in 0.25 dBm units. Enforced via sdkconfig + runtime. */
#define WIFI_MAX_TX_POWER_QDBM 40

/* SPEC §3.2 exact hello. Do not alter field names/shapes. */
#define HELLO_JSON \
    "{\"type\":\"hello\",\"version\":1,\"features\":{\"mcp\":true}," \
    "\"transport\":\"websocket\"," \
    "\"audio_params\":{\"format\":\"opus\",\"sample_rate\":16000," \
    "\"channels\":1,\"frame_duration\":60}}"

#define CONNECTED_BIT BIT0
static EventGroupHandle_t s_wifi_events;
static esp_websocket_client_handle_t s_ws;
static char s_ws_url[256];
static char s_ota_token[128]; /* Bearer token from OTA response, empty if none */
static bool s_got_session_id;

/* ---------- small helpers ---------- */

static void urldecode(char *dst, size_t dstsz, const char *src)
{
    size_t di = 0;
    for (size_t i = 0; src[i] && di + 1 < dstsz; i++) {
        if (src[i] == '+') {
            dst[di++] = ' ';
        } else if (src[i] == '%' && isxdigit((int)src[i + 1]) && isxdigit((int)src[i + 2])) {
            char h[3] = { src[i + 1], src[i + 2], 0 };
            dst[di++] = (char)strtol(h, NULL, 16);
            i += 2;
        } else {
            dst[di++] = src[i];
        }
    }
    dst[di] = 0;
}

/* Returns heap string (caller frees) or NULL on missing/corrupt/empty. Never panics. */
static char *nvs_read_str(const char *ns, const char *key)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ns, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open %s: %s", ns, esp_err_to_name(err));
        return NULL;
    }
    size_t len = 0;
    err = nvs_get_str(h, key, NULL, &len);
    if (err != ESP_OK || len == 0 || len > 512) {
        ESP_LOGW(TAG, "nvs %s/%s missing or bad (err=%s len=%u)",
                 ns, key, esp_err_to_name(err), (unsigned)len);
        nvs_close(h);
        return NULL;
    }
    char *buf = malloc(len);
    if (!buf) {
        nvs_close(h);
        return NULL;
    }
    err = nvs_get_str(h, key, buf, &len);
    nvs_close(h);
    if (err != ESP_OK || buf[0] == '\0') {
        free(buf);
        return NULL;
    }
    return buf;
}

static bool nvs_write_str(const char *ns, const char *key, const char *val)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open rw: %s", esp_err_to_name(err));
        return false;
    }
    err = nvs_set_str(h, key, val);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs write %s/%s: %s", ns, key, esp_err_to_name(err));
        return false;
    }
    return true;
}

/* Client-Id UUID (E1): persisted once in NVS wifi/client_id, else generated.
 * Best-effort persist; never panics. out always NUL-terminated. */
static void client_id_ensure(char out[64])
{
    out[0] = 0;
    nvs_handle_t h;
    if (nvs_open(WIFI_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        size_t len = 64;
        if (nvs_get_str(h, NVS_KEY_CLIENT_ID, out, &len) == ESP_OK && out[0]) {
            nvs_close(h);
            return;
        }
        nvs_close(h);
    }
    uint32_t r0 = esp_random(), r1 = esp_random();
    uint32_t r2 = esp_random(), r3 = esp_random();
    snprintf(out, 64, "%08lx-%04lx-%04lx-%04lx-%08lx%04lx",
             (unsigned long)r0,
             (unsigned long)((r1 >> 16) & 0xffff), (unsigned long)(r1 & 0xffff),
             (unsigned long)((r2 >> 16) & 0xffff),
             (unsigned long)r3, (unsigned long)(r2 & 0xffff));
    if (!nvs_write_str(WIFI_NVS_NS, NVS_KEY_CLIENT_ID, out)) {
        ESP_LOGW(TAG, "client_id generated but not persisted (continuing)");
    }
}

/* B3 SNTP: real epoch before timezone apply. NON-BLOCKING: boot continues
 * immediately (voice online in ~10 s); sync completes in background. TZ is
 * set separately (SPEC §3.1 offset +7 fallback, or OTA-provided). */
static void sntp_start_background(void)
{
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    if (esp_netif_sntp_init(&cfg) != ESP_OK) {
        ESP_LOGW(TAG, "sntp init failed, clock stays epoch (night-sleep waits)");
        return;
    }
    ESP_LOGI(TAG, "sntp started in background (pool.ntp.org)");
}

/* Local +7 fallback (SPEC §3.1: server timezone_offset = 7). POSIX sign is
 * inverted: UTC-7 == +07:00 east. */
static void tz_apply_fallback(void)
{
    setenv("TZ", "UTC-7", 1);
    tzset();
}

/* ---------- portal AP stub ---------- */
/* Missing/corrupt NVS -> serve AP "xiaozhi-setup", accept POST of
 * ssid/password/ota_url, store to NVS, reboot. Blocks until saved. */

static const char *PORTAL_FORM =
    "<html><body><h1>xiaozhi setup</h1>"
    "<form method='POST' action='/save'>"
    "SSID:<input name='ssid'><br>"
    "Password:<input name='password' type='password'><br>"
    "OTA URL:<input name='ota_url' size='60'><br>"
    "<input type='submit' value='Save & reboot'></form></body></html>";

static esp_err_t portal_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, PORTAL_FORM, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* Parse one urlencoded field into out (always NUL-terminated). */
static void form_field(const char *body, const char *name, char *out, size_t outsz)
{
    out[0] = 0;
    size_t nl = strlen(name);
    const char *p = body;
    while ((p = strstr(p, name)) != NULL) {
        if ((p == body || p[-1] == '&') && p[nl] == '=') {
            p += nl + 1;
            const char *end = strchr(p, '&');
            size_t rawlen = end ? (size_t)(end - p) : strlen(p);
            if (rawlen > 1023) {
                rawlen = 1023;
            }
            char raw[1024];
            memcpy(raw, p, rawlen);
            raw[rawlen] = 0;
            urldecode(out, outsz, raw);
            return;
        }
        p += nl;
    }
}

static esp_err_t portal_save_handler(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > 2048) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad length");
        return ESP_FAIL;
    }
    char *body = malloc((size_t)req->content_len + 1);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_FAIL;
    }
    int got = httpd_req_recv(req, body, (size_t)req->content_len);
    if (got <= 0) {
        free(body);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recv failed");
        return ESP_FAIL;
    }
    body[got] = 0;

    char ssid[64], pass[128], ota[256];
    form_field(body, "ssid", ssid, sizeof(ssid));
    form_field(body, "password", pass, sizeof(pass));
    form_field(body, "ota_url", ota, sizeof(ota));
    free(body);

    if (!ssid[0] || !ota[0]) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "ssid and ota_url required");
        return ESP_FAIL;
    }
    bool ok = nvs_write_str(WIFI_NVS_NS, NVS_KEY_SSID, ssid)
        && nvs_write_str(WIFI_NVS_NS, NVS_KEY_PASS, pass)
        && nvs_write_str(WIFI_NVS_NS, NVS_KEY_OTA_URL, ota);
    if (!ok) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "nvs store failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "portal: saved ssid=%s ota_url=%s, rebooting", ssid, ota);
    httpd_resp_sendstr(req, "saved, rebooting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK; /* unreachable */
}

static void portal_run_blocking(void)
{
    ESP_LOGW(TAG, "NVS wifi/ota_url missing or corrupt -> portal AP stub");
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "portal wifi_init failed, retrying in 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
        return;
    }
    wifi_config_t ap = { 0 };
    strcpy((char *)ap.ap.ssid, "xiaozhi-setup");
    ap.ap.ssid_len = strlen("xiaozhi-setup");
    ap.ap.channel = 6;
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_OPEN;
    if (esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK
        || esp_wifi_set_config(WIFI_IF_AP, &ap) != ESP_OK
        || esp_wifi_start() != ESP_OK) {
        ESP_LOGE(TAG, "portal AP start failed, rebooting in 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
        return;
    }
    httpd_handle_t srv = NULL;
    httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
    if (httpd_start(&srv, &hc) == ESP_OK) {
        httpd_uri_t gu = { .uri = "/", .method = HTTP_GET, .handler = portal_get_handler };
        httpd_uri_t pu = { .uri = "/save", .method = HTTP_POST, .handler = portal_save_handler };
        httpd_register_uri_handler(srv, &gu);
        httpd_register_uri_handler(srv, &pu);
    }
    ESP_LOGI(TAG, "portal up: join AP xiaozhi-setup, open http://192.168.4.1/");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000)); /* wait for POST -> reboot */
    }
}

/* ---------- WiFi STA with auto-reconnect (30 s retry forever) ---------- */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_events, CONNECTED_BIT);
        ESP_LOGW(TAG, "wifi disconnected, reconnecting...");
        esp_err_t err = esp_wifi_connect();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "reconnect now failed (%s); supervisor retries", esp_err_to_name(err));
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_events, CONNECTED_BIT);
        ESP_LOGI(TAG, "wifi got IP");
    }
}

/* Supervisor: if link is down, attempt connect; ticks every 30 s forever. */
static void wifi_supervisor_task(void *arg)
{
    (void)arg;
    while (1) {
        if (!(xEventGroupGetBits(s_wifi_events) & CONNECTED_BIT)) {
            ESP_LOGI(TAG, "supervisor: link down, esp_wifi_connect()");
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "supervisor connect: %s", esp_err_to_name(err));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

static void wifi_sta_start(const char *ssid, const char *pass)
{
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "wifi_init failed, rebooting in 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
        return;
    }
    if (esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL) != ESP_OK
        || esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "event register failed, rebooting in 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
        return;
    }
    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid, ssid, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, pass ? pass : "", sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK
        || esp_wifi_set_config(WIFI_IF_STA, &wc) != ESP_OK
        || esp_wifi_start() != ESP_OK) {
        ESP_LOGE(TAG, "wifi start failed, rebooting in 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
        return;
    }
    /* SPEC §2.3: cap TX power at runtime too (sdkconfig does it at PHY init). */
    esp_err_t terr = esp_wifi_set_max_tx_power(WIFI_MAX_TX_POWER_QDBM);
    ESP_LOGI(TAG, "TX power cap 10dBm (40): %s", esp_err_to_name(terr));
    if (xTaskCreate(wifi_supervisor_task, "wifi_sup", 3072, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "supervisor task create failed");
    }
    ESP_LOGI(TAG, "wifi STA connecting to %s (retry forever, 30s supervisor)", ssid);
    /* Wait for first connection, forever (no timeout panic). Test #7. */
    while (!(xEventGroupWaitBits(s_wifi_events, CONNECTED_BIT, pdFALSE, pdTRUE,
                                 pdMS_TO_TICKS(10000)) & CONNECTED_BIT)) {
        ESP_LOGI(TAG, "still waiting for wifi...");
    }
}

/* ---------- OTA POST with retry; timezone apply; WS URL select ---------- */

static void mac_str(char out[18])
{
    uint8_t mac[6] = { 0 };
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* Try nested shapes servers return; fall back to WS_URL_FALLBACK. */
static void pick_ws_url(cJSON *root, char out[256])
{
    const char *found = NULL;
    static const char *paths[][3] = {
        { "websocket", "url", NULL },
        { "server", "url", NULL },
        { "websocket", "server", NULL },
        { "ws_url", NULL, NULL },
        { "websocket_url", NULL, NULL },
    };
    for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); i++) {
        cJSON *node = root;
        for (int k = 0; paths[i][k] && node; k++) {
            node = cJSON_GetObjectItemCaseSensitive(node, paths[i][k]);
        }
        if (cJSON_IsString(node) && node->valuestring[0]) {
            found = node->valuestring;
            break;
        }
    }
    if (found) {
        strncpy(out, found, 255);
    } else {
        strncpy(out, WS_URL_FALLBACK, 255);
    }
    out[255] = 0;
}

static void apply_timezone(cJSON *root)
{
    cJSON *tz = cJSON_GetObjectItemCaseSensitive(root, "timezone_offset");
    if (!cJSON_IsNumber(tz)) {
        cJSON *tzw = cJSON_GetObjectItemCaseSensitive(root, "timezone");
        if (cJSON_IsObject(tzw)) {
            tz = cJSON_GetObjectItemCaseSensitive(tzw, "offset");
        }
    }
    if (!cJSON_IsNumber(tz)) {
        ESP_LOGI(TAG, "no timezone_offset in OTA response, TZ unchanged");
        return;
    }
    /* TZ env (POSIX inverted sign), NOT a clock shift: SNTP owns UTC. */
    int off_h = (int)tz->valuedouble;
    char buf[16];
    snprintf(buf, sizeof(buf), "UTC%+d", -off_h);
    setenv("TZ", buf, 1);
    tzset();
    ESP_LOGI(TAG, "TZ set to %s (offset %+d)", buf, off_h);
}

/* Bearer token shapes seen on xiaozhi servers; empty if absent. */
static void pick_token(cJSON *root)
{
    cJSON *t = cJSON_GetObjectItemCaseSensitive(root, "token");
    if (!cJSON_IsString(t)) {
        t = cJSON_GetObjectItemCaseSensitive(root, "access_token");
    }
    if (!cJSON_IsString(t)) {
        cJSON *srv = cJSON_GetObjectItemCaseSensitive(root, "server");
        if (cJSON_IsObject(srv)) {
            t = cJSON_GetObjectItemCaseSensitive(srv, "token");
        }
    }
    if (cJSON_IsString(t) && t->valuestring[0]) {
        strncpy(s_ota_token, t->valuestring, sizeof(s_ota_token) - 1);
        s_ota_token[sizeof(s_ota_token) - 1] = 0;
    } else {
        s_ota_token[0] = 0;
    }
}

/* POST device info, parse WS url + token + timezone.
 * Bounded retries then SPEC §3.1 fallback: the OTA handler may be down
 * while WS is alive. After OTA_MAX_ATTEMPTS failures (~40 s) continue
 * with WS_URL_FALLBACK instead of blocking boot forever. Never panics. */
#define OTA_MAX_ATTEMPTS 8
static void ota_fetch_blocking(const char *ota_url)
{
    char macs[18];
    mac_str(macs);
    ESP_LOGI(TAG, "device MAC %s", macs);
    char client_id[64];
    client_id_ensure(client_id);

    /* B2: E1 system-info body. flash_size in bytes (16 MB N16R8). */
    char body[512];
    snprintf(body, sizeof(body),
             "{\"chip\":\"esp32s3\",\"chip_model_name\":\"esp32s3\","
             "\"mac\":\"%s\",\"uuid\":\"%s\",\"version\":\"p1-sketch\","
             "\"application\":{\"name\":\"xiaozhi-p1\",\"version\":\"0.0.1\"},"
             "\"flash_mb\":16,\"flash_size\":16777216,\"volume\":%d}",
             macs, client_id, VOLUME_DEFAULT);

    int attempts = 0;
    while (1) {
        esp_http_client_config_t cc = {
            .url = ota_url,
            .method = HTTP_METHOD_POST,
            .timeout_ms = 8000,
        };
        esp_http_client_handle_t cli = esp_http_client_init(&cc);
        if (!cli) {
            ESP_LOGW(TAG, "ota: client init OOM, retry in 5s");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        esp_http_client_set_header(cli, "Content-Type", "application/json");
        /* 78/xiaozhi-esp32 style identity headers; some servers require them. */
        esp_http_client_set_header(cli, "Device-Id", macs);
        esp_http_client_set_header(cli, "Client-Id", client_id);
        esp_http_client_set_header(cli, "Activation-Version", "1");
        esp_http_client_set_header(cli, "User-Agent", "xiaozhi-p1/0.0.1");
        esp_http_client_set_post_field(cli, body, strlen(body));
        esp_err_t err = esp_http_client_perform(cli);
        int status = esp_http_client_get_status_code(cli);
        if (err != ESP_OK || status < 200 || status >= 300) {
            ESP_LOGW(TAG, "ota attempt %d/%d: POST %s (status=%d), retry in 5s",
                     attempts + 1, OTA_MAX_ATTEMPTS,
                     esp_err_to_name(err), status);
            esp_http_client_cleanup(cli);
            vTaskDelay(pdMS_TO_TICKS(5000));
            if (++attempts >= OTA_MAX_ATTEMPTS) {
                break;
            }
            continue;
        }
        int len = esp_http_client_get_content_length(cli);
        if (len <= 0 || len > 8192) {
            len = 4096;
        }
        char *resp = malloc((size_t)len + 1);
        if (!resp) {
            esp_http_client_cleanup(cli);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        int rlen = esp_http_client_read_response(cli, resp, len);
        esp_http_client_cleanup(cli);
        if (rlen <= 0) {
            /* HTTP 200 + empty body = server alive, no OTA info: retrying
             * won't help. Fail fast (2 tries) instead of burning 40 s. */
            ESP_LOGW(TAG, "ota attempt %d: empty body (status=%d), %s",
                     attempts + 1, status,
                     attempts + 1 >= 2 ? "falling back" : "retry in 5s");
            free(resp);
            if (++attempts >= 2) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        resp[rlen] = 0;
        ESP_LOGI(TAG, "ota response: %.300s", resp);
        cJSON *root = cJSON_Parse(resp);
        free(resp);
        if (!root) {
            ESP_LOGW(TAG, "ota attempt %d/%d: bad JSON, retry in 5s",
                     attempts + 1, OTA_MAX_ATTEMPTS);
            vTaskDelay(pdMS_TO_TICKS(5000));
            if (++attempts >= OTA_MAX_ATTEMPTS) {
                break;
            }
            continue;
        }
        pick_ws_url(root, s_ws_url);
        pick_token(root);
        apply_timezone(root);
        cJSON_Delete(root);
        ESP_LOGI(TAG, "ota OK, ws=%s", s_ws_url);
        return;
    }
    strncpy(s_ws_url, WS_URL_FALLBACK, sizeof(s_ws_url) - 1);
    ESP_LOGW(TAG, "ota failed %d attempts, SPEC fallback ws=%s", attempts, s_ws_url);
}

/* ---------- WebSocket: hello -> session_id + heartbeat ---------- */

/* Old-firmware architecture (verified in hyvuonggia/xiaozhi-deskemoji-taibai:
 * OpenAudioChannel on conversation start, CloseAudioChannel on leave): the
 * audio channel exists ONLY during a conversation. Keeping it open while idle
 * makes the server run its 10 s no-voice timeout and speak the end_prompt
 * goodbye ("tạm biệt sếp") for no reason. So: open on wake, close when the
 * conversation window closes. */
static volatile bool s_ws_intentional_close;

/* Audio-channel lifecycle used by the voice path (wake opens, idle closes).
 * The voice path only flips a request flag: creating/destroying the client
 * from the audio task deadlocked the device (live-tested 2026-10-31). A small
 * control task owns the real lifecycle. */
void ws_open_for_wake(void);
void ws_close_idle(void);
static void ws_client_create(void);
static volatile bool s_ws_want_open;
static bool s_ws_ctl_on;

static void ws_control_task(void *arg)
{
    (void)arg;
    while (1) {
        if (s_ws_want_open && !s_ws) {
            s_ws_intentional_close = false;
            s_got_session_id = false;
            ws_client_create(); /* lazy: no idle client eating internal RAM */
            if (s_ws) {
                esp_err_t err = esp_websocket_client_start(s_ws);
                ESP_LOGI(TAG, "ws open (wake): %s", esp_err_to_name(err));
            }
        } else if (!s_ws_want_open && s_ws) {
            esp_websocket_client_handle_t h = s_ws;
            s_ws = NULL;
            s_ws_intentional_close = true;
            s_got_session_id = false;
            voice_path_set_ws(NULL);
            (void)esp_websocket_client_stop(h);
            (void)esp_websocket_client_destroy(h);
            ESP_LOGI(TAG, "ws close (idle, client destroyed)");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void send_hello(void)
{
    if (!s_ws) {
        return;
    }
    int rc = esp_websocket_client_send_text(s_ws, HELLO_JSON, strlen(HELLO_JSON),
                                            pdMS_TO_TICKS(5000));
    if (rc < 0) {
        ESP_LOGW(TAG, "hello send failed");
    } else {
        ESP_LOGI(TAG, "hello sent (v1 opus 16k/60ms)");
    }
}

static void ws_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)base;
    /* arg is the client handle this handler was registered for. A client
     * destroyed at the end of a conversation can still emit a late
     * DISCONNECTED/ERROR; ignoring stale handles stops that ghost event from
     * tearing down the NEW session (it did: voice went silent ~1 s in). */
    if (arg != (void *)s_ws) {
        return;
    }
    esp_websocket_event_data_t *ev = data;
    if (id == WEBSOCKET_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "ws connected");
        send_hello();
        voice_path_on_ws_connected();
    } else if (id == WEBSOCKET_EVENT_DISCONNECTED || id == WEBSOCKET_EVENT_CLOSED) {
        voice_path_on_ws_disconnected(); /* stop streaming; fresh WAKE next time */
        if (s_ws_intentional_close) {
            ESP_LOGI(TAG, "ws closed (idle, no reconnect until next wake)");
            return;
        }
        /* Driver auto-reconnect (reconnect_timeout_ms=10 s) handles this;
         * a second hand-rolled task used to OOM on internal RAM and shadow
         * the driver's own retry. */
        ESP_LOGW(TAG, "ws disconnected (driver will retry)");
    } else if (id == WEBSOCKET_EVENT_DATA && ev && ev->data_ptr && ev->data_len > 0) {
        /* P2b voice path: binary opus -> speaker, text control -> dispatch.
         * F7 note (left as-is, low-risk deferral): the text frame is parsed
         * twice — once inside voice_path_on_ws_text() and once below for
         * session_id/audio_params logging. Unifying needs a new parsed-cJSON
         * voice-path entry point; deferred to keep this hunk narrow. */
        if (ev->op_code == WS_TRANSPORT_OPCODES_BINARY) {
            voice_path_on_ws_binary((const uint8_t *)ev->data_ptr, (size_t)ev->data_len);
            return; /* never parse/log opus as text (serial flood + CPU) */
        }
        voice_path_on_ws_text((const char *)ev->data_ptr, (size_t)ev->data_len);
        char *msg = malloc((size_t)ev->data_len + 1);
        if (!msg) {
            return;
        }
        memcpy(msg, ev->data_ptr, (size_t)ev->data_len);
        msg[ev->data_len] = 0;
        ESP_LOGI(TAG, "ws rx: %.300s", msg);
        cJSON *root = cJSON_Parse(msg);
        free(msg);
        if (!root) {
            return;
        }
        cJSON *sid = cJSON_GetObjectItemCaseSensitive(root, "session_id");
        if (cJSON_IsString(sid)) {
            s_got_session_id = true;
            ESP_LOGI(TAG, "session_id=%s", sid->valuestring);
        }
        cJSON *ap = cJSON_GetObjectItemCaseSensitive(root, "audio_params");
        if (cJSON_IsObject(ap)) {
            char *echo = cJSON_PrintUnformatted(ap);
            ESP_LOGI(TAG, "audio_params echo: %s", echo ? echo : "?");
            free(echo);
        }
        cJSON_Delete(root);
    } else if (id == WEBSOCKET_EVENT_ERROR) {
        ESP_LOGW(TAG, "ws error");
    }
}

static void ws_client_create(void)
{
    esp_websocket_client_config_t cc = {
        .uri = s_ws_url[0] ? s_ws_url : WS_URL_FALLBACK,
        .reconnect_timeout_ms = 10000,
        .network_timeout_ms = 10000,
    };
    s_ws = esp_websocket_client_init(&cc);
    if (!s_ws) {
        ESP_LOGE(TAG, "ws init OOM, rebooting in 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
        return;
    }
    /* B1 (E1 confirmed): handshake headers. No header -> no handshake.
     * Each append is soft-fail (logged, connection still attempted). */
    char macs[18];
    mac_str(macs);
    char client_id[64];
    client_id_ensure(client_id);
    char bearer[160];
    snprintf(bearer, sizeof(bearer), "Bearer %s", s_ota_token);
    if (esp_websocket_client_append_header(s_ws, "Authorization", bearer) != ESP_OK) {
        ESP_LOGW(TAG, "ws header Authorization not set");
    }
    if (esp_websocket_client_append_header(s_ws, "Protocol-Version", "1") != ESP_OK) {
        ESP_LOGW(TAG, "ws header Protocol-Version not set");
    }
    if (esp_websocket_client_append_header(s_ws, "Device-Id", macs) != ESP_OK) {
        ESP_LOGW(TAG, "ws header Device-Id not set");
    }
    if (esp_websocket_client_append_header(s_ws, "Client-Id", client_id) != ESP_OK) {
        ESP_LOGW(TAG, "ws header Client-Id not set");
    }
    if (esp_websocket_register_events(s_ws, WEBSOCKET_EVENT_ANY, ws_event_handler, s_ws) != ESP_OK) {
        ESP_LOGE(TAG, "ws event register failed");
    }
    /* Do NOT start here: the channel opens on the first wake word (see
     * ws_open_for_wake) so an idle robot never triggers the server goodbye. */
    voice_path_set_ws(s_ws); /* handle changes per conversation */
}

/* Called by the voice path on a wake word: open the audio channel. */
void ws_open_for_wake(void)
{
    s_ws_want_open = true; /* the control task does the real work */
}

/* Called when the conversation window closes: drop the audio channel so the
 * server has no idle session to farewell. */
void ws_close_idle(void)
{
    s_ws_want_open = false; /* the control task does the real work */
}

/* ---------- app_main ---------- */

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        esp_err_t e2 = nvs_flash_erase();
        if (e2 == ESP_OK) {
            err = nvs_flash_init();
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init: %s (continuing, portal may reset)", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "P1 sketch boot, vol_default=%d, tx_cap_qdbm=%d",
             VOLUME_DEFAULT, WIFI_MAX_TX_POWER_QDBM);

    char *ssid = nvs_read_str(WIFI_NVS_NS, NVS_KEY_SSID);
    char *pass = nvs_read_str(WIFI_NVS_NS, NVS_KEY_PASS);
    char *ota_url = nvs_read_str(WIFI_NVS_NS, NVS_KEY_OTA_URL);
    if (!ssid || !ota_url) {
        free(ssid);
        free(pass);
        free(ota_url);
        if (esp_netif_init() != ESP_OK) {
            ESP_LOGE(TAG, "netif init failed before portal");
        }
        if (esp_event_loop_create_default() != ESP_OK) {
            ESP_LOGW(TAG, "event loop exists, continuing");
        }
        portal_run_blocking(); /* never returns */
        return;
    }

    s_wifi_events = xEventGroupCreate();
    if (!s_wifi_events) {
        ESP_LOGE(TAG, "event group OOM, rebooting in 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
        return;
    }
    if (esp_netif_init() != ESP_OK) {
        ESP_LOGE(TAG, "netif init failed");
    }
    if (esp_event_loop_create_default() != ESP_OK) {
        ESP_LOGW(TAG, "event loop exists, continuing");
    }

    wifi_sta_start(ssid, pass ? pass : "");
    /* Modem sleep stalls WS writes (seen as transport_poll_write(0) right
     * after streaming starts). Latency + stability matter more than mA here. */
    esp_err_t ps = esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_LOGI(TAG, "wifi ps none: %s", esp_err_to_name(ps));
    free(ssid);
    free(pass);

    ESP_LOGI(TAG, "msg_filter selftest fails=%d", msg_filter_selftest());

    /* B3: clock syncs in background; TZ +7 fallback now, OTA may refine. */
    tz_apply_fallback();
    sntp_start_background();

    ota_fetch_blocking(ota_url ? ota_url : OTA_URL_FALLBACK);
    free(ota_url);

    voice_path_start(NULL, ws_open_for_wake, ws_close_idle);
    if (xTaskCreate(ws_control_task, "ws_ctl", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "ws control task OOM (voice channel unavailable)");
    }

    /* P2 structure: audio + servo + buttons. Each module owns its task(s);
     * see lock-contract note at top of file. All soft-fail, never panic.
     * P3 order: buttons/servo/audio, then display, then watchdog last. */
    buttons_init();
    servo_init();      /* centers 90/90 + single-nod boot demo */
    audio_in_start();  /* mic task prio 8 */
    audio_out_start(); /* speaker task prio 5 + 120 ms silence prime */
    audio_selftest();  /* mic RMS + opus stats + tone; self-delays, log only */
#if __has_include("display.h")
    display_init();
#endif
    watchdog_init();   /* supervisor prio 3, 1 s tick; owns reboot policy */

    /* 1 Hz heartbeat: proves main loop alive; watchdog reboots if >30 s stale
     * (SPEC §5.4#16). Reset reason is logged by watchdog_init(). */
    uint32_t up = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        up++;
        watchdog_heartbeat();
        bool wifi = (xEventGroupGetBits(s_wifi_events) & CONNECTED_BIT) ? true : false;
#if __has_include("display.h")
        /* Status icons + night sleep 01:00-07:00 server-local (SPEC §5.6#23).
         * time() is real epoch after SNTP + timezone_offset (B3). */
        display_set_status(wifi, s_got_session_id, buttons_get_volume());
        display_set_offline(!wifi); /* MAT KET NOI while the link is down */
        time_t now = time(NULL);
        if (now > 1700000000) {
            struct tm tmv;
            localtime_r(&now, &tmv);
            display_set_night_sleep(tmv.tm_hour >= 1 && tmv.tm_hour < 7);
        }
#endif
        ESP_LOGI(TAG, "heartbeat up=%lus wifi=%d session=%d",
                 (unsigned long)up, wifi ? 1 : 0,
                 s_got_session_id ? 1 : 0);
        ESP_LOGI(TAG, "mem free=%lu min-ever=%lu int=%lu int-min=%lu",
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)esp_get_minimum_free_heap_size(),
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    }
}
