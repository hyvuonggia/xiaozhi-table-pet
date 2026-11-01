/* Hardened SH1106 OLED driver. See display.h for the contract.
 * Raw i2c_master with finite timeouts only; bus-high check before every
 * frame; own private mutex; render-on-change <=5 fps; counters + rescue.
 */

#include "display.h"

#include <string.h>
#include <strings.h>

#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static const char *TAG = "display";

/* ---- pins / bus (SPEC section 2.2 exact, section 2.4 measured best) ---- */
#define DISP_SDA_GPIO    GPIO_NUM_41
#define DISP_SCL_GPIO    GPIO_NUM_42
#define DISP_I2C_PORT    I2C_NUM_0
#define DISP_I2C_ADDR    0x3C
#define DISP_I2C_HZ      50000
#define DISP_XFER_MS     100          /* finite timeout, every transaction */
#define DISP_W           128
#define DISP_H           64
#define DISP_PAGES       8
#define DISP_LOCK_MS     2000         /* SPEC section 5.4 #17: <= 2 s */
#define DISP_MIN_GAP_MS  200          /* <= 5 fps */
#define DISP_TASK_PRIO   2
#define DISP_TASK_STACK  4096         /* >= 3072; fb ops + 129 B tx buf */
#define DISP_UNHEALTHY_MS 90000       /* stuck wanted-frame -> unhealthy */
#define DISP_RETRY_INIT_LOOPS 100     /* task tick 50 ms -> re-init ~5 s */
#define DISP_DROP_LOG_EVERY   50      /* stuck bus must not spam the log */

/* ---- screen state (function first, cosmetic later) ----
 * The panel shows one of three states, never animations:
 *   IDLE  "SAN SANG"  (quiet, mic gated)
 *   LISTEN "DANG NGHE" (voice energy detected, streaming uplink)
 *   SPEAK "DANG NOI"   (TTS audio playing)
 * Priority SPEAK > LISTEN > IDLE. Emotion API is kept for compatibility but
 * no longer drives pixels. */
typedef enum {
    ST_IDLE = 0,      /* SAN SANG   - not in a conversation        */
    ST_LISTEN,        /* DANG NGHE  - mic armed / user speaking     */
    ST_THINK,         /* DANG NGHI  - speech sent, waiting for LLM  */
    ST_SPEAK,         /* DANG NOI   - TTS playing                   */
    ST_OFFLINE,       /* MAT KET NOI- no WiFi                       */
} disp_state_t;

/* ---- state (all guarded by s_lock except volatile counters) ---- */
static SemaphoreHandle_t s_lock;
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static bool s_inited;
static bool s_hw_ok;                  /* bus+device+init-seq done */
static uint8_t s_fb[DISP_PAGES * DISP_W];
static bool s_listening;
static bool s_thinking;
static bool s_offline;
static int s_dot_phase;
static int s_rendered = -1; /* last state painted on glass (-1 = never) */
static bool s_wifi, s_session;
static int s_vol = 50;                /* SPEC section 2.3 default */
static bool s_speaking;
static bool s_night;
static bool s_panel_off;              /* 0xAE already sent */
static bool s_dirty = true;           /* boot frame wanted */
static volatile uint32_t s_requested = 1;  /* boot frame */
static volatile uint32_t s_completed;
static uint32_t s_drops;
static uint32_t s_consec_err;
static uint32_t s_boot_ms;
static uint32_t s_last_request_ms;
static uint32_t s_last_complete_ms;
static uint32_t s_last_attempt_ms;

static disp_state_t display_state_locked(void);
static void gpio_clock_rescue(void);

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* Mark dirty + count a wanted frame. Call with s_lock held. */
static void mark_dirty_locked(void)
{
    if (!s_dirty) {
        s_dirty = true;
    }
    s_requested++;
    s_last_request_ms = now_ms();
}

static bool take_lock(void)
{
    if (!s_lock) {
        return false;
    }
    return xSemaphoreTake(s_lock, pdMS_TO_TICKS(DISP_LOCK_MS)) == pdTRUE;
}

/* Both lines must idle high or the bus is stuck (SPEC section 5.1 #1). */
static bool bus_high(void)
{
    return gpio_get_level(DISP_SDA_GPIO) == 1
        && gpio_get_level(DISP_SCL_GPIO) == 1;
}

/* ---- low-level I2C (finite timeouts, soft-fail) ---- */

static bool disp_cmd_batch(const uint8_t *cmd, size_t n)
{
    if (!s_dev || !cmd || n == 0 || n > 30) {
        return false;
    }
    uint8_t buf[31];
    buf[0] = 0x00; /* command stream */
    memcpy(&buf[1], cmd, n);
    return i2c_master_transmit(s_dev, buf, n + 1, DISP_XFER_MS) == ESP_OK;
}

static bool disp_cmd1(uint8_t c)
{
    return disp_cmd_batch(&c, 1);
}

/* Full SH1106 init (contrast 0x81 0x4F per SPEC section 5.6 #24). */
static bool disp_send_init_seq(void)
{
    static const uint8_t seq[] = {
        0xAE,             /* display off */
        0xD5, 0x80,       /* clock divide */
        0xA8, 0x3F,       /* multiplex 64 */
        0xD3, 0x00,       /* display offset 0 */
        0x40,             /* start line 0 */
        0x8D, 0x14,       /* charge pump on */
        0x20, 0x02,       /* page addressing */
        0xA1,             /* segment remap */
        0xC8,             /* COM scan descending */
        0xDA, 0x12,       /* COM pins */
        0x81, 0x4F,       /* contrast (SPEC) */
        0xD9, 0xF1,       /* precharge */
        0xDB, 0x40,       /* VCOMH */
        0xA4,             /* resume from RAM */
        0xA6,             /* normal (not inverted) */
        0xAF,             /* display on */
    };
    return disp_cmd_batch(seq, sizeof(seq));
}

/* Create bus + device. Caller must have bus_high() true first. */
static bool disp_hw_init(void)
{
    if (s_hw_ok) {
        return true;
    }
    if (!bus_high()) {
        return false;
    }
    i2c_master_bus_config_t bus_cfg;
    memset(&bus_cfg, 0, sizeof(bus_cfg));
    bus_cfg.i2c_port = DISP_I2C_PORT;
    bus_cfg.sda_io_num = DISP_SDA_GPIO;
    bus_cfg.scl_io_num = DISP_SCL_GPIO;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true; /* no ext pullups (2.4) */
    if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) {
        s_bus = NULL;
        return false;
    }
    i2c_device_config_t dev_cfg;
    memset(&dev_cfg, 0, sizeof(dev_cfg));
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = DISP_I2C_ADDR;
    dev_cfg.scl_speed_hz = DISP_I2C_HZ;
    if (i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev) != ESP_OK) {
        s_dev = NULL;
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        return false;
    }
    if (!disp_send_init_seq()) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        return false;
    }
    s_hw_ok = true;
    return true;
}

static void disp_hw_teardown(void)
{
    if (s_dev) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    if (s_bus) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
    s_hw_ok = false;
}

/* Write one page: set page + column base 2 (SH1106 132-wide RAM), then
 * 128 data bytes. False on any error (caller keeps dirty). */
static bool disp_write_page(int page, const uint8_t *col)
{
    uint8_t set[] = { 0x00, (uint8_t)(0xB0 | (page & 7)), 0x02, 0x10 };
    if (i2c_master_transmit(s_dev, set, sizeof(set), DISP_XFER_MS) != ESP_OK) {
        return false;
    }
    static uint8_t databuf[DISP_W + 1];
    databuf[0] = 0x40; /* data stream */
    memcpy(&databuf[1], col, DISP_W);
    return i2c_master_transmit(s_dev, databuf, sizeof(databuf),
                               DISP_XFER_MS) == ESP_OK;
}

/* ---- framebuffer ops ---- */

static void px(int x, int y, bool on)
{
    if (x < 0 || x >= DISP_W || y < 0 || y >= DISP_H) {
        return;
    }
    uint8_t *b = &s_fb[(y >> 3) * DISP_W + x];
    if (on) {
        *b |= (uint8_t)(1u << (y & 7));
    } else {
        *b &= (uint8_t)~(1u << (y & 7));
    }
}

static void hline(int x0, int x1, int y)
{
    if (x1 < x0) {
        int t = x0; x0 = x1; x1 = t;
    }
    for (int x = x0; x <= x1; x++) {
        px(x, y, true);
    }
}

static void line(int x0, int y0, int x1, int y1)
{
    int dx = x1 - x0 >= 0 ? x1 - x0 : x0 - x1;
    int dy = y1 - y0 >= 0 ? y1 - y0 : y0 - y1;
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    while (1) {
        px(x0, y0, true);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int e2 = err * 2;
        if (e2 > -dy) {
            err -= dy; x0 += sx;
        }
        if (e2 < dx) {
            err += dx; y0 += sy;
        }
    }
}

static void circle(int cx, int cy, int r)
{
    int x = r, y = 0, e = 1 - r;
    while (x >= y) {
        px(cx + x, cy + y, true); px(cx - x, cy + y, true);
        px(cx + x, cy - y, true); px(cx - x, cy - y, true);
        px(cx + y, cy + x, true); px(cx - y, cy + x, true);
        px(cx + y, cy - x, true); px(cx - y, cy - x, true);
        y++;
        if (e < 0) {
            e += 2 * y + 1;
        } else {
            x--; e += 2 * (y - x) + 1;
        }
    }
}

static void filled_circle(int cx, int cy, int r)
{
    for (int y = -r; y <= r; y++) {
        for (int x = -r; x <= r; x++) {
            if (x * x + y * y <= r * r) {
                px(cx + x, cy + y, true);
            }
        }
    }
}

/* Arc centered (cx,cy) radius r over [deg0,deg1] (screen coords y-down:
 * 0=east, 90=south, 180=west, 270=north), ~2 px thick, octant-tested. */
static void arc(int cx, int cy, int r, int deg0, int deg1)
{
    int o0 = deg0 / 45, o1 = deg1 / 45;
    for (int y = -r - 1; y <= r + 1; y++) {
        for (int x = -r - 1; x <= r + 1; x++) {
            int rr = x * x + y * y;
            if (rr < (r - 1) * (r - 1) || rr > (r + 1) * (r + 1)) {
                continue;
            }
            if (x == 0 && y == 0) {
                continue;
            }
            int ax = x < 0 ? -x : x, ay = y < 0 ? -y : y;
            int oct; /* 0=E,1=SE,2=S,3=SW,4=W,5=NW,6=N,7=NE (y-down) */
            if (ax >= 2 * ay) {
                oct = (x >= 0) ? 0 : 4;
            } else if (ay >= 2 * ax) {
                oct = (y >= 0) ? 2 : 6;
            } else if (x >= 0) {
                oct = (y >= 0) ? 1 : 7;
            } else {
                oct = (y >= 0) ? 3 : 5;
            }
            if (o0 <= oct && oct <= o1) {
                px(cx + x, cy + y, true);
            }
        }
    }
}

/* 5x7 LSB-top font, digits + 'V' only (status row volume). */
static const uint8_t FONT_DIGITS[10][5] = {
    { 0x3E, 0x51, 0x49, 0x45, 0x3E }, /* 0 */
    { 0x00, 0x42, 0x7F, 0x40, 0x00 }, /* 1 */
    { 0x42, 0x61, 0x51, 0x49, 0x46 }, /* 2 */
    { 0x21, 0x41, 0x45, 0x4B, 0x31 }, /* 3 */
    { 0x18, 0x14, 0x12, 0x7F, 0x10 }, /* 4 */
    { 0x27, 0x45, 0x45, 0x45, 0x39 }, /* 5 */
    { 0x3C, 0x4A, 0x49, 0x49, 0x30 }, /* 6 */
    { 0x01, 0x71, 0x09, 0x05, 0x03 }, /* 7 */
    { 0x36, 0x49, 0x49, 0x49, 0x36 }, /* 8 */
    { 0x06, 0x49, 0x49, 0x29, 0x1E }, /* 9 */
};
static const uint8_t FONT_V[5] = { 0x1F, 0x20, 0x40, 0x20, 0x1F };

static void draw_glyph(int x, int y, const uint8_t *cols)
{
    for (int c = 0; c < 5; c++) {
        uint8_t col = cols[c];
        for (int r = 0; r < 7; r++) {
            if (col & (1u << r)) {
                px(x + c, y + r, true);
            }
        }
    }
}

static void draw_vol(int x, int y, int vol)
{
    draw_glyph(x, y, FONT_V);
    x += 6;
    if (vol >= 100) {
        draw_glyph(x, y, FONT_DIGITS[1]); x += 6;
        draw_glyph(x, y, FONT_DIGITS[0]); x += 6;
        draw_glyph(x, y, FONT_DIGITS[0]);
    } else {
        draw_glyph(x, y, FONT_DIGITS[(vol / 10) % 10]); x += 6;
        draw_glyph(x, y, FONT_DIGITS[vol % 10]);
    }
}

/* ---- content ---- */

/* Status row (y 0..8): wifi arcs, session box, mic glyph, Vnn. */
static void draw_status(bool wifi, bool session, int vol)
{
    /* wifi: dot + 2 arcs when on, hollow dot when off */
    filled_circle(6, 5, 1);
    if (wifi) {
        arc(6, 5, 4, 225, 315);
        arc(6, 5, 7, 225, 315);
    } else {
        circle(6, 5, 3);
    }
    /* session: filled box when live, outline when idle */
    if (session) {
        for (int y = 1; y <= 7; y++) {
            hline(20, 26, y);
        }
    } else {
        hline(20, 26, 1); hline(20, 26, 7);
        for (int y = 1; y <= 7; y++) {
            px(20, y, true); px(26, y, true);
        }
    }
    /* mic: capsule + stand (informational: lit while awake) */
    for (int y = 1; y <= 5; y++) {
        hline(34, 38, y);
    }
    hline(33, 39, 6);
    line(36, 6, 36, 8); hline(33, 39, 8);
    draw_vol(98, 1, vol);
    hline(0, DISP_W - 1, 9);
}

/* 5x7 LSB-top font, only the letters needed for state labels
 * (DANG NGHE / DANG NOI / SAN SANG): A D E G H I N O S. */
static const uint8_t FONT_A[5] = { 0x7E, 0x11, 0x11, 0x11, 0x7E };
static const uint8_t FONT_D[5] = { 0x7F, 0x41, 0x41, 0x22, 0x1C };
static const uint8_t FONT_E[5] = { 0x7F, 0x49, 0x49, 0x49, 0x41 };
static const uint8_t FONT_G[5] = { 0x3E, 0x41, 0x49, 0x4B, 0x3A };
static const uint8_t FONT_H[5] = { 0x7F, 0x08, 0x08, 0x08, 0x7F };
static const uint8_t FONT_I[5] = { 0x00, 0x41, 0x7F, 0x41, 0x00 };
static const uint8_t FONT_N[5] = { 0x7F, 0x10, 0x08, 0x04, 0x7F };
static const uint8_t FONT_O[5] = { 0x3E, 0x41, 0x41, 0x41, 0x3E };
static const uint8_t FONT_S[5] = { 0x46, 0x49, 0x49, 0x49, 0x31 };
static const uint8_t FONT_M[5] = { 0x7F, 0x02, 0x0C, 0x02, 0x7F };
static const uint8_t FONT_K[5] = { 0x7F, 0x08, 0x14, 0x22, 0x41 };
static const uint8_t FONT_T[5] = { 0x01, 0x01, 0x7F, 0x01, 0x01 };

static const uint8_t *state_glyph(char c)
{
    switch (c) {
    case 'A': return FONT_A;
    case 'D': return FONT_D;
    case 'E': return FONT_E;
    case 'G': return FONT_G;
    case 'H': return FONT_H;
    case 'I': return FONT_I;
    case 'N': return FONT_N;
    case 'O': return FONT_O;
    case 'S': return FONT_S;
    case 'M': return FONT_M;
    case 'K': return FONT_K;
    case 'T': return FONT_T;
    default: return NULL; /* space + unknown: blank advance */
    }
}

/* Centered 5x7 caps label at row y (7 px tall). */
static void draw_text(int y, const char *s)
{
    size_t n = strlen(s);
    int w = (int)n * 6 - 1;
    int x = (DISP_W - w) / 2;
    if (x < 0) {
        x = 0;
    }
    for (size_t i = 0; i < n; i++) {
        const uint8_t *g = state_glyph(s[i]);
        if (g) {
            draw_glyph(x, y, g);
        }
        x += 6;
    }
}

/* State art, content area y 12..52 (status row owns 0..9, label at 54). */
static void draw_state(disp_state_t st)
{
    switch (st) {
    case ST_SPEAK: {
        /* megaphone + outgoing arcs (east bubble: octants 7 then 0..1) */
        for (int y = 30; y <= 44; y++) {
            hline(34, 40, y);
        }
        line(40, 30, 54, 24); line(40, 44, 54, 50);
        line(54, 24, 54, 50);
        arc(60, 37, 5, 315, 360); arc(60, 37, 5, 0, 45);
        arc(60, 37, 10, 315, 360); arc(60, 37, 10, 0, 45);
        arc(60, 37, 15, 315, 360); arc(60, 37, 15, 0, 45);
        draw_text(54, "DANG NOI");
        break;
    }
    case ST_LISTEN: {
        /* big mic capsule + sound bars both sides */
        for (int y = 22; y <= 36; y++) {
            hline(60, 68, y);
        }
        arc(64, 36, 4, 0, 180);
        hline(60, 68, 37);
        line(64, 41, 64, 46); hline(58, 70, 46);
        line(50, 26, 50, 42); line(78, 26, 78, 42);
        line(44, 29, 44, 39); line(84, 29, 84, 39);
        draw_text(54, "DANG NGHE");
        break;
    }
    case ST_THINK: {
        /* speech bubble with 1-3 dots (slow animation = "working") */
        circle(64, 33, 17);
        line(57, 48, 53, 52); /* bubble tail (short: no label overlap) */
        line(57, 48, 60, 51);
        int dots = 1 + (s_dot_phase % 3);
        for (int i = 0; i < dots; i++) {
            filled_circle(56 + i * 8, 33, 2);
        }
        draw_text(54, "DANG NGHI");
        break;
    }
    case ST_OFFLINE: {
        /* wifi arcs with a slash */
        arc(64, 36, 10, 200, 340);
        arc(64, 36, 17, 200, 340);
        filled_circle(64, 42, 2);
        line(50, 20, 78, 50);
        draw_text(54, "MAT KET NOI");
        break;
    }
    case ST_IDLE:
    default:
        /* check in ring = ready */
        circle(64, 34, 13);
        line(56, 34, 62, 40);
        line(62, 40, 73, 27);
        draw_text(54, "SAN SANG");
        break;
    }
}

/* Compose the full framebuffer from the snapshot. */
static void compose_frame(disp_state_t st, bool wifi, bool session, int vol)
{
    memset(s_fb, 0, sizeof(s_fb));
    draw_status(wifi, session, vol);
    draw_state(st);
}

/* Attempt one full frame. Bus-high check first; false = dropped (dirty
 * kept). Assumes s_hw_ok. */
static bool render_frame(disp_state_t st, bool wifi, bool session, int vol)
{
    if (!bus_high()) {
        return false;
    }
    compose_frame(st, wifi, session, vol);
    for (int p = 0; p < DISP_PAGES; p++) {
        if (!disp_write_page(p, &s_fb[p * DISP_W])) {
            return false;
        }
    }
    return true;
}

/* ---- render task (prio 2, render-on-change, <=5 fps) ---- */

static void disp_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "task on (prio %d, render-on-change <=5fps)", DISP_TASK_PRIO);
    uint32_t loop = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(50));
        loop++;
        if (!take_lock()) {
            continue; /* lock busy: skip slice, never exit */
        }
        /* Lazy (re-)init when the bus never came up. A stuck bus (no external
         * pullups: a glitch leaves SDA low) needs the 9-clock rescue, not
         * endless plain retries — otherwise the screen stays dark for the
         * watchdog's full 90 s. */
        if (!s_hw_ok && (loop % DISP_RETRY_INIT_LOOPS) == 0) {
            uint32_t up = now_ms() - s_boot_ms;
            bool need_rescue = up > 10000;
            xSemaphoreGive(s_lock);
            if (need_rescue) {
                ESP_LOGW(TAG, "panel still dark after %lus: bus rescue",
                         (unsigned long)(up / 1000));
                gpio_clock_rescue();
                if (s_bus) {
                    i2c_master_bus_reset(s_bus); /* best-effort */
                }
            }
            if (!take_lock()) {
                continue;
            }
            bool ok = disp_hw_init();
            if (ok) {
                mark_dirty_locked();
                s_consec_err = 0;
                ESP_LOGI(TAG, "hw (re-)init OK (50kHz, 0x4F)");
            }
            xSemaphoreGive(s_lock);
            continue;
        }
        /* Night sleep transitions: tiny on/off command, always allowed. */
        if (s_hw_ok && s_night != s_panel_off && bus_high()) {
            bool want_off = s_night;
            xSemaphoreGive(s_lock);
            bool ok = disp_cmd1(want_off ? 0xAE : 0xAF);
            if (take_lock()) {
                if (s_night == want_off) {
                    s_panel_off = want_off;
                    if (!want_off) {
                        mark_dirty_locked(); /* wake -> full repaint */
                    }
                }
                if (!ok) {
                    s_consec_err++;
                }
                xSemaphoreGive(s_lock);
            }
            if (ok) {
                ESP_LOGI(TAG, "panel %s", want_off ? "off (night)" : "on");
            }
            continue;
        }
        if (s_thinking && !s_speaking && s_hw_ok && !s_night && !s_panel_off
            && (now_ms() - s_last_attempt_ms) > 900) {
            s_dot_phase++;
            mark_dirty_locked(); /* slow dots: "still working" */
        }
        bool want = s_dirty && s_hw_ok && !s_night && !s_panel_off
            && (display_state_locked() != s_rendered
                || !s_speaking);
        /* While speaking only state transitions repaint (bus quiet rule);
         * state flips always repaint so NGHE/NOI/NGHI stays truthful. */
        uint32_t gap = now_ms() - s_last_attempt_ms;
        if (!want || gap < DISP_MIN_GAP_MS) {
            xSemaphoreGive(s_lock);
            continue;
        }
        /* Snapshot under lock, release before the slow I2C. */
        disp_state_t st = display_state_locked();
        bool wifi = s_wifi, sess = s_session;
        int vol = s_vol;
        xSemaphoreGive(s_lock);

        s_last_attempt_ms = now_ms();
        bool ok = render_frame(st, wifi, sess, vol);

        if (take_lock()) {
            if (ok) {
                s_completed++;
                s_last_complete_ms = now_ms();
                s_consec_err = 0;
                s_rendered = (int)st;
                /* Clear dirty only if nobody re-dirtied mid-frame. */
                s_dirty = false;
                ESP_LOGI(TAG, "frame ok: req=%lu done=%lu drops=%lu",
                         (unsigned long)s_requested,
                         (unsigned long)s_completed,
                         (unsigned long)s_drops);
            } else {
                s_drops++;
                s_consec_err++;
                /* dirty stays: full repaint retried <=3 s (200 ms tick). */
                if (s_drops == 1 || (s_drops % DISP_DROP_LOG_EVERY) == 0) {
                    ESP_LOGW(TAG, "frame dropped (bus low/err): "
                             "req=%lu done=%lu drops=%lu consec=%lu",
                             (unsigned long)s_requested,
                             (unsigned long)s_completed,
                             (unsigned long)s_drops,
                             (unsigned long)s_consec_err);
                }
                if (!s_hw_ok || s_consec_err > 200) {
                    disp_hw_teardown(); /* force re-init path */
                }
            }
            xSemaphoreGive(s_lock);
        }
    }
}

/* ---- public API ---- */

void display_init(void)
{
    if (s_inited) {
        ESP_LOGI(TAG, "already init");
        return;
    }
    s_boot_ms = now_ms();
    s_last_request_ms = s_boot_ms;
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) {
        ESP_LOGE(TAG, "mutex OOM, display disabled (no panic)");
        return;
    }
    /* Belt-and-braces pullups; the bus driver also enables them. */
    gpio_pullup_en(DISP_SDA_GPIO);
    gpio_pullup_en(DISP_SCL_GPIO);
    if (take_lock()) {
        if (disp_hw_init()) {
            ESP_LOGI(TAG, "hw init OK: SH1106 128x64 0x3C 50kHz "
                     " pullups, contrast 0x4F");
        } else {
            ESP_LOGW(TAG, "hw init deferred (bus low?): task retries");
        }
        xSemaphoreGive(s_lock);
    }
    s_inited = true;
    if (xTaskCreateWithCaps(disp_task, "display", DISP_TASK_STACK,
                        NULL, DISP_TASK_PRIO, NULL, MALLOC_CAP_SPIRAM) != pdPASS) {
        ESP_LOGW(TAG, "task create failed (setters still safe)");
        return;
    }
    ESP_LOGI(TAG, "init OK: boot state screen wanted "
             "(req=%lu)", (unsigned long)s_requested);
}

/* Emotion API kept for compatibility; the state screen ignores it
 * (function-first pass: no cosmetic faces). Logged only. */
void display_set_emotion(const char *emotion)
{
    ESP_LOGI(TAG, "emotion=%s (ignored: state screen active)",
             emotion ? emotion : "(null)");
}

void display_set_listening(bool listening)
{
    if (!take_lock()) {
        ESP_LOGW(TAG, "set_listening skipped: lock busy");
        return;
    }
    if (listening != s_listening) {
        s_listening = listening;
        mark_dirty_locked();
        ESP_LOGI(TAG, "listening=%d", listening ? 1 : 0);
    }
    xSemaphoreGive(s_lock);
}

/* Priority: speaking > thinking > listening > offline > idle. */
static disp_state_t display_state_locked(void)
{
    if (s_speaking) {
        return ST_SPEAK;
    }
    if (s_thinking) {
        return ST_THINK;
    }
    if (s_listening) {
        return ST_LISTEN;
    }
    if (s_offline) {
        return ST_OFFLINE;
    }
    return ST_IDLE;
}

void display_set_thinking(bool thinking)
{
    if (!take_lock()) {
        return;
    }
    if (thinking != s_thinking) {
        s_thinking = thinking;
        mark_dirty_locked();
        ESP_LOGI(TAG, "thinking=%d", thinking ? 1 : 0);
    }
    xSemaphoreGive(s_lock);
}

void display_set_offline(bool offline)
{
    if (!take_lock()) {
        return;
    }
    if (offline != s_offline) {
        s_offline = offline;
        mark_dirty_locked();
        ESP_LOGI(TAG, "offline=%d", offline ? 1 : 0);
    }
    xSemaphoreGive(s_lock);
}

void display_set_status(bool wifi, bool session, int vol)
{
    if (vol < 0) {
        vol = 0;
    }
    if (vol > 100) {
        vol = 100;
    }
    if (!take_lock()) {
        ESP_LOGW(TAG, "set_status skipped: lock busy");
        return;
    }
    if (wifi != s_wifi || session != s_session || vol != s_vol) {
        s_wifi = wifi;
        s_session = session;
        s_vol = vol;
        mark_dirty_locked();
    }
    xSemaphoreGive(s_lock);
}

void display_set_speaking(bool speaking)
{
    if (!take_lock()) {
        ESP_LOGW(TAG, "set_speaking skipped: lock busy");
        return;
    }
    if (speaking != s_speaking) {
        s_speaking = speaking;
        mark_dirty_locked(); /* state flip always repaints (even mid-speech) */
        ESP_LOGI(TAG, "speaking=%d", speaking ? 1 : 0);
    }
    xSemaphoreGive(s_lock);
}

void display_set_night_sleep(bool sleeping)
{
    if (!take_lock()) {
        ESP_LOGW(TAG, "set_night skipped: lock busy");
        return;
    }
    if (sleeping != s_night) {
        s_night = sleeping;
        ESP_LOGI(TAG, "night=%d", sleeping ? 1 : 0);
        /* task sends 0xAE/0xAF and repaints on wake. */
    }
    xSemaphoreGive(s_lock);
}

uint32_t display_completed_frames(void)
{
    return s_completed;
}

uint32_t display_requested_frames(void)
{
    return s_requested;
}

bool display_healthy(void)
{
    if (!s_inited) {
        return false;
    }
    uint32_t t = now_ms();
    if (!take_lock()) {
        return true; /* lock busy = task alive; not our verdict to fail */
    }
    bool ok = true;
    if (s_dirty && s_requested > s_completed
        && (t - s_last_request_ms) > DISP_UNHEALTHY_MS) {
        ok = false; /* wanted frame stuck >90 s */
    }
    if (s_completed == 0 && (t - s_boot_ms) > DISP_UNHEALTHY_MS) {
        ok = false; /* bus never came up >90 s after boot */
    }
    xSemaphoreGive(s_lock);
    return ok;
}

/* 9 full-byte clocks + STOP on bare GPIO, then pullups + re-init. */
static void gpio_clock_rescue(void)
{
    gpio_reset_pin(DISP_SCL_GPIO);
    gpio_reset_pin(DISP_SDA_GPIO);
    gpio_set_direction(DISP_SCL_GPIO, GPIO_MODE_OUTPUT_OD);
    gpio_set_direction(DISP_SDA_GPIO, GPIO_MODE_OUTPUT_OD);
    gpio_pullup_en(DISP_SCL_GPIO);
    gpio_pullup_en(DISP_SDA_GPIO);
    gpio_set_level(DISP_SDA_GPIO, 1);
    gpio_set_level(DISP_SCL_GPIO, 1);
    esp_rom_delay_us(50);
    for (int i = 0; i < 9; i++) {
        gpio_set_level(DISP_SCL_GPIO, 0);
        esp_rom_delay_us(50);
        gpio_set_level(DISP_SCL_GPIO, 1);
        esp_rom_delay_us(50);
    }
    /* STOP: SDA low -> SCL high -> SDA high. */
    gpio_set_level(DISP_SDA_GPIO, 0);
    esp_rom_delay_us(50);
    gpio_set_level(DISP_SCL_GPIO, 1);
    esp_rom_delay_us(50);
    gpio_set_level(DISP_SDA_GPIO, 1);
    esp_rom_delay_us(50);
    gpio_pullup_en(DISP_SDA_GPIO);
    gpio_pullup_en(DISP_SCL_GPIO);
}

bool display_rescue_bus(void)
{
    if (!take_lock()) {
        ESP_LOGW(TAG, "rescue skipped: lock busy");
        return false;
    }
    ESP_LOGI(TAG, "rescue: bus_reset -> 9 clocks+STOP -> re-init");
    if (s_bus) {
        i2c_master_bus_reset(s_bus); /* best-effort, ignore result */
    }
    disp_hw_teardown();
    xSemaphoreGive(s_lock); /* GPIO bit-bang needs no lock held */

    gpio_clock_rescue();

    if (!take_lock()) {
        return false;
    }
    bool ok = disp_hw_init();
    if (ok) {
        mark_dirty_locked();
        s_consec_err = 0;
        ESP_LOGI(TAG, "rescue OK: panel answered init");
    } else {
        ESP_LOGW(TAG, "rescue FAILED: panel silent (watchdog owns 90 s)");
    }
    xSemaphoreGive(s_lock);
    return ok;
}
