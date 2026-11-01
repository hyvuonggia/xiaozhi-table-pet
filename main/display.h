#pragma once

/* Hardened SH1106 OLED driver (128x64, I2C 0x3C, SDA=GPIO41/SCL=GPIO42).
 *
 * SPEC section 2.4/5.1/5.6/9: 50 kHz, internal pullups only, bus errors are
 * EXPECTED (no external pullups) and must never hang or panic.
 *
 * Hardening contract (SPEC section 5.1):
 *  - Raw i2c_master only (NO panel-IO: its infinite timeout wedged the old
 *    firmware). Every transaction has a FINITE timeout (<=100 ms).
 *  - Before EVERY frame: SDA && SCL must both read high, else the frame is
 *    DROPPED (dirty kept) and we return. Never blocks.
 *  - Render-on-change only, <=5 fps (200 ms min interval between attempts).
 *  - Own private static mutex ONLY; try-lock <=2 s, unlock-only-if-locked.
 *    Shared with NO other module (P2 lock contract).
 *  - Dropped/dirty frames repaint fully within <=3 s (retry every 200 ms).
 *  - Frame watchdog: requested vs completed counters below; the watchdog
 *    lane (90 s-unhealthy -> reboot) uses display_healthy().
 *  - Bus rescue: display_rescue_bus() = bus_reset -> 9 GPIO clocks + STOP
 *    -> re-enable pullups -> re-init. Returns ok/fail.
 *
 * Content (function-first pass, user decision 2026-10-25): NO expressive
 * eyes — the panel shows one of three STATES with icon + caps label:
 * SAN SANG (idle) / DANG NGHE (listening) / DANG NGHI (thinking) /
 * DANG NOI (speaking) / MAT KET NOI (no WiFi).
 * Priority: speaking > thinking > listening > offline > idle.
 * Status icon row (wifi/session/mic/vol), night sleep 01:00-07:00
 * server-local (caller calls display_set_night_sleep; we blank via 0xAE),
 * contrast 0x81 0x4F. State transitions always repaint (even mid-speech);
 * nothing else repaints while speaking. Boot shows SAN SANG + icons.
 * No LVGL: direct page framebuffer.
 *
 * State wiring: voice path calls display_set_listening() (voice energy gate)
 * and display_set_speaking() (TTS start/stop).
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Idempotent. Inits pullups + I2C + SH1106, paints boot frame
 * (SAN SANG + icons), spawns the render task (prio 2). Soft-fails:
 * if the bus is down at boot the task keeps retrying; never aborts. */
void display_init(void);

/* Compatibility no-op (state screen ignores emotions). Logged only. */
void display_set_emotion(const char *emotion);

/* true = mic armed / user speaking. Drives DANG NGHE. */
void display_set_listening(bool listening);

/* true = speech sent, waiting for the assistant's answer. Drives DANG NGHI
 * (slow dot animation). Cleared when TTS starts/stops. */
void display_set_thinking(bool thinking);

/* true = no WiFi. Drives MAT KET NOI (only when idle). */
void display_set_offline(bool offline);

/* Status row state. vol is clamped to 0..100. */
void display_set_status(bool wifi, bool session, int vol);

/* true while TTS audio is playing: drives DANG NOI. The
 * false transition repaints with the latest state. */
void display_set_speaking(bool speaking);

/* true = night sleep: panel off (0xAE), frames skipped. false = panel
 * on (0xAF) + full repaint. The 01:00-07:00 clock lives in the caller. */
void display_set_night_sleep(bool sleeping);

/* Frame watchdog counters (SPEC section 5.1 #4): requested counts frames
 * the logic wanted; completed counts frames fully written. A growing gap
 * with no recovery means the bus/panel is dead. */
uint32_t display_completed_frames(void);
uint32_t display_requested_frames(void);

/* False when: never inited, or a wanted frame has been stuck unpainted
 * for >90 s, or the bus never came up >90 s after boot. Idle with no
 * pending frames reads healthy (render-on-change may be quiet for hours). */
bool display_healthy(void);

/* Rescue from any task (watchdog lane): bus_reset -> release driver ->
 * 9 GPIO clocks (full byte) + STOP -> pullups back on -> re-init ->
 * mark dirty for repaint. Try-lock <=2 s (false when busy). True = the
 * panel answered init after rescue. */
bool display_rescue_bus(void);

#ifdef __cplusplus
}
#endif
