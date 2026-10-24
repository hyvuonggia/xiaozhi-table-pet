#pragma once

/* Mic path: INMP441 on I2S0 STD RX mono 24 kHz -> resample 16 kHz ->
 * 60 ms frames (960 samples) -> Opus encode (16 kHz mono).
 *
 * Stability contract (SPEC section 5.2):
 *  - One failed I2S read NEVER ends the task: skip a 32 ms slice, continue.
 *  - First read uses a long finite timeout (>= 300 ms, DMA empty at boot),
 *    later reads use a short finite timeout. No infinite waits anywhere.
 *  - audio_in_last_ok_tick() exposes the last successful read tick so a
 *    supervisor can reboot after 25 s with no success.
 */

#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Idempotent. Starts I2S0 RX, mic task (prio 8), resampler, Opus encoder. */
void audio_in_start(void);

/* Idempotent. Stops the mic task and releases the RX channel + encoder. */
void audio_in_stop(void);

/* Tick of the last successful I2S read, or 0 if none yet. */
TickType_t audio_in_last_ok_tick(void);

/* Pop one encoded Opus frame (60 ms) for uplink.
 * Returns encoded bytes (>0), 0 on timeout/empty, <0 when not running. */
int audio_in_pop_opus(uint8_t *buf, size_t cap, uint32_t timeout_ms);

/* Most recent 16 kHz frame RMS (mean-removed), updated by the mic task. */
uint32_t audio_in_recent_rms(void);

/* Opus stats for diagnostics (cumulative since start). */
void audio_in_opus_stats(uint32_t *frames, uint32_t *bytes, uint32_t *last_bytes);

/* Boot self-test (no server needed). Spawns a task, returns immediately.
 * The task waits 5 s, logs mic RMS over 3 s, plays a 440 Hz tone, logs
 * Opus bytes/frame and underruns. Log-only, never panics. Idempotent. */
void audio_selftest(void);

#ifdef __cplusplus
}
#endif
