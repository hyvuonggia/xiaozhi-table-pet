#pragma once

/* Speaker path: Opus decode at 16 kHz mono (== server rate, NO resample) ->
 * I2S1 STD TX mono 16 kHz to MAX98357A.
 *
 * Stability contract (SPEC section 5.3):
 *  - Playback queue holds >= 8 frames (~480 ms) so CPU stalls don't gap audio.
 *  - After output enable, ~120 ms of silence is pushed first (cold amp/DMA
 *    eats the first syllable otherwise).
 *  - Every I2S write uses a finite timeout; a missed write drops that frame
 *    and counts an underrun instead of stalling the task.
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Idempotent. Starts I2S1 TX, speaker task (prio 5), Opus decoder.
 * Pushes ~120 ms silence right after enable. */
void audio_out_start(void);

/* Idempotent. Stops the speaker task and releases the TX channel + decoder. */
void audio_out_stop(void);

/* Queue one Opus packet for playback. Copies data; drops on full queue.
 * Returns 0 on queued, <0 when dropped / not running. Never blocks. */
int audio_out_push_opus(const uint8_t *data, size_t len);

/* Play a sine tone (blocking, finite timeouts, log-only on error).
 * freq_hz clamped to 50..4000, ms clamped to 50..5000. */
void audio_out_tone(int freq_hz, int ms);

/* Playback volume 0..100, default 50 (SPEC section 2.3). Linear attenuation. */
void audio_out_set_volume(int vol_0_100);

/* Cumulative dropped output frames (queue full or write miss). */
uint32_t audio_out_underruns(void);

#ifdef __cplusplus
}
#endif
