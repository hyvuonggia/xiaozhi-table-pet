#pragma once

/* Wake word gating (WAKE lane): real WakeNet keyword spotting on the
 * 16 kHz mono mic stream, replacing the old energy-only auto-open.
 *
 * Flow: audio_in mic task resamples 24 k -> 16 k, then calls
 * wakeword_feed_pcm16() once per 60 ms frame (960 int16 samples).
 * A dedicated detect task (prio 4, below mic prio 8) drains an internal
 * ring and runs esp-sr WakeNet (wn9_alexa, "Alexa") chunk by chunk.
 * voice_path.c opens its conversation window on wakeword_consume().
 *
 * Stability contract: init soft-fails (log model name + wake phrase,
 * never panic); feed is non-blocking memcpy-only (never disturbs Opus
 * encode); all waits finite, soft-fail everywhere, CI grep-ban clean;
 * all waits finite; OOM/degraded paths log and continue without wake.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Idempotent. Loads srmodels from the "model" partition, creates the
 * wn9_alexa WakeNet instance (DET_MODE_95), spawns the detect task.
 * Logs the model name + wake phrase. Never panics. */
void wakeword_init(void);

/* Copy 16 kHz mono int16 samples into the detect ring. Cheap
 * (memcpy under a short critical section), never blocks, safe to call
 * from the mic task hot path. Drops oldest on overflow (counted). */
void wakeword_feed_pcm16(const int16_t *pcm, int n);

/* Returns true exactly once per detected wake word (clears the flag).
 * voice_path.c polls this to open the conversation window. */
bool wakeword_consume(void);

/* Tick of the most recent detection, or 0 if none yet. */
TickType_t wakeword_last_wake_tick(void);

/* Neural VAD (vadnet1_medium, same 16 kHz feed): discriminates real speech
 * from room music/TV noise. Energy/RMS alone cannot — music trips it.
 * voice_path.c confirms streaming ONLY on VAD speech, never on energy.
 *
 * Init is fail-closed (loud log, never panic): if the model is missing,
 * wakeword_vad_ready() stays false and the window never opens on speech.
 * All accessors are cheap, non-blocking, and safe from any task. */
bool wakeword_vad_ready(void);            /* model loaded + detecting */
uint32_t wakeword_vad_streak(void);       /* consecutive VAD_SPEECH frames */
TickType_t wakeword_vad_last_speech_tick(void); /* last VAD_SPEECH tick, 0 = none */
bool wakeword_vad_speech_now(void);       /* most recent VAD frame was speech */
void wakeword_vad_reset(void);            /* clear streak/state (call on wake) */
/* Drop every buffered VAD frame and clear the streak. Called when the
 * follow-up window opens: the VAD ring still holds the tail of our own TTS
 * (the mic hears the speaker), which otherwise re-opens the channel ~40 ms
 * after the answer and makes the robot talk to itself. */
void wakeword_vad_flush(void);

#ifdef __cplusplus
}
#endif
