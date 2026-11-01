#pragma once

/* Voice path P2b: mic Opus uplink -> WS, WS binary -> speaker.
 *
 * Wiring (see main.c WS handler, narrow routing only):
 *  - voice_path_start(s_ws) once after WS client creation.
 *  - WS DATA frames: binary -> voice_path_on_ws_binary(),
 *    text -> voice_path_on_ws_text().
 *  - WS CONNECTED/DISCONNECTED -> on_ws_connected()/on_ws_disconnected().
 *
 * Protocol (SPEC section 3, E1/E2 research):
 *  - Uplink: raw Opus 16 kHz mono 60 ms frames as WS BINARY (v1, not
 *    base64). Wake-word gated (WAKE lane): streaming starts ONLY on a
 *    WakeNet "Alexa" trigger (listen{state:start,mode:auto}), continues
 *    while the conversation window is open (min 6 s, +2 s quiet hangover),
 *    then listen{state:stop,mode:auto} so the server answers immediately.
 *    Room noise can never open the stream.
 *  - Downlink: binary frames -> audio_out_push_opus() (drop-on-full).
 *    Text frames: tts text is filtered by msg_filter_is_tool_json() first
 *    (E2 rule: never speak tool JSON); mcp JSON-RPC answered minimally.
 *
 * BOOT short-press re-listen: buttons lane owns buttons.c; call
 * voice_path_relisten() from the button hook (orchestrator wires it).
 * Barge-in (abort while speaking on loud mic input) is SKIPPED: no VAD
 * on device. P3 owns it.
 *
 * Stability: finite waits/timeouts only, drop-on-full, soft-fail on
 * OOM/bad JSON/NULL, no shared mutex with display (lock contract).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_websocket_client.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Idempotent. Stores WS handle + channel callbacks, spawns uplink task
 * (prio 4). open_cb() is called on a wake word (channel opens only for the
 * conversation), close_cb() when the window closes (no idle server session,
 * so the server never speaks its goodbye). */
/* Update the WS handle when the audio channel is recreated per conversation.
 * NULL = channel gone (session/streaming state is cleared). */
void voice_path_set_ws(esp_websocket_client_handle_t ws);

void voice_path_start(esp_websocket_client_handle_t ws,
                      void (*open_cb)(void), void (*close_cb)(void));

/* Lifecycle hooks from the WS event handler (narrow routing in main.c). */
void voice_path_on_ws_connected(void);
void voice_path_on_ws_disconnected(void);

/* Downlink entry points from the WS DATA handler. Both copy/parse
 * synchronously; safe to call with the event data pointer. */
void voice_path_on_ws_text(const char *data, size_t len);
void voice_path_on_ws_binary(const uint8_t *data, size_t len);

/* Re-send listen{state:start,mode:auto} (BOOT short-press hook).
 * Best-effort, finite timeout, log-only on failure. */
void voice_path_relisten(void);

/* True while session is active and we have asked the server to listen. */
bool voice_path_is_listening(void);

#ifdef __cplusplus
}
#endif
