#pragma once

/* Supervisor (SPEC §5.4#16, §5.2#10, §5.1#4/#7; E3 lessons).
 *
 * Timestamp-watch task, prio 3, ticks every 1 s (NO long vTaskDelay inside
 * the monitor — every deadline is a timestamp comparison so observation
 * continues while waiting):
 *  - main-loop heartbeat: main.c stamps watchdog_heartbeat() each 1 Hz
 *    loop; >30 s stale -> log + esp_restart.
 *  - mic: audio_in_last_ok_tick() >25 s stale WHILE session active
 *    (voice_path_is_listening()) -> log + esp_restart. Skipped while no
 *    session or no successful read yet (boot priming).
 *  - display frames (SPEC §5.1#4: completed frames, never task liveness):
 *    display_requested_frames() vs display_completed_frames() progress plus
 *    the lane's own display_healthy(). Unhealthy >90 s -> display_rescue_bus()
 *    first, reboot if still dead; counters forgiven after 10 min healthy.
 *  - volume wiring (P2 F6): polls buttons_get_volume() -> 
 *    audio_out_set_volume() on change; BOOT short-press edge-detect here
 *    (100 ms sub-samples inside the 1 s tick — buttons.c owns the real
 *    debounce, this lane never touches it) -> voice_path_relisten().
 *    BOOT hold >= 2 s = display-stub parity (log only, no-op).
 *
 * Reboot accounting: esp_reset_reason() + RTC-retained last-reboot-reason
 * are logged at watchdog_init(); the retained word is set before every
 * esp_restart() issued from this file. No banned tokens (CI-checked).
 */

#ifdef __cplusplus
extern "C" {
#endif

/* Idempotent. Logs reset reason, starts supervisor task (prio 3). Call last
 * at boot, after buttons/servo/audio/display init. */
void watchdog_init(void);

/* Stamp the main-loop heartbeat. Call once per 1 Hz loop iteration. */
void watchdog_heartbeat(void);

#ifdef __cplusplus
}
#endif
