/* Buttons lane — BOOT + volume keys (SPEC §2.2 pins, §2.3 volume).
 *
 * Pins (active-low, internal pull-up, polled — no ISR complexity):
 *  - GPIO0  BOOT: hold >= 2 s -> display-mode hook stub (log only).
 *  - GPIO40 VOL+: press +10 (clamp 100), hold >= 2 s -> 100.
 *  - GPIO39 VOL-: press -10 (clamp 0),   hold >= 2 s -> 0.
 *
 * Shared volume variable starts at 50 (SPEC §2.3, default <= 50). The audio
 * lane reads it via buttons_get_volume(). Single writer (poll task) plus
 * rare test writes via buttons_set_volume(); the int itself is the shared
 * state — no mutex shared with any other lane (SPEC lock contract: display
 * shares NO mutex with audio/state/servo).
 *
 * Poll task: priority 3, 50 ms period, 2-sample software debounce
 * (~100 ms), all waits finite. Every press/hold/release is logged.
 *
 * No abort-on-error or infinite-wait primitives anywhere (CI-banned, SPEC §6).
 */
#ifndef BUTTONS_H
#define BUTTONS_H

/* Configure GPIOs and start the poll task (prio 3, 50 ms). Idempotent. */
void buttons_init(void);

/* Current volume 0..100 (starts at 50). Safe to call from any task. */
int buttons_get_volume(void);

/* Clamp to 0..100, store, log. Intended for tests / remote control. */
void buttons_set_volume(int vol);

#endif /* BUTTONS_H */
