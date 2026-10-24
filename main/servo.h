/* Servo lane — SG90/MG90 2-axis head (SPEC §2.2 pins, §5.5 stability).
 *
 * THE servo lock (SPEC §5.5 #19): exactly ONE static mutex lives in servo.c
 * and guards ALL PWM writes. Every move funnels through one task-safe
 * function (servo_move_both, static in servo.c). Lock discipline (SPEC §5.4
 * #17): xSemaphoreTake with timeout <= 2 s (never an infinite wait), and the
 * caller unlocks ONLY if the take succeeded. Never drive both LEDC channels
 * from two tasks directly — call servo_center / servo_nod_once /
 * servo_shake_once and let the funnel serialize.
 *
 * Hardware: GPIO11 = horizontal (LEDC ch0), GPIO12 = vertical (LEDC ch1),
 * 50 Hz, 500–2500 us pulse. Idle pose is 90/90 to reduce holding current
 * (SPEC §5.5 #20). Motion commands are debounced 3 s (SPEC §5.5 #22) because
 * VAD can repeat the same sentence.
 *
 * No abort-on-error or infinite-wait primitives anywhere (CI-banned, SPEC §6).
 */
#ifndef SERVO_H
#define SERVO_H

/* Install LEDC, create THE servo lock, center to 90/90, then run a boot
 * demo: exactly ONE nod with an angle trace in the monitor log. */
void servo_init(void);

/* Return both axes to 90/90 (idle pose). Not debounced. */
void servo_center(void);

/* EXACTLY one nod: vertical down-up, one cycle (SPEC §5.5 #21).
 * Debounced: ignored if a motion ran < 3 s ago. */
void servo_nod_once(void);

/* EXACTLY one shake: horizontal left-right, one cycle.
 * Debounced: ignored if a motion ran < 3 s ago. */
void servo_shake_once(void);

#endif /* SERVO_H */
