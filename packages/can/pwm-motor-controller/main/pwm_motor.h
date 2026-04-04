#pragma once

#include <stdint.h>

/**
 * PWM motor controller — 50 Hz LEDC output for servo / RC-style motors.
 *
 * GPIO and timer constants are defined in pwm_motor.c; edit them there to
 * match your hardware.
 *
 * Pulse width → position mapping (standard servo / ESC convention):
 *    500 µs  — minimum
 *   1500 µs  — neutral / stop
 *   2000 µs  — full forward / maximum
 */

/**
 * Initialise the LEDC timer and channel.  Must be called once before any
 * other function in this module.  Output starts at neutral (1500 µs).
 */
void pwm_motor_init(void);

/**
 * Set the pulse width directly.
 *
 * @param pulse_us  Pulse width in microseconds.  Clamped to [PWM_MIN_US,
 *                  PWM_MAX_US] defined in pwm_motor.c.
 */
void pwm_motor_set_us(uint32_t pulse_us);

/**
 * Set position as a normalised value.
 *
 * @param value  0.0 = minimum (500 µs), 0.5 = neutral (1500 µs),
 *               1.0 = maximum (2000 µs).  Clamped to [0.0, 1.0].
 */
void pwm_motor_set_normalized(float value);

/** Drive to neutral (1500 µs). */
void pwm_motor_neutral(void);
