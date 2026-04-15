#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t gpio_num;
    ledc_mode_t speed_mode;
    ledc_timer_t timer_num;
    ledc_channel_t channel;
    ledc_timer_bit_t duty_resolution;
    ledc_clk_cfg_t clk_cfg;
    uint32_t freq_hz;
    uint32_t min_pulse_us;
    uint32_t neutral_pulse_us;
    uint32_t max_pulse_us;
} pwm_motor_config_t;

/**
 * Return a baseline 50 Hz servo/ESC-style PWM config for the selected GPIO.
 */
pwm_motor_config_t pwm_motor_default_config(gpio_num_t gpio_num);

/**
 * Configure the LEDC timer/channel and start output at neutral.
 */
esp_err_t pwm_motor_init(const pwm_motor_config_t *cfg);

/**
 * Set the PWM pulse width in microseconds.
 *
 * Values outside the configured range are clamped.
 */
esp_err_t pwm_motor_set_us(uint32_t pulse_us);

/**
 * Set output as a normalized value.
 *
 * 0.0 = min pulse, 0.5 = neutral, 1.0 = max pulse.
 * Values outside [0.0, 1.0] are clamped.
 */
esp_err_t pwm_motor_set_normalized(float value);

/**
 * Drive output to the configured neutral pulse width.
 */
esp_err_t pwm_motor_neutral(void);

#ifdef __cplusplus
}
#endif
