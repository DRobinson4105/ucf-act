#include "pwm_motor.h"

#include <inttypes.h>
#include <stdbool.h>

#include "esp_log.h"

static const char *TAG = "pwm_motor";

static pwm_motor_config_t s_cfg;
static bool s_initialized;

static uint32_t clamp_us(uint32_t pulse_us)
{
    if (pulse_us < s_cfg.min_pulse_us) {
        return s_cfg.min_pulse_us;
    }

    if (pulse_us > s_cfg.max_pulse_us) {
        return s_cfg.max_pulse_us;
    }

    return pulse_us;
}

static uint32_t us_to_duty(uint32_t pulse_us)
{
    const uint32_t period_us = 1000000U / s_cfg.freq_hz;
    const uint32_t duty_max = (uint32_t)((1ULL << (uint32_t)s_cfg.duty_resolution) - 1ULL);

    return (uint32_t)(((uint64_t)pulse_us * duty_max) / period_us);
}

static esp_err_t validate_config(const pwm_motor_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!GPIO_IS_VALID_OUTPUT_GPIO(cfg->gpio_num) || cfg->freq_hz == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!(cfg->min_pulse_us < cfg->neutral_pulse_us && cfg->neutral_pulse_us < cfg->max_pulse_us)) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

pwm_motor_config_t pwm_motor_default_config(gpio_num_t gpio_num)
{
    pwm_motor_config_t cfg = {
        .gpio_num = gpio_num,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .channel = LEDC_CHANNEL_0,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .clk_cfg = LEDC_AUTO_CLK,
        .freq_hz = 50U,
        .min_pulse_us = 500U,
        .neutral_pulse_us = 1500U,
        .max_pulse_us = 2500U,
    };

    return cfg;
}

esp_err_t pwm_motor_init(const pwm_motor_config_t *cfg)
{
    ledc_timer_config_t timer_cfg = {0};
    ledc_channel_config_t channel_cfg = {0};
    esp_err_t err = validate_config(cfg);

    if (err != ESP_OK) {
        return err;
    }

    s_cfg = *cfg;

    timer_cfg.speed_mode = s_cfg.speed_mode;
    timer_cfg.duty_resolution = s_cfg.duty_resolution;
    timer_cfg.timer_num = s_cfg.timer_num;
    timer_cfg.freq_hz = s_cfg.freq_hz;
    timer_cfg.clk_cfg = s_cfg.clk_cfg;
    timer_cfg.deconfigure = false;

    err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return err;
    }

    channel_cfg.gpio_num = s_cfg.gpio_num;
    channel_cfg.speed_mode = s_cfg.speed_mode;
    channel_cfg.channel = s_cfg.channel;
    channel_cfg.intr_type = LEDC_INTR_DISABLE;
    channel_cfg.timer_sel = s_cfg.timer_num;
    channel_cfg.duty = us_to_duty(s_cfg.neutral_pulse_us);
    channel_cfg.hpoint = 0U;
    channel_cfg.flags.output_invert = 0U;
    channel_cfg.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;

    err = ledc_channel_config(&channel_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;

    ESP_LOGI(TAG,
             "PWM ready gpio=%d freq=%" PRIu32 "Hz neutral=%" PRIu32 "us min=%" PRIu32 "us max=%" PRIu32 "us",
             (int)s_cfg.gpio_num,
             s_cfg.freq_hz,
             s_cfg.neutral_pulse_us,
             s_cfg.min_pulse_us,
             s_cfg.max_pulse_us);
    return ESP_OK;
}

esp_err_t pwm_motor_set_us(uint32_t pulse_us)
{
    uint32_t duty;
    esp_err_t err;

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    pulse_us = clamp_us(pulse_us);
    duty = us_to_duty(pulse_us);

    err = ledc_set_duty(s_cfg.speed_mode, s_cfg.channel, duty);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_set_duty failed: %s", esp_err_to_name(err));
        return err;
    }

    err = ledc_update_duty(s_cfg.speed_mode, s_cfg.channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_update_duty failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGD(TAG, "pulse=%" PRIu32 "us duty=%" PRIu32, pulse_us, duty);
    return ESP_OK;
}

esp_err_t pwm_motor_set_normalized(float value)
{
    uint32_t pulse_us;

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (value < 0.0f) {
        value = 0.0f;
    } else if (value > 1.0f) {
        value = 1.0f;
    }

    pulse_us = s_cfg.min_pulse_us +
               (uint32_t)(value * (float)(s_cfg.max_pulse_us - s_cfg.min_pulse_us));
    return pwm_motor_set_us(pulse_us);
}

esp_err_t pwm_motor_neutral(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return pwm_motor_set_us(s_cfg.neutral_pulse_us);
}
