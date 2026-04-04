#include "pwm_motor.h"

#include "driver/ledc.h"
#include "esp_log.h"

// ── Hardware configuration ────────────────────────────────────────────────────

#define PWM_GPIO        GPIO_NUM_21
#define PWM_FREQ_HZ     50
#define PWM_RESOLUTION  LEDC_TIMER_16_BIT   // 65535 counts per period
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_CHANNEL     LEDC_CHANNEL_0

// Pulse width bounds (µs) — standard servo / ESC range
#define PWM_MIN_US      500
#define PWM_NEUTRAL_US  1500
#define PWM_MAX_US      2500

// ── Internal helpers ──────────────────────────────────────────────────────────

static const char *TAG = "pwm_motor";

// Period in µs for the configured frequency
#define PERIOD_US       (1000000u / PWM_FREQ_HZ)   // 20 000 µs at 50 Hz

// Maximum duty count for the selected resolution
#define DUTY_MAX        ((1u << 16) - 1)            // 65535

static uint32_t us_to_duty(uint32_t pulse_us)
{
    // duty = (pulse_us / period_us) * DUTY_MAX
    // Use 64-bit intermediate to avoid overflow
    return (uint32_t)(((uint64_t)pulse_us * DUTY_MAX) / PERIOD_US);
}

static uint32_t clamp_us(uint32_t pulse_us)
{
    if (pulse_us < PWM_MIN_US) return PWM_MIN_US;
    if (pulse_us > PWM_MAX_US) return PWM_MAX_US;
    return pulse_us;
}

// ── Public API ────────────────────────────────────────────────────────────────

void pwm_motor_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,   // C6: only low-speed exists
        .duty_resolution = PWM_RESOLUTION,
        .timer_num       = PWM_TIMER,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .gpio_num   = PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = PWM_CHANNEL,
        .timer_sel  = PWM_TIMER,
        .duty       = us_to_duty(PWM_NEUTRAL_US),
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));

    ESP_LOGI(TAG, "PWM ready: gpio=%d freq=%d Hz res=16-bit neutral=%d µs",
             PWM_GPIO, PWM_FREQ_HZ, PWM_NEUTRAL_US);
}

void pwm_motor_set_us(uint32_t pulse_us)
{
    pulse_us = clamp_us(pulse_us);
    uint32_t duty = us_to_duty(pulse_us);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);

    ESP_LOGD(TAG, "pulse=%lu µs  duty=%lu", (unsigned long)pulse_us, (unsigned long)duty);
}

void pwm_motor_set_normalized(float value)
{
    if (value < 0.0f) value = 0.0f;
    if (value > 1.0f) value = 1.0f;

    uint32_t pulse_us = PWM_MIN_US + (uint32_t)(value * (PWM_MAX_US - PWM_MIN_US));
    pwm_motor_set_us(pulse_us);
}

void pwm_motor_neutral(void)
{
    pwm_motor_set_us(PWM_NEUTRAL_US);
}
