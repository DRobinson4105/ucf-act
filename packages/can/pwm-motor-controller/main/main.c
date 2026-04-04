/**
 * @file main.c
 * @brief PWM motor controller — 50 Hz LEDC output on ESP32-C6.
 *
 * Demonstrates the pwm_motor API with a continuous triangle sweep from minimum
 * (500 µs) to maximum (2500 µs) and back.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "pwm_motor.h"

static const char *TAG = "main";

// Sweep parameters
#define STEP_US         10      // pulse width increment per tick
#define TICK_MS         20      // delay between steps (50 ticks/s)

void app_main(void)
{
    pwm_motor_init();
    pwm_motor_neutral();

    ESP_LOGI(TAG, "Starting sweep demo");

    uint32_t pulse_us = 1500;   // start at neutral
    int32_t  step     = STEP_US;

    while (true)
    {
        pwm_motor_set_us(pulse_us);
        ESP_LOGI(TAG, "pulse=%lu µs", (unsigned long)pulse_us);

        // Reverse direction at bounds, hold at neutral briefly on each pass
        int32_t next = (int32_t)pulse_us + step;
        if (next >= 2500)
        {
            pulse_us = 2500;
            step     = -STEP_US;
        }
        else if (next <= 500)
        {
            pulse_us = 500;
            step     = STEP_US;
        }
        else
        {
            pulse_us = (uint32_t)next;
        }

        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}
