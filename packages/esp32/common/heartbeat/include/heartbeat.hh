#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t gpio;
    TickType_t interval_ticks;
    TickType_t activity_window_ticks;
    bool active_high;
    const char *label;
    bool use_ws2812;
    gpio_num_t ws2812_gpio;
    uint8_t idle_red;
    uint8_t idle_green;
    uint8_t idle_blue;
    uint8_t activity_red;
    uint8_t activity_green;
    uint8_t activity_blue;
    uint8_t error_red;
    uint8_t error_green;
    uint8_t error_blue;
} heartbeat_config_t;

// initialize heartbeat GPIO and state
esp_err_t heartbeat_init(const heartbeat_config_t *config);

// update heartbeat state based on current tick
void heartbeat_tick(const heartbeat_config_t *config, TickType_t now_ticks);

// mark recent activity (e.g., CAN traffic)
void heartbeat_mark_activity(TickType_t now_ticks);

// set or clear error state
void heartbeat_set_error(bool active);

#ifdef __cplusplus
}
#endif
