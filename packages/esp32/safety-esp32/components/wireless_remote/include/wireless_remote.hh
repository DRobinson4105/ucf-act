#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t gpio;
    int active_level; // 0 or 1
    bool enable_pullup;
    bool enable_pulldown;
} wireless_remote_config_t;

// initialize the input GPIO for the wireless remote
esp_err_t wireless_remote_init(const wireless_remote_config_t *config);

// returns true when the remote is active
bool wireless_remote_is_active(const wireless_remote_config_t *config);

#ifdef __cplusplus
}
#endif
