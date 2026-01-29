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
} nc_mushroom_config_t;

// initialize the input GPIO for the mushroom switch
esp_err_t nc_mushroom_init(const nc_mushroom_config_t *config);

// returns true when the input is in the active (E-stop) state
bool nc_mushroom_read_active(const nc_mushroom_config_t *config);

#ifdef __cplusplus
}
#endif
