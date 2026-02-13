/**
 * @file optocoupler_pc817.hh
 * @brief F/R switch decode + debounce for dual PC817 inputs.
 */
#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "can_protocol.hh"

#ifdef __cplusplus
extern "C" {
#endif

#define FR_PC817_DEBOUNCE_MS 20

typedef uint8_t fr_state_t;  // FR_STATE_* values

typedef struct {
    gpio_num_t forward_gpio;
    gpio_num_t reverse_gpio;
} optocoupler_pc817_config_t;

esp_err_t optocoupler_pc817_init(const optocoupler_pc817_config_t *config);
fr_state_t optocoupler_pc817_get_state(void);
fr_state_t optocoupler_pc817_get_state_raw(void);
void optocoupler_pc817_update(uint32_t now_ms);

#ifdef __cplusplus
}
#endif
