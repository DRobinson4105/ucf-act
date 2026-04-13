/**
 * @file relay_dpdt_my5nj.h
 * @brief MY5NJ DPDT relay driver.
 */
#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    gpio_num_t gpio;
} relay_dpdt_my5nj_config_t;

esp_err_t relay_dpdt_my5nj_init(const relay_dpdt_my5nj_config_t *config);
esp_err_t relay_dpdt_my5nj_energize(void);
esp_err_t relay_dpdt_my5nj_deenergize(void);
bool relay_dpdt_my5nj_is_energized(void);

#ifdef __cplusplus
}
#endif
