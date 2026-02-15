/**
 * @file adc_12bitsar.hh
 * @brief Calibrated ESP32-C6 SAR ADC reader.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    adc_unit_t adc_unit;
    adc_channel_t adc_channel;
} adc_12bitsar_config_t;

esp_err_t adc_12bitsar_init(const adc_12bitsar_config_t *config);
void adc_12bitsar_deinit(void);
esp_err_t adc_12bitsar_read_mv_checked(uint16_t *out_mv);
uint16_t adc_12bitsar_read_mv(void);
bool adc_12bitsar_is_calibrated(void);

#ifdef __cplusplus
}
#endif
