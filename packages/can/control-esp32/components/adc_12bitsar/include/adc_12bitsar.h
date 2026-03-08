/**
 * @file adc_12bitsar.h
 * @brief Calibrated ESP32-C6 SAR ADC reader.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	adc_unit_t adc_unit;
	adc_channel_t adc_channel;
} adc_12bitsar_config_t;

/**
 * @brief Initialize the ESP32-C6 SAR ADC with calibration.
 *
 * Configures the ADC unit and channel specified in @p config,
 * performs calibration if eFuse data is available, and prepares
 * the driver for one-shot voltage readings.
 *
 * @param config  ADC unit and channel selection
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t adc_12bitsar_init(const adc_12bitsar_config_t *config);

/**
 * @brief Deinitialize the ADC and free resources.
 *
 * Releases the ADC handle and any calibration data allocated
 * during init. The driver must be re-initialized before further use.
 */
void adc_12bitsar_deinit(void);

/**
 * @brief Read the ADC value in millivolts with error checking.
 *
 * Performs a one-shot ADC conversion and applies calibration to
 * produce a millivolt result. Returns an error code if the read
 * or calibration conversion fails.
 *
 * @param out_mv  Pointer to store the reading in millivolts
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t adc_12bitsar_read_mv_checked(uint16_t *out_mv);

/**
 * @brief Read the ADC value in millivolts (convenience wrapper).
 *
 * Calls adc_12bitsar_read_mv_checked() internally and returns the
 * millivolt value directly. Returns 0 if an error occurs during
 * the read.
 *
 * @return ADC reading in millivolts, or 0 on error
 */
uint16_t adc_12bitsar_read_mv(void);

/**
 * @brief Check if ADC calibration was successful.
 *
 * Returns whether the calibration step in adc_12bitsar_init()
 * completed successfully. Uncalibrated readings may have lower
 * accuracy.
 *
 * @return true if calibration is active, false otherwise
 */
bool adc_12bitsar_is_calibrated(void);

#ifdef __cplusplus
}
#endif
