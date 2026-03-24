/**
 * @file battery_monitor.h
 * @brief Battery pack voltage, current, and SOC monitoring for 48V lead-acid.
 *
 * Reads pack voltage via a resistor divider and pack current via a
 * Hall-effect current sensor, both through ESP32-C6 ADC1
 * channels.  Estimates state of charge using a voltage lookup table
 * with coulomb-counting refinement.
 *
 * Hardware assumptions:
 *   - 6 × 8V lead-acid batteries in series (48V nominal, 24 cells)
 *   - Voltage divider (e.g. 180kΩ / 10kΩ) scales pack voltage to 0-3.3V
 *   - Hall-effect current sensor (e.g. LEM HTFS-200-P) with output divider
 *   - Both sensor outputs connected to ADC1 channels on the ESP32-C6
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "can_protocol.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	int voltage_gpio;           // GPIO for pack voltage divider (must be on ADC1)
	int current_gpio;           // GPIO for current sensor output (must be on ADC1)
	uint16_t divider_ratio;     // Voltage divider multiplier (e.g. 19 for 180k/10k)
	uint16_t current_zero_mv;   // Sensor output at 0A in mV (e.g. 2500 for 5V supply)
	float current_sens_uv;      // Sensor sensitivity in µV/mA (e.g. 6.25 for 6.25mV/A)
	float current_output_scale; // Output divider ratio applied to sensor (e.g. 0.6)
	uint32_t capacity_mah;      // Nominal battery capacity in mAh (e.g. 150000)
} battery_monitor_config_t;

/**
 * @brief Initialize the battery monitor ADC channels.
 *
 * Configures ADC1 with two channels (voltage and current), performs
 * calibration if eFuse data is available, and prepares the driver
 * for periodic update calls.
 *
 * @param config  Sensor configuration (GPIOs, divider ratio, etc.)
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t battery_monitor_init(const battery_monitor_config_t *config);

/**
 * @brief Deinitialize the battery monitor and free ADC resources.
 */
void battery_monitor_deinit(void);

/**
 * @brief Update battery readings and SOC estimate.
 *
 * Reads both ADC channels, applies filtering, updates coulomb
 * counting, and recomputes the SOC estimate.  Call at 10-20 Hz
 * from the safety task loop.
 *
 * @param now_ms  Current time in milliseconds (monotonic)
 */
void battery_monitor_update(uint32_t now_ms);

/**
 * @brief Get the latest battery status for CAN transmission.
 *
 * Copies the most recent voltage, current, SOC, and flags into
 * the provided battery_status_t struct.
 *
 * @param out  Destination struct (must not be NULL)
 */
void battery_monitor_get_status(battery_status_t *out);

/**
 * @brief Check if the battery monitor ADC is functioning.
 *
 * Returns false if either ADC channel has produced consecutive
 * read failures.
 *
 * @return true if both channels are reading successfully
 */
bool battery_monitor_is_healthy(void);

#ifdef __cplusplus
}
#endif
