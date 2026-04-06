/**
 * @file dac_mcp4728.h
 * @brief MCP4728 4-channel 12-bit I2C DAC driver for throttle control.
 *
 * Uses one channel (A) of the MCP4728 with VDD as the voltage reference
 * to produce 0-VDD (~3.3V) output.  An external op-amp (LM358) in
 * non-inverting configuration (gain ~2.47) scales this to 0-8.5V for
 * the Curtis 1204 throttle input.
 *
 * Hardware:
 *   - MCP4728 VDD powered from ESP32 3.3V
 *   - I2C SDA/SCL at 3.3V logic (no level shifting needed)
 *   - Channel A output -> LM358 non-inverting amplifier (gain ~2.47)
 *   - LM358 powered from 24V rail, output 0-8.5V -> Curtis Pin 3
 *   - LDAC pin tied to GND for immediate output update
 *   - Power-on reset outputs 0V (safe default)
 *
 * The driver accepts throttle levels 0-4095 (12-bit) and verifies
 * each write by reading back the channel register.
 */
#pragma once

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// Configuration
// ============================================================================

typedef struct
{
	i2c_port_num_t i2c_port; // I2C port number (I2C_NUM_0)
	gpio_num_t sda;          // I2C data pin
	gpio_num_t scl;          // I2C clock pin
	uint8_t device_addr;     // 7-bit I2C address (default 0x60)
	uint32_t clk_speed_hz;   // I2C clock speed (recommend 100000)
} dac_mcp4728_config_t;

#define DAC_MCP4728_CONFIG_DEFAULT()       \
	{                                      \
		.i2c_port = I2C_NUM_0,             \
		.sda = GPIO_NUM_6,                 \
		.scl = GPIO_NUM_7,                 \
		.device_addr = 0x60,               \
		.clk_speed_hz = 100000,            \
	}

#define DAC_MCP4728_LEVEL_MIN 0
#define DAC_MCP4728_LEVEL_MAX 4095

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize the MCP4728 DAC over I2C.
 *
 * Configures the I2C bus, adds the MCP4728 as a device, and writes
 * channel A to 0 (safe state).  Uses internal 2.048V reference with
 * 2x gain for 0-4.096V output range.
 *
 * @param config  I2C pin assignments and device address
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t dac_mcp4728_init(const dac_mcp4728_config_t *config);

// ============================================================================
// Output Control
// ============================================================================

/**
 * @brief Set the throttle output level.
 *
 * Writes a 12-bit value to channel A and verifies by reading back.
 * Output voltage = (level / 4096) * 4.096V, scaled by the external
 * op-amp to 0-8.5V.
 *
 * @param level  Output level 0-4095 (0 = 0V, 4095 = ~4.096V)
 * @return ESP_OK on success, ESP_ERR_INVALID_RESPONSE on verify mismatch
 */
esp_err_t dac_mcp4728_set_level(uint16_t level);

/**
 * @brief Set output to 0 (safe/idle state).
 *
 * @return ESP_OK on success
 */
esp_err_t dac_mcp4728_disable(void);

/**
 * @brief Enable autonomous mode at level 0.
 *
 * Marks the DAC as active for autonomous throttle control.
 * Output starts at 0 (no throttle).
 *
 * @return ESP_OK on success
 */
esp_err_t dac_mcp4728_enable_autonomous(void);

/**
 * @brief Emergency stop: zero output and clear autonomous flag.
 *
 * @return ESP_OK on success
 */
esp_err_t dac_mcp4728_emergency_stop(void);

/**
 * @brief Get the current output level.
 *
 * @return Current level (0-4095)
 */
uint16_t dac_mcp4728_get_level(void);

/**
 * @brief Check if autonomous mode is active.
 *
 * @return true if autonomous mode is enabled
 */
bool dac_mcp4728_is_autonomous(void);

#ifdef __cplusplus
}
#endif
