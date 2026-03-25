/**
 * @file digipot_mcp41hvx1.h
 * @brief MCP41HVX1 high-voltage digital potentiometer for throttle level selection.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// MCP41HVX1 Digital Potentiometer Driver
// ============================================================================
// Controls throttle output level using an MCP41HVX1 SPI-controlled high-voltage
// digital potentiometer (256 positions). Throttle source selection and pedal
// microswitch bypass are handled separately by relay_dpdt_my5nj.
//
// Hardware:
//   - P0A connects to Curtis Pin 2 (8.5V reference)
//   - P0B connects to Curtis B- (ground reference)
//   - P0W (wiper) output connects to DPDT relay
//   - V+ powered from 24V rail, V- and DGND to GND bus, VL from ESP32 3.3V
//   - 256 wiper positions (0 = minimum throttle, 255 = maximum throttle)
//   - SPI interface (SDI, SCK, CS)
//
// Safe state: wiper position 0 (minimum throttle)
// ============================================================================

// ============================================================================
// Configuration
// ============================================================================

// digipot_mcp41hvx1_config_t - SPI pin assignments
//   spi_host: SPI host device (SPI2_HOST on ESP32-C6)
//   sdi:      Serial Data In (host → device)
//   sck:      Serial Clock line
//   cs:       Chip Select line (active low)
typedef struct
{
	spi_host_device_t spi_host;
	gpio_num_t sdi;
	gpio_num_t sck;
	gpio_num_t cs;
} digipot_mcp41hvx1_config_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize SPI bus and set the digipot to its safe state.
 *
 * Configures the SPI bus with the given pin assignments, adds the MCP41HVX1
 * as an SPI device (1 MHz clock, mode 0), and drives the wiper to position 0
 * (minimum throttle) so the output is at its safe state on startup.
 *
 * @param config  SPI pin assignments for SDI, SCK, and CS
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t digipot_mcp41hvx1_init(const digipot_mcp41hvx1_config_t *config);

// ============================================================================
// Wiper Control
// ============================================================================

/**
 * @brief Set the wiper position.
 *
 * Writes the 8-bit wiper position to the MCP41HVX1 volatile wiper register
 * via SPI. The wiper position maps linearly to throttle output voltage.
 *
 * @param position  Wiper position 0-255 (0 = minimum throttle, 255 = maximum)
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t digipot_mcp41hvx1_set_wiper(uint8_t position);

/**
 * @brief Disable the digipot output by setting the wiper to position 0.
 *
 * Drives the wiper to the minimum position (0) to set the output to
 * minimum throttle without affecting the throttle source relay state.
 *
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t digipot_mcp41hvx1_disable(void);

/**
 * @brief Get the current wiper position.
 *
 * Returns the most recently set wiper position, or 0 if the digipot
 * output is currently disabled/idle.
 *
 * @return Wiper position 0-255
 */
uint8_t digipot_mcp41hvx1_get_wiper(void);

// ============================================================================
// Autonomous Mode Control
// ============================================================================

/**
 * @brief Enable autonomous mode.
 *
 * Sets the wiper to position 0 and marks autonomous mode as active.
 * The caller must energize the DPDT relay (relay_dpdt_my5nj) separately
 * to route the digipot output to the motor controller.
 *
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t digipot_mcp41hvx1_enable_autonomous(void);

/**
 * @brief Emergency stop: immediately zero the wiper and clear autonomous flag.
 *
 * Drives the wiper to position 0 to cut throttle output and resets the
 * autonomous mode flag. The caller must de-energize the DPDT relay
 * separately to complete the emergency stop sequence.
 *
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t digipot_mcp41hvx1_emergency_stop(void);

/**
 * @brief Check if the digipot is currently in autonomous mode.
 *
 * Returns true after a successful call to
 * digipot_mcp41hvx1_enable_autonomous() and until autonomous mode
 * is cleared by digipot_mcp41hvx1_emergency_stop().
 *
 * @return true if autonomous mode is active, false otherwise
 */
bool digipot_mcp41hvx1_is_autonomous(void);

#ifdef __cplusplus
}
#endif
