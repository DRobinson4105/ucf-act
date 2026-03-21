/**
 * @file ultrasonic_a02yyuw.h
 * @brief A02YYUW ultrasonic distance sensor driver interface.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// Ultrasonic Distance Sensor Driver (A02YYUW)
// ============================================================================
// Reads distance measurements from the A02YYUW waterproof ultrasonic sensor.
//
// Hardware:
//   - A02YYUW: Waterproof UART ultrasonic rangefinder
//   - Operating voltage: 3.3V-5V DC
//   - Measuring range: 30mm - 4500mm
//   - Resolution: 1mm
//   - UART output: 9600 baud, 8N1
//   - Output format: 4-byte frames [0xFF][HIGH][LOW][CHECKSUM]
//
// Usage:
//   - Sensor outputs readings automatically (no trigger required)
//   - Driver buffers latest reading from UART RX
//   - Application polls for distance or checks threshold
//
// Safety:
//   - Used for obstacle detection during autonomous operation
//   - If object detected closer than threshold, e-stop triggered
//   - Timeout/no-data condition should be treated as fault
// ============================================================================

// ============================================================================
// Configuration
// ============================================================================

// ultrasonic_a02yyuw_config_t - UART interface configuration
//   uart_num:  UART peripheral number (UART_NUM_0, UART_NUM_1, etc.)
//   tx_gpio:   GPIO for UART TX (may be unused by sensor)
//   rx_gpio:   GPIO for UART RX (receives distance data)
//   baud_rate: UART baud rate (typically 9600)
typedef struct
{
	uart_port_t uart_num;
	int tx_gpio;
	int rx_gpio;
	int baud_rate;
} ultrasonic_a02yyuw_config_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize UART for ultrasonic sensor communication.
 *
 * Configures the specified UART peripheral and starts a background task
 * that continuously reads 4-byte distance frames from the A02YYUW sensor.
 *
 * @param config  UART interface configuration (port, GPIOs, baud rate)
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t ultrasonic_a02yyuw_init(const ultrasonic_a02yyuw_config_t *config);

/**
 * @brief Deinitialize UART and background task resources.
 *
 * Stops the UART receive task and releases the UART peripheral so it
 * can be reconfigured or used by another driver.
 */
void ultrasonic_a02yyuw_deinit(void);

// ============================================================================
// Distance Reading
// ============================================================================

/**
 * @brief Get the latest distance measurement in millimeters.
 *
 * Returns the most recent valid distance reading buffered from the
 * sensor's UART output. The output pointer is only written when a
 * fresh sample is available.
 *
 * @param out_distance_mm  Pointer to store distance (only written on success)
 * @return true if a fresh sample is available, false if no new data
 */
bool ultrasonic_a02yyuw_get_distance_mm(uint16_t *out_distance_mm);

/**
 * @brief Check if a detected object is closer than the given threshold.
 *
 * Compares the latest distance reading against @p threshold_mm and
 * returns true when an obstacle is within that range.
 *
 * @param threshold_mm    Minimum safe distance in millimeters
 * @param out_distance_mm Optional pointer to store actual distance (may be NULL)
 * @return true if distance <= threshold_mm (obstacle detected), false otherwise
 */
bool ultrasonic_a02yyuw_is_too_close(uint16_t threshold_mm, uint16_t *out_distance_mm);

// ============================================================================
// Health Check
// ============================================================================

/**
 * @brief Check if the sensor is healthy (has received recent valid frames).
 *
 * Returns true when at least one checksum-valid UART frame has been received
 * within the last 500 ms. Frame health is tracked independently from
 * distance-range validity: out-of-range/no-echo frames can still be healthy.
 * Use this to detect sensor failure or disconnection in safety systems; a
 * false return should be treated as a fault condition.
 *
 * @return true if sensor data is current, false if stale or absent
 */
bool ultrasonic_a02yyuw_is_healthy(void);

#ifdef __cplusplus
}
#endif
