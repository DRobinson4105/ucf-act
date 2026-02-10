/**
 * @file ultrasonic_a02yyuw.hh
 * @brief A02YYUW ultrasonic distance sensor driver interface.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
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
typedef struct {
    uart_port_t uart_num;
    int tx_gpio;
    int rx_gpio;
    int baud_rate;
} ultrasonic_a02yyuw_config_t;

// ============================================================================
// Initialization
// ============================================================================

// Initialize UART for ultrasonic sensor communication
esp_err_t ultrasonic_a02yyuw_init(const ultrasonic_a02yyuw_config_t *config);

// ============================================================================
// Distance Reading
// ============================================================================

// Get latest distance measurement in millimeters
// Returns true if a fresh sample is available, false if no new data
// out_distance_mm: pointer to store distance (only written if returning true)
bool ultrasonic_a02yyuw_get_distance_mm(uint16_t *out_distance_mm);

// Check if detected object is closer than threshold
// Returns true if distance <= threshold_mm (obstacle detected)
// out_distance_mm: optional pointer to store actual distance (can be NULL)
bool ultrasonic_a02yyuw_is_too_close(uint16_t threshold_mm, uint16_t *out_distance_mm);

// ============================================================================
// Health Check
// ============================================================================

// Check if sensor is healthy (has received valid data recently)
// Returns true if sensor has reported data within the last 500ms
// Use this to detect sensor failure/disconnection for safety systems
bool ultrasonic_a02yyuw_is_healthy(void);

#ifdef __cplusplus
}
#endif
