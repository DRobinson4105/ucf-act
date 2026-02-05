#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Ultrasonic Distance Sensor Driver
// =============================================================================
// Reads distance measurements from UART-based ultrasonic sensor.
//
// Hardware:
//   - UART ultrasonic rangefinder (e.g., JSN-SR04T with UART mode)
//   - Continuously outputs distance readings at configured baud rate
//   - Typical range: 20mm to 4500mm
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
// =============================================================================

// =============================================================================
// Configuration
// =============================================================================

// ultrasonic_config_t - UART interface configuration
//   uart_num:  UART peripheral number (UART_NUM_0, UART_NUM_1, etc.)
//   tx_gpio:   GPIO for UART TX (may be unused by sensor)
//   rx_gpio:   GPIO for UART RX (receives distance data)
//   baud_rate: UART baud rate (typically 9600 or 115200)
typedef struct {
    uart_port_t uart_num;
    int tx_gpio;
    int rx_gpio;
    int baud_rate;
} ultrasonic_config_t;

// =============================================================================
// Initialization
// =============================================================================

// Initialize UART for ultrasonic sensor communication
esp_err_t ultrasonic_init(const ultrasonic_config_t *config);

// =============================================================================
// Distance Reading
// =============================================================================

// Get latest distance measurement in millimeters
// Returns true if a fresh sample is available, false if no new data
// out_distance_mm: pointer to store distance (only written if returning true)
bool ultrasonic_get_distance_mm(uint16_t *out_distance_mm);

// Check if detected object is closer than threshold
// Returns true if distance <= threshold_mm (obstacle detected)
// out_distance_mm: optional pointer to store actual distance (can be NULL)
bool ultrasonic_is_too_close(uint16_t threshold_mm, uint16_t *out_distance_mm);

#ifdef __cplusplus
}
#endif
