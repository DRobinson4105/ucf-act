/**
 * @file can_twai.hh
 * @brief CAN bus TWAI driver wrapper for ESP-IDF.
 */
#pragma once

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "driver/twai.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// CAN TWAI Driver Wrapper
// ============================================================================
// Wrapper around ESP-IDF TWAI driver for CAN bus communication.
// Supports both standard 11-bit and extended 29-bit CAN frames.
//
// Bus configuration:
//   - Bit rate: 1 Mbps
//   - Filter: Accept all messages
//   - Mode: Normal (TX and RX enabled)
// ============================================================================

// ============================================================================
// Initialization
// ============================================================================

// Initialize TWAI peripheral with 1 Mbps timing and accept-all filter
esp_err_t can_twai_init_default(gpio_num_t tx_gpio, gpio_num_t rx_gpio);

// ============================================================================
// Transmit Functions
// ============================================================================

// Transmit standard 11-bit CAN frame with 8-byte payload
// Returns ESP_OK on success, ESP_ERR_TIMEOUT if TX queue full
esp_err_t can_twai_send(uint32_t identifier, const uint8_t data[8], TickType_t timeout);

// Transmit extended 29-bit CAN frame with variable payload length
// Used for UIM2852CA motor protocol (SimpleCAN)
esp_err_t can_twai_send_extended(uint32_t identifier, const uint8_t *data, uint8_t dlc, TickType_t timeout);

// ============================================================================
// Receive Functions
// ============================================================================

// Receive next CAN frame with timeout
// Returns ESP_OK if frame received, ESP_ERR_TIMEOUT if no frame available
// Check msg->extd to determine frame type: 0=standard 11-bit, 1=extended 29-bit
esp_err_t can_twai_receive(twai_message_t *msg, TickType_t timeout);

// ============================================================================
// Bus Health
// ============================================================================

// Check TWAI bus status for error states. Returns true if bus is healthy.
// When bus-off is detected, caller should invoke can_twai_recover_bus_off().
bool can_twai_bus_ok(void);

// Attempt recovery from bus-off state.
// Calls twai_initiate_recovery() and twai_start() with a brief delay.
// Returns ESP_OK on successful recovery.
esp_err_t can_twai_recover_bus_off(void);

#ifdef __cplusplus
}
#endif
