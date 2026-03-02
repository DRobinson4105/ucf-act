/**
 * @file can_twai.h
 * @brief CAN bus TWAI driver wrapper for ESP-IDF.
 */
#pragma once

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "driver/twai.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
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

/**
 * @brief Initialize TWAI peripheral with 1 Mbps timing and accept-all filter.
 *
 * Configures the ESP-IDF TWAI driver in normal mode at 1 Mbps with no
 * message filtering. Must be called before any transmit or receive operation.
 *
 * @param tx_gpio  GPIO pin for CAN TX
 * @param rx_gpio  GPIO pin for CAN RX
 * @return ESP_OK on success, or an error code if driver installation fails
 */
esp_err_t can_twai_init_default(gpio_num_t tx_gpio, gpio_num_t rx_gpio);

// ============================================================================
// Transmit Functions
// ============================================================================

/**
 * @brief Transmit a standard 11-bit CAN frame with 8-byte payload.
 *
 * Queues a standard CAN frame for transmission. Blocks up to @p timeout
 * ticks if the TX queue is full.
 *
 * @param identifier  11-bit CAN identifier
 * @param data        Pointer to 8-byte payload
 * @param timeout     Maximum ticks to wait for space in the TX queue
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if TX queue full
 */
esp_err_t can_twai_send(uint32_t identifier, const uint8_t data[8], TickType_t timeout);

/**
 * @brief Transmit an extended 29-bit CAN frame with variable payload length.
 *
 * Queues an extended CAN frame for transmission. Used for protocols that
 * require 29-bit identifiers, such as the UIM2852CA motor protocol (SimpleCAN).
 *
 * @param identifier  29-bit extended CAN identifier
 * @param data        Pointer to payload data
 * @param dlc         Data length code (0–8)
 * @param timeout     Maximum ticks to wait for space in the TX queue
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t can_twai_send_extended(uint32_t identifier, const uint8_t *data, uint8_t dlc, TickType_t timeout);

// ============================================================================
// Receive Functions
// ============================================================================

/**
 * @brief Receive the next CAN frame with timeout.
 *
 * Blocks until a frame is available or @p timeout ticks elapse. Check
 * msg->extd to determine frame type: 0 = standard 11-bit, 1 = extended 29-bit.
 *
 * @param msg      Pointer to message structure to fill with received frame
 * @param timeout  Maximum ticks to wait for an incoming frame
 * @return ESP_OK if a frame was received, ESP_ERR_TIMEOUT if no frame available
 */
esp_err_t can_twai_receive(twai_message_t *msg, TickType_t timeout);

// ============================================================================
// Bus Health
// ============================================================================

/**
 * @brief Check TWAI bus health status.
 *
 * Queries the TWAI driver for error states. When bus-off is detected,
 * the caller should invoke can_twai_recover() to attempt recovery.
 *
 * @return true if the bus is healthy (running), false if in an error state
 */
bool can_twai_bus_ok(void);

/**
 * @brief Attempt CAN bus recovery with 3-tier escalation.
 *
 * Tries progressively more aggressive recovery strategies:
 *   1. Bus-off recovery (CAN protocol: twai_initiate_recovery)
 *   2. Stop/start (soft driver reset)
 *   3. Full driver uninstall/reinstall
 *
 * Returns ESP_OK as soon as any tier succeeds.
 *
 * @param tx_gpio   GPIO pin for CAN TX (needed for tier-3 reinstall)
 * @param rx_gpio   GPIO pin for CAN RX (needed for tier-3 reinstall)
 * @param log_tag   Caller identifier for recovery log messages (e.g., "SAFETY")
 * @return ESP_OK on successful recovery, or an error code if all tiers fail
 */
esp_err_t can_twai_recover(gpio_num_t tx_gpio, gpio_num_t rx_gpio, const char *log_tag);

#ifdef __cplusplus
}
#endif
