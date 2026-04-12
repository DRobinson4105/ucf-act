/**
 * @file can_twai.h
 * @brief CAN bus TWAI driver wrapper for ESP-IDF.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

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
 * @brief Transmit a CAN frame (standard or extended).
 *
 * Sends a fully constructed twai_message_t. The frame type is determined
 * by the msg->extd field (0 = standard 11-bit, 1 = extended 29-bit).
 *
 * @param msg      Fully constructed CAN frame to send
 * @param timeout  Maximum ticks to wait for space in the TX queue
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t can_twai_send(const twai_message_t *msg, TickType_t timeout);

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

/**
 * @brief Wait for CAN RX task to exit TWAI API calls and all in-flight TX
 *        to complete.
 *
 * Must be called after setting g_twai_ready = false (to prevent the RX task
 * from re-entering twai_receive). Waits for the RX task to return from any
 * in-progress TWAI call, then spin-waits for in-flight TX to drain.
 *
 * @param lock        Spinlock protecting the in-flight counter
 * @param in_flight   Pointer to the in-flight TX counter
 */
void can_twai_quiesce(portMUX_TYPE *lock, volatile uint8_t *in_flight);

// ============================================================================
// TX Failure Tracking
// ============================================================================
// Pure-function tracker for consecutive CAN TX failures. Counts consecutive
// failures and signals when a threshold is reached, prompting the caller to
// invoke can_twai_recover().

typedef struct
{
	uint8_t fail_count; /**< current consecutive failure count */
	uint8_t threshold;  /**< trigger recovery at this count */
	bool tx_ok;         /**< result of last TX */
} can_tx_track_inputs_t;

typedef struct
{
	uint8_t new_fail_count; /**< updated count */
	bool trigger_recovery;  /**< true if threshold reached */
} can_tx_track_result_t;

/**
 * @brief Track CAN TX success/failure and determine if recovery is needed.
 *
 * Pure function — no side effects, no global state.
 *
 * @param inputs  Current TX result and failure tracking state
 * @return Updated count and whether recovery should be triggered
 */
can_tx_track_result_t can_tx_track(const can_tx_track_inputs_t *inputs);

#ifdef __cplusplus
}
#endif
