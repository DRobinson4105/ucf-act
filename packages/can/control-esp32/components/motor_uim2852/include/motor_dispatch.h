/**
 * @file motor_dispatch.h
 * @brief Blocking single-command execution engine for UIM2852 motor CAN traffic.
 *
 * Provides synchronous command-response dispatch with ACK matching,
 * error correlation, and observer callbacks for side-traffic. The TX
 * and RX transports are injected at init time so the dispatch layer
 * works with any CAN driver (twai_port, can_twai, etc.).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "motor_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_DISPATCH_RESULT_OK = 0,
    MOTOR_DISPATCH_RESULT_TIMEOUT,
    MOTOR_DISPATCH_RESULT_REMOTE_ERROR,
    MOTOR_DISPATCH_RESULT_TRANSPORT_ERROR,
    MOTOR_DISPATCH_RESULT_INVALID_ARGUMENT,
    MOTOR_DISPATCH_RESULT_INTERNAL_ERROR,
    MOTOR_DISPATCH_RESULT_BUSY,
} motor_dispatch_result_t;

typedef void (*motor_dispatch_observer_fn)(const motor_rx_t *rx, void *ctx);

typedef struct {
    motor_dispatch_observer_fn on_notification;
    motor_dispatch_observer_fn on_error;
    void *ctx;
} motor_dispatch_observer_t;

typedef struct {
    bool active;
    motor_cmd_t cmd;
    bool expects_ack;
    TickType_t started_at;
} motor_dispatch_pending_t;

/**
 * @brief TX function signature for sending a CAN frame.
 *
 * Callers pass can_twai_send (or equivalent) at init time.
 */
typedef esp_err_t (*motor_dispatch_tx_fn)(const twai_message_t *msg, TickType_t timeout);

/**
 * @brief Initialize the motor dispatch layer.
 *
 * @param tx_fn           Function to transmit a CAN frame
 * @param rx_queue_depth  Number of twai_message_t slots in the RX queue (typically 16).
 *                        The RX queue is fed externally via motor_dispatch_enqueue_rx().
 * @return ESP_OK on success, ESP_ERR_NO_MEM if queue creation fails
 */
esp_err_t motor_dispatch_init(motor_dispatch_tx_fn tx_fn, uint8_t rx_queue_depth);

/**
 * @brief Tear down the motor dispatch layer.
 */
void motor_dispatch_deinit(void);

/**
 * @brief Enqueue a received CAN frame for dispatch processing.
 *
 * Called by the CAN RX task for motor frames. Non-blocking; drops the
 * frame silently if the queue is full rather than stalling the RX task.
 *
 * @param msg  CAN frame to enqueue (copied into queue)
 * @return true if enqueued, false if queue full or not initialized
 */
bool motor_dispatch_enqueue_rx(const twai_message_t *msg);

/**
 * @brief Execute a motor command with optional ACK wait.
 *
 * Sends the command via the TX function provided at init. Then waits
 * for a matching response on the RX queue.
 *
 * ACK path: wait up to timeout_ticks for motor_rx_matches_cmd() match.
 * No-ACK path: observe bus for no_ack_grace_ticks; succeed if no
 * related thrown error appears.
 *
 * Side-traffic (notifications, unrelated errors) is forwarded to the
 * observer without affecting completion.
 */
motor_dispatch_result_t motor_dispatch_exec(const motor_cmd_t *cmd,
                                            TickType_t timeout_ticks,
                                            TickType_t no_ack_grace_ticks,
                                            const motor_dispatch_observer_t *observer,
                                            motor_rx_t *out_response,
                                            motor_rx_t *out_related_error);

bool motor_dispatch_has_pending(void);
esp_err_t motor_dispatch_get_pending(motor_dispatch_pending_t *out_pending);

/** @brief Reset internal state (test support only). */
void motor_dispatch_test_reset_state(void);

#ifdef __cplusplus
}
#endif
