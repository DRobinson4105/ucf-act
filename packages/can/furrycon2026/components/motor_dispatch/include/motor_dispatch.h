#pragma once

#include <stdbool.h>

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

/*
 * Blocking single-command execution engine.
 *
 * Matching-response path:
 * - send the command
 * - wait up to timeout_ticks for a response frame that matches
 *   motor_rx_matches_cmd(...)
 * - fail early if a related thrown error arrives first
 *
 * A matching response frame is terminal success even when it is not a simple
 * same-base ACK. That includes normal ACK-kind frames, asymmetric response
 * families such as DV or LM-set replies, and ER command responses that are
 * structurally represented as MOTOR_RX_KIND_ERROR frames by motor_rx.
 *
 * No-ACK path:
 * - send the command
 * - observe the bus for no_ack_grace_ticks
 * - succeed only if no related thrown error is observed during that window
 *
 * For commands sent without requesting ACK, success means the command was
 * transmitted successfully and no related thrown error was observed during the
 * grace window. This is not a positive acknowledgment from the motor.
 *
 * Side-traffic notifications and unrelated error frames can be surfaced
 * upward through the optional observer without affecting completion rules.
 */
motor_dispatch_result_t motor_dispatch_exec(const motor_cmd_t *cmd,
                                            TickType_t timeout_ticks,
                                            TickType_t no_ack_grace_ticks,
                                            const motor_dispatch_observer_t *observer,
                                            motor_rx_t *out_response,
                                            motor_rx_t *out_related_error);

bool motor_dispatch_has_pending(void);
esp_err_t motor_dispatch_get_pending(motor_dispatch_pending_t *out_pending);

#ifdef __cplusplus
}
#endif
