/**
 * @file motor_dispatch.cpp
 * @brief Blocking single-command execution engine for UIM2852 motor CAN traffic.
 */

#include "motor_dispatch.h"
#include "motor_dispatch_priv.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "motor_rx.h"

static const char *TAG = "motor_dispatch";

static portMUX_TYPE s_dispatch_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_slot_reserved = false;
static motor_dispatch_pending_t s_pending;

// Injected TX function and RX queue — set by motor_dispatch_init().
static motor_dispatch_tx_fn s_tx_fn = nullptr;
static QueueHandle_t s_rx_queue = nullptr;

static void clear_rx_output(motor_rx_t *out)
{
    if (out != nullptr) {
        memset(out, 0, sizeof(*out));
    }
}

static bool reserve_slot(void)
{
    bool reserved = false;

    taskENTER_CRITICAL(&s_dispatch_lock);
    if (!s_slot_reserved) {
        s_slot_reserved = true;
        memset(&s_pending, 0, sizeof(s_pending));
        reserved = true;
    }
    taskEXIT_CRITICAL(&s_dispatch_lock);

    return reserved;
}

static void activate_pending(const motor_cmd_t *cmd, bool expects_ack, TickType_t started_at)
{
    taskENTER_CRITICAL(&s_dispatch_lock);
    s_pending.active = true;
    s_pending.cmd = *cmd;
    s_pending.expects_ack = expects_ack;
    s_pending.started_at = started_at;
    taskEXIT_CRITICAL(&s_dispatch_lock);
}

static void clear_slot(void)
{
    taskENTER_CRITICAL(&s_dispatch_lock);
    s_slot_reserved = false;
    memset(&s_pending, 0, sizeof(s_pending));
    taskEXIT_CRITICAL(&s_dispatch_lock);
}

static void notify_observer(const motor_dispatch_observer_t *observer,
                            const motor_rx_t *rx)
{
    if (observer == nullptr || rx == nullptr) {
        return;
    }

    if (motor_rx_is_notification(rx)) {
        if (observer->on_notification != nullptr) {
            observer->on_notification(rx, observer->ctx);
        }
        return;
    }

    if (motor_rx_is_error(rx) && observer->on_error != nullptr) {
        observer->on_error(rx, observer->ctx);
    }
}

static bool related_error_matches_cmd(const motor_rx_t *rx, const motor_cmd_t *cmd)
{
    uint8_t related_base = 0U;
    uint8_t related_subindex = 0U;
    uint16_t pt_row_index = 0U;

    if (rx == nullptr || cmd == nullptr || !motor_rx_er_is_thrown_error(rx)) {
        return false;
    }

    // ER command responses are parsed as ERROR-kind frames. If the frame is the
    // expected command response, it is not a thrown remote error.
    if (motor_rx_matches_cmd(rx, cmd)) {
        return false;
    }

    if (rx->producer_id != cmd->target_id) {
        return false;
    }

    if (!motor_rx_error_related_base_code(rx, &related_base) ||
        related_base != cmd->base_code) {
        return false;
    }

    if (!cmd->has_index) {
        return true;
    }

    if (cmd->object == MOTOR_OBJECT_PT) {
        // PT carries a 16-bit queue row selector in payload bytes d0..d1.
        // ER side-traffic exposes only a single-byte related subindex. When the
        // queued PT row fits in 8 bits we can require an exact match on that
        // low byte; otherwise the error frame does not provide enough metadata
        // to distinguish 16-bit rows, so base-level correlation is the safe
        // fallback.
        pt_row_index = (uint16_t)cmd->index;
        if (pt_row_index <= UINT8_MAX &&
            motor_rx_error_related_subindex(rx, &related_subindex)) {
            return related_subindex == (uint8_t)pt_row_index;
        }

        return true;
    }

    return motor_rx_error_related_subindex(rx, &related_subindex) &&
           related_subindex == (uint8_t)cmd->index;
}

static motor_dispatch_result_t wait_for_terminal_outcome(const motor_cmd_t *cmd,
                                                         bool expects_ack,
                                                         TickType_t wait_ticks,
                                                         const motor_dispatch_observer_t *observer,
                                                         motor_rx_t *out_response,
                                                         motor_rx_t *out_related_error)
{
    TimeOut_t timeout_state;
    TickType_t remaining_ticks = wait_ticks;

    vTaskSetTimeOutState(&timeout_state);

    for (;;) {
        twai_message_t msg;
        motor_rx_t rx;

        // Receive from the externally-fed RX queue (not directly from TWAI).
        // This allows the CAN RX task to demux heartbeat vs motor frames.
        if (xQueueReceive(s_rx_queue, &msg, remaining_ticks) != pdTRUE) {
            // No-ACK success is grace-window silence with no related thrown
            // error after a successful transmit.
            return expects_ack ? MOTOR_DISPATCH_RESULT_TIMEOUT : MOTOR_DISPATCH_RESULT_OK;
        }

        esp_err_t err = motor_rx_parse(&msg, &rx);
        if (err != ESP_OK) {
            ESP_LOGW(TAG,
                     "dropping unparsable frame id=0x%08" PRIX32 " dlc=%u rc=%s",
                     msg.identifier,
                     (unsigned)msg.data_length_code,
                     esp_err_to_name(err));
        } else {
            // Terminal success is any response frame that motor_rx considers a
            // match for the command, not only same-base ACKs.
            if (motor_rx_is_notification(&rx)) {
                notify_observer(observer, &rx);
                continue;
            }

            if (expects_ack && motor_rx_matches_cmd(&rx, cmd)) {
                if (out_response != nullptr) {
                    *out_response = rx;
                }
                return MOTOR_DISPATCH_RESULT_OK;
            }

            if (related_error_matches_cmd(&rx, cmd)) {
                if (out_related_error != nullptr) {
                    *out_related_error = rx;
                }
                return MOTOR_DISPATCH_RESULT_REMOTE_ERROR;
            }

            if (motor_rx_is_error(&rx)) {
                notify_observer(observer, &rx);
            }
        }

        if (xTaskCheckForTimeOut(&timeout_state, &remaining_ticks) != pdFALSE) {
            return expects_ack ? MOTOR_DISPATCH_RESULT_TIMEOUT : MOTOR_DISPATCH_RESULT_OK;
        }
    }
}

motor_dispatch_result_t motor_dispatch_exec(const motor_cmd_t *cmd,
                                            TickType_t timeout_ticks,
                                            TickType_t no_ack_grace_ticks,
                                            const motor_dispatch_observer_t *observer,
                                            motor_rx_t *out_response,
                                            motor_rx_t *out_related_error)
{
    clear_rx_output(out_response);
    clear_rx_output(out_related_error);

    if (cmd == nullptr || s_tx_fn == nullptr) {
        return MOTOR_DISPATCH_RESULT_INVALID_ARGUMENT;
    }

    bool expects_ack = cmd->ack_requested;

    if (!reserve_slot()) {
        return MOTOR_DISPATCH_RESULT_BUSY;
    }

    // TX via injected function (twai_port_send in motor-uim2852, can_twai wrapper in control-esp32)
    esp_err_t err = s_tx_fn(&cmd->msg, timeout_ticks);
    if (err != ESP_OK) {
        clear_slot();
        return (err == ESP_ERR_INVALID_ARG) ?
            MOTOR_DISPATCH_RESULT_INVALID_ARGUMENT :
            MOTOR_DISPATCH_RESULT_TRANSPORT_ERROR;
    }

    TickType_t started_at = xTaskGetTickCount();
    activate_pending(cmd, expects_ack, started_at);

    TickType_t wait_ticks = expects_ack ? timeout_ticks : no_ack_grace_ticks;
    motor_dispatch_result_t result = wait_for_terminal_outcome(cmd, expects_ack, wait_ticks, observer, out_response, out_related_error);

    clear_slot();
    return result;
}

// ============================================================================
// Init / Deinit / Enqueue
// ============================================================================

esp_err_t motor_dispatch_init(motor_dispatch_tx_fn tx_fn, uint8_t rx_queue_depth)
{
    if (tx_fn == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_rx_queue != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    s_rx_queue = xQueueCreate(rx_queue_depth, sizeof(twai_message_t));
    if (s_rx_queue == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    s_tx_fn = tx_fn;
    clear_slot();
    return ESP_OK;
}

void motor_dispatch_deinit(void)
{
    if (s_rx_queue != nullptr) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = nullptr;
    }
    s_tx_fn = nullptr;
    clear_slot();
}

bool motor_dispatch_enqueue_rx(const twai_message_t *msg)
{
    if (s_rx_queue == nullptr || msg == nullptr) {
        return false;
    }

    // Non-blocking: drop frame if queue full rather than stalling the
    // priority-7 CAN RX task.
    return xQueueSendToBack(s_rx_queue, msg, 0) == pdTRUE;
}

// ============================================================================
// Query / Test Support
// ============================================================================

bool motor_dispatch_has_pending(void)
{
    bool active;

    taskENTER_CRITICAL(&s_dispatch_lock);
    active = s_pending.active;
    taskEXIT_CRITICAL(&s_dispatch_lock);

    return active;
}

esp_err_t motor_dispatch_get_pending(motor_dispatch_pending_t *out_pending)
{
    if (out_pending == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    taskENTER_CRITICAL(&s_dispatch_lock);
    if (!s_pending.active) {
        taskEXIT_CRITICAL(&s_dispatch_lock);
        return ESP_ERR_NOT_FOUND;
    }

    *out_pending = s_pending;
    taskEXIT_CRITICAL(&s_dispatch_lock);
    return ESP_OK;
}

void motor_dispatch_test_reset_state(void)
{
    clear_slot();
}
