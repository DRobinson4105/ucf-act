/*
 * Responsibility:
 * Nonblocking single-request semantic execution wrapper around motor_dispatch.
 */

#include "motor_exec.h"
#include "motor_exec_internal.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/task.h"
#include "motor_diag.h"

#define MOTOR_EXEC_WORKER_STACK_WORDS 4096U
#define MOTOR_EXEC_WORKER_PRIORITY    (tskIDLE_PRIORITY + 1U)

static const char *TAG = "motor_exec";

static motor_dispatch_result_t dispatch_default(const motor_cmd_t *cmd,
                                                TickType_t timeout_ticks,
                                                TickType_t no_ack_grace_ticks,
                                                const motor_dispatch_observer_t *observer,
                                                motor_rx_t *out_response,
                                                motor_rx_t *out_related_error);

typedef struct {
    bool active;
    motor_exec_request_id_t request_id;
    motor_exec_role_t role;
    motor_exec_log_domain_t log_domain;
    motor_exec_operation_t operation;
    motor_cmd_t cmd;
    motor_diag_cmd_desc_t cmd_desc;
    motor_exec_submit_opts_t opts;
} motor_exec_pending_t;

typedef struct {
    motor_exec_log_domain_t log_domain;
} motor_exec_observer_ctx_t;

static TaskHandle_t s_worker_task = NULL;
static bool s_worker_creating = false;
static bool s_pending_active = false;
static motor_exec_request_id_t s_next_request_id = 1U;
static motor_exec_pending_t s_pending;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static motor_exec_log_cfg_t s_log_cfg = {
    .enabled = true,
    .setup_enabled = true,
    .log_notifications = true,
    .log_unrelated_errors = true,
    .motion_enabled = true,
};
static motor_exec_dispatch_fn_t s_dispatch_fn = dispatch_default;
static motor_exec_rx_log_fn_t s_rx_log_fn = motor_diag_log_rx;

static void copy_string(char *dst, size_t cap, const char *src);
static void format_into(char *dst, size_t cap, const char *fmt, ...);
static void format_value_summary(char *dst, size_t cap, const motor_diag_value_desc_t *value);
static void fill_exec_value(motor_exec_value_t *dst, const motor_diag_value_desc_t *src);

static motor_dispatch_result_t dispatch_default(const motor_cmd_t *cmd,
                                                TickType_t timeout_ticks,
                                                TickType_t no_ack_grace_ticks,
                                                const motor_dispatch_observer_t *observer,
                                                motor_rx_t *out_response,
                                                motor_rx_t *out_related_error)
{
    return motor_dispatch_exec(cmd, timeout_ticks, no_ack_grace_ticks, observer, out_response, out_related_error);
}

static void copy_string(char *dst, size_t cap, const char *src)
{
    if (dst == NULL || cap == 0U) {
        return;
    }

    if (src == NULL) {
        dst[0] = '\0';
        return;
    }

    (void)snprintf(dst, cap, "%s", src);
}

static void format_into(char *dst, size_t cap, const char *fmt, ...)
{
    va_list ap;

    if (dst == NULL || cap == 0U) {
        return;
    }

    va_start(ap, fmt);
    (void)vsnprintf(dst, cap, fmt, ap);
    va_end(ap);
}

static void format_value_summary(char *dst, size_t cap, const motor_diag_value_desc_t *value)
{
    if (dst == NULL || cap == 0U) {
        return;
    }

    dst[0] = '\0';

    if (value == NULL) {
        return;
    }

    if (value->has_value) {
        switch (value->kind) {
            case MOTOR_DIAG_VALUE_KIND_ENUM:
            case MOTOR_DIAG_VALUE_KIND_TEXT:
                if (value->text != NULL) {
                    copy_string(dst, cap, value->text);
                } else {
                    format_into(dst, cap, "%" PRId64,
                                value->has_raw_number ? value->raw_number : value->number);
                }
                break;
            case MOTOR_DIAG_VALUE_KIND_U8:
            case MOTOR_DIAG_VALUE_KIND_U16:
            case MOTOR_DIAG_VALUE_KIND_U32:
            case MOTOR_DIAG_VALUE_KIND_I32:
                format_into(dst, cap, "%" PRId64,
                            value->has_raw_number ? value->raw_number : value->number);
                break;
            case MOTOR_DIAG_VALUE_KIND_NONE:
            case MOTOR_DIAG_VALUE_KIND_RAW_BYTES:
            default:
                break;
        }

        if (value->units != NULL && dst[0] != '\0') {
            size_t used = strlen(dst);
            if (used + 1U < cap) {
                (void)snprintf(dst + used, cap - used, " %s", value->units);
            }
        }
        return;
    }

    if (value->kind == MOTOR_DIAG_VALUE_KIND_RAW_BYTES && value->raw_len > 0U) {
        size_t used = 0U;
        uint8_t i;

        for (i = 0U; i < value->raw_len && used + 1U < cap; ++i) {
            int written = snprintf(dst + used,
                                   cap - used,
                                   "%s0x%02X",
                                   i == 0U ? "" : " ",
                                   value->raw[i]);
            if (written < 0) {
                dst[0] = '\0';
                return;
            }
            if ((size_t)written >= cap - used) {
                dst[cap - 1U] = '\0';
                return;
            }
            used += (size_t)written;
        }
    }
}

static void fill_exec_value(motor_exec_value_t *dst, const motor_diag_value_desc_t *src)
{
    if (dst == NULL || src == NULL) {
        return;
    }

    memset(dst, 0, sizeof(*dst));
    dst->has_value = src->has_value;
    dst->number = src->number;
    dst->has_raw_number = src->has_raw_number;
    dst->raw_number = src->raw_number;
    copy_string(dst->text, sizeof(dst->text), src->text);
    copy_string(dst->units, sizeof(dst->units), src->units);

    switch (src->kind) {
        case MOTOR_DIAG_VALUE_KIND_U8:
            dst->kind = MOTOR_EXEC_VALUE_KIND_U8;
            break;
        case MOTOR_DIAG_VALUE_KIND_U16:
            dst->kind = MOTOR_EXEC_VALUE_KIND_U16;
            break;
        case MOTOR_DIAG_VALUE_KIND_U32:
            dst->kind = MOTOR_EXEC_VALUE_KIND_U32;
            break;
        case MOTOR_DIAG_VALUE_KIND_I32:
            dst->kind = MOTOR_EXEC_VALUE_KIND_I32;
            break;
        case MOTOR_DIAG_VALUE_KIND_ENUM:
            dst->kind = MOTOR_EXEC_VALUE_KIND_ENUM;
            break;
        case MOTOR_DIAG_VALUE_KIND_TEXT:
            dst->kind = MOTOR_EXEC_VALUE_KIND_TEXT;
            break;
        case MOTOR_DIAG_VALUE_KIND_NONE:
        case MOTOR_DIAG_VALUE_KIND_RAW_BYTES:
        default:
            dst->kind = MOTOR_EXEC_VALUE_KIND_NONE;
            break;
    }
}

static motor_exec_operation_t classify_operation(const motor_cmd_t *cmd,
                                                 const motor_diag_cmd_desc_t *cmd_desc)
{
    if (cmd == NULL) {
        return MOTOR_EXEC_OPERATION_UNKNOWN;
    }

    if (cmd->object == MOTOR_OBJECT_ER && cmd->msg.data_length_code > 1U) {
        return MOTOR_EXEC_OPERATION_CLEAR;
    }

    if (cmd_desc == NULL) {
        return MOTOR_EXEC_OPERATION_UNKNOWN;
    }

    switch (cmd_desc->kind) {
        case MOTOR_DIAG_CMD_KIND_GET:
            return MOTOR_EXEC_OPERATION_GET;
        case MOTOR_DIAG_CMD_KIND_SET:
            return MOTOR_EXEC_OPERATION_SET;
        case MOTOR_DIAG_CMD_KIND_ACTION:
            return MOTOR_EXEC_OPERATION_ACTION;
        default:
            return MOTOR_EXEC_OPERATION_UNKNOWN;
    }
}

static motor_exec_log_domain_t classify_log_domain(const motor_cmd_t *cmd)
{
    if (cmd == NULL) {
        return MOTOR_EXEC_LOG_DOMAIN_MOTION;
    }

    switch (cmd->object) {
        case MOTOR_OBJECT_PP:
        case MOTOR_OBJECT_IC:
        case MOTOR_OBJECT_IE:
        case MOTOR_OBJECT_ER:
        case MOTOR_OBJECT_QE:
        case MOTOR_OBJECT_SY:
        case MOTOR_OBJECT_MT:
        case MOTOR_OBJECT_IL:
            return MOTOR_EXEC_LOG_DOMAIN_SETUP;
        default:
            return MOTOR_EXEC_LOG_DOMAIN_MOTION;
    }
}

static bool log_allowed_for_domain(motor_exec_log_domain_t domain)
{
    motor_exec_log_cfg_t cfg;

    portENTER_CRITICAL(&s_lock);
    cfg = s_log_cfg;
    portEXIT_CRITICAL(&s_lock);

    if (!cfg.enabled) {
        return false;
    }

    return domain == MOTOR_EXEC_LOG_DOMAIN_SETUP ? cfg.setup_enabled : cfg.motion_enabled;
}

static bool side_traffic_log_allowed(motor_exec_log_domain_t domain, bool is_notification)
{
    motor_exec_log_cfg_t cfg;

    portENTER_CRITICAL(&s_lock);
    cfg = s_log_cfg;
    portEXIT_CRITICAL(&s_lock);

    if (!cfg.enabled) {
        return false;
    }

    if (!(domain == MOTOR_EXEC_LOG_DOMAIN_SETUP ? cfg.setup_enabled : cfg.motion_enabled)) {
        return false;
    }

    return is_notification ? cfg.log_notifications : cfg.log_unrelated_errors;
}

static void log_side_traffic(const motor_rx_t *rx, void *ctx)
{
    const motor_exec_observer_ctx_t *observer_ctx = (const motor_exec_observer_ctx_t *)ctx;
    const bool is_notification = motor_rx_is_notification(rx);

    if (rx == NULL || observer_ctx == NULL) {
        return;
    }

    if (!side_traffic_log_allowed(observer_ctx->log_domain, is_notification)) {
        return;
    }

    if (s_rx_log_fn != NULL) {
        s_rx_log_fn(rx);
    }
}

static esp_err_t validate_opts(const motor_exec_submit_opts_t *opts)
{
    if (opts == NULL || opts->on_complete == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static bool reserve_pending_slot(motor_exec_role_t role,
                                 const motor_cmd_t *cmd,
                                 const motor_exec_submit_opts_t *opts,
                                 motor_exec_log_domain_t log_domain,
                                 motor_exec_operation_t operation,
                                 const motor_diag_cmd_desc_t *cmd_desc,
                                 motor_exec_request_id_t *out_request_id)
{
    bool reserved = false;

    if (cmd == NULL || opts == NULL || cmd_desc == NULL || out_request_id == NULL) {
        return false;
    }

    portENTER_CRITICAL(&s_lock);
    if (!s_pending_active) {
        memset(&s_pending, 0, sizeof(s_pending));
        s_pending.active = true;
        s_pending.request_id = s_next_request_id++;
        s_pending.role = role;
        s_pending.log_domain = log_domain;
        s_pending.operation = operation;
        s_pending.cmd = *cmd;
        s_pending.cmd_desc = *cmd_desc;
        s_pending.opts = *opts;
        s_pending_active = true;
        *out_request_id = s_pending.request_id;
        reserved = true;
    }
    portEXIT_CRITICAL(&s_lock);

    return reserved;
}

static void clear_pending_slot(void)
{
    portENTER_CRITICAL(&s_lock);
    memset(&s_pending, 0, sizeof(s_pending));
    s_pending_active = false;
    portEXIT_CRITICAL(&s_lock);
}

static bool copy_pending_snapshot(motor_exec_pending_t *out_pending)
{
    bool has_pending = false;

    if (out_pending == NULL) {
        return false;
    }

    portENTER_CRITICAL(&s_lock);
    if (s_pending_active) {
        *out_pending = s_pending;
        has_pending = true;
    }
    portEXIT_CRITICAL(&s_lock);

    return has_pending;
}

static void log_submit(motor_exec_request_id_t request_id,
                       motor_exec_role_t role,
                       motor_exec_log_domain_t log_domain,
                       const motor_cmd_t *cmd)
{
    if (cmd == NULL || !log_allowed_for_domain(log_domain)) {
        return;
    }

    ESP_LOGI(TAG, "req=%" PRIu32 " submitted role=%d", request_id, (int)role);
    motor_diag_log_cmd(cmd);
}

static void log_completion(const motor_exec_pending_t *pending,
                           const motor_exec_result_t *result)
{
    if (pending == NULL || result == NULL || !log_allowed_for_domain(pending->log_domain)) {
        return;
    }

    if (result->status == MOTOR_EXEC_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "req=%" PRIu32 " %s", result->request_id, result->completion_summary);
        return;
    }

    ESP_LOGW(TAG, "req=%" PRIu32 " %s", result->request_id, result->completion_summary);
}

static void fill_semantic_command_fields(const motor_diag_cmd_desc_t *cmd_desc,
                                         motor_exec_result_t *out)
{
    if (cmd_desc == NULL || out == NULL) {
        return;
    }

    copy_string(out->command_family_symbol, sizeof(out->command_family_symbol), cmd_desc->family_symbol);
    out->has_parameter_index = cmd_desc->has_index;
    out->parameter_index = cmd_desc->index;
    copy_string(out->parameter_name, sizeof(out->parameter_name), cmd_desc->semantic_name);
    format_value_summary(out->parameter_value_summary,
                         sizeof(out->parameter_value_summary),
                         &cmd_desc->value);
}

static void fill_response_value_summary(const motor_diag_rx_desc_t *response_desc,
                                        motor_exec_result_t *out)
{
    if (response_desc == NULL || out == NULL) {
        return;
    }

    format_value_summary(out->response_value_summary,
                         sizeof(out->response_value_summary),
                         &response_desc->value);
    out->has_response_value_summary = out->response_value_summary[0] != '\0';
    fill_exec_value(&out->response_value, &response_desc->value);
    out->has_response_value = response_desc->value.has_value;
}

static void fill_remote_error(const motor_diag_rx_desc_t *error_desc,
                              motor_exec_result_t *out)
{
    if (error_desc == NULL || out == NULL || !error_desc->has_error_code) {
        return;
    }

    out->remote_error.valid = true;
    out->remote_error.code = error_desc->error_code;
    copy_string(out->remote_error.name, sizeof(out->remote_error.name), error_desc->error_name);
    out->remote_error.has_related_family = error_desc->has_related_family;
    copy_string(out->remote_error.related_family_symbol,
                sizeof(out->remote_error.related_family_symbol),
                error_desc->related_family_symbol);
    out->remote_error.has_related_index = error_desc->has_related_index;
    out->remote_error.related_index = error_desc->related_index;
}

static void build_completion_summary(const motor_exec_pending_t *pending,
                                     const motor_exec_result_t *result,
                                     char *out_summary,
                                     size_t out_cap)
{
    if (pending == NULL || result == NULL || out_summary == NULL || out_cap == 0U) {
        return;
    }

    switch (result->status) {
        case MOTOR_EXEC_STATUS_SUCCESS:
            if (result->completion_summary[0] != '\0') {
                format_into(out_summary, out_cap, "completed successfully: %s", result->completion_summary);
            } else {
                copy_string(out_summary, out_cap,
                            "completed successfully without ACK; no related remote error was observed");
            }
            break;
        case MOTOR_EXEC_STATUS_TIMEOUT:
            format_into(out_summary, out_cap, "timed out waiting for terminal completion of %s",
                        result->command_summary);
            break;
        case MOTOR_EXEC_STATUS_REMOTE_ERROR:
            if (result->remote_error.valid && result->remote_error.name[0] != '\0') {
                format_into(out_summary, out_cap, "remote error: %s", result->remote_error.name);
            } else {
                copy_string(out_summary, out_cap, "remote error");
            }
            break;
        case MOTOR_EXEC_STATUS_TRANSPORT_ERROR:
            format_into(out_summary, out_cap, "transport error while executing %s", result->command_summary);
            break;
        case MOTOR_EXEC_STATUS_INTERNAL_ERROR:
        default:
            format_into(out_summary, out_cap, "internal execution error while executing %s",
                        result->command_summary);
            break;
    }
}

static void populate_result(const motor_exec_pending_t *pending,
                            motor_dispatch_result_t dispatch_result,
                            const motor_rx_t *response,
                            const motor_rx_t *related_error,
                            motor_exec_result_t *out)
{
    if (pending == NULL || out == NULL) {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->request_id = pending->request_id;
    out->node_id = pending->cmd.target_id;
    out->role = pending->role;
    out->ack_expected = pending->cmd.ack_requested;
    out->operation = pending->operation;
    out->object = pending->cmd.object;
    out->dispatch_result = dispatch_result;
    fill_semantic_command_fields(&pending->cmd_desc, out);
    (void)motor_diag_format_cmd(out->command_summary, sizeof(out->command_summary), &pending->cmd_desc);

    switch (dispatch_result) {
        case MOTOR_DISPATCH_RESULT_OK:
            out->status = MOTOR_EXEC_STATUS_SUCCESS;
            if (pending->cmd.ack_requested && response != NULL) {
                motor_diag_rx_desc_t response_desc;

                if (motor_diag_describe_rx(response, &response_desc)) {
                    fill_response_value_summary(&response_desc, out);
                    (void)motor_diag_format_rx(out->completion_summary,
                                               sizeof(out->completion_summary),
                                               &response_desc);
                }
            }
            break;
        case MOTOR_DISPATCH_RESULT_TIMEOUT:
            out->status = MOTOR_EXEC_STATUS_TIMEOUT;
            break;
        case MOTOR_DISPATCH_RESULT_REMOTE_ERROR:
            out->status = MOTOR_EXEC_STATUS_REMOTE_ERROR;
            if (related_error != NULL) {
                motor_diag_rx_desc_t error_desc;

                if (motor_diag_describe_rx(related_error, &error_desc)) {
                    fill_remote_error(&error_desc, out);
                    (void)motor_diag_format_rx(out->completion_summary,
                                               sizeof(out->completion_summary),
                                               &error_desc);
                }
            }
            break;
        case MOTOR_DISPATCH_RESULT_TRANSPORT_ERROR:
            out->status = MOTOR_EXEC_STATUS_TRANSPORT_ERROR;
            break;
        case MOTOR_DISPATCH_RESULT_INTERNAL_ERROR:
        case MOTOR_DISPATCH_RESULT_INVALID_ARGUMENT:
        case MOTOR_DISPATCH_RESULT_BUSY:
        default:
            out->status = MOTOR_EXEC_STATUS_INTERNAL_ERROR;
            break;
    }

    if (out->completion_summary[0] == '\0') {
        build_completion_summary(pending, out, out->completion_summary, sizeof(out->completion_summary));
    }
}

static void worker_task(void *arg)
{
    (void)arg;

    for (;;) {
        motor_exec_pending_t pending;
        motor_exec_observer_ctx_t observer_ctx;
        motor_dispatch_observer_t observer = {0};
        motor_dispatch_result_t dispatch_result;
        motor_rx_t response = {0};
        motor_rx_t related_error = {0};
        motor_exec_result_t result;
        motor_exec_completion_fn callback;
        void *callback_ctx;

        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!copy_pending_snapshot(&pending)) {
            continue;
        }

        observer_ctx.log_domain = pending.log_domain;
        observer.on_notification = log_side_traffic;
        observer.on_error = log_side_traffic;
        observer.ctx = &observer_ctx;

        dispatch_result = s_dispatch_fn(&pending.cmd,
                                        pending.opts.timing.timeout_ticks,
                                        pending.opts.timing.no_ack_grace_ticks,
                                        &observer,
                                        &response,
                                        &related_error);

        populate_result(&pending,
                        dispatch_result,
                        dispatch_result == MOTOR_DISPATCH_RESULT_OK ? &response : NULL,
                        dispatch_result == MOTOR_DISPATCH_RESULT_REMOTE_ERROR ? &related_error : NULL,
                        &result);

        callback = pending.opts.on_complete;
        callback_ctx = pending.opts.ctx;
        clear_pending_slot();
        log_completion(&pending, &result);

        if (callback != NULL) {
            callback(&result, callback_ctx);
        }
    }
}

static esp_err_t ensure_worker_task(void)
{
    TaskHandle_t worker = NULL;

    for (;;) {
        portENTER_CRITICAL(&s_lock);
        if (s_worker_task != NULL) {
            portEXIT_CRITICAL(&s_lock);
            return ESP_OK;
        }

        if (!s_worker_creating) {
            /*
             * Invariant:
             * - exactly one submitter may transition from "no worker" to
             *   "creating"
             * - the task handle is published only after xTaskCreate() returns
             * - submitters notify the worker only after ensure_worker_task()
             *   returns
             *
             * FreeRTOS task notifications are latched, so notifying
             * immediately after handle publication is safe even if the worker
             * has not yet blocked in ulTaskNotifyTake().
             */
            s_worker_creating = true;
            portEXIT_CRITICAL(&s_lock);
            break;
        }
        portEXIT_CRITICAL(&s_lock);

        /*
         * Another submitter is creating the singleton worker. Yield and retry
         * rather than failing the caller with a transient setup race.
         */
        taskYIELD();
    }

    if (xTaskCreate(worker_task,
                    "motor_exec",
                    MOTOR_EXEC_WORKER_STACK_WORDS,
                    NULL,
                    MOTOR_EXEC_WORKER_PRIORITY,
                    &worker) != pdPASS) {
        portENTER_CRITICAL(&s_lock);
        s_worker_creating = false;
        portEXIT_CRITICAL(&s_lock);
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&s_lock);
    s_worker_task = worker;
    s_worker_creating = false;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

static motor_exec_submit_result_t submit_cmd(motor_exec_role_t role,
                                             const motor_cmd_t *cmd,
                                             const motor_exec_submit_opts_t *opts)
{
    motor_exec_submit_result_t result = {
        .accepted = false,
        .code = MOTOR_EXEC_SUBMIT_INVALID_ARGUMENT,
        .err = ESP_ERR_INVALID_ARG,
        .request_id = 0U,
    };
    motor_diag_cmd_desc_t cmd_desc;
    motor_exec_log_domain_t log_domain;
    motor_exec_operation_t operation;
    esp_err_t err;

    if (cmd == NULL) {
        return result;
    }

    err = validate_opts(opts);
    if (err != ESP_OK) {
        return result;
    }

    err = ensure_worker_task();
    if (err != ESP_OK) {
        result.code = MOTOR_EXEC_SUBMIT_SCHEDULING_FAILED;
        result.err = err;
        return result;
    }

    memset(&cmd_desc, 0, sizeof(cmd_desc));
    if (!motor_diag_describe_cmd(cmd, &cmd_desc)) {
        result.code = MOTOR_EXEC_SUBMIT_BUILD_FAILED;
        result.err = ESP_ERR_INVALID_STATE;
        return result;
    }
    log_domain = classify_log_domain(cmd);
    operation = classify_operation(cmd, &cmd_desc);

    if (!reserve_pending_slot(role,
                              cmd,
                              opts,
                              log_domain,
                              operation,
                              &cmd_desc,
                              &result.request_id)) {
        result.code = MOTOR_EXEC_SUBMIT_BUSY;
        result.err = ESP_ERR_INVALID_STATE;
        return result;
    }

    log_submit(result.request_id, role, log_domain, cmd);
    xTaskNotifyGive(s_worker_task);

    result.accepted = true;
    result.code = MOTOR_EXEC_SUBMIT_OK;
    result.err = ESP_OK;
    return result;
}

static motor_exec_submit_result_t submit_from_builder(esp_err_t builder_rc,
                                                      const motor_cmd_t *cmd,
                                                      const motor_exec_submit_opts_t *opts)
{
    motor_exec_submit_result_t result = {
        .accepted = false,
        .code = MOTOR_EXEC_SUBMIT_BUILD_FAILED,
        .err = builder_rc,
        .request_id = 0U,
    };

    if (builder_rc != ESP_OK) {
        result.code = (builder_rc == ESP_ERR_INVALID_ARG) ?
            MOTOR_EXEC_SUBMIT_INVALID_ARGUMENT :
            MOTOR_EXEC_SUBMIT_BUILD_FAILED;
        return result;
    }

    return submit_cmd(MOTOR_EXEC_ROLE_BRAKE, cmd, opts);
}

motor_exec_log_cfg_t motor_exec_default_log_cfg(void)
{
    return (motor_exec_log_cfg_t) {
        .enabled = true,
        .setup_enabled = true,
        .log_notifications = true,
        .log_unrelated_errors = true,
        .motion_enabled = true,
    };
}

motor_exec_timing_t motor_exec_default_timing(void)
{
    return (motor_exec_timing_t) {
        .timeout_ticks = pdMS_TO_TICKS(100),
        .no_ack_grace_ticks = pdMS_TO_TICKS(50),
    };
}

motor_exec_timing_t motor_exec_reboot_timing(void)
{
    return (motor_exec_timing_t) {
        .timeout_ticks = pdMS_TO_TICKS(1000),
        .no_ack_grace_ticks = pdMS_TO_TICKS(50),
    };
}

esp_err_t motor_exec_set_log_cfg(const motor_exec_log_cfg_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);
    s_log_cfg = *cfg;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t motor_exec_get_log_cfg(motor_exec_log_cfg_t *out_cfg)
{
    if (out_cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);
    *out_cfg = s_log_cfg;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

bool motor_exec_has_pending(void)
{
    bool active;

    portENTER_CRITICAL(&s_lock);
    active = s_pending_active;
    portEXIT_CRITICAL(&s_lock);
    return active;
}

motor_exec_submit_result_t motor_exec_brake_pp_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_pp_index_t index,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_pp_get(node_id, ack_requested, index, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_pp_set_u8(uint8_t node_id,
                                                      bool ack_requested,
                                                      motor_pp_index_t index,
                                                      uint8_t value,
                                                      const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_pp_set_u8(node_id, ack_requested, index, value, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_ic_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_ic_index_t index,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_ic_get(node_id, ack_requested, index, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_ic_set_u16(uint8_t node_id,
                                                       bool ack_requested,
                                                       motor_ic_index_t index,
                                                       uint16_t value,
                                                       const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_ic_set_u16(node_id, ack_requested, index, value, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_ie_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_ie_index_t index,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_ie_get(node_id, ack_requested, index, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_ie_set_u16(uint8_t node_id,
                                                       bool ack_requested,
                                                       motor_ie_index_t index,
                                                       uint16_t value,
                                                       const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_ie_set_u16(node_id, ack_requested, index, value, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_er_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_er_index_t index,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_er_get(node_id, ack_requested, index, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_er_clear_all(uint8_t node_id,
                                                         bool ack_requested,
                                                         const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_er_clear_all(node_id, ack_requested, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_qe_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_qe_index_t index,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_qe_get(node_id, ack_requested, index, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_qe_set_u16(uint8_t node_id,
                                                       bool ack_requested,
                                                       motor_qe_index_t index,
                                                       uint16_t value,
                                                       const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_qe_set_u16(node_id, ack_requested, index, value, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_sy_reboot(uint8_t node_id,
                                                      const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_sy_reboot(node_id, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_sy_restore_defaults(uint8_t node_id,
                                                                const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_sy_restore_factory_defaults(node_id, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_mt_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_mt_index_t index,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_mt_get(node_id, ack_requested, index, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_mt_set_u16(uint8_t node_id,
                                                       bool ack_requested,
                                                       motor_mt_index_t index,
                                                       uint16_t value,
                                                       const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_mt_set_u16(node_id, ack_requested, index, value, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_il_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_il_index_t index,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_il_get(node_id, ack_requested, index, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_il_set_u16(uint8_t node_id,
                                                       bool ack_requested,
                                                       motor_il_index_t index,
                                                       uint16_t value,
                                                       const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_il_set_u16(node_id, ack_requested, index, value, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_mo_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_mo_get(node_id, ack_requested, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_mo_set(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_mo_state_t state,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_mo_set(node_id, ack_requested, state, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_bg_begin(uint8_t node_id,
                                                     bool ack_requested,
                                                     const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_bg_begin(node_id, ack_requested, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_st_stop(uint8_t node_id,
                                                    bool ack_requested,
                                                    const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_st_stop(node_id, ack_requested, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_pa_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_pa_get(node_id, ack_requested, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_pa_set(uint8_t node_id,
                                                   bool ack_requested,
                                                   int32_t position,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_pa_set(node_id, ack_requested, position, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_mp_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_mp_index_t index,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_mp_get(node_id, ack_requested, index, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_mp_set_u16(uint8_t node_id,
                                                       bool ack_requested,
                                                       motor_mp_index_t index,
                                                       uint16_t value,
                                                       const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_mp_set_u16(node_id, ack_requested, index, value, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_pv_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_pv_get(node_id, ack_requested, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_pv_set(uint8_t node_id,
                                                   bool ack_requested,
                                                   uint16_t value,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_pv_set(node_id, ack_requested, value, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_pt_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   uint16_t row_index,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_pt_get(node_id, ack_requested, row_index, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_pt_set(uint8_t node_id,
                                                   bool ack_requested,
                                                   uint16_t row_index,
                                                   int32_t queued_position,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_pt_set(node_id, ack_requested, row_index, queued_position, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_og_set_origin(uint8_t node_id,
                                                          bool ack_requested,
                                                          const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_og_set_origin(node_id, ack_requested, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_lm_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_lm_index_t index,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_lm_get(node_id, ack_requested, index, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_lm_set_i32(uint8_t node_id,
                                                       bool ack_requested,
                                                       motor_lm_index_t index,
                                                       int32_t value,
                                                       const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_lm_set_i32(node_id, ack_requested, index, value, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_sd_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_sd_get(node_id, ack_requested, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_sd_set_u32(uint8_t node_id,
                                                       bool ack_requested,
                                                       uint32_t value,
                                                       const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_sd_set_u32(node_id, ack_requested, value, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_bl_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_bl_get(node_id, ack_requested, &cmd), &cmd, opts);
}

motor_exec_submit_result_t motor_exec_brake_bl_set_u32(uint8_t node_id,
                                                       bool ack_requested,
                                                       uint32_t value,
                                                       const motor_exec_submit_opts_t *opts)
{
    motor_cmd_t cmd;

    return submit_from_builder(motor_cmd_bl_set_u32(node_id, ack_requested, value, &cmd), &cmd, opts);
}

void motor_exec_test_set_dispatch_fn(motor_exec_dispatch_fn_t fn)
{
    portENTER_CRITICAL(&s_lock);
    s_dispatch_fn = fn != NULL ? fn : dispatch_default;
    portEXIT_CRITICAL(&s_lock);
}

void motor_exec_test_set_rx_log_fn(motor_exec_rx_log_fn_t fn)
{
    portENTER_CRITICAL(&s_lock);
    s_rx_log_fn = fn != NULL ? fn : motor_diag_log_rx;
    portEXIT_CRITICAL(&s_lock);
}

void motor_exec_test_reset_state(void)
{
    portENTER_CRITICAL(&s_lock);
    memset(&s_pending, 0, sizeof(s_pending));
    s_pending_active = false;
    s_next_request_id = 1U;
    s_log_cfg = motor_exec_default_log_cfg();
    s_dispatch_fn = dispatch_default;
    s_rx_log_fn = motor_diag_log_rx;
    portEXIT_CRITICAL(&s_lock);
}
