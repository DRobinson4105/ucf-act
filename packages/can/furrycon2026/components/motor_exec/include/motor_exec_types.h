/*
 * Responsibility:
 * Public request, result, and callback types for the nonblocking motor_exec
 * execution wrapper.
 */

#ifndef MOTOR_EXEC_TYPES_H
#define MOTOR_EXEC_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "motor_dispatch.h"
#include "motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t motor_exec_request_id_t;

typedef enum {
    MOTOR_EXEC_ROLE_NONE = 0,
    MOTOR_EXEC_ROLE_BRAKE,
} motor_exec_role_t;

typedef enum {
    MOTOR_EXEC_OPERATION_UNKNOWN = 0,
    MOTOR_EXEC_OPERATION_GET,
    MOTOR_EXEC_OPERATION_SET,
    MOTOR_EXEC_OPERATION_CLEAR,
    MOTOR_EXEC_OPERATION_ACTION,
} motor_exec_operation_t;

typedef enum {
    MOTOR_EXEC_STATUS_UNKNOWN = 0,
    MOTOR_EXEC_STATUS_SUCCESS,
    MOTOR_EXEC_STATUS_TIMEOUT,
    MOTOR_EXEC_STATUS_REMOTE_ERROR,
    MOTOR_EXEC_STATUS_TRANSPORT_ERROR,
    MOTOR_EXEC_STATUS_INTERNAL_ERROR,
} motor_exec_status_t;

typedef enum {
    MOTOR_EXEC_SUBMIT_OK = 0,
    MOTOR_EXEC_SUBMIT_INVALID_ARGUMENT,
    MOTOR_EXEC_SUBMIT_BUILD_FAILED,
    MOTOR_EXEC_SUBMIT_BUSY,
    MOTOR_EXEC_SUBMIT_SCHEDULING_FAILED,
} motor_exec_submit_code_t;

typedef enum {
    MOTOR_EXEC_LOG_DOMAIN_SETUP = 0,
    MOTOR_EXEC_LOG_DOMAIN_MOTION,
} motor_exec_log_domain_t;

typedef enum {
    MOTOR_EXEC_VALUE_KIND_NONE = 0,
    MOTOR_EXEC_VALUE_KIND_U8,
    MOTOR_EXEC_VALUE_KIND_U16,
    MOTOR_EXEC_VALUE_KIND_U32,
    MOTOR_EXEC_VALUE_KIND_I32,
    MOTOR_EXEC_VALUE_KIND_ENUM,
    MOTOR_EXEC_VALUE_KIND_TEXT,
} motor_exec_value_kind_t;

typedef struct {
    bool enabled;
    bool setup_enabled;
    bool motion_enabled;
} motor_exec_log_cfg_t;

typedef struct {
    motor_exec_value_kind_t kind;
    bool has_value;
    int64_t number;
    bool has_raw_number;
    int64_t raw_number;
    char text[64];
    char units[16];
} motor_exec_value_t;

typedef struct {
    TickType_t timeout_ticks;
    TickType_t no_ack_grace_ticks;
} motor_exec_timing_t;

typedef struct {
    bool accepted;
    motor_exec_submit_code_t code;
    esp_err_t err;
    motor_exec_request_id_t request_id;
} motor_exec_submit_result_t;

typedef struct {
    bool valid;
    uint8_t code;
    /* Inline text fields are copied into the result and remain valid after callback return. */
    char name[96];
    bool has_related_family;
    char related_family_symbol[8];
    bool has_related_index;
    uint16_t related_index;
} motor_exec_remote_error_t;

/*
 * Copy-safe terminal result payload for completion callbacks.
 * All human-readable text is stored inline so callers may keep a by-value copy
 * after the callback returns without depending on internal motor_exec storage.
 */
typedef struct {
    motor_exec_request_id_t request_id;
    uint8_t node_id;
    motor_exec_role_t role;
    bool ack_expected;
    motor_exec_operation_t operation;
    motor_object_t object;
    motor_exec_status_t status;
    motor_dispatch_result_t dispatch_result;
    char command_family_symbol[8];
    bool has_parameter_index;
    uint16_t parameter_index;
    char parameter_name[96];
    char parameter_value_summary[64];
    bool has_response_value_summary;
    char response_value_summary[64];
    bool has_response_value;
    motor_exec_value_t response_value;
    motor_exec_remote_error_t remote_error;
    char command_summary[160];
    char completion_summary[160];
} motor_exec_result_t;

typedef void (*motor_exec_completion_fn)(const motor_exec_result_t *result, void *ctx);

typedef struct {
    motor_exec_completion_fn on_complete;
    void *ctx;
    /* Use motor_exec_default_timing() unless a command needs a different policy. */
    motor_exec_timing_t timing;
} motor_exec_submit_opts_t;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_EXEC_TYPES_H */
