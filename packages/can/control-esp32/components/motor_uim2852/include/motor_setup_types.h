/*
 * Responsibility:
 * Public plan, step, verification, and result types for blocking multi-step
 * motor setup execution built above motor_exec.
 */

#ifndef MOTOR_SETUP_TYPES_H
#define MOTOR_SETUP_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "motor_exec.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_SETUP_STEP_KIND_UNKNOWN = 0,
    MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
    MOTOR_SETUP_STEP_KIND_READ_ONLY,
    MOTOR_SETUP_STEP_KIND_WRITE_THEN_VERIFY,
} motor_setup_step_kind_t;

typedef enum {
    MOTOR_SETUP_COMPARE_NONE = 0,
    MOTOR_SETUP_COMPARE_INT_EQ,
    MOTOR_SETUP_COMPARE_ENUM_EQ,
    MOTOR_SETUP_COMPARE_BOOL_EQ,
} motor_setup_compare_kind_t;

typedef enum {
    MOTOR_SETUP_VALUE_KIND_NONE = 0,
    MOTOR_SETUP_VALUE_KIND_U8,
    MOTOR_SETUP_VALUE_KIND_U16,
    MOTOR_SETUP_VALUE_KIND_U32,
    MOTOR_SETUP_VALUE_KIND_I32,
    MOTOR_SETUP_VALUE_KIND_BOOL,
    MOTOR_SETUP_VALUE_KIND_ENUM,
} motor_setup_value_kind_t;

typedef enum {
    MOTOR_SETUP_FAILURE_NONE = 0,
    MOTOR_SETUP_FAILURE_INVALID_PLAN,
    MOTOR_SETUP_FAILURE_EXEC_SUBMIT,
    MOTOR_SETUP_FAILURE_EXEC_RESULT,
    MOTOR_SETUP_FAILURE_VERIFY_SUBMIT,
    MOTOR_SETUP_FAILURE_VERIFY_RESULT,
    MOTOR_SETUP_FAILURE_VERIFY_MISMATCH,
    MOTOR_SETUP_FAILURE_INTERNAL,
} motor_setup_failure_reason_t;

typedef enum {
    MOTOR_SETUP_STATUS_UNKNOWN = 0,
    MOTOR_SETUP_STATUS_SUCCESS,
    MOTOR_SETUP_STATUS_INVALID_PLAN,
    MOTOR_SETUP_STATUS_FAILED,
} motor_setup_status_t;

typedef struct {
    motor_setup_value_kind_t kind;
    union {
        uint8_t u8;
        uint16_t u16;
        uint32_t u32;
        int32_t i32;
        bool boolean;
        int32_t enum_value;
    } as;
} motor_setup_value_t;

typedef struct {
    motor_exec_role_t role;
    uint8_t node_id;
    motor_exec_operation_t operation;
    motor_object_t object;
    bool ack_requested;
    bool has_index;
    uint16_t index;
    bool has_value;
    motor_setup_value_t value;
} motor_setup_action_t;

typedef struct {
    const char *name;
    motor_setup_step_kind_t kind;
    motor_setup_action_t exec_action;
    bool has_exec_timing_override;
    motor_exec_timing_t exec_timing;
    motor_setup_compare_kind_t compare_kind;
    motor_setup_value_t expected_value;
    motor_setup_action_t verify_action;
    bool has_verify_timing_override;
    motor_exec_timing_t verify_timing;
} motor_setup_step_t;

typedef struct {
    const char *name;
    const motor_setup_step_t *steps;
    size_t step_count;
} motor_setup_plan_t;

typedef struct {
    bool enabled;
    bool log_step_success;
} motor_setup_log_cfg_t;

typedef struct {
    const char *step_name;
    size_t step_index;
    bool execution_attempted;
    motor_exec_submit_result_t execution_submit;
    bool execution_success;
    motor_exec_result_t execution_result;
    bool verification_attempted;
    motor_exec_submit_result_t verification_submit;
    bool verification_success;
    motor_exec_result_t verification_result;
    bool comparison_attempted;
    bool comparison_success;
    bool has_actual_value;
    motor_setup_value_t actual_value;
    motor_setup_failure_reason_t failure_reason;
} motor_setup_step_result_t;

typedef struct {
    motor_setup_step_result_t *step_results;
    size_t step_results_capacity;
} motor_setup_run_cfg_t;

typedef struct {
    motor_setup_status_t status;
    const char *plan_name;
    size_t total_steps;
    size_t completed_steps;
    bool has_failure;
    size_t failing_step_index;
    const char *failing_step_name;
    motor_setup_failure_reason_t failure_reason;
    motor_setup_step_result_t failing_step;
    size_t step_results_written;
} motor_setup_result_t;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_SETUP_TYPES_H */
