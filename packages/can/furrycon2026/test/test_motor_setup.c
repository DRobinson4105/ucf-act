/*
 * Responsibility:
 * Unity coverage for the blocking motor_setup plan runner.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "unity.h"

#include "motor_exec.h"
#include "motor_setup.h"
#include "motor_setup_binding.h"
#include "motor_setup_internal.h"

typedef struct {
    motor_exec_submit_result_t submits[16];
    motor_exec_result_t results[16];
    int call_count;
    motor_setup_action_t actions[16];
} fake_submit_t;

static fake_submit_t s_fake;

static void motor_setup_test_begin(void);
static void motor_setup_test_end(void);

static motor_exec_submit_result_t fake_submit(const motor_setup_action_t *action,
                                              const motor_exec_submit_opts_t *opts)
{
    int index = s_fake.call_count++;

    TEST_ASSERT_NOT_NULL(action);
    TEST_ASSERT_NOT_NULL(opts);
    TEST_ASSERT_NOT_NULL(opts->on_complete);
    TEST_ASSERT_LESS_THAN(16, index);

    s_fake.actions[index] = *action;
    if (!s_fake.submits[index].accepted) {
        return s_fake.submits[index];
    }

    opts->on_complete(&s_fake.results[index], opts->ctx);
    return s_fake.submits[index];
}

static motor_exec_result_t make_success_result(motor_object_t object,
                                               motor_exec_value_kind_t kind,
                                               int64_t raw_value)
{
    motor_exec_result_t result = {
        .status = MOTOR_EXEC_STATUS_SUCCESS,
        .object = object,
        .dispatch_result = MOTOR_DISPATCH_RESULT_OK,
        .has_response_value = kind != MOTOR_EXEC_VALUE_KIND_NONE,
        .has_response_value_summary = kind != MOTOR_EXEC_VALUE_KIND_NONE,
    };

    result.response_value.kind = kind;
    result.response_value.has_value = kind != MOTOR_EXEC_VALUE_KIND_NONE;
    result.response_value.has_raw_number = kind != MOTOR_EXEC_VALUE_KIND_NONE;
    result.response_value.raw_number = raw_value;
    result.response_value.number = raw_value;
    return result;
}

TEST_CASE("motor setup compares and records widened U32 response values", "[motor_setup]")
{
    static motor_setup_result_t result;
    static motor_setup_step_result_t history[1];
    static const motor_setup_step_t steps[] = {
        {
            .name = "read widened numeric response",
            .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_PP,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_PP_INDEX_NODE_ID,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_INT_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_U32, .as.u32 = 70000U},
        },
    };
    static const motor_setup_plan_t plan = {
        .name = "u32 consumer",
        .steps = steps,
        .step_count = 1U,
    };
    const motor_setup_run_cfg_t cfg = {
        .step_results = history,
        .step_results_capacity = 1U,
    };

    motor_setup_test_begin();
    memset(&result, 0, sizeof(result));
    memset(history, 0, sizeof(history));
    s_fake.submits[0] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 1U};
    s_fake.results[0] = make_success_result(MOTOR_OBJECT_PP, MOTOR_EXEC_VALUE_KIND_U32, 70000);

    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_run_plan(&plan, &cfg, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_STATUS_SUCCESS, result.status);
    TEST_ASSERT_TRUE(history[0].comparison_attempted);
    TEST_ASSERT_TRUE(history[0].comparison_success);
    TEST_ASSERT_TRUE(history[0].has_actual_value);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_VALUE_KIND_U32, history[0].actual_value.kind);
    TEST_ASSERT_EQUAL_UINT32(70000U, history[0].actual_value.as.u32);
    motor_setup_test_end();
}

static void motor_setup_test_begin(void)
{
    memset(&s_fake, 0, sizeof(s_fake));
    motor_setup_test_reset_state();
    motor_setup_test_set_submit_fn(fake_submit);
}

static void motor_setup_test_end(void)
{
    motor_setup_test_reset_state();
}

TEST_CASE("motor setup binding support stays aligned with exposed command surface", "[motor_setup]")
{
    motor_setup_action_t supported_get = {
        .role = MOTOR_EXEC_ROLE_BRAKE,
        .node_id = 6U,
        .operation = MOTOR_EXEC_OPERATION_GET,
        .object = MOTOR_OBJECT_PP,
        .ack_requested = true,
        .has_index = true,
        .index = MOTOR_PP_INDEX_NODE_ID,
    };
    motor_setup_action_t supported_set = {
        .role = MOTOR_EXEC_ROLE_BRAKE,
        .node_id = 6U,
        .operation = MOTOR_EXEC_OPERATION_SET,
        .object = MOTOR_OBJECT_LM,
        .ack_requested = true,
        .has_index = true,
        .index = MOTOR_LM_INDEX_MAX_ACCEL_DECEL,
        .has_value = true,
        .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as.i32 = 3333},
    };
    motor_setup_action_t supported_new_indexed_family = {
        .role = MOTOR_EXEC_ROLE_BRAKE,
        .node_id = 6U,
        .operation = MOTOR_EXEC_OPERATION_GET,
        .object = MOTOR_OBJECT_IE,
        .ack_requested = true,
        .has_index = true,
        .index = MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION,
    };
    motor_setup_action_t supported_new_scalar = {
        .role = MOTOR_EXEC_ROLE_BRAKE,
        .node_id = 6U,
        .operation = MOTOR_EXEC_OPERATION_SET,
        .object = MOTOR_OBJECT_SD,
        .ack_requested = true,
        .has_value = true,
        .value = {.kind = MOTOR_SETUP_VALUE_KIND_U32, .as.u32 = 321000U},
    };
    motor_setup_action_t supported_sy_action = {
        .role = MOTOR_EXEC_ROLE_BRAKE,
        .node_id = 6U,
        .operation = MOTOR_EXEC_OPERATION_ACTION,
        .object = MOTOR_OBJECT_SY,
        .ack_requested = false,
        .has_index = true,
        .index = MOTOR_SY_OP_REBOOT,
    };
    motor_setup_action_t unsupported_shape = {
        .role = MOTOR_EXEC_ROLE_BRAKE,
        .node_id = 6U,
        .operation = MOTOR_EXEC_OPERATION_SET,
        .object = MOTOR_OBJECT_SD,
        .ack_requested = true,
        .has_index = true,
        .index = 1U,
        .has_value = true,
        .value = {.kind = MOTOR_SETUP_VALUE_KIND_U32, .as.u32 = 321000U},
    };

    TEST_ASSERT_TRUE(motor_setup_action_binding_supported(&supported_get));
    TEST_ASSERT_TRUE(motor_setup_action_binding_supported(&supported_set));
    TEST_ASSERT_TRUE(motor_setup_action_binding_supported(&supported_new_indexed_family));
    TEST_ASSERT_TRUE(motor_setup_action_binding_supported(&supported_new_scalar));
    TEST_ASSERT_TRUE(motor_setup_action_binding_supported(&supported_sy_action));
    TEST_ASSERT_TRUE(motor_setup_action_binding_supported(&unsupported_shape));
    TEST_ASSERT_FALSE(motor_setup_action_binding_supported(NULL));
}

TEST_CASE("motor setup supports representative new family read-only steps", "[motor_setup]")
{
    static motor_setup_result_t result;
    static motor_setup_step_result_t history[6];
    static const motor_setup_step_t steps[] = {
        {
            .name = "read ie fifo empty",
            .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_IE,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_ENUM_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_ENUM, .as.enum_value = MOTOR_IE_STATE_ENABLE},
        },
        {
            .name = "read qe battery",
            .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_QE,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_QE_INDEX_BATTERY_VOLTAGE,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_INT_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as.u16 = 24000U},
        },
        {
            .name = "read il stall behavior",
            .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_IL,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_IL_INDEX_STALL_BEHAVIOR,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_INT_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as.u16 = 2U},
        },
        {
            .name = "read mp mode",
            .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_MP,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_ENUM_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_ENUM, .as.enum_value = MOTOR_MP_MODE_SINGLE},
        },
        {
            .name = "read sd",
            .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_SD,
                .ack_requested = true,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_INT_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_U32, .as.u32 = 321000U},
        },
        {
            .name = "read bl",
            .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_BL,
                .ack_requested = true,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_INT_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_U32, .as.u32 = 65535U},
        },
    };
    static const motor_setup_plan_t plan = {
        .name = "new family reads",
        .steps = steps,
        .step_count = 6U,
    };
    const motor_setup_run_cfg_t cfg = {
        .step_results = history,
        .step_results_capacity = 6U,
    };

    motor_setup_test_begin();
    memset(&result, 0, sizeof(result));
    memset(history, 0, sizeof(history));
    s_fake.submits[0] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 1U};
    s_fake.submits[1] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 2U};
    s_fake.submits[2] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 3U};
    s_fake.submits[3] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 4U};
    s_fake.submits[4] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 5U};
    s_fake.submits[5] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 6U};
    s_fake.results[0] = make_success_result(MOTOR_OBJECT_IE, MOTOR_EXEC_VALUE_KIND_ENUM, MOTOR_IE_STATE_ENABLE);
    s_fake.results[1] = make_success_result(MOTOR_OBJECT_QE, MOTOR_EXEC_VALUE_KIND_U16, 24000);
    s_fake.results[2] = make_success_result(MOTOR_OBJECT_IL, MOTOR_EXEC_VALUE_KIND_U16, 2);
    s_fake.results[3] = make_success_result(MOTOR_OBJECT_MP, MOTOR_EXEC_VALUE_KIND_ENUM, MOTOR_MP_MODE_SINGLE);
    s_fake.results[4] = make_success_result(MOTOR_OBJECT_SD, MOTOR_EXEC_VALUE_KIND_U32, 321000);
    s_fake.results[5] = make_success_result(MOTOR_OBJECT_BL, MOTOR_EXEC_VALUE_KIND_U32, 65535);

    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_run_plan(&plan, &cfg, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_STATUS_SUCCESS, result.status);
    TEST_ASSERT_EQUAL(6, s_fake.call_count);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_VALUE_KIND_ENUM, history[0].actual_value.kind);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_VALUE_KIND_U16, history[1].actual_value.kind);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_VALUE_KIND_U16, history[2].actual_value.kind);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_VALUE_KIND_ENUM, history[3].actual_value.kind);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_VALUE_KIND_U32, history[4].actual_value.kind);
    TEST_ASSERT_EQUAL_UINT32(321000U, history[4].actual_value.as.u32);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_VALUE_KIND_U32, history[5].actual_value.kind);
    TEST_ASSERT_EQUAL_UINT32(65535U, history[5].actual_value.as.u32);
    motor_setup_test_end();
}

TEST_CASE("motor setup accepts ER readback steps without comparable response values", "[motor_setup]")
{
    static motor_setup_result_t result;
    static motor_setup_step_result_t history[1];
    static const motor_setup_step_t steps[] = {
        {
            .name = "read newest error history",
            .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_ER,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_ER_INDEX_HISTORY_1,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_NONE,
        },
    };
    static const motor_setup_plan_t plan = {
        .name = "er readback",
        .steps = steps,
        .step_count = 1U,
    };
    const motor_setup_run_cfg_t cfg = {
        .step_results = history,
        .step_results_capacity = 1U,
    };

    motor_setup_test_begin();
    memset(&result, 0, sizeof(result));
    memset(history, 0, sizeof(history));
    s_fake.submits[0] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 7U};
    s_fake.results[0] = make_success_result(MOTOR_OBJECT_ER, MOTOR_EXEC_VALUE_KIND_NONE, 0);

    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_run_plan(&plan, &cfg, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_STATUS_SUCCESS, result.status);
    TEST_ASSERT_TRUE(history[0].execution_attempted);
    TEST_ASSERT_TRUE(history[0].execution_success);
    TEST_ASSERT_TRUE(history[0].verification_attempted);
    TEST_ASSERT_TRUE(history[0].verification_success);
    TEST_ASSERT_FALSE(history[0].comparison_attempted);
    TEST_ASSERT_FALSE(history[0].comparison_success);
    TEST_ASSERT_FALSE(history[0].has_actual_value);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_FAILURE_NONE, history[0].failure_reason);
    motor_setup_test_end();
}

TEST_CASE("motor setup rejects null or malformed plans", "[motor_setup]")
{
    static motor_setup_result_t result;
    static const motor_setup_step_t steps[] = {
        {
            .name = "bad",
            .kind = MOTOR_SETUP_STEP_KIND_WRITE_THEN_VERIFY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_SET,
                .object = MOTOR_OBJECT_PP,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_PP_INDEX_BITRATE,
                .has_value = true,
                .value = {.kind = MOTOR_SETUP_VALUE_KIND_U8, .as.u8 = MOTOR_PP_BITRATE_500K},
            },
        },
    };
    static const motor_setup_plan_t plan = {
        .name = "bad",
        .steps = steps,
        .step_count = 1U,
    };

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_setup_validate_plan(NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_setup_run_plan(NULL, NULL, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_STATUS_INVALID_PLAN, result.status);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_setup_validate_plan(&plan));
}

TEST_CASE("motor setup treats empty plan as successful no-op", "[motor_setup]")
{
    static motor_setup_result_t result;
    static const motor_setup_plan_t plan = {
        .name = "empty",
        .steps = NULL,
        .step_count = 0U,
    };

    motor_setup_test_begin();
    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_run_plan(&plan, NULL, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_STATUS_SUCCESS, result.status);
    TEST_ASSERT_EQUAL_UINT32(0U, result.total_steps);
    TEST_ASSERT_EQUAL_UINT32(0U, result.completed_steps);
    TEST_ASSERT_EQUAL(0, s_fake.call_count);
    motor_setup_test_end();
}

TEST_CASE("motor setup executes steps in order and records history", "[motor_setup]")
{
    static motor_setup_result_t result;
    static motor_setup_step_result_t history[2];
    static const motor_setup_step_t steps[] = {
        {
            .name = "set bitrate",
            .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_SET,
                .object = MOTOR_OBJECT_PP,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_PP_INDEX_BITRATE,
                .has_value = true,
                .value = {.kind = MOTOR_SETUP_VALUE_KIND_U8, .as.u8 = MOTOR_PP_BITRATE_500K},
            },
        },
        {
            .name = "read closed loop",
            .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_IC,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_IC_INDEX_USE_CLOSED_LOOP,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_ENUM_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_ENUM, .as.enum_value = 1},
        },
    };
    static const motor_setup_plan_t plan = {
        .name = "ordered",
        .steps = steps,
        .step_count = 2U,
    };
    const motor_setup_run_cfg_t cfg = {
        .step_results = history,
        .step_results_capacity = 2U,
    };

    motor_setup_test_begin();
    memset(&result, 0, sizeof(result));
    memset(history, 0, sizeof(history));
    s_fake.submits[0] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 1U};
    s_fake.submits[1] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 2U};
    s_fake.results[0] = make_success_result(MOTOR_OBJECT_PP, MOTOR_EXEC_VALUE_KIND_NONE, 0);
    s_fake.results[1] = make_success_result(MOTOR_OBJECT_IC, MOTOR_EXEC_VALUE_KIND_ENUM, 1);

    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_run_plan(&plan, &cfg, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_STATUS_SUCCESS, result.status);
    TEST_ASSERT_EQUAL(2, s_fake.call_count);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_PP, s_fake.actions[0].object);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_IC, s_fake.actions[1].object);
    TEST_ASSERT_EQUAL_UINT32(2U, result.completed_steps);
    TEST_ASSERT_EQUAL_UINT32(2U, result.step_results_written);
    TEST_ASSERT_TRUE(history[0].execution_attempted);
    TEST_ASSERT_TRUE(history[0].execution_success);
    TEST_ASSERT_TRUE(history[1].verification_attempted);
    TEST_ASSERT_TRUE(history[1].comparison_attempted);
    TEST_ASSERT_TRUE(history[1].comparison_success);
    motor_setup_test_end();
}

TEST_CASE("motor setup stops on first execution failure", "[motor_setup]")
{
    static motor_setup_result_t result;
    static motor_setup_step_result_t history[2];
    static const motor_setup_step_t steps[] = {
        {
            .name = "set motor",
            .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_SET,
                .object = MOTOR_OBJECT_MO,
                .ack_requested = true,
                .has_value = true,
                .value = {.kind = MOTOR_SETUP_VALUE_KIND_ENUM, .as.enum_value = MOTOR_MO_STATE_ENABLE},
            },
        },
        {
            .name = "never reached",
            .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_ACTION,
                .object = MOTOR_OBJECT_BG,
                .ack_requested = true,
            },
        },
    };
    static const motor_setup_plan_t plan = {
        .name = "stop first failure",
        .steps = steps,
        .step_count = 2U,
    };
    const motor_setup_run_cfg_t cfg = {
        .step_results = history,
        .step_results_capacity = 2U,
    };

    motor_setup_test_begin();
    memset(&result, 0, sizeof(result));
    memset(history, 0, sizeof(history));
    s_fake.submits[0] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 1U};
    s_fake.results[0] = make_success_result(MOTOR_OBJECT_MO, MOTOR_EXEC_VALUE_KIND_NONE, 0);
    s_fake.results[0].status = MOTOR_EXEC_STATUS_TIMEOUT;
    s_fake.results[0].dispatch_result = MOTOR_DISPATCH_RESULT_TIMEOUT;

    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_run_plan(&plan, &cfg, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_STATUS_FAILED, result.status);
    TEST_ASSERT_EQUAL_UINT32(0U, result.completed_steps);
    TEST_ASSERT_EQUAL_UINT32(0U, result.failing_step_index);
    TEST_ASSERT_EQUAL_STRING("set motor", result.failing_step_name);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_FAILURE_EXEC_RESULT, result.failure_reason);
    TEST_ASSERT_EQUAL(1, s_fake.call_count);
    TEST_ASSERT_FALSE(history[0].verification_attempted);
    motor_setup_test_end();
}

TEST_CASE("motor setup supports SY action steps with indexed opcodes", "[motor_setup]")
{
    static motor_setup_result_t result;
    static motor_setup_step_result_t history[2];
    static const motor_setup_step_t steps[] = {
        {
            .name = "reboot controller",
            .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_ACTION,
                .object = MOTOR_OBJECT_SY,
                .ack_requested = false,
                .has_index = true,
                .index = MOTOR_SY_OP_REBOOT,
            },
        },
        {
            .name = "restore defaults",
            .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_ACTION,
                .object = MOTOR_OBJECT_SY,
                .ack_requested = false,
                .has_index = true,
                .index = MOTOR_SY_OP_RESTORE_FACTORY_DEFAULTS,
            },
        },
    };
    static const motor_setup_plan_t plan = {
        .name = "sy actions",
        .steps = steps,
        .step_count = 2U,
    };
    const motor_setup_run_cfg_t cfg = {
        .step_results = history,
        .step_results_capacity = 2U,
    };

    motor_setup_test_begin();
    memset(&result, 0, sizeof(result));
    memset(history, 0, sizeof(history));
    s_fake.submits[0] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 1U};
    s_fake.submits[1] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 2U};
    s_fake.results[0] = make_success_result(MOTOR_OBJECT_SY, MOTOR_EXEC_VALUE_KIND_NONE, 0);
    s_fake.results[1] = make_success_result(MOTOR_OBJECT_SY, MOTOR_EXEC_VALUE_KIND_NONE, 0);

    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_run_plan(&plan, &cfg, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_STATUS_SUCCESS, result.status);
    TEST_ASSERT_EQUAL(2, s_fake.call_count);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_SY, s_fake.actions[0].object);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_OPERATION_ACTION, s_fake.actions[0].operation);
    TEST_ASSERT_TRUE(s_fake.actions[0].has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_SY_OP_REBOOT, s_fake.actions[0].index);
    TEST_ASSERT_FALSE(s_fake.actions[0].ack_requested);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_SY, s_fake.actions[1].object);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_OPERATION_ACTION, s_fake.actions[1].operation);
    TEST_ASSERT_TRUE(s_fake.actions[1].has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_SY_OP_RESTORE_FACTORY_DEFAULTS, s_fake.actions[1].index);
    TEST_ASSERT_FALSE(s_fake.actions[1].ack_requested);
    TEST_ASSERT_TRUE(history[0].execution_attempted);
    TEST_ASSERT_TRUE(history[0].execution_success);
    TEST_ASSERT_TRUE(history[1].execution_attempted);
    TEST_ASSERT_TRUE(history[1].execution_success);
    motor_setup_test_end();
}

TEST_CASE("motor setup supports write then verify success", "[motor_setup]")
{
    static motor_setup_result_t result;
    static motor_setup_step_result_t history[1];
    static const motor_setup_step_t steps[] = {
        {
            .name = "set closed loop",
            .kind = MOTOR_SETUP_STEP_KIND_WRITE_THEN_VERIFY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_SET,
                .object = MOTOR_OBJECT_IC,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_IC_INDEX_USE_CLOSED_LOOP,
                .has_value = true,
                .value = {.kind = MOTOR_SETUP_VALUE_KIND_BOOL, .as.boolean = true},
            },
            .verify_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_IC,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_IC_INDEX_USE_CLOSED_LOOP,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_BOOL_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_BOOL, .as.boolean = true},
        },
    };
    static const motor_setup_plan_t plan = {
        .name = "write verify ok",
        .steps = steps,
        .step_count = 1U,
    };
    const motor_setup_run_cfg_t cfg = {
        .step_results = history,
        .step_results_capacity = 1U,
    };

    motor_setup_test_begin();
    memset(&result, 0, sizeof(result));
    memset(history, 0, sizeof(history));
    s_fake.submits[0] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 10U};
    s_fake.submits[1] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 11U};
    s_fake.results[0] = make_success_result(MOTOR_OBJECT_IC, MOTOR_EXEC_VALUE_KIND_NONE, 0);
    s_fake.results[1] = make_success_result(MOTOR_OBJECT_IC, MOTOR_EXEC_VALUE_KIND_ENUM, 1);

    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_run_plan(&plan, &cfg, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_STATUS_SUCCESS, result.status);
    TEST_ASSERT_TRUE(history[0].execution_success);
    TEST_ASSERT_TRUE(history[0].verification_attempted);
    TEST_ASSERT_TRUE(history[0].verification_success);
    TEST_ASSERT_TRUE(history[0].comparison_attempted);
    TEST_ASSERT_TRUE(history[0].comparison_success);
    TEST_ASSERT_TRUE(history[0].has_actual_value);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_VALUE_KIND_ENUM, history[0].actual_value.kind);
    TEST_ASSERT_EQUAL(1, history[0].actual_value.as.enum_value);
    motor_setup_test_end();
}

TEST_CASE("motor setup supports write then verify for indexed u16 and scalar u32 families", "[motor_setup]")
{
    static motor_setup_result_t result;
    static motor_setup_step_result_t history[2];
    static const motor_setup_step_t steps[] = {
        {
            .name = "set qe max error",
            .kind = MOTOR_SETUP_STEP_KIND_WRITE_THEN_VERIFY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_SET,
                .object = MOTOR_OBJECT_QE,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_QE_INDEX_MAX_POSITION_ERROR,
                .has_value = true,
                .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as.u16 = 123U},
            },
            .verify_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_QE,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_QE_INDEX_MAX_POSITION_ERROR,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_INT_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as.u16 = 123U},
        },
        {
            .name = "set sd",
            .kind = MOTOR_SETUP_STEP_KIND_WRITE_THEN_VERIFY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_SET,
                .object = MOTOR_OBJECT_SD,
                .ack_requested = true,
                .has_value = true,
                .value = {.kind = MOTOR_SETUP_VALUE_KIND_U32, .as.u32 = 321000U},
            },
            .verify_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_SD,
                .ack_requested = true,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_INT_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_U32, .as.u32 = 321000U},
        },
    };
    static const motor_setup_plan_t plan = {
        .name = "new family write verify ok",
        .steps = steps,
        .step_count = 2U,
    };
    const motor_setup_run_cfg_t cfg = {
        .step_results = history,
        .step_results_capacity = 2U,
    };

    motor_setup_test_begin();
    memset(&result, 0, sizeof(result));
    memset(history, 0, sizeof(history));
    s_fake.submits[0] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 20U};
    s_fake.submits[1] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 21U};
    s_fake.submits[2] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 22U};
    s_fake.submits[3] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 23U};
    s_fake.results[0] = make_success_result(MOTOR_OBJECT_QE, MOTOR_EXEC_VALUE_KIND_NONE, 0);
    s_fake.results[1] = make_success_result(MOTOR_OBJECT_QE, MOTOR_EXEC_VALUE_KIND_U16, 123);
    s_fake.results[2] = make_success_result(MOTOR_OBJECT_SD, MOTOR_EXEC_VALUE_KIND_NONE, 0);
    s_fake.results[3] = make_success_result(MOTOR_OBJECT_SD, MOTOR_EXEC_VALUE_KIND_U32, 321000);

    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_run_plan(&plan, &cfg, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_STATUS_SUCCESS, result.status);
    TEST_ASSERT_TRUE(history[0].execution_attempted);
    TEST_ASSERT_TRUE(history[0].verification_attempted);
    TEST_ASSERT_TRUE(history[0].comparison_success);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_VALUE_KIND_U16, history[0].actual_value.kind);
    TEST_ASSERT_TRUE(history[1].execution_attempted);
    TEST_ASSERT_TRUE(history[1].verification_attempted);
    TEST_ASSERT_TRUE(history[1].comparison_success);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_VALUE_KIND_U32, history[1].actual_value.kind);
    TEST_ASSERT_EQUAL_UINT32(321000U, history[1].actual_value.as.u32);
    motor_setup_test_end();
}

TEST_CASE("motor setup reports verification mismatch and skips verify after exec submit failure", "[motor_setup]")
{
    static motor_setup_result_t result;
    static motor_setup_step_result_t history[2];
    static const motor_setup_step_t steps[] = {
        {
            .name = "bitrate mismatch",
            .kind = MOTOR_SETUP_STEP_KIND_WRITE_THEN_VERIFY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_SET,
                .object = MOTOR_OBJECT_PP,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_PP_INDEX_BITRATE,
                .has_value = true,
                .value = {.kind = MOTOR_SETUP_VALUE_KIND_U8, .as.u8 = MOTOR_PP_BITRATE_500K},
            },
            .verify_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_PP,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_PP_INDEX_BITRATE,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_INT_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_U8, .as.u8 = MOTOR_PP_BITRATE_500K},
        },
        {
            .name = "verify skipped after submit fail",
            .kind = MOTOR_SETUP_STEP_KIND_WRITE_THEN_VERIFY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_SET,
                .object = MOTOR_OBJECT_PP,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_PP_INDEX_GROUP_ID,
                .has_value = true,
                .value = {.kind = MOTOR_SETUP_VALUE_KIND_U8, .as.u8 = 7},
            },
            .verify_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_PP,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_PP_INDEX_GROUP_ID,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_INT_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_U8, .as.u8 = 7},
        },
    };
    static const motor_setup_plan_t mismatch_plan = {
        .name = "mismatch",
        .steps = steps,
        .step_count = 1U,
    };
    static const motor_setup_plan_t submit_fail_plan = {
        .name = "submit fail",
        .steps = &steps[1],
        .step_count = 1U,
    };
    const motor_setup_run_cfg_t cfg = {
        .step_results = history,
        .step_results_capacity = 2U,
    };

    motor_setup_test_begin();
    memset(&result, 0, sizeof(result));
    memset(history, 0, sizeof(history));
    s_fake.submits[0] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 1U};
    s_fake.submits[1] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 2U};
    s_fake.results[0] = make_success_result(MOTOR_OBJECT_PP, MOTOR_EXEC_VALUE_KIND_NONE, 0);
    s_fake.results[1] = make_success_result(MOTOR_OBJECT_PP, MOTOR_EXEC_VALUE_KIND_ENUM, MOTOR_PP_BITRATE_250K);

    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_run_plan(&mismatch_plan, &cfg, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_STATUS_FAILED, result.status);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_FAILURE_VERIFY_MISMATCH, result.failure_reason);
    TEST_ASSERT_TRUE(history[0].verification_attempted);
    TEST_ASSERT_TRUE(history[0].comparison_attempted);
    TEST_ASSERT_FALSE(history[0].comparison_success);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_VALUE_KIND_ENUM, history[0].actual_value.kind);

    memset(history, 0, sizeof(history));
    memset(&result, 0, sizeof(result));
    memset(&s_fake, 0, sizeof(s_fake));
    motor_setup_test_set_submit_fn(fake_submit);
    s_fake.submits[0] = (motor_exec_submit_result_t){.accepted = false, .code = MOTOR_EXEC_SUBMIT_BUSY, .err = ESP_ERR_INVALID_STATE, .request_id = 0U};

    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_run_plan(&submit_fail_plan, &cfg, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_FAILURE_EXEC_SUBMIT, result.failure_reason);
    TEST_ASSERT_FALSE(history[0].verification_attempted);
    motor_setup_test_end();
}

TEST_CASE("motor setup detects mismatch for new indexed and scalar families", "[motor_setup]")
{
    static motor_setup_result_t result;
    static motor_setup_step_result_t history[1];
    static const motor_setup_step_t indexed_step[] = {
        {
            .name = "ie mismatch",
            .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_IE,
                .ack_requested = true,
                .has_index = true,
                .index = MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_ENUM_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_ENUM, .as.enum_value = MOTOR_IE_STATE_ENABLE},
        },
    };
    static const motor_setup_step_t scalar_step[] = {
        {
            .name = "bl mismatch",
            .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_BL,
                .ack_requested = true,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_INT_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_U32, .as.u32 = 65535U},
        },
    };
    static const motor_setup_plan_t indexed_plan = {
        .name = "indexed mismatch",
        .steps = indexed_step,
        .step_count = 1U,
    };
    static const motor_setup_plan_t scalar_plan = {
        .name = "scalar mismatch",
        .steps = scalar_step,
        .step_count = 1U,
    };
    const motor_setup_run_cfg_t cfg = {
        .step_results = history,
        .step_results_capacity = 1U,
    };

    motor_setup_test_begin();
    memset(&result, 0, sizeof(result));
    memset(history, 0, sizeof(history));
    s_fake.submits[0] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 30U};
    s_fake.results[0] = make_success_result(MOTOR_OBJECT_IE, MOTOR_EXEC_VALUE_KIND_ENUM, MOTOR_IE_STATE_DISABLE);

    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_run_plan(&indexed_plan, &cfg, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_STATUS_FAILED, result.status);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_FAILURE_VERIFY_MISMATCH, result.failure_reason);
    TEST_ASSERT_TRUE(history[0].comparison_attempted);
    TEST_ASSERT_FALSE(history[0].comparison_success);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_VALUE_KIND_ENUM, history[0].actual_value.kind);

    memset(history, 0, sizeof(history));
    memset(&result, 0, sizeof(result));
    memset(&s_fake, 0, sizeof(s_fake));
    motor_setup_test_set_submit_fn(fake_submit);
    s_fake.submits[0] = (motor_exec_submit_result_t){.accepted = true, .code = MOTOR_EXEC_SUBMIT_OK, .err = ESP_OK, .request_id = 31U};
    s_fake.results[0] = make_success_result(MOTOR_OBJECT_BL, MOTOR_EXEC_VALUE_KIND_U32, 1234);

    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_run_plan(&scalar_plan, &cfg, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_STATUS_FAILED, result.status);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_FAILURE_VERIFY_MISMATCH, result.failure_reason);
    TEST_ASSERT_TRUE(history[0].comparison_attempted);
    TEST_ASSERT_FALSE(history[0].comparison_success);
    TEST_ASSERT_EQUAL(MOTOR_SETUP_VALUE_KIND_U32, history[0].actual_value.kind);
    TEST_ASSERT_EQUAL_UINT32(1234U, history[0].actual_value.as.u32);
    motor_setup_test_end();
}

TEST_CASE("motor setup rejects invalid new-family action shapes cleanly", "[motor_setup]")
{
    static motor_setup_result_t result;
    static const motor_setup_step_t missing_index_steps[] = {
        {
            .name = "ie missing index",
            .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_IE,
                .ack_requested = true,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_ENUM_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_ENUM, .as.enum_value = MOTOR_IE_STATE_ENABLE},
        },
    };
    static const motor_setup_step_t scalar_has_index_steps[] = {
        {
            .name = "sd invalid index",
            .kind = MOTOR_SETUP_STEP_KIND_WRITE_THEN_VERIFY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_SET,
                .object = MOTOR_OBJECT_SD,
                .ack_requested = true,
                .has_index = true,
                .index = 1U,
                .has_value = true,
                .value = {.kind = MOTOR_SETUP_VALUE_KIND_U32, .as.u32 = 100U},
            },
            .verify_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_GET,
                .object = MOTOR_OBJECT_SD,
                .ack_requested = true,
            },
            .compare_kind = MOTOR_SETUP_COMPARE_INT_EQ,
            .expected_value = {.kind = MOTOR_SETUP_VALUE_KIND_U32, .as.u32 = 100U},
        },
    };
    static const motor_setup_plan_t missing_index_plan = {
        .name = "missing index",
        .steps = missing_index_steps,
        .step_count = 1U,
    };
    static const motor_setup_plan_t scalar_has_index_plan = {
        .name = "scalar has index",
        .steps = scalar_has_index_steps,
        .step_count = 1U,
    };
    static const motor_setup_step_t sy_missing_index_steps[] = {
        {
            .name = "sy missing index",
            .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
            .exec_action = {
                .role = MOTOR_EXEC_ROLE_BRAKE,
                .node_id = 6U,
                .operation = MOTOR_EXEC_OPERATION_ACTION,
                .object = MOTOR_OBJECT_SY,
                .ack_requested = false,
            },
        },
    };
    static const motor_setup_plan_t sy_missing_index_plan = {
        .name = "sy missing index",
        .steps = sy_missing_index_steps,
        .step_count = 1U,
    };

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_setup_validate_plan(&missing_index_plan));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_setup_validate_plan(&scalar_has_index_plan));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_setup_validate_plan(&sy_missing_index_plan));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_setup_run_plan(&missing_index_plan, NULL, &result));
    TEST_ASSERT_EQUAL(MOTOR_SETUP_STATUS_INVALID_PLAN, result.status);
}

TEST_CASE("motor setup logging config helpers round trip", "[motor_setup]")
{
    motor_setup_log_cfg_t cfg;

    motor_setup_test_begin();
    cfg = motor_setup_default_log_cfg();
    TEST_ASSERT_TRUE(cfg.enabled);
    TEST_ASSERT_TRUE(cfg.log_step_success);
    cfg.log_step_success = false;
    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_set_log_cfg(&cfg));
    memset(&cfg, 0, sizeof(cfg));
    TEST_ASSERT_EQUAL(ESP_OK, motor_setup_get_log_cfg(&cfg));
    TEST_ASSERT_TRUE(cfg.enabled);
    TEST_ASSERT_FALSE(cfg.log_step_success);
    motor_setup_test_end();
}
