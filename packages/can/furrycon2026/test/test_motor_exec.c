/*
 * Responsibility:
 * Focused Unity coverage for the nonblocking motor_exec wrapper.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "unity.h"

#include "esp_err.h"
#include "motor_codec.h"
#include "motor_exec.h"
#include "motor_exec_internal.h"

typedef struct {
    motor_dispatch_result_t result;
    bool block;
    bool fill_response;
    bool fill_error;
    bool emit_notification;
    bool emit_unrelated_error;
    twai_message_t response_msg;
    twai_message_t error_msg;
    twai_message_t notification_msg;
    twai_message_t unrelated_error_msg;
    SemaphoreHandle_t enter_sem;
    SemaphoreHandle_t release_sem;
    int calls;
    bool saw_observer;
    motor_cmd_t last_cmd;
    TickType_t last_timeout_ticks;
    TickType_t last_no_ack_grace_ticks;
} fake_dispatch_t;

typedef struct {
    int call_count;
    motor_exec_result_t last_result;
    SemaphoreHandle_t done_sem;
} callback_ctx_t;

static fake_dispatch_t s_fake_dispatch;
static int s_rx_log_calls;
static motor_rx_t s_last_logged_rx;

static twai_message_t make_ext_frame(uint8_t producer_id,
                                     uint8_t cw_raw,
                                     uint8_t dlc,
                                     const uint8_t *data)
{
    twai_message_t msg = {
        .extd = 1,
        .rtr = 0,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
        .identifier = motor_codec_build_ext_id_endpoints(producer_id, 0x01U, cw_raw),
        .data_length_code = dlc,
    };

    if (data != NULL && dlc > 0U) {
        memcpy(msg.data, data, dlc);
    }

    return msg;
}

static motor_dispatch_result_t fake_dispatch_exec(const motor_cmd_t *cmd,
                                                  TickType_t timeout_ticks,
                                                  TickType_t no_ack_grace_ticks,
                                                  const motor_dispatch_observer_t *observer,
                                                  motor_rx_t *out_response,
                                                  motor_rx_t *out_related_error)
{
    s_fake_dispatch.calls++;
    s_fake_dispatch.saw_observer = observer != NULL;
    s_fake_dispatch.last_cmd = *cmd;
    s_fake_dispatch.last_timeout_ticks = timeout_ticks;
    s_fake_dispatch.last_no_ack_grace_ticks = no_ack_grace_ticks;

    if (s_fake_dispatch.enter_sem != NULL) {
        xSemaphoreGive(s_fake_dispatch.enter_sem);
    }

    if (s_fake_dispatch.block && s_fake_dispatch.release_sem != NULL) {
        TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(s_fake_dispatch.release_sem, pdMS_TO_TICKS(1000)));
    }

    if (s_fake_dispatch.emit_notification && observer != NULL && observer->on_notification != NULL) {
        motor_rx_t notification_rx;

        TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&s_fake_dispatch.notification_msg, &notification_rx));
        observer->on_notification(&notification_rx, observer->ctx);
    }

    if (s_fake_dispatch.emit_unrelated_error && observer != NULL && observer->on_error != NULL) {
        motor_rx_t error_rx;

        TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&s_fake_dispatch.unrelated_error_msg, &error_rx));
        observer->on_error(&error_rx, observer->ctx);
    }

    if (s_fake_dispatch.fill_response && out_response != NULL) {
        TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&s_fake_dispatch.response_msg, out_response));
    }

    if (s_fake_dispatch.fill_error && out_related_error != NULL) {
        TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&s_fake_dispatch.error_msg, out_related_error));
    }

    return s_fake_dispatch.result;
}

static void capture_rx_log(const motor_rx_t *rx)
{
    TEST_ASSERT_NOT_NULL(rx);

    s_rx_log_calls++;
    s_last_logged_rx = *rx;
}

static void completion_cb(const motor_exec_result_t *result, void *ctx)
{
    callback_ctx_t *callback_ctx = (callback_ctx_t *)ctx;

    TEST_ASSERT_NOT_NULL(result);
    TEST_ASSERT_NOT_NULL(callback_ctx);

    callback_ctx->call_count++;
    callback_ctx->last_result = *result;
    if (callback_ctx->done_sem != NULL) {
        xSemaphoreGive(callback_ctx->done_sem);
    }
}

static motor_exec_submit_opts_t make_opts(callback_ctx_t *ctx)
{
    motor_exec_submit_opts_t opts = {
        .on_complete = completion_cb,
        .ctx = ctx,
        .timing = {0},
    };

    opts.timing = motor_exec_default_timing();
    return opts;
}

static void reset_fake_dispatch(void)
{
    memset(&s_fake_dispatch, 0, sizeof(s_fake_dispatch));
    s_fake_dispatch.result = MOTOR_DISPATCH_RESULT_OK;
    s_rx_log_calls = 0;
    memset(&s_last_logged_rx, 0, sizeof(s_last_logged_rx));
}

static void motor_exec_test_begin(void)
{
    motor_exec_test_reset_state();
    reset_fake_dispatch();
    motor_exec_test_set_dispatch_fn(fake_dispatch_exec);
    motor_exec_test_set_rx_log_fn(capture_rx_log);
}

static void motor_exec_test_end(void)
{
    if (s_fake_dispatch.enter_sem != NULL) {
        vSemaphoreDelete(s_fake_dispatch.enter_sem);
        s_fake_dispatch.enter_sem = NULL;
    }

    if (s_fake_dispatch.release_sem != NULL) {
        vSemaphoreDelete(s_fake_dispatch.release_sem);
        s_fake_dispatch.release_sem = NULL;
    }

    motor_exec_test_reset_state();
}

TEST_CASE("motor exec accepts valid submit and returns request id", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_opts_t opts;
    motor_exec_submit_result_t submit;

    motor_exec_test_begin();
    ctx.done_sem = xSemaphoreCreateBinary();
    opts = make_opts(&ctx);

    static const uint8_t data[] = {MOTOR_PP_INDEX_BITRATE, MOTOR_PP_BITRATE_500K};
    s_fake_dispatch.fill_response = true;
    s_fake_dispatch.response_msg = make_ext_frame(6U, 0x01U, 2U, data);

    submit = motor_exec_brake_pp_get(6U, true, MOTOR_PP_INDEX_BITRATE, &opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_SUBMIT_OK, submit.code);
    TEST_ASSERT_EQUAL(ESP_OK, submit.err);
    TEST_ASSERT_NOT_EQUAL(0U, submit.request_id);
    TEST_ASSERT_TRUE(motor_exec_has_pending());
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(1, ctx.call_count);
    TEST_ASSERT_EQUAL_UINT32(submit.request_id, ctx.last_result.request_id);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, ctx.last_result.status);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_ROLE_BRAKE, ctx.last_result.role);
    TEST_ASSERT_EQUAL_UINT8(6U, ctx.last_result.node_id);
    TEST_ASSERT_TRUE(ctx.last_result.ack_expected);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_OPERATION_GET, ctx.last_result.operation);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_PP, ctx.last_result.object);
    TEST_ASSERT_EQUAL_STRING("PP", ctx.last_result.command_family_symbol);
    TEST_ASSERT_TRUE(ctx.last_result.has_parameter_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_PP_INDEX_BITRATE, ctx.last_result.parameter_index);
    TEST_ASSERT_EQUAL_STRING("CAN bitrate", ctx.last_result.parameter_name);
    TEST_ASSERT_EQUAL_STRING("", ctx.last_result.parameter_value_summary);
    TEST_ASSERT_EQUAL_STRING("TX node=6 PP[5] GET (CAN bitrate)", ctx.last_result.command_summary);
    TEST_ASSERT_TRUE(ctx.last_result.has_response_value_summary);
    TEST_ASSERT_EQUAL_STRING("500000 bps", ctx.last_result.response_value_summary);
    TEST_ASSERT_EQUAL_STRING("RX node=6 ACK PP[5] CAN bitrate = 500000 bps", ctx.last_result.completion_summary);
    TEST_ASSERT_FALSE(motor_exec_has_pending());

    vSemaphoreDelete(ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec rejects second submit while one request is pending", "[motor_exec]")
{
    callback_ctx_t first_ctx = {0};
    callback_ctx_t second_ctx = {0};
    motor_exec_submit_opts_t first_opts;
    motor_exec_submit_opts_t second_opts;
    motor_exec_submit_result_t first_submit;
    motor_exec_submit_result_t second_submit;

    motor_exec_test_begin();
    first_ctx.done_sem = xSemaphoreCreateBinary();
    second_ctx.done_sem = xSemaphoreCreateBinary();
    first_opts = make_opts(&first_ctx);
    second_opts = make_opts(&second_ctx);

    s_fake_dispatch.block = true;
    s_fake_dispatch.enter_sem = xSemaphoreCreateBinary();
    s_fake_dispatch.release_sem = xSemaphoreCreateBinary();

    first_submit = motor_exec_brake_bg_begin(6U, true, &first_opts);
    TEST_ASSERT_TRUE(first_submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(s_fake_dispatch.enter_sem, pdMS_TO_TICKS(1000)));

    second_submit = motor_exec_brake_st_stop(6U, true, &second_opts);
    TEST_ASSERT_FALSE(second_submit.accepted);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_SUBMIT_BUSY, second_submit.code);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, second_submit.err);
    TEST_ASSERT_EQUAL(0, second_ctx.call_count);

    xSemaphoreGive(s_fake_dispatch.release_sem);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(first_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(1, first_ctx.call_count);

    vSemaphoreDelete(first_ctx.done_sem);
    vSemaphoreDelete(second_ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec rejects invalid arguments immediately", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_result_t submit;

    motor_exec_test_begin();
    submit = motor_exec_brake_pa_set(6U, true, 1234, NULL);
    TEST_ASSERT_FALSE(submit.accepted);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_SUBMIT_INVALID_ARGUMENT, submit.code);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, submit.err);
    TEST_ASSERT_EQUAL(0, ctx.call_count);
    motor_exec_test_end();
}

TEST_CASE("motor exec surfaces command-builder validation failure at submit time", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_opts_t opts;
    motor_exec_submit_result_t submit;

    motor_exec_test_begin();
    ctx.done_sem = xSemaphoreCreateBinary();
    opts = make_opts(&ctx);

    submit = motor_exec_brake_mt_set_u16(6U, true, MOTOR_MT_INDEX_WORKING_CURRENT, 1000U, &opts);
    TEST_ASSERT_FALSE(submit.accepted);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_SUBMIT_INVALID_ARGUMENT, submit.code);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, submit.err);
    TEST_ASSERT_EQUAL(0, ctx.call_count);

    vSemaphoreDelete(ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec submits PT and PV wrappers with typed summaries", "[motor_exec]")
{
    callback_ctx_t pv_ctx = {0};
    callback_ctx_t pt_ctx = {0};
    motor_exec_submit_opts_t pv_opts;
    motor_exec_submit_opts_t pt_opts;
    motor_exec_submit_result_t submit;

    motor_exec_test_begin();

    pv_ctx.done_sem = xSemaphoreCreateBinary();
    pv_opts = make_opts(&pv_ctx);
    s_fake_dispatch.fill_response = true;
    s_fake_dispatch.response_msg = make_ext_frame(6U, 0x23, 8U,
                                                  (const uint8_t[]){0x21, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF});

    submit = motor_exec_brake_pv_get(6U, true, &pv_opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(pv_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_PV, s_fake_dispatch.last_cmd.object);
    TEST_ASSERT_EQUAL_UINT8(0U, s_fake_dispatch.last_cmd.msg.data_length_code);
    TEST_ASSERT_EQUAL_STRING("PV", pv_ctx.last_result.command_family_symbol);
    TEST_ASSERT_EQUAL_STRING("PVT row index", pv_ctx.last_result.parameter_name);
    TEST_ASSERT_EQUAL_STRING("RX node=6 ACK PV PVT row index = 33", pv_ctx.last_result.completion_summary);

    reset_fake_dispatch();
    pt_ctx.done_sem = xSemaphoreCreateBinary();
    pt_opts = make_opts(&pt_ctx);
    s_fake_dispatch.fill_response = true;
    s_fake_dispatch.response_msg = make_ext_frame(6U, 0x24, 8U,
                                                  (const uint8_t[]){0x0A, 0x01, 0xA0, 0x86, 0x01, 0x00, 0x12, 0x34});

    submit = motor_exec_brake_pt_set(6U, true, 266U, 100000, &pt_opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(pt_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_PT, s_fake_dispatch.last_cmd.object);
    TEST_ASSERT_EQUAL_UINT8(8U, s_fake_dispatch.last_cmd.msg.data_length_code);
    TEST_ASSERT_EQUAL_UINT16(266U, s_fake_dispatch.last_cmd.index);
    TEST_ASSERT_EQUAL_STRING("PT", pt_ctx.last_result.command_family_symbol);
    TEST_ASSERT_TRUE(pt_ctx.last_result.has_parameter_index);
    TEST_ASSERT_EQUAL_UINT16(266U, pt_ctx.last_result.parameter_index);
    TEST_ASSERT_EQUAL_STRING("PT queued position", pt_ctx.last_result.parameter_name);
    TEST_ASSERT_EQUAL_STRING("100000 pulse", pt_ctx.last_result.response_value_summary);
    TEST_ASSERT_EQUAL_STRING("RX node=6 ACK PT[266] PT queued position = 100000 pulse",
                             pt_ctx.last_result.completion_summary);

    vSemaphoreDelete(pv_ctx.done_sem);
    vSemaphoreDelete(pt_ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec validates PT write then readback through wrappers and parsed ACKs", "[motor_exec]")
{
    callback_ctx_t set_ctx = {0};
    callback_ctx_t get_ctx = {0};
    motor_exec_submit_opts_t set_opts;
    motor_exec_submit_opts_t get_opts;
    motor_exec_submit_result_t submit;
    motor_rx_t parsed_response;
    const uint16_t row_index = 266U;
    const int32_t queued_position = 100000;

    motor_exec_test_begin();

    set_ctx.done_sem = xSemaphoreCreateBinary();
    set_opts = make_opts(&set_ctx);
    s_fake_dispatch.fill_response = true;
    s_fake_dispatch.response_msg = make_ext_frame(6U, 0x24, 8U,
                                                  (const uint8_t[]){0x0A, 0x01, 0xA0, 0x86, 0x01, 0x00, 0x12, 0x34});

    submit = motor_exec_brake_pt_set(6U, true, row_index, queued_position, &set_opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(set_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, set_ctx.last_result.status);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_PT, s_fake_dispatch.last_cmd.object);
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&s_fake_dispatch.response_msg, &parsed_response));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&parsed_response, &s_fake_dispatch.last_cmd));
    TEST_ASSERT_TRUE(set_ctx.last_result.has_response_value);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_VALUE_KIND_I32, set_ctx.last_result.response_value.kind);
    TEST_ASSERT_EQUAL_INT32(queued_position, (int32_t)set_ctx.last_result.response_value.raw_number);

    reset_fake_dispatch();
    get_ctx.done_sem = xSemaphoreCreateBinary();
    get_opts = make_opts(&get_ctx);
    s_fake_dispatch.fill_response = true;
    s_fake_dispatch.response_msg = make_ext_frame(6U, 0x24, 8U,
                                                  (const uint8_t[]){0x0A, 0x01, 0xA0, 0x86, 0x01, 0x00, 0x56, 0x78});

    submit = motor_exec_brake_pt_get(6U, true, row_index, &get_opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(get_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, get_ctx.last_result.status);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_PT, s_fake_dispatch.last_cmd.object);
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&s_fake_dispatch.response_msg, &parsed_response));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&parsed_response, &s_fake_dispatch.last_cmd));
    TEST_ASSERT_TRUE(get_ctx.last_result.has_response_value);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_VALUE_KIND_I32, get_ctx.last_result.response_value.kind);
    TEST_ASSERT_EQUAL_INT32(queued_position, (int32_t)get_ctx.last_result.response_value.raw_number);
    TEST_ASSERT_EQUAL_STRING("RX node=6 ACK PT[266] PT queued position = 100000 pulse",
                             get_ctx.last_result.completion_summary);

    vSemaphoreDelete(set_ctx.done_sem);
    vSemaphoreDelete(get_ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec accepts PT SET ACKs that echo a different row with the requested position", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_opts_t opts;
    motor_exec_submit_result_t submit;

    motor_exec_test_begin();

    ctx.done_sem = xSemaphoreCreateBinary();
    opts = make_opts(&ctx);
    s_fake_dispatch.fill_response = true;
    s_fake_dispatch.response_msg = make_ext_frame(6U, 0x24, 8U,
                                                  (const uint8_t[]){0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});

    submit = motor_exec_brake_pt_set(6U, true, 0U, 0, &opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, ctx.last_result.status);
    TEST_ASSERT_EQUAL_STRING("RX node=6 ACK PT[1] PT queued position = 0 pulse",
                             ctx.last_result.completion_summary);

    vSemaphoreDelete(ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec callback fires only on terminal completion", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_opts_t opts;
    motor_exec_submit_result_t submit;

    motor_exec_test_begin();
    ctx.done_sem = xSemaphoreCreateBinary();
    opts = make_opts(&ctx);
    s_fake_dispatch.block = true;
    s_fake_dispatch.enter_sem = xSemaphoreCreateBinary();
    s_fake_dispatch.release_sem = xSemaphoreCreateBinary();

    submit = motor_exec_brake_pa_set(6U, true, 1000, &opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(s_fake_dispatch.enter_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(0, ctx.call_count);

    xSemaphoreGive(s_fake_dispatch.release_sem);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(1, ctx.call_count);

    vSemaphoreDelete(ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec maps timeout completion semantically", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_opts_t opts;
    motor_exec_submit_result_t submit;

    motor_exec_test_begin();
    ctx.done_sem = xSemaphoreCreateBinary();
    opts = make_opts(&ctx);
    s_fake_dispatch.result = MOTOR_DISPATCH_RESULT_TIMEOUT;

    submit = motor_exec_brake_pa_get(6U, true, &opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_TIMEOUT, ctx.last_result.status);
    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_TIMEOUT, ctx.last_result.dispatch_result);
    TEST_ASSERT_FALSE(ctx.last_result.has_response_value_summary);
    TEST_ASSERT_NOT_NULL(strstr(ctx.last_result.completion_summary, "timed out"));

    vSemaphoreDelete(ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec treats ER GET history response as success", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_opts_t opts;
    motor_exec_submit_result_t submit;
    uint8_t error_data[] = {
        MOTOR_ER_INDEX_HISTORY_1,
        0x11U,
        0x8FU,
        0x00U,
        0x00U,
        0x00U
    };

    motor_exec_test_begin();
    ctx.done_sem = xSemaphoreCreateBinary();
    opts = make_opts(&ctx);
    s_fake_dispatch.result = MOTOR_DISPATCH_RESULT_OK;
    s_fake_dispatch.fill_response = true;
    s_fake_dispatch.response_msg = make_ext_frame(6U, 0x0FU, 6U, error_data);

    submit = motor_exec_brake_er_get(6U, true, MOTOR_ER_INDEX_HISTORY_1, &opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, ctx.last_result.status);
    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_OK, ctx.last_result.dispatch_result);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_OPERATION_GET, ctx.last_result.operation);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_ER, ctx.last_result.object);
    TEST_ASSERT_FALSE(ctx.last_result.remote_error.valid);

    vSemaphoreDelete(ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec treats ER clear-all zero response as success", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_opts_t opts;
    motor_exec_submit_result_t submit;
    uint8_t clear_data[] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};

    motor_exec_test_begin();
    ctx.done_sem = xSemaphoreCreateBinary();
    opts = make_opts(&ctx);
    s_fake_dispatch.result = MOTOR_DISPATCH_RESULT_OK;
    s_fake_dispatch.fill_response = true;
    s_fake_dispatch.response_msg = make_ext_frame(6U, 0x0FU, 6U, clear_data);

    submit = motor_exec_brake_er_clear_all(6U, true, &opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, ctx.last_result.status);
    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_OK, ctx.last_result.dispatch_result);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_OPERATION_CLEAR, ctx.last_result.operation);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_ER, ctx.last_result.object);
    TEST_ASSERT_FALSE(ctx.last_result.remote_error.valid);
    TEST_ASSERT_NOT_NULL(strstr(ctx.last_result.completion_summary, "Clear all errors"));

    vSemaphoreDelete(ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec maps remote error completion semantically", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_opts_t opts;
    motor_exec_submit_result_t submit;
    uint8_t error_data[] = {
        MOTOR_ER_INDEX_CLEAR_ALL,
        0x3EU,
        0x00U,
        0x00U,
        0x00U,
        0x00U
    };

    motor_exec_test_begin();
    ctx.done_sem = xSemaphoreCreateBinary();
    opts = make_opts(&ctx);
    s_fake_dispatch.result = MOTOR_DISPATCH_RESULT_REMOTE_ERROR;
    s_fake_dispatch.fill_error = true;
    error_data[2] = motor_codec_compose_cw(0x16U, true);
    s_fake_dispatch.error_msg = make_ext_frame(6U, 0x0FU, 6U, error_data);

    submit = motor_exec_brake_bg_begin(6U, true, &opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_REMOTE_ERROR, ctx.last_result.status);
    TEST_ASSERT_TRUE(ctx.last_result.remote_error.valid);
    TEST_ASSERT_EQUAL_HEX8(0x3E, ctx.last_result.remote_error.code);
    TEST_ASSERT_EQUAL_STRING("BG", ctx.last_result.remote_error.related_family_symbol);
    TEST_ASSERT_TRUE(ctx.last_result.remote_error.has_related_index);
    TEST_ASSERT_EQUAL_UINT16(0U, ctx.last_result.remote_error.related_index);
    TEST_ASSERT_NOT_NULL(strstr(ctx.last_result.completion_summary, "BG is not allowed"));

    vSemaphoreDelete(ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec maps transport error completion semantically", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_opts_t opts;
    motor_exec_submit_result_t submit;

    motor_exec_test_begin();
    ctx.done_sem = xSemaphoreCreateBinary();
    opts = make_opts(&ctx);
    s_fake_dispatch.result = MOTOR_DISPATCH_RESULT_TRANSPORT_ERROR;

    submit = motor_exec_brake_mo_set(6U, true, MOTOR_MO_STATE_ENABLE, &opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_TRANSPORT_ERROR, ctx.last_result.status);
    TEST_ASSERT_NOT_NULL(strstr(ctx.last_result.completion_summary, "transport error"));

    vSemaphoreDelete(ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec maps no-ack success without pretending there was an ACK", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_opts_t opts;
    motor_exec_submit_result_t submit;

    motor_exec_test_begin();
    ctx.done_sem = xSemaphoreCreateBinary();
    opts = make_opts(&ctx);

    submit = motor_exec_brake_bg_begin(6U, false, &opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, ctx.last_result.status);
    TEST_ASSERT_FALSE(ctx.last_result.ack_expected);
    TEST_ASSERT_FALSE(ctx.last_result.has_response_value_summary);
    TEST_ASSERT_NOT_NULL(strstr(ctx.last_result.completion_summary, "without ACK"));

    vSemaphoreDelete(ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec exposes semantic command meaning for set and clear operations", "[motor_exec]")
{
    callback_ctx_t set_ctx = {0};
    callback_ctx_t clear_ctx = {0};
    motor_exec_submit_opts_t set_opts;
    motor_exec_submit_opts_t clear_opts;
    motor_exec_submit_result_t submit;

    motor_exec_test_begin();
    set_ctx.done_sem = xSemaphoreCreateBinary();
    clear_ctx.done_sem = xSemaphoreCreateBinary();
    set_opts = make_opts(&set_ctx);
    clear_opts = make_opts(&clear_ctx);

    submit = motor_exec_brake_pa_set(6U, true, 4000, &set_opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(set_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_OPERATION_SET, set_ctx.last_result.operation);
    TEST_ASSERT_EQUAL_STRING("PA", set_ctx.last_result.command_family_symbol);
    TEST_ASSERT_EQUAL_STRING("Absolute position", set_ctx.last_result.parameter_name);
    TEST_ASSERT_EQUAL_STRING("4000 pulse", set_ctx.last_result.parameter_value_summary);

    submit = motor_exec_brake_er_clear_all(6U, true, &clear_opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(clear_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_OPERATION_CLEAR, clear_ctx.last_result.operation);
    TEST_ASSERT_EQUAL_STRING("Clear all errors", clear_ctx.last_result.parameter_name);

    vSemaphoreDelete(set_ctx.done_sem);
    vSemaphoreDelete(clear_ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec accepts representative new indexed and scalar commands", "[motor_exec]")
{
    callback_ctx_t ie_ctx = {0};
    callback_ctx_t sd_ctx = {0};
    motor_exec_submit_opts_t ie_opts;
    motor_exec_submit_opts_t sd_opts;
    motor_exec_submit_result_t submit;
    uint8_t sd_data[4];

    motor_exec_test_begin();
    ie_ctx.done_sem = xSemaphoreCreateBinary();
    sd_ctx.done_sem = xSemaphoreCreateBinary();
    ie_opts = make_opts(&ie_ctx);
    sd_opts = make_opts(&sd_ctx);

    s_fake_dispatch.fill_response = true;
    s_fake_dispatch.response_msg = make_ext_frame(6U, 0x07U, 3U,
                                                  (const uint8_t[]){MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION, 0x01, 0x00});
    submit = motor_exec_brake_ie_get(6U, true, MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION, &ie_opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ie_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, ie_ctx.last_result.status);
    TEST_ASSERT_EQUAL_STRING("IE", ie_ctx.last_result.command_family_symbol);
    TEST_ASSERT_EQUAL_STRING("FIFO empty notification", ie_ctx.last_result.parameter_name);
    TEST_ASSERT_TRUE(ie_ctx.last_result.has_response_value);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_VALUE_KIND_ENUM, ie_ctx.last_result.response_value.kind);
    TEST_ASSERT_EQUAL_STRING("enabled", ie_ctx.last_result.response_value.text);
    TEST_ASSERT_EQUAL_STRING("RX node=6 ACK IE[10] FIFO empty notification = enabled", ie_ctx.last_result.completion_summary);

    motor_codec_pack_u32_le(sd_data, 321000U);
    s_fake_dispatch.fill_response = true;
    s_fake_dispatch.response_msg = make_ext_frame(6U, 0x1CU, 4U, sd_data);
    submit = motor_exec_brake_sd_set_u32(6U, true, 321000U, &sd_opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(sd_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, sd_ctx.last_result.status);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_OPERATION_SET, sd_ctx.last_result.operation);
    TEST_ASSERT_EQUAL_STRING("SD", sd_ctx.last_result.command_family_symbol);
    TEST_ASSERT_EQUAL_STRING("Stop deceleration", sd_ctx.last_result.parameter_name);
    TEST_ASSERT_EQUAL_STRING("321000 pulse/sec^2", sd_ctx.last_result.parameter_value_summary);
    TEST_ASSERT_TRUE(sd_ctx.last_result.has_response_value);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_VALUE_KIND_U32, sd_ctx.last_result.response_value.kind);
    TEST_ASSERT_EQUAL(321000, sd_ctx.last_result.response_value.raw_number);

    vSemaphoreDelete(ie_ctx.done_sem);
    vSemaphoreDelete(sd_ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec accepts SY no-ACK action wrappers with reboot timing", "[motor_exec]")
{
    callback_ctx_t reboot_ctx = {0};
    callback_ctx_t restore_ctx = {0};
    motor_exec_submit_opts_t reboot_opts;
    motor_exec_submit_opts_t restore_opts;
    motor_exec_submit_result_t submit;

    motor_exec_test_begin();
    reboot_ctx.done_sem = xSemaphoreCreateBinary();
    restore_ctx.done_sem = xSemaphoreCreateBinary();
    reboot_opts = make_opts(&reboot_ctx);
    restore_opts = make_opts(&restore_ctx);
    reboot_opts.timing = motor_exec_reboot_timing();
    restore_opts.timing = motor_exec_reboot_timing();

    submit = motor_exec_brake_sy_reboot(6U, &reboot_opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(reboot_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, reboot_ctx.last_result.status);
    TEST_ASSERT_FALSE(reboot_ctx.last_result.ack_expected);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_OPERATION_ACTION, reboot_ctx.last_result.operation);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_SY, reboot_ctx.last_result.object);
    TEST_ASSERT_EQUAL_STRING("SY", reboot_ctx.last_result.command_family_symbol);
    TEST_ASSERT_TRUE(reboot_ctx.last_result.has_parameter_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_SY_OP_REBOOT, reboot_ctx.last_result.parameter_index);
    TEST_ASSERT_EQUAL_STRING("Reboot", reboot_ctx.last_result.parameter_name);
    TEST_ASSERT_EQUAL_STRING("TX node=6 SY[1] ACTION (Reboot)", reboot_ctx.last_result.command_summary);
    TEST_ASSERT_FALSE(reboot_ctx.last_result.has_response_value);
    TEST_ASSERT_FALSE(reboot_ctx.last_result.has_response_value_summary);
    TEST_ASSERT_EQUAL_STRING("completed successfully without ACK; no related remote error was observed",
                             reboot_ctx.last_result.completion_summary);
    TEST_ASSERT_EQUAL(pdMS_TO_TICKS(1000), s_fake_dispatch.last_timeout_ticks);
    TEST_ASSERT_EQUAL(pdMS_TO_TICKS(50), s_fake_dispatch.last_no_ack_grace_ticks);

    submit = motor_exec_brake_sy_restore_defaults(6U, &restore_opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(restore_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, restore_ctx.last_result.status);
    TEST_ASSERT_FALSE(restore_ctx.last_result.ack_expected);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_OPERATION_ACTION, restore_ctx.last_result.operation);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_SY, restore_ctx.last_result.object);
    TEST_ASSERT_EQUAL_STRING("SY", restore_ctx.last_result.command_family_symbol);
    TEST_ASSERT_TRUE(restore_ctx.last_result.has_parameter_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_SY_OP_RESTORE_FACTORY_DEFAULTS, restore_ctx.last_result.parameter_index);
    TEST_ASSERT_EQUAL_STRING("Restore factory defaults", restore_ctx.last_result.parameter_name);
    TEST_ASSERT_EQUAL_STRING("TX node=6 SY[2] ACTION (Restore factory defaults)",
                             restore_ctx.last_result.command_summary);

    vSemaphoreDelete(reboot_ctx.done_sem);
    vSemaphoreDelete(restore_ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec captures representative typed responses across numeric and text families", "[motor_exec]")
{
    callback_ctx_t lm_ctx = {0};
    callback_ctx_t mp_ctx = {0};
    motor_exec_submit_opts_t lm_opts;
    motor_exec_submit_opts_t mp_opts;
    motor_exec_submit_result_t submit;
    uint8_t lm_data[5] = {MOTOR_LM_INDEX_MAX_ACCEL_DECEL};

    motor_exec_test_begin();
    lm_ctx.done_sem = xSemaphoreCreateBinary();
    mp_ctx.done_sem = xSemaphoreCreateBinary();
    lm_opts = make_opts(&lm_ctx);
    mp_opts = make_opts(&mp_ctx);

    motor_codec_pack_i32_le(&lm_data[1], 3333);
    s_fake_dispatch.fill_response = true;
    s_fake_dispatch.response_msg = make_ext_frame(6U, 0x2CU, 5U, lm_data);
    submit = motor_exec_brake_lm_get(6U, true, MOTOR_LM_INDEX_MAX_ACCEL_DECEL, &lm_opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(lm_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, lm_ctx.last_result.status);
    TEST_ASSERT_TRUE(lm_ctx.last_result.has_response_value);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_VALUE_KIND_I32, lm_ctx.last_result.response_value.kind);
    TEST_ASSERT_EQUAL(3333, lm_ctx.last_result.response_value.raw_number);
    TEST_ASSERT_EQUAL_STRING("3333 pulse/sec^2", lm_ctx.last_result.response_value_summary);

    s_fake_dispatch.fill_response = true;
    s_fake_dispatch.response_msg = make_ext_frame(6U, 0x22U, 3U,
                                                  (const uint8_t[]){MOTOR_MP_INDEX_PT_MOTION_TIME, 0x00, 0x00});
    submit = motor_exec_brake_mp_get(6U, true, MOTOR_MP_INDEX_PT_MOTION_TIME, &mp_opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(mp_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, mp_ctx.last_result.status);
    TEST_ASSERT_TRUE(mp_ctx.last_result.has_response_value);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_VALUE_KIND_TEXT, mp_ctx.last_result.response_value.kind);
    TEST_ASSERT_EQUAL_STRING("0", mp_ctx.last_result.response_value.text);
    TEST_ASSERT_EQUAL_STRING("0", mp_ctx.last_result.response_value_summary);
    TEST_ASSERT_EQUAL_STRING("RX node=6 ACK MP[4] PT motion time = 0 (zero may have a special meaning)", mp_ctx.last_result.completion_summary);

    vSemaphoreDelete(lm_ctx.done_sem);
    vSemaphoreDelete(mp_ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec surfaces new builder validation failures", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_opts_t opts;
    motor_exec_submit_result_t submit;

    motor_exec_test_begin();
    ctx.done_sem = xSemaphoreCreateBinary();
    opts = make_opts(&ctx);

    submit = motor_exec_brake_ie_set_u16(6U, true, MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION, 2U, &opts);
    TEST_ASSERT_FALSE(submit.accepted);
    TEST_ASSERT_EQUAL(MOTOR_EXEC_SUBMIT_INVALID_ARGUMENT, submit.code);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, submit.err);

    vSemaphoreDelete(ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec logging controls are configurable and split by domain", "[motor_exec]")
{
    motor_exec_log_cfg_t cfg;

    motor_exec_test_begin();
    cfg = motor_exec_default_log_cfg();
    TEST_ASSERT_TRUE(cfg.enabled);
    TEST_ASSERT_TRUE(cfg.setup_enabled);
    TEST_ASSERT_TRUE(cfg.motion_enabled);
    TEST_ASSERT_TRUE(cfg.log_notifications);
    TEST_ASSERT_TRUE(cfg.log_unrelated_errors);

    cfg.enabled = true;
    cfg.setup_enabled = false;
    cfg.motion_enabled = true;
    cfg.log_notifications = false;
    cfg.log_unrelated_errors = true;
    TEST_ASSERT_EQUAL(ESP_OK, motor_exec_set_log_cfg(&cfg));
    memset(&cfg, 0, sizeof(cfg));
    TEST_ASSERT_EQUAL(ESP_OK, motor_exec_get_log_cfg(&cfg));
    TEST_ASSERT_TRUE(cfg.enabled);
    TEST_ASSERT_FALSE(cfg.setup_enabled);
    TEST_ASSERT_TRUE(cfg.motion_enabled);
    TEST_ASSERT_FALSE(cfg.log_notifications);
    TEST_ASSERT_TRUE(cfg.log_unrelated_errors);
    motor_exec_test_end();
}

TEST_CASE("motor exec logs side-traffic notifications semantically through the dispatch observer", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_opts_t opts;
    motor_exec_submit_result_t submit;

    motor_exec_test_begin();
    ctx.done_sem = xSemaphoreCreateBinary();
    opts = make_opts(&ctx);
    s_fake_dispatch.emit_notification = true;
    s_fake_dispatch.notification_msg = make_ext_frame(0x33U, 0x5AU, 2U,
                                                      (const uint8_t[]){0x44U, 0x02U});

    submit = motor_exec_brake_bg_begin(6U, false, &opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_TRUE(s_fake_dispatch.saw_observer);
    TEST_ASSERT_EQUAL(1, s_rx_log_calls);
    TEST_ASSERT_TRUE(motor_rx_is_notification(&s_last_logged_rx));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, ctx.last_result.status);
    TEST_ASSERT_FALSE(ctx.last_result.ack_expected);

    vSemaphoreDelete(ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec logs unrelated motor-side errors semantically through the dispatch observer", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_opts_t opts;
    motor_exec_submit_result_t submit;
    uint8_t error_data[] = {
        MOTOR_ER_INDEX_HISTORY_2, 0x77U, 0x26U, MOTOR_PP_INDEX_NODE_ID, 0x00U, 0x00U
    };

    motor_exec_test_begin();
    ctx.done_sem = xSemaphoreCreateBinary();
    opts = make_opts(&ctx);
    s_fake_dispatch.emit_unrelated_error = true;
    s_fake_dispatch.unrelated_error_msg = make_ext_frame(0x05U, 0x0FU, 6U, error_data);

    submit = motor_exec_brake_bg_begin(6U, false, &opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_TRUE(s_fake_dispatch.saw_observer);
    TEST_ASSERT_EQUAL(1, s_rx_log_calls);
    TEST_ASSERT_TRUE(motor_rx_is_error(&s_last_logged_rx));
    TEST_ASSERT_EQUAL(MOTOR_EXEC_STATUS_SUCCESS, ctx.last_result.status);
    TEST_ASSERT_FALSE(ctx.last_result.remote_error.valid);

    vSemaphoreDelete(ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec side-traffic log flags can suppress notification and unrelated error logging", "[motor_exec]")
{
    callback_ctx_t notify_ctx = {0};
    callback_ctx_t error_ctx = {0};
    motor_exec_submit_opts_t notify_opts;
    motor_exec_submit_opts_t error_opts;
    motor_exec_submit_result_t submit;
    motor_exec_log_cfg_t cfg;
    uint8_t error_data[] = {
        MOTOR_ER_INDEX_HISTORY_2, 0x77U, 0x26U, MOTOR_PP_INDEX_NODE_ID, 0x00U, 0x00U
    };

    motor_exec_test_begin();
    notify_ctx.done_sem = xSemaphoreCreateBinary();
    error_ctx.done_sem = xSemaphoreCreateBinary();
    notify_opts = make_opts(&notify_ctx);
    error_opts = make_opts(&error_ctx);

    cfg = motor_exec_default_log_cfg();
    cfg.log_notifications = false;
    cfg.log_unrelated_errors = false;
    TEST_ASSERT_EQUAL(ESP_OK, motor_exec_set_log_cfg(&cfg));

    s_fake_dispatch.emit_notification = true;
    s_fake_dispatch.notification_msg = make_ext_frame(0x33U, 0x5AU, 2U,
                                                      (const uint8_t[]){0x44U, 0x02U});
    submit = motor_exec_brake_bg_begin(6U, false, &notify_opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(notify_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(0, s_rx_log_calls);

    reset_fake_dispatch();
    cfg = motor_exec_default_log_cfg();
    cfg.log_notifications = true;
    cfg.log_unrelated_errors = false;
    TEST_ASSERT_EQUAL(ESP_OK, motor_exec_set_log_cfg(&cfg));
    s_fake_dispatch.emit_unrelated_error = true;
    s_fake_dispatch.unrelated_error_msg = make_ext_frame(0x05U, 0x0FU, 6U, error_data);
    submit = motor_exec_brake_bg_begin(6U, false, &error_opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(error_ctx.done_sem, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(0, s_rx_log_calls);

    vSemaphoreDelete(notify_ctx.done_sem);
    vSemaphoreDelete(error_ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec copied callback results remain usable after callback return", "[motor_exec]")
{
    callback_ctx_t ctx = {0};
    motor_exec_submit_opts_t opts;
    motor_exec_submit_result_t submit;

    motor_exec_test_begin();
    ctx.done_sem = xSemaphoreCreateBinary();
    opts = make_opts(&ctx);

    submit = motor_exec_brake_pa_set(6U, true, 4321, &opts);
    TEST_ASSERT_TRUE(submit.accepted);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.done_sem, pdMS_TO_TICKS(1000)));

    TEST_ASSERT_EQUAL_STRING("PA", ctx.last_result.command_family_symbol);
    TEST_ASSERT_EQUAL_STRING("Absolute position", ctx.last_result.parameter_name);
    TEST_ASSERT_EQUAL_STRING("4321 pulse", ctx.last_result.parameter_value_summary);
    TEST_ASSERT_NOT_NULL(strstr(ctx.last_result.command_summary, "PA"));

    vSemaphoreDelete(ctx.done_sem);
    motor_exec_test_end();
}

TEST_CASE("motor exec default timing matches agreed policy", "[motor_exec]")
{
    motor_exec_timing_t timing = motor_exec_default_timing();
    motor_exec_timing_t reboot_timing = motor_exec_reboot_timing();

    TEST_ASSERT_EQUAL(pdMS_TO_TICKS(100), timing.timeout_ticks);
    TEST_ASSERT_EQUAL(pdMS_TO_TICKS(50), timing.no_ack_grace_ticks);
    TEST_ASSERT_EQUAL(pdMS_TO_TICKS(1000), reboot_timing.timeout_ticks);
    TEST_ASSERT_EQUAL(pdMS_TO_TICKS(50), reboot_timing.no_ack_grace_ticks);
}
