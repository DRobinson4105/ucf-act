/*
 * Responsibility:
 * Unity coverage for the synchronous motor dispatch layer.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "unity.h"

#include "esp_err.h"
#include "motor_codec.h"
#include "motor_dispatch.h"
#include "motor_dispatch_priv.h"
#include "motor_protocol.h"
#include "twai_port.h"

typedef struct {
    int install_calls;
    int uninstall_calls;
    int start_calls;
    int stop_calls;
    int transmit_calls;
    int receive_calls;

    esp_err_t install_rc;
    esp_err_t uninstall_rc;
    esp_err_t start_rc;
    esp_err_t stop_rc;
    esp_err_t transmit_rc;
    esp_err_t receive_fallback_rc;

    twai_message_t last_tx_msg;
    TickType_t last_tx_timeout;
    TickType_t last_rx_timeout;

    twai_message_t rx_queue[8];
    size_t rx_queue_len;
    size_t rx_queue_index;

    SemaphoreHandle_t receive_enter_sem;
    SemaphoreHandle_t receive_release_sem;
} fake_driver_t;

typedef struct {
    int notification_calls;
    int error_calls;
    motor_rx_kind_t last_kind;
    uint8_t last_base_code;
    motor_object_t last_object;
} observer_ctx_t;

typedef struct {
    motor_cmd_t cmd;
    TickType_t timeout_ticks;
    TickType_t no_ack_grace_ticks;
    motor_dispatch_result_t result;
    SemaphoreHandle_t done_sem;
} exec_task_ctx_t;

static fake_driver_t s_fake;

static twai_port_cfg_t make_cfg(void)
{
    twai_port_cfg_t cfg = {
        .tx_gpio = 21,
        .rx_gpio = 20,
        .bitrate = 500000,
        .alerts_enabled = TWAI_ALERT_RX_DATA,
        .tx_queue_len = 8,
        .rx_queue_len = 12,
        .mode = TWAI_PORT_MODE_NORMAL,
        .log_tx = false,
        .log_rx = false,
        .log_alerts = false,
        .log_ext_only = false,
        .default_timeout_ticks = pdMS_TO_TICKS(25),
        .filter_mode = TWAI_PORT_FILTER_ACCEPT_ALL,
    };

    return cfg;
}

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
        .identifier = motor_codec_build_ext_id(producer_id, cw_raw),
        .data_length_code = dlc,
    };

    if (data != NULL && dlc > 0U) {
        memcpy(msg.data, data, dlc);
    }

    return msg;
}

static void fake_reset(void)
{
    memset(&s_fake, 0, sizeof(s_fake));
    s_fake.install_rc = ESP_OK;
    s_fake.uninstall_rc = ESP_OK;
    s_fake.start_rc = ESP_OK;
    s_fake.stop_rc = ESP_OK;
    s_fake.transmit_rc = ESP_OK;
    s_fake.receive_fallback_rc = ESP_ERR_TIMEOUT;
}

static esp_err_t fake_driver_install(const twai_general_config_t *g_config,
                                     const twai_timing_config_t *t_config,
                                     const twai_filter_config_t *f_config)
{
    (void)g_config;
    (void)t_config;
    (void)f_config;
    s_fake.install_calls++;
    return s_fake.install_rc;
}

static esp_err_t fake_driver_uninstall(void)
{
    s_fake.uninstall_calls++;
    return s_fake.uninstall_rc;
}

static esp_err_t fake_start(void)
{
    s_fake.start_calls++;
    return s_fake.start_rc;
}

static esp_err_t fake_stop(void)
{
    s_fake.stop_calls++;
    return s_fake.stop_rc;
}

static esp_err_t fake_transmit(twai_message_t *message, TickType_t ticks_to_wait)
{
    s_fake.transmit_calls++;
    s_fake.last_tx_timeout = ticks_to_wait;
    if (message != NULL) {
        s_fake.last_tx_msg = *message;
    }
    return s_fake.transmit_rc;
}

static esp_err_t fake_receive(twai_message_t *message, TickType_t ticks_to_wait)
{
    s_fake.receive_calls++;
    s_fake.last_rx_timeout = ticks_to_wait;

    if (s_fake.receive_enter_sem != NULL) {
        xSemaphoreGive(s_fake.receive_enter_sem);
    }

    if (s_fake.receive_release_sem != NULL) {
        if (xSemaphoreTake(s_fake.receive_release_sem, ticks_to_wait) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }

    if (s_fake.rx_queue_index < s_fake.rx_queue_len) {
        *message = s_fake.rx_queue[s_fake.rx_queue_index++];
        return ESP_OK;
    }

    return s_fake.receive_fallback_rc;
}

static esp_err_t fake_read_alerts(uint32_t *alerts, TickType_t ticks_to_wait)
{
    (void)alerts;
    (void)ticks_to_wait;
    return ESP_OK;
}

static esp_err_t fake_get_status_info(twai_status_info_t *status_info)
{
    if (status_info != NULL) {
        memset(status_info, 0, sizeof(*status_info));
        status_info->state = TWAI_STATE_RUNNING;
    }
    return ESP_OK;
}

static const twai_port_driver_ops_t s_fake_ops = {
    .driver_install = fake_driver_install,
    .driver_uninstall = fake_driver_uninstall,
    .start = fake_start,
    .stop = fake_stop,
    .transmit = fake_transmit,
    .receive = fake_receive,
    .read_alerts = fake_read_alerts,
    .get_status_info = fake_get_status_info,
};

static void queue_rx_frame(twai_message_t msg)
{
    TEST_ASSERT_LESS_THAN((int)(sizeof(s_fake.rx_queue) / sizeof(s_fake.rx_queue[0])),
                          (int)s_fake.rx_queue_len);
    s_fake.rx_queue[s_fake.rx_queue_len++] = msg;
}

static void exec_task(void *arg)
{
    exec_task_ctx_t *ctx = (exec_task_ctx_t *)arg;

    ctx->result = motor_dispatch_exec(&ctx->cmd,
                                      ctx->timeout_ticks,
                                      ctx->no_ack_grace_ticks,
                                      NULL,
                                      NULL,
                                      NULL);
    xSemaphoreGive(ctx->done_sem);
    vTaskDelete(NULL);
}

static void dispatch_test_begin(void)
{
    twai_port_cfg_t cfg;

    twai_port_test_reset_state();
    motor_dispatch_test_reset_state();
    fake_reset();
    twai_port_test_set_driver_ops(&s_fake_ops);
    cfg = make_cfg();
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_init(&cfg));
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_start());
}

static void dispatch_test_end(void)
{
    if (s_fake.receive_enter_sem != NULL) {
        vSemaphoreDelete(s_fake.receive_enter_sem);
        s_fake.receive_enter_sem = NULL;
    }

    if (s_fake.receive_release_sem != NULL) {
        vSemaphoreDelete(s_fake.receive_release_sem);
        s_fake.receive_release_sem = NULL;
    }

    if (twai_port_is_initialized()) {
        TEST_ASSERT_EQUAL(ESP_OK, twai_port_deinit());
    }

    motor_dispatch_test_reset_state();
    twai_port_test_reset_state();
}

static void observer_record_notification(const motor_rx_t *rx, void *ctx)
{
    observer_ctx_t *observer_ctx = (observer_ctx_t *)ctx;

    TEST_ASSERT_NOT_NULL(observer_ctx);
    TEST_ASSERT_NOT_NULL(rx);

    observer_ctx->notification_calls++;
    observer_ctx->last_kind = rx->kind;
    observer_ctx->last_base_code = rx->base_code;
    observer_ctx->last_object = rx->object;
}

static void observer_record_error(const motor_rx_t *rx, void *ctx)
{
    observer_ctx_t *observer_ctx = (observer_ctx_t *)ctx;

    TEST_ASSERT_NOT_NULL(observer_ctx);
    TEST_ASSERT_NOT_NULL(rx);

    observer_ctx->error_calls++;
    observer_ctx->last_kind = rx->kind;
    observer_ctx->last_base_code = rx->base_code;
    observer_ctx->last_object = rx->object;
}

TEST_CASE("motor dispatch completes ACK exchange with matching response", "[motor_dispatch]")
{
    motor_cmd_t cmd;
    motor_rx_t response;
    motor_dispatch_pending_t pending;
    uint8_t data[] = {MOTOR_PP_INDEX_NODE_ID, 0x2A};

    dispatch_test_begin();

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_get(0x05, true, MOTOR_PP_INDEX_NODE_ID, &cmd));
    queue_rx_frame(make_ext_frame(0x05, 0x01, 2U, data));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_OK,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(20), pdMS_TO_TICKS(5), NULL, &response, NULL));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&response, &cmd));
    TEST_ASSERT_FALSE(motor_dispatch_has_pending());
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, motor_dispatch_get_pending(&pending));

    dispatch_test_end();
}

TEST_CASE("motor dispatch times out waiting for ACK", "[motor_dispatch]")
{
    motor_cmd_t cmd;

    dispatch_test_begin();

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_get(0x05, true, MOTOR_PP_INDEX_NODE_ID, &cmd));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_TIMEOUT,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(10), pdMS_TO_TICKS(5), NULL, NULL, NULL));
    TEST_ASSERT_FALSE(motor_dispatch_has_pending());

    dispatch_test_end();
}

TEST_CASE("motor dispatch succeeds for no-ACK command after grace window", "[motor_dispatch]")
{
    motor_cmd_t cmd;

    dispatch_test_begin();

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_bg_begin(0x09, false, &cmd));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_OK,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(20), pdMS_TO_TICKS(5), NULL, NULL, NULL));
    TEST_ASSERT_FALSE(motor_dispatch_has_pending());

    dispatch_test_end();
}

TEST_CASE("motor dispatch fails on related thrown error before ACK", "[motor_dispatch]")
{
    motor_cmd_t cmd;
    motor_rx_t related_error;
    uint8_t error_data[] = {
        MOTOR_ER_INDEX_CLEAR_ALL, 0x91, 0x00, MOTOR_PP_INDEX_NODE_ID, 0x00, 0x00
    };

    dispatch_test_begin();

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_get(0x05, true, MOTOR_PP_INDEX_NODE_ID, &cmd));
    error_data[2] = cmd.cw_raw;
    queue_rx_frame(make_ext_frame(0x05, 0x0F, 6U, error_data));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_REMOTE_ERROR,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(20), pdMS_TO_TICKS(5), NULL, NULL, &related_error));
    TEST_ASSERT_TRUE(motor_rx_is_error(&related_error));
    TEST_ASSERT_FALSE(motor_dispatch_has_pending());

    dispatch_test_end();
}

TEST_CASE("motor dispatch ignores unrelated notification while waiting", "[motor_dispatch]")
{
    motor_cmd_t cmd;
    motor_rx_t response;
    uint8_t notify_data[] = {0x01, 0x02};
    uint8_t ack_data[] = {MOTOR_PP_INDEX_NODE_ID, 0x2A};

    dispatch_test_begin();

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_get(0x05, true, MOTOR_PP_INDEX_NODE_ID, &cmd));
    queue_rx_frame(make_ext_frame(0x33, 0x7F, 2U, notify_data));
    queue_rx_frame(make_ext_frame(0x05, 0x01, 2U, ack_data));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_OK,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(20), pdMS_TO_TICKS(5), NULL, &response, NULL));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&response, &cmd));

    dispatch_test_end();
}

TEST_CASE("motor dispatch ignores unrelated error for completion", "[motor_dispatch]")
{
    motor_cmd_t cmd;
    motor_rx_t response;
    uint8_t unrelated_error_data[] = {
        MOTOR_ER_INDEX_HISTORY_2, 0x77, 0x26, MOTOR_PP_INDEX_NODE_ID, 0x00, 0x00
    };
    uint8_t ack_data[] = {MOTOR_PP_INDEX_NODE_ID, 0x2A};

    dispatch_test_begin();

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_get(0x05, true, MOTOR_PP_INDEX_NODE_ID, &cmd));
    queue_rx_frame(make_ext_frame(0x05, 0x0F, 6U, unrelated_error_data));
    queue_rx_frame(make_ext_frame(0x05, 0x01, 2U, ack_data));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_OK,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(20), pdMS_TO_TICKS(5), NULL, &response, NULL));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&response, &cmd));

    dispatch_test_end();
}

TEST_CASE("motor dispatch returns busy while another exchange is active", "[motor_dispatch]")
{
    exec_task_ctx_t ctx;
    motor_cmd_t second_cmd;

    dispatch_test_begin();

    memset(&ctx, 0, sizeof(ctx));
    ctx.done_sem = xSemaphoreCreateBinary();
    TEST_ASSERT_NOT_NULL(ctx.done_sem);
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_get(0x05, true, MOTOR_PP_INDEX_NODE_ID, &ctx.cmd));
    ctx.timeout_ticks = pdMS_TO_TICKS(20);
    ctx.no_ack_grace_ticks = pdMS_TO_TICKS(5);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_get(0x06, true, MOTOR_PP_INDEX_NODE_ID, &second_cmd));

    s_fake.receive_enter_sem = xSemaphoreCreateBinary();
    s_fake.receive_release_sem = xSemaphoreCreateBinary();
    TEST_ASSERT_NOT_NULL(s_fake.receive_enter_sem);
    TEST_ASSERT_NOT_NULL(s_fake.receive_release_sem);

    TEST_ASSERT_EQUAL(pdPASS,
                      xTaskCreate(exec_task, "dispatch_exec", 4096, &ctx, tskIDLE_PRIORITY + 1, NULL));
    TEST_ASSERT_EQUAL(pdTRUE,
                      xSemaphoreTake(s_fake.receive_enter_sem, pdMS_TO_TICKS(50)));
    TEST_ASSERT_TRUE(motor_dispatch_has_pending());

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_BUSY,
                      motor_dispatch_exec(&second_cmd, pdMS_TO_TICKS(20), pdMS_TO_TICKS(5), NULL, NULL, NULL));

    xSemaphoreGive(s_fake.receive_release_sem);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(ctx.done_sem, pdMS_TO_TICKS(100)));
    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_TIMEOUT, ctx.result);
    TEST_ASSERT_FALSE(motor_dispatch_has_pending());

    vSemaphoreDelete(ctx.done_sem);

    dispatch_test_end();
}

TEST_CASE("motor dispatch matches ER command responses through motor_rx_matches_cmd", "[motor_dispatch]")
{
    motor_cmd_t cmd;
    motor_rx_t response;
    uint8_t er_data[] = {
        MOTOR_ER_INDEX_HISTORY_4, 0x91, 0xA4, 0x07, 0x00, 0x00
    };

    dispatch_test_begin();

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_er_get(0x7E, true, MOTOR_ER_INDEX_HISTORY_4, &cmd));
    queue_rx_frame(make_ext_frame(0x7E, 0x0F, 6U, er_data));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_OK,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(20), pdMS_TO_TICKS(5), NULL, &response, NULL));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&response, &cmd));
    TEST_ASSERT_TRUE(motor_rx_is_error(&response));

    dispatch_test_end();
}

TEST_CASE("motor dispatch treats ER GET 10 history response as terminal success", "[motor_dispatch]")
{
    motor_cmd_t cmd;
    motor_rx_t response;
    uint8_t er_data[] = {
        MOTOR_ER_INDEX_HISTORY_1, 0x11, 0x8F, 0x00, 0x00, 0x00
    };

    dispatch_test_begin();

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_er_get(0x7E, true, MOTOR_ER_INDEX_HISTORY_1, &cmd));
    queue_rx_frame(make_ext_frame(0x7E, 0x0F, 6U, er_data));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_OK,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(20), pdMS_TO_TICKS(5), NULL, &response, NULL));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&response, &cmd));
    TEST_ASSERT_TRUE(motor_rx_is_error(&response));

    dispatch_test_end();
}

TEST_CASE("motor dispatch treats ER clear-all zero response as terminal success", "[motor_dispatch]")
{
    motor_cmd_t cmd;
    motor_rx_t response;
    uint8_t er_data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    dispatch_test_begin();

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_er_clear_all(0x7E, true, &cmd));
    queue_rx_frame(make_ext_frame(0x7E, 0x0F, 6U, er_data));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_OK,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(20), pdMS_TO_TICKS(5), NULL, &response, NULL));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&response, &cmd));
    TEST_ASSERT_TRUE(motor_rx_er_is_clear_all_response(&response));

    dispatch_test_end();
}

TEST_CASE("motor dispatch treats thrown ER d0 zero during ER GET as remote error", "[motor_dispatch]")
{
    motor_cmd_t cmd;
    motor_rx_t related_error;
    uint8_t error_data[] = {
        MOTOR_ER_INDEX_CLEAR_ALL, 0x11, 0x00, 0x00, 0x00, 0x00
    };

    dispatch_test_begin();

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_er_get(0x7E, true, MOTOR_ER_INDEX_HISTORY_1, &cmd));
    error_data[2] = cmd.cw_raw;
    error_data[3] = (uint8_t)cmd.index;
    queue_rx_frame(make_ext_frame(0x7E, 0x0F, 6U, error_data));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_REMOTE_ERROR,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(20), pdMS_TO_TICKS(5), NULL, NULL, &related_error));
    TEST_ASSERT_TRUE(motor_rx_er_is_thrown_error(&related_error));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&related_error, &cmd));

    dispatch_test_end();
}

TEST_CASE("motor dispatch completes asymmetric PA set through DV response", "[motor_dispatch]")
{
    motor_cmd_t cmd;
    motor_rx_t response;
    uint8_t dv_data[5] = {4U};

    dispatch_test_begin();

    motor_codec_pack_i32_le(&dv_data[1], -123456);
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pa_set(0x0A, true, -123456, &cmd));
    queue_rx_frame(make_ext_frame(0x0A, 0x2E, 5U, dv_data));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_OK,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(20), pdMS_TO_TICKS(5), NULL, &response, NULL));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&response, &cmd));
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_DV, response.object);

    dispatch_test_end();
}

TEST_CASE("motor dispatch completes LM set through 0x24 response shape", "[motor_dispatch]")
{
    motor_cmd_t cmd;
    motor_rx_t response;
    uint8_t lm_data[5] = {MOTOR_LM_INDEX_MAX_ACCEL_DECEL};

    dispatch_test_begin();

    motor_codec_pack_i32_le(&lm_data[1], 7890);
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_lm_set_i32(0x2F, true, MOTOR_LM_INDEX_MAX_ACCEL_DECEL, 7890, &cmd));
    queue_rx_frame(make_ext_frame(0x2F, 0x24, 5U, lm_data));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_OK,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(20), pdMS_TO_TICKS(5), NULL, &response, NULL));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&response, &cmd));
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_LM, response.object);
    TEST_ASSERT_EQUAL_HEX8(0x24, response.base_code);

    dispatch_test_end();
}

TEST_CASE("motor dispatch ignores malformed near-match and waits for real response", "[motor_dispatch]")
{
    motor_cmd_t cmd;
    motor_rx_t response;
    uint8_t bad_ie_data[] = {MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION, 0x01};
    uint8_t good_ie_data[] = {MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION, 0x01, 0x00};

    dispatch_test_begin();

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_ie_get(0x05, true, MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION, &cmd));
    queue_rx_frame(make_ext_frame(0x05, 0x07, 2U, bad_ie_data));
    queue_rx_frame(make_ext_frame(0x05, 0x07, 3U, good_ie_data));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_OK,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(20), pdMS_TO_TICKS(5), NULL, &response, NULL));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&response, &cmd));
    TEST_ASSERT_EQUAL_HEX8(0x07, response.base_code);

    dispatch_test_end();
}

TEST_CASE("motor dispatch observer receives unrelated notification and error side traffic", "[motor_dispatch]")
{
    motor_cmd_t cmd;
    motor_rx_t response;
    observer_ctx_t observer_ctx = {0};
    motor_dispatch_observer_t observer = {
        .on_notification = observer_record_notification,
        .on_error = observer_record_error,
        .ctx = &observer_ctx,
    };
    uint8_t notify_data[] = {0x44, 0x02};
    uint8_t unrelated_error_data[] = {
        MOTOR_ER_INDEX_HISTORY_2, 0x77, 0x26, MOTOR_PP_INDEX_NODE_ID, 0x00, 0x00
    };
    uint8_t ack_data[] = {MOTOR_PP_INDEX_NODE_ID, 0x2A};

    dispatch_test_begin();

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_get(0x05, true, MOTOR_PP_INDEX_NODE_ID, &cmd));
    queue_rx_frame(make_ext_frame(0x33, 0x5A, 2U, notify_data));
    queue_rx_frame(make_ext_frame(0x05, 0x0F, 6U, unrelated_error_data));
    queue_rx_frame(make_ext_frame(0x05, 0x01, 2U, ack_data));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_OK,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(20), pdMS_TO_TICKS(5), &observer, &response, NULL));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&response, &cmd));
    TEST_ASSERT_EQUAL(1, observer_ctx.notification_calls);
    TEST_ASSERT_EQUAL(1, observer_ctx.error_calls);
    TEST_ASSERT_EQUAL(MOTOR_RX_KIND_ERROR, observer_ctx.last_kind);
    TEST_ASSERT_EQUAL_HEX8(0x0F, observer_ctx.last_base_code);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_ER, observer_ctx.last_object);

    dispatch_test_end();
}

TEST_CASE("motor dispatch observer side traffic does not change no-ACK terminal behavior", "[motor_dispatch]")
{
    motor_cmd_t cmd;
    observer_ctx_t observer_ctx = {0};
    /* This build uses 100 Hz ticks, so 5 ms rounds down to 0 ticks. */
    const TickType_t no_ack_grace_ticks = pdMS_TO_TICKS(20);
    motor_dispatch_observer_t observer = {
        .on_notification = observer_record_notification,
        .on_error = observer_record_error,
        .ctx = &observer_ctx,
    };
    uint8_t notify_data[] = {0x44, 0x02};
    uint8_t unrelated_error_data[] = {
        MOTOR_ER_INDEX_HISTORY_2, 0x77, 0x26, MOTOR_PP_INDEX_NODE_ID, 0x00, 0x00
    };

    dispatch_test_begin();

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_bg_begin(0x09, false, &cmd));
    queue_rx_frame(make_ext_frame(0x33, 0x5A, 2U, notify_data));
    queue_rx_frame(make_ext_frame(0x05, 0x0F, 6U, unrelated_error_data));

    TEST_ASSERT_EQUAL(MOTOR_DISPATCH_RESULT_OK,
                      motor_dispatch_exec(&cmd, pdMS_TO_TICKS(20), no_ack_grace_ticks, &observer, NULL, NULL));
    TEST_ASSERT_EQUAL(1, observer_ctx.notification_calls);
    TEST_ASSERT_EQUAL(1, observer_ctx.error_calls);
    TEST_ASSERT_FALSE(motor_dispatch_has_pending());

    dispatch_test_end();
}
