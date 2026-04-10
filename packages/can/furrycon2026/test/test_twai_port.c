/*
 * Responsibility:
 * Unity coverage for the TWAI transport port public API and state machine.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"

#include "twai_port.h"
#include "test_log.h"

typedef struct {
    int install_calls;
    int uninstall_calls;
    int start_calls;
    int stop_calls;
    int transmit_calls;
    int receive_calls;
    int read_alerts_calls;
    int get_status_calls;

    esp_err_t install_rc;
    esp_err_t uninstall_rc;
    esp_err_t start_rc;
    esp_err_t stop_rc;
    esp_err_t transmit_rc;
    esp_err_t receive_fallback_rc;
    esp_err_t read_alerts_rc;
    esp_err_t get_status_rc;

    twai_general_config_t last_general_cfg;
    twai_timing_config_t last_timing_cfg;
    twai_filter_config_t last_filter_cfg;
    twai_message_t last_tx_msg;
    TickType_t last_tx_timeout;
    TickType_t last_rx_timeout;
    uint32_t alerts_value;
    twai_status_info_t status_value;

    twai_message_t rx_queue[8];
    size_t rx_queue_len;
    size_t rx_queue_index;

    SemaphoreHandle_t receive_enter_sem;
    SemaphoreHandle_t receive_release_sem;
} fake_driver_t;

typedef struct {
    TaskHandle_t task;
    SemaphoreHandle_t done_sem;
    esp_err_t rc;
    twai_message_t msg;
    TickType_t timeout_ticks;
} receive_task_ctx_t;

static fake_driver_t s_fake;

static twai_port_cfg_t make_cfg(void)
{
    twai_port_cfg_t cfg = {
        .tx_gpio = 21,
        .rx_gpio = 20,
        .bitrate = 500000,
        .alerts_enabled = TWAI_ALERT_RX_DATA | TWAI_ALERT_BUS_OFF,
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

static void fake_reset(void)
{
    memset(&s_fake, 0, sizeof(s_fake));
    s_fake.install_rc = ESP_OK;
    s_fake.uninstall_rc = ESP_OK;
    s_fake.start_rc = ESP_OK;
    s_fake.stop_rc = ESP_OK;
    s_fake.transmit_rc = ESP_OK;
    s_fake.receive_fallback_rc = ESP_ERR_TIMEOUT;
    s_fake.read_alerts_rc = ESP_OK;
    s_fake.get_status_rc = ESP_OK;
    s_fake.status_value.state = TWAI_STATE_RUNNING;
}

static esp_err_t fake_driver_install(const twai_general_config_t *g_config,
                                     const twai_timing_config_t *t_config,
                                     const twai_filter_config_t *f_config)
{
    s_fake.install_calls++;
    if (g_config != NULL) {
        s_fake.last_general_cfg = *g_config;
    }
    if (t_config != NULL) {
        s_fake.last_timing_cfg = *t_config;
    }
    if (f_config != NULL) {
        s_fake.last_filter_cfg = *f_config;
    }
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
    s_fake.read_alerts_calls++;
    s_fake.last_rx_timeout = ticks_to_wait;
    if (alerts != NULL) {
        *alerts = s_fake.alerts_value;
    }
    return s_fake.read_alerts_rc;
}

static esp_err_t fake_get_status_info(twai_status_info_t *status_info)
{
    s_fake.get_status_calls++;
    if (status_info != NULL) {
        *status_info = s_fake.status_value;
    }
    return s_fake.get_status_rc;
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

static void receive_task(void *arg)
{
    receive_task_ctx_t *ctx = (receive_task_ctx_t *)arg;
    ctx->rc = twai_port_receive(&ctx->msg, ctx->timeout_ticks);
    xSemaphoreGive(ctx->done_sem);
    vTaskDelete(NULL);
}

void test_twai_port_set_up(void)
{
    twai_port_test_reset_state();
    fake_reset();
    twai_port_test_set_driver_ops(&s_fake_ops);
}

void test_twai_port_tear_down(void)
{
    if (s_fake.receive_enter_sem != NULL) {
        vSemaphoreDelete(s_fake.receive_enter_sem);
        s_fake.receive_enter_sem = NULL;
    }

    if (s_fake.receive_release_sem != NULL) {
        vSemaphoreDelete(s_fake.receive_release_sem);
        s_fake.receive_release_sem = NULL;
    }

    twai_port_test_reset_state();
}

TEST_CASE("twai port rejects invalid configuration", "[twai_port]")
{
    twai_port_cfg_t cfg = make_cfg();

    LOG_SECTION("twai_port config validation");
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, twai_port_init(NULL));
    LOG_INPUT("cfg=NULL");
    LOG_ERROR("rc=%s", esp_err_to_name(ESP_ERR_INVALID_ARG));

    cfg.tx_queue_len = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, twai_port_init(&cfg));
    LOG_INPUT("tx_queue_len=%u rx_queue_len=%u bitrate=%lu mode=%d filter=%d",
              cfg.tx_queue_len, cfg.rx_queue_len, (unsigned long)cfg.bitrate, cfg.mode, cfg.filter_mode);
    LOG_ERROR("rc=%s", esp_err_to_name(ESP_ERR_INVALID_ARG));

    cfg = make_cfg();
    cfg.bitrate = 333333;
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, twai_port_init(&cfg));
    LOG_INPUT("unsupported bitrate=%lu", (unsigned long)cfg.bitrate);
    LOG_ERROR("rc=%s", esp_err_to_name(ESP_ERR_NOT_SUPPORTED));

    cfg = make_cfg();
    cfg.mode = (twai_port_mode_t)99;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, twai_port_init(&cfg));
    LOG_INPUT("invalid mode=%d", cfg.mode);
    LOG_ERROR("rc=%s", esp_err_to_name(ESP_ERR_INVALID_ARG));

    cfg = make_cfg();
    cfg.filter_mode = (twai_port_filter_mode_t)99;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, twai_port_init(&cfg));
    LOG_INPUT("invalid filter_mode=%d", cfg.filter_mode);
    LOG_ERROR("rc=%s install_calls=%d", esp_err_to_name(ESP_ERR_INVALID_ARG), s_fake.install_calls);

    TEST_ASSERT_EQUAL(0, s_fake.install_calls);
}

TEST_CASE("twai port enforces lifecycle state transitions", "[twai_port]")
{
    twai_port_cfg_t cfg = make_cfg();

    LOG_SECTION("twai_port lifecycle");
    LOG_INPUT("start before init");
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, twai_port_start());
    LOG_ERROR("rc=%s", esp_err_to_name(ESP_ERR_INVALID_STATE));

    LOG_INPUT("init bitrate=%lu tx_gpio=%d rx_gpio=%d", (unsigned long)cfg.bitrate, cfg.tx_gpio, cfg.rx_gpio);
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_init(&cfg));
    TEST_ASSERT_TRUE(twai_port_is_initialized());
    TEST_ASSERT_EQUAL(TWAI_PORT_STATE_STOPPED, twai_port_get_state());
    LOG_OUTPUT("after init state=%d initialized=%d install_calls=%d",
               twai_port_get_state(), twai_port_is_initialized(), s_fake.install_calls);

    TEST_ASSERT_EQUAL(ESP_OK, twai_port_start());
    TEST_ASSERT_TRUE(twai_port_is_running());
    TEST_ASSERT_EQUAL(TWAI_PORT_STATE_RUNNING, twai_port_get_state());
    LOG_OUTPUT("after start state=%d running=%d start_calls=%d",
               twai_port_get_state(), twai_port_is_running(), s_fake.start_calls);

    TEST_ASSERT_EQUAL(ESP_OK, twai_port_stop());
    TEST_ASSERT_FALSE(twai_port_is_running());
    TEST_ASSERT_EQUAL(TWAI_PORT_STATE_STOPPED, twai_port_get_state());
    LOG_OUTPUT("after stop state=%d running=%d stop_calls=%d",
               twai_port_get_state(), twai_port_is_running(), s_fake.stop_calls);

    TEST_ASSERT_EQUAL(ESP_OK, twai_port_deinit());
    TEST_ASSERT_FALSE(twai_port_is_initialized());
    TEST_ASSERT_EQUAL(TWAI_PORT_STATE_UNINITIALIZED, twai_port_get_state());
    LOG_OUTPUT("after deinit state=%d initialized=%d uninstall_calls=%d",
               twai_port_get_state(), twai_port_is_initialized(), s_fake.uninstall_calls);

    TEST_ASSERT_EQUAL(1, s_fake.install_calls);
    TEST_ASSERT_EQUAL(1, s_fake.start_calls);
    TEST_ASSERT_EQUAL(1, s_fake.stop_calls);
    TEST_ASSERT_EQUAL(1, s_fake.uninstall_calls);
}

TEST_CASE("twai port maps standard tx frame fields", "[twai_port]")
{
    static const uint8_t payload[] = {0x11, 0x22, 0x33, 0x44};
    twai_port_cfg_t cfg = make_cfg();

    LOG_SECTION("twai_port tx frame mapping");
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_init(&cfg));
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_start());

    LOG_INPUT("std_id=0x%03X dlc=%u data=[%02X %02X %02X %02X]",
              0x321, (unsigned)sizeof(payload), payload[0], payload[1], payload[2], payload[3]);
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_send_std(0x321, payload, sizeof(payload), pdMS_TO_TICKS(10)));

    TEST_ASSERT_EQUAL(1, s_fake.transmit_calls);
    TEST_ASSERT_EQUAL_HEX32(0x321, s_fake.last_tx_msg.identifier);
    TEST_ASSERT_FALSE(s_fake.last_tx_msg.extd);
    TEST_ASSERT_FALSE(s_fake.last_tx_msg.rtr);
    TEST_ASSERT_EQUAL_UINT8(sizeof(payload), s_fake.last_tx_msg.data_length_code);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, s_fake.last_tx_msg.data, sizeof(payload));
    LOG_OUTPUT("tx identifier=0x%08lX extd=%d rtr=%d dlc=%u timeout_ticks=%lu",
               (unsigned long)s_fake.last_tx_msg.identifier, s_fake.last_tx_msg.extd,
               s_fake.last_tx_msg.rtr, s_fake.last_tx_msg.data_length_code,
               (unsigned long)s_fake.last_tx_timeout);
}

TEST_CASE("twai port maps extended tx frame fields", "[twai_port]")
{
    static const uint8_t payload[] = {0xDE, 0xAD, 0xBE};
    twai_port_cfg_t cfg = make_cfg();

    TEST_ASSERT_EQUAL(ESP_OK, twai_port_init(&cfg));
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_start());

    LOG_INPUT("ext_id=0x%08lX dlc=%u data=[%02X %02X %02X]",
              (unsigned long)0x1ABCDE0, (unsigned)sizeof(payload), payload[0], payload[1], payload[2]);
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_send_ext(0x1ABCDE0, payload, sizeof(payload), pdMS_TO_TICKS(7)));

    TEST_ASSERT_EQUAL(1, s_fake.transmit_calls);
    TEST_ASSERT_EQUAL_HEX32(0x1ABCDE0, s_fake.last_tx_msg.identifier);
    TEST_ASSERT_TRUE(s_fake.last_tx_msg.extd);
    TEST_ASSERT_FALSE(s_fake.last_tx_msg.rtr);
    TEST_ASSERT_EQUAL_UINT8(sizeof(payload), s_fake.last_tx_msg.data_length_code);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(payload, s_fake.last_tx_msg.data, sizeof(payload));
    LOG_OUTPUT("tx identifier=0x%08lX extd=%d rtr=%d dlc=%u timeout_ticks=%lu",
               (unsigned long)s_fake.last_tx_msg.identifier, s_fake.last_tx_msg.extd,
               s_fake.last_tx_msg.rtr, s_fake.last_tx_msg.data_length_code,
               (unsigned long)s_fake.last_tx_timeout);
}

TEST_CASE("twai port receive returns queued standard frame intact", "[twai_port]")
{
    twai_port_cfg_t cfg = make_cfg();
    twai_message_t msg = {0};
    static const uint8_t expected[] = {0x01, 0x02, 0x03};

    LOG_SECTION("twai_port rx behavior");
    s_fake.rx_queue[0] = (twai_message_t) {
        .identifier = 0x456,
        .data_length_code = 3,
        .data = {0x01, 0x02, 0x03},
        .extd = 0,
        .rtr = 0,
    };
    s_fake.rx_queue_len = 1;
    LOG_INPUT("queued rx std frame id=0x%03X dlc=%u data=[%02X %02X %02X]",
              0x456, 3U, expected[0], expected[1], expected[2]);

    TEST_ASSERT_EQUAL(ESP_OK, twai_port_init(&cfg));
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_start());
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_receive(&msg, pdMS_TO_TICKS(20)));

    TEST_ASSERT_EQUAL(1, s_fake.receive_calls);
    TEST_ASSERT_EQUAL_HEX32(0x456, msg.identifier);
    TEST_ASSERT_FALSE(msg.extd);
    TEST_ASSERT_FALSE(msg.rtr);
    TEST_ASSERT_EQUAL_UINT8(3, msg.data_length_code);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, msg.data, 3);
    LOG_OUTPUT("received id=0x%08lX extd=%d rtr=%d dlc=%u receive_calls=%d",
               (unsigned long)msg.identifier, msg.extd, msg.rtr,
               msg.data_length_code, s_fake.receive_calls);
}

TEST_CASE("twai port receive applies software frame filter mode", "[twai_port]")
{
    twai_port_cfg_t cfg = make_cfg();
    twai_message_t msg = {0};
    static const uint8_t expected[] = {0x55, 0xAA};

    cfg.filter_mode = TWAI_PORT_FILTER_STD_ONLY;
    s_fake.rx_queue[0] = (twai_message_t) {
        .identifier = 0x1ABCDE0,
        .data_length_code = 1,
        .data = {0x99},
        .extd = 1,
    };
    s_fake.rx_queue[1] = (twai_message_t) {
        .identifier = 0x123,
        .data_length_code = 2,
        .data = {0x55, 0xAA},
        .extd = 0,
    };
    s_fake.rx_queue_len = 2;
    LOG_INPUT("filter_mode=STD_ONLY queued frames=[ext:0x%08X, std:0x%03X]",
              0x1ABCDE0, 0x123);

    TEST_ASSERT_EQUAL(ESP_OK, twai_port_init(&cfg));
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_start());
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_receive(&msg, pdMS_TO_TICKS(20)));

    TEST_ASSERT_EQUAL(2, s_fake.receive_calls);
    TEST_ASSERT_EQUAL_HEX32(0x123, msg.identifier);
    TEST_ASSERT_FALSE(msg.extd);
    TEST_ASSERT_EQUAL_UINT8(2, msg.data_length_code);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, msg.data, 2);
    LOG_OUTPUT("filtered receive_calls=%d returned id=0x%08lX extd=%d data=[%02X %02X]",
               s_fake.receive_calls, (unsigned long)msg.identifier, msg.extd, msg.data[0], msg.data[1]);
}

TEST_CASE("twai port quiesce and resume gate send and receive", "[twai_port]")
{
    static const uint8_t payload[] = {0xA5};
    twai_port_cfg_t cfg = make_cfg();

    LOG_SECTION("twai_port quiesce and resume");
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_init(&cfg));
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_start());
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_quiesce(pdMS_TO_TICKS(20)));
    TEST_ASSERT_TRUE(twai_port_is_quiesced());
    LOG_OUTPUT("after quiesce state=%d quiesced=%d", twai_port_get_state(), twai_port_is_quiesced());

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, twai_port_send_std(0x123, payload, sizeof(payload), pdMS_TO_TICKS(5)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, twai_port_receive(&(twai_message_t){0}, pdMS_TO_TICKS(5)));
    LOG_ERROR("send while quiesced rc=%s", esp_err_to_name(ESP_ERR_INVALID_STATE));
    LOG_ERROR("receive while quiesced rc=%s", esp_err_to_name(ESP_ERR_INVALID_STATE));

    TEST_ASSERT_EQUAL(ESP_OK, twai_port_resume());
    TEST_ASSERT_TRUE(twai_port_is_running());
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_send_std(0x123, payload, sizeof(payload), pdMS_TO_TICKS(5)));
    LOG_OUTPUT("after resume state=%d running=%d transmit_calls=%d",
               twai_port_get_state(), twai_port_is_running(), s_fake.transmit_calls);

    TEST_ASSERT_EQUAL(1, s_fake.transmit_calls);
}

TEST_CASE("twai port quiesce waits for in flight receive", "[twai_port]")
{
    twai_port_cfg_t cfg = make_cfg();
    receive_task_ctx_t task_ctx = {
        .done_sem = xSemaphoreCreateBinary(),
        .timeout_ticks = pdMS_TO_TICKS(200),
        .rc = ESP_FAIL,
    };

    LOG_SECTION("twai_port in-flight receive drain");
    TEST_ASSERT_NOT_NULL(task_ctx.done_sem);
    s_fake.receive_enter_sem = xSemaphoreCreateBinary();
    s_fake.receive_release_sem = xSemaphoreCreateBinary();
    TEST_ASSERT_NOT_NULL(s_fake.receive_enter_sem);
    TEST_ASSERT_NOT_NULL(s_fake.receive_release_sem);

    s_fake.rx_queue[0] = (twai_message_t) {
        .identifier = 0x111,
        .data_length_code = 1,
        .data = {0x42},
    };
    s_fake.rx_queue_len = 1;

    TEST_ASSERT_EQUAL(ESP_OK, twai_port_init(&cfg));
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_start());
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(receive_task, "twai_rx", 4096, &task_ctx, 5, &task_ctx.task));
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(s_fake.receive_enter_sem, pdMS_TO_TICKS(50)));
    LOG_INPUT("receive task entered fake driver, attempting quiesce with short timeout");

    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, twai_port_quiesce(pdMS_TO_TICKS(10)));
    TEST_ASSERT_TRUE(twai_port_is_running());
    LOG_ERROR("quiesce short timeout rc=%s state=%d", esp_err_to_name(ESP_ERR_TIMEOUT), twai_port_get_state());

    xSemaphoreGive(s_fake.receive_release_sem);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(task_ctx.done_sem, pdMS_TO_TICKS(100)));
    TEST_ASSERT_EQUAL(ESP_OK, task_ctx.rc);
    LOG_OUTPUT("receive task completed rc=%s id=0x%08lX", esp_err_to_name(task_ctx.rc), (unsigned long)task_ctx.msg.identifier);

    TEST_ASSERT_EQUAL(ESP_OK, twai_port_quiesce(pdMS_TO_TICKS(20)));
    TEST_ASSERT_TRUE(twai_port_is_quiesced());
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_resume());
    LOG_OUTPUT("second quiesce succeeded, resumed state=%d", twai_port_get_state());

    vSemaphoreDelete(task_ctx.done_sem);
    task_ctx.done_sem = NULL;
}

TEST_CASE("twai port exposes status alerts and bus off state", "[twai_port]")
{
    twai_port_cfg_t cfg = make_cfg();
    twai_status_info_t status = {0};
    uint32_t alerts = 0;

    LOG_SECTION("twai_port status and alerts");
    s_fake.status_value.state = TWAI_STATE_BUS_OFF;
    s_fake.status_value.tx_error_counter = 17;
    s_fake.status_value.rx_error_counter = 9;
    s_fake.alerts_value = TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_QUEUE_FULL;
    LOG_INPUT("status.state=BUS_OFF tx_err=%lu rx_err=%lu alerts=0x%08lX",
              (unsigned long)s_fake.status_value.tx_error_counter,
              (unsigned long)s_fake.status_value.rx_error_counter,
              (unsigned long)s_fake.alerts_value);

    TEST_ASSERT_EQUAL(ESP_OK, twai_port_init(&cfg));
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_start());

    TEST_ASSERT_EQUAL(ESP_OK, twai_port_get_status(&status));
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_read_alerts(&alerts, pdMS_TO_TICKS(12)));
    TEST_ASSERT_TRUE(twai_port_is_bus_off());

    TEST_ASSERT_EQUAL(TWAI_STATE_BUS_OFF, status.state);
    TEST_ASSERT_EQUAL_UINT32(17, status.tx_error_counter);
    TEST_ASSERT_EQUAL_UINT32(9, status.rx_error_counter);
    TEST_ASSERT_EQUAL_HEX32(TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_QUEUE_FULL, alerts);
    TEST_ASSERT_EQUAL(2, s_fake.get_status_calls);
    TEST_ASSERT_EQUAL(1, s_fake.read_alerts_calls);
    LOG_OUTPUT("status.state=%d tx_err=%lu rx_err=%lu alerts=0x%08lX bus_off=%d",
               status.state, (unsigned long)status.tx_error_counter,
               (unsigned long)status.rx_error_counter, (unsigned long)alerts,
               twai_port_is_bus_off());
}

TEST_CASE("twai port set bitrate reconfigures and preserves other fields", "[twai_port]")
{
    twai_port_cfg_t cfg = make_cfg();
    twai_port_cfg_t current = {0};

    LOG_SECTION("twai_port reconfigure");
    cfg.tx_gpio = 4;
    cfg.rx_gpio = 5;
    cfg.tx_queue_len = 3;
    cfg.rx_queue_len = 9;
    cfg.mode = TWAI_PORT_MODE_NO_ACK;
    cfg.filter_mode = TWAI_PORT_FILTER_EXT_ONLY;
    cfg.log_tx = true;
    cfg.log_rx = true;
    cfg.log_alerts = true;
    cfg.log_ext_only = true;
    cfg.default_timeout_ticks = pdMS_TO_TICKS(99);
    LOG_INPUT("initial bitrate=%lu new bitrate=%lu mode=%d filter=%d tx_queue=%u rx_queue=%u",
              (unsigned long)cfg.bitrate, (unsigned long)250000, cfg.mode, cfg.filter_mode,
              cfg.tx_queue_len, cfg.rx_queue_len);

    TEST_ASSERT_EQUAL(ESP_OK, twai_port_init(&cfg));
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_start());
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_set_bitrate_blocking(250000, pdMS_TO_TICKS(30)));
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_get_cfg(&current));

    TEST_ASSERT_EQUAL_UINT32(250000, current.bitrate);
    TEST_ASSERT_EQUAL(cfg.tx_gpio, current.tx_gpio);
    TEST_ASSERT_EQUAL(cfg.rx_gpio, current.rx_gpio);
    TEST_ASSERT_EQUAL(cfg.tx_queue_len, current.tx_queue_len);
    TEST_ASSERT_EQUAL(cfg.rx_queue_len, current.rx_queue_len);
    TEST_ASSERT_EQUAL(cfg.mode, current.mode);
    TEST_ASSERT_EQUAL(cfg.filter_mode, current.filter_mode);
    TEST_ASSERT_EQUAL(cfg.alerts_enabled, current.alerts_enabled);
    TEST_ASSERT_EQUAL(cfg.log_tx, current.log_tx);
    TEST_ASSERT_EQUAL(cfg.log_rx, current.log_rx);
    TEST_ASSERT_EQUAL(cfg.log_alerts, current.log_alerts);
    TEST_ASSERT_EQUAL(cfg.log_ext_only, current.log_ext_only);
    TEST_ASSERT_EQUAL(cfg.default_timeout_ticks, current.default_timeout_ticks);

    TEST_ASSERT_EQUAL(2, s_fake.install_calls);
    TEST_ASSERT_EQUAL(1, s_fake.uninstall_calls);
    TEST_ASSERT_EQUAL(1, s_fake.stop_calls);
    TEST_ASSERT_EQUAL(2, s_fake.start_calls);
    TEST_ASSERT_EQUAL(TWAI_MODE_NO_ACK, s_fake.last_general_cfg.mode);
    TEST_ASSERT_EQUAL(4, s_fake.last_general_cfg.tx_io);
    TEST_ASSERT_EQUAL(5, s_fake.last_general_cfg.rx_io);
    TEST_ASSERT_EQUAL(3, s_fake.last_general_cfg.tx_queue_len);
    TEST_ASSERT_EQUAL(9, s_fake.last_general_cfg.rx_queue_len);
    TEST_ASSERT_EQUAL_HEX32(cfg.alerts_enabled, s_fake.last_general_cfg.alerts_enabled);
    LOG_OUTPUT("install_calls=%d uninstall_calls=%d stop_calls=%d start_calls=%d final bitrate=%lu",
               s_fake.install_calls, s_fake.uninstall_calls, s_fake.stop_calls, s_fake.start_calls,
               (unsigned long)current.bitrate);
    LOG_OUTPUT("general_cfg mode=%d tx_io=%d rx_io=%d tx_queue=%lu rx_queue=%lu alerts=0x%08lX",
               s_fake.last_general_cfg.mode, s_fake.last_general_cfg.tx_io, s_fake.last_general_cfg.rx_io,
               (unsigned long)s_fake.last_general_cfg.tx_queue_len,
               (unsigned long)s_fake.last_general_cfg.rx_queue_len,
               (unsigned long)s_fake.last_general_cfg.alerts_enabled);
}

TEST_CASE("twai port set bitrate from quiesced state restarts and reopens send gate", "[twai_port]")
{
    static const uint8_t payload[] = {0x5A};
    twai_port_cfg_t cfg = make_cfg();
    twai_port_cfg_t current = {0};

    LOG_SECTION("twai_port reconfigure from quiesced");
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_init(&cfg));
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_start());
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_quiesce(pdMS_TO_TICKS(20)));
    TEST_ASSERT_TRUE(twai_port_is_quiesced());

    TEST_ASSERT_EQUAL(ESP_OK, twai_port_set_bitrate_blocking(250000, pdMS_TO_TICKS(30)));
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_get_cfg(&current));
    TEST_ASSERT_TRUE(twai_port_is_running());
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_send_std(0x123, payload, sizeof(payload), pdMS_TO_TICKS(5)));

    TEST_ASSERT_EQUAL_UINT32(250000, current.bitrate);
    TEST_ASSERT_EQUAL(2, s_fake.install_calls);
    TEST_ASSERT_EQUAL(1, s_fake.uninstall_calls);
    TEST_ASSERT_EQUAL(1, s_fake.stop_calls);
    TEST_ASSERT_EQUAL(2, s_fake.start_calls);
    TEST_ASSERT_EQUAL(1, s_fake.transmit_calls);
    LOG_OUTPUT("state=%d bitrate=%lu stop_calls=%d start_calls=%d transmit_calls=%d",
               twai_port_get_state(), (unsigned long)current.bitrate,
               s_fake.stop_calls, s_fake.start_calls, s_fake.transmit_calls);
}

TEST_CASE("twai port set bitrate rejects unsupported bitrate without reconfigure", "[twai_port]")
{
    twai_port_cfg_t cfg = make_cfg();

    LOG_SECTION("twai_port reconfigure validation");
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_init(&cfg));
    TEST_ASSERT_EQUAL(ESP_OK, twai_port_start());
    LOG_INPUT("attempt set_bitrate unsupported=%lu", (unsigned long)333333);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, twai_port_set_bitrate_blocking(333333, pdMS_TO_TICKS(30)));

    TEST_ASSERT_EQUAL(1, s_fake.install_calls);
    TEST_ASSERT_EQUAL(1, s_fake.start_calls);
    TEST_ASSERT_EQUAL(0, s_fake.stop_calls);
    TEST_ASSERT_EQUAL(0, s_fake.uninstall_calls);
    TEST_ASSERT_EQUAL(TWAI_PORT_STATE_RUNNING, twai_port_get_state());
    LOG_ERROR("rc=%s", esp_err_to_name(ESP_ERR_NOT_SUPPORTED));
    LOG_OUTPUT("state=%d install_calls=%d start_calls=%d stop_calls=%d uninstall_calls=%d",
               twai_port_get_state(), s_fake.install_calls, s_fake.start_calls,
               s_fake.stop_calls, s_fake.uninstall_calls);
}
