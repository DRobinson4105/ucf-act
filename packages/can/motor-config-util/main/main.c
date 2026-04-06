#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"

static const char *TAG = "sc3_sender";

#define PRODUCER_ID  4
#define CONSUMER_ID  0

_Static_assert(PRODUCER_ID == 4, "producer_id must be 4");
_Static_assert(CONSUMER_ID == 0, "consumer_id must be 0");

/*
 * SimpleCAN 3.0 CAN-ID encoding (29-bit extended frame):
 *   SID = ((consumer_id << 1) & 0x003F) | 0x0100
 *   EID = (((consumer_id << 1) & 0x00C0) << 8) | cw
 *   CAN-ID = (SID << 18) | EID
 */
static uint32_t make_can_id(uint8_t consumer_id, uint8_t cw)
{
    uint32_t s = (uint32_t)(consumer_id << 1);
    return (((s & 0x003Fu) | 0x0100u) << 18) | (((s & 0x00C0u) << 8) | cw);
}

static void log_alerts(void)
{
    uint32_t alerts;
    twai_read_alerts(&alerts, 0);
    if (alerts) {
        ESP_LOGW(TAG, "TWAI alerts: 0x%08lX%s%s%s%s%s%s%s%s%s%s%s",
                 alerts,
                 (alerts & TWAI_ALERT_BUS_OFF)         ? " BUS_OFF"         : "",
                 (alerts & TWAI_ALERT_BUS_RECOVERED)   ? " BUS_RECOVERED"   : "",
                 (alerts & TWAI_ALERT_BUS_ERROR)       ? " BUS_ERROR"       : "",
                 (alerts & TWAI_ALERT_TX_FAILED)       ? " TX_FAILED"       : "",
                 (alerts & TWAI_ALERT_TX_SUCCESS)      ? " TX_SUCCESS"      : "",
                 (alerts & TWAI_ALERT_RX_DATA)         ? " RX_DATA"         : "",
                 (alerts & TWAI_ALERT_ERR_PASS)        ? " ERR_PASS"        : "",
                 (alerts & TWAI_ALERT_ABOVE_ERR_WARN)  ? " ABOVE_ERR_WARN"  : "",
                 (alerts & TWAI_ALERT_BELOW_ERR_WARN)  ? " BELOW_ERR_WARN"  : "",
                 (alerts & TWAI_ALERT_ERR_ACTIVE)      ? " ERR_ACTIVE"      : "",
                 (alerts & TWAI_ALERT_TX_RETRIED)      ? " TX_RETRIED"      : "");
    }
}

static void log_frame(const twai_message_t *msg, const char *dir)
{
    char buf[32];
    int pos = 0;
    for (int i = 0; i < msg->data_length_code && i < 8; i++) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%s0x%02X", i ? " " : "", msg->data[i]);
    }
    ESP_LOGI(TAG, "%s  ID=0x%08lX  DLC=%u  data={%s}",
             dir, msg->identifier, msg->data_length_code, buf);
}

static void send_frame(uint8_t consumer_id, uint8_t cw, const uint8_t *data, uint8_t dlc)
{
    twai_message_t msg = {
        .flags             = TWAI_MSG_FLAG_EXTD,
        .identifier        = make_can_id(consumer_id, cw),
        .data_length_code  = dlc,
    };
    memcpy(msg.data, data, dlc);
    log_frame(&msg, "TX");
    ESP_ERROR_CHECK(twai_transmit(&msg, pdMS_TO_TICKS(100)));
}

/* Returns true if an ack frame with the expected DLC was received within timeout_ms. */
static bool recv_ack(uint8_t expected_dlc, uint32_t timeout_ms)
{
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(timeout_ms)) != ESP_OK) {
        return false;
    }
    log_frame(&rx, "RX");
    if (rx.data_length_code != expected_dlc) {
        ESP_LOGW(TAG, "ack DLC mismatch: expected %u got %u", expected_dlc, rx.data_length_code);
        return false;
    }
    return true;
}

static void set_baud(uint32_t kbps)
{
    ESP_ERROR_CHECK(twai_stop());
    ESP_ERROR_CHECK(twai_driver_uninstall());

    twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(CONFIG_CAN_TX_GPIO, CONFIG_CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_filter_config_t  f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_timing_config_t  t_cfg = (kbps == 500) ? (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS()
                                                 : (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();

    ESP_ERROR_CHECK(twai_driver_install(&g_cfg, &t_cfg, &f_cfg));
    ESP_ERROR_CHECK(twai_start());
    ESP_ERROR_CHECK(twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL));
    ESP_LOGI(TAG, "TWAI set to %lu kbps", kbps);
}

void app_main(void)
{
    /* ---- 500 kbps ---- */
    {
        twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(CONFIG_CAN_TX_GPIO, CONFIG_CAN_RX_GPIO, TWAI_MODE_NORMAL);
        twai_timing_config_t  t_cfg = TWAI_TIMING_CONFIG_500KBITS();
        twai_filter_config_t  f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        ESP_ERROR_CHECK(twai_driver_install(&g_cfg, &t_cfg, &f_cfg));
        ESP_ERROR_CHECK(twai_start());
        ESP_ERROR_CHECK(twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL));
        ESP_LOGI(TAG, "TWAI started at 500 kbps  producer=%u consumer=%u", PRODUCER_ID, CONSUMER_ID);
    }

    /* cw=0x81, ins=[5, 0], optional ack DLC=2 */
    {
        uint8_t data[] = {5, 0};
        send_frame(CONSUMER_ID, 0x81, data, sizeof(data));
        log_alerts();
        if (!recv_ack(2, 500)) {
            ESP_LOGI(TAG, "cw=0x81: no ack (expected occasionally)");
        }
    }

    /* cw=0x7E, ins=[1], no ack */
    {
        uint8_t data[] = {1};
        send_frame(CONSUMER_ID, 0x7E, data, sizeof(data));
        log_alerts();
    }

    /* ---- 1000 kbps ---- */
    set_baud(1000);
    vTaskDelay(pdMS_TO_TICKS(2000)); /* wait for motor to reboot at new bit rate */

    /* cw=0x81, ins=[5], ack DLC=2 */
    {
        uint8_t data[] = {5};
        send_frame(CONSUMER_ID, 0x81, data, sizeof(data));
        log_alerts();
        ESP_ERROR_CHECK(recv_ack(2, 500) ? ESP_OK : ESP_ERR_TIMEOUT);
    }

    /* cw=0x81, ins=[7, d1], ack DLC=2 — d1 depends on motor target */
    {
        uint8_t data[] = {7, CONFIG_MOTOR_TARGET == 1 ? 6 : 7};
        send_frame(CONSUMER_ID, 0x81, data, sizeof(data));
        log_alerts();
        ESP_ERROR_CHECK(recv_ack(2, 500) ? ESP_OK : ESP_ERR_TIMEOUT);
    }

    ESP_LOGI(TAG, "motor target set to %d (%s)",
             CONFIG_MOTOR_TARGET, CONFIG_MOTOR_TARGET == 1 ? "braking" : "steering");

    /* cw=0x21, no ack */
    {
        twai_message_t msg = {
            .flags            = TWAI_MSG_FLAG_EXTD,
            .identifier       = make_can_id(CONSUMER_ID, 0x21),
            .data_length_code = 0,
        };
        log_frame(&msg, "TX");
        ESP_ERROR_CHECK(twai_transmit(&msg, pdMS_TO_TICKS(100)));
        log_alerts();
    }

    /* cw=0x7E, ins=[1], no ack — repeated at 1000 kbps */
    {
        uint8_t data[] = {1};
        send_frame(CONSUMER_ID, 0x7E, data, sizeof(data));
        log_alerts();
    }

#if CONFIG_FACTORY_RESET
    /* cw=0x7E, ins=[2], no ack — factory reset command */
    {
        uint8_t data[] = {2};
        send_frame(CONSUMER_ID, 0x7E, data, sizeof(data));
        log_alerts();
    }

    /* cw=0x7E, ins=[1], no ack — save after factory reset */
    {
        uint8_t data[] = {1};
        send_frame(CONSUMER_ID, 0x7E, data, sizeof(data));
        log_alerts();
    }
    ESP_LOGI(TAG, "factory reset sent");
#endif

    ESP_LOGI(TAG, "Done");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
