#include "serial_input.h"

#include <stddef.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

#define PLANNER_UART_PORT    UART_NUM_0
#define PLANNER_UART_BAUD    1000000U
#define PLANNER_UART_RX_BUF  256U

static const char *TAG = "serial_input";

static esp_err_t serial_input_init_backend(void)
{
    const uart_port_t port = PLANNER_UART_PORT;
    uart_config_t uart_cfg = {
        .baud_rate = (int)PLANNER_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err;

    err = uart_param_config(port, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config(UART%d) failed: %s", (int)port, esp_err_to_name(err));
        return err;
    }

    if (!uart_is_driver_installed(port)) {
        err = uart_driver_install(port, (int)(PLANNER_UART_RX_BUF * 2U), 0, 0, NULL, 0);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "uart_driver_install(UART%d) failed: %s", (int)port, esp_err_to_name(err));
            return err;
        }
    }

    ESP_LOGI(TAG,
             "Planner UART ready uart=%d baud=%u rx_buf=%u",
             (int)port,
             (unsigned)PLANNER_UART_BAUD,
             (unsigned)(PLANNER_UART_RX_BUF * 2U));
    return ESP_OK;
}

static int serial_input_read_backend(uint8_t *buf, size_t len, TickType_t timeout_ticks)
{
    return uart_read_bytes(PLANNER_UART_PORT, buf, len, timeout_ticks);
}

static const char *serial_input_backend_name(void)
{
    return "uart";
}

esp_err_t serial_input_init(void)
{
    return serial_input_init_backend();
}

esp_err_t serial_input_read(serial_input_frame_t *out, TickType_t timeout_ticks)
{
    uint8_t byte;
    uint8_t payload[SERIAL_INPUT_FRAME_LEN - 1U];
    size_t  got;
    int     n;

    if (out == NULL) {
        ESP_LOGE(TAG, "serial_input_read called with null output");
        return ESP_ERR_INVALID_ARG;
    }
    do {
        n = serial_input_read_backend(&byte, 1U, timeout_ticks);
        if (n <= 0) {
            ESP_LOGD(TAG,
                     "Timed out waiting for sync byte backend=%s timeout_ticks=%lu",
                     serial_input_backend_name(),
                     (unsigned long)timeout_ticks);
            return ESP_ERR_TIMEOUT;
        }
        if (byte != SERIAL_INPUT_SYNC) {
            ESP_LOGD(TAG, "Discarding unsynced byte backend=%s byte=0x%02X", serial_input_backend_name(), byte);
        }
    } while (byte != SERIAL_INPUT_SYNC);

    got = 0U;
    while (got < sizeof(payload)) {
        n = serial_input_read_backend(payload + got, sizeof(payload) - got, timeout_ticks);
        if (n <= 0) {
            ESP_LOGD(TAG,
                     "Timed out reading payload backend=%s got=%u expected=%u timeout_ticks=%lu",
                     serial_input_backend_name(),
                     (unsigned)got,
                     (unsigned)sizeof(payload),
                     (unsigned long)timeout_ticks);
            return ESP_ERR_TIMEOUT;
        }
        got += (size_t)n;
    }

    out->seq      = payload[0];
    out->throttle = ((uint16_t)payload[1] << 8) | payload[2];
    out->steering = ((uint16_t)payload[3] << 8) | payload[4];
    out->braking  = payload[5];

    ESP_LOGI(TAG,
             "frame backend=%s seq=%u throttle=%u steering=%u braking=%u raw=[0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]",
             serial_input_backend_name(),
             (unsigned)out->seq,
             (unsigned)out->throttle,
             (unsigned)out->steering,
             (unsigned)out->braking,
             SERIAL_INPUT_SYNC,
             payload[0],
             payload[1],
             payload[2],
             payload[3],
             payload[4],
             payload[5]);

    return ESP_OK;
}
