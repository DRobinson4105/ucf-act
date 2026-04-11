#include "serial_input.h"

#include <stddef.h>

#include "sdkconfig.h"
#if CONFIG_ESP_CONSOLE_UART
#include "driver/uart.h"
#endif
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#endif
#include "esp_err.h"
#include "esp_log.h"

#define RX_BUF_SIZE 256
#define TX_BUF_SIZE 64

static const char *TAG = "serial_input";

#if CONFIG_ESP_CONSOLE_UART
static esp_err_t serial_input_init_backend(void)
{
    const uart_port_t port = (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM;

    if (uart_is_driver_installed(port)) {
        ESP_LOGI(TAG, "Using UART%d input on existing console driver", (int)port);
        return ESP_OK;
    }

    esp_err_t err = uart_driver_install(port, RX_BUF_SIZE, 0, 0, NULL, 0);

    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "UART%d driver installed rx_buf=%u baud=%u",
                 (int)port,
                 (unsigned)RX_BUF_SIZE,
                 (unsigned)CONFIG_ESP_CONSOLE_UART_BAUDRATE);
    } else {
        ESP_LOGE(TAG, "uart_driver_install(UART%d) failed: %s", (int)port, esp_err_to_name(err));
    }

    return err;
}

static int serial_input_read_backend(uint8_t *buf, size_t len, TickType_t timeout_ticks)
{
    return uart_read_bytes((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM, buf, len, timeout_ticks);
}

static const char *serial_input_backend_name(void)
{
    return "uart";
}
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG
static esp_err_t serial_input_init_backend(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = RX_BUF_SIZE,
        .tx_buffer_size = TX_BUF_SIZE,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);

    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "Using USB Serial JTAG input rx_buf=%u tx_buf=%u",
                 (unsigned)RX_BUF_SIZE,
                 (unsigned)TX_BUF_SIZE);
    } else {
        ESP_LOGE(TAG, "usb_serial_jtag_driver_install failed: %s", esp_err_to_name(err));
    }

    return err;
}

static int serial_input_read_backend(uint8_t *buf, size_t len, TickType_t timeout_ticks)
{
    return usb_serial_jtag_read_bytes(buf, (uint32_t)len, timeout_ticks);
}

static const char *serial_input_backend_name(void)
{
    return "usb_serial_jtag";
}
#else
#error "serial_input requires a supported console backend"
#endif

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
