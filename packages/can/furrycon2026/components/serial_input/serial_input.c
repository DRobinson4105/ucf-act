#include "serial_input.h"

#include <stddef.h>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"

#define RX_BUF_SIZE 256
#define TX_BUF_SIZE 64

esp_err_t serial_input_init(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = RX_BUF_SIZE,
        .tx_buffer_size = TX_BUF_SIZE,
    };
    return usb_serial_jtag_driver_install(&cfg);
}

esp_err_t serial_input_read(serial_input_frame_t *out, TickType_t timeout_ticks)
{
    uint8_t byte;
    uint8_t payload[SERIAL_INPUT_FRAME_LEN - 1U];
    size_t  got;
    int     n;

    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    do {
        n = usb_serial_jtag_read_bytes(&byte, 1, timeout_ticks);
        if (n <= 0) {
            return ESP_ERR_TIMEOUT;
        }
    } while (byte != SERIAL_INPUT_SYNC);

    got = 0U;
    while (got < sizeof(payload)) {
        n = usb_serial_jtag_read_bytes(payload + got, (uint32_t)(sizeof(payload) - got), timeout_ticks);
        if (n <= 0) {
            return ESP_ERR_TIMEOUT;
        }
        got += (size_t)n;
    }

    out->seq      = payload[0];
    out->throttle = ((uint16_t)payload[1] << 8) | payload[2];
    out->steering = ((uint16_t)payload[3] << 8) | payload[4];
    out->braking  = payload[5];

    return ESP_OK;
}
