/**
 * @file usb_serial_link.h
 * @brief Framed serial transport for the Orin links.
 *
 * Runs on UART0 at 1 Mbaud. On the ESP32-C6-DevKitM-1 the board's
 * "COM" USB-C connector is wired through the onboard CP2102N bridge
 * to UART0, so plugging the COM port into the Orin carries this
 * framing. The separate native USB-C ("USB") is left free for
 * `idf.py monitor` via the secondary USB Serial JTAG console.
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define USB_SERIAL_LINK_MAX_PAYLOAD_LEN 8

typedef struct
{
	uint8_t type;
	uint8_t payload_len;
	uint8_t payload[USB_SERIAL_LINK_MAX_PAYLOAD_LEN];
} usb_serial_link_message_t;

esp_err_t usb_serial_link_init(void);
bool usb_serial_link_is_connected(void);
esp_err_t usb_serial_link_read_message(usb_serial_link_message_t *msg, uint8_t sync_byte, TickType_t sync_timeout,
                                       TickType_t data_timeout);
esp_err_t usb_serial_link_write_message(const usb_serial_link_message_t *msg, uint8_t sync_byte,
                                        TickType_t write_timeout, TickType_t flush_timeout);

#ifdef __cplusplus
}
#endif
