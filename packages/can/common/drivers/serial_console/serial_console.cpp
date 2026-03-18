/**
 * @file serial_console.cpp
 * @brief Non-blocking serial input via USB Serial JTAG.
 */

#include "serial_console.h"

#include "driver/usb_serial_jtag.h"
#include "esp_log.h"

static const char *TAG = "serial_console";

esp_err_t serial_console_init(void)
{
	if (usb_serial_jtag_is_driver_installed())
	{
		ESP_LOGI(TAG, "USB Serial JTAG driver already installed");
		return ESP_OK;
	}

	usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
	esp_err_t err = usb_serial_jtag_driver_install(&cfg);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "USB Serial JTAG driver install failed: %s", esp_err_to_name(err));
		return err;
	}

	ESP_LOGI(TAG, "USB Serial JTAG driver installed");
	return ESP_OK;
}

int serial_console_read_char(void)
{
	uint8_t byte = 0;
	int len = usb_serial_jtag_read_bytes(&byte, 1, 0); // 0 tick = non-blocking
	return (len > 0) ? (int)byte : -1;
}
