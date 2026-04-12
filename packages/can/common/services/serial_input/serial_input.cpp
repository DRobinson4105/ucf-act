/**
 * @file serial_input.cpp
 * @brief Non-blocking serial input via USB Serial JTAG.
 */

#include "serial_input.h"

#include "driver/usb_serial_jtag.h"
#include "esp_log.h"

static const char *TAG = "serial_input";

esp_err_t serial_input_init(void)
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

int serial_input_read_char(void)
{
	uint8_t byte = 0;
	int len = usb_serial_jtag_read_bytes(&byte, 1, 0);
	return (len > 0) ? (int)byte : -1;
}

bool serial_input_accum_feed(serial_input_accum_t *accum, int ch)
{
	if (!accum || ch < 0)
		return false;

	if (ch >= '0' && ch <= '9')
	{
		if (accum->value < 0)
			accum->value = 0;
		accum->value = accum->value * 10 + (ch - '0');
		if (accum->value > accum->max)
			accum->value = accum->max;
		return false;
	}

	if ((ch == ' ' || ch == '\r' || ch == '\n') && accum->value >= 0)
		return true;

	if (accum->value >= 0)
		accum->value = -1;

	return false;
}

void serial_input_accum_reset(serial_input_accum_t *accum)
{
	if (accum)
		accum->value = -1;
}
