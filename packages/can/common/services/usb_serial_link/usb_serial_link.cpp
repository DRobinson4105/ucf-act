/**
 * @file usb_serial_link.cpp
 * @brief Framed serial transport for the Orin links (UART0 backend).
 *
 * Despite the "usb" prefix (kept so both control↔Orin and safety↔Orin
 * continue to share one API), this runs on UART0 at 1 Mbaud. On the
 * ESP32-C6-DevKitM-1 the "COM" USB-C connector is routed through the
 * onboard CP2102N bridge to UART0, so a USB cable from the board's
 * COM port to the Orin carries this framing. The separate native USB-C
 * ("USB") stays free for `idf.py monitor` via the secondary
 * USB Serial JTAG console.
 */

#include "usb_serial_link.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/semphr.h"

namespace
{

const char *TAG = "usb_serial_link";

constexpr uart_port_t ORIN_LINK_UART_PORT = UART_NUM_0;
constexpr int ORIN_LINK_UART_BAUD = 1000000;
constexpr int ORIN_LINK_RX_BUF_SIZE = 512;

SemaphoreHandle_t s_tx_mutex = nullptr;

TickType_t remaining_ticks(TickType_t start_tick, TickType_t timeout)
{
	if (timeout == portMAX_DELAY)
		return portMAX_DELAY;

	TickType_t elapsed = xTaskGetTickCount() - start_tick;
	return (elapsed >= timeout) ? 0 : (timeout - elapsed);
}

int read_exact(void *buf, uint32_t length, TickType_t timeout)
{
	if (!buf || length == 0)
		return 0;

	uint8_t *dst = static_cast<uint8_t *>(buf);
	uint32_t total = 0;
	TickType_t start = xTaskGetTickCount();

	while (total < length)
	{
		TickType_t wait = remaining_ticks(start, timeout);
		if (timeout != portMAX_DELAY && wait == 0)
			break;

		int n = uart_read_bytes(ORIN_LINK_UART_PORT, dst + total, length - total, wait);
		if (n <= 0)
			break;
		total += (uint32_t)n;
	}

	return (int)total;
}

} // namespace

esp_err_t usb_serial_link_init(void)
{
	if (uart_is_driver_installed(ORIN_LINK_UART_PORT))
	{
		if (!s_tx_mutex)
			s_tx_mutex = xSemaphoreCreateMutex();
		return s_tx_mutex ? ESP_OK : ESP_ERR_NO_MEM;
	}

	uart_config_t cfg = {};
	cfg.baud_rate = ORIN_LINK_UART_BAUD;
	cfg.data_bits = UART_DATA_8_BITS;
	cfg.parity = UART_PARITY_DISABLE;
	cfg.stop_bits = UART_STOP_BITS_1;
	cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	cfg.source_clk = UART_SCLK_DEFAULT;

	esp_err_t err = uart_param_config(ORIN_LINK_UART_PORT, &cfg);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "uart_param_config(UART%d) failed: %s", (int)ORIN_LINK_UART_PORT, esp_err_to_name(err));
		return err;
	}

	err = uart_driver_install(ORIN_LINK_UART_PORT, ORIN_LINK_RX_BUF_SIZE, 0, 0, nullptr, 0);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "uart_driver_install(UART%d) failed: %s", (int)ORIN_LINK_UART_PORT, esp_err_to_name(err));
		return err;
	}

	if (!s_tx_mutex)
		s_tx_mutex = xSemaphoreCreateMutex();
	if (!s_tx_mutex)
		return ESP_ERR_NO_MEM;

	ESP_LOGI(TAG, "Orin link ready on UART%d @ %d baud", (int)ORIN_LINK_UART_PORT, ORIN_LINK_UART_BAUD);
	return ESP_OK;
}

bool usb_serial_link_is_connected(void)
{
	return uart_is_driver_installed(ORIN_LINK_UART_PORT);
}

esp_err_t usb_serial_link_read_message(usb_serial_link_message_t *msg, uint8_t sync_byte, TickType_t sync_timeout,
                                       TickType_t data_timeout)
{
	if (!msg)
		return ESP_ERR_INVALID_ARG;
	if (!uart_is_driver_installed(ORIN_LINK_UART_PORT))
		return ESP_ERR_INVALID_STATE;

	uint8_t byte = 0;
	TickType_t sync_start = xTaskGetTickCount();
	while (true)
	{
		TickType_t wait = remaining_ticks(sync_start, sync_timeout);
		if (sync_timeout != portMAX_DELAY && wait == 0)
			return ESP_ERR_TIMEOUT;

		int n = uart_read_bytes(ORIN_LINK_UART_PORT, &byte, 1, wait);
		if (n <= 0)
			return ESP_ERR_TIMEOUT;
		if (byte == sync_byte)
			break;
	}

	uint8_t header[2] = {};
	if (read_exact(header, sizeof(header), data_timeout) != (int)sizeof(header))
		return ESP_ERR_TIMEOUT;

	msg->type = header[0];
	msg->payload_len = header[1];
	if (msg->payload_len > USB_SERIAL_LINK_MAX_PAYLOAD_LEN)
	{
#ifdef CONFIG_LOG_TRANSPORT_ORIN_LINK_FRAMING
		ESP_LOGW(TAG, "RX frame rejected: payload_len=%u exceeds max=%u (type=0x%02X)",
		         msg->payload_len, USB_SERIAL_LINK_MAX_PAYLOAD_LEN, msg->type);
#endif
		return ESP_ERR_INVALID_SIZE;
	}
	if (msg->payload_len == 0)
	{
#ifdef CONFIG_LOG_TRANSPORT_ORIN_LINK_FRAMING
		ESP_LOGI(TAG, "RX frame: type=0x%02X len=0", msg->type);
#endif
		return ESP_OK;
	}

	if (read_exact(msg->payload, msg->payload_len, data_timeout) != msg->payload_len)
		return ESP_ERR_TIMEOUT;

#ifdef CONFIG_LOG_TRANSPORT_ORIN_LINK_FRAMING
	ESP_LOGI(TAG, "RX frame: type=0x%02X len=%u", msg->type, msg->payload_len);
#endif

	return ESP_OK;
}

esp_err_t usb_serial_link_write_message(const usb_serial_link_message_t *msg, uint8_t sync_byte,
                                        TickType_t write_timeout, TickType_t flush_timeout)
{
	if (!msg)
		return ESP_ERR_INVALID_ARG;
	if (msg->payload_len > USB_SERIAL_LINK_MAX_PAYLOAD_LEN)
		return ESP_ERR_INVALID_SIZE;
	if (!uart_is_driver_installed(ORIN_LINK_UART_PORT))
		return ESP_ERR_INVALID_STATE;
	if (!s_tx_mutex)
		return ESP_ERR_INVALID_STATE;

	uint8_t frame[3 + USB_SERIAL_LINK_MAX_PAYLOAD_LEN] = {sync_byte, msg->type, msg->payload_len};
	for (uint8_t i = 0; i < msg->payload_len; ++i)
		frame[3 + i] = msg->payload[i];

	if (xSemaphoreTake(s_tx_mutex, write_timeout) != pdTRUE)
		return ESP_ERR_TIMEOUT;

	size_t frame_len = (size_t)(3 + msg->payload_len);
	int written = uart_write_bytes(ORIN_LINK_UART_PORT, frame, frame_len);
	esp_err_t flush_err =
	    (written == (int)frame_len) ? uart_wait_tx_done(ORIN_LINK_UART_PORT, flush_timeout) : ESP_FAIL;

	xSemaphoreGive(s_tx_mutex);
	if (written != (int)frame_len)
		return ESP_FAIL;
	return flush_err;
}
