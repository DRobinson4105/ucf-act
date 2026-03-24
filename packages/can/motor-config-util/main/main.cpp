/**
 * @file main.cpp
 * @brief One-shot utility to configure UIM2852CA stepper motor baud rate and node ID.
 *
 * The UIM2852CA factory default baud rate is 500 Kbps, but the UCF-ACT CAN bus
 * runs at 1 Mbps. This utility configures baud rate and node ID via CAN.
 *
 * Configuration is done via `idf.py menuconfig` under "Motor Configuration":
 *   - MOTOR_CURRENT_NODE_ID: the node ID the motor currently has
 *   - MOTOR_TARGET_NODE_ID: the node ID to assign (7=steering, 6=braking)
 *   - MOTOR_SET_BAUD_1MBPS: whether to write baud rate to 1 Mbps
 *   - MOTOR_BUS_BAUD_1MBPS: whether to connect at 1 Mbps or 500 Kbps
 *
 * After running, power-cycle the motor for changes to take effect.
 *
 * Usage:
 *   idf.py menuconfig          # set current/target node ID and baud options
 *   idf.py set-target esp32c6
 *   idf.py flash monitor
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/twai.h"

#include "stepper_protocol_uim2852.h"

static const char *TAG = "MOTOR_CFG";

// CAN bus GPIO pins (same on both Safety and Control ESP32-C6 boards)
static constexpr gpio_num_t TWAI_TX_GPIO = GPIO_NUM_4;
static constexpr gpio_num_t TWAI_RX_GPIO = GPIO_NUM_5;

static const char *bitrate_name(uint8_t val)
{
	switch (val)
	{
	case 0:
		return "1 Mbps";
	case 1:
		return "800 Kbps";
	case 2:
		return "500 Kbps";
	case 3:
		return "250 Kbps";
	case 4:
		return "125 Kbps";
	default:
		return "UNKNOWN";
	}
}

// ============================================================================
// TWAI Helpers
// ============================================================================

static esp_err_t twai_init_at(const twai_timing_config_t *timing)
{
	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);
	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

	esp_err_t err = twai_driver_install(&g_config, timing, &f_config);
	if (err != ESP_OK)
		return err;

	err = twai_start();
	if (err != ESP_OK)
	{
		twai_driver_uninstall();
		return err;
	}
	return ESP_OK;
}

static void twai_deinit(void)
{
	twai_stop();
	twai_driver_uninstall();
}

// ============================================================================
// Motor Communication (accepts response from any producer ID)
// ============================================================================

// Send PP set and wait for ACK from any producer.
static bool set_pp(uint8_t consumer_id, uint8_t index, uint8_t value)
{
	uint8_t data[8] = {};
	uint8_t dlc = stepper_uim2852_build_pp_set(data, index, value);

	uint32_t can_id = stepper_uim2852_make_can_id(consumer_id, STEPPER_UIM2852_CW_PP | STEPPER_UIM2852_CW_ACK_BIT);
	twai_message_t msg = {};
	msg.identifier = can_id;
	msg.extd = 1;
	msg.data_length_code = dlc;
	for (int i = 0; i < dlc; i++)
		msg.data[i] = data[i];

	esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "  TX failed: %s", esp_err_to_name(err));
		return false;
	}

	TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
	while (xTaskGetTickCount() < deadline)
	{
		twai_message_t rx = {};
		if (twai_receive(&rx, pdMS_TO_TICKS(50)) != ESP_OK)
			continue;
		if (!rx.extd)
			continue;

		uint8_t producer_id = 0, resp_cw = 0;
		if (stepper_uim2852_parse_can_id(rx.identifier, &producer_id, &resp_cw))
		{
			uint8_t cw_base = stepper_uim2852_cw_base(resp_cw);
			if (cw_base == STEPPER_UIM2852_CW_PP && rx.data_length_code >= 2 && rx.data[0] == index &&
			    rx.data[1] == value)
			{
				ESP_LOGI(TAG, "  ACK from producer %d: PP[%d] = %d", producer_id, index, value);
				return true;
			}
		}
	}
	ESP_LOGE(TAG, "  No ACK received");
	return false;
}

// Query PP[index] and accept response from any producer.
static bool query_pp(uint8_t consumer_id, uint8_t index, uint8_t *out)
{
	uint8_t data[8] = {};
	uint8_t dlc = stepper_uim2852_build_pp_query(data, index);

	uint32_t can_id = stepper_uim2852_make_can_id(consumer_id, STEPPER_UIM2852_CW_PP | STEPPER_UIM2852_CW_ACK_BIT);
	twai_message_t msg = {};
	msg.identifier = can_id;
	msg.extd = 1;
	msg.data_length_code = dlc;
	for (int i = 0; i < dlc; i++)
		msg.data[i] = data[i];

	esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
	if (err != ESP_OK)
		return false;

	TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
	while (xTaskGetTickCount() < deadline)
	{
		twai_message_t rx = {};
		if (twai_receive(&rx, pdMS_TO_TICKS(50)) != ESP_OK)
			continue;
		if (!rx.extd)
			continue;

		uint8_t producer_id = 0, resp_cw = 0;
		if (!stepper_uim2852_parse_can_id(rx.identifier, &producer_id, &resp_cw))
			continue;

		uint8_t cw_base = stepper_uim2852_cw_base(resp_cw);
		if (cw_base == STEPPER_UIM2852_CW_PP && rx.data_length_code >= 2 && rx.data[0] == index)
		{
			*out = rx.data[1];
			return true;
		}
	}
	return false;
}

// ============================================================================
// Main
// ============================================================================

extern "C" void app_main(void)
{
	static constexpr uint8_t CURRENT_ID = CONFIG_MOTOR_CURRENT_NODE_ID;
	static constexpr uint8_t TARGET_ID = CONFIG_MOTOR_TARGET_NODE_ID;

#ifdef CONFIG_MOTOR_BUS_BAUD_1MBPS
	const char *bus_baud_label = "1 Mbps";
	twai_timing_config_t bus_timing = TWAI_TIMING_CONFIG_1MBITS();
#else
	const char *bus_baud_label = "500 Kbps";
	twai_timing_config_t bus_timing = TWAI_TIMING_CONFIG_500KBITS();
#endif

	ESP_LOGI(TAG, "========================================");
	ESP_LOGI(TAG, "  UIM2852CA Motor Configuration Utility");
	ESP_LOGI(TAG, "  Bus baud:    %s", bus_baud_label);
	ESP_LOGI(TAG, "  Current ID:  %d", CURRENT_ID);
	ESP_LOGI(TAG, "  Target ID:   %d", TARGET_ID);
#ifdef CONFIG_MOTOR_SET_BAUD_1MBPS
	ESP_LOGI(TAG, "  Set baud:    1 Mbps");
#else
	ESP_LOGI(TAG, "  Set baud:    (skip)");
#endif
	ESP_LOGI(TAG, "========================================");

	vTaskDelay(pdMS_TO_TICKS(500));

	esp_err_t err = twai_init_at(&bus_timing);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "TWAI init failed: %s", esp_err_to_name(err));
		return;
	}
	vTaskDelay(pdMS_TO_TICKS(200));

	// Probe: can we reach the motor?
	ESP_LOGI(TAG, "Probing motor at node %d...", CURRENT_ID);
	uint8_t current_rate = 0xFF;
	if (!query_pp(CURRENT_ID, STEPPER_UIM2852_PP_BITRATE, &current_rate))
	{
		ESP_LOGE(TAG, "No response from node %d. Check wiring and bus baud setting.", CURRENT_ID);
		twai_deinit();
		return;
	}
	ESP_LOGI(TAG, "Motor responded. Current baud rate: %s (PP[5]=%d)", bitrate_name(current_rate), current_rate);

	uint8_t current_node_id = 0;
	if (query_pp(CURRENT_ID, STEPPER_UIM2852_PP_NODE_ID, &current_node_id))
		ESP_LOGI(TAG, "Motor reports node ID: %d", current_node_id);

	bool success = true;

	// Set baud rate if requested
#ifdef CONFIG_MOTOR_SET_BAUD_1MBPS
	if (current_rate == 0)
	{
		ESP_LOGI(TAG, "Baud rate already 1 Mbps — skipping");
	}
	else
	{
		ESP_LOGI(TAG, "Setting baud rate to 1 Mbps (PP[5]=0)...");
		if (set_pp(CURRENT_ID, STEPPER_UIM2852_PP_BITRATE, 0))
		{
			uint8_t verify = 0xFF;
			if (query_pp(CURRENT_ID, STEPPER_UIM2852_PP_BITRATE, &verify) && verify == 0)
				ESP_LOGI(TAG, "Verified: PP[5]=0 (1 Mbps)");
			else
				ESP_LOGW(TAG, "Readback mismatch — may still work after power cycle");
		}
		else
		{
			ESP_LOGE(TAG, "FAILED to set baud rate");
			success = false;
		}
	}
#endif

	// Set node ID
	if (current_node_id == TARGET_ID)
	{
		ESP_LOGI(TAG, "Node ID already %d — skipping", TARGET_ID);
	}
	else
	{
		ESP_LOGI(TAG, "Setting node ID to %d (PP[7]=%d)...", TARGET_ID, TARGET_ID);
		if (set_pp(CURRENT_ID, STEPPER_UIM2852_PP_NODE_ID, TARGET_ID))
		{
			uint8_t verify_id = 0;
			if (query_pp(CURRENT_ID, STEPPER_UIM2852_PP_NODE_ID, &verify_id) && verify_id == TARGET_ID)
				ESP_LOGI(TAG, "Verified: node ID = %d", TARGET_ID);
			else
				ESP_LOGW(TAG, "Readback mismatch — may still work after power cycle");
		}
		else
		{
			ESP_LOGE(TAG, "FAILED to set node ID");
			success = false;
		}
	}

	// Send reboot+save command
	ESP_LOGI(TAG, "Sending reboot+save command (SY, D0=1) to node %d...", CURRENT_ID);
	{
		uint8_t sy_data[1] = {0x01};
		uint32_t sy_id = stepper_uim2852_make_can_id(CURRENT_ID, STEPPER_UIM2852_CW_SY);
		twai_message_t msg = {};
		msg.identifier = sy_id;
		msg.extd = 1;
		msg.data_length_code = 1;
		msg.data[0] = sy_data[0];
		esp_err_t sy_err = twai_transmit(&msg, pdMS_TO_TICKS(100));
		if (sy_err == ESP_OK)
			ESP_LOGI(TAG, "Reboot+save command sent");
		else
			ESP_LOGW(TAG, "Reboot+save TX failed: %s (motor may need manual power cycle)", esp_err_to_name(sy_err));
	}

	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "========================================");
	if (success)
	{
		ESP_LOGI(TAG, "  CONFIGURATION COMPLETE");
		ESP_LOGI(TAG, "  Motor should reboot automatically.");
		ESP_LOGI(TAG, "  If not, power-cycle the motor now.");
	}
	else
	{
		ESP_LOGE(TAG, "  CONFIGURATION FAILED (see errors above)");
	}
	ESP_LOGI(TAG, "========================================");

	twai_deinit();
}
