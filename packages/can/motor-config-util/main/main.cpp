/**
 * @file main.cpp
 * @brief One-shot utility to configure UIM2852CA stepper motor CAN baud rate.
 *
 * The UIM2852CA factory default baud rate is 500 Kbps, but the UCF-ACT CAN bus
 * runs at 1 Mbps.  This utility:
 *
 *   1. Initializes TWAI at 500 Kbps (factory default)
 *   2. Queries both motors' current baud rate setting
 *   3. Sets baud rate to 1 Mbps (PP[5] = 0) on each motor
 *   4. Verifies the write by reading back
 *   5. Instructs you to power-cycle the motors (baud rate takes effect on reboot)
 *
 * Flash onto the Control ESP32 (or Safety ESP32 — same GPIO 4/5 CAN pins).
 * After running, flash the normal firmware back.
 *
 * Usage:
 *   idf.py -p /dev/cu.usbmodem* set-target esp32c6
 *   idf.py -p /dev/cu.usbmodem* flash monitor
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/twai.h"

#include "stepper_protocol_uim2852.h"

static const char *TAG = "MOTOR_CFG";

// CAN bus GPIO pins (same on both Safety and Control ESP32-C6 boards)
static constexpr gpio_num_t TWAI_TX_GPIO = GPIO_NUM_4;
static constexpr gpio_num_t TWAI_RX_GPIO = GPIO_NUM_5;

// Motor node IDs — address the motor at its current node ID.
// Both motors were set to node 6 (braking). This one needs to become node 7.
static constexpr uint8_t MOTORS[] = {6};
static const char *MOTOR_NAMES[] = {"MOTOR (currently node 6, changing to 7)"};

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

// Send an extended CAN frame and return ESP_OK/error.
static esp_err_t send_frame(uint8_t motor_id, uint8_t cw, const uint8_t *data, uint8_t dlc)
{
	twai_message_t msg = {};
	msg.identifier = stepper_uim2852_make_can_id(motor_id, cw);
	msg.extd = 1;
	msg.data_length_code = dlc;
	for (int i = 0; i < dlc && i < 8; i++)
		msg.data[i] = data[i];
	return twai_transmit(&msg, pdMS_TO_TICKS(100));
}

// Wait for a response from a specific motor with a specific CW.
// Returns true if response received, fills resp_data and resp_dlc.
static bool wait_response(uint8_t expected_motor_id, uint8_t expected_cw,
                          uint8_t *resp_data, uint8_t *resp_dlc, int timeout_ms)
{
	TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

	while (xTaskGetTickCount() < deadline)
	{
		twai_message_t rx = {};
		if (twai_receive(&rx, pdMS_TO_TICKS(50)) != ESP_OK)
			continue;

		if (!rx.extd)
			continue;

		uint8_t producer_id = 0;
		uint8_t cw = 0;
		if (!stepper_uim2852_parse_can_id(rx.identifier, &producer_id, &cw))
			continue;

		if (producer_id == expected_motor_id && cw == expected_cw)
		{
			if (resp_dlc)
				*resp_dlc = rx.data_length_code;
			if (resp_data)
			{
				for (int i = 0; i < rx.data_length_code && i < 8; i++)
					resp_data[i] = rx.data[i];
			}
			return true;
		}
	}
	return false;
}

// Query PP[index] from a motor.  Returns true if successful, value stored in *out.
static bool query_pp(uint8_t motor_id, uint8_t index, uint8_t *out)
{
	uint8_t data[8] = {};
	uint8_t dlc = stepper_uim2852_build_pp_query(data, index);

	// Request with ACK
	esp_err_t err = send_frame(motor_id, STEPPER_UIM2852_CW_PP | STEPPER_UIM2852_CW_ACK_BIT, data, dlc);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "  TX failed: %s", esp_err_to_name(err));
		return false;
	}

	uint8_t resp[8] = {};
	uint8_t resp_dlc = 0;
	if (!wait_response(motor_id, STEPPER_UIM2852_CW_PP | STEPPER_UIM2852_CW_ACK_BIT, resp, &resp_dlc, 500))
	{
		ESP_LOGE(TAG, "  No response (timeout)");
		return false;
	}

	// PP query response: DL=2, d0=index, d1=value
	if (resp_dlc >= 2 && resp[0] == index)
	{
		*out = resp[1];
		return true;
	}

	ESP_LOGE(TAG, "  Unexpected response: dlc=%d d0=0x%02X", resp_dlc, resp[0]);
	return false;
}

// Set PP[index] = value on a motor.  Returns true if ACK received.
static bool set_pp(uint8_t motor_id, uint8_t index, uint8_t value)
{
	uint8_t data[8] = {};
	uint8_t dlc = stepper_uim2852_build_pp_set(data, index, value);

	esp_err_t err = send_frame(motor_id, STEPPER_UIM2852_CW_PP | STEPPER_UIM2852_CW_ACK_BIT, data, dlc);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "  TX failed: %s", esp_err_to_name(err));
		return false;
	}

	uint8_t resp[8] = {};
	uint8_t resp_dlc = 0;
	if (!wait_response(motor_id, STEPPER_UIM2852_CW_PP | STEPPER_UIM2852_CW_ACK_BIT, resp, &resp_dlc, 500))
	{
		ESP_LOGE(TAG, "  No ACK (timeout)");
		return false;
	}

	// PP set ACK: DL=2, d0=index, d1=value (echo)
	if (resp_dlc >= 2 && resp[0] == index && resp[1] == value)
		return true;

	ESP_LOGW(TAG, "  ACK mismatch: expected [%d,%d] got [%d,%d]", index, value, resp[0], resp[1]);
	return false;
}

// All baud rates to scan, in order: 500K, 250K, 125K, 800K, 1M
struct baud_entry
{
	const char *name;
	twai_timing_config_t timing;
	uint8_t pp_value; // corresponding PP[5] value
};

static const baud_entry BAUD_TABLE[] = {
	{"500 Kbps", TWAI_TIMING_CONFIG_500KBITS(), 2},
	{"250 Kbps", TWAI_TIMING_CONFIG_250KBITS(), 3},
	{"125 Kbps", TWAI_TIMING_CONFIG_125KBITS(), 4},
	{"800 Kbps", TWAI_TIMING_CONFIG_800KBITS(), 1},
	{"1 Mbps", TWAI_TIMING_CONFIG_1MBITS(), 0},
};
static constexpr int BAUD_COUNT = sizeof(BAUD_TABLE) / sizeof(BAUD_TABLE[0]);

static esp_err_t twai_init_at(const twai_timing_config_t *timing, twai_mode_t mode = TWAI_MODE_NORMAL)
{
	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, mode);
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

// Try to query a motor at the current baud rate. Returns true if it responds.
static bool probe_motor(uint8_t motor_id)
{
	uint8_t dummy = 0;
	return query_pp(motor_id, STEPPER_UIM2852_PP_BITRATE, &dummy);
}

// Configure a single motor to 1 Mbps. Returns true on success.
static bool configure_motor(uint8_t motor_id, const char *name)
{
	// Step 1: Query current baud rate
	ESP_LOGI(TAG, "Querying current baud rate (PP[5])...");
	uint8_t current_rate = 0xFF;
	if (!query_pp(motor_id, STEPPER_UIM2852_PP_BITRATE, &current_rate))
	{
		ESP_LOGE(TAG, "  Motor responded to probe but failed query");
		return false;
	}
	ESP_LOGI(TAG, "Current baud rate: %s (value %d)", bitrate_name(current_rate), current_rate);

	if (current_rate == 0)
	{
		ESP_LOGI(TAG, "Already at 1 Mbps — skipping");
		return true;
	}

	// Step 2: Set baud rate to 1 Mbps
	ESP_LOGI(TAG, "Setting baud rate to 1 Mbps (PP[5] = 0)...");
	if (!set_pp(motor_id, STEPPER_UIM2852_PP_BITRATE, 0))
	{
		ESP_LOGE(TAG, "FAILED to set baud rate on %s", name);
		return false;
	}
	ESP_LOGI(TAG, "Set command acknowledged");

	// Step 3: Verify by reading back
	ESP_LOGI(TAG, "Verifying...");
	uint8_t verify_rate = 0xFF;
	if (query_pp(motor_id, STEPPER_UIM2852_PP_BITRATE, &verify_rate) && verify_rate == 0)
	{
		ESP_LOGI(TAG, "Verified: baud rate set to 1 Mbps");
		return true;
	}

	ESP_LOGW(TAG, "Readback returned %d — may still work after power cycle", verify_rate);
	return true; // ACK was received, assume it worked
}

// Send a frame and log ANY response (no filtering).
static void send_and_log_any_response(uint8_t motor_id, uint8_t cw, const uint8_t *data, uint8_t dlc,
                                       const char *label, int timeout_ms)
{
	uint32_t can_id = stepper_uim2852_make_can_id(motor_id, cw);
	ESP_LOGI(TAG, "  TX %s: id=0x%08lX extd=1 dlc=%d data=[%02X %02X]",
	         label, (unsigned long)can_id, dlc, data[0], data[1]);

	twai_message_t msg = {};
	msg.identifier = can_id;
	msg.extd = 1;
	msg.data_length_code = dlc;
	for (int i = 0; i < dlc && i < 8; i++)
		msg.data[i] = data[i];

	esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "  TX failed: %s", esp_err_to_name(err));
		return;
	}
	ESP_LOGI(TAG, "  TX sent (ACK received from bus)");

	// Listen for ANY response
	TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
	int rx_count = 0;
	while (xTaskGetTickCount() < deadline)
	{
		twai_message_t rx = {};
		if (twai_receive(&rx, pdMS_TO_TICKS(50)) != ESP_OK)
			continue;

		rx_count++;
		ESP_LOGI(TAG, "  RX #%d: id=0x%08lX extd=%d dlc=%d data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
		         rx_count, (unsigned long)rx.identifier, rx.extd, rx.data_length_code,
		         rx.data[0], rx.data[1], rx.data[2], rx.data[3],
		         rx.data[4], rx.data[5], rx.data[6], rx.data[7]);

		if (rx.extd)
		{
			uint8_t producer_id = 0, resp_cw = 0;
			if (stepper_uim2852_parse_can_id(rx.identifier, &producer_id, &resp_cw))
				ESP_LOGI(TAG, "         -> UIM2852: producer=%d CW=0x%02X", producer_id, resp_cw);
			else
				ESP_LOGI(TAG, "         -> Not a valid UIM2852 ID");
		}
	}

	if (rx_count == 0)
		ESP_LOGW(TAG, "  No response frames received");
}

// Phase 1: Passive listen — see if the motor is transmitting anything.
// Uses TWAI_MODE_LISTEN_ONLY so we don't transmit or affect the bus.
static void listen_phase(void)
{
	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "=== PHASE 1: Passive Bus Listener ===");
	ESP_LOGI(TAG, "Listening for ANY CAN traffic at each baud rate...");
	ESP_LOGI(TAG, "(If motor is transmitting, we'll see frames here)");

	for (int b = 0; b < BAUD_COUNT; b++)
	{
		ESP_LOGI(TAG, "");
		ESP_LOGI(TAG, "Listening at %s for 2 seconds...", BAUD_TABLE[b].name);

		esp_err_t err = twai_init_at(&BAUD_TABLE[b].timing, TWAI_MODE_LISTEN_ONLY);
		if (err != ESP_OK)
		{
			ESP_LOGE(TAG, "  TWAI init failed: %s", esp_err_to_name(err));
			continue;
		}

		int rx_count = 0;
		TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);

		while (xTaskGetTickCount() < deadline)
		{
			twai_message_t rx = {};
			if (twai_receive(&rx, pdMS_TO_TICKS(100)) == ESP_OK)
			{
				rx_count++;
				// Log first few frames in detail
				if (rx_count <= 5)
				{
					ESP_LOGI(TAG, "  RX #%d: id=0x%08lX extd=%d dlc=%d data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
					         rx_count, (unsigned long)rx.identifier, rx.extd, rx.data_length_code,
					         rx.data[0], rx.data[1], rx.data[2], rx.data[3],
					         rx.data[4], rx.data[5], rx.data[6], rx.data[7]);

					if (rx.extd)
					{
						uint8_t producer_id = 0, cw = 0;
						if (stepper_uim2852_parse_can_id(rx.identifier, &producer_id, &cw))
						{
							ESP_LOGI(TAG, "         -> UIM2852 frame: producer=%d CW=0x%02X", producer_id, cw);
						}
					}
				}
			}
		}

		// Also check bus status
		twai_status_info_t status = {};
		twai_get_status_info(&status);

		if (rx_count > 0)
		{
			ESP_LOGI(TAG, "  FOUND %d frames at %s!", rx_count, BAUD_TABLE[b].name);
		}
		else
		{
			ESP_LOGI(TAG, "  No frames received (TX_err=%lu RX_err=%lu bus_off=%d)",
			         (unsigned long)status.tx_error_counter,
			         (unsigned long)status.rx_error_counter,
			         status.state == TWAI_STATE_BUS_OFF ? 1 : 0);
		}

		twai_deinit();
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

// Phase 2: Focused 500K probe — motors ARE ACKing at this rate.
// Send queries to various node IDs and log ALL responses.
static void probe_phase(void)
{
	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "=== PHASE 2: Focused 500K Probe ===");
	ESP_LOGI(TAG, "Motors are ACKing at 500K. Testing queries...");

	esp_err_t err = twai_init_at(&BAUD_TABLE[0].timing); // 500K
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "TWAI init failed");
		return;
	}
	vTaskDelay(pdMS_TO_TICKS(200));

	uint8_t data[8] = {};

	// Test 1: PP query to factory default node 5 (with ACK bit)
	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "--- Test 1: PP[5] query to node 5 (with ACK) ---");
	uint8_t dlc = stepper_uim2852_build_pp_query(data, STEPPER_UIM2852_PP_BITRATE);
	send_and_log_any_response(5, STEPPER_UIM2852_CW_PP | STEPPER_UIM2852_CW_ACK_BIT, data, dlc,
	                           "PP query node=5", 1000);

	// Test 2: PP query to node 5 (WITHOUT ACK bit)
	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "--- Test 2: PP[5] query to node 5 (no ACK bit) ---");
	dlc = stepper_uim2852_build_pp_query(data, STEPPER_UIM2852_PP_BITRATE);
	send_and_log_any_response(5, STEPPER_UIM2852_CW_PP, data, dlc,
	                           "PP query node=5 no-ack", 1000);

	// Test 3: PP query to global broadcast ID 0
	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "--- Test 3: PP[5] query to broadcast node 0 ---");
	dlc = stepper_uim2852_build_pp_query(data, STEPPER_UIM2852_PP_BITRATE);
	send_and_log_any_response(0, STEPPER_UIM2852_CW_PP | STEPPER_UIM2852_CW_ACK_BIT, data, dlc,
	                           "PP query broadcast", 1000);

	// Test 4: MS (motion status) query to node 5
	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "--- Test 4: MS[0] status query to node 5 ---");
	memset(data, 0, 8);
	data[0] = 0; // MS index 0
	send_and_log_any_response(5, 0x11 | STEPPER_UIM2852_CW_ACK_BIT, data, 1,
	                           "MS query node=5", 1000);

	// Test 5: ML (model string) query to node 5
	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "--- Test 5: ML model string query to node 5 ---");
	memset(data, 0, 8);
	send_and_log_any_response(5, 0x0B | STEPPER_UIM2852_CW_ACK_BIT, data, 0,
	                           "ML query node=5", 1000);

	// Test 6: Try standard (11-bit) CAN frame in case motors don't use extended
	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "--- Test 6: Standard 11-bit CAN frame (0x605, SDO read) ---");
	{
		// CANopen-style SDO read of object 0x1000 (device type) — in case motors use CANopen
		twai_message_t msg = {};
		msg.identifier = 0x600 + 5; // SDO TX to node 5
		msg.extd = 0;               // standard 11-bit
		msg.data_length_code = 8;
		msg.data[0] = 0x40; // SDO upload request
		msg.data[1] = 0x00; // index low
		msg.data[2] = 0x10; // index high (0x1000)
		msg.data[3] = 0x00; // subindex

		ESP_LOGI(TAG, "  TX: std id=0x605 dlc=8 data=[40 00 10 00 ...]");
		err = twai_transmit(&msg, pdMS_TO_TICKS(100));
		if (err != ESP_OK)
		{
			ESP_LOGE(TAG, "  TX failed: %s", esp_err_to_name(err));
		}
		else
		{
			ESP_LOGI(TAG, "  TX sent (ACK received)");
			TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
			int rx_count = 0;
			while (xTaskGetTickCount() < deadline)
			{
				twai_message_t rx = {};
				if (twai_receive(&rx, pdMS_TO_TICKS(50)) == ESP_OK)
				{
					rx_count++;
					ESP_LOGI(TAG, "  RX: id=0x%08lX extd=%d dlc=%d data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
					         (unsigned long)rx.identifier, rx.extd, rx.data_length_code,
					         rx.data[0], rx.data[1], rx.data[2], rx.data[3],
					         rx.data[4], rx.data[5], rx.data[6], rx.data[7]);
				}
			}
			if (rx_count == 0)
				ESP_LOGW(TAG, "  No response");
		}
	}

	twai_deinit();
}

// Send PP set and wait for response from ANY producer ID
static bool set_pp_any_producer(uint8_t consumer_id, uint8_t index, uint8_t value)
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

	// Accept response from any producer
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
			if (resp_cw == (STEPPER_UIM2852_CW_PP) && rx.data_length_code >= 2 &&
			    rx.data[0] == index && rx.data[1] == value)
			{
				ESP_LOGI(TAG, "  ACK from producer %d: PP[%d] = %d", producer_id, index, value);
				return true;
			}
			// Also accept with ACK bit set in CW
			if (resp_cw == (STEPPER_UIM2852_CW_PP | STEPPER_UIM2852_CW_ACK_BIT) &&
			    rx.data_length_code >= 2 && rx.data[0] == index && rx.data[1] == value)
			{
				ESP_LOGI(TAG, "  ACK from producer %d: PP[%d] = %d", producer_id, index, value);
				return true;
			}
		}
	}
	ESP_LOGE(TAG, "  No ACK received");
	return false;
}

// Query PP and accept response from any producer
static bool query_pp_any_producer(uint8_t consumer_id, uint8_t index, uint8_t *out, uint8_t *producer_out)
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

		// Accept PP response (CW 0x01 with or without ACK bit)
		if ((resp_cw == STEPPER_UIM2852_CW_PP || resp_cw == (STEPPER_UIM2852_CW_PP | STEPPER_UIM2852_CW_ACK_BIT)) &&
		    rx.data_length_code >= 2 && rx.data[0] == index)
		{
			*out = rx.data[1];
			if (producer_out)
				*producer_out = producer_id;
			return true;
		}
	}
	return false;
}

extern "C" void app_main(void)
{
	ESP_LOGI(TAG, "========================================");
	ESP_LOGI(TAG, "  UIM2852CA Baud Rate Configuration");
	ESP_LOGI(TAG, "  Bus rate: 1 Mbps (motor already configured)");
	ESP_LOGI(TAG, "========================================");

	vTaskDelay(pdMS_TO_TICKS(500));

	esp_err_t err = twai_init_at(&BAUD_TABLE[4].timing); // 1 Mbps
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "TWAI init failed: %s", esp_err_to_name(err));
		return;
	}
	vTaskDelay(pdMS_TO_TICKS(200));

	// Configure each motor one at a time.
	// Address as consumer_id=5 (factory default), accept response from any producer.
	// Connect only ONE motor at a time if both have the same node ID.
	int motor_count = sizeof(MOTORS) / sizeof(MOTORS[0]);
	int success_count = 0;

	for (int i = 0; i < motor_count; i++)
	{
		uint8_t consumer_id = MOTORS[i];
		const char *name = MOTOR_NAMES[i];

		ESP_LOGI(TAG, "");
		ESP_LOGI(TAG, "--- %s ---", name);

		// Query current baud rate
		uint8_t current_rate = 0xFF;
		uint8_t producer = 0;
		ESP_LOGI(TAG, "Querying baud rate (PP[5]) addressed to node %d...", consumer_id);
		if (!query_pp_any_producer(consumer_id, STEPPER_UIM2852_PP_BITRATE, &current_rate, &producer))
		{
			ESP_LOGE(TAG, "No response from %s", name);
			continue;
		}
		ESP_LOGI(TAG, "Response from producer %d: baud = %s (PP[5]=%d)", producer, bitrate_name(current_rate), current_rate);

		// Query node ID
		uint8_t node_id = 0;
		if (query_pp_any_producer(consumer_id, STEPPER_UIM2852_PP_NODE_ID, &node_id, nullptr))
			ESP_LOGI(TAG, "Motor reports node ID = %d", node_id);

		if (current_rate == 0)
		{
			ESP_LOGI(TAG, "Already at 1 Mbps");
		}
		else
		{
			// Set baud rate to 1 Mbps
			ESP_LOGI(TAG, "Setting baud rate to 1 Mbps (PP[5] = 0)...");
			if (set_pp_any_producer(consumer_id, STEPPER_UIM2852_PP_BITRATE, 0))
			{
				ESP_LOGI(TAG, "SUCCESS");
				uint8_t verify = 0xFF;
				if (query_pp_any_producer(consumer_id, STEPPER_UIM2852_PP_BITRATE, &verify, nullptr) && verify == 0)
					ESP_LOGI(TAG, "Verified: PP[5] = 0 (1 Mbps)");
			}
			else
			{
				ESP_LOGE(TAG, "FAILED to set baud rate");
				continue;
			}
		}

		// Always set node ID (even if baud was already correct)
		static constexpr uint8_t TARGET_NODE_ID = 7; // <<< CHANGE THIS: 7=steering, 6=braking
		ESP_LOGI(TAG, "Setting node ID to %d (PP[7] = %d)...", TARGET_NODE_ID, TARGET_NODE_ID);
		if (set_pp_any_producer(consumer_id, STEPPER_UIM2852_PP_NODE_ID, TARGET_NODE_ID))
		{
			ESP_LOGI(TAG, "SUCCESS");
			uint8_t verify_id = 0;
			if (query_pp_any_producer(consumer_id, STEPPER_UIM2852_PP_NODE_ID, &verify_id, nullptr) && verify_id == TARGET_NODE_ID)
				ESP_LOGI(TAG, "Verified: node ID = %d", TARGET_NODE_ID);
			success_count++;
		}
		else
		{
			ESP_LOGE(TAG, "FAILED to set node ID");
		}
	}

	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "========================================");
	if (success_count == motor_count)
	{
		ESP_LOGI(TAG, "  ALL MOTORS CONFIGURED (%d/%d)", success_count, motor_count);
		ESP_LOGI(TAG, "");
		ESP_LOGI(TAG, "  >>> POWER-CYCLE THE MOTORS NOW <<<");
		ESP_LOGI(TAG, "  New baud rate takes effect on reboot.");
	}
	else
	{
		ESP_LOGE(TAG, "  PARTIAL: %d/%d configured", success_count, motor_count);
		ESP_LOGE(TAG, "  If both motors share node ID 5,");
		ESP_LOGE(TAG, "  connect one at a time and re-run.");
	}
	ESP_LOGI(TAG, "========================================");

	twai_deinit();
}
