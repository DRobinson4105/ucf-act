/**
 * @file stepper_motor_uim2852.cpp
 * @brief UIM2852CA Stepper Motor Control Implementation
 */

#include "stepper_motor_uim2852.h"

#include <string.h>

#include "esp_log.h"
#include "can_twai.h"

static const char *TAG = "STEPPER";

// Default timeout for CAN transmit
static const TickType_t TX_TIMEOUT = pdMS_TO_TICKS(20);

// (Notification callback is now per-motor, stored in stepper_motor_uim2852_t)

// ============================================================================
// Internal Helpers
// ============================================================================

/**
 * @brief Transmit a SimpleCAN3.0 instruction frame to a stepper motor.
 *
 * Builds the 29-bit extended CAN ID from the motor's node ID and
 * command word, optionally sets the ACK request bit, and sends
 * the frame via the TWAI peripheral.  On success, updates the
 * motor's last-command timestamp and ACK-pending flag.
 *
 * @param motor  Motor instance (must be initialized)
 * @param cw     Command word (e.g. STEPPER_UIM2852_CW_MO)
 * @param data   Frame payload bytes
 * @param dl     Data length (0-8)
 * @return ESP_OK on success, or an error code from the TWAI driver
 */
static esp_err_t send_instruction(stepper_motor_uim2852_t *motor, uint8_t cw, const uint8_t *data, uint8_t dl)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	// Add ACK bit if configured
	uint8_t cw_to_send = motor->config.request_ack ? (cw | STEPPER_UIM2852_CW_ACK_BIT) : cw;

	// Calculate 29-bit CAN ID
	uint32_t can_id = stepper_uim2852_make_can_id(motor->config.node_id, cw_to_send);

	// Send extended frame
	esp_err_t err = can_twai_send_extended(can_id, data, dl, TX_TIMEOUT);

	if (err == ESP_OK)
	{
		motor->last_command_tick = xTaskGetTickCount();
		motor->last_cw_sent = cw;
		if (motor->config.request_ack)
			motor->ack_pending = true;
	}

	return err;
}

// ============================================================================
// Initialization
// ============================================================================

void stepper_motor_uim2852_deinit(stepper_motor_uim2852_t *motor)
{
	if (!motor)
		return;
	if (motor->query_sem)
	{
		vSemaphoreDelete(motor->query_sem);
		motor->query_sem = NULL;
	}
	motor->initialized = false;
}

esp_err_t stepper_motor_uim2852_init(stepper_motor_uim2852_t *motor, const stepper_motor_uim2852_config_t *config)
{
	if (!motor)
		return ESP_ERR_INVALID_ARG;

	// Free any previously allocated resources before re-initializing.
	// This prevents semaphore leaks when init is called on an already-
	// initialized motor (e.g. during fault retry).
	stepper_motor_uim2852_deinit(motor);

	memset(motor, 0, sizeof(stepper_motor_uim2852_t));

	if (config)
		motor->config = *config;
	else
	{
		stepper_motor_uim2852_config_t defaults = STEPPER_MOTOR_UIM2852_CONFIG_DEFAULT();
		motor->config = defaults;
	}

	motor->microstep_resolution = 16; // Assume 16 until queried
	motor->pulses_per_rev = 3200;     // 16 * 200

	// Create binary semaphore for synchronous query_param
	motor->query_sem = xSemaphoreCreateBinary();
	if (!motor->query_sem)
	{
		ESP_LOGE(TAG, "Failed to create query semaphore");
		return ESP_ERR_NO_MEM;
	}
	motor->query_pending_cw = 0;
	portMUX_INITIALIZE(&motor->lock);
	motor->initialized = true;

	return ESP_OK;
}

esp_err_t stepper_motor_uim2852_configure(stepper_motor_uim2852_t *motor)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	uint8_t data[8];
	uint8_t dl;
	esp_err_t err;

	// Clear any stale error/status flags from previous session.
	dl = stepper_uim2852_build_ms_clear(data);
	err = send_instruction(motor, STEPPER_UIM2852_CW_MS, data, dl);
	if (err != ESP_OK)
		return err;

	vTaskDelay(pdMS_TO_TICKS(100));

	taskENTER_CRITICAL(&motor->lock);
	motor->status.error_detected = false;
	taskEXIT_CRITICAL(&motor->lock);

	// Set default motion parameters (accel/decel used as safety limits for PT mode)
	err = stepper_motor_uim2852_set_accel(motor, motor->config.default_accel);
	if (err != ESP_OK)
		return err;

	err = stepper_motor_uim2852_set_decel(motor, motor->config.default_decel);
	if (err != ESP_OK)
		return err;

	// Set emergency stop deceleration
	dl = stepper_uim2852_build_sd(data, motor->config.stop_decel);
	err = send_instruction(motor, STEPPER_UIM2852_CW_SD, data, dl);
	if (err != ESP_OK)
		return err;

	// CW 0x06: set control mode (closed-loop)
	const uint8_t cw06_data[3] = {0x06, 0x01, 0x00};
	err = send_instruction(motor, 0x06, cw06_data, 3);
	if (err != ESP_OK)
		return err;

	// CW 0x06: set positive motor direction to clockwise
	const uint8_t cw06_dir_data[3] = {0x01, 0x00, 0x00};
	err = send_instruction(motor, 0x06, cw06_dir_data, 3);
	if (err != ESP_OK)
		return err;

	// CW 0x10: set motor current to 0.8 A (ins DL=3, ack DL=3)
	const uint8_t cw10_data[3] = {0x01, 0x08, 0x00};
	err = send_instruction(motor, 0x10, cw10_data, 3);

	return err;
}

esp_err_t stepper_motor_uim2852_set_limits(stepper_motor_uim2852_t *motor, int32_t lower_limit, int32_t upper_limit)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	esp_err_t err;

	// Set lower working limit (LM[1])
	err = stepper_motor_uim2852_set_param(motor, STEPPER_UIM2852_CW_LM, STEPPER_UIM2852_LM_LOWER_WORK, lower_limit);
	if (err != ESP_OK)
		return err;

	// Set upper working limit (LM[2])
	err = stepper_motor_uim2852_set_param(motor, STEPPER_UIM2852_CW_LM, STEPPER_UIM2852_LM_UPPER_WORK, upper_limit);
	if (err != ESP_OK)
		return err;

	// Enable software limits via IC[7]=1
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_ic_set(data, STEPPER_UIM2852_IC_SOFTWARE_LIMITS, 1);
	err = send_instruction(motor, STEPPER_UIM2852_CW_IC, data, dl);

	return err;
}

// ============================================================================
// Basic Control
// ============================================================================

esp_err_t stepper_motor_uim2852_enable(stepper_motor_uim2852_t *motor)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_mo(data, true);

	esp_err_t err = send_instruction(motor, STEPPER_UIM2852_CW_MO, data, dl);
	if (err == ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->driver_enabled = true;
		taskEXIT_CRITICAL(&motor->lock);
	}
	return err;
}

esp_err_t stepper_motor_uim2852_disable(stepper_motor_uim2852_t *motor)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_mo(data, false);

	esp_err_t err = send_instruction(motor, STEPPER_UIM2852_CW_MO, data, dl);
	if (err == ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->driver_enabled = false;
		motor->motion_in_progress = false;
		taskEXIT_CRITICAL(&motor->lock);
	}
	return err;
}

esp_err_t stepper_motor_uim2852_stop(stepper_motor_uim2852_t *motor)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_st(data);

	esp_err_t err = send_instruction(motor, STEPPER_UIM2852_CW_ST, data, dl);
	if (err == ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->motion_in_progress = false;
		taskEXIT_CRITICAL(&motor->lock);
	}
	return err;
}

esp_err_t stepper_motor_uim2852_emergency_stop(stepper_motor_uim2852_t *motor)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	// First set very high SD rate for emergency (10x configured stop rate)
	uint8_t data[8];
	uint8_t dl;

	uint32_t emergency_decel = motor->config.stop_decel * 10;
	if (emergency_decel < motor->config.stop_decel)
		emergency_decel = UINT32_MAX; // overflow guard
	dl = stepper_uim2852_build_sd(data, emergency_decel);
	esp_err_t sd_err = send_instruction(motor, STEPPER_UIM2852_CW_SD, data, dl);
	(void)sd_err;

	// Issue stop command
	dl = stepper_uim2852_build_st(data);
	esp_err_t err = send_instruction(motor, STEPPER_UIM2852_CW_ST, data, dl);

	if (err == ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->motion_in_progress = false;
		taskEXIT_CRITICAL(&motor->lock);
	}
	return err;
}

// ============================================================================
// Motion Parameters
// ============================================================================

esp_err_t stepper_motor_uim2852_set_accel(stepper_motor_uim2852_t *motor, uint32_t accel_rate)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_ac(data, accel_rate);
	return send_instruction(motor, STEPPER_UIM2852_CW_AC, data, dl);
}

esp_err_t stepper_motor_uim2852_set_decel(stepper_motor_uim2852_t *motor, uint32_t decel_rate)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_dc(data, decel_rate);
	return send_instruction(motor, STEPPER_UIM2852_CW_DC, data, dl);
}

esp_err_t stepper_motor_uim2852_set_origin(stepper_motor_uim2852_t *motor)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_og(data);

	esp_err_t err = send_instruction(motor, STEPPER_UIM2852_CW_OG, data, dl);
	if (err == ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->absolute_position = 0;
		taskEXIT_CRITICAL(&motor->lock);
	}
	return err;
}

// ============================================================================
// Position-Time (PT) Interpolation Mode
// ============================================================================

esp_err_t stepper_motor_uim2852_pt_configure(stepper_motor_uim2852_t *motor)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	uint8_t data[8];
	uint8_t dl;
	esp_err_t err;

	// CW 0x22: queue resetter (MP[0]=0)
	const uint8_t cw22_mp0_data[3] = {0x00, 0x00, 0x00};
	err = send_instruction(motor, 0x22, cw22_mp0_data, 3);
	if (err != ESP_OK)
		return err;

	// CW 0x22: set frame execution time (MP[4]=100)
	const uint8_t cw22_time_data[3] = {0x04, 0x64, 0x00};
	err = send_instruction(motor, 0x22, cw22_time_data, 3);
	if (err != ESP_OK)
		return err;

	// CW 0x22: set PT mode to PVT FIFO mode (MP[3]=0)
	const uint8_t cw22_data[3] = {0x03, 0x00, 0x00};
	err = send_instruction(motor, 0x22, cw22_data, 3);
	if (err != ESP_OK)
		return err;

	// CW 0x22: MP[5]=1
	const uint8_t cw22_mp5_data[3] = {0x05, 0x01, 0x00};
	err = send_instruction(motor, 0x22, cw22_mp5_data, 3);
	if (err != ESP_OK)
		return err;

	// Enable PVT FIFO empty notification (IE[10]=1)
	dl = stepper_uim2852_build_ie_set(data, STEPPER_UIM2852_IE_PVT_FIFO_EMPTY, 1);
	err = send_instruction(motor, STEPPER_UIM2852_CW_IE, data, dl);
	if (err != ESP_OK)
		return err;

	// Enable PVT FIFO low warning notification (IE[11]=1)
	dl = stepper_uim2852_build_ie_set(data, STEPPER_UIM2852_IE_PVT_FIFO_LOW, 1);
	err = send_instruction(motor, STEPPER_UIM2852_CW_IE, data, dl);
	if (err != ESP_OK)
		return err;

	return ESP_OK;
}

esp_err_t stepper_motor_uim2852_pt_start(stepper_motor_uim2852_t *motor)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	uint8_t data[8];
	// PV mode=1 (PT linear interpolation), start=true
	uint8_t dl = stepper_uim2852_build_pv(data, 1, true);
	esp_err_t err = send_instruction(motor, STEPPER_UIM2852_CW_PV, data, dl);

	if (err == ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->pt_mode_active = true;
		motor->pt_fifo_empty = false;
		motor->pt_fifo_low = false;
		taskEXIT_CRITICAL(&motor->lock);
	}
	return err;
}

esp_err_t stepper_motor_uim2852_pt_stop(stepper_motor_uim2852_t *motor)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	uint8_t data[8];
	uint8_t dl;
	esp_err_t err;

	// PV mode=1, start=false — stop consuming FIFO waypoints
	dl = stepper_uim2852_build_pv(data, 1, false);
	err = send_instruction(motor, STEPPER_UIM2852_CW_PV, data, dl);
	// Continue even on error — best-effort stop

	// Deceleration stop to halt any residual motion
	dl = stepper_uim2852_build_st(data);
	esp_err_t st_err = send_instruction(motor, STEPPER_UIM2852_CW_ST, data, dl);

	taskENTER_CRITICAL(&motor->lock);
	motor->pt_mode_active = false;
	motor->motion_in_progress = false;
	taskEXIT_CRITICAL(&motor->lock);

	// Return first error if any
	return (err != ESP_OK) ? err : st_err;
}

esp_err_t stepper_motor_uim2852_pt_feed(stepper_motor_uim2852_t *motor, int32_t position, uint32_t time_ms)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	// Reject waypoint feeds when PT mode hasn't been started
	if (!motor->pt_mode_active)
		return ESP_ERR_INVALID_STATE;

	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_qf(data, position, time_ms);
	esp_err_t err = send_instruction(motor, STEPPER_UIM2852_CW_QF, data, dl);

	if (err == ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->pt_fifo_empty = false;
		motor->pt_fifo_low = false;
		motor->target_position = position;
		taskEXIT_CRITICAL(&motor->lock);
	}
	return err;
}

// ============================================================================
// Status Query
// ============================================================================

esp_err_t stepper_motor_uim2852_query_status(stepper_motor_uim2852_t *motor)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_ms(data, STEPPER_UIM2852_MS_FLAGS_RELPOS);
	return send_instruction(motor, STEPPER_UIM2852_CW_MS, data, dl);
}

esp_err_t stepper_motor_uim2852_query_position(stepper_motor_uim2852_t *motor)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_ms(data, STEPPER_UIM2852_MS_SPEED_ABSPOS);
	return send_instruction(motor, STEPPER_UIM2852_CW_MS, data, dl);
}

esp_err_t stepper_motor_uim2852_clear_status(stepper_motor_uim2852_t *motor)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_ms_clear(data);
	return send_instruction(motor, STEPPER_UIM2852_CW_MS, data, dl);
}

// ============================================================================
// CAN Frame Processing
// ============================================================================

bool stepper_motor_uim2852_process_frame(stepper_motor_uim2852_t *motor, const twai_message_t *msg)
{
	if (!motor || !motor->initialized || !msg)
		return false;

	// Must be extended frame
	if (!msg->extd)
		return false;

	// Parse CAN ID
	uint8_t producer_id, cw;
	if (!stepper_uim2852_parse_can_id(msg->identifier, &producer_id, &cw))
		return false;

	// Both motors respond with producer_id=1.  Route frames by checking
	// whether this motor is actively waiting for a response.  For
	// unsolicited frames (status polls, notifications, errors), accept
	// on any instance — the caller's !matched guard prevents duplicates.
	uint8_t cw_base_peek = stepper_uim2852_cw_base(cw);

	bool is_unsolicited = (cw_base_peek == STEPPER_UIM2852_CW_MS ||
	                       cw_base_peek == STEPPER_UIM2852_CW_NOTIFY ||
	                       cw_base_peek == STEPPER_UIM2852_CW_ER);

	if (!is_unsolicited)
	{
		// This is a query/command response — only accept if we're waiting
		taskENTER_CRITICAL(&motor->lock);
		bool expecting = motor->ack_pending || (motor->query_pending_cw != 0);
		taskEXIT_CRITICAL(&motor->lock);
		if (!expecting)
			return false;
	}

	// Update response timestamp
	taskENTER_CRITICAL(&motor->lock);
	motor->last_response_tick = xTaskGetTickCount();
	motor->ack_pending = false;
	taskEXIT_CRITICAL(&motor->lock);

	uint8_t cw_base = stepper_uim2852_cw_base(cw);
	uint8_t dl = msg->data_length_code;
	const uint8_t *data = msg->data;

	// Handle different response types
	switch (cw_base)
	{
	case STEPPER_UIM2852_CW_MS:
		// Motion status response
		if (dl >= 1)
		{
			uint8_t index = data[0];
			if (index == STEPPER_UIM2852_MS_FLAGS_RELPOS)
			{
				taskENTER_CRITICAL(&motor->lock);
				stepper_uim2852_parse_ms0(data, dl, &motor->status);

				// Update motion state from flags
				motor->driver_enabled = motor->status.driver_on;
				if (motor->status.in_position || motor->status.stopped)
					motor->motion_in_progress = false;
				taskEXIT_CRITICAL(&motor->lock);
			}
			else if (index == STEPPER_UIM2852_MS_SPEED_ABSPOS)
			{
				taskENTER_CRITICAL(&motor->lock);
				stepper_uim2852_parse_ms1(data, dl, &motor->current_speed, &motor->absolute_position);
				taskEXIT_CRITICAL(&motor->lock);
			}
		}
		break;

	case STEPPER_UIM2852_CW_NOTIFY:
		// Real-time notification
		{
			stepper_uim2852_notification_t notif = {};
			notif.node_id = motor->config.node_id;

			if (stepper_uim2852_parse_notification(data, dl, &notif))
			{
				// Handle specific notifications
				if (notif.type == STEPPER_UIM2852_STATUS_PTP_COMPLETE)
				{
					taskENTER_CRITICAL(&motor->lock);
					motor->motion_in_progress = false;
					motor->status.in_position = true;
					motor->absolute_position = notif.position;
					taskEXIT_CRITICAL(&motor->lock);
				}
				else if (notif.type == STEPPER_UIM2852_ALARM_STALL)
				{
					taskENTER_CRITICAL(&motor->lock);
					motor->status.stall_detected = true;
					motor->motion_in_progress = false;
					taskEXIT_CRITICAL(&motor->lock);
				}
				else if (notif.type == STEPPER_UIM2852_STATUS_PVT_FIFO_EMPTY)
				{
					taskENTER_CRITICAL(&motor->lock);
					motor->pt_fifo_empty = true;
					taskEXIT_CRITICAL(&motor->lock);
				}
				else if (notif.type == STEPPER_UIM2852_STATUS_PVT_FIFO_LOW)
				{
					taskENTER_CRITICAL(&motor->lock);
					motor->pt_fifo_low = true;
					taskEXIT_CRITICAL(&motor->lock);
				}

				// Call per-motor notification callback if set
				if (motor->notify_callback)
					motor->notify_callback(motor, &notif);
			}
		}
		break;

	case STEPPER_UIM2852_CW_ER:
		// Error report — suppress spurious 0x33/0x34 errors caused by
		// motor misinterpreting Safety heartbeat frames as commands.
		{
			stepper_uim2852_error_t error = {};
			if (stepper_uim2852_parse_error(data, dl, &error))
			{
				if (error.error_code != STEPPER_UIM2852_ERR_SUBINDEX &&
				    error.error_code != STEPPER_UIM2852_ERR_DATA)
				{
					taskENTER_CRITICAL(&motor->lock);
					motor->status.error_detected = true;
					motor->last_error = error;
					taskEXIT_CRITICAL(&motor->lock);
					ESP_LOGE(TAG, "Node %u: Error 0x%02X on CW=0x%02X[%u]", motor->config.node_id, error.error_code,
					         error.related_cw, error.subindex);
				}
			}
		}
		break;

	case STEPPER_UIM2852_CW_MO:
		// ACK for motor enable/disable
		if (dl >= 1)
		{
			taskENTER_CRITICAL(&motor->lock);
			motor->driver_enabled = (data[0] != 0);
			taskEXIT_CRITICAL(&motor->lock);
		}
		break;
	case STEPPER_UIM2852_CW_PP:
	case STEPPER_UIM2852_CW_IC:
	case STEPPER_UIM2852_CW_IE:
	case STEPPER_UIM2852_CW_MT:
	case STEPPER_UIM2852_CW_LM:
	case STEPPER_UIM2852_CW_QE:
		// Parameter response — unblock query_param if this matches the pending query
		{
			taskENTER_CRITICAL(&motor->lock);
			bool match = (motor->query_pending_cw == cw_base && dl >= 2 && data[0] == motor->query_pending_idx);
			taskEXIT_CRITICAL(&motor->lock);
			if (match)
			{
				// Delegate to protocol layer for correct sign-extended parsing
				uint8_t parsed_idx;
				int32_t val = 0;
				stepper_uim2852_parse_param_response(data, dl, &parsed_idx, &val);
				motor->query_result = val;
				taskENTER_CRITICAL(&motor->lock);
				motor->query_pending_cw = 0;
				taskEXIT_CRITICAL(&motor->lock);
				xSemaphoreGive(motor->query_sem);
			}
		}
		break;

	default:
		// ACK for other commands
		break;
	}

	return true;
}

// ============================================================================
// Liveness Watchdog
// ============================================================================

bool stepper_motor_uim2852_check_liveness(const stepper_motor_uim2852_t *motor, TickType_t now_tick,
                                          TickType_t timeout_ticks)
{
	if (!motor || !motor->initialized)
		return false;

	stepper_motor_uim2852_t *m = (stepper_motor_uim2852_t *)motor;
	taskENTER_CRITICAL(&m->lock);
	bool driver_enabled = m->driver_enabled;
	TickType_t last_response_tick = m->last_response_tick;
	taskEXIT_CRITICAL(&m->lock);

	// Only flag timeout if the driver has been enabled (we expect responses)
	// and at least one response has been received (last_response_tick != 0).
	if (!driver_enabled)
		return false;
	if (last_response_tick == 0)
		return false;

	TickType_t elapsed = now_tick - last_response_tick;
	return (elapsed > timeout_ticks);
}

// ============================================================================
// Notification Callback
// ============================================================================

void stepper_motor_uim2852_set_notify_callback(stepper_motor_uim2852_t *motor,
                                               stepper_motor_uim2852_notify_cb_t callback)
{
	if (!motor)
		return;
	taskENTER_CRITICAL(&motor->lock);
	motor->notify_callback = callback;
	taskEXIT_CRITICAL(&motor->lock);
}

// ============================================================================
// Low-Level Access
// ============================================================================

esp_err_t stepper_motor_uim2852_send_instruction(stepper_motor_uim2852_t *motor, uint8_t cw, const uint8_t *data,
                                                 uint8_t dl)
{
	return send_instruction(motor, cw, data, dl);
}

esp_err_t stepper_motor_uim2852_query_param(stepper_motor_uim2852_t *motor, uint8_t cw, uint8_t index, int32_t *value)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	// Drain any stale signal from a previous query
	xSemaphoreTake(motor->query_sem, 0);

	// Record what we are waiting for so process_frame can match the response
	motor->query_result = 0;
	taskENTER_CRITICAL(&motor->lock);
	motor->query_pending_idx = index;
	motor->query_pending_cw = stepper_uim2852_cw_base(cw); // store base CW for matching
	taskEXIT_CRITICAL(&motor->lock);

	uint8_t data[8] = {};
	data[0] = index;

	esp_err_t err = send_instruction(motor, cw, data, 1);
	if (err != ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->query_pending_cw = 0;
		taskEXIT_CRITICAL(&motor->lock);
		return err;
	}

	// Block until process_frame signals the semaphore or we time out (500 ms)
	if (xSemaphoreTake(motor->query_sem, pdMS_TO_TICKS(500)) != pdTRUE)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->query_pending_cw = 0;
		taskEXIT_CRITICAL(&motor->lock);
		return ESP_ERR_TIMEOUT;
	}

	if (value)
		*value = motor->query_result;
	return ESP_OK;
}

esp_err_t stepper_motor_uim2852_set_param(stepper_motor_uim2852_t *motor, uint8_t cw, uint8_t index, int32_t value)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	uint8_t data[8] = {};
	data[0] = index;

	// Determine DL based on CW type per spec:
	//   PP: DL=2 (u8 value)
	//   IC, IE, MT, QE: DL=3 (u16 LE value)
	//   LM: DL=5 (s32 LE value)
	uint8_t cw_base = stepper_uim2852_cw_base(cw);
	uint8_t dl;

	if (cw_base == STEPPER_UIM2852_CW_PP)
	{
		data[1] = (uint8_t)(value & 0xFF);
		dl = 2;
	}
	else if (cw_base == STEPPER_UIM2852_CW_IC || cw_base == STEPPER_UIM2852_CW_IE || cw_base == STEPPER_UIM2852_CW_MT ||
	         cw_base == STEPPER_UIM2852_CW_QE)
	{
		data[1] = (uint8_t)(value & 0xFF);
		data[2] = (uint8_t)((value >> 8) & 0xFF);
		dl = 3;
	}
	else
	{
		// LM and other 32-bit params
		data[1] = (uint8_t)(value & 0xFF);
		data[2] = (uint8_t)((value >> 8) & 0xFF);
		data[3] = (uint8_t)((value >> 16) & 0xFF);
		data[4] = (uint8_t)((value >> 24) & 0xFF);
		dl = 5;
	}

	return send_instruction(motor, cw, data, dl);
}
