/**
 * @file control_actuator_runtime.cpp
 * @brief Control actuator runtime helpers for state transitions and PT stream scheduling.
 */

#include "control_actuator_runtime.h"

#include "control_config.h"
#include "control_globals.h"
#include "control_health.h"
#include "dac_mcp4728.h"
#include "esp_log.h"
#include "motor_dispatch.h"
#include "motor_protocol.h"
#include "relay_dpdt_my5nj.h"

/**
 * @brief Convert an override reason enum to a human-readable string.
 *
 * @param reason  Override reason to convert
 * @return Static string label for logging
 */
const char *override_reason_to_string(override_reason_t reason)
{
	switch (reason)
	{
	case OVERRIDE_REASON_THROTTLE:
		return "throttle";
	case OVERRIDE_REASON_REVERSE:
		return "reverse";
	case OVERRIDE_REASON_STEERING:
		return "steering";
	case OVERRIDE_REASON_BRAKING:
		return "braking";
	case OVERRIDE_REASON_NONE:
	default:
		return "none";
	}
}

/**
 * @brief Convert a disable reason enum to a human-readable string.
 *
 * @param reason  Disable reason to convert
 * @return Static string label for logging
 */
const char *disable_reason_to_string(disable_reason_t reason)
{
	switch (reason)
	{
	case CONTROL_DISABLE_REASON_SAFETY_RETREAT:
		return "safety_retreat";
	case CONTROL_DISABLE_REASON_MOTOR_FAULT:
		return "motor_fault";
	case CONTROL_DISABLE_REASON_SENSOR_INVALID:
		return "sensor_invalid";
	case CONTROL_DISABLE_REASON_INTERNAL:
		return "internal";
	case CONTROL_DISABLE_REASON_NONE:
	default:
		return "none";
	}
}

/**
 * @brief Convert an enable-abort reason enum to a human-readable string.
 *
 * @param reason  Abort reason to convert
 * @return Static string label for logging
 */
const char *abort_reason_to_string(abort_reason_t reason)
{
	switch (reason)
	{
	case CONTROL_ABORT_REASON_SAFETY_RETREAT:
		return "safety_retreat";
	case CONTROL_ABORT_REASON_PEDAL_PRESSED:
		return "pedal_pressed";
	case CONTROL_ABORT_REASON_FR_IN_REVERSE:
		return "fr_in_reverse";
	case CONTROL_ABORT_REASON_MOTOR_FAULT:
		return "motor_fault";
	case CONTROL_ABORT_REASON_SENSOR_INVALID:
		return "sensor_invalid";
	case CONTROL_ABORT_REASON_NONE:
	default:
		return "none";
	}
}

static bool tick_deadline_reached(TickType_t now, TickType_t deadline)
{
	return (int32_t)(now - deadline) >= 0;
}

/**
 * @brief Feed one PT row on the motor's 100 ms cadence, reusing the last target when needed.
 *
 * The control loop runs at 20 ms, but the planner only updates every 100 ms.
 * This helper maintains a one-segment cushion in the motor's PT FIFO by:
 * - preloading two PT rows when PT motion starts (or restarts after FIFO empty)
 * - enqueueing one new PT row every PT frame interval
 * - attempting the enqueue up to one control tick early so a single late tick
 *   can retry before the current motor segment expires
 *
 * The position passed in is the latest clamped planner-derived target. If the
 * planner skipped a cycle, the caller simply passes the same target again, and
 * the helper keeps the FIFO alive by repeating that hold position.
 *
 * @param state           Motor state (read under lock for PT bookkeeping)
 * @param node_id         CAN node ID of the motor
 * @param position        Absolute PT target position to queue
 * @param now_tick        Current RTOS tick
 * @param next_feed_tick  [in/out] Next scheduled PT enqueue deadline
 * @param fed_frame       [out] True if at least one PT frame was sent
 * @return ESP_OK on success, or ESP_FAIL on dispatch failure
 */
esp_err_t feed_pt_stream_if_due(motor_uim2852_state_t *state, uint8_t node_id, int32_t position, TickType_t now_tick,
                                TickType_t *next_feed_tick, bool *fed_frame)
{
	if (!state || !next_feed_tick || !fed_frame)
		return ESP_ERR_INVALID_ARG;

	*fed_frame = false;

	// Read PT bookkeeping under lock
	uint16_t frame_time_ms;
	bool motion_started;
	uint8_t prefill_count;
	uint16_t write_index;
	taskENTER_CRITICAL(&state->lock);
	frame_time_ms = state->pt_frame_time_ms;
	motion_started = state->pt_motion_started;
	prefill_count = state->pt_prefill_count;
	write_index = state->pt_write_index;
	taskEXIT_CRITICAL(&state->lock);

	TickType_t interval_ticks = pdMS_TO_TICKS(frame_time_ms);
	if (interval_ticks == 0)
		interval_ticks = 1;

	TickType_t early_margin = CONTROL_LOOP_INTERVAL;
	if (early_margin > interval_ticks)
		early_margin = interval_ticks;

	bool priming_stream = !motion_started;
	uint8_t frames_to_send = 0;

	if (priming_stream)
	{
		frames_to_send = 1;
		if (prefill_count < PT_STREAM_STARTUP_FRAMES)
			frames_to_send = (uint8_t)(PT_STREAM_STARTUP_FRAMES - prefill_count);
	}
	else
	{
		if (*next_feed_tick == 0)
			*next_feed_tick = now_tick + interval_ticks;

		TickType_t send_check_tick = now_tick + early_margin;
		if (tick_deadline_reached(send_check_tick, *next_feed_tick))
			frames_to_send = 1;
	}

	for (uint8_t i = 0; i < frames_to_send; ++i)
	{
		motor_cmd_t cmd;
		motor_cmd_pt_set(node_id, true, write_index, position, &cmd);
		motor_dispatch_result_t res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20),
		                                                  nullptr, nullptr, nullptr);
		if (res != MOTOR_DISPATCH_RESULT_OK)
			return ESP_FAIL;

		// Advance write index under lock
		taskENTER_CRITICAL(&state->lock);
		state->pt_write_index++;
		state->pt_prefill_count++;
		taskEXIT_CRITICAL(&state->lock);
		write_index++;

		*fed_frame = true;
	}

	if (*fed_frame)
	{
		if (priming_stream)
			*next_feed_tick = now_tick + interval_ticks;
		else
			*next_feed_tick += interval_ticks;
	}

	return ESP_OK;
}

int32_t compute_braking_pt_step_limit(const motor_uim2852_state_t *state)
{
	if (!state || state->pt_frame_time_ms == 0 || BRAKING_PT_STEP_LIMIT_PER_500MS <= 0)
		return 0;

	int64_t scaled = ((int64_t)BRAKING_PT_STEP_LIMIT_PER_500MS * (int64_t)state->pt_frame_time_ms + 499) / 500;
	if (scaled <= 0)
		return 1;
	if (scaled > INT32_MAX)
		return INT32_MAX;
	return (int32_t)scaled;
}

/**
 * @brief Immediately disable all autonomous actuators to a safe state.
 *
 * Executes the full shutdown sequence: disables and emergency-stops
 * the DAC throttle path, de-energizes the DPDT relay, stops both
 * steering and braking PT streams, then runs motor_uim2852_shutdown()
 * (emergency stop, brake, disable) on each motor.
 * Best-effort — individual failures are logged but do not prevent
 * subsequent shutdown steps.
 */
void disable_autonomous_actuators(void)
{
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	handle_runtime_error(dac_mcp4728_disable(), &g_dac_ready, "DAC", "disable command failed");
	handle_runtime_error(dac_mcp4728_emergency_stop(), &g_dac_ready, "DAC",
	                     "emergency stop command failed");
	handle_runtime_error(relay_dpdt_my5nj_deenergize(), &g_dpdt_relay_ready, "DPDT_RELAY",
	                     "de-energize command failed");
#endif

	// Stop PT interpolation before emergency stop (clears FIFO consumption).
	// Best-effort: errors are ignored since emergency stop follows immediately.
	{
		motor_cmd_t cmd;
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
		motor_cmd_st_stop(MOTOR_NODE_STEERING, true, &cmd);
		(void)motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
		motor_cmd_st_stop(MOTOR_NODE_BRAKING, true, &cmd);
		(void)motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
#endif
	}

#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
	motor_uim2852_shutdown(MOTOR_NODE_STEERING, &g_motor_uim2852_steering_ready, "MOTOR_UIM2852_STEERING");
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
	motor_uim2852_shutdown(MOTOR_NODE_BRAKING, &g_motor_uim2852_braking_ready, "MOTOR_UIM2852_BRAKING");
#endif
}

/**
 * @brief Handle a driver override event by disabling all autonomous actuators.
 *
 * Logs the override reason (if logging is enabled) and delegates to
 * disable_autonomous_actuators() for the full shutdown sequence.
 *
 * @param reason  Override trigger (pedal, F/R change, steering/braking error)
 */
void execute_trigger_override(override_reason_t reason)
{
#ifdef CONFIG_LOG_CONTROL_OVERRIDE_CHANGES
	ESP_LOGI(TAG, "OVERRIDE TRIGGERED (reason=%s)", override_reason_to_string(reason));
#else
	(void)reason;
#endif

	disable_autonomous_actuators();
}

/**
 * @brief Disable autonomy without classifying the event as a driver override.
 *
 * Used when Safety retreats the target state, or when a motor/sensor
 * fault forces a transition out of ACTIVE.  Logs the reason and
 * delegates to disable_autonomous_actuators().
 *
 * @param reason              Why autonomy is being disabled
 * @param safety_fault_flags   Safety fault bitmask (for Safety-retreat logging)
 * @param safety_stop_flags   Safety stop bitmask (for Safety-retreat logging)
 * @param control_fault_flags  Control's own issue code (for local issue logging)
 */
void execute_disable_autonomy(disable_reason_t reason, node_fault_t safety_fault_flags, node_stop_t safety_stop_flags,
                              node_fault_t control_fault_flags)
{
#if defined(CONFIG_LOG_CONTROL_STATE_CHANGES) || defined(CONFIG_LOG_CONTROL_ENABLE_SEQUENCE)
	if (reason == CONTROL_DISABLE_REASON_SAFETY_RETREAT)
	{
		ESP_LOGI(TAG, "AUTONOMY DISABLED (reason=%s, safety_fault=%s, safety_stop=%s)",
		         disable_reason_to_string(reason), node_fault_to_string(safety_fault_flags),
		         node_stop_to_string(safety_stop_flags));
	}
	else
	{
		ESP_LOGI(TAG, "AUTONOMY DISABLED (reason=%s, control_issue=%s)", disable_reason_to_string(reason),
		         node_fault_to_string(control_fault_flags));
	}
#else
	(void)reason;
	(void)safety_fault_flags;
	(void)safety_stop_flags;
	(void)control_fault_flags;
#endif

	disable_autonomous_actuators();
}

/**
 * @brief Begin the autonomous enable sequence.
 *
 * Sets the DAC output to level 0 (idle).
 * The DPDT relay is NOT energized here — both poles (throttle source
 * switching and pedal bypass) switch together in execute_complete_enable().
 */
void execute_start_enable()
{
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
	ESP_LOGI(TAG, "Starting autonomous enable sequence");
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	handle_runtime_error(dac_mcp4728_set_level(0), &g_dac_ready, "DAC", "set level command failed");
#endif
}

/**
 * @brief Complete the autonomous enable sequence.
 *
 * Releases motor brakes, enables motor drivers (MO=1), configures
 * and starts PT interpolation mode for both steering and braking,
 * energizes the DPDT relay (both throttle source switching and pedal
 * bypass), and enables the DAC-based autonomous throttle path.
 * If any required motor setup fails, rolls back all changes and
 * returns without completing.
 *
 * Detailed sequence:
 *   1. Clear status and enable each motor (steering, braking) with
 *      short inter-command delays for the driver to settle.
 *   2. Sync the braking motor's target position to its current
 *      absolute position so the first PT segment starts in place.
 *   3. If either motor enable fails, disable both motors,
 *      de-energize the DPDT relay, disable the DAC, and return.
 *   4. Configure and start PT interpolation mode on both motors.
 *      On failure, perform the same rollback as step 3.
 *   5. Energize the DPDT relay (switches both throttle source and
 *      pedal bypass poles) and wait for contact settle time.
 *   6. Enable the DAC autonomous throttle path.
 */
void execute_complete_enable()
{
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
	ESP_LOGI(TAG, "Completing autonomous enable sequence");
#endif
	motor_cmd_t cmd;
	motor_dispatch_result_t res;
	bool steer_ok = true;
	bool brake_ok = true;

#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
	// Clear errors on steering motor
	motor_cmd_er_clear_all(MOTOR_NODE_STEERING, true, &cmd);
	(void)motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
	vTaskDelay(pdMS_TO_TICKS(10));
	// Enable steering motor (MO=1)
	motor_cmd_mo_set(MOTOR_NODE_STEERING, true, MOTOR_MO_STATE_ENABLE, &cmd);
	res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
	steer_ok = (res == MOTOR_DISPATCH_RESULT_OK);
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
	if (!steer_ok)
		ESP_LOGI(TAG, "Steering enable failed: dispatch result %d", (int)res);
#endif
	vTaskDelay(pdMS_TO_TICKS(5));
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
	// Clear errors on braking motor
	motor_cmd_er_clear_all(MOTOR_NODE_BRAKING, true, &cmd);
	(void)motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
	vTaskDelay(pdMS_TO_TICKS(10));
	// Enable braking motor (MO=1)
	motor_cmd_mo_set(MOTOR_NODE_BRAKING, true, MOTOR_MO_STATE_ENABLE, &cmd);
	res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
	brake_ok = (res == MOTOR_DISPATCH_RESULT_OK);
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
	if (!brake_ok)
		ESP_LOGI(TAG, "Braking enable failed: dispatch result %d", (int)res);
#endif
	vTaskDelay(pdMS_TO_TICKS(5));
	// Sync braking target position to current position
	taskENTER_CRITICAL(&g_braking_state.lock);
	g_braking_state.target_position = g_braking_state.absolute_position;
	taskEXIT_CRITICAL(&g_braking_state.lock);
#endif

	if (!steer_ok || !brake_ok)
	{
		ESP_LOGE(TAG, "Motor enable failed, aborting autonomous enable");
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
		motor_cmd_mo_set(MOTOR_NODE_STEERING, true, MOTOR_MO_STATE_DISABLE, &cmd);
		(void)motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
		motor_cmd_mo_set(MOTOR_NODE_BRAKING, true, MOTOR_MO_STATE_DISABLE, &cmd);
		(void)motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
		handle_runtime_error(relay_dpdt_my5nj_deenergize(), &g_dpdt_relay_ready, "DPDT_RELAY",
		                     "de-energize command failed");
		handle_runtime_error(dac_mcp4728_disable(), &g_dac_ready, "DAC", "disable command failed");
#endif
		return;
	}

	// Configure and arm PT FIFO mode for both steering and braking.
	// PT configure: set FIFO mode (MP[3]=0), set motion time (MP[4]), reset table (MP[0]=0)
	// PT start: BG (begin motion)
	bool pt_ok = true;
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
	if (pt_ok)
	{
		motor_cmd_mp_set_u16(MOTOR_NODE_STEERING, true, MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE,
		                     (uint16_t)MOTOR_MP_MODE_FIFO, &cmd);
		res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
		pt_ok = (res == MOTOR_DISPATCH_RESULT_OK);
	}
	if (pt_ok)
	{
		motor_cmd_mp_set_u16(MOTOR_NODE_STEERING, true, MOTOR_MP_INDEX_PT_MOTION_TIME,
		                     g_steering_state.pt_frame_time_ms, &cmd);
		res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
		pt_ok = (res == MOTOR_DISPATCH_RESULT_OK);
	}
	if (pt_ok)
	{
		// Reset PT table (write 0 to MP[0])
		motor_cmd_mp_set_u16(MOTOR_NODE_STEERING, true, MOTOR_MP_INDEX_CURRENT_QUEUE_LEVEL_OR_RESET_TABLE,
		                     0, &cmd);
		res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
		pt_ok = (res == MOTOR_DISPATCH_RESULT_OK);
	}
	if (pt_ok)
	{
		taskENTER_CRITICAL(&g_steering_state.lock);
		g_steering_state.pt_write_index = 0;
		g_steering_state.pt_prefill_count = 0;
		g_steering_state.pt_motion_started = false;
		taskEXIT_CRITICAL(&g_steering_state.lock);

		motor_cmd_bg_begin(MOTOR_NODE_STEERING, true, &cmd);
		res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
		pt_ok = (res == MOTOR_DISPATCH_RESULT_OK);
	}
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
	if (pt_ok)
	{
		motor_cmd_mp_set_u16(MOTOR_NODE_BRAKING, true, MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE,
		                     (uint16_t)MOTOR_MP_MODE_FIFO, &cmd);
		res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
		pt_ok = (res == MOTOR_DISPATCH_RESULT_OK);
	}
	if (pt_ok)
	{
		motor_cmd_mp_set_u16(MOTOR_NODE_BRAKING, true, MOTOR_MP_INDEX_PT_MOTION_TIME,
		                     g_braking_state.pt_frame_time_ms, &cmd);
		res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
		pt_ok = (res == MOTOR_DISPATCH_RESULT_OK);
	}
	if (pt_ok)
	{
		motor_cmd_mp_set_u16(MOTOR_NODE_BRAKING, true, MOTOR_MP_INDEX_CURRENT_QUEUE_LEVEL_OR_RESET_TABLE,
		                     0, &cmd);
		res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
		pt_ok = (res == MOTOR_DISPATCH_RESULT_OK);
	}
	if (pt_ok)
	{
		taskENTER_CRITICAL(&g_braking_state.lock);
		g_braking_state.pt_write_index = 0;
		g_braking_state.pt_prefill_count = 0;
		g_braking_state.pt_motion_started = false;
		taskEXIT_CRITICAL(&g_braking_state.lock);

		motor_cmd_bg_begin(MOTOR_NODE_BRAKING, true, &cmd);
		res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
		pt_ok = (res == MOTOR_DISPATCH_RESULT_OK);
	}
#endif
	if (!pt_ok)
	{
		ESP_LOGE(TAG, "PT mode setup failed — aborting enable");
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
		motor_cmd_mo_set(MOTOR_NODE_STEERING, true, MOTOR_MO_STATE_DISABLE, &cmd);
		(void)motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
		motor_cmd_mo_set(MOTOR_NODE_BRAKING, true, MOTOR_MO_STATE_DISABLE, &cmd);
		(void)motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
		handle_runtime_error(relay_dpdt_my5nj_deenergize(), &g_dpdt_relay_ready, "DPDT_RELAY",
		                     "de-energize command failed");
		handle_runtime_error(dac_mcp4728_disable(), &g_dac_ready, "DAC", "disable command failed");
#endif
		return;
	}

#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	// Energize DPDT relay (both poles: throttle source + pedal bypass) and wait for contacts to settle
	handle_runtime_error(relay_dpdt_my5nj_energize(), &g_dpdt_relay_ready, "DPDT_RELAY", "energize command failed");
	vTaskDelay(pdMS_TO_TICKS(50)); // relay contact settle time

	handle_runtime_error(dac_mcp4728_enable_autonomous(), &g_dac_ready, "DAC",
	                     "enable autonomous command failed");
#endif
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
	ESP_LOGI(TAG, "AUTONOMOUS MODE ACTIVE");
#endif
}

/**
 * @brief Abort the enable sequence and return actuators to safe state.
 *
 * De-energizes the DPDT relay and disables the DAC throttle path.
 * Called when Safety retreats, the pedal is pressed, or the F/R
 * selector leaves Forward during the enable dwell.
 *
 * @param reason  Why the enable sequence is being aborted
 */
void execute_abort_enable(abort_reason_t reason)
{
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
	ESP_LOGI(TAG, "Enable sequence aborted (reason=%s)", abort_reason_to_string(reason));
#else
	(void)reason;
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	handle_runtime_error(relay_dpdt_my5nj_deenergize(), &g_dpdt_relay_ready, "DPDT_RELAY",
	                     "de-energize command failed");
	handle_runtime_error(dac_mcp4728_disable(), &g_dac_ready, "DAC", "disable command failed");
#endif
}
