/**
 * @file control_loop.cpp
 * @brief FreeRTOS task implementations for the Control ESP32 runtime.
 */
#ifndef CONFIG_CONTROL_TEST_MODE

#include "control_loop.h"

#include "can_protocol.h"
#include "can_twai.h"
#include "control_actuator_runtime.h"
#include "control_can_rx.h"
#include "control_config.h"
#include "control_driver_inputs.h"
#include "control_globals.h"
#include "control_health.h"
#include "control_heartbeat_tx.h"
#include "control_logic.h"
#include "dac_mcp4728.h"
#include "esp_log.h"
#include "heartbeat_monitor.h"
#include "node_support.h"
#include "status_led.h"

static void track_can_tx(esp_err_t err)
{
	control_heartbeat_tx_context_t ctx = {
		.tag = TAG,
		.tag_tx = TAG_TX,
		.can_tx_lock = &g_can_tx_lock,
		.hb_seq_lock = &g_hb_seq_lock,
		.twai_ready = &g_twai_ready,
		.can_recovery_in_progress = &g_can_recovery_in_progress,
		.can_tx_in_flight = &g_can_tx_in_flight,
		.heartbeat_seq = &g_heartbeat_seq,
		.control_state = &g_control_state,
		.fault_flags = &g_fault_flags,
		.stop_flags = &g_stop_flags,
		.heartbeat_status_flags = &g_heartbeat_status_flags,
		.orin_link_ready = &g_orin_link_ready,
		.can_tx_fail_count = &g_recovery.can_tx_fail_count,
		.can_tx_fail_threshold = CAN_TX_CONSEC_FAIL_THRESHOLD,
	};
	control_heartbeat_tx_track(&ctx, err);
}

static void send_control_heartbeat(void)
{
	control_heartbeat_tx_context_t ctx = {
		.tag = TAG,
		.tag_tx = TAG_TX,
		.can_tx_lock = &g_can_tx_lock,
		.hb_seq_lock = &g_hb_seq_lock,
		.twai_ready = &g_twai_ready,
		.can_recovery_in_progress = &g_can_recovery_in_progress,
		.can_tx_in_flight = &g_can_tx_in_flight,
		.heartbeat_seq = &g_heartbeat_seq,
		.control_state = &g_control_state,
		.fault_flags = &g_fault_flags,
		.stop_flags = &g_stop_flags,
		.heartbeat_status_flags = &g_heartbeat_status_flags,
		.orin_link_ready = &g_orin_link_ready,
		.can_tx_fail_count = &g_recovery.can_tx_fail_count,
		.can_tx_fail_threshold = CAN_TX_CONSEC_FAIL_THRESHOLD,
	};
	control_heartbeat_tx_send(&ctx);
}

// ============================================================================
// CAN RX Task
// ============================================================================

/**
 * @brief FreeRTOS task that receives and dispatches inbound CAN frames.
 *
 * Runs at highest priority (7).  Processes Safety heartbeats (0x100)
 * and extended motor responses.
 * Updates the shared command_snapshot under g_cmd_lock and feeds the
 * heartbeat monitor.  Sleeps 100ms when TWAI is down to avoid
 * starving lower-priority tasks.
 *
 * @param param  Unused (NULL)
 */
void can_rx_task(void *param)
{
	(void)param;
	node_task_wdt_add_self_or_log(TAG, "can_rx_task");
	bool wdt_reset_failed = false;
	twai_message_t msg{};

	while (true)
	{
		node_task_wdt_reset_or_log(TAG, "can_rx_task", &wdt_reset_failed);

		// Skip TWAI API calls while driver is down or being recovered.
		// Sleeping 100ms (not 5ms) prevents this priority-7 task from
		// starving lower-priority tasks when the bus is unavailable.
		if (!g_twai_ready)
		{
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}
		if (can_twai_receive(&msg, CAN_RX_TIMEOUT) != ESP_OK)
		{
			vTaskDelay(pdMS_TO_TICKS(5));
			continue;
		}

		if (msg.rtr)
			continue;

#if !defined(CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING) || !defined(CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING)
		// Handle extended frames from UIM2852CA motors.
		// Each motor's producer_id equals its node_id, so the RX
		// parser routes by producer_id to the matching motor_uim2852_state_t.
		if (msg.extd)
		{
			(void)control_can_rx_process_motor_message(
				&msg,
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
				&g_braking_state,
#else
				nullptr,
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
				&g_steering_state,
#else
				nullptr,
#endif
				&g_cmd_lock, &g_cmd, TAG_RX);
			continue;
		}
#else
		if (msg.extd)
			continue;
#endif

		// Process Safety heartbeat (0x100)
		if (msg.identifier == CAN_ID_SAFETY_HEARTBEAT)
		{
			(void)control_can_rx_process_safety_heartbeat(&msg, &g_cmd_lock, &g_cmd, &g_hb_monitor, g_node_safety,
			                                              TAG_RX);
		}
	}
}

// ============================================================================
// Control Task
// ============================================================================

/**
 * @brief FreeRTOS task that runs the control state machine at 50 Hz.
 *
 * Each tick: retries failed components, snapshots CAN RX state,
 * polls driver inputs (pedal ADC, F/R sensor), evaluates motor
 * liveness, runs the pure control_compute_step() function, executes
 * hardware actions (throttle, steering, braking, relays), and sends
 * an immediate heartbeat on state change.  Runs at priority 6.
 *
 * @param param  Unused (NULL)
 */
void control_task(void *param)
{
	(void)param;
	node_task_wdt_add_self_or_log(TAG, "control_task");
	bool wdt_reset_failed = false;

	// Declare loop-state variables before any goto to satisfy C++ scoping rules
#ifdef CONFIG_LOG_CONTROL_STATE_CHANGES
	node_state_t prev_state = 0xFF;
#endif
#ifdef CONFIG_LOG_CONTROL_FAULT_CHANGES
	node_fault_t prev_fault_flags = 0xFF;
#endif
#ifdef CONFIG_LOG_CONTROL_SAFETY_TARGET_CHANGES
	node_state_t prev_target_state = 0xFF;
	node_fault_t prev_safety_fault = 0xFF;
	node_stop_t prev_safety_stop = 0xFF;
#endif
	int32_t last_steering_sent = MOTOR_DEDUP_RESET_STEERING;
	int32_t last_braking_sent = MOTOR_DEDUP_RESET_BRAKING;
	TickType_t next_steering_pt_feed_tick = 0;
	TickType_t next_braking_pt_feed_tick = 0;

	g_throttle = {};

#ifdef CONFIG_LOG_CONTROL_STATE_CHANGES
	prev_state = g_control_state;
#endif
#ifdef CONFIG_LOG_CONTROL_FAULT_CHANGES
	prev_fault_flags = g_fault_flags;
#endif
#ifdef CONFIG_LOG_CONTROL_SAFETY_TARGET_CHANGES
	taskENTER_CRITICAL(&g_cmd_lock);
	prev_target_state = g_cmd.target_state;
	prev_safety_fault = g_cmd.safety_fault_flags;
	prev_safety_stop = g_cmd.safety_stop_flags;
	taskEXIT_CRITICAL(&g_cmd_lock);
#endif

	while (true)
	{
		uint32_t now_ms = node_get_time_ms();
		TickType_t now_tick = xTaskGetTickCount();

		retry_failed_components();

		// Take a thread-safe snapshot of CAN RX state
		command_snapshot_t cmd_local = control_can_rx_snapshot_take_and_clear_motor_fault(&g_cmd_lock, &g_cmd);

		if (cmd_local.motor_fault_code == NODE_FAULT_CONTROL_MOTOR_COMM)
		{
			mark_component_lost(&g_motor_uim2852_steering_ready, "MOTOR_UIM2852_STEERING", "runtime communication fault");
			mark_component_lost(&g_motor_uim2852_braking_ready, "MOTOR_UIM2852_BRAKING", "runtime communication fault");
		}

		control_driver_inputs_update_result_t input_update = {};
		control_driver_inputs_update(&g_driver_inputs, &g_driver_inputs_cfg, now_ms, &input_update);
		if (input_update.pedal_runtime_lost)
		{
			mark_component_lost(&g_driver_inputs.pedal_ready, "PEDAL_INPUT", "runtime adc read failure");
		}
		fr_state_t fr_state = control_driver_inputs_fr_state(&g_driver_inputs);

		// Apply test bypasses
#ifdef CONFIG_BYPASS_INPUT_FR_SENSOR
		fr_state = FR_STATE_FORWARD;
#endif
#ifdef CONFIG_BYPASS_SAFETY_TARGET_MIRROR
		cmd_local.target_state = NODE_STATE_ACTIVE;
#endif
#ifdef CONFIG_BYPASS_SAFETY_ESTOP_MIRROR
		cmd_local.safety_fault_flags = NODE_FAULT_NONE;
		cmd_local.safety_stop_flags = NODE_STOP_NONE;
#endif
#ifdef CONFIG_BYPASS_PLANNER_COMMAND_INPUTS
		// Simulate a planner command payload of all zeros, then apply the same
		// translation used by the Orin link RX path so actuator behavior matches
		// what real planner traffic would produce. braking_position=0 maps to
		// fully released = BRAKE_RELEASE_POSITION (see BRAKING_PLANNER_ZERO_POSITION).
		cmd_local.throttle_cmd = 0;
		cmd_local.steering_cmd = ((int32_t)0 - 360) * (3200 * 50) / 360;
		cmd_local.braking_cmd = BRAKING_PLANNER_ZERO_POSITION;
#endif

		// Check Safety heartbeat timeout — retreat to READY if Safety is gone.
		heartbeat_monitor_check_timeouts(&g_hb_monitor);
		bool safety_alive = heartbeat_monitor_is_alive(&g_hb_monitor, g_node_safety);
#ifdef CONFIG_BYPASS_SAFETY_LIVENESS_CHECKS
		safety_alive = true;
#endif
		if (!safety_alive)
		{
			cmd_local.target_state = NODE_STATE_NOT_READY; // retreat to safe state
		}

		// Check Planner command freshness - zero throttle if stale
#ifndef CONFIG_BYPASS_PLANNER_COMMAND_STALE_CHECKS
		TickType_t planner_age = xTaskGetTickCount() - cmd_local.last_planner_cmd_tick;
		bool planner_cmd_stale = false;

		if (g_control_state == NODE_STATE_ACTIVE && cmd_local.last_planner_cmd_tick > 0)
		{
			// Timeout: no command received for PLANNER_CMD_TIMEOUT
			if (planner_age > PLANNER_CMD_TIMEOUT)
			{
				planner_cmd_stale = true;
			}
			// Stale sequence: same sequence seen PLANNER_CMD_STALE_COUNT times
			// AND command is also past half the timeout window. This prevents
			// false stale detection when Planner legitimately retransmits the
			// same command rapidly within the timeout.
			else if (cmd_local.planner_cmd_stale_count >= PLANNER_CMD_STALE_COUNT &&
			         planner_age > (PLANNER_CMD_TIMEOUT / 2))
			{
				planner_cmd_stale = true;
			}
		}

		if (planner_cmd_stale)
		{
			cmd_local.throttle_cmd = 0;
			// Keep last steering/braking (don't jerk)
		}
#endif

		// Build inputs for pure state machine step
		bool pedal_pressed_now = control_driver_inputs_pedal_pressed(&g_driver_inputs, &g_driver_inputs_cfg);
		bool pedal_rearmed_now = control_driver_inputs_pedal_rearmed(&g_driver_inputs, &g_driver_inputs_cfg);
#ifdef CONFIG_BYPASS_INPUT_PEDAL_ADC
		pedal_pressed_now = false;
		pedal_rearmed_now = true;
#endif

		// Compute motor position error (only meaningful when motor is not
		// actively moving — check in-position/stopped AND error > threshold).
		bool steering_pos_err = false;
		bool braking_pos_err = false;
		int32_t current_braking_position = 0;
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
		if (g_motor_uim2852_steering_ready && !g_steering_state.motion_in_progress)
		{
			taskENTER_CRITICAL(&g_steering_state.lock);
			int32_t steer_abs = g_steering_state.absolute_position;
			int32_t steer_tgt = g_steering_state.target_position;
			taskEXIT_CRITICAL(&g_steering_state.lock);
			int32_t steer_err_val = (steer_abs > steer_tgt) ? (steer_abs - steer_tgt) : (steer_tgt - steer_abs);
			steering_pos_err = (steer_err_val > MOTOR_UIM2852_POSITION_ERROR_THRESHOLD);
		}
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
		if (g_motor_uim2852_braking_ready)
		{
			taskENTER_CRITICAL(&g_braking_state.lock);
			current_braking_position = g_braking_state.absolute_position;
			taskEXIT_CRITICAL(&g_braking_state.lock);
		}
		if (g_motor_uim2852_braking_ready && !g_braking_state.motion_in_progress)
		{
			taskENTER_CRITICAL(&g_braking_state.lock);
			int32_t brake_abs = g_braking_state.absolute_position;
			int32_t brake_tgt = g_braking_state.target_position;
			taskEXIT_CRITICAL(&g_braking_state.lock);
			int32_t brake_err_val = (brake_abs > brake_tgt) ? (brake_abs - brake_tgt) : (brake_tgt - brake_abs);
			braking_pos_err = (brake_err_val > MOTOR_UIM2852_POSITION_ERROR_THRESHOLD);
		}
#endif

		control_inputs_t step_in = {
			.target_state = cmd_local.target_state,
			.throttle_cmd = cmd_local.throttle_cmd,
			.steering_cmd = cmd_local.steering_cmd,
			.braking_cmd = cmd_local.braking_cmd,
			.motor_fault_code = cmd_local.motor_fault_code,
			.stop_flags = g_stop_flags,
			.fr_state = fr_state,
			.pedal_pressed = pedal_pressed_now,
			.pedal_rearmed = pedal_rearmed_now,
			.steering_position_error = steering_pos_err,
			.braking_position_error = braking_pos_err,
			.now_ms = now_ms,
			.boot_start_ms = g_boot_start_ms,
			.init_dwell_ms = INIT_DWELL_MS,
			.enable_start_ms = g_enable_start_ms,
			.enable_sequence_ms = ENABLE_SEQUENCE_MS,
			.enable_work_done = g_enable_work_done,
			.throttle_current = g_throttle.current_level,
			.last_throttle_change_ms = g_throttle.last_change_ms,
			.throttle_slew_interval_ms = THROTTLE_SLEW_INTERVAL_MS,
			.throttle_slew_step = THROTTLE_SLEW_STEP,
			.throttle_min = THROTTLE_LEVEL_MIN,
			.throttle_max = THROTTLE_LEVEL_MAX,
			.last_steering_sent = last_steering_sent,
			.last_braking_sent = last_braking_sent,
			.current_braking_position = current_braking_position,
			.braking_step_limit = compute_braking_pt_step_limit(&g_braking_state),
			.steering_min = STEERING_POSITION_MIN,
			.steering_max = STEERING_POSITION_MAX,
			.braking_min = BRAKE_RELEASE_POSITION,
			.braking_max = BRAKING_POSITION_MAX,
		};

		// Compute next state (pure function — no side effects)
		control_step_result_t step = control_compute_step(g_control_state, g_fault_flags, &step_in);

		// Log precondition failures while not autonomy-ready (edge-triggered)
#ifdef CONFIG_LOG_CONTROL_PRECONDITION_BLOCKED
		{
			static precondition_fail_t prev_precondition_fail = PRECONDITION_OK;
			if (step.precondition_fail != PRECONDITION_OK && step.precondition_fail != prev_precondition_fail)
			{
				ESP_LOGW(TAG, "Enable blocked: fr=%s pedal_pressed=%d pedal_rearmed=%d fault=%s",
				         fr_state_to_string(fr_state), pedal_pressed_now, pedal_rearmed_now,
				         node_fault_to_string(g_fault_flags));
			}
			else if (step.precondition_fail == PRECONDITION_OK && prev_precondition_fail != PRECONDITION_OK)
			{
				ESP_LOGI(TAG, "Enable preconditions cleared");
			}
			prev_precondition_fail = step.precondition_fail;
		}
#endif

		// Execute hardware actions indicated by the step result
		if (step.actions & CONTROL_ACTION_TRIGGER_OVERRIDE)
		{
			execute_trigger_override(step.override_reason);
		}
		if (step.actions & CONTROL_ACTION_DISABLE_AUTONOMY)
		{
			execute_disable_autonomy(step.disable_reason, cmd_local.safety_fault_flags, cmd_local.safety_stop_flags,
			                         step.new_fault_flags);
		}
		if (step.actions & CONTROL_ACTION_START_ENABLE)
		{
			execute_start_enable();
		}
		if (step.actions & CONTROL_ACTION_COMPLETE_ENABLE)
		{
			bool motor_uim2852_ready_for_enable = true;
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
			motor_uim2852_ready_for_enable = motor_uim2852_ready_for_enable && g_motor_uim2852_steering_ready;
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
			motor_uim2852_ready_for_enable = motor_uim2852_ready_for_enable && g_motor_uim2852_braking_ready;
#endif

			if (!motor_uim2852_ready_for_enable)
			{
				// Hold ENABLE and keep relay power on so motor retries can recover.
				// Do not emit FAULT here; otherwise Safety would drop relay power and
				// motor init would deadlock waiting for power.
				step.new_state = NODE_STATE_ENABLE;
				step.new_fault_flags = NODE_FAULT_NONE;
				step.enable_work_done = false; // retry COMPLETE_ENABLE on next tick
				step.status_flags = (node_status_flags_t)(step.status_flags & ~NODE_STATUS_FLAG_ENABLE_COMPLETE);
			}
			else
			{
				execute_complete_enable();
#if !defined(CONFIG_BYPASS_ACTUATOR_THROTTLE)
				// If complete_enable failed, DAC remains non-autonomous — keep retrying.
				if (!dac_mcp4728_is_autonomous())
				{
					step.new_state = NODE_STATE_ENABLE;
					step.new_fault_flags = NODE_FAULT_NONE;
					step.enable_work_done = false; // retry COMPLETE_ENABLE on next tick
					step.status_flags = (node_status_flags_t)(step.status_flags & ~NODE_STATUS_FLAG_ENABLE_COMPLETE);
				}
#endif
			}
		}
		if (step.actions & CONTROL_ACTION_ABORT_ENABLE)
		{
			execute_abort_enable(step.abort_reason);
		}
		if (step.actions & CONTROL_ACTION_APPLY_THROTTLE)
		{
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
			handle_runtime_error(dac_mcp4728_set_level((uint16_t)step.throttle_level), &g_dac_ready, "DAC",
			                     "set level command failed");
#endif
#ifdef CONFIG_LOG_CONTROL_THROTTLE_CHANGES
			{
				static int16_t prev_level = -1;
				if (step.throttle_level != prev_level)
				{
					ESP_LOGI(TAG, "Throttle level: %d -> %d", prev_level, step.throttle_level);
					prev_level = step.throttle_level;
				}
			}
#endif
		}

		// Steering stays on the PT stream cadence using the latest clamped
		// planner-derived target. If Planner skips a cycle, we repeat the last
		// steering target so the FIFO does not run dry.
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
		if (g_motor_uim2852_steering_ready && step.new_state == NODE_STATE_ACTIVE)
		{
			bool fed_frame = false;
			esp_err_t err = feed_pt_stream_if_due(&g_steering_state, MOTOR_NODE_STEERING, step.steering_position, now_tick,
			                                      &next_steering_pt_feed_tick, &fed_frame);
			if (err != ESP_OK && step.send_steering)
			{
				step.new_last_steering = last_steering_sent; // keep old value on failure
			}
#ifdef CONFIG_LOG_ACTUATOR_MOTOR_UIM2852_COMMAND_TX
			if (err != ESP_OK)
			{
				ESP_LOGI(TAG_TX, "Steering PT feed failed: %s", esp_err_to_name(err));
			}
			else if (fed_frame)
			{
				ESP_LOGI(TAG_TX, "Steering PT feed -> %ld%s", (long)step.steering_position,
				         step.send_steering ? "" : " (repeat)");
			}
#endif
			if (fed_frame || err != ESP_OK)
				track_can_tx(err);
		}
		else
		{
			next_steering_pt_feed_tick = 0;
		}
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
		if (g_motor_uim2852_braking_ready && step.new_state == NODE_STATE_ACTIVE)
		{
			bool fed_frame = false;
			esp_err_t err =
				feed_pt_stream_if_due(&g_braking_state, MOTOR_NODE_BRAKING, step.braking_position, now_tick, &next_braking_pt_feed_tick,
				                      &fed_frame);
			if (err != ESP_OK && step.send_braking)
			{
				step.new_last_braking = last_braking_sent;
			}
#ifdef CONFIG_LOG_ACTUATOR_MOTOR_UIM2852_COMMAND_TX
			if (err != ESP_OK)
			{
				ESP_LOGI(TAG_TX, "Braking PT feed failed: %s", esp_err_to_name(err));
			}
			else if (fed_frame)
			{
				ESP_LOGI(TAG_TX, "Braking PT feed -> %ld%s", (long)step.braking_position,
				         step.send_braking ? "" : " (repeat)");
			}
#endif
			if (fed_frame || err != ESP_OK)
				track_can_tx(err);
		}
		else
		{
			next_braking_pt_feed_tick = 0;
		}
#endif

		// Detect fault entry (before overwriting g_fault_flags)
		if (step.new_fault_flags != NODE_FAULT_NONE && step.new_fault_flags != g_fault_flags)
		{
			const char *detail = fault_detail_string(step.new_fault_flags, fr_state, true);
			if (detail)
			{
				ESP_LOGE(TAG, "Fault asserted: %s (detail=%s)", node_fault_to_string(step.new_fault_flags), detail);
			}
			else
			{
				ESP_LOGE(TAG, "Fault asserted: %s", node_fault_to_string(step.new_fault_flags));
			}
		}

		// Detect state change for immediate heartbeat send
		node_state_t old_state = NODE_STATE_INIT;
		node_state_t new_state = NODE_STATE_INIT;

		// Apply state updates as an atomic group for heartbeat snapshots.
		node_status_flags_t status_flags = step.status_flags;
		taskENTER_CRITICAL(&g_hb_seq_lock);
		old_state = g_control_state;
		g_control_state = step.new_state;
		g_fault_flags = step.new_fault_flags;
		g_stop_flags = step.new_stop_flags;
		g_heartbeat_status_flags = status_flags;
		new_state = g_control_state;
		taskEXIT_CRITICAL(&g_hb_seq_lock);

		g_throttle.current_level = step.throttle_level;
		g_throttle.last_change_ms = step.throttle_change_ms;
		g_enable_start_ms = step.enable_start_ms;
		g_enable_work_done = step.enable_work_done;
		last_steering_sent = step.new_last_steering;
		last_braking_sent = step.new_last_braking;

		if (step.new_fault_flags == NODE_FAULT_CONTROL_THROTTLE_INIT && g_dac_ready)
		{
			mark_component_lost(&g_dac_ready, "DAC", "runtime init fault");
		}
		if (step.new_fault_flags == NODE_FAULT_CONTROL_RELAY_INIT && g_dpdt_relay_ready)
		{
			mark_component_lost(&g_dpdt_relay_ready, "DPDT_RELAY", "runtime init fault");
		}
		if (step.new_fault_flags == NODE_FAULT_CONTROL_SENSOR_INVALID && g_driver_inputs.fr_ready)
		{
			char fr_detail[64];
			snprintf(fr_detail, sizeof(fr_detail), "runtime sensor invalid (fr=%s)", fr_state_to_string(fr_state));
			mark_component_lost(&g_driver_inputs.fr_ready, "FR_INPUT", fr_detail);
		}
		if (step.new_fault_flags == NODE_FAULT_CONTROL_MOTOR_COMM)
		{
			mark_component_lost(&g_motor_uim2852_steering_ready, "MOTOR_UIM2852_STEERING", "runtime motor communication fault");
			mark_component_lost(&g_motor_uim2852_braking_ready, "MOTOR_UIM2852_BRAKING", "runtime motor communication fault");
		}

		if (!all_required_components_ready())
		{
			taskENTER_CRITICAL(&g_hb_seq_lock);
			g_control_state = NODE_STATE_NOT_READY;
			g_fault_flags = primary_fault_from_component_health();
			g_heartbeat_status_flags = 0;
			new_state = g_control_state;
			taskEXIT_CRITICAL(&g_hb_seq_lock);
		}
		else if (g_fault_flags == NODE_FAULT_CONTROL_THROTTLE_INIT || g_fault_flags == NODE_FAULT_CONTROL_RELAY_INIT)
		{
			taskENTER_CRITICAL(&g_hb_seq_lock);
			g_fault_flags = NODE_FAULT_NONE;
			g_heartbeat_status_flags = 0;
			new_state = g_control_state;
			taskEXIT_CRITICAL(&g_hb_seq_lock);
		}

		// Send immediate heartbeat on state change
		if (new_state != old_state)
		{
			send_control_heartbeat();
		}

		// Log control-local transitions using local snapshots (not volatile reads)
#ifdef CONFIG_LOG_CONTROL_STATE_CHANGES
		if (new_state != prev_state)
		{
			const char *reason = "none";
			if (step.actions & CONTROL_ACTION_TRIGGER_OVERRIDE)
				reason = override_reason_to_string(step.override_reason);
			else if (step.actions & CONTROL_ACTION_DISABLE_AUTONOMY)
				reason = disable_reason_to_string(step.disable_reason);
			else if (step.actions & CONTROL_ACTION_START_ENABLE)
				reason = "start_enable";
			else if (step.actions & CONTROL_ACTION_COMPLETE_ENABLE)
				reason = "enable_complete";
			else if (step.actions & CONTROL_ACTION_ABORT_ENABLE)
				reason = "abort_enable";

			ESP_LOGI(TAG, "State: %s -> %s (reason=%s)", node_state_to_string(prev_state),
			         node_state_to_string(new_state), reason);
		}
		prev_state = new_state;
#endif

#ifdef CONFIG_LOG_CONTROL_FAULT_CHANGES
		node_fault_t new_fault = step.new_fault_flags;
		if (new_fault != prev_fault_flags)
		{
			const char *detail = fault_detail_string(new_fault, fr_state, true);
			if (detail)
			{
				ESP_LOGI(TAG, "Fault: %s -> %s (detail=%s)", node_fault_to_string(prev_fault_flags),
				         node_fault_to_string(new_fault), detail);
			}
			else
			{
				ESP_LOGI(TAG, "Fault: %s -> %s", node_fault_to_string(prev_fault_flags),
				         node_fault_to_string(new_fault));
			}
		}
		prev_fault_flags = new_fault;
#endif

		// Log Safety mirrored state/fault changes separately
#ifdef CONFIG_LOG_CONTROL_SAFETY_TARGET_CHANGES
		if (cmd_local.target_state != prev_target_state)
		{
			ESP_LOGI(TAG, "Safety target: %s -> %s", node_state_to_string(prev_target_state),
			         node_state_to_string(cmd_local.target_state));
		}
		if (cmd_local.safety_fault_flags != prev_safety_fault)
		{
			ESP_LOGI(TAG, "Safety fault: %s -> %s", node_fault_to_string(prev_safety_fault),
			         node_fault_to_string(cmd_local.safety_fault_flags));
		}
		if (cmd_local.safety_stop_flags != prev_safety_stop)
		{
			ESP_LOGI(TAG, "Safety stop: %s -> %s", node_stop_to_string(prev_safety_stop),
			         node_stop_to_string(cmd_local.safety_stop_flags));
		}

		prev_target_state = cmd_local.target_state;
		prev_safety_fault = cmd_local.safety_fault_flags;
		prev_safety_stop = cmd_local.safety_stop_flags;
#endif

		node_task_wdt_reset_or_log(TAG, "control_task", &wdt_reset_failed);
		vTaskDelay(CONTROL_LOOP_INTERVAL);
	}
}

// ============================================================================
// Heartbeat Task
// ============================================================================

/**
 * @brief FreeRTOS task that sends periodic heartbeats and updates the LED.
 *
 * Runs at priority 4 (lowest of the three tasks).  Every
 * HEARTBEAT_SEND_INTERVAL (100ms) updates the WS2812 status LED
 * color to reflect the current control state and transmits the
 * Control heartbeat CAN frame.
 *
 * @param param  Unused (NULL)
 */
void heartbeat_task(void *param)
{
	(void)param;
	node_task_wdt_add_self_or_log(TAG, "heartbeat_task");
	bool wdt_reset_failed = false;

	while (true)
	{
		node_task_wdt_reset_or_log(TAG, "heartbeat_task", &wdt_reset_failed);

		status_led_set_state(g_control_state);

		// Send heartbeat (periodic 100ms)
		send_control_heartbeat();

		vTaskDelay(HEARTBEAT_SEND_INTERVAL);
	}
}

#endif // !CONFIG_CONTROL_TEST_MODE
