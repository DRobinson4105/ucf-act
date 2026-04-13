/**
 * @file safety_loop.cpp
 * @brief FreeRTOS task implementations for the Safety ESP32 runtime.
 *
 * Contains the CAN RX, safety monitoring, and heartbeat tasks extracted
 * from main.cpp.
 */

#include "safety_loop.h"

#include "battery_monitor.h"
#include "can_protocol.h"
#include "can_twai.h"
#include "esp_log.h"
#include "node_support.h"
#include "relay_srd05vdc.h"
#include "safety_can_rx.h"
#include "safety_config.h"
#include "safety_globals.h"
#include "safety_health.h"
#include "safety_local_inputs.h"
#include "safety_logic.h"
#include "status_led.h"
#include "system_state_machine.h"

// ============================================================================
// CAN RX Task
// ============================================================================

/**
 * @brief FreeRTOS task that receives CAN heartbeats from Control.
 *
 * Runs at highest priority (7).  Processes Control heartbeats (0x120),
 * updating the heartbeat monitor timestamps and mirroring state/fault/
 * status_flags/stop_flags into shared globals.
 * Sleeps 100ms when TWAI is down to avoid starving lower-priority tasks.
 *
 * @param param  Unused (NULL)
 */
// CAN RX task is no longer needed on Safety — Control heartbeats now arrive
// via Orin UART link. The task entry point is kept for build compatibility
// but the task is no longer created in safety_init.
void can_rx_task(void *param)
{
	(void)param;
	vTaskDelete(nullptr);
}

// ============================================================================
// Safety Task
// ============================================================================

/**
 * @brief FreeRTOS task that runs the safety monitoring loop at 20 Hz.
 *
 * Each tick: retries failed components, reads e-stop inputs (push
 * button, RF remote, ultrasonic) with debounce, checks heartbeat
 * timeouts for Planner and Control, evaluates all safety conditions
 * via the pure safety_evaluate() function, manages the power relay,
 * runs the system state machine via system_state_step(), and sends
 * an immediate heartbeat on state change.  Runs at priority 6.
 *
 * Decision logic (e-stop bitmask evaluation, relay) is delegated to
 * the pure safety_logic module.  State advancement is delegated to
 * the pure system_state module.  Both are unit-testable on host.
 *
 * @param param  Unused (NULL)
 */
void safety_task(void *param)
{
	(void)param;
	node_task_wdt_add_self_or_log(TAG, "safety_task");
	bool wdt_reset_failed = false;
	// One-shot autonomy request gate:
	// - latch on Planner request rising edge
	// - consume when transitioning READY -> ENABLE
	// - re-arm only after request drops
	bool request_level_prev = false;
	bool request_latched = false;
	uint32_t active_target_entered_ms = 0;
	uint32_t enable_target_entered_ms = 0;
#ifdef CONFIG_LOG_SAFETY_FAULT_CHANGES
	node_fault_t prev_fault_flags = NODE_FAULT_NONE;
	node_stop_t prev_stop_flags = NODE_STOP_NONE;
#endif
#ifdef CONFIG_LOG_ACTUATOR_POWER_RELAY_CHANGES
	bool prev_relay_enabled = false;
#endif

	while (true)
	{
		retry_failed_components();
		uint32_t now_ms = node_get_time_ms();

		[[maybe_unused]] bool raw_push_button = g_push_button_init_ok ? estop_input_is_active(&g_push_button_cfg) : true;
		[[maybe_unused]] bool raw_rf_remote = g_rf_remote_init_ok ? estop_input_is_active(&g_rf_remote_cfg) : true;

#ifdef CONFIG_LOG_INPUT_PUSH_BUTTON_CHANGES
		{
			static bool s_prev = false;
			static bool s_first = true;
			if (raw_push_button != s_prev || s_first)
			{
				ESP_LOGI(TAG, "Push button %s", raw_push_button ? "PRESSED" : "released");
				s_prev = raw_push_button;
				s_first = false;
			}
		}
#endif
#ifdef CONFIG_LOG_INPUT_RF_REMOTE_CHANGES
		{
			static bool s_prev = false;
			static bool s_first = true;
			if (raw_rf_remote != s_prev || s_first)
			{
				ESP_LOGI(TAG, "RF remote %s", raw_rf_remote ? "ENGAGED" : "disengaged");
				s_prev = raw_rf_remote;
				s_first = false;
			}
		}
#endif

		safety_local_inputs_snapshot_t local_inputs = {};
		safety_local_inputs_poll(&g_local_inputs, &g_local_inputs_cfg, g_push_button_init_ok, &g_push_button_cfg,
		                         g_rf_remote_init_ok, &g_rf_remote_cfg, g_ultrasonic_init_ok, now_ms, &local_inputs);

		safety_inputs_t inputs = {
			.push_button_active = local_inputs.push_button_active,
			.rf_remote_active = local_inputs.rf_remote_active,
			.ultrasonic_too_close = local_inputs.ultrasonic_too_close,
			.ultrasonic_healthy = local_inputs.ultrasonic_healthy,
			.planner_alive = true,                  // updated below from heartbeat monitor
			.control_alive = true,                  // updated below from heartbeat monitor
			.planner_fault_flags = NODE_FAULT_NONE, // updated from grouped snapshot below
			.control_fault_flags = NODE_FAULT_NONE, // updated from grouped snapshot below
			.planner_stop_flags = NODE_STOP_NONE,   // updated from grouped snapshot below
			.control_stop_flags = NODE_STOP_NONE,   // updated from grouped snapshot below
		};

		hb_mirror_snapshot_t snap = safety_hb_mirror_read(&g_hb_mirror_lock, &g_hb_mirror);
		inputs.planner_fault_flags = snap.planner_fault_flags;
		inputs.control_fault_flags = snap.control_fault_flags;
		inputs.planner_stop_flags = snap.planner_stop_flags;
		inputs.control_stop_flags = snap.control_stop_flags;

		// Check heartbeat timeouts
		heartbeat_monitor_check_timeouts(&g_hb_monitor);
		inputs.planner_alive = heartbeat_monitor_is_alive(&g_hb_monitor, g_node_planner);
		inputs.control_alive = heartbeat_monitor_is_alive(&g_hb_monitor, g_node_control);

		// Apply test bypasses
#ifdef CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS
		inputs.planner_alive = true;
		inputs.planner_fault_flags = NODE_FAULT_NONE;
		inputs.planner_stop_flags = NODE_STOP_NONE;
#endif
#ifdef CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS
		inputs.control_alive = true;
		inputs.control_fault_flags = NODE_FAULT_NONE;
		inputs.control_stop_flags = NODE_STOP_NONE;
#endif
#ifdef CONFIG_BYPASS_INPUT_PUSH_BUTTON
		inputs.push_button_active = false;
#endif
#ifdef CONFIG_BYPASS_INPUT_RF_REMOTE
		inputs.rf_remote_active = false;
#endif
#ifdef CONFIG_BYPASS_INPUT_ULTRASONIC
		inputs.ultrasonic_too_close = false;
		inputs.ultrasonic_healthy = true;
#endif

		// Ultrasonic health edge logging uses the hysteresis-filtered value.
		// Transition tracking is maintained by the hysteresis block above;
		// we only log here when the filtered state actually changes.
		//
		// A startup grace period (ULTRASONIC_LOG_GRACE_TICKS) suppresses
		// LOST/REGAINED logging while the sensor's UART RX task establishes
		// its first valid measurement. Without this, the sensor always
		// appears briefly unhealthy at boot (no data yet) then immediately
		// recovers, producing a spurious LOST -> REGAINED pair.
#ifndef CONFIG_BYPASS_INPUT_ULTRASONIC
		{
			static bool s_ultrasonic_log_prev = false;
			static bool s_ultrasonic_log_seen = false;
			if (g_ultrasonic_init_ok)
			{
				TickType_t since_init = xTaskGetTickCount() - g_ultrasonic_init_tick;
				if (since_init < ULTRASONIC_LOG_GRACE_TICKS)
				{
					// Grace period: don't log, but track state so the
					// baseline is correct when the window expires.
					s_ultrasonic_log_prev = local_inputs.ultrasonic_healthy;
					s_ultrasonic_log_seen = true;
				}
				else if (!s_ultrasonic_log_seen)
				{
					// First sample after grace: accept as baseline, no log.
					s_ultrasonic_log_prev = local_inputs.ultrasonic_healthy;
					s_ultrasonic_log_seen = true;
				}
				else if (local_inputs.ultrasonic_healthy != s_ultrasonic_log_prev)
				{
					if (local_inputs.ultrasonic_healthy)
					{
						log_component_regained("ULTRASONIC");
					}
					else
					{
						log_component_lost("ULTRASONIC", "runtime health timeout");
					}
					s_ultrasonic_log_prev = local_inputs.ultrasonic_healthy;
				}
			}
		}
#endif

		// Update battery monitor (non-safety-critical, informational only)
#ifdef CONFIG_BYPASS_INPUT_BATTERY_MONITOR
		g_battery_soc = 50;
#else
		if (g_battery_monitor_init_ok)
		{
			battery_monitor_update(now_ms);
			g_battery_soc = battery_monitor_get_soc();
		}
#endif

		// Evaluate all safety conditions (pure function — no side effects)
		safety_decision_t decision = safety_evaluate(&inputs);

		// Missing critical output/transport components force fail-safe behavior.
		// Relay unavailable = system cannot cut power = force relay fault.
		if (!g_relay_init_ok)
		{
			decision.stop_active = true;
			decision.fault_flags |= NODE_FAULT_SAFETY_RELAY_UNAVAILABLE;
		}

		// Snapshot state values used by heartbeat send path.
		// Keep these updates atomic with respect to send_safety_heartbeat().
		node_state_t current_target = g_target_state;

		// Run system state machine (pure function)
		system_state_inputs_t ss_in = {
			.current_target = current_target,
			.now_ms = now_ms,
			.boot_start_ms = g_boot_start_ms,
			.init_dwell_ms = INIT_DWELL_MS,
			.stop_active = decision.stop_active,
			.planner_state = snap.planner_state,
			.control_state = snap.control_state,
			.planner_alive = inputs.planner_alive,
			.control_alive = inputs.control_alive,
			.planner_enable_complete = (snap.planner_status_flags & NODE_STATUS_FLAG_ENABLE_COMPLETE) != 0,
			.control_enable_complete = (snap.control_status_flags & NODE_STATUS_FLAG_ENABLE_COMPLETE) != 0,
			.autonomy_request = false,
			.autonomy_hold = false,
			.active_entry_grace =
				(current_target == NODE_STATE_ACTIVE && (now_ms - active_target_entered_ms) < ACTIVE_ENTRY_GRACE_MS),
			.enable_elapsed_ms =
				(current_target == NODE_STATE_ENABLE && enable_target_entered_ms > 0)
					? (now_ms - enable_target_entered_ms) : 0,
			.enable_timeout_ms = ENABLE_TIMEOUT_MS,
		};

		// Planner bypass: simulate a cooperative Planner for bench bring-up.
		// Mirror Safety's own target so the state machine sees a node that
		// tracks Safety through every phase: READY, ENABLE, and ACTIVE.
#ifdef CONFIG_BYPASS_PLANNER_STATE_MIRROR
		if (g_target_state == NODE_STATE_ACTIVE)
			ss_in.planner_state = NODE_STATE_ACTIVE;
		else if (g_target_state == NODE_STATE_ENABLE)
			ss_in.planner_state = NODE_STATE_ENABLE;
		else
			ss_in.planner_state = NODE_STATE_READY;
		ss_in.planner_enable_complete = true;
#endif
		// Control bypass: same mirroring for Control.
#ifdef CONFIG_BYPASS_CONTROL_STATE_MIRROR
		if (g_target_state == NODE_STATE_ACTIVE)
			ss_in.control_state = NODE_STATE_ACTIVE;
		else if (g_target_state == NODE_STATE_ENABLE)
			ss_in.control_state = NODE_STATE_ENABLE;
		else
			ss_in.control_state = NODE_STATE_READY;
		ss_in.control_enable_complete = true;
#endif

		bool planner_request_level = (snap.planner_status_flags & NODE_STATUS_FLAG_AUTONOMY_REQUEST) != 0;
		bool request_accept_window =
			(ss_in.current_target == NODE_STATE_READY && !ss_in.stop_active && ss_in.planner_alive &&
		     ss_in.control_alive && ss_in.planner_state == NODE_STATE_READY && ss_in.control_state == NODE_STATE_READY);
#ifdef CONFIG_BYPASS_PLANNER_AUTONOMY_GATE
		planner_request_level = true;
#endif

		if (!planner_request_level)
		{
#ifdef CONFIG_LOG_SAFETY_STATE_CHANGES
			if (request_level_prev)
				ESP_LOGI(TAG, "Autonomy request re-armed (request dropped)");
#endif
			request_level_prev = false;
		}
		else if (!request_level_prev)
		{
			request_level_prev = true;
			if (request_accept_window)
			{
				request_latched = true;
#ifdef CONFIG_LOG_SAFETY_STATE_CHANGES
				ESP_LOGI(TAG, "Autonomy request latched");
#endif
			}
		}

		ss_in.autonomy_request = request_latched;
		ss_in.autonomy_hold = planner_request_level;
#ifdef CONFIG_BYPASS_PLANNER_AUTONOMY_GATE
		ss_in.autonomy_request = true;
		ss_in.autonomy_hold = true;
#endif

#ifdef CONFIG_LOG_SAFETY_STATE_CHANGES
		static bool prev_waiting_for_request = false;
		bool waiting_for_request =
			(ss_in.current_target == NODE_STATE_READY && !ss_in.stop_active && ss_in.planner_alive &&
		     ss_in.control_alive && ss_in.planner_state == NODE_STATE_READY &&
		     ss_in.control_state == NODE_STATE_READY && !ss_in.autonomy_request);
		if (waiting_for_request && !prev_waiting_for_request)
		{
			ESP_LOGI(TAG, "Waiting for autonomy request");
		}
		else if (!waiting_for_request && prev_waiting_for_request)
		{
			ESP_LOGI(TAG, "Autonomy request wait cleared");
		}
		prev_waiting_for_request = waiting_for_request;
#endif

		system_state_result_t ss_out = system_state_step(&ss_in);

		if (ss_out.new_target == NODE_STATE_ENABLE)
		{
			if (current_target != NODE_STATE_ENABLE)
				enable_target_entered_ms = now_ms;
		}
		else
		{
			enable_target_entered_ms = 0;
		}

		if (ss_out.new_target == NODE_STATE_ACTIVE)
		{
			if (current_target != NODE_STATE_ACTIVE)
				active_target_entered_ms = now_ms;
		}
		else
		{
			active_target_entered_ms = 0;
		}

		// Relay power is allowed only when e-stop is clear and target is
		// in the autonomy power window (ENABLE/ACTIVE).
		bool relay_should_enable =
			!decision.stop_active && (ss_out.new_target == NODE_STATE_ENABLE || ss_out.new_target == NODE_STATE_ACTIVE);

#ifndef CONFIG_BYPASS_PLANNER_AUTONOMY_GATE
		if (ss_out.target_changed && current_target == NODE_STATE_READY && ss_out.new_target == NODE_STATE_ENABLE &&
		    request_latched)
		{
			request_latched = false;
#ifdef CONFIG_LOG_SAFETY_STATE_CHANGES
			ESP_LOGI(TAG, "Autonomy request consumed");
#endif
		}
#endif

#ifdef CONFIG_LOG_SAFETY_STATE_CHANGES
		if (ss_out.target_changed)
		{
			const char *reason = "none";
			if (current_target == NODE_STATE_READY && ss_out.new_target == NODE_STATE_ENABLE)
				reason = "autonomy_request";
			else if (current_target == NODE_STATE_ENABLE && ss_out.new_target == NODE_STATE_ACTIVE)
				reason = "enable_complete";
			else if (ss_out.new_target == NODE_STATE_READY || ss_out.new_target == NODE_STATE_NOT_READY)
			{
				if ((current_target == NODE_STATE_ENABLE || current_target == NODE_STATE_ACTIVE) &&
				    !ss_in.autonomy_hold && decision.fault_flags == NODE_FAULT_NONE &&
				    decision.stop_flags == NODE_STOP_NONE)
					reason = "autonomy_halt";
				else if (ss_out.new_target == NODE_STATE_NOT_READY && decision.fault_flags == NODE_FAULT_NONE &&
				         decision.stop_flags == NODE_STOP_NONE)
					reason = "not_ready";
				else if (decision.stop_flags != NODE_STOP_NONE)
					reason = node_stop_to_string(decision.stop_flags);
				else
					reason = safety_fault_to_log_string(decision.fault_flags);
			}

			ESP_LOGI(TAG, "Target: %s -> %s (reason=%s)", node_state_to_string(current_target),
			         node_state_to_string(ss_out.new_target), reason);

			// Log node detail when retreat is caused by a specific node
			if ((ss_out.new_target == NODE_STATE_READY || ss_out.new_target == NODE_STATE_NOT_READY) &&
			    (decision.fault_flags != NODE_FAULT_NONE || decision.stop_flags != NODE_STOP_NONE))
			{
				if (decision.fault_flags & NODE_FAULT_SAFETY_CONTROL_ISSUE)
					ESP_LOGI(TAG, "  Control issue: %s", node_fault_to_string(snap.control_fault_flags));
				if (decision.fault_flags & NODE_FAULT_SAFETY_PLANNER_ISSUE)
					ESP_LOGI(TAG, "  Planner issue: %s", node_fault_to_string(snap.planner_fault_flags));
				if ((decision.stop_flags & NODE_STOP_OPERATOR_ANY) != 0)
					ESP_LOGI(TAG, "  Control stop: %s", node_stop_to_string(snap.control_stop_flags));
				if (decision.stop_flags & NODE_STOP_APP_REQUEST)
					ESP_LOGI(TAG, "  Planner stop: %s", node_stop_to_string(snap.planner_stop_flags));
			}
		}
#endif

		// Log stop/fault code changes (edge-triggered)
#ifdef CONFIG_LOG_SAFETY_FAULT_CHANGES
		if (decision.fault_flags != prev_fault_flags)
		{
			if (decision.fault_flags != NODE_FAULT_NONE)
			{
				ESP_LOGW(TAG, "Safety fault: %s -> %s", safety_fault_to_log_string(prev_fault_flags),
				         safety_fault_to_log_string(decision.fault_flags));
			}
			else
			{
				ESP_LOGI(TAG, "Safety fault: %s -> %s", safety_fault_to_log_string(prev_fault_flags),
				         safety_fault_to_log_string(decision.fault_flags));
			}
		}
		if (decision.stop_flags != prev_stop_flags)
		{
			ESP_LOGI(TAG, "Safety stop: %s -> %s", node_stop_to_string(prev_stop_flags),
			         node_stop_to_string(decision.stop_flags));
		}
		prev_fault_flags = decision.fault_flags;
		prev_stop_flags = decision.stop_flags;
#endif

		// Publish new target + cause channels atomically for heartbeat snapshots.
		taskENTER_CRITICAL(&g_safety_hb_seq_lock);
		g_fault_flags = decision.fault_flags;
		g_stop_flags = decision.stop_flags;
		g_target_state = ss_out.new_target;
		taskEXIT_CRITICAL(&g_safety_hb_seq_lock);

		// Control power relay based on safety decision
		if (g_relay_init_ok)
		{
			if (relay_should_enable)
			{
				esp_err_t relay_err = relay_srd05vdc_enable(&g_relay_cfg);
				if (relay_err != ESP_OK)
				{
					ESP_LOGE(TAG, "relay_srd05vdc_enable FAILED: %s — relay may be stuck OFF",
					         esp_err_to_name(relay_err));
					mark_component_lost(&g_relay_init_ok, "RELAY", "runtime enable command failed");
					// Immediately force e-stop in this iteration — don't wait
					// for the next loop cycle to detect g_relay_init_ok == false.
					taskENTER_CRITICAL(&g_safety_hb_seq_lock);
					g_fault_flags = NODE_FAULT_SAFETY_RELAY_UNAVAILABLE;
					g_stop_flags = NODE_STOP_NONE;
					g_target_state = NODE_STATE_NOT_READY;
					taskEXIT_CRITICAL(&g_safety_hb_seq_lock);
					send_safety_heartbeat(true);
				}
#ifdef CONFIG_LOG_ACTUATOR_POWER_RELAY_CHANGES
				else if (!prev_relay_enabled)
					ESP_LOGI(TAG, "Power relay ENABLED");
#endif
			}
			else
			{
				esp_err_t relay_err = relay_srd05vdc_disable(&g_relay_cfg);
				if (relay_err != ESP_OK)
				{
					ESP_LOGE(TAG, "relay_srd05vdc_disable FAILED: %s — relay may be stuck ON!",
					         esp_err_to_name(relay_err));
					mark_component_lost(&g_relay_init_ok, "RELAY", "runtime disable command failed");
					// Immediately force e-stop — relay state is indeterminate.
					taskENTER_CRITICAL(&g_safety_hb_seq_lock);
					g_fault_flags = NODE_FAULT_SAFETY_RELAY_UNAVAILABLE;
					g_stop_flags = NODE_STOP_NONE;
					g_target_state = NODE_STATE_NOT_READY;
					taskEXIT_CRITICAL(&g_safety_hb_seq_lock);
					send_safety_heartbeat(true);
				}
#ifdef CONFIG_LOG_ACTUATOR_POWER_RELAY_CHANGES
				else if (prev_relay_enabled)
					ESP_LOGI(TAG, "Power relay DISABLED");
#endif
			}
#ifdef CONFIG_LOG_ACTUATOR_POWER_RELAY_CHANGES
			prev_relay_enabled = relay_should_enable;
#endif
		}

		// Send immediate heartbeat on target state change
		if (ss_out.target_changed)
		{
			send_safety_heartbeat(true);
		}

		node_task_wdt_reset_or_log(TAG, "safety_task", &wdt_reset_failed);
		vTaskDelay(SAFETY_LOOP_INTERVAL);
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
 * color to reflect the current Safety target state and transmits the
 * Safety heartbeat CAN frame.
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

		status_led_set_state(g_target_state);

		// Send Safety heartbeat (periodic)
		send_safety_heartbeat(false);

		vTaskDelay(HEARTBEAT_SEND_INTERVAL);
	}
}
