/**
 * @file control_health.cpp
 * @brief Component health evaluation, fault gating, and recovery orchestration for Control ESP32.
 */

#include "control_health.h"

#include "can_twai.h"
#include "control_config.h"
#include "control_globals.h"
#include "control_recovery.h"
#include "node_support.h"

/**
 * @brief Check whether all non-bypassed base components report ready.
 *
 * Evaluates the health flags for every component that is not
 * compile-time bypassed, excluding motor comm availability.
 *
 * Motor comm is intentionally excluded from this baseline readiness check:
 * motors are powered by Safety's relay only in ENABLE/ACTIVE, so requiring
 * them in NOT_READY/READY would create a power-dependency deadlock.
 *
 * @return true if every required non-motor component is initialized and healthy
 */
bool all_required_components_ready(void)
{
	bool ready = true;
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	if (!g_dac_ready)
		ready = false;
	if (!g_dpdt_relay_ready)
		ready = false;
#endif
#ifndef CONFIG_BYPASS_INPUT_PEDAL_ADC
	if (!g_driver_inputs.pedal_ready)
		ready = false;
#endif
#ifndef CONFIG_BYPASS_INPUT_FR_SENSOR
	if (!g_driver_inputs.fr_ready)
		ready = false;
#endif
	return ready;
}

/**
 * @brief Return the highest-priority fault code for the first unhealthy base component.
 *
 * Scans non-bypassed base components in priority order (throttle, relay,
 * sensors) and returns the fault code for the first one that is not ready.
 * Motor comm health is handled by runtime motor fault paths in ENABLE/ACTIVE.
 * Returns NODE_FAULT_NONE if all baseline components are healthy.
 *
 * @return Fault code corresponding to the first failed component
 */
node_fault_t primary_fault_from_component_health(void)
{
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	if (!g_dac_ready)
		return NODE_FAULT_CONTROL_THROTTLE_INIT;
	if (!g_dpdt_relay_ready)
		return NODE_FAULT_CONTROL_RELAY_INIT;
#endif
#if !defined(CONFIG_BYPASS_INPUT_PEDAL_ADC) || !defined(CONFIG_BYPASS_INPUT_FR_SENSOR)
	if (!g_driver_inputs.pedal_ready || !g_driver_inputs.fr_ready)
		return NODE_FAULT_CONTROL_SENSOR_INVALID;
#endif
	return NODE_FAULT_NONE;
}

/**
 * @brief Convert an F/R gear state enum to a human-readable string.
 *
 * @param state  Gear state to convert
 * @return Static string label ("forward", "reverse", "neutral", "wiring_fault", or "unknown")
 */
const char *fr_state_to_string(fr_state_t state)
{
	switch (state)
	{
	case FR_STATE_FORWARD:
		return "forward";
	case FR_STATE_REVERSE:
		return "reverse";
	case FR_STATE_NEUTRAL:
		return "neutral";
	case FR_STATE_INVALID:
		return "wiring_fault";
	default:
		return "unknown";
	}
}

/**
 * @brief Build a diagnostic detail string for NODE_FAULT_CONTROL_SENSOR_INVALID.
 *
 * Identifies which sensor input caused the fault: F/R invalid reading,
 * pedal input unavailable, F/R input unavailable, or a combination.
 *
 * @param fr_state       Current F/R gear state
 * @param has_fr_sample  true if at least one F/R reading has been taken
 * @return Static detail string describing the sensor failure
 */
const char *sensor_fault_detail_string(fr_state_t fr_state, bool has_fr_sample)
{
	if (has_fr_sample && fr_state == FR_STATE_INVALID)
	{
		return "fr_state_invalid";
	}

	bool pedal_unavailable = false;
	bool fr_unavailable = false;

#ifndef CONFIG_BYPASS_INPUT_PEDAL_ADC
	pedal_unavailable = !g_driver_inputs.pedal_ready;
#endif
#ifndef CONFIG_BYPASS_INPUT_FR_SENSOR
	fr_unavailable = !g_driver_inputs.fr_ready;
#endif

	if (pedal_unavailable && fr_unavailable)
		return "pedal_input+fr_input_unavailable";
	if (pedal_unavailable)
		return "pedal_input_unavailable";
	if (fr_unavailable)
		return "fr_input_unavailable";
	return "unspecified";
}

/**
 * @brief Return a diagnostic detail string for a given fault code, or NULL.
 *
 * Currently only provides detail for NODE_FAULT_CONTROL_SENSOR_INVALID.
 * All other fault codes return NULL (no additional detail).
 *
 * @param fault_flags    Active fault flags
 * @param fr_state       Current F/R gear state
 * @param has_fr_sample  true if at least one F/R reading has been taken
 * @return Detail string or NULL if no additional detail is available
 */
const char *fault_detail_string(node_fault_t fault_flags, fr_state_t fr_state, bool has_fr_sample)
{
	if (fault_flags == NODE_FAULT_CONTROL_SENSOR_INVALID)
	{
		return sensor_fault_detail_string(fr_state, has_fr_sample);
	}
	return nullptr;
}

/** @brief Mark a component as lost and log the failure (forwarded from node_support). */
void mark_component_lost(volatile bool *ready, const char *name, const char *detail)
{
	node_mark_component_lost(ready, TAG, name, detail);
}

/**
 * @brief Mark a component lost if a runtime operation returned an error.
 *
 * No-op when err is ESP_OK.  Otherwise delegates to mark_component_lost()
 * to clear the ready flag and log the failure.
 *
 * @param err     Error code from the component operation
 * @param ready   Pointer to the component's health flag
 * @param name    Component name for logging
 * @param detail  Failure detail string for logging
 */
[[maybe_unused]] void handle_runtime_error(esp_err_t err, volatile bool *ready, const char *name, const char *detail)
{
	if (err == ESP_OK)
		return;
	mark_component_lost(ready, name, detail);
}

/**
 * @brief Execute the safe shutdown sequence for a UIM2852 motor.
 *
 * Issues ST (stop) then disables the motor driver (MO=0) via the
 * motor dispatch layer.  Each step is best-effort — failures are
 * logged but do not abort the sequence so subsequent steps still execute.
 *
 * @param node_id  CAN node ID of the motor to shut down
 * @param ready    Pointer to the motor's health flag
 * @param label    Motor name for error logging (e.g. "MOTOR_UIM2852_STEERING")
 */
[[maybe_unused]] void motor_uim2852_shutdown(uint8_t node_id, volatile bool *ready, const char *label)
{
	motor_cmd_t cmd;
	motor_dispatch_result_t res;

	// Emergency stop (ST)
	motor_cmd_st_stop(node_id, true, &cmd);
	res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
	if (res != MOTOR_DISPATCH_RESULT_OK)
		mark_component_lost(ready, label, "emergency stop command failed");

	// Disable motor (MO=0)
	motor_cmd_mo_set(node_id, true, MOTOR_MO_STATE_DISABLE, &cmd);
	res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
	if (res != MOTOR_DISPATCH_RESULT_OK)
		mark_component_lost(ready, label, "disable command failed");
}

/**
 * @brief Apply base component-health fault gating.
 *
 * If any required base component is unhealthy, force fault_flags to the
 * highest-priority component fault and retreat state to NOT_READY.
 * If all required base components recover and the current fault code is a
 * base component fault, clear it.
 */
void update_control_state_from_component_health(void)
{
	if (!all_required_components_ready())
	{
		taskENTER_CRITICAL(&g_hb_seq_lock);
		g_fault_flags = primary_fault_from_component_health();
		if (g_control_state != NODE_STATE_INIT)
			g_control_state = NODE_STATE_NOT_READY;
		taskEXIT_CRITICAL(&g_hb_seq_lock);
		return;
	}

	if (g_fault_flags == NODE_FAULT_CONTROL_THROTTLE_INIT || g_fault_flags == NODE_FAULT_CONTROL_RELAY_INIT)
	{
		taskENTER_CRITICAL(&g_hb_seq_lock);
		g_fault_flags = NODE_FAULT_NONE;
		taskEXIT_CRITICAL(&g_hb_seq_lock);
	}
}

/**
 * @brief Wait for CAN RX and heartbeat tasks to finish in-flight TWAI calls.
 *
 * Must be called after setting g_twai_ready = false, before touching
 * the TWAI driver for recovery. Forwarded from node_support.
 */
void quiesce_can_rx()
{
	can_twai_quiesce(&g_can_tx_lock, &g_can_tx_in_flight);
}

/**
 * @brief Re-initialize any failed components at a rate-limited cadence.
 *
 * Called every control loop tick, but paced at RETRY_INTERVAL_MS to
 * avoid hammering init calls.  Retries are infinite with no cap.
 * For TWAI recovery, performs a full teardown/reinit cycle with an
 * active probe heartbeat to verify bus connectivity.  On success,
 * calls update_control_state_from_component_health() to potentially
 * clear base component issue causes.
 */
void retry_failed_components(void)
{
	control_recovery_context_t ctx = {
		.tag = TAG,
		.last_retry_ms = &g_last_retry_ms,
		.retry_interval_ms = RETRY_INTERVAL_MS,
		.retry_log_every_n = RETRY_LOG_EVERY_N,
		.can_tx_lock = &g_can_tx_lock,
		.can_recovery_in_progress = &g_can_recovery_in_progress,
		.twai_ready = &g_twai_ready,
		.twai_tx_gpio = TWAI_TX_GPIO,
		.twai_rx_gpio = TWAI_RX_GPIO,
		.probe_state = g_control_state,
		.probe_fault_flags = g_fault_flags,
		.probe_stop_flags = g_stop_flags,
		.probe_status_flags = g_heartbeat_status_flags,
		.quiesce_can_rx = quiesce_can_rx,
		.dac_cfg = &g_dac_cfg,
		.dac_ready = &g_dac_ready,
		.dpdt_relay_cfg = &g_dpdt_relay_cfg,
		.dpdt_relay_ready = &g_dpdt_relay_ready,
		.driver_inputs = &g_driver_inputs,
		.driver_inputs_cfg = &g_driver_inputs_cfg,
		.fr_pc817_cfg = &g_fr_pc817_cfg,
		.motor_uim2852_braking_ready = &g_motor_uim2852_braking_ready,
		.motor_uim2852_steering_ready = &g_motor_uim2852_steering_ready,
	};
	control_retry_failed_components(&ctx);
	update_control_state_from_component_health();
}
