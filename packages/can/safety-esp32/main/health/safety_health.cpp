/**
 * @file safety_health.cpp
 * @brief Health-related function implementations for the Safety ESP32 node.
 */

#include "safety_health.h"
#include "safety_globals.h"
#include "safety_config.h"
#include "node_support.h"
#include "can_twai.h"
#include "safety_recovery.h"
#include "safety_heartbeat_tx.h"

/** @brief Log that a component has been lost (forwarded from node_support). */
void log_component_lost(const char *name, const char *detail)
{
	node_log_component_lost(TAG, name, detail);
}

/** @brief Log that a component has been recovered (forwarded from node_support). */
void log_component_regained(const char *name)
{
	node_log_component_regained(TAG, name);
}

/** @brief Mark a component as lost and log the failure (forwarded from node_support). */
void mark_component_lost(volatile bool *ready, const char *name, const char *detail)
{
	node_mark_component_lost(ready, TAG, name, detail);
}

/**
 * @brief Convert Safety fault flags to a logging string.
 *
 * @param fault_flags  Fault flags to convert
 * @return Static string label for logging
 */
const char *safety_fault_to_log_string(node_fault_t fault_flags)
{
	return node_fault_to_string(fault_flags);
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
 * Called every safety loop tick, but paced at RETRY_INTERVAL_MS to
 * avoid hammering init calls.  Retries are infinite with no cap.
 * Covers push button, RF remote, relay, ultrasonic, and TWAI.
 * TWAI recovery uses a full teardown/reinit cycle with an active
 * probe heartbeat to verify bus connectivity.
 */
void retry_failed_components(void)
{
	safety_recovery_context_t ctx = {
		.tag = TAG,
		.last_retry_ms = &g_last_retry_ms,
		.retry_interval_ms = RETRY_INTERVAL_MS,
		.retry_log_every_n = RETRY_LOG_EVERY_N,
		.can_tx_lock = &g_can_tx_lock,
		.can_recovery_in_progress = &g_can_recovery_in_progress,
		.twai_ready = &g_twai_ready,
		.twai_tx_gpio = TWAI_TX_GPIO,
		.twai_rx_gpio = TWAI_RX_GPIO,
		.probe_state = g_target_state,
		.probe_fault_flags = g_fault_flags,
		.probe_stop_flags = g_stop_flags,
		.probe_soc_pct = g_battery_soc,
		.quiesce_can_rx = quiesce_can_rx,
		.push_button_cfg = &g_push_button_cfg,
		.push_button_init_ok = &g_push_button_init_ok,
		.rf_remote_cfg = &g_rf_remote_cfg,
		.rf_remote_init_ok = &g_rf_remote_init_ok,
		.relay_cfg = &g_relay_cfg,
		.relay_init_ok = &g_relay_init_ok,
		.ultrasonic_cfg = &g_ultrasonic_cfg,
		.ultrasonic_init_ok = &g_ultrasonic_init_ok,
		.ultrasonic_init_tick = &g_ultrasonic_init_tick,
		.local_inputs = &g_local_inputs,
		.local_inputs_cfg = &g_local_inputs_cfg,
		.battery_cfg = &g_battery_cfg,
		.battery_monitor_init_ok = &g_battery_monitor_init_ok,
	};
	safety_retry_failed_components(&ctx);
}

/**
 * @brief Encode and transmit the Safety heartbeat frame on CAN.
 *
 * Atomically snapshots the current target state, e-stop fault code,
 * and sequence number, encodes them into the CAN heartbeat format,
 * and sends via TWAI.  Called from safety_task on state change
 * (immediate) and from heartbeat_task on the 100ms periodic timer.
 * Guards against concurrent recovery by checking g_twai_ready and
 * tracking in-flight TX count under spinlock.
 *
 * @param log_as_change  true for state-change-triggered sends (logged as "change"),
 *                       false for periodic sends (logged as "periodic")
 */
void send_safety_heartbeat(bool log_as_change)
{
	safety_heartbeat_tx_context_t ctx = {
		.tag = TAG,
		.tag_tx = TAG_TX,
		.can_tx_lock = &g_can_tx_lock,
		.hb_seq_lock = &g_safety_hb_seq_lock,
		.twai_ready = &g_twai_ready,
		.can_recovery_in_progress = &g_can_recovery_in_progress,
		.can_tx_in_flight = &g_can_tx_in_flight,
		.heartbeat_seq = &g_safety_hb_seq,
		.target_state = &g_target_state,
		.fault_flags = &g_fault_flags,
		.stop_flags = &g_stop_flags,
		.battery_soc = &g_battery_soc,
		.hb_tx_failing = &g_hb_tx_failing,
		.orin_link_ready = &g_orin_link_ready,
		.can_tx_fail_count = &g_can_tx_fail_count,
		.can_tx_fail_threshold = CAN_TX_FAIL_THRESHOLD,
		.fault_to_log_string = safety_fault_to_log_string,
	};
	safety_heartbeat_tx_send(&ctx, log_as_change);
}
