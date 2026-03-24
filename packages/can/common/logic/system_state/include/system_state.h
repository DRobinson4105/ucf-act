/**
 * @file system_state.h
 * @brief Pure decision logic for Safety ESP32 system state machine.
 *
 * The Safety ESP32 is the system target-state authority — it advances only
 * NOT_READY -> READY -> ENABLE -> ACTIVE.
 *
 * This module is a pure function library with no hardware dependencies,
 * allowing comprehensive unit testing on the host.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "can_protocol.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// System State Machine Inputs
// ============================================================================

typedef struct
{
	// Current target state (what Safety last commanded)
	node_state_t current_target;

	// Timing for INIT -> NOT_READY dwell
	uint32_t now_ms;
	uint32_t boot_start_ms;
	uint32_t init_dwell_ms;

	// Stop/fault evaluation result (from safety_logic)
	bool stop_active;

	// Node actual states (from heartbeats)
	node_state_t planner_state;
	node_state_t control_state;

	// Heartbeat liveness
	bool planner_alive;
	bool control_alive;

	// Enable completion flags (from heartbeat status_flags field)
	bool planner_enable_complete;
	bool control_enable_complete;

	// Planner/Orin autonomy-enable request gate (from Planner heartbeat status_flags)
	bool autonomy_request;

	// Planner/Orin autonomy hold level (from Planner heartbeat status_flags level).
	// If this drops while target is ENABLE or ACTIVE, Safety retreats target.
	bool autonomy_hold;

	// True only for a short window right after Safety first enters ACTIVE.
	// During this grace period, nodes are allowed to still report ENABLE
	// while they consume the new ACTIVE target and switch their live state.
	bool active_entry_grace;
} system_state_inputs_t;

// ============================================================================
// System State Machine Output
// ============================================================================

typedef struct
{
	// New target state to broadcast
	node_state_t new_target;

	// Whether the target changed (caller should broadcast immediately)
	bool target_changed;
} system_state_result_t;

// ============================================================================
// Pure Decision Function
// ============================================================================

/**
 * @brief Compute the next system target state.
 *
 * Given the current target, stop/fault status, and node states/flags, determine
 * what the new target state should be.
 *
 * Evaluation order (highest priority first):
 *   1) INIT dwell gate:
 *      - While dwell has not elapsed, stay INIT.
 *      - When dwell elapses, transition to READY if clear/alive/ready; else NOT_READY.
 *   2) Problem retreat:
 *      - Any non-INIT target retreats to NOT_READY on a problem
 *        (stop/fault active or node timeout/liveness loss).
 *   3) Autonomy hold drop:
 *      - In ENABLE/ACTIVE, if autonomy_hold drops, retreat to READY when both
 *        nodes are READY, otherwise NOT_READY.
 *   4) Forward path:
 *      - NOT_READY -> READY when both nodes report READY.
 *      - READY -> ENABLE when Planner autonomy request is asserted.
 *      - ENABLE -> ACTIVE when both nodes report ENABLE and both set enable_complete.
 *      - ACTIVE stays ACTIVE only when both nodes report ACTIVE
 *        (ENABLE is accepted during active_entry_grace).
 *
 * @param inputs  All state machine inputs
 * @return New target state and whether it changed
 */
system_state_result_t system_state_step(const system_state_inputs_t *inputs);

#ifdef __cplusplus
}
#endif
