/**
 * @file system_state.h
 * @brief Pure decision logic for Safety ESP32 system state machine.
 *
 * The Safety ESP32 is the system target-state authority — it advances only
 * READY -> ENABLING -> ACTIVE. Any node can report OVERRIDE or FAULT as a
 * local live state for immediate safety; Safety reacts by pulling target back
 * to READY.
 *
 * This module is a pure function library with no hardware dependencies,
 * allowing comprehensive unit testing on the host.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// System State Machine Inputs
// ============================================================================

typedef struct {
    // Current target state (what Safety last commanded)
    uint8_t current_target;

    // Timing for INIT -> READY dwell
    uint32_t now_ms;
    uint32_t boot_start_ms;
    uint32_t init_dwell_ms;

    // E-stop evaluation result (from safety_logic)
    bool estop_active;

    // Node actual states (from heartbeats)
    uint8_t planner_state;
    uint8_t control_state;

    // Heartbeat liveness
    bool planner_alive;
    bool control_alive;

    // Enable completion flags (from heartbeat flags field)
    bool planner_enable_complete;
    bool control_enable_complete;

    // Planner/Orin autonomy-enable request gate (from Planner heartbeat flags)
    bool autonomy_request;

    // Planner/Orin autonomy hold level (from Planner heartbeat flags level).
    // If this drops while target is ENABLING or ACTIVE, Safety retreats target to READY.
    bool autonomy_hold;
} system_state_inputs_t;

// ============================================================================
// System State Machine Output
// ============================================================================

typedef struct {
    // New target state to broadcast
    uint8_t new_target;

    // Whether the target changed (caller should broadcast immediately)
    bool target_changed;
} system_state_result_t;

// ============================================================================
// Pure Decision Function
// ============================================================================

/**
 * @brief Compute the next system target state.
 *
 * Given the current target, e-stop status, and node states/flags, determine
 * what the new target state should be.
 *
 * Transitions:
 *   - READY -> ENABLING: both nodes READY, no e-stop, both alive,
 *                        Planner autonomy request asserted
 *   - ENABLING -> ACTIVE: both nodes ENABLING, both enable_complete, no e-stop
 *   - ENABLING/ACTIVE -> READY: Planner autonomy hold dropped (halt command)
 *   - ANY target -> READY: e-stop active, node fault, node override, node timeout
 *   - INIT -> READY: after init_dwell_ms has elapsed since boot_start_ms
 *
 * @param inputs  All state machine inputs
 * @return New target state and whether it changed
 */
system_state_result_t system_state_step(const system_state_inputs_t *inputs);

#ifdef __cplusplus
}
#endif
