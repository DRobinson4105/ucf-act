/**
 * @file control_logic.h
 * @brief Pure decision logic for the Control ESP32 state machine.
 *
 * This module extracts the core control state machine, throttle slew-rate
 * limiter, and enable-precondition checker as pure functions with no
 * hardware dependencies. This allows comprehensive unit testing on the host.
 *
 * The caller (control_task in main.cpp) reads sensors, calls these functions
 * to obtain decisions, then executes the hardware side effects.
 *
 * Control reacts to the Safety ESP32's system command (target_state) rather
 * than a simple permission boolean. Safety is the system state authority.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "can_protocol.h"
#include "control_domain_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

// Valid throttle level range for the DG408DJZ multiplexer (3-bit, 0-7).
//
// F/R state interpretation (pre-bypass vs post-bypass):
//
// The anti-arcing microswitch (forward_gpio) is in series after the throttle
// microswitch in the cart's 48V circuit.  Before the pedal bypass relay
// (JD-2912) is energized during the ENABLE sequence, the anti-arcing switch
// cannot conduct — so FORWARD and NEUTRAL are indistinguishable (both read
// as NEUTRAL).  The buzzer microswitch (reverse_gpio) is independent and
// always readable.
//
// Pre-bypass (INIT / NOT_READY / READY):
//   - Can only distinguish "buzzer off" (FORWARD or NEUTRAL) from
//     "buzzer on" (REVERSE — reads as FR_STATE_INVALID pre-bypass).
//   - Preconditions require "not in reverse": FORWARD and NEUTRAL both pass.
//   - FR_STATE_INVALID is treated as "reverse" (blocks READY, no fault).
//
// Post-bypass (ENABLE after START_ENABLE / ACTIVE):
//   - Full truth table is readable (bypass relay allows anti-arc current).
//   - FORWARD: normal throttle operation.
//   - NEUTRAL: stay ACTIVE but zero throttle (cart can't drive in neutral).
//   - REVERSE: override (disable all actuators).
//   - INVALID: wiring fault → FAULT (anti-arc should be readable).
#define THROTTLE_LEVEL_MIN 0
#define THROTTLE_LEVEL_MAX 7

// ============================================================================
// Enable Preconditions
// ============================================================================

typedef struct
{
	fr_state_t fr_state;     // FR_STATE_* from control_domain_types.h
	bool pedal_pressed;      // true if pedal above threshold
	bool pedal_rearmed;      // true if pedal below threshold for 500ms
	node_fault_t fault_code; // NODE_FAULT_* from can_protocol.h
} precondition_inputs_t;

/**
 * @brief Check preconditions and return a bitmask of failures.
 *
 * Returns PRECONDITION_OK (0) when all preconditions pass, or a bitmask
 * of PRECONDITION_FAIL_* flags indicating which conditions failed.
 *
 * @param inputs  Current sensor readings
 * @return Bitmask of PRECONDITION_FAIL_* flags (0 = all pass)
 */
precondition_fail_t control_check_preconditions(const precondition_inputs_t *inputs);

// ============================================================================
// Throttle Slew Rate Limiter
// ============================================================================

typedef struct
{
	int8_t current;            // current throttle level (0-7)
	int8_t target;             // target throttle level (0-7)
	uint32_t last_change_ms;   // timestamp of last level change
	uint32_t now_ms;           // current time
	uint32_t slew_interval_ms; // minimum ms between level changes
} throttle_slew_inputs_t;

typedef struct
{
	int8_t new_level; // output: throttle level after slew
	bool changed;     // true if level was changed this step
} throttle_slew_result_t;

/**
 * @brief Compute the next throttle level with slew rate limiting.
 *
 * Moves current toward target by at most 1 step per slew_interval_ms.
 *
 * @param inputs  Throttle state and timing
 * @return New throttle level and whether it changed
 */
throttle_slew_result_t control_compute_throttle_slew(const throttle_slew_inputs_t *inputs);

// ============================================================================
// Control State Machine
// ============================================================================

// --- Input / Output Structs ---

typedef struct
{
	// Safety system command
	node_state_t target_state; // NODE_STATE_* from Safety's heartbeat

	// Planner commands (valid when ACTIVE)
	int8_t throttle_cmd;
	int32_t steering_cmd;
	int16_t braking_cmd;
	node_fault_t motor_fault_code; // one-shot from CAN RX (NODE_FAULT_*)

	// Sensor state
	// NOTE: pedal/fr values are expected to be debounced by the caller.
	// A single noisy sample can trigger override or block enable.
	fr_state_t fr_state; // FR_STATE_*
	bool pedal_pressed;
	bool pedal_rearmed;

	// Stepper position error (computed by caller from motor feedback)
	// NOTE: caller should apply hysteresis/filtering before asserting these.
	bool steering_position_error; // true if steering position diverged beyond threshold
	bool braking_position_error;  // true if braking position diverged beyond threshold

	// Timing
	uint32_t now_ms;
	uint32_t boot_start_ms;      // boot/startup timestamp
	uint32_t init_dwell_ms;      // minimum INIT dwell before READY
	uint32_t enable_start_ms;    // when enable sequence began
	uint32_t enable_sequence_ms; // required hold time to complete enable
	bool enable_work_done;       // true after COMPLETE_ENABLE has fired once

	// Throttle slew
	int8_t throttle_current;
	uint32_t last_throttle_change_ms;
	uint32_t throttle_slew_interval_ms;

	// Stepper dedup
	int32_t last_steering_sent;
	int16_t last_braking_sent;

	// Command envelope limits.
	// If min == max == 0, the envelope is treated as unconfigured and the
	// command is forced to neutral (0) for fail-safe behavior.
	int32_t steering_min; // minimum allowed steering position
	int32_t steering_max; // maximum allowed steering position
	int16_t braking_min;  // minimum allowed braking position
	int16_t braking_max;  // maximum allowed braking position
} control_inputs_t;

typedef struct
{
	// Updated state
	node_state_t new_state;                // NODE_STATE_* from can_protocol.h
	node_fault_t new_fault_code;           // NODE_FAULT_* (0 = none)
	override_reason_t override_reason;     // OVERRIDE_REASON_* (set when triggering override)
	disable_reason_t disable_reason;       // CONTROL_DISABLE_REASON_* (non-override disable)
	abort_reason_t abort_reason;           // CONTROL_ABORT_REASON_* (set when aborting enable)
	precondition_fail_t precondition_fail; // PRECONDITION_FAIL_* bitmask (set when READY blocked)

	// Heartbeat flags to send
	heartbeat_flags_t heartbeat_flags; // HEARTBEAT_FLAG_* to include in next heartbeat

	// Actions for the caller to execute
	control_actions_t actions; // bitmask of CONTROL_ACTION_*

	// Throttle output
	int8_t throttle_level;       // new throttle level (valid when APPLY_THROTTLE set)
	uint32_t throttle_change_ms; // updated last_throttle_change_ms

	// Stepper outputs (valid when state is ACTIVE and no override triggered)
	bool send_steering;        // true if steering position changed
	bool send_braking;         // true if braking position changed
	int32_t steering_position; // position to send
	int16_t braking_position;  // position to send
	int32_t new_last_steering; // updated dedup tracker
	int16_t new_last_braking;  // updated dedup tracker

	// Recovery tracking
	uint32_t enable_start_ms; // updated enable start time
	bool enable_work_done;    // true after COMPLETE_ENABLE has fired once
} control_step_result_t;

/**
 * @brief Compute one step of the control state machine.
 *
 * Pure function: reads current state + inputs, produces next state + action list.
 * The caller executes the hardware side effects indicated by the actions bitmask.
 *
 * Control reacts to Safety's target_state (NOT_READY/READY/ENABLE/ACTIVE).
 * Any other target value is treated as NOT_READY for a safe retreat.
 *
 * Live-state behavior:
 *   - INIT -> NOT_READY <-> READY -> ENABLE -> ACTIVE
 *   - ACTIVE -> OVERRIDE -> NOT_READY/READY
 *   - Any live state can enter FAULT
 *   - FAULT returns to NOT_READY/READY when the active fault condition clears
 *
 * @param current_state  Current NODE_STATE_* value
 * @param current_fault  Current fault code
 * @param inputs         All sensor/CAN/timing inputs
 * @return Step result with new state and actions to execute
 */
control_step_result_t control_compute_step(node_state_t current_state, node_fault_t current_fault,
                                           const control_inputs_t *inputs);

// ============================================================================
// Command Envelope Clamping
// ============================================================================

/**
 * @brief Clamp a signed 16-bit command value to [min, max].
 *
 * Used to enforce safe steering and braking position envelopes before
 * forwarding planner commands to stepper motors.
 *
 * @param value  Raw command value from planner
 * @param min    Minimum allowed value (inclusive)
 * @param max    Maximum allowed value (inclusive)
 * @return Clamped value
 */
int32_t control_clamp_command(int32_t value, int32_t min, int32_t max);

#ifdef __cplusplus
}
#endif
