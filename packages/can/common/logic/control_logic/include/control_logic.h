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

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Enable Preconditions
// ============================================================================

typedef struct {
    uint8_t fr_state;           // FR_STATE_* from can_protocol.hh
    bool pedal_pressed;         // true if pedal above threshold
    bool pedal_rearmed;         // true if pedal below threshold for 500ms
    uint8_t fault_code;         // NODE_FAULT_* from can_protocol.hh
} precondition_inputs_t;

/**
 * @brief Check whether autonomous mode enable preconditions are met.
 *
 * @param inputs  Current sensor readings
 * @return true if all preconditions pass (FR forward, no pedal, pedal rearmed, no fault)
 */
bool control_check_preconditions(const precondition_inputs_t *inputs);

/**
 * @brief Check preconditions and return a bitmask of failures.
 *
 * Returns PRECONDITION_OK (0) when all preconditions pass, or a bitmask
 * of PRECONDITION_FAIL_* flags indicating which conditions failed.
 *
 * @param inputs  Current sensor readings
 * @return Bitmask of PRECONDITION_FAIL_* flags (0 = all pass)
 */
uint8_t control_check_preconditions_detailed(const precondition_inputs_t *inputs);

// ============================================================================
// Throttle Slew Rate Limiter
// ============================================================================

typedef struct {
    int8_t current;             // current throttle level (0-7)
    int8_t target;              // target throttle level (0-7)
    uint32_t last_change_ms;    // timestamp of last level change
    uint32_t now_ms;            // current time
    uint32_t slew_interval_ms;  // minimum ms between level changes
} throttle_slew_inputs_t;

typedef struct {
    int8_t new_level;           // output: throttle level after slew
    bool changed;               // true if level was changed this step
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

// Actions the caller must execute after compute_control_step().
// Multiple actions can be OR'd together as a bitmask.
#define CONTROL_ACTION_NONE             0x0000
#define CONTROL_ACTION_START_ENABLE     0x0001  // Begin enable sequence (relay, mux level 0)
#define CONTROL_ACTION_COMPLETE_ENABLE  0x0002  // Finish enable (steppers on, mux autonomous)
#define CONTROL_ACTION_ABORT_ENABLE     0x0004  // Cancel enable (relay off, mux disable)
#define CONTROL_ACTION_TRIGGER_OVERRIDE 0x0008  // Emergency disable all actuators
#define CONTROL_ACTION_ATTEMPT_RECOVERY 0x0010  // Try fault recovery
#define CONTROL_ACTION_APPLY_THROTTLE   0x0020  // Update DG408DJZ mux to new throttle level
#define CONTROL_ACTION_DISABLE_AUTONOMY 0x0040  // Disable actuators (non-override retreat/fault)

// Reasons for CONTROL_ACTION_DISABLE_AUTONOMY.
#define CONTROL_DISABLE_REASON_NONE           0x00
#define CONTROL_DISABLE_REASON_SAFETY_RETREAT 0x01
#define CONTROL_DISABLE_REASON_MOTOR_FAULT    0x02
#define CONTROL_DISABLE_REASON_SENSOR_INVALID 0x03

// Reasons for CONTROL_ACTION_ABORT_ENABLE.
#define CONTROL_ABORT_REASON_NONE             0x00
#define CONTROL_ABORT_REASON_SAFETY_RETREAT   0x01
#define CONTROL_ABORT_REASON_PEDAL_PRESSED    0x02
#define CONTROL_ABORT_REASON_FR_NOT_FORWARD   0x03
#define CONTROL_ABORT_REASON_MOTOR_FAULT      0x04
#define CONTROL_ABORT_REASON_SENSOR_INVALID   0x05

// Enable precondition failure flags (bitmask for diagnostics).
// Indicates which preconditions block autonomy readiness/entry.
#define PRECONDITION_OK                       0x00
#define PRECONDITION_FAIL_FR_NOT_FORWARD      0x01
#define PRECONDITION_FAIL_PEDAL_PRESSED       0x02
#define PRECONDITION_FAIL_PEDAL_NOT_REARMED   0x04
#define PRECONDITION_FAIL_ACTIVE_FAULT        0x08
#define PRECONDITION_FAIL_ALL                 (PRECONDITION_FAIL_FR_NOT_FORWARD | \
                                              PRECONDITION_FAIL_PEDAL_PRESSED | \
                                              PRECONDITION_FAIL_PEDAL_NOT_REARMED | \
                                              PRECONDITION_FAIL_ACTIVE_FAULT)

typedef struct {
    // Safety system command
    uint8_t target_state;               // NODE_STATE_* from Safety's heartbeat

    // Planner commands (valid when ACTIVE)
    int8_t throttle_target;
    int16_t steering_cmd;
    int16_t braking_cmd;
    uint8_t motor_fault_code;           // one-shot from CAN RX (NODE_FAULT_*)

    // Sensor state
    // NOTE: pedal/fr values are expected to be debounced by the caller.
    // A single noisy sample can trigger override or block enable.
    uint8_t fr_state;                   // FR_STATE_*
    bool pedal_pressed;
    bool pedal_rearmed;
    bool fr_is_invalid;                 // true if FR reads as INVALID

    // Stepper position error (computed by caller from motor feedback)
    // NOTE: caller should apply hysteresis/filtering before asserting these.
    bool steering_position_error;       // true if steering position diverged beyond threshold
    bool braking_position_error;        // true if braking position diverged beyond threshold

    // Timing
    uint32_t now_ms;
    uint32_t boot_start_ms;             // boot/startup timestamp
    uint32_t init_dwell_ms;             // minimum INIT dwell before READY
    uint32_t enable_start_ms;           // when enable sequence began
    uint32_t enable_sequence_ms;        // required hold time to complete enable
    bool enable_work_done;              // true after COMPLETE_ENABLE has fired once

    // Throttle slew
    int8_t throttle_current;
    uint32_t last_throttle_change_ms;
    uint32_t throttle_slew_interval_ms;

    // Stepper dedup
    int16_t last_steering_sent;
    int16_t last_braking_sent;

    // Command envelope limits.
    // If min == max == 0, the envelope is treated as unconfigured and the
    // command is forced to neutral (0) for fail-safe behavior.
    int16_t steering_min;           // minimum allowed steering position
    int16_t steering_max;           // maximum allowed steering position
    int16_t braking_min;            // minimum allowed braking position
    int16_t braking_max;            // maximum allowed braking position
} control_inputs_t;

typedef struct {
    // Updated state
    uint8_t new_state;              // NODE_STATE_* from can_protocol.hh
    uint8_t new_fault_code;         // NODE_FAULT_* (0 = none)
    uint8_t override_reason;        // OVERRIDE_REASON_* (set when triggering override)
    uint8_t disable_reason;         // CONTROL_DISABLE_REASON_* (non-override disable)
    uint8_t abort_reason;           // CONTROL_ABORT_REASON_* (set when aborting enable)
    uint8_t precondition_fail;      // PRECONDITION_FAIL_* bitmask (set when READY blocked)

    // Heartbeat flags to send
    uint8_t heartbeat_flags;        // HEARTBEAT_FLAG_* to include in next heartbeat

    // Actions for the caller to execute
    uint16_t actions;               // bitmask of CONTROL_ACTION_*

    // Throttle output
    int8_t throttle_level;          // new throttle level (valid when APPLY_THROTTLE set)
    uint32_t throttle_change_ms;    // updated last_throttle_change_ms

    // Stepper outputs (valid when state is ACTIVE and no override triggered)
    bool send_steering;             // true if steering position changed
    bool send_braking;              // true if braking position changed
    int16_t steering_position;      // position to send
    int16_t braking_position;       // position to send
    int16_t new_last_steering;      // updated dedup tracker
    int16_t new_last_braking;       // updated dedup tracker

    // Recovery tracking
    uint32_t enable_start_ms;       // updated enable start time
    bool enable_work_done;          // true after COMPLETE_ENABLE has fired once
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
control_step_result_t control_compute_step(uint8_t current_state, uint8_t current_fault,
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
int16_t control_clamp_command(int16_t value, int16_t min, int16_t max);

// ============================================================================
// CAN TX Failure Tracking
// ============================================================================

typedef struct {
    uint8_t fail_count;         // current consecutive failure count
    uint8_t threshold;          // trigger recovery at this count
    bool tx_ok;                 // result of last TX
} can_tx_track_inputs_t;

typedef struct {
    uint8_t new_fail_count;     // updated count
    bool trigger_recovery;      // true if threshold reached
} can_tx_track_result_t;

/**
 * @brief Track CAN TX success/failure and determine if recovery is needed.
 *
 * @param inputs  Current TX result and failure tracking state
 * @return Updated count and whether recovery should be triggered
 */
can_tx_track_result_t control_track_can_tx(const can_tx_track_inputs_t *inputs);

#ifdef __cplusplus
}
#endif
