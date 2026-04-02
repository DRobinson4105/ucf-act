/**
 * @file control_types.h
 * @brief Shared Control semantic types and constants.
 *
 * These values are not CAN wire-format definitions. They model local
 * Control-domain behavior (driver input decoding, state-machine actions,
 * and diagnostics) used by both pure logic and hardware-facing modules.
 */
#pragma once

#include <stdint.h>
#include <limits.h>

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// Driver Input Domain (F/R switch)
// ============================================================================

typedef uint8_t fr_state_t; // FR_STATE_* values

#define FR_STATE_NEUTRAL 0x00
#define FR_STATE_FORWARD 0x01
#define FR_STATE_REVERSE 0x02
#define FR_STATE_INVALID 0x03

// ============================================================================
// Control Policy Constants
// ============================================================================

// Sentinel value for stepper dedup trackers — forces next command to send.
// Both steering and braking trackers use int32_t so full actuator travel is preserved.
#define STEPPER_DEDUP_RESET_STEERING INT32_MIN
#define STEPPER_DEDUP_RESET_BRAKING  INT32_MIN

// ============================================================================
// Override / Preconditions / Actions
// ============================================================================

#define OVERRIDE_REASON_NONE     0x00
#define OVERRIDE_REASON_REVERSE  0x01
#define OVERRIDE_REASON_THROTTLE 0x02
#define OVERRIDE_REASON_STEERING 0x03
#define OVERRIDE_REASON_BRAKING  0x04
typedef uint8_t override_reason_t; // OVERRIDE_REASON_* values

#define PRECONDITION_OK                     0x00
#define PRECONDITION_FAIL_FR_IN_REVERSE     0x01
#define PRECONDITION_FAIL_PEDAL_PRESSED     0x02
#define PRECONDITION_FAIL_PEDAL_NOT_REARMED 0x04
#define PRECONDITION_FAIL_ACTIVE_FAULT      0x08
#define PRECONDITION_FAIL_STOP_ACTIVE       0x10
#define PRECONDITION_FAIL_ALL                                                                                  \
	(PRECONDITION_FAIL_FR_IN_REVERSE | PRECONDITION_FAIL_PEDAL_PRESSED | PRECONDITION_FAIL_PEDAL_NOT_REARMED | \
	 PRECONDITION_FAIL_ACTIVE_FAULT | PRECONDITION_FAIL_STOP_ACTIVE)
typedef uint8_t precondition_fail_t; // PRECONDITION_FAIL_* bitmask

#define CONTROL_ACTION_NONE             0x0000
#define CONTROL_ACTION_START_ENABLE     0x0001 // Begin enable sequence (relay, throttle level 0)
#define CONTROL_ACTION_COMPLETE_ENABLE  0x0002 // Finish enable (steppers on, throttle autonomous)
#define CONTROL_ACTION_ABORT_ENABLE     0x0004 // Cancel enable (relay off, throttle disable)
#define CONTROL_ACTION_TRIGGER_OVERRIDE 0x0008 // Emergency disable all actuators
// 0x0010 reserved
#define CONTROL_ACTION_APPLY_THROTTLE   0x0020 // Update throttle actuator to new level
#define CONTROL_ACTION_DISABLE_AUTONOMY 0x0040 // Disable actuators (non-override retreat/fault)
typedef uint16_t control_actions_t;            // CONTROL_ACTION_* bitmask

#define CONTROL_DISABLE_REASON_NONE           0x00
#define CONTROL_DISABLE_REASON_SAFETY_RETREAT 0x01
#define CONTROL_DISABLE_REASON_MOTOR_FAULT    0x02
#define CONTROL_DISABLE_REASON_SENSOR_INVALID 0x03
#define CONTROL_DISABLE_REASON_INTERNAL       0x04 // corrupted state or unexpected condition
typedef uint8_t disable_reason_t;                  // CONTROL_DISABLE_REASON_* values

#define CONTROL_ABORT_REASON_NONE           0x00
#define CONTROL_ABORT_REASON_SAFETY_RETREAT 0x01
#define CONTROL_ABORT_REASON_PEDAL_PRESSED  0x02
#define CONTROL_ABORT_REASON_FR_IN_REVERSE  0x03
#define CONTROL_ABORT_REASON_MOTOR_FAULT    0x04
#define CONTROL_ABORT_REASON_SENSOR_INVALID 0x05
typedef uint8_t abort_reason_t; // CONTROL_ABORT_REASON_* values

#ifdef __cplusplus
}
#endif
