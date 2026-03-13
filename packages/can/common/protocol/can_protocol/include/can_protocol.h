/**
 * @file can_protocol.h
 * @brief Shared CAN bus protocol definitions for Safety, Control, and Planner nodes.
 *
 * All nodes share a unified state and fault code enum. Safety ESP32 acts as the
 * system state authority — it commands target state using NOT_READY/READY/ENABLE/ACTIVE.
 * Planner and Control own their live states and can enter OVERRIDE or FAULT locally
 * for immediate safety; local FAULT clears back to NOT_READY/READY when faults clear.
 *
 * Every node sends an identical heartbeat format (node_heartbeat_t) at 100ms.
 * Safety's "state" field carries the system target state; its "fault_code" field
 * carries the e-stop reason using the NODE_FAULT_ESTOP_* range.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// Semantic Typedefs
// ============================================================================
// Lightweight aliases for uint8_t that document the domain meaning of each
// byte-sized field.  They are NOT distinct types (no enum class) — they
// remain freely interchangeable with uint8_t so existing C code compiles
// without casts.

typedef uint8_t node_state_t;      // NODE_STATE_* values
typedef uint8_t node_fault_t;      // NODE_FAULT_* values (scalar or estop bitmask)
typedef uint8_t heartbeat_flags_t; // HEARTBEAT_FLAG_* bitmask
typedef uint8_t heartbeat_seq_t;   // Rolling 0-255 heartbeat sequence counter
// ============================================================================
// CAN Message ID Allocation
// ============================================================================

// Safety ESP32 (0x100-0x10F)
#define CAN_ID_SAFETY_HEARTBEAT      0x100 // Safety heartbeat (target_state, estop fault_code)
#define CAN_ID_SAFETY_BATTERY_STATUS 0x101 // Battery voltage, current, SOC (1 Hz)

// Planner (0x110-0x11F)
#define CAN_ID_PLANNER_HEARTBEAT 0x110 // Planner heartbeat (seq, state, fault_code, flags)
#define CAN_ID_PLANNER_COMMAND   0x111 // Throttle/steering/braking commands

// Control ESP32 (0x120-0x12F)
#define CAN_ID_CONTROL_HEARTBEAT 0x120 // Control heartbeat (seq, state, fault_code, flags)

// ============================================================================
// Timing Constants
// ============================================================================

#define HEARTBEAT_INTERVAL_MS 100
#define HEARTBEAT_TIMEOUT_MS  500

// ============================================================================
// Unified Node States (NODE_STATE_*)
// ============================================================================
// All nodes share the same state enum.
// Safety heartbeat uses NOT_READY/READY/ENABLE/ACTIVE as commanded target states.
// Nodes report live state and may use OVERRIDE/FAULT locally for immediate safety.

#define NODE_STATE_INIT      0 // Booting, not ready
#define NODE_STATE_NOT_READY 1 // Running/manual-safe, but autonomy preconditions not satisfied
#define NODE_STATE_READY     2 // Running/manual-safe and autonomy preconditions satisfied
#define NODE_STATE_ENABLE    3 // Preparing for autonomous (hardware settle / planning init)
#define NODE_STATE_ACTIVE    4 // Autonomous mode — processing/sending commands
#define NODE_STATE_OVERRIDE  5 // Driver took over (local, immediate)
#define NODE_STATE_FAULT     6 // Fault condition (local, immediate)

// ============================================================================
// Unified Fault Codes (NODE_FAULT_*)
// ============================================================================
// Two encoding modes in the same byte:
//   0x00:       No fault
//   0x01-0x7F:  E-stop BITMASK (Safety heartbeat only) — multiple bits can be set
//   0x80-0x8F:  Planner faults (scalar, one at a time)
//   0x90-0x9F:  Control faults (scalar, one at a time)
//   0xFF:       General / unspecified
//
// Safety's fault_code is a bitmask: if button AND remote are both active,
// fault_code = 0x01 | 0x02 = 0x03. Use node_estop_to_string() to decode.
// Planner/Control fault_code is a single scalar value.

// Common
#define NODE_FAULT_NONE    0x00
#define NODE_FAULT_GENERAL 0xFF // Unspecified fault

// E-stop bitmask flags (0x01-0x40, used only by Safety heartbeat)
// Multiple bits can be set simultaneously when multiple e-stops are active.
#define NODE_FAULT_ESTOP_BUTTON          0x01 // bit 0: push_button_hb2es544 pressed
#define NODE_FAULT_ESTOP_REMOTE          0x02 // bit 1: rf_remote_ev1527 kill active
#define NODE_FAULT_ESTOP_ULTRASONIC      0x04 // bit 2: ultrasonic_a02yyuw obstacle or unhealthy
#define NODE_FAULT_ESTOP_PLANNER         0x08 // bit 3: Planner reported FAULT state
#define NODE_FAULT_ESTOP_PLANNER_TIMEOUT 0x10 // bit 4: Planner heartbeat timeout
#define NODE_FAULT_ESTOP_CONTROL         0x20 // bit 5: Control ESP32 reported FAULT state
#define NODE_FAULT_ESTOP_CONTROL_TIMEOUT 0x40 // bit 6: Control ESP32 heartbeat timeout

// Mask for any estop bit set
#define NODE_FAULT_ESTOP_ANY 0x7F

// Planner faults (0x80-0x8F, scalar)
#define NODE_FAULT_PERCEPTION       0x80 // Camera/LiDAR failure
#define NODE_FAULT_LOCALIZATION     0x81 // Localization lost
#define NODE_FAULT_PLANNING         0x82 // Path planner failure
#define NODE_FAULT_PLANNER_HARDWARE 0x83 // Planner hardware issue (thermal, etc.)

// Control faults (0x90-0x9F, scalar)
#define NODE_FAULT_THROTTLE_INIT  0x90 // Throttle mux init failed
#define NODE_FAULT_CAN_TX         0x91 // CAN transmit failures
#define NODE_FAULT_MOTOR_COMM     0x92 // Stepper communication lost
#define NODE_FAULT_SENSOR_INVALID 0x93 // F/R sensor invalid reading
#define NODE_FAULT_RELAY_INIT     0x94 // Relay initialization failed

// ============================================================================
// Battery Status Flags (CAN_ID_SAFETY_BATTERY_STATUS byte 5)
// ============================================================================

#define BATTERY_FLAG_CHARGING     0x01 // Current flowing into battery (charger/regen)
#define BATTERY_FLAG_LOW_WARNING  0x02 // SOC <= 20%
#define BATTERY_FLAG_CRITICAL     0x04 // SOC <= 10%
#define BATTERY_FLAG_SENSOR_FAULT 0x08 // ADC read failure

// ============================================================================
// Heartbeat Flags
// ============================================================================

#define HEARTBEAT_FLAG_ENABLE_COMPLETE  0x01 // Node finished ENABLE, ready for ACTIVE
#define HEARTBEAT_FLAG_AUTONOMY_REQUEST 0x02 // Planner requests/holds autonomy (enable gate + halt on drop)
// 0x04 reserved

// ============================================================================
// Node Heartbeat (0x100, 0x110, 0x120)
// ============================================================================
// Sent by ALL nodes periodically (100ms) and immediately on state change.
// byte 0: sequence counter (0-255, wraps)
// byte 1: state (NODE_STATE_* — for Safety: system target state)
// byte 2: fault_code (NODE_FAULT_* — for Safety: e-stop reason, 0 when advancing)
// byte 3: flags (HEARTBEAT_FLAG_*)
//         - HEARTBEAT_FLAG_ENABLE_COMPLETE: node finished ENABLE
//         - HEARTBEAT_FLAG_AUTONOMY_REQUEST: Planner/Orin requests autonomy enable and
//           must stay asserted while autonomy should remain enabled
// byte 4-7: reserved

typedef struct
{
	heartbeat_seq_t sequence;
	node_state_t state;
	node_fault_t fault_code;
	heartbeat_flags_t flags;
} node_heartbeat_t;

// ============================================================================
// Planner Command (0x111)
// ============================================================================
// Sent by Planner when in ACTIVE state.
// byte 0: sequence counter (0-255, wraps)
// byte 1: throttle level (0-7)
// byte 2-3: steering position (int16, encoder counts, little-endian)
// byte 4-5: braking position (int16, encoder counts, little-endian)
// byte 6-7: reserved
// NOTE: This struct is for logical representation only — not packed to CAN frame layout.
// Encoding/decoding uses explicit byte-level access via can_encode/decode_planner_command().

typedef struct
{
	heartbeat_seq_t sequence;
	uint8_t throttle;
	int16_t steering_position;
	int16_t braking_position;
} planner_command_t;

// ============================================================================
// Helper Functions — Pack/Unpack
// ============================================================================

/**
 * @brief Pack a uint16_t value as little-endian into a byte buffer.
 *
 * Stores the low byte at buf[0] and the high byte at buf[1].
 *
 * @param buf    Destination buffer (must have room for at least 2 bytes)
 * @param value  The 16-bit unsigned value to pack
 */
static inline void can_pack_le16(uint8_t *buf, uint16_t value)
{
	buf[0] = (uint8_t)(value & 0xFF);
	buf[1] = (uint8_t)((value >> 8) & 0xFF);
}

/**
 * @brief Unpack a little-endian uint16_t from a byte buffer.
 *
 * Reads buf[0] as the low byte and buf[1] as the high byte.
 *
 * @param buf  Source buffer (must contain at least 2 bytes)
 * @return The reconstructed 16-bit unsigned value
 */
static inline uint16_t can_unpack_le16(const uint8_t *buf)
{
	return (uint16_t)(buf[0] | (buf[1] << 8));
}

/**
 * @brief Pack an int16_t value as little-endian into a byte buffer.
 *
 * Signed wrapper around can_pack_le16(). The value is reinterpreted as
 * uint16_t for storage.
 *
 * @param buf    Destination buffer (must have room for at least 2 bytes)
 * @param value  The 16-bit signed value to pack
 */
static inline void can_pack_le16s(uint8_t *buf, int16_t value)
{
	can_pack_le16(buf, (uint16_t)value);
}

/**
 * @brief Unpack a little-endian int16_t from a byte buffer.
 *
 * Signed wrapper around can_unpack_le16(). The raw uint16_t is
 * reinterpreted as int16_t.
 *
 * @param buf  Source buffer (must contain at least 2 bytes)
 * @return The reconstructed 16-bit signed value
 */
static inline int16_t can_unpack_le16s(const uint8_t *buf)
{
	return (int16_t)can_unpack_le16(buf);
}

// ============================================================================
// Encode/Decode — Node Heartbeat (shared by ALL nodes: Safety, Planner, Control)
// ============================================================================

/**
 * @brief Encode a heartbeat struct into an 8-byte CAN data frame.
 *
 * Writes sequence, state, fault_code, and flags into bytes 0-3 and
 * zeroes the reserved bytes 4-7. Silently returns if either pointer
 * is NULL.
 *
 * @param data  Destination buffer (must have room for 8 bytes)
 * @param hb    Heartbeat struct to encode
 */
static inline void can_encode_heartbeat(uint8_t *data, const node_heartbeat_t *hb)
{
	if (!data || !hb)
		return;
	data[0] = hb->sequence;
	data[1] = hb->state;
	data[2] = hb->fault_code;
	data[3] = hb->flags;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
}

/**
 * @brief Decode an 8-byte CAN data frame into a heartbeat struct.
 *
 * Reads sequence, state, fault_code, and flags from bytes 0-3.
 * Returns false if any pointer is NULL or the DLC is not exactly 8.
 *
 * @param data  Source CAN data buffer (8 bytes)
 * @param dlc   Data length code (must be 8)
 * @param hb    Destination heartbeat struct
 * @return true on success, false on invalid arguments
 */
static inline bool can_decode_heartbeat(const uint8_t *data, uint8_t dlc, node_heartbeat_t *hb)
{
	if (!data || !hb || dlc != 8)
		return false;
	hb->sequence = data[0];
	hb->state = data[1];
	hb->fault_code = data[2];
	hb->flags = data[3];
	return true;
}

// ============================================================================
// Encode/Decode — Planner Command
// ============================================================================

/**
 * @brief Encode a planner command struct into an 8-byte CAN data frame.
 *
 * Writes sequence (byte 0), throttle (byte 1), steering position
 * (bytes 2-3, little-endian), braking position (bytes 4-5, little-endian),
 * and zeroes the reserved bytes 6-7. Silently returns if either pointer
 * is NULL.
 *
 * @param data  Destination buffer (must have room for 8 bytes)
 * @param cmd   Planner command struct to encode
 */
static inline void can_encode_planner_command(uint8_t *data, const planner_command_t *cmd)
{
	if (!data || !cmd)
		return;
	data[0] = cmd->sequence;
	data[1] = cmd->throttle;
	can_pack_le16s(&data[2], cmd->steering_position);
	can_pack_le16s(&data[4], cmd->braking_position);
	data[6] = 0; // reserved
	data[7] = 0; // reserved
}

/**
 * @brief Decode CAN data into a planner command struct.
 *
 * Reads sequence (byte 0), throttle (byte 1, masked to 3 bits),
 * steering position (bytes 2-3), and braking position (bytes 4-5).
 * Returns false if any pointer is NULL or the DLC is less than 6.
 *
 * @param data  Source CAN data buffer (at least 6 bytes)
 * @param dlc   Data length code (must be >= 6)
 * @param cmd   Destination planner command struct
 * @return true on success, false on invalid arguments
 */
static inline bool can_decode_planner_command(const uint8_t *data, uint8_t dlc, planner_command_t *cmd)
{
	if (!data || !cmd || dlc < 6)
		return false;
	cmd->sequence = data[0];
	cmd->throttle = (uint8_t)(data[1] & 0x07); // 3-bit field, max = 7
	cmd->steering_position = can_unpack_le16s(&data[2]);
	cmd->braking_position = can_unpack_le16s(&data[4]);
	return true;
}

// ============================================================================
// Battery Status (0x101)
// ============================================================================
// Sent by Safety ESP32 at 1 Hz.
// byte 0-1: pack voltage in mV (uint16 LE, 0-65535 → 0-65.5V)
// byte 2-3: pack current in 10mA units (int16 LE, ±327.67A)
//           positive = discharge, negative = charge
// byte 4:   SOC percentage (uint8, 0-100)
// byte 5:   flags (BATTERY_FLAG_*)
// byte 6-7: reserved

typedef struct
{
	uint16_t voltage_mv;   // Pack voltage in millivolts
	int16_t current_10ma;  // Pack current in 10mA units (positive=discharge)
	uint8_t soc_pct;       // State of charge 0-100%
	uint8_t flags;         // BATTERY_FLAG_* bitmask
} battery_status_t;

/**
 * @brief Encode a battery status struct into an 8-byte CAN data frame.
 *
 * Writes voltage (bytes 0-1, LE), current (bytes 2-3, LE), SOC (byte 4),
 * flags (byte 5), and zeroes the reserved bytes 6-7.
 *
 * @param data  Destination buffer (must have room for 8 bytes)
 * @param bat   Battery status struct to encode
 */
static inline void can_encode_battery_status(uint8_t *data, const battery_status_t *bat)
{
	if (!data || !bat)
		return;
	can_pack_le16(&data[0], bat->voltage_mv);
	can_pack_le16s(&data[2], bat->current_10ma);
	data[4] = bat->soc_pct;
	data[5] = bat->flags;
	data[6] = 0;
	data[7] = 0;
}

/**
 * @brief Decode an 8-byte CAN data frame into a battery status struct.
 *
 * Reads voltage (bytes 0-1), current (bytes 2-3), SOC (byte 4),
 * and flags (byte 5). Returns false if any pointer is NULL or
 * the DLC is less than 6.
 *
 * @param data  Source CAN data buffer (at least 6 bytes)
 * @param dlc   Data length code (must be >= 6)
 * @param bat   Destination battery status struct
 * @return true on success, false on invalid arguments
 */
static inline bool can_decode_battery_status(const uint8_t *data, uint8_t dlc, battery_status_t *bat)
{
	if (!data || !bat || dlc < 6)
		return false;
	bat->voltage_mv = can_unpack_le16(&data[0]);
	bat->current_10ma = can_unpack_le16s(&data[2]);
	bat->soc_pct = data[4];
	bat->flags = data[5];
	return true;
}

// ============================================================================
// Debug String Helpers
// ============================================================================

/**
 * @brief Convert a node state enum value to a human-readable string.
 *
 * Returns a static string literal for each NODE_STATE_* value,
 * or "UNKNOWN" for unrecognized values.
 *
 * @param state  Node state value (NODE_STATE_*)
 * @return Pointer to a static string describing the state
 */
static inline const char *node_state_to_string(node_state_t state)
{
	switch (state)
	{
	case NODE_STATE_INIT:
		return "INIT";
	case NODE_STATE_NOT_READY:
		return "NOT_READY";
	case NODE_STATE_READY:
		return "READY";
	case NODE_STATE_ENABLE:
		return "ENABLE";
	case NODE_STATE_ACTIVE:
		return "ACTIVE";
	case NODE_STATE_OVERRIDE:
		return "OVERRIDE";
	case NODE_STATE_FAULT:
		return "FAULT";
	default:
		return "UNKNOWN";
	}
}

/**
 * @brief Decode an e-stop bitmask into a human-readable string (re-entrant).
 *
 * Writes into a caller-provided buffer, making this variant safe for use
 * from multiple threads or ISRs. Multiple active e-stop sources are
 * separated by '+'. Returns "none" when no bits are set.
 *
 * Example: fault=0x05 produces "button+ultrasonic".
 *
 * @param fault    E-stop bitmask (NODE_FAULT_ESTOP_* flags)
 * @param buf      Destination buffer for the result string
 * @param buf_len  Size of @p buf in bytes
 * @return Pointer to @p buf containing the null-terminated result
 */
static inline const char *node_estop_to_string_r(node_fault_t fault, char *buf, size_t buf_len)
{
	if (!buf || buf_len == 0)
		return "";
	if (fault == NODE_FAULT_NONE)
	{
		snprintf(buf, buf_len, "none");
		return buf;
	}

	char *p = buf;
	char *end = buf + buf_len;

#define ESTOP_APPEND(flag, name)         \
	do                                   \
	{                                    \
		if ((fault & (flag)) && p < end) \
		{                                \
			if (p != buf && p < end - 1) \
				*p++ = '+';              \
			const char *s = name;        \
			while (*s && p < end - 1)    \
				*p++ = *s++;             \
		}                                \
	} while (0)

	ESTOP_APPEND(NODE_FAULT_ESTOP_BUTTON, "button");
	ESTOP_APPEND(NODE_FAULT_ESTOP_REMOTE, "remote");
	ESTOP_APPEND(NODE_FAULT_ESTOP_ULTRASONIC, "ultrasonic");
	ESTOP_APPEND(NODE_FAULT_ESTOP_PLANNER, "planner");
	ESTOP_APPEND(NODE_FAULT_ESTOP_PLANNER_TIMEOUT, "planner_timeout");
	ESTOP_APPEND(NODE_FAULT_ESTOP_CONTROL, "control");
	ESTOP_APPEND(NODE_FAULT_ESTOP_CONTROL_TIMEOUT, "control_timeout");

#undef ESTOP_APPEND

	if (p >= end)
		buf[buf_len - 1] = '\0';
	else
		*p = '\0';
	return buf;
}

/**
 * @brief Decode an e-stop bitmask into a human-readable string.
 *
 * Convenience wrapper around node_estop_to_string_r() that uses a
 * thread-local buffer, so independent tasks/threads don't clobber
 * each other. Not safe to call from ISRs; use the _r variant instead.
 *
 * @param fault  E-stop bitmask (NODE_FAULT_ESTOP_* flags)
 * @return Pointer to a thread-local string describing the active e-stops
 */
static inline const char *node_estop_to_string(node_fault_t fault)
{
	static thread_local char buf[80];
	return node_estop_to_string_r(fault, buf, sizeof(buf));
}

/**
 * @brief Decode a fault code into a human-readable string.
 *
 * For e-stop bitmask values (0x01-0x7F), delegates to
 * node_estop_to_string(). For scalar planner/control faults (0x80+),
 * returns a fixed string literal. Uses a thread-local buffer for the
 * e-stop case; use node_fault_to_string_r() from ISRs.
 *
 * @param fault  Fault code (NODE_FAULT_*)
 * @return Pointer to a string describing the fault
 */
static inline const char *node_fault_to_string(node_fault_t fault)
{
	// Estop bitmask range (0x01-0x7F)
	if (fault != NODE_FAULT_NONE && fault <= NODE_FAULT_ESTOP_ANY)
		return node_estop_to_string(fault);

	switch (fault)
	{
	case NODE_FAULT_NONE:
		return "none";
	// Planner faults
	case NODE_FAULT_PERCEPTION:
		return "perception";
	case NODE_FAULT_LOCALIZATION:
		return "localization";
	case NODE_FAULT_PLANNING:
		return "planning";
	case NODE_FAULT_PLANNER_HARDWARE:
		return "planner_hardware";
	// Control faults
	case NODE_FAULT_THROTTLE_INIT:
		return "throttle_init";
	case NODE_FAULT_CAN_TX:
		return "can_tx";
	case NODE_FAULT_MOTOR_COMM:
		return "motor_comm";
	case NODE_FAULT_SENSOR_INVALID:
		return "sensor_invalid";
	case NODE_FAULT_RELAY_INIT:
		return "relay_init";
	// General
	case NODE_FAULT_GENERAL:
		return "general";
	default:
		return "unknown";
	}
}

/**
 * @brief Decode a fault code into a human-readable string (re-entrant).
 *
 * Re-entrant/thread-safe variant of node_fault_to_string(). For e-stop
 * bitmask values (0x01-0x7F), writes the decoded string into the
 * caller-provided buffer via node_estop_to_string_r(). For scalar
 * faults, returns a static string literal directly.
 *
 * @param fault    Fault code (NODE_FAULT_*)
 * @param buf      Destination buffer (used only for e-stop bitmask decoding)
 * @param buf_len  Size of @p buf in bytes
 * @return Pointer to a string describing the fault
 */
static inline const char *node_fault_to_string_r(node_fault_t fault, char *buf, size_t buf_len)
{
	if (fault != NODE_FAULT_NONE && fault <= NODE_FAULT_ESTOP_ANY)
		return node_estop_to_string_r(fault, buf, buf_len);
	return node_fault_to_string(fault);
}

#ifdef __cplusplus
}
#endif
