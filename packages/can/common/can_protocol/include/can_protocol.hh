/**
 * @file can_protocol.hh
 * @brief Shared CAN bus protocol definitions for Safety, Control, and Planner nodes.
 *
 * All nodes share a unified state and fault code enum. Safety ESP32 acts as the
 * system state authority — it is the only node that can advance state forward
 * (READY -> ENABLING -> ACTIVE). Any node can enter OVERRIDE or FAULT locally
 * for immediate safety.
 *
 * Every node sends an identical heartbeat format (node_heartbeat_t) at 100ms.
 * Safety's "state" field carries the system target state; its "fault_code" field
 * carries the e-stop reason using the NODE_FAULT_ESTOP_* range.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// CAN Message ID Allocation
// ============================================================================

// Safety ESP32 (0x100-0x10F)
#define CAN_ID_SAFETY_HEARTBEAT      0x100   // Safety heartbeat (target_state, estop fault_code)

// Planner (0x110-0x11F)
#define CAN_ID_PLANNER_HEARTBEAT     0x110   // Planner heartbeat (seq, state, fault_code, flags)
#define CAN_ID_PLANNER_COMMAND       0x111   // Throttle/steering/braking commands

// Control ESP32 (0x120-0x12F)
#define CAN_ID_CONTROL_HEARTBEAT     0x120   // Control heartbeat (seq, state, fault_code, flags)

// UIM2852CA Motors - use CAN 2.0B extended 29-bit frames with SimpleCAN protocol
// See stepper_protocol_uim2852.h for CAN ID calculation and protocol details
#define UIM2852_MASTER_ID            4   // Producer ID for host controller
#define UIM2852_NODE_STEERING        5   // Steering motor node ID
#define UIM2852_NODE_BRAKING         6   // Braking motor node ID
#define UIM2852_GLOBAL_ID            0   // Broadcast to all motors

// ============================================================================
// Timing Constants
// ============================================================================

#define HEARTBEAT_INTERVAL_MS        100
#define HEARTBEAT_TIMEOUT_MS         500

// ============================================================================
// Unified Node States (NODE_STATE_*)
// ============================================================================
// All nodes share the same state enum. Safety commands forward transitions.
// Nodes can enter OVERRIDE or FAULT locally for immediate safety.
// For Safety's heartbeat, 'state' carries the system target state.

#define NODE_STATE_INIT              0   // Booting, not ready
#define NODE_STATE_READY             1   // Running, idle — waiting for Safety to advance
#define NODE_STATE_ENABLING          2   // Preparing for autonomous (hardware settle / planning init)
#define NODE_STATE_ACTIVE            3   // Autonomous mode — processing/sending commands
#define NODE_STATE_OVERRIDE          4   // Driver took over (local, immediate)
#define NODE_STATE_FAULT             5   // Fault condition (local, immediate)

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
#define NODE_FAULT_NONE              0x00
#define NODE_FAULT_GENERAL           0xFF   // Unspecified fault

// E-stop bitmask flags (0x01-0x40, used only by Safety heartbeat)
// Multiple bits can be set simultaneously when multiple e-stops are active.
#define NODE_FAULT_ESTOP_BUTTON          0x01   // bit 0: push_button_hb2es544 pressed
#define NODE_FAULT_ESTOP_REMOTE          0x02   // bit 1: rf_remote_ev1527 kill active
#define NODE_FAULT_ESTOP_ULTRASONIC      0x04   // bit 2: ultrasonic_a02yyuw obstacle or unhealthy
#define NODE_FAULT_ESTOP_PLANNER         0x08   // bit 3: Planner reported FAULT state
#define NODE_FAULT_ESTOP_PLANNER_TIMEOUT 0x10   // bit 4: Planner heartbeat timeout
#define NODE_FAULT_ESTOP_CONTROL         0x20   // bit 5: Control ESP32 reported FAULT state
#define NODE_FAULT_ESTOP_CONTROL_TIMEOUT 0x40   // bit 6: Control ESP32 heartbeat timeout

// Mask for any estop bit set
#define NODE_FAULT_ESTOP_ANY         0x7F

// Planner faults (0x80-0x8F, scalar)
#define NODE_FAULT_PERCEPTION        0x80   // Camera/LiDAR failure
#define NODE_FAULT_LOCALIZATION      0x81   // Localization lost
#define NODE_FAULT_PLANNING          0x82   // Path planner failure
#define NODE_FAULT_PLANNER_HARDWARE  0x83   // Planner hardware issue (thermal, etc.)

// Control faults (0x90-0x9F, scalar)
#define NODE_FAULT_THROTTLE_INIT     0x90   // Throttle mux init failed
#define NODE_FAULT_CAN_TX            0x91   // CAN transmit failures
#define NODE_FAULT_MOTOR_COMM        0x92   // Stepper communication lost
#define NODE_FAULT_SENSOR_INVALID    0x93   // F/R sensor invalid reading
#define NODE_FAULT_RELAY_INIT        0x94   // Relay initialization failed

// ============================================================================
// Heartbeat Flags
// ============================================================================

#define HEARTBEAT_FLAG_ENABLE_COMPLETE  0x01  // Node finished ENABLING, ready for ACTIVE

// ============================================================================
// Node Heartbeat (0x100, 0x110, 0x120)
// ============================================================================
// Sent by ALL nodes periodically (100ms) and immediately on state change.
// byte 0: sequence counter (0-255, wraps)
// byte 1: state (NODE_STATE_* — for Safety: system target state)
// byte 2: fault_code (NODE_FAULT_* — for Safety: e-stop reason, 0 when advancing)
// byte 3: flags (HEARTBEAT_FLAG_*)
// byte 4-7: reserved

typedef struct {
    uint8_t sequence;
    uint8_t state;
    uint8_t fault_code;
    uint8_t flags;
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

typedef struct {
    uint8_t sequence;
    uint8_t throttle;
    int16_t steering_position;
    int16_t braking_position;
} planner_command_t;

// ============================================================================
// F/R Switch States (used internally by Control, shared for protocol reference)
// ============================================================================

#define FR_STATE_NEUTRAL             0x00
#define FR_STATE_FORWARD             0x01
#define FR_STATE_REVERSE             0x02
#define FR_STATE_INVALID             0x03

// ============================================================================
// Override Reasons (internal to Control — NOT transmitted over CAN)
// ============================================================================

#define OVERRIDE_REASON_NONE         0x00
#define OVERRIDE_REASON_PEDAL        0x01
#define OVERRIDE_REASON_FR_CHANGED   0x02

// ============================================================================
// Stale Command Detection
// ============================================================================

#define PLANNER_CMD_STALE_COUNT      10  // same sequence N times = stale

// ============================================================================
// Helper Functions — Pack/Unpack
// ============================================================================

static inline void can_pack_le16(uint8_t *buf, uint16_t value) {
    buf[0] = (uint8_t)(value & 0xFF);
    buf[1] = (uint8_t)((value >> 8) & 0xFF);
}

static inline uint16_t can_unpack_le16(const uint8_t *buf) {
    return (uint16_t)(buf[0] | (buf[1] << 8));
}

static inline void can_pack_le16s(uint8_t *buf, int16_t value) {
    can_pack_le16(buf, (uint16_t)value);
}

static inline int16_t can_unpack_le16s(const uint8_t *buf) {
    return (int16_t)can_unpack_le16(buf);
}

// ============================================================================
// Encode/Decode — Node Heartbeat (shared by ALL nodes: Safety, Planner, Control)
// ============================================================================

static inline void can_encode_heartbeat(uint8_t *data, const node_heartbeat_t *hb) {
    data[0] = hb->sequence;
    data[1] = hb->state;
    data[2] = hb->fault_code;
    data[3] = hb->flags;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
}

static inline void can_decode_heartbeat(const uint8_t *data, node_heartbeat_t *hb) {
    hb->sequence = data[0];
    hb->state = data[1];
    hb->fault_code = data[2];
    hb->flags = data[3];
}

// ============================================================================
// Encode/Decode — Planner Command
// ============================================================================

static inline void can_encode_planner_command(uint8_t *data, const planner_command_t *cmd) {
    data[0] = cmd->sequence;
    data[1] = cmd->throttle;
    can_pack_le16s(&data[2], cmd->steering_position);
    can_pack_le16s(&data[4], cmd->braking_position);
    data[6] = 0;  // reserved
    data[7] = 0;  // reserved
}

static inline void can_decode_planner_command(const uint8_t *data, planner_command_t *cmd) {
    cmd->sequence = data[0];
    cmd->throttle = data[1];
    cmd->steering_position = can_unpack_le16s(&data[2]);
    cmd->braking_position = can_unpack_le16s(&data[4]);
}

// ============================================================================
// Debug String Helpers
// ============================================================================

static inline const char* node_state_to_string(uint8_t state) {
    switch (state) {
        case NODE_STATE_INIT:     return "INIT";
        case NODE_STATE_READY:    return "READY";
        case NODE_STATE_ENABLING: return "ENABLING";
        case NODE_STATE_ACTIVE:   return "ACTIVE";
        case NODE_STATE_OVERRIDE: return "OVERRIDE";
        case NODE_STATE_FAULT:    return "FAULT";
        default:                  return "UNKNOWN";
    }
}

// Decode an estop bitmask into a human-readable string.
// Returns a pointer to a static buffer — NOT thread-safe, but fine for
// sequential ESP_LOG calls. Multiple active faults separated by '+'.
// Example: fault=0x05 -> "button+ultrasonic"
static inline const char* node_estop_to_string(uint8_t fault) {
    if (fault == NODE_FAULT_NONE) return "none";

    // Static buffer sized for worst case: all 7 flags with '+' separators
    // "button+remote+ultrasonic+planner+planner_timeout+control+control_timeout" = 71 chars
    static char buf[80];
    char *p = buf;
    char *end = buf + sizeof(buf);

    #define ESTOP_APPEND(flag, name) do { \
        if ((fault & (flag)) && p < end) { \
            if (p != buf && p < end) *p++ = '+'; \
            const char *s = name; \
            while (*s && p < end - 1) *p++ = *s++; \
        } \
    } while(0)

    ESTOP_APPEND(NODE_FAULT_ESTOP_BUTTON,          "button");
    ESTOP_APPEND(NODE_FAULT_ESTOP_REMOTE,           "remote");
    ESTOP_APPEND(NODE_FAULT_ESTOP_ULTRASONIC,       "ultrasonic");
    ESTOP_APPEND(NODE_FAULT_ESTOP_PLANNER,          "planner");
    ESTOP_APPEND(NODE_FAULT_ESTOP_PLANNER_TIMEOUT,  "planner_timeout");
    ESTOP_APPEND(NODE_FAULT_ESTOP_CONTROL,          "control");
    ESTOP_APPEND(NODE_FAULT_ESTOP_CONTROL_TIMEOUT,  "control_timeout");

    #undef ESTOP_APPEND

    *p = '\0';
    return buf;
}

// Decode a fault code into a human-readable string.
// For estop bitmask values (0x01-0x7F), delegates to node_estop_to_string().
// For scalar planner/control faults (0x80+), returns a fixed string.
static inline const char* node_fault_to_string(uint8_t fault) {
    // Estop bitmask range (0x01-0x7F)
    if (fault != NODE_FAULT_NONE && fault <= NODE_FAULT_ESTOP_ANY)
        return node_estop_to_string(fault);

    switch (fault) {
        case NODE_FAULT_NONE:                return "none";
        // Planner faults
        case NODE_FAULT_PERCEPTION:          return "perception";
        case NODE_FAULT_LOCALIZATION:        return "localization";
        case NODE_FAULT_PLANNING:            return "planning";
        case NODE_FAULT_PLANNER_HARDWARE:    return "planner_hardware";
        // Control faults
        case NODE_FAULT_THROTTLE_INIT:       return "throttle_init";
        case NODE_FAULT_CAN_TX:              return "can_tx";
        case NODE_FAULT_MOTOR_COMM:          return "motor_comm";
        case NODE_FAULT_SENSOR_INVALID:      return "sensor_invalid";
        case NODE_FAULT_RELAY_INIT:          return "relay_init";
        // General
        case NODE_FAULT_GENERAL:             return "general";
        default:                             return "unknown";
    }
}

#ifdef __cplusplus
}
#endif
