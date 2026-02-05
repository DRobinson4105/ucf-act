#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// CAN Message ID Allocation
// =============================================================================

// Safety ESP32 (0x100-0x10F)
#define CAN_ID_SAFETY_AUTO_ALLOWED   0x101   // Autonomy allowed broadcast

// Orin (0x110-0x11F)
#define CAN_ID_ORIN_HEARTBEAT        0x110   // Orin alive signal (seq, state)
#define CAN_ID_ORIN_COMMAND          0x111   // Throttle/steering/braking commands

// Control ESP32 (0x120-0x12F)
#define CAN_ID_CONTROL_HEARTBEAT     0x120   // Control alive (seq, state, fault_code)
#define CAN_ID_CONTROL_STATUS        0x121   // Throttle, F/R, sensors, positions

// UIM2852CA Motors - use CAN 2.0B extended 29-bit frames with SimpleCAN protocol
// See uim2852_protocol.h for CAN ID calculation and protocol details
#define UIM2852_MASTER_ID            4   // Producer ID for host controller
#define UIM2852_NODE_STEERING        5   // Steering motor node ID
#define UIM2852_NODE_BRAKING         6   // Braking motor node ID
#define UIM2852_GLOBAL_ID            0   // Broadcast to all motors

// =============================================================================
// Timing Constants
// =============================================================================

#define HEARTBEAT_INTERVAL_MS        100
#define HEARTBEAT_TIMEOUT_MS         500

// =============================================================================
// Safety ESP32 Messages
// =============================================================================

// Safety auto allowed frame layout:
// byte 0: allowed (1=autonomy allowed, 0=blocked)
// byte 1: block reason (AUTO_BLOCKED_REASON_*)
// byte 2: estop reason (ESTOP_REASON_*, if blocked by estop)
// byte 3-7: reserved
typedef struct {
    uint8_t allowed;
    uint8_t block_reason;
    uint8_t estop_reason;
} safety_auto_allowed_t;

#define AUTO_BLOCKED_REASON_NONE     0x00   // Autonomy allowed
#define AUTO_BLOCKED_REASON_ESTOP    0x01   // Blocked due to e-stop

// E-stop reasons (why e-stop is active)
#define ESTOP_REASON_NONE            0x00
#define ESTOP_REASON_MUSHROOM        0x01
#define ESTOP_REASON_REMOTE          0x02
#define ESTOP_REASON_ULTRASONIC      0x03
#define ESTOP_REASON_ORIN_ERROR      0x04
#define ESTOP_REASON_ORIN_TIMEOUT    0x05
#define ESTOP_REASON_CONTROL_TIMEOUT 0x06
#define ESTOP_REASON_CONTROL_ERROR   0x07

// =============================================================================
// Orin Messages
// =============================================================================

// Orin heartbeat frame layout:
// byte 0: sequence counter (0-255, wraps)
// byte 1: state (ORIN_STATE_*)
// byte 2-7: reserved
typedef struct {
    uint8_t sequence;
    uint8_t state;
} orin_heartbeat_t;

#define ORIN_STATE_INIT              0
#define ORIN_STATE_READY             1
#define ORIN_STATE_AUTONOMOUS        2
#define ORIN_STATE_MANUAL            3
#define ORIN_STATE_FAULT             4

// Orin command frame layout:
// byte 0: throttle level (0-7)
// byte 1: reserved
// byte 2-3: steering position (int16, encoder counts, little-endian)
// byte 4-5: braking position (int16, encoder counts, little-endian)
// byte 6: reserved
// byte 7: sequence counter
typedef struct {
    uint8_t throttle;
    int16_t steering_position;
    int16_t braking_position;
    uint8_t sequence;
} orin_command_t;

// =============================================================================
// Control ESP32 Messages
// =============================================================================

// Control heartbeat frame layout:
// byte 0: sequence counter
// byte 1: state (CONTROL_STATE_*)
// byte 2: fault code (CONTROL_FAULT_*)
// byte 3-7: reserved
typedef struct {
    uint8_t sequence;
    uint8_t state;
    uint8_t fault_code;
} control_heartbeat_t;

#define CONTROL_STATE_INIT           0
#define CONTROL_STATE_READY          1
#define CONTROL_STATE_ENABLING       2
#define CONTROL_STATE_ACTIVE         3
#define CONTROL_STATE_OVERRIDE       4
#define CONTROL_STATE_FAULT          5

#define CONTROL_FAULT_NONE           0x00
#define CONTROL_FAULT_THROTTLE_INIT  0x01
#define CONTROL_FAULT_CAN_TX         0x02
#define CONTROL_FAULT_MOTOR_COMM     0x03
#define CONTROL_FAULT_SENSOR_INVALID 0x04

// Control status frame layout:
// byte 0: throttle level (0-7, or 0xFF if disabled)
// byte 1: F/R (forward reverse switch) state (FR_STATE_*)
// byte 2: sensor flags bitmask
// byte 3: override reason
// byte 4-5: steering position (little-endian)
// byte 6-7: braking position (little-endian)
typedef struct {
    uint8_t throttle_level;
    uint8_t fr_state;
    uint8_t sensor_flags;
    uint8_t override_reason;
    int16_t steering_position;
    int16_t braking_position;
} control_status_t;

#define OVERRIDE_REASON_NONE         0x00
#define OVERRIDE_REASON_PEDAL        0x01
#define OVERRIDE_REASON_FR_CHANGED   0x02

#define FR_STATE_NEUTRAL             0x00
#define FR_STATE_FORWARD             0x01
#define FR_STATE_REVERSE             0x02
#define FR_STATE_INVALID             0x03

// Sensor flag bits
#define SENSOR_FLAG_PEDAL_PRESSED    0x01
#define SENSOR_FLAG_FR_FORWARD       0x02
#define SENSOR_FLAG_FR_REVERSE       0x04
#define SENSOR_FLAG_ENABLE_RELAY     0x08
#define SENSOR_FLAG_THROTTLE_RELAY   0x10

// =============================================================================
// Helper Functions
// =============================================================================

// Pack/unpack unsigned little-endian (least significant units first) values
static inline void can_pack_le16(uint8_t *buf, uint16_t value) {
    buf[0] = (uint8_t)(value & 0xFF);
    buf[1] = (uint8_t)((value >> 8) & 0xFF);
}

static inline uint16_t can_unpack_le16(const uint8_t *buf) {
    return (uint16_t)(buf[0] | (buf[1] << 8));
}

// Pack/unpack signed little-endian (least significant units first) values
static inline void can_pack_le16s(uint8_t *buf, int16_t value) {
    can_pack_le16(buf, (uint16_t)value);
}

static inline int16_t can_unpack_le16s(const uint8_t *buf) {
    return (int16_t)can_unpack_le16(buf);
}

// Encode/decode orin command
static inline void can_encode_orin_command(uint8_t *data, const orin_command_t *cmd) {
    data[0] = cmd->throttle;
    data[1] = 0;  // reserved
    can_pack_le16s(&data[2], cmd->steering_position);
    can_pack_le16s(&data[4], cmd->braking_position);
    data[6] = 0;  // reserved
    data[7] = cmd->sequence;
}

static inline void can_decode_orin_command(const uint8_t *data, orin_command_t *cmd) {
    cmd->throttle = data[0];
    cmd->steering_position = can_unpack_le16s(&data[2]);
    cmd->braking_position = can_unpack_le16s(&data[4]);
    cmd->sequence = data[7];
}

// Encode/decode safety auto allowed
static inline void can_encode_safety_auto_allowed(uint8_t *data, const safety_auto_allowed_t *msg) {
    data[0] = msg->allowed;
    data[1] = msg->block_reason;
    data[2] = msg->estop_reason;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
}

static inline void can_decode_safety_auto_allowed(const uint8_t *data, safety_auto_allowed_t *msg) {
    msg->allowed = data[0];
    msg->block_reason = data[1];
    msg->estop_reason = data[2];
}

#ifdef __cplusplus
}
#endif
