# CAN Protocol

CAN bus protocol definitions shared between all nodes. Standard frames use 11-bit IDs at 1 Mbps. UIM2852CA motors use extended 29-bit frames.

## Architecture

Three nodes share a unified state enum and an identical heartbeat format (`node_heartbeat_t`). Safety ESP32 is the system state authority and commands target state using NOT_READY/READY/ENABLE/ACTIVE.

Cause reporting is split into two heartbeat channels:
- `stop_flags` (`NODE_STOP_*`): non-fault stop inputs (button, remote, obstacle, app request, operator intervention)
- `fault_code` (`NODE_FAULT_*`): issues/timeouts (Safety bitmask, Planner/Control scalar issue)

| Node                      | Role                                                                   |
|---------------------------|------------------------------------------------------------------------|
| Safety ESP32              | Monitors sensors, controls power relay, broadcasts system target state |
| Planner (Jetson AGX Orin) | Decides path, sends throttle/steering/braking commands                 |
| Control ESP32             | Executes actuator commands (throttle mux, steering/braking motors)     |

## ID Allocation

| Range           | Node             | Description                                     |
|-----------------|------------------|-------------------------------------------------|
| 0x100-0x10F     | Safety ESP32     | Heartbeat (system target state + stop/fault channels) |
| 0x110-0x11F     | Planner          | Heartbeat and commands                          |
| 0x120-0x12F     | Control ESP32    | Heartbeat                                       |
| Extended 29-bit | UIM2852CA Motors | SimpleCAN protocol (steering=5, braking=6)      |

## Message Summary

### Standard 11-bit Frames

| ID    | Name              | Sender  | Receiver         | Rate              | Description                                   |
|-------|-------------------|---------|------------------|-------------------|-----------------------------------------------|
| 0x100 | SAFETY_HEARTBEAT  | Safety  | Control, Planner | 100ms + on change | System target state, `fault_code`, `stop_flags` |
| 0x110 | PLANNER_HEARTBEAT | Planner | Safety           | 100ms             | Planner alive signal (seq, state, fault_code, status_flags) |
| 0x111 | PLANNER_COMMAND   | Planner | Control          | Continuous        | Throttle, steering, braking commands          |
| 0x120 | CONTROL_HEARTBEAT | Control | Safety           | 100ms + on change | Control alive signal (seq, state, fault_code, stop_flags, status_flags) |

All three heartbeats use the same `node_heartbeat_t` struct and `can_encode_heartbeat()`/`can_decode_heartbeat()` functions.

### Extended 29-bit Frames (UIM2852CA Motors)

| Node ID | Motor    | Master ID |
|---------|----------|-----------|
| 5       | Steering | 4         |
| 6       | Braking  | 4         |

See `stepper_protocol_uim2852.h` for CAN ID encoding formula and protocol details.

## Frame Layouts

### Node Heartbeat (0x100, 0x110, 0x120)

All nodes use the same wire format. The consumer knows the source by CAN ID and interprets `fault_code` accordingly.

| Byte | Field      | Type  | Description                                                                          |
|------|------------|-------|--------------------------------------------------------------------------------------|
| 0    | sequence   | uint8 | Rolling counter 0-255                                                                |
| 1    | state      | uint8 | NODE_STATE_* (Safety heartbeat commands NOT_READY/READY/ENABLE/ACTIVE target states) |
| 2    | fault_code | uint8 | NODE_FAULT_* (Safety fault bitmask, Planner/Control scalar issue)                   |
| 3    | status_flags | uint8 | NODE_STATUS_FLAG_* bitmask                                                    |
| 4    | stop_flags | uint8 | NODE_STOP_* bitmask (non-fault stop causes)                                          |
| 5-7  | reserved   | -     | Zero-filled                                                                          |

### Planner Command (0x111)

| Byte | Field    | Type     | Description                        |
|------|----------|----------|------------------------------------|
| 0    | sequence | uint8    | Rolling counter 0-255              |
| 1    | throttle | uint8    | Throttle level 0-255               |
| 2    | steer_hi | uint8    | Steering MSB (0-720)               |
| 3    | steer_lo | uint8    | Steering LSB (0-720)               |
| 4    | braking  | uint8    | Braking value                      |
| 5-7  | reserved | -        | Zero-filled                        |

## Unified Node States (NODE_STATE_*)

All nodes share the same state enum. Safety heartbeat commands NOT_READY/READY/ENABLE/ACTIVE target states; nodes report live state using the same byte.

| Code | State     | Description                                                   |
|------|-----------|---------------------------------------------------------------|
| 0    | INIT      | Booting, not ready                                            |
| 1    | NOT_READY | Running/manual-safe, but autonomy preconditions not satisfied |
| 2    | READY     | Running/manual-safe and autonomy preconditions satisfied      |
| 3    | ENABLE    | Preparing for autonomous (hardware settle / planning init)    |
| 4    | ACTIVE    | Autonomous mode — processing/sending commands                 |

## Unified Cause Codes (NODE_FAULT_*)

Two encoding modes in the same byte:
- **Safety fault bitmask (0x01-0x3F)**: Used only by Safety heartbeat. Multiple bits can be set.
- **Scalar issue causes (0x80+)**: Used by Planner/Control heartbeats (one value at a time).

### Safety fault bitmask (Safety heartbeat only)

Multiple fault conditions can be active at once; `fault_code` is OR'd together.

| Bit | Code | Constant                           | Trigger                                |
|-----|------|------------------------------------|----------------------------------------|
| -   | 0x00 | NONE                               | No fault                               |
| 0   | 0x01 | SAFETY_ULTRASONIC_UNHEALTHY        | Ultrasonic sensor unhealthy/timeout    |
| 1   | 0x02 | SAFETY_PLANNER_ISSUE               | Planner reported scalar issue          |
| 2   | 0x04 | SAFETY_PLANNER_TIMEOUT             | Planner heartbeat timeout (500ms)      |
| 3   | 0x08 | SAFETY_CONTROL_ISSUE               | Control reported scalar issue          |
| 4   | 0x10 | SAFETY_CONTROL_TIMEOUT             | Control heartbeat timeout (500ms)      |
| 5   | 0x20 | SAFETY_RELAY_UNAVAILABLE           | Safety relay path unavailable          |

### Planner issue causes (0x80-0x8F, scalar)

| Code | Constant         | Description                            |
|------|------------------|----------------------------------------|
| 0x80 | PERCEPTION       | Camera/LiDAR failure                   |
| 0x81 | LOCALIZATION     | Localization lost                      |
| 0x82 | PLANNING         | Path planner failure                   |
| 0x83 | PLANNER_HARDWARE | Planner hardware issue (thermal, etc.) |

### Control scalar causes (0x90-0x9F)

| Code | Constant       | Description                        |
|------|----------------|------------------------------------|
| 0x90 | THROTTLE_INIT  | Throttle mux initialization failed |
| 0x91 | CAN_TX         | CAN transmit failures              |
| 0x92 | MOTOR_COMM     | Stepper communication lost         |
| 0x93 | SENSOR_INVALID | F/R sensor invalid reading         |
| 0x94 | RELAY_INIT     | Relay initialization failed        |

## Stop Flags (NODE_STOP_*)

`stop_flags` carries non-fault stop inputs. Multiple bits can be set.

| Bit | Code | Constant                 | Trigger                                      |
|-----|------|--------------------------|----------------------------------------------|
| -   | 0x00 | NONE                     | No stop input                                |
| 0   | 0x01 | PUSH_BUTTON              | Safety push button pressed                   |
| 1   | 0x02 | REMOTE                   | Safety RF remote kill active                 |
| 2   | 0x04 | ULTRASONIC_OBSTACLE      | Safety ultrasonic obstacle detected          |
| 3   | 0x08 | APP_REQUEST              | Planner/app requested autonomy stop          |
| 4   | 0x10 | OPERATOR_PEDAL           | Control pedal intervention                   |
| 5   | 0x20 | OPERATOR_FR              | Control F/R intervention                     |
| 6   | 0x40 | OPERATOR_STEER           | Control steering intervention                |
| 7   | 0x80 | OPERATOR_BRAKE           | Control braking intervention                 |

### General

| Code | Constant | Description       |
|------|----------|-------------------|
| 0xFF | GENERAL  | Unspecified issue |

## Heartbeat Status Flags

| Bit | Flag             | Description                                                                                                                             |
|-----|------------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 0   | ENABLE_COMPLETE  | Node finished ENABLE, ready for ACTIVE                                                                                                  |
| 1   | AUTONOMY_REQUEST | Planner requests autonomy enable (READY -> ENABLE gate) and keeps it asserted while autonomy should remain active (drop = halt retreat) |
| 2   | RESERVED         | Reserved                                                                                                                                |

## Override Reasons (OVERRIDE_REASON_*)

Internal to Control — NOT transmitted over CAN.

| Code | Reason     | Description                   |
|------|------------|-------------------------------|
| 0x00 | NONE       | No override                   |
| 0x01 | PEDAL      | Accelerator pedal pressed     |
| 0x02 | FR_CHANGED | F/R switch moved to Reverse while ACTIVE |
| 0x03 | STEERING   | Steering wheel override        |
| 0x04 | BRAKING    | Brake pedal override           |

## Stale Command Detection

Control tracks the Planner command sequence number. If the same sequence is seen `PLANNER_CMD_STALE_COUNT` (10) consecutive times, Control zeros the throttle as a safety measure.

## Timing

| Parameter              | Value           | Description                                                 |
|------------------------|-----------------|-------------------------------------------------------------|
| Heartbeat interval     | 100ms           | All nodes send heartbeat every 100ms                        |
| Immediate heartbeat    | On state change | Both Safety and Control send immediately when state changes |
| Heartbeat timeout      | 500ms           | Node considered dead after 500ms silence                    |
| Enable sequence settle | 200ms           | Delay before completing autonomous enable                   |
| Throttle slew rate     | 100ms/level     | Max 1 throttle level change per 100ms                       |
| Pedal re-arm time      | 500ms           | Pedal must be released 500ms to re-arm                      |
| F/R debounce           | 20ms            | Switch debounce time                                        |
