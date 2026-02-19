# CAN Protocol

CAN bus protocol definitions shared between all nodes. Standard frames use 11-bit IDs at 1 Mbps. UIM2852CA motors use extended 29-bit frames.

## Architecture

Three nodes share a unified state/fault enum and an identical heartbeat format (`node_heartbeat_t`). Safety ESP32 is the system state authority and commands target state using NOT_READY/READY/ENABLE/ACTIVE. Planner and Control own live-state transitions and can enter OVERRIDE or FAULT locally for immediate safety.

| Node                      | Role                                                                   |
|---------------------------|------------------------------------------------------------------------|
| Safety ESP32              | Monitors sensors, controls power relay, broadcasts system target state |
| Planner (Jetson AGX Orin) | Decides path, sends throttle/steering/braking commands                 |
| Control ESP32             | Executes actuator commands (throttle mux, steering/braking motors)     |

## ID Allocation

| Range           | Node             | Description                                     |
|-----------------|------------------|-------------------------------------------------|
| 0x100-0x10F     | Safety ESP32     | Heartbeat (system target state + e-stop reason) |
| 0x110-0x11F     | Planner          | Heartbeat and commands                          |
| 0x120-0x12F     | Control ESP32    | Heartbeat                                       |
| Extended 29-bit | UIM2852CA Motors | SimpleCAN protocol (steering=5, braking=6)      |

## Message Summary

### Standard 11-bit Frames

| ID    | Name              | Sender  | Receiver         | Rate              | Description                                   |
|-------|-------------------|---------|------------------|-------------------|-----------------------------------------------|
| 0x100 | SAFETY_HEARTBEAT  | Safety  | Control, Planner | 100ms + on change | System target state, e-stop fault code        |
| 0x110 | PLANNER_HEARTBEAT | Planner | Safety           | 100ms             | Planner alive signal (seq, state, fault_code, flags) |
| 0x111 | PLANNER_COMMAND   | Planner | Control          | Continuous        | Throttle, steering, braking commands          |
| 0x120 | CONTROL_HEARTBEAT | Control | Safety           | 100ms + on change | Control alive signal (seq, state, fault_code, flags) |

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
| 2    | fault_code | uint8 | NODE_FAULT_* (Safety: e-stop reason, 0 when safe)                                    |
| 3    | flags      | uint8 | HEARTBEAT_FLAG_* bitmask                                                             |
| 4-7  | reserved   | -     | Zero-filled                                                                          |

### Planner Command (0x111)

| Byte | Field    | Type     | Description                        |
|------|----------|----------|------------------------------------|
| 0    | sequence | uint8    | Rolling counter 0-255              |
| 1    | throttle | uint8    | Throttle level 0-7                 |
| 2-3  | steering | int16 LE | Steering position (encoder counts) |
| 4-5  | braking  | int16 LE | Braking position (encoder counts)  |
| 6-7  | reserved | -        | Zero-filled                        |

## Unified Node States (NODE_STATE_*)

All nodes share the same state enum. Safety heartbeat commands NOT_READY/READY/ENABLE/ACTIVE target states; nodes report live states and may enter OVERRIDE/FAULT locally.

| Code | State     | Description                                                   |
|------|-----------|---------------------------------------------------------------|
| 0    | INIT      | Booting, not ready                                            |
| 1    | NOT_READY | Running/manual-safe, but autonomy preconditions not satisfied |
| 2    | READY     | Running/manual-safe and autonomy preconditions satisfied      |
| 3    | ENABLE    | Preparing for autonomous (hardware settle / planning init)    |
| 4    | ACTIVE    | Autonomous mode — processing/sending commands                 |
| 5    | OVERRIDE  | Driver took over (local, immediate)                           |
| 6    | FAULT     | Fault condition (local, immediate)                            |

## Unified Fault Codes (NODE_FAULT_*)

Two encoding modes in the same byte:
- **E-stop bitmask (0x01-0x7F)**: Used only by Safety's heartbeat. Multiple bits can be set simultaneously.
- **Scalar faults (0x80+)**: Used by Planner and Control heartbeats. One value at a time.

### E-stop bitmask (Safety heartbeat only)

Multiple conditions can be active at once. The fault_code byte is OR'd together. For example, button + remote = 0x01 | 0x02 = 0x03.

| Bit | Code | Constant              | Trigger                              |
|-----|------|-----------------------|--------------------------------------|
| -   | 0x00 | NONE                  | No fault                             |
| 0   | 0x01 | ESTOP_BUTTON          | Push button HB2-ES544 pressed        |
| 1   | 0x02 | ESTOP_REMOTE          | RF remote EV1527 kill active         |
| 2   | 0x04 | ESTOP_ULTRASONIC      | Ultrasonic A02YYUW obstacle or fault |
| 3   | 0x08 | ESTOP_PLANNER         | Planner reported FAULT state         |
| 4   | 0x10 | ESTOP_PLANNER_TIMEOUT | Planner heartbeat timeout (500ms)    |
| 5   | 0x20 | ESTOP_CONTROL         | Control reported FAULT state         |
| 6   | 0x40 | ESTOP_CONTROL_TIMEOUT | Control heartbeat timeout (500ms)    |

### Planner faults (0x80-0x8F, scalar)

| Code | Constant         | Description                            |
|------|------------------|----------------------------------------|
| 0x80 | PERCEPTION       | Camera/LiDAR failure                   |
| 0x81 | LOCALIZATION     | Localization lost                      |
| 0x82 | PLANNING         | Path planner failure                   |
| 0x83 | PLANNER_HARDWARE | Planner hardware issue (thermal, etc.) |

### Control faults (0x90-0x9F, scalar)

| Code | Constant       | Description                        |
|------|----------------|------------------------------------|
| 0x90 | THROTTLE_INIT  | Throttle mux initialization failed |
| 0x91 | CAN_TX         | CAN transmit failures              |
| 0x92 | MOTOR_COMM     | Stepper communication lost         |
| 0x93 | SENSOR_INVALID | F/R sensor invalid reading         |
| 0x94 | RELAY_INIT     | Relay initialization failed        |

### General

| Code | Constant | Description       |
|------|----------|-------------------|
| 0xFF | GENERAL  | Unspecified fault |

## Heartbeat Flags

| Bit | Flag             | Description                                                                                                                             |
|-----|------------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 0   | ENABLE_COMPLETE  | Node finished ENABLE, ready for ACTIVE                                                                                                  |
| 1   | AUTONOMY_REQUEST | Planner requests autonomy enable (READY -> ENABLE gate) and keeps it asserted while autonomy should remain active (drop = halt retreat) |
| 2   | RESERVED         | Reserved (legacy ENABLE_READY bit; ignored)                                                                                             |

## F/R Switch States (FR_STATE_*)

| Code | State   | Description                  |
|------|---------|------------------------------|
| 0    | NEUTRAL | Switch in neutral            |
| 1    | FORWARD | Switch in forward            |
| 2    | REVERSE | Switch in reverse            |
| 3    | INVALID | Both switches active (fault) |

## Override Reasons (OVERRIDE_REASON_*)

Internal to Control — NOT transmitted over CAN.

| Code | Reason     | Description                   |
|------|------------|-------------------------------|
| 0x00 | NONE       | No override                   |
| 0x01 | PEDAL      | Accelerator pedal pressed     |
| 0x02 | FR_CHANGED | F/R switch moved from Forward |

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
