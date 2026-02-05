# CAN Protocol

CAN bus protocol definitions shared between all nodes. Standard frames use 11-bit IDs at 1 Mbps. UIM2852CA motors use extended 29-bit frames.

## ID Allocation

| Range | Node | Description |
|-------|------|-------------|
| 0x101 | Safety ESP32 | Autonomy permission broadcast |
| 0x110-0x11F | Orin | Heartbeat and commands |
| 0x120-0x12F | Control ESP32 | Heartbeat and status |
| Extended 29-bit | UIM2852CA Motors | SimpleCAN protocol (steering=5, braking=6) |

## Message Summary

### Standard 11-bit Frames

| ID | Name | Sender | Receiver | Description |
|----|------|--------|----------|-------------|
| 0x101 | SAFETY_AUTO_ALLOWED | Safety | Control | Autonomy allowed/blocked with reason |
| 0x110 | ORIN_HEARTBEAT | Orin | Safety | Orin alive signal (seq, state) |
| 0x111 | ORIN_COMMAND | Orin | Control | Throttle, steering, braking commands |
| 0x120 | CONTROL_HEARTBEAT | Control | Safety | Control alive (seq, state, fault) |
| 0x121 | CONTROL_STATUS | Control | Orin | Throttle, F/R, sensors, motor positions |

### Extended 29-bit Frames (UIM2852CA Motors)

| Node ID | Motor | Master ID |
|---------|-------|-----------|
| 5 | Steering | 4 |
| 6 | Braking | 4 |

See `uim2852_protocol.h` for CAN ID encoding formula and protocol details.

## Frame Layouts

### Safety Auto Allowed (0x101)

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | allowed | uint8 | 1=autonomy allowed, 0=blocked |
| 1 | block_reason | uint8 | AUTO_BLOCKED_REASON_* |
| 2 | estop_reason | uint8 | ESTOP_REASON_* (if blocked) |
| 3-7 | reserved | - | - |

### Orin Heartbeat (0x110)

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | sequence | uint8 | Rolling counter 0-255 |
| 1 | state | uint8 | ORIN_STATE_* |
| 2-7 | reserved | - | - |

### Orin Command (0x111)

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | throttle | uint8 | Throttle level 0-7 |
| 1 | reserved | - | - |
| 2-3 | steering | int16 LE | Steering position (encoder counts) |
| 4-5 | braking | int16 LE | Braking position (encoder counts) |
| 6 | reserved | - | - |
| 7 | sequence | uint8 | Rolling counter |

### Control Heartbeat (0x120)

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | sequence | uint8 | Rolling counter |
| 1 | state | uint8 | CONTROL_STATE_* |
| 2 | fault_code | uint8 | CONTROL_FAULT_* |
| 3-7 | reserved | - | - |

### Control Status (0x121)

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | throttle_level | uint8 | 0-7, or 0xFF if disabled |
| 1 | fr_state | uint8 | FR_STATE_* |
| 2 | sensor_flags | uint8 | SENSOR_FLAG_* bitmask |
| 3 | override_reason | uint8 | OVERRIDE_REASON_* |
| 4-5 | steering_pos | int16 LE | Current steering command |
| 6-7 | braking_pos | int16 LE | Current braking command |

## State Codes

### Orin States (ORIN_STATE_*)

| Code | State | Description |
|------|-------|-------------|
| 0 | INIT | Initializing |
| 1 | READY | Ready for operation |
| 2 | AUTONOMOUS | Autonomous mode active |
| 3 | MANUAL | Manual/teleop mode |
| 4 | FAULT | Fault condition |

### Control States (CONTROL_STATE_*)

| Code | State | Description |
|------|-------|-------------|
| 0 | INIT | Initializing |
| 1 | READY | Manual mode, waiting for auto permission |
| 2 | ENABLING | Transitioning to autonomous (200ms settle) |
| 3 | ACTIVE | Autonomous mode active |
| 4 | OVERRIDE | Driver override triggered |
| 5 | FAULT | Fault condition (requires power cycle) |

### Control Fault Codes (CONTROL_FAULT_*)

| Code | Fault | Description |
|------|-------|-------------|
| 0x00 | NONE | No fault |
| 0x01 | THROTTLE_INIT | Throttle mux initialization failed |
| 0x02 | CAN_TX | CAN transmit failed |
| 0x03 | MOTOR_COMM | Motor communication error |
| 0x04 | SENSOR_INVALID | F/R sensor invalid state |

### Override Reasons (OVERRIDE_REASON_*)

| Code | Reason | Description |
|------|--------|-------------|
| 0x00 | NONE | No override |
| 0x01 | PEDAL | Accelerator pedal pressed |
| 0x02 | FR_CHANGED | F/R switch moved from Forward |

### Auto Blocked Reasons (AUTO_BLOCKED_REASON_*)

| Code | Reason | Description |
|------|--------|-------------|
| 0x00 | NONE | Autonomy allowed |
| 0x01 | ESTOP | E-stop active (see estop_reason) |

### E-stop Reasons (ESTOP_REASON_*)

| Code | Reason | Description |
|------|--------|-------------|
| 0x00 | NONE | No e-stop |
| 0x01 | MUSHROOM | Mushroom button pressed |
| 0x02 | REMOTE | Wireless remote kill |
| 0x03 | ULTRASONIC | Obstacle detected |
| 0x04 | ORIN_ERROR | Orin heartbeat state == FAULT |
| 0x05 | ORIN_TIMEOUT | Orin heartbeat timeout |
| 0x06 | CONTROL_TIMEOUT | Control heartbeat timeout |
| 0x07 | CONTROL_ERROR | Control heartbeat state == FAULT |

### F/R Switch States (FR_STATE_*)

| Code | State | Description |
|------|-------|-------------|
| 0 | NEUTRAL | Switch in neutral |
| 1 | FORWARD | Switch in forward |
| 2 | REVERSE | Switch in reverse |
| 3 | INVALID | Both switches active (fault) |

## Flag Bitmasks

### Sensor Flags (Control Status byte 2)

| Bit | Flag | Description |
|-----|------|-------------|
| 0 | PEDAL_PRESSED | Pedal ADC above threshold |
| 1 | FR_FORWARD | Direction optocoupler active |
| 2 | FR_REVERSE | Reverse optocoupler active |
| 3 | ENABLE_RELAY | Pedal bypass relay energized |
| 4 | THROTTLE_RELAY | Throttle mux relay energized |

## Timing

| Parameter | Value | Description |
|-----------|-------|-------------|
| Heartbeat interval | 100ms | All nodes send heartbeat every 100ms |
| Heartbeat timeout | 500ms | Node considered dead after 500ms silence |
| Enable sequence settle | 200ms | Delay before completing autonomous enable |
| Throttle slew rate | 100ms/level | Max 1 throttle level change per 100ms |
| Pedal re-arm time | 500ms | Pedal must be released 500ms to re-arm |
| F/R debounce | 20ms | Switch debounce time |
