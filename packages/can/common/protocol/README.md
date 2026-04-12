# Shared Protocols

Wire-format definitions shared between all nodes. Safety, Control, and the UIM2852CA motors communicate over CAN. The Orin talks to Safety and Control directly over **two independent UART0 links**, each routed through the ESP32-C6-DevKitM-1 "COM" USB-C port (CP2102N bridge) at 1 Mbaud, framed by `orin_link_protocol` on top of `usb_serial_link`.

## Architecture

Three nodes share a unified state enum and an identical heartbeat format (`node_heartbeat_t`). Safety ESP32 is the system state authority and commands target state using NOT_READY/READY/ENABLE/ACTIVE.

Cause reporting is split into two heartbeat channels:

- `stop_flags` (`NODE_STOP_*`): non-fault stop inputs (button, remote, obstacle, app request, operator intervention)
- `fault_flags` (`NODE_FAULT_*`): node-specific fault bitmasks with prefix-based range encoding

| Node                      | Role                                                                            |
| ------------------------- | ------------------------------------------------------------------------------- |
| Safety ESP32              | Monitors sensors, controls power relay, broadcasts system target state          |
| Planner (Jetson AGX Orin) | Decides path, sends throttle/steering/braking commands over its Control UART    |
| Control ESP32             | Executes actuator commands (throttle, steering/braking motors)                  |

## Transport Split

- CAN (1 Mbps, shared bus):
  - Safety ↔ Control heartbeats
  - Control ↔ steering/braking motors (UIM2852CA SimpleCAN, extended 29-bit)
- Orin ↔ Control UART0 link (`usb_serial_link` + `orin_link_protocol`):
  - Orin → Control: `PLANNER_COMMAND`
  - Control → Orin: `CONTROL_HEARTBEAT`
- Orin ↔ Safety UART0 link (`usb_serial_link` + `orin_link_protocol`):
  - Orin → Safety: `PLANNER_HEARTBEAT`
  - Safety → Orin: `SAFETY_HEARTBEAT`

The two Orin UART links are physically separate cables — one USB-C from the Control board's COM port to the Orin, another USB-C from the Safety board's COM port to the Orin. Each cable only carries the subset of messages listed for that pair. The Control cable never sees `PLANNER_HEARTBEAT`; the Safety cable never sees `PLANNER_COMMAND`.

## CAN ID Allocation

| Range           | Node             | Description                                           |
| --------------- | ---------------- | ----------------------------------------------------- |
| 0x100-0x10F     | Safety ESP32     | Heartbeat (system target state + stop/fault channels) |
| 0x120-0x12F     | Control ESP32    | Heartbeat                                             |
| Extended 29-bit | UIM2852CA Motors | SimpleCAN protocol (steering=7, braking=6)            |

## Message Summary

### CAN Messages

| ID       | Name              | Sender         | Receiver       | Rate              | Description                                                                    |
| -------- | ----------------- | -------------- | -------------- | ----------------- | ------------------------------------------------------------------------------ |
| 0x100    | SAFETY_HEARTBEAT  | Safety         | Control        | 100ms + on change | System target state, `fault_flags`, `stop_flags`, battery SOC                  |
| 0x120    | CONTROL_HEARTBEAT | Control        | Safety         | 100ms + on change | Control alive signal (seq, state, fault_flags, stop_flags, status_flags)       |
| Extended | MOTOR_UIM2852_*         | Control/Motors | Control/Motors | as needed         | UIM2852CA SimpleCAN instructions and responses for the steering/braking motors |

The two on-CAN heartbeats share the `node_heartbeat_t` struct and use `can_encode_heartbeat()`/`can_decode_heartbeat()`. The Orin-link heartbeats (planner/control/safety mirror) reuse the same struct and encoder, just framed by `orin_link_protocol` instead of a CAN frame.

### Orin UART Messages

`orin_link_protocol` wraps each message in a fixed-overhead framing emitted by `usb_serial_link`:

```
+--------+--------+----------+-----------+
|  0xAA  |  type  | payload  | payload   |
|  sync  |        |  length  |  bytes    |
+--------+--------+----------+-----------+
 1 byte   1 byte    1 byte     N bytes
```

- **Sync byte** is always `0xAA` (`ORIN_LINK_SYNC_BYTE` in `orin_link_protocol.h`). It lets the receiver resync after a glitch or partial write.
- **Type byte** identifies the message class (see table below).
- **Length byte** is the number of payload bytes that follow. Valid values: `6` for `PLANNER_COMMAND`, `8` for every heartbeat variant. Max allowed = `USB_SERIAL_LINK_MAX_PAYLOAD_LEN` (8).
- **Payload** is the raw bytes produced by the matching `can_protocol` encoder.

Message types:

| Type | Name                  | Cable                   | Direction        | Payload length | Payload format      |
| ---- | --------------------- | ----------------------- | ---------------- | -------------- | ------------------- |
| 0x01 | `PLANNER_COMMAND`     | Orin ↔ Control COM UART | Orin → Control   | 6 bytes        | `planner_command_t` |
| 0x02 | `PLANNER_HEARTBEAT`   | Orin ↔ Safety COM UART  | Orin → Safety    | 8 bytes        | `node_heartbeat_t`  |
| 0x03 | `CONTROL_HEARTBEAT`   | Orin ↔ Control COM UART | Control → Orin   | 8 bytes        | `node_heartbeat_t`  |
| 0x04 | `SAFETY_HEARTBEAT`    | Orin ↔ Safety COM UART  | Safety → Orin    | 8 bytes        | `node_heartbeat_t`  |

**Concrete example** — a `PLANNER_COMMAND` with `sequence=0x05`, `throttle=0x0320` (800), `steering_position=0x02D0` (720), `braking_position=0x03` (level 3, fully engaged) goes on the wire as:

```
AA 01 06 05 03 20 02 D0 03
```

Byte-by-byte: `AA` sync, `01` type, `06` length, then `05 03 20 02 D0 03` = sequence, throttle MSB, throttle LSB, steering MSB, steering LSB, braking.

A `CONTROL_HEARTBEAT` from Control → Orin with `sequence=0x10`, `state=ACTIVE (4)`, `fault_flags=0x00`, `status_flags=0x01`, `stop_flags=0x00`, `soc_pct=0` goes on the wire as:

```
AA 03 08 10 04 00 01 00 00 00 00
```

(The trailing two `00` bytes are the reserved tail of `node_heartbeat_t`.)

### Extended 29-bit Frames (UIM2852CA Motors)

| Node ID | Motor    |
| ------- | -------- |
| 7       | Steering |
| 6       | Braking  |

See `motor_protocol.h` in `control-esp32/components/motor_uim2852/include/` for CAN ID encoding and protocol details.

## Frame Layouts

### Node Heartbeat Payload (CAN 0x100/0x120, Orin UART heartbeat payloads — 8 bytes)

All nodes use the same wire format. The consumer knows the source by CAN ID or Orin-link message type and interprets `fault_flags` accordingly.

| Byte | Field        | Type  | Description                                                                           |
| ---- | ------------ | ----- | ------------------------------------------------------------------------------------- |
| 0    | sequence     | uint8 | Rolling counter 0-255                                                                 |
| 1    | state        | uint8 | NODE*STATE*\* (Safety heartbeat commands NOT_READY/READY/ENABLE/ACTIVE target states) |
| 2    | fault_flags  | uint8 | NODE*FAULT*\* (node-specific fault bitmask, 0 when clear)                             |
| 3    | status_flags | uint8 | NODE*STATUS_FLAG*\* bitmask                                                           |
| 4    | stop_flags   | uint8 | NODE*STOP*\* bitmask (non-fault stop causes)                                          |
| 5    | soc_pct      | uint8 | Battery state of charge 0-100% (Safety heartbeat only, 0 on others)                   |
| 6-7  | reserved     | -     | Zero-filled                                                                           |

### Planner Command Payload (Orin UART message type `0x01` — 6 bytes)

Emitted by `can_encode_planner_command()`. `orin_link_encode_planner_command_message()` sets `payload_len = 6` — only bytes 0-5 ship on the UART link. There is no byte 6 or 7 in the framed message; the encoder writes zero-padding into a larger local buffer but the link transmits just the first 6.

| Byte | Field       | Type   | Description                                             |
| ---- | ----------- | ------ | ------------------------------------------------------- |
| 0    | sequence    | uint8  | Rolling counter 0-255                                   |
| 1    | throttle_hi | uint8  | Throttle MSB (0-4095, deadband below ~800)              |
| 2    | throttle_lo | uint8  | Throttle LSB                                            |
| 3    | steer_hi    | uint8  | Steering MSB (0-720)                                    |
| 4    | steer_lo    | uint8  | Steering LSB (0-720)                                    |
| 5    | braking     | uint8  | 4-level braking command (0-3, see `PLANNER_BRAKING_MAX_LEVEL`) |

**Braking levels.** The braking byte is a discrete 4-level command, not a raw pulse count. Control translates it to a UIM2852 absolute-position target on receipt:

| Planner value | Meaning               | Control target position (pulses)          |
| ------------- | --------------------- | ------------------------------------------ |
| `0`           | Fully released        | `CONFIG_BRAKE_RELEASE_POSITION`            |
| `1`           | One-third engaged     | `2/3 × CONFIG_BRAKE_RELEASE_POSITION`      |
| `2`           | Two-thirds engaged    | `1/3 × CONFIG_BRAKE_RELEASE_POSITION`      |
| `3`           | Fully engaged         | `0`                                        |

Values above `3` are clamped to `3` (fully engaged — the fail-safe direction) by `control_can_rx_process_planner_command_payload()` before translation.

## Unified Node States (NODE*STATE*\*)

All nodes share the same state enum. Safety heartbeat commands NOT_READY/READY/ENABLE/ACTIVE target states; nodes report live state using the same byte.

| Code | State     | Description                                                   |
| ---- | --------- | ------------------------------------------------------------- |
| 0    | INIT      | Booting, not ready                                            |
| 1    | NOT_READY | Running/manual-safe, but autonomy preconditions not satisfied |
| 2    | READY     | Running/manual-safe and autonomy preconditions satisfied      |
| 3    | ENABLE    | Preparing for autonomous (hardware settle / planning init)    |
| 4    | ACTIVE    | Autonomous mode — processing/sending commands                 |

## Fault Flags (NODE*FAULT*\*)

All nodes use bitmask encoding with node-specific prefixes. Multiple faults can be set simultaneously by ORing flags. The prefix bits keep the ranges disjoint so a fault value alone identifies which node it belongs to.

| Range     | Node    | Prefix | Fault bits | Description                |
| --------- | ------- | ------ | ---------- | -------------------------- |
| 0x00      | Any     | -      | -          | No fault                   |
| 0x01-0x3F | Safety  | none   | bits 0-5   | Safety-level fault bitmask |
| 0x40-0x4F | Planner | 0x40   | bits 0-3   | Planner fault bitmask      |
| 0x80-0x9F | Control | 0x80   | bits 0-4   | Control fault bitmask      |
| 0xFF      | Any     | -      | -          | General / unspecified      |

### Safety fault bitmask (0x01-0x3F, Safety heartbeat only)

| Bit | Code | Constant                    | Trigger                             |
| --- | ---- | --------------------------- | ----------------------------------- |
| 0   | 0x01 | SAFETY_ULTRASONIC_UNHEALTHY | Ultrasonic sensor unhealthy/timeout |
| 1   | 0x02 | SAFETY_PLANNER_ISSUE        | Planner reported fault              |
| 2   | 0x04 | SAFETY_PLANNER_TIMEOUT      | Planner heartbeat timeout (500ms)   |
| 3   | 0x08 | SAFETY_CONTROL_ISSUE        | Control reported fault              |
| 4   | 0x10 | SAFETY_CONTROL_TIMEOUT      | Control heartbeat timeout (500ms)   |
| 5   | 0x20 | SAFETY_RELAY_UNAVAILABLE    | Safety relay path unavailable       |

### Planner fault bitmask (0x40-0x4F, Planner heartbeat only)

| Bit | Code | Constant             | Trigger                        |
| --- | ---- | -------------------- | ------------------------------ |
| 0   | 0x41 | PLANNER_PERCEPTION   | Camera/LiDAR failure           |
| 1   | 0x42 | PLANNER_LOCALIZATION | Localization lost              |
| 2   | 0x44 | PLANNER_PLANNING     | Path planner failure           |
| 3   | 0x48 | PLANNER_HARDWARE     | Hardware issue (thermal, etc.) |

### Control fault bitmask (0x80-0x9F, Control heartbeat only)

| Bit | Code | Constant               | Trigger                     |
| --- | ---- | ---------------------- | --------------------------- |
| 0   | 0x81 | CONTROL_THROTTLE_INIT  | Throttle init failed        |
| 1   | 0x82 | CONTROL_CAN_TX         | CAN transmit failures       |
| 2   | 0x84 | CONTROL_MOTOR_COMM     | Motor communication lost  |
| 3   | 0x88 | CONTROL_SENSOR_INVALID | F/R sensor invalid reading  |
| 4   | 0x90 | CONTROL_RELAY_INIT     | Relay initialization failed |

## Stop Flags (NODE*STOP*\*)

`stop_flags` carries non-fault stop inputs. Multiple bits can be set.

| Bit | Code | Constant            | Trigger                             |
| --- | ---- | ------------------- | ----------------------------------- |
| -   | 0x00 | NONE                | No stop input                       |
| 0   | 0x01 | PUSH_BUTTON         | Safety push button pressed          |
| 1   | 0x02 | REMOTE              | Safety RF remote kill active        |
| 2   | 0x04 | ULTRASONIC_OBSTACLE | Safety ultrasonic obstacle detected |
| 3   | 0x08 | APP_REQUEST         | Planner/app requested autonomy stop |
| 4   | 0x10 | OPERATOR_REVERSE    | Control reverse intervention        |
| 5   | 0x20 | OPERATOR_THROTTLE   | Control throttle intervention       |
| 6   | 0x40 | OPERATOR_STEER      | Control steering intervention       |
| 7   | 0x80 | OPERATOR_BRAKE      | Control braking intervention        |

### General

| Code | Constant | Description       |
| ---- | -------- | ----------------- |
| 0xFF | GENERAL  | Unspecified fault |

## Heartbeat Status Flags

| Bit | Flag             | Description                                                                                                                             |
| --- | ---------------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| 0   | ENABLE_COMPLETE  | Node finished ENABLE, ready for ACTIVE                                                                                                  |
| 1   | AUTONOMY_REQUEST | Planner requests autonomy enable (READY -> ENABLE gate) and keeps it asserted while autonomy should remain active (drop = halt retreat) |
| 2   | RESERVED         | Reserved                                                                                                                                |

## Override Reasons (OVERRIDE*REASON*\*)

Internal to Control — not transmitted over CAN or the Orin UART link.

| Code | Reason   | Description                              |
| ---- | -------- | ---------------------------------------- |
| 0x00 | NONE     | No override                              |
| 0x01 | REVERSE  | F/R switch moved to Reverse while ACTIVE |
| 0x02 | THROTTLE | Accelerator pedal pressed                |
| 0x03 | STEERING | Steering position error (external force) |
| 0x04 | BRAKING  | Braking position error (external force)  |

## Stale Command Detection

Control tracks the Planner command sequence number. If the same sequence is seen 10 consecutive times, Control zeros the throttle as a safety measure.

## Timing

| Parameter               | Value           | Description                                                 |
| ----------------------- | --------------- | ----------------------------------------------------------- |
| Heartbeat interval      | 100ms           | All nodes send heartbeat every 100ms                        |
| Immediate heartbeat     | On state change | Both Safety and Control send immediately when state changes |
| Heartbeat timeout       | 500ms           | Node considered dead after 500ms silence                    |
| Enable sequence settle  | 200ms           | Delay before completing autonomous enable                   |
| Throttle slew rate      | 200 steps/100ms | Max 200 DAC steps per 100ms interval (0-4095 range, deadband below ~800) |
| Pedal re-arm time       | 500ms           | Pedal must be released 500ms to re-arm                      |
| F/R debounce            | 20ms            | Switch debounce time                                        |
| Planner command timeout | 500ms           | Control zeros throttle if no command within window          |
| Planner stale count     | 10              | Same sequence 10 times + past half timeout = stale          |
| Enable timeout          | 5000ms          | Safety retreats from ENABLE if nodes don't complete         |
