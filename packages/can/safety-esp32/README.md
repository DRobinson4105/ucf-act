# safety-esp32

ESP-IDF firmware for the Safety ESP32-C6. Acts as the **system state authority** — it is the only node that can advance the system forward (READY -> ENABLING -> ACTIVE). Monitors all safety inputs, controls the 24V power relay, and broadcasts the system target state via a unified heartbeat.

## System State Authority

Safety owns the system target state and broadcasts it as `state` in its heartbeat (0x100). All nodes observe Safety's heartbeat to know the current system target. Safety advances the target when all nodes are healthy and ready; it retreats to READY when any e-stop, fault, override, or timeout is detected.

**State transitions (target state):**
- **INIT -> READY**: Always, after boot
- **READY -> ENABLING**: Both Planner and Control report READY, both alive, no e-stop
- **ENABLING -> ACTIVE**: Both nodes report ENABLING with `HEARTBEAT_FLAG_ENABLE_COMPLETE`, no e-stop
- **ANY -> READY**: E-stop active, node FAULT, node OVERRIDE, node timeout

## Safety Logic

The Safety ESP32 continuously monitors:
1. **Hardware e-stops**: Push button (HB2-ES544), RF remote (EV1527)
2. **Obstacle detection**: Ultrasonic sensor A02YYUW (threshold: 1000mm ~3.3ft)
3. **Node liveness**: Planner and Control heartbeats (timeout: 500ms)
4. **Node health**: Planner and Control state fields (FAULT/OVERRIDE triggers retreat)

**Power relay behavior:**
- ENABLED when all inputs are clear (no e-stop)
- DISABLED immediately when any e-stop condition detected

**E-stop priority** (first match wins):
1. Push button pressed (HB2-ES544)
2. RF remote kill (EV1527)
3. Ultrasonic triggered (A02YYUW obstacle detected OR sensor not responding)
4. Planner heartbeat state == FAULT
5. Planner heartbeat timeout (500ms)
6. Control heartbeat state == FAULT
7. Control heartbeat timeout (500ms)

## CAN Messages

### Receives

| ID | Name | Description |
|----|------|-------------|
| 0x110 | PLANNER_HEARTBEAT | Planner alive (seq, state, fault_code, flags) |
| 0x120 | CONTROL_HEARTBEAT | Control alive (seq, state, fault_code, flags) |

### Sends

| ID | Name | Rate | Description |
|----|------|------|-------------|
| 0x100 | SAFETY_HEARTBEAT | 100ms + immediate on state change | System target state + e-stop fault code (same `node_heartbeat_t` format as all nodes) |

Safety's heartbeat `state` field = system target state (NODE_STATE_*). Its `fault_code` field = e-stop reason (NODE_FAULT_ESTOP_*, 0 when safe).

### Heartbeat Monitoring

| Node | Timeout | Tracked Fields |
|------|---------|----------------|
| Planner | 500ms | sequence, state (FAULT triggers e-stop) |
| Control | 500ms | sequence, state (FAULT triggers e-stop) |

## E-stop Fault Codes (NODE_FAULT_ESTOP_*)

| Code | Constant | Trigger |
|------|----------|---------|
| 0x00 | NONE | System OK |
| 0x01 | ESTOP_MUSHROOM | Push button HB2-ES544 pressed |
| 0x02 | ESTOP_REMOTE | RF remote EV1527 kill signal active |
| 0x03 | ESTOP_ULTRASONIC | Ultrasonic A02YYUW obstacle (<1000mm) or not responding |
| 0x04 | ESTOP_PLANNER | Planner heartbeat state == FAULT |
| 0x05 | ESTOP_PLANNER_TIMEOUT | No Planner heartbeat for 500ms |
| 0x06 | ESTOP_CONTROL | Control heartbeat state == FAULT |
| 0x07 | ESTOP_CONTROL_TIMEOUT | No Control heartbeat for 500ms |

## Pin Configuration

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| 2 | Power Relay | Output | Active HIGH, pull-down |
| 4 | CAN TX | Output | TWAI peripheral |
| 5 | CAN RX | Input | TWAI peripheral |
| 6 | Push Button HB2-ES544 | Input | Pull-up, active HIGH (NC switch opens on press) |
| 7 | RF Remote EV1527 | Input | Pull-up, active HIGH |
| 8 | Status LED | Output | WS2812 RGB LED |
| 10 | Ultrasonic A02YYUW TX | Output | UART1 TX (sensor RX, mode select) |
| 11 | Ultrasonic A02YYUW RX | Input | UART1 RX, 9600 baud (sensor TX) |

### LED Behavior

| Color | State |
|-------|-------|
| Green blink | Normal, no e-stop |
| Blue blink | CAN activity |
| Red blink | E-stop active |

## Components

| Component | Description |
|-----------|-------------|
| `push_button_hb2es544` | mxuteek HB2-ES544 NC e-stop push button |
| `rf_remote_ev1527` | DieseRC 433MHz RF remote (EV1527 encoding) |
| `ultrasonic_a02yyuw` | A02YYUW waterproof UART ultrasonic sensor |
| `power_relay` | 24V relay/transistor control for motor power |
| `can_twai` | CAN bus driver wrapper (shared) |
| `can_protocol` | Message definitions (shared) |
| `heartbeat` | WS2812 status LED driver (shared) |
| `heartbeat_monitor` | CAN node liveness tracking (shared) |
| `safety_logic` | Extracted e-stop evaluation logic (shared, tested) |
| `system_state` | System state machine — target state advancement (shared, tested) |
| `debug_console` | Interactive UART REPL for bench testing (Kconfig-gated, off by default) |

## Ultrasonic Sensor

- **Protocol**: 4-byte UART frames at 9600 baud
- **Frame format**: `0xFF | HIGH_BYTE | LOW_BYTE | CHECKSUM`
- **Checksum**: `(0xFF + HIGH + LOW) & 0xFF`
- **Stop threshold**: 1000mm (~3.3 ft)
- **Sample timeout**: 200ms (stale readings ignored)

## Debug Console

An optional interactive UART console for bench testing without the full system connected. Disabled by default. See [common/debug_console/README.md](../common/debug_console/README.md) for details.

Commands: `bypass planner|control|all`, `unbypass planner|control|all`, `status`

Enable with `CONFIG_ENABLE_DEBUG_CONSOLE=y` in sdkconfig.defaults.

## Build

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p PORT flash monitor
```
