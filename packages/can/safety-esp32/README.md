# safety-esp32

ESP-IDF firmware for the Safety ESP32-C6. Monitors all safety inputs and controls the 24V power relay. Acts as the authority for autonomous permission - Control ESP32 cannot enable autonomous mode unless Safety broadcasts `allowed=1`.

## Safety Logic

The Safety ESP32 continuously monitors:
1. **Hardware e-stops**: Push button (HB2-ES544), RF remote (EV1527)
2. **Obstacle detection**: Ultrasonic sensor A02YYUW (threshold: 1000mm=~3.3ft)
3. **Node liveness**: Orin and Control heartbeats (timeout: 500ms)
4. **Node health**: Orin and Control state fields (FAULT state triggers e-stop)

**Power relay behavior:**
- ENABLED when all inputs are clear (autonomy allowed)
- DISABLED immediately when any e-stop condition detected

**E-stop priority** (first match wins):
1. Push button pressed (HB2-ES544)
2. RF remote kill (EV1527)
3. Ultrasonic triggered (A02YYUW obstacle detected OR sensor not responding)
4. Orin heartbeat state == FAULT
5. Orin heartbeat timeout (500ms)
6. Control heartbeat timeout (500ms)
7. Control heartbeat state == FAULT

## CAN Messages

### Receives

| ID | Name | Description |
|----|------|-------------|
| 0x110 | ORIN_HEARTBEAT | Orin alive (seq, state) - FAULT state triggers e-stop |
| 0x120 | CONTROL_HEARTBEAT | Control alive (seq, state, fault) - FAULT state triggers e-stop |

### Sends

| ID | Name | Rate | Description |
|----|------|------|-------------|
| 0x101 | SAFETY_AUTO_ALLOWED | On change + 500ms periodic when blocked | Permission broadcast |

### Heartbeat Monitoring

| Node | Timeout | Tracked Fields |
|------|---------|----------------|
| Orin | 500ms | sequence, state (FAULT triggers e-stop) |
| Control | 500ms | sequence, state (FAULT triggers e-stop) |

## E-stop Reasons (ESTOP_REASON_*)

| Code | Constant | Trigger |
|------|----------|---------|
| 0x00 | NONE | System OK |
| 0x01 | MUSHROOM | Push button HB2-ES544 opened (pressed) |
| 0x02 | REMOTE | RF remote EV1527 kill signal active |
| 0x03 | ULTRASONIC | Ultrasonic A02YYUW obstacle (<1000mm) or not responding |
| 0x04 | ORIN_ERROR | Orin heartbeat state == FAULT |
| 0x05 | ORIN_TIMEOUT | No Orin heartbeat for 500ms |
| 0x06 | CONTROL_TIMEOUT | No Control heartbeat for 500ms |
| 0x07 | CONTROL_ERROR | Control heartbeat state == FAULT |

## Autonomy Blocked Reasons (AUTO_BLOCKED_REASON_*)

| Code | Constant | Meaning |
|------|----------|---------|
| 0x00 | NONE | Autonomy allowed |
| 0x01 | ESTOP | E-stop active (see estop_reason field) |

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
| Green blink | Normal, autonomy allowed |
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

## Ultrasonic Sensor

- **Protocol**: 4-byte UART frames at 9600 baud
- **Frame format**: `0xFF | HIGH_BYTE | LOW_BYTE | CHECKSUM`
- **Checksum**: `(0xFF + HIGH + LOW) & 0xFF`
- **Stop threshold**: 1000mm (~3.3 ft)
- **Sample timeout**: 200ms (stale readings ignored)

## Build

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p PORT flash monitor
```
