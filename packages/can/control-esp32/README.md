# control-esp32

ESP-IDF firmware for the Control ESP32-C6. Receives commands from the Jetson Orin over CAN and controls throttle, steering, and braking actuators.

## State Machine

**Normal operation:** INIT → READY → ENABLING → ACTIVE

**Driver takeover:** ACTIVE → OVERRIDE → READY → (can re-enable)

### States

| State | Description |
|-------|-------------|
| INIT | Hardware initializing. Transitions to READY immediately after boot. |
| READY | Manual mode. Cart drives normally via pedal. Waiting for Safety permission. |
| ENABLING | 200ms transition. Enable relay energized, waiting to switch throttle source. |
| ACTIVE | Autonomous mode. Executing throttle/steering/braking commands from Orin. |
| OVERRIDE | Safe state after driver takeover. All actuators disabled, manual control restored. |
| FAULT | Hardware failure detected. Requires power cycle to clear. |

### Transitions

**READY → ENABLING** requires ALL:
- Safety grants permission (`CAN_ID_SAFETY_AUTO_ALLOWED` with `allowed=1`)
- F/R switch in Forward position
- Pedal not pressed
- Pedal re-armed (released for 500ms after any press)
- No active fault codes

**ENABLING → ACTIVE** after 200ms settle time completes. Can abort back to READY if Safety revokes permission, pedal pressed, or F/R moved.

**ACTIVE → OVERRIDE** triggered immediately by ANY:
- Pedal pressed (no debounce - immediate response)
- F/R switch moved from Forward (debounced)
- Safety revokes permission (e-stop or node timeout)

**OVERRIDE → READY** auto-clears when ALL conditions met:
- Safety allows autonomous
- F/R switch in Forward position
- Pedal re-armed (released for 500ms)

Note: The system automatically returns to READY and can re-enable autonomous mode when override conditions clear. This allows seamless recovery after brief driver interventions.

## CAN Messages

### Receives (Standard 11-bit Frames)

| ID | Name | Description |
|----|------|-------------|
| 0x101 | SAFETY_AUTO_ALLOWED | Autonomy permission (allowed, block_reason, estop_reason) |
| 0x111 | ORIN_COMMAND | Commands (throttle 0-7, steering_pos, braking_pos, seq) |

### Sends (Standard 11-bit Frames)

| ID | Name | Rate | Description |
|----|------|------|-------------|
| 0x120 | CONTROL_HEARTBEAT | 100ms | Alive signal (seq, state, fault_code) |
| 0x121 | CONTROL_STATUS | 100ms | Throttle, F/R, sensors, override_reason, positions |

### UIM2852CA Motors (Extended 29-bit Frames)

| Motor | Node ID | Description |
|-------|---------|-------------|
| Steering | 5 | Linear actuator for steering column |
| Braking | 6 | Linear actuator for brake pedal |

Master controller ID: 4. See `uim2852_protocol.h` for CAN ID encoding.

## Pin Configuration

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| 0 | Pedal ADC | Analog In | ADC1_CH0, voltage divider (270k/150k), threshold 360mV |
| 2 | Throttle Mux A0 | Output | DG408 address bit 0 (LSB) |
| 3 | Throttle Mux A1 | Output | DG408 address bit 1 |
| 4 | CAN TX | Output | TWAI peripheral |
| 5 | CAN RX | Input | TWAI peripheral |
| 6 | Throttle Mux A2 | Output | DG408 address bit 2 (MSB) |
| 7 | Throttle Mux EN | Output | DG408 enable (10k pull-down) |
| 8 | Status LED | Output | WS2812 RGB LED |
| 9 | Throttle Relay | Output | AEDIKO relay module (NO=autonomous) |
| 10 | Enable MOSFET | Output | IRLZ44N gate (10k pull-down), bypasses pedal microswitch |
| 14 | F/R Direction | Input | PC817 optocoupler, pull-up, active LOW |
| 15 | F/R Reverse | Input | PC817 optocoupler, pull-up, active LOW |

### F/R Optocoupler Logic

| GPIO14 (Dir) | GPIO15 (Rev) | State |
|--------------|--------------|-------|
| LOW | HIGH | Forward |
| LOW | LOW | Reverse |
| HIGH | HIGH | Neutral |
| HIGH | LOW | Invalid (fault) |

## Components

| Component | Description |
|-----------|-------------|
| `throttle_mux` | DG408 8-channel mux + DPDT throttle relay control |
| `uim2852_motor` | UIM2852CA servo stepper motor control API |
| `enable_relay` | MOSFET driver for pedal microswitch bypass relay |
| `override_sensors` | Pedal ADC reading + F/R optocoupler debouncing |
| `can_twai` | CAN bus driver wrapper (shared) |
| `can_protocol` | Message definitions and encode/decode (shared) |
| `heartbeat` | WS2812 status LED driver (shared) |

## Throttle Control

The throttle system uses an 8-channel analog multiplexer to select from 8 resistor taps:
- Level 0: Idle (minimum throttle)
- Level 7: Maximum throttle
- Slew rate limited: max 1 level change per 100ms

Enable sequence (READY → ACTIVE):
1. Set mux to level 0
2. Energize enable relay (GPIO10) - bypasses pedal microswitch
3. Wait 200ms for Curtis controller to recognize
4. Energize throttle relay (GPIO9) - switches to mux output
5. Enable steering and braking motors

## Build

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p PORT flash monitor
```
