# control-esp32

ESP-IDF firmware for the Control ESP32-C6. Receives commands from the Planner (Jetson AGX Orin) over CAN and controls throttle, steering, and braking actuators. Follows the system target state broadcast by Safety ESP32.

## State Machine

**Normal operation:** INIT -> READY -> ENABLING -> ACTIVE

**Driver takeover:** ACTIVE -> OVERRIDE -> READY -> (can re-enable)

Control follows the system target state from Safety's heartbeat (0x100). It transitions locally through ENABLING and ACTIVE, and can enter OVERRIDE or FAULT immediately for safety.

### States

| State | Description |
|-------|-------------|
| INIT | Hardware initializing. Transitions to READY immediately after boot. |
| READY | Manual mode. Cart drives normally via pedal. Waiting for Safety to advance. |
| ENABLING | 200ms transition. Enable relay energized, waiting to switch throttle source. Sets `HEARTBEAT_FLAG_ENABLE_COMPLETE` when done. |
| ACTIVE | Autonomous mode. Executing throttle/steering/braking commands from Planner. |
| OVERRIDE | Safe state after driver takeover. All actuators disabled, manual control restored. |
| FAULT | Hardware failure detected. Requires power cycle to clear. |

### Transitions

**READY -> ENABLING** requires ALL:
- Safety target state is ENABLING (from Safety heartbeat `state` field)
- F/R switch in Forward position
- Pedal not pressed
- Pedal re-armed (released for 500ms after any press)
- No active fault codes

**ENABLING -> ACTIVE** after 200ms settle time completes. Can abort back to READY if Safety retreats, pedal pressed, or F/R moved.

**ACTIVE -> OVERRIDE** triggered immediately by ANY:
- Pedal pressed (no debounce - immediate response)
- F/R switch moved from Forward (debounced)
- Safety target state retreated (e-stop or node timeout)

**OVERRIDE -> READY** auto-clears when ALL conditions met:
- Safety target state is ENABLING or higher
- F/R switch in Forward position
- Pedal re-armed (released for 500ms)

Note: The system automatically returns to READY and can re-enable autonomous mode when override conditions clear. This allows seamless recovery after brief driver interventions.

## CAN Messages

### Receives (Standard 11-bit Frames)

| ID | Name | Description |
|----|------|-------------|
| 0x100 | SAFETY_HEARTBEAT | System target state, e-stop fault code (same `node_heartbeat_t` format) |
| 0x111 | PLANNER_COMMAND | Commands (sequence, throttle 0-7, steering_pos, braking_pos) |

### Sends (Standard 11-bit Frames)

| ID | Name | Rate | Description |
|----|------|------|-------------|
| 0x120 | CONTROL_HEARTBEAT | 100ms + immediate on state change | Alive signal (seq, state, fault_code, flags) |

### Stale Planner Command Detection

Control tracks the Planner command sequence number. If the same sequence is seen 10 consecutive times (`PLANNER_CMD_STALE_COUNT`), Control zeros the throttle as a safety measure.

### UIM2852CA Motors (Extended 29-bit Frames)

| Motor | Node ID | Description |
|-------|---------|-------------|
| Steering | 5 | Linear actuator for steering column |
| Braking | 6 | Linear actuator for brake pedal |

Master controller ID: 4. See `stepper_protocol_uim2852.h` for CAN ID encoding.

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
| `stepper_motor_uim2852` | UIM2852CA closed-loop stepper motor control API |
| `enable_relay` | MOSFET driver for pedal microswitch bypass relay |
| `override_sensors` | Pedal ADC reading + F/R optocoupler debouncing |
| `can_twai` | CAN bus driver wrapper (shared) |
| `can_protocol` | Message definitions and encode/decode (shared) |
| `heartbeat` | WS2812 status LED driver (shared) |
| `heartbeat_monitor` | CAN node liveness tracking (shared) |
| `stepper_protocol_uim2852` | UIM2852 SimpleCAN protocol library (shared) |
| `control_logic` | Extracted state machine decision logic (shared, tested) |
| `debug_console` | Interactive UART REPL for bench testing (Kconfig-gated, off by default) |

## Throttle Control

The throttle system uses an 8-channel analog multiplexer to select from 8 resistor taps:
- Level 0: Idle (minimum throttle)
- Level 7: Maximum throttle
- Slew rate limited: max 1 level change per 100ms

Enable sequence (READY -> ACTIVE):
1. Set mux to level 0
2. Energize enable relay (GPIO10) - bypasses pedal microswitch
3. Wait 200ms for Curtis controller to recognize
4. Energize throttle relay (GPIO9) - switches to mux output
5. Enable steering and braking motors

## Debug Console

An optional interactive UART console for bench testing without the full system connected. Disabled by default. See [common/debug_console/README.md](../common/debug_console/README.md) for details.

Commands: `sim fr forward|neutral|reverse|off`, `sim auto 0|1|off`, `sim planner <thr> <steer> <brake>`, `sim planner stop`, `sim off`, `status`

Enable with `CONFIG_ENABLE_DEBUG_CONSOLE=y` in sdkconfig.defaults.

## Build

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p PORT flash monitor
```
