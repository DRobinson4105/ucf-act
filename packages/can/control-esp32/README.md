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
| FAULT | Hardware failure detected. Recoverable via cooldown + re-init; esp_restart() after max attempts. |

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

**ACTIVE -> READY** when Safety target state retreats (e-stop or node timeout).
This is a commanded retreat from Safety, not a human override event.

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

### LED Behavior

| Color | State |
|-------|-------|
| Green blink | Local Control state READY |
| Orange blink | Local Control state ENABLING |
| Blue blink | Local Control state ACTIVE |
| Red blink | Non-nominal local state (OVERRIDE or FAULT) |

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
| `multiplexer_dg408djz` | DG408DJZ 8-channel analog mux + DPDT throttle relay control |
| `stepper_motor_uim2852` | UIM2852CA closed-loop stepper motor control API |
| `relay_jd2912` | JD-2912 pedal bypass relay driver (via IRLZ44N MOSFET) |
| `override_sensors` | Pedal ADC reading + F/R optocoupler debouncing |
| `can_twai` | CAN bus driver wrapper (shared) |
| `can_protocol` | Message definitions and encode/decode (shared) |
| `heartbeat` | WS2812 status LED driver (shared) |
| `stepper_protocol_uim2852` | UIM2852 SimpleCAN protocol library (shared) |
| `control_logic` | Extracted state machine decision logic (shared, tested) |

## Throttle Control

The throttle system uses an 8-channel analog multiplexer to select from 8 resistor taps:
- Level 0: Idle (minimum throttle)
- Level 7: Maximum throttle
- Slew rate limited: max 1 level change per 100ms

Enable sequence (READY -> ACTIVE):
1. Set mux to level 0
2. Energize pedal bypass relay (GPIO10) - JD-2912 bypasses pedal microswitch
3. Wait 200ms for Curtis controller to recognize
4. Energize throttle relay (GPIO9) - switches to mux output
5. Enable steering and braking motors

## Test Bypasses

Compile-time Kconfig flags for bench testing without the full system connected. All default to off (disabled). Enable via `idf.py menuconfig` under **Test Bypasses** (top-level), or add to `sdkconfig.defaults`:

| Flag | Effect |
|------|--------|
| `CONFIG_BYPASS_SAFETY_TARGET_STATE` | Force target_state to ACTIVE (ignore Safety target state) |
| `CONFIG_BYPASS_SAFETY_ESTOP_FAULT` | Force Safety estop_fault_code to NONE in Control inputs/logging |
| `CONFIG_BYPASS_FR_SENSOR` | Force F/R state to Forward (skip optocoupler reading) |
| `CONFIG_BYPASS_PLANNER_CMD_INPUTS` | Force Planner command inputs to zero throttle/steering/braking |
| `CONFIG_BYPASS_PLANNER_CMD_STALE_CHECK` | Disable Planner command timeout/stale checks |
| `CONFIG_BYPASS_STEPPER_MOTORS` | Skip stepper motor init/configure/commands (no UIM2852CA needed) |
| `CONFIG_BYPASS_PEDAL_OVERRIDE` | Ignore pedal ADC (always not pressed, always re-armed) |
| `CONFIG_BYPASS_ENABLE_RELAY` | Skip JD-2912 pedal bypass relay energize/de-energize |
| `CONFIG_BYPASS_MULTIPLEXER` | Skip DG408DJZ mux and throttle relay control |

## Debug Logging

Compile-time Kconfig flags for verbose logging. Enable via `idf.py menuconfig` under **Debug Logging** (top-level):

### CAN Bus

| Flag | Default | Effect |
|------|---------|--------|
| `CONFIG_LOG_HEARTBEAT_TX` | off | Log every periodic heartbeat TX (very verbose) |
| `CONFIG_LOG_HEARTBEAT_RX` | off | Log every received Safety heartbeat, not just changes |
| `CONFIG_LOG_PLANNER_COMMANDS` | off | Log every Planner command RX + timeout/stale warnings |
| `CONFIG_LOG_CAN_RECOVERY` | off | Log CAN bus recovery events (stop/start, reinstall, bus-off) |

### Control Logic

| Flag | Default | Effect |
|------|---------|--------|
| `CONFIG_LOG_STATE_CHANGES` | off | Log control state/fault changes and transition reasons |
| `CONFIG_LOG_SAFETY_MIRROR_CHANGES` | off | Log mirrored Safety target/fault changes seen by Control |
| `CONFIG_LOG_STATE_TICK` | off | Log state-machine evaluation every 20ms cycle (very verbose) |
| `CONFIG_LOG_ENABLE_SEQUENCE` | off | Log enable/disable sequence steps (start, complete, abort) |
| `CONFIG_LOG_RECOVERY` | off | Log fault recovery attempts (re-init, success/failure) |
| `CONFIG_LOG_THROTTLE` | off | Log throttle level changes |
| `CONFIG_LOG_THROTTLE_TICK` | off | Log throttle current/target every 20ms cycle (very verbose) |
| `CONFIG_LOG_STEPPER_COMMANDS` | off | Log stepper motor position commands |
| `CONFIG_LOG_OVERRIDE` | off | Log driver override trigger events (pedal/FR) |

### LED (WS2812)

| Flag | Default | Effect |
|------|---------|--------|
| `CONFIG_LOG_LED_STATE_CHANGES` | off | Log LED color/reason mode changes |
| `CONFIG_LOG_LED_BLINKS` | off | Log every LED ON blink with color/reason (very verbose) |

### Multiplexer (DG408DJZ)

| Flag | Default | Effect |
|------|---------|--------|
| `CONFIG_LOG_MUX_LEVEL` | off | Log mux level changes with A2/A1/A0 values |

### Pedal Bypass Relay (JD-2912)

| Flag | Default | Effect |
|------|---------|--------|
| `CONFIG_LOG_PEDAL_RELAY` | **on** | Log pedal bypass relay energize/de-energize |

### Override Sensors

| Flag | Default | Effect |
|------|---------|--------|
| `CONFIG_LOG_PEDAL_ADC` | off | Log pedal ADC millivolt readings (extremely verbose) |
| `CONFIG_LOG_FR_DEBOUNCE` | off | Log F/R switch debounce transitions |
| `CONFIG_LOG_FR_STATE` | **on** | Log F/R gear selector state changes |

### Stepper Motor (UIM2852)

| Flag | Default | Effect |
|------|---------|--------|
| `CONFIG_LOG_STEPPER_MOTION` | off | Log stepper motion commands (PA/PR/ST) |
| `CONFIG_LOG_STEPPER_RX` | off | Log stepper CAN RX frame details (MS, params, ACKs) |

## Build

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p PORT flash monitor
```
