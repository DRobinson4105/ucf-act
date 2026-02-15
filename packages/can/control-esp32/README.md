# control-esp32

ESP-IDF firmware for the Control ESP32-C6. Receives commands from the Planner (Jetson AGX Orin) over CAN and controls throttle, steering, and braking actuators. Follows the system target state broadcast by Safety ESP32.

## State Machine

Control follows Safety's target state (`READY`/`ENABLING`/`ACTIVE`) but owns its own live state machine:

```text
INIT -> READY -> ENABLING -> ACTIVE
 V      ^ ^ V       V        V   V
 |      | | |       |        |   |
 +------|-+-+---- FAULT -----+-<-+
        +------- OVERRIDE -------+
```

- Any live state can enter `FAULT`.
- `FAULT` is independent from `OVERRIDE` and returns to `READY` when fault conditions clear.
- `OVERRIDE` is a separate retreat path and returns to `READY` when override conditions clear.
- `OVERRIDE` can still enter `FAULT` if a component/runtime fault is detected.

### States

| State | Description |
|-------|-------------|
| INIT | Hardware initializing. Transitions to READY immediately after boot. |
| READY | Manual mode. Cart drives normally via manual controls. Waiting for Safety to advance. |
| ENABLING | Enable relay energized, waiting to switch throttle source. Sets `HEARTBEAT_FLAG_ENABLE_COMPLETE` when done. |
| ACTIVE | Autonomous mode. Executing throttle/steering/braking commands from Planner. |
| OVERRIDE | Safe state after driver takeover. All actuators disabled, manual control restored. Separate from FAULT. |
| FAULT | Faulted state. Entered from any state on local component/runtime fault; returns to READY once faults clear. |

### Transitions

**READY -> ENABLING** requires ALL:
- Safety target state is ENABLING or ACTIVE (from Safety heartbeat `state` field)
- F/R switch in Forward position
- Pedal not pressed
- Pedal re-armed (released for 500ms after any press)
- No active fault codes

**ENABLING -> ACTIVE** Safety target reaches ACTIVE. Can abort back to READY if Safety retreats, pedal pressed, or F/R moved.

**ACTIVE -> OVERRIDE** triggered immediately by ANY:
- Pedal press (no debounce - immediate response)
- F/R switch moved from Forward (debounced)
- Steering position error (actual encoder diverged >200 pulses from commanded target after motion complete)
- Braking position error (actual encoder diverged >200 pulses from commanded target after motion complete)

**ACTIVE -> READY** when Safety target state retreats (e-stop, node fault/override/timeout, or Planner/Orin halt autonomy command).
This is a commanded retreat from Safety, not a human override event.

**ANY -> FAULT** when a local component/runtime fault is detected.

**FAULT -> READY** when local fault conditions are cleared and required components recover.

**OVERRIDE -> READY** auto-clears when ALL conditions met:
- Safety target state is READY or higher
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
| 14 | F/R Forward | Input | PC817 optocoupler, pull-up, active LOW |
| 15 | F/R Reverse | Input | PC817 optocoupler, pull-up, active LOW |

### LED Behavior

| Color | State |
|-------|-------|
| Green blink | Local Control state READY |
| Orange blink | Local Control state ENABLING |
| Blue blink | Local Control state ACTIVE |
| Red blink | Non-nominal local state (OVERRIDE or FAULT) |

### F/R Optocoupler Logic

| GPIO14 (Forward) | GPIO15 (Reverse) | State |
|--------------|--------------|-------|
| LOW | HIGH | Forward |
| HIGH | LOW | Reverse |
| HIGH | HIGH | Neutral |
| LOW | LOW | Invalid (fault) |

## Components

| Component | Description |
|-----------|-------------|
| `multiplexer_dg408djz` | DG408DJZ 8-channel analog mux + DPDT throttle relay control |
| `stepper_motor_uim2852` | UIM2852CA closed-loop stepper motor control API |
| `relay_jd2912` | JD-2912 pedal bypass relay driver (via IRLZ44N MOSFET) |
| `adc_12bitsar` | Dedicated ESP32-C6 12-bit SAR ADC read/calibration helper |
| `optocoupler_pc817` | Dedicated F/R PC817 decode + debounce helper |
| `can_twai` | CAN bus driver wrapper (shared) |
| `can_protocol` | Message definitions and encode/decode (shared) |
| `led_ws2812` | WS2812 status LED driver (shared) |
| `stepper_protocol_uim2852` | UIM2852 SimpleCAN protocol library (shared) |
| `control_logic` | Extracted state machine decision logic (shared, tested) |

Driver-input policy (pedal re-arm + F/R gating) now lives directly in `control-esp32/main/main.cpp`
and consumes `adc_12bitsar` + `optocoupler_pc817` as low-level hardware components.

### Hardware Detection Limitations

Not all components can detect physical absence at init or runtime. Components that are output-only GPIOs have no feedback path from the external hardware -- the ESP32 drives pins but receives no acknowledgment.

| Component | Detects absence? | Why |
|-----------|-----------------|-----|
| `multiplexer_dg408djz` | No | Output-only GPIO. Init readback tests the MCU output latch, not the external IC. The DG408DJZ is a purely analog device with no feedback path. |
| `relay_jd2912` | No | Output-only GPIO. Same readback pattern as the mux -- verifies the MCU register, not whether the relay/MOSFET is physically present. |
| `adc_12bitsar` | No | ADC init configures an internal ESP32 peripheral. A floating/disconnected pin reads ~0 mV, which is indistinguishable from "pedal not pressed." Safe direction (override never triggers), but pedal override detection is silently disabled. |
| `optocoupler_pc817` | **Yes** | Pull-ups + active-low signaling: disconnected = both HIGH = NEUTRAL. `init_fr_inputs()` rejects NEUTRAL as invalid, triggering FAULT. |
| `stepper_motor_uim2852` | **Yes** | Init performs a CAN handshake (query status). No response = timeout = init failure, triggering FAULT. |

For the mux and relay, detecting hardware absence would require board-level changes (e.g., adding a sense/feedback line). For the pedal ADC, a plausibility range check on the idle reading could improve detection but is not yet implemented.

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
| `CONFIG_BYPASS_SAFETY_TARGET_MIRROR` | Force target_state to ACTIVE (ignore Safety target mirror) |
| `CONFIG_BYPASS_SAFETY_ESTOP_MIRROR` | Force Safety estop_fault_code to NONE in Control inputs/logging |
| `CONFIG_BYPASS_PLANNER_COMMAND_INPUTS` | Force Planner command inputs to zero throttle/steering/braking |
| `CONFIG_BYPASS_PLANNER_COMMAND_STALE_CHECKS` | Disable Planner command timeout/stale checks |
| `CONFIG_BYPASS_INPUT_PEDAL_ADC` | Skip pedal ADC readings (always not pressed, always re-armed) |
| `CONFIG_BYPASS_INPUT_FR_SENSOR` | Force F/R state to FORWARD (skip optocoupler channel reading) |
| `CONFIG_BYPASS_ACTUATOR_MULTIPLEXER` | Skip multiplexer and throttle relay control |
| `CONFIG_BYPASS_ACTUATOR_PEDAL_RELAY` | Skip pedal bypass relay energize/de-energize |
| `CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING` | Skip steering stepper motor (node 5) init/configure/commands |
| `CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING` | Skip braking stepper motor (node 6) init/configure/commands |

## Debug Logging

Compile-time Kconfig flags for verbose logging. Enable via `idf.py menuconfig` under **Debug Logging** (top-level):

### CAN Traffic & Planner I/O

| Flag | Default | Effect |
|------|---------|--------|
| `CONFIG_LOG_CAN_HEARTBEAT_TX` | off | Log every periodic heartbeat TX (very verbose) |
| `CONFIG_LOG_CAN_HEARTBEAT_RX` | off | Log every received Safety heartbeat, not just changes |
| `CONFIG_LOG_CAN_PLANNER_COMMAND_RX` | off | Log every Planner command RX + timeout/stale warnings |

### Component Health & Recovery

| Flag | Default | Effect |
|------|---------|--------|
| `CONFIG_LOG_COMPONENT_LOST` | on | Log component LOST edge transitions |
| `CONFIG_LOG_COMPONENT_REGAINED` | on | Log component REGAINED edge transitions |
| `CONFIG_LOG_CAN_RECOVERY` | off | Log CAN bus recovery events (stop/start, reinstall, bus-off) |
| `CONFIG_LOG_RETRY_TWAI` | off | Log repeated TWAI retry attempts (startup and runtime faults) |
| `CONFIG_LOG_RETRY_MULTIPLEXER` | off | Log multiplexer retry attempts while faulted |
| `CONFIG_LOG_RETRY_PEDAL_RELAY` | off | Log pedal relay retry attempts while faulted |
| `CONFIG_LOG_RETRY_PEDAL_INPUT` | off | Log pedal ADC retry attempts while faulted |
| `CONFIG_LOG_RETRY_FR_INPUT` | off | Log F/R input retry attempts while faulted |
| `CONFIG_LOG_RETRY_STEPPER_STEERING` | off | Log steering stepper retry attempts |
| `CONFIG_LOG_RETRY_STEPPER_BRAKING` | off | Log braking stepper retry attempts |

Retries are unbounded for failed required components, paced at 500ms intervals (no maximum attempt limit). HEARTBEAT_LED is non-critical for FAULT gating and is initialized once at startup.

### Control Logic

| Flag | Default | Effect |
|------|---------|--------|
| `CONFIG_LOG_CONTROL_STATE_CHANGES` | on | Log control state transitions and transition reasons |
| `CONFIG_LOG_CONTROL_FAULT_CHANGES` | off | Log control fault code changes |
| `CONFIG_LOG_CONTROL_STATE_TICK` | off | Log state-machine evaluation every 20ms cycle (very verbose) |
| `CONFIG_LOG_CONTROL_ENABLE_SEQUENCE` | off | Log enable/disable sequence steps (start, complete, abort) |
| `CONFIG_LOG_CONTROL_OVERRIDE` | off | Log driver override trigger events (pedal/F/R) |
| `CONFIG_LOG_CONTROL_THROTTLE_CHANGES` | off | Log throttle level changes |
| `CONFIG_LOG_CONTROL_THROTTLE_TICK` | off | Log throttle current/target every 20ms cycle (very verbose) |
| `CONFIG_LOG_CONTROL_SAFETY_MIRROR_CHANGES` | off | Log mirrored Safety target/fault changes seen by Control |

### Inputs

| Flag | Default | Effect |
|------|---------|--------|
| `CONFIG_LOG_INPUT_PEDAL_ADC` | off | Log pedal ADC millivolt readings (extremely verbose) |
| `CONFIG_LOG_INPUT_FR_DEBOUNCE` | off | Log F/R switch debounce transitions |
| `CONFIG_LOG_INPUT_FR_STATE` | off | Log F/R selector state changes |

### Actuators

| Flag | Default | Effect |
|------|---------|--------|
| `CONFIG_LOG_ACTUATOR_MUX_LEVEL` | off | Log multiplexer level changes with A2/A1/A0 values |
| `CONFIG_LOG_ACTUATOR_PEDAL_RELAY` | off | Log pedal bypass relay energize/de-energize |
| `CONFIG_LOG_ACTUATOR_STEPPER_COMMAND_TX` | off | Log stepper position commands sent over CAN |
| `CONFIG_LOG_ACTUATOR_STEPPER_MOTION_TX` | off | Log stepper motion commands (PA/PR/ST) |
| `CONFIG_LOG_ACTUATOR_STEPPER_RX` | off | Log parsed stepper CAN RX frames (MS, params, ACKs) |

### HEARTBEAT_LED

| Flag | Default | Effect |
|------|---------|--------|
| `CONFIG_LOG_HEARTBEAT_LED_STATE_CHANGES` | off | Log HEARTBEAT_LED color/reason mode changes |
| `CONFIG_LOG_HEARTBEAT_LED_BLINK_PULSES` | off | Log every HEARTBEAT_LED ON pulse with color/reason (very verbose) |

## Build

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p PORT flash monitor
```
