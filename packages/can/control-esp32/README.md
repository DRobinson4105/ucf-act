# control-esp32

ESP-IDF firmware for the Control ESP32-C6. Receives commands from the Planner (Jetson AGX Orin) over CAN and controls/monitors throttle, steering, and braking actuators. Follows the system target state broadcast by Safety ESP32.

## State Machine

Control follows Safety's target state (`NOT_READY`/`READY`/`ENABLE`/`ACTIVE`) but owns its own live state machine.

High-level rules:
- `READY` means local preconditions are currently satisfied.
- `NOT_READY` means local preconditions are not satisfied (not a fault).
- `FAULT` means a component/runtime error condition; it is distinct from `NOT_READY`.
- `OVERRIDE` is a human-intervention retreat path, separate from `FAULT`.
- `FAULT` and `OVERRIDE` both recover to `NOT_READY` or `READY` based on current preconditions.

### States

| State     | Description                                                                                                           |
|-----------|-----------------------------------------------------------------------------------------------------------------------|
| INIT      | Hardware initializing.                                                                                                |
| NOT_READY | Manual mode, but local autonomy preconditions are not satisfied.                                                      |
| READY     | Manual mode with local autonomy preconditions satisfied. Waiting for Safety to advance.                               |
| ENABLE    | Enable relay energized, waiting to switch throttle source. Sets `HEARTBEAT_FLAG_ENABLE_COMPLETE` when done.           |
| ACTIVE    | Autonomous mode. Executing throttle/steering/braking commands from Planner.                                           |
| OVERRIDE  | Safe state after driver takeover. All actuators disabled, manual control restored. Separate from FAULT.               |
| FAULT     | Faulted state. Entered from any state on local component/runtime fault; returns to NOT_READY/READY once faults clear. |

### Transitions

**`INIT -> NOT_READY/READY`** occurs after init dwell. If local preconditions pass at that moment, transition to `READY`; otherwise transition to `NOT_READY`.

**`NOT_READY <-> READY`** is driven by local preconditions:
- F/R switch in Forward position
- Pedal not pressed
- Pedal re-armed (released for 500ms after any press)
- No active fault codes

**`READY -> ENABLE`** requires all:
- Safety target state is ENABLE or ACTIVE (from Safety heartbeat `state` field)
- F/R switch in Forward position
- Pedal not pressed
- Pedal re-armed (released for 500ms after any press)
- No active fault codes

**`ENABLE -> ACTIVE`** advances when Safety target reaches `ACTIVE`. It can abort back to `NOT_READY`/`READY` if Safety retreats, pedal is pressed, or F/R moves.

**`ACTIVE -> OVERRIDE`** is triggered immediately by any:
- Pedal press (no debounce - immediate response)
- F/R switch moved from Forward (debounced)
- Steering position error (actual encoder diverged >200 pulses from commanded target after motion complete)
- Braking position error (actual encoder diverged >200 pulses from commanded target after motion complete)

**`ACTIVE -> NOT_READY/READY`** occurs when Safety target retreats (e-stop, node fault/override/timeout, or Planner/Orin autonomy halt).
This is a commanded retreat from Safety, not a human override event.

**`ANY -> FAULT`** occurs when a local component/runtime fault is detected.

**`FAULT -> NOT_READY/READY`** occurs when local fault conditions clear and required components recover. Recovery target is recomputed from current preconditions.

**`OVERRIDE -> NOT_READY/READY`** auto-clears when all conditions are met:
- F/R switch in Forward position
- Pedal re-armed (released for 500ms)

Recovery target is recomputed from current preconditions.

Note: The system automatically returns to NOT_READY/READY based on current preconditions and can re-enable autonomous mode when conditions clear.

## CAN Messages

### Receives (Standard 11-bit Frames)

| ID    | Name             | Description                                                             |
|-------|------------------|-------------------------------------------------------------------------|
| 0x100 | SAFETY_HEARTBEAT | System target state, e-stop fault code (same `node_heartbeat_t` format) |
| 0x111 | PLANNER_COMMAND  | Commands (sequence, throttle 0-7, steering_pos, braking_pos)            |

### Sends (Standard 11-bit Frames)

| ID    | Name              | Rate                              | Description                                  |
|-------|-------------------|-----------------------------------|----------------------------------------------|
| 0x120 | CONTROL_HEARTBEAT | 100ms + immediate on state change | Alive signal (seq, state, fault_code, flags) |

### Stale Planner Command Detection

Control tracks the Planner command sequence number. If the same sequence is seen 10 consecutive times (`PLANNER_CMD_STALE_COUNT`), Control zeros the throttle as a safety measure.

### UIM2852CA Motors (Extended 29-bit Frames)

| Motor    | Node ID | Description                         |
|----------|---------|-------------------------------------|
| Steering | 5       | Linear actuator for steering column |
| Braking  | 6       | Linear actuator for brake pedal     |

Master controller ID: 4. See `stepper_protocol_uim2852.h` for CAN ID encoding.

## Pin Configuration

| GPIO | Function        | Direction | Notes                                                    |
|------|-----------------|-----------|----------------------------------------------------------|
| 0    | Pedal ADC       | Analog In | ADC1_CH0, voltage divider (270k/150k), threshold 360mV   |
| 2    | Throttle Mux A0 | Output    | DG408 address bit 0 (LSB)                                |
| 3    | Throttle Mux A1 | Output    | DG408 address bit 1                                      |
| 4    | CAN TX          | Output    | TWAI peripheral (SN65HVD230 transceiver)                 |
| 5    | CAN RX          | Input     | TWAI peripheral (SN65HVD230 transceiver)                 |
| 6    | Throttle Mux A2 | Output    | DG408 address bit 2 (MSB)                                |
| 7    | Throttle Mux EN | Output    | DG408 enable (10k pull-down)                             |
| 8    | Status LED      | Output    | WS2812 RGB LED                                           |
| 9    | Throttle Relay  | Output    | AEDIKO relay module (NO=autonomous)                      |
| 10   | Enable MOSFET   | Output    | IRLZ44N gate (10k pull-down), bypasses pedal microswitch |
| 22   | F/R Forward     | Input     | PC817 optocoupler, pull-up, active LOW                   |
| 23   | F/R Reverse     | Input     | PC817 optocoupler, pull-up, active LOW                   |

### LED Behavior

| Color        | State                                       |
|--------------|---------------------------------------------|
| Solid green  | Local Control state READY                   |
| Solid orange | Local Control state ENABLE                  |
| Solid blue   | Local Control state ACTIVE                  |
| Solid red    | Non-nominal local state (OVERRIDE or FAULT) |

## Wiring

Each subsection covers production (on-cart) and bench wiring for Control ESP32 interfaces where applicable. F/R hardware wiring in this revision is cart-only; on bench, use the F/R input bypass flag if cart switch wiring is not present. For CAN bus topology and stepper motor wiring shared across all nodes, see [CAN Bus Wiring](../README.md#can-bus-wiring).

### Wiring Summary

| Interface                                  | Bench vs Production                                               |
|--------------------------------------------|-------------------------------------------------------------------|
| CAN bus (SN65HVD230)                       | Same — see [root README](../README.md#can-bus-wiring)             |
| Throttle system (DG408 mux + AEDIKO relay) | Same hardware — bench output unloaded (no Curtis controller)      |
| Pedal bypass relay (JD-2912 via IRLZ44N)   | Same hardware — bench relay switches with no 48V load             |
| Pedal ADC                                  | Same — bench divider floating reads ~0mV (safe default)           |
| F/R optocouplers (PC817)                   | Cart wiring only — production-style 48V switch wiring via 4.7k    |
| Status LED (WS2812)                        | Same                                                              |

### CAN Bus (SN65HVD230)

GPIO 4 (TX) and GPIO 5 (RX) connect to a WAVESHARE SN65HVD230 CAN transceiver module. Control ESP32's Waveshare board has the onboard termination resistor **removed** (termination is on Safety ESP32 and Planner/Orin). See the [root README](../README.md#can-bus-wiring) for the full 5-node bus topology including stepper motors.

### Throttle System (DG408DJZ Mux + AEDIKO Relay)

8-channel analog multiplexer selects throttle levels 0-7 using this resistor ladder:

| Resistor # | Resistance (Ohms) |
|------------|-------------------|
| 1          | 910               |
| 2          | 750               |
| 3          | 910               |
| 4          | 1000              |
| 5          | 1000              |
| 6          | 1000              |

The six series resistors create seven physical ladder taps (`T0..T6`). Mux channels are mapped `CH0..CH6 -> T0..T6`, and `CH7` is intentionally duplicated to `T6` (max throttle).

Address lines A0-A2 (GPIO 2/3/6) select the channel; EN (GPIO 7) gates the output. An AEDIKO SRD-05VDC-SL-C relay (GPIO 9) switches the Curtis controller throttle input between manual pedal pot (NC, de-energized) and mux output (NO, energized). EN has a 10k pull-down to ensure the mux is disabled on reset.

**Production:** Mux output feeds into the Curtis motor controller throttle input through the relay. The ladder taps and channel mapping are tuned to the Curtis controller's expected voltage range.

**Bench:** Same DG408 + resistor ladder + relay hardware. The mux output is unloaded (no Curtis controller connected). Useful for verifying channel selection with a DMM on the mux output. The relay will audibly click when energized — verifies GPIO 9 output.

### Pedal Bypass Relay (JD-2912 via IRLZ44N)

ESP32 GPIO 10 drives an IRLZ44N N-channel MOSFET gate, which switches the JD-2912 automotive relay coil. The relay bypasses the accelerator pedal microswitch so the Curtis controller accepts throttle input during autonomous mode. 10k pull-down on the MOSFET gate ensures the relay stays de-energized (manual pedal mode) on boot/reset.

**Production:** MOSFET switches 12V to the JD-2912 coil. Relay contacts bypass the pedal microswitch in the cart's 48V throttle circuit.

**Bench:** Same hardware. MOSFET/relay tested with 12V bench supply — relay clicks to verify switching. Without 12V, the GPIO output can be verified with a DMM on the MOSFET gate.

### Pedal ADC

GPIO 0 (ADC1_CH0) reads the accelerator pedal position through a voltage divider (270k/150k). Threshold for "pressed" is 360 mV.

**Production:** Voltage divider connected to the cart's accelerator pedal potentiometer wiper. Pedal pressed produces >360 mV; released reads ~0 mV.

**Bench:** Same voltage divider. With no pedal connected, the floating input reads ~0 mV (treated as "not pressed" — safe default).

### F/R Optocouplers (PC817)

The F/R decode is wired for a **1999 Club Car DS 48V** forward/reverse switch assembly. The switch has three microswitches; two are used for direction sensing:

| Microswitch    | GPIO              | Active When                                                     |
|----------------|-------------------|-----------------------------------------------------------------|
| Anti-arcing    | 22 (forward_gpio) | Forward **and** Reverse (opens solenoid coil during transition) |
| Reverse buzzer | 23 (reverse_gpio) | Reverse only (activates backup buzzer)                          |

State decode (PC817 optocouplers, active LOW with internal pull-up):

| GPIO 22 (Anti-arc) | GPIO 23 (Buzzer) | State           | Meaning                                |
|--------------------|------------------|-----------------|----------------------------------------|
| LOW                | HIGH             | Forward         | Anti-arc ON, buzzer OFF                |
| LOW                | LOW              | Reverse         | Both ON                                |
| HIGH               | HIGH             | Neutral         | Neither ON (handle in center detent)   |
| HIGH               | LOW              | Invalid (fault) | Buzzer without anti-arc = wiring fault |

Each PC817 provides galvanic isolation between the cart's 48V signal circuits and the ESP32:

**Production (48V):** 4.7k ohm current-limiting resistor on pin 1 for each F/R channel on the **PC817 LED side**. Keep the cart-side F/R switch wiring isolated from the ESP32 GPIO side (do not tie the cart-side LED return for these channels to ESP32 GND). Microswitch closed -> LED on -> phototransistor conducts -> GPIO pulled LOW (active). Microswitch open -> LED off -> phototransistor off -> GPIO pulled HIGH by internal pull-up.

**Bench:** If cart F/R switch wiring is absent on bench, use `CONFIG_BYPASS_INPUT_FR_SENSOR` to force Forward.

### Status LED (WS2812)

GPIO 8 drives the data input of the onboard WS2812 RGB status LED (no external wiring required).

## Components

| Component                  | Description                                                |
|----------------------------|------------------------------------------------------------|
| `multiplexer_dg408djz`     | DG408DJZ 8-channel analog mux for throttle level selection |
| `stepper_motor_uim2852`    | UIM2852CA closed-loop stepper motor control API            |
| `relay_jd2912`             | JD-2912 pedal bypass relay driver (via IRLZ44N MOSFET)     |
| `adc_12bitsar`             | Dedicated ESP32-C6 12-bit SAR ADC read/calibration helper  |
| `optocoupler_pc817`        | Dedicated F/R PC817 decode + debounce helper               |
| `can_twai`                 | CAN bus driver wrapper (shared)                            |
| `can_protocol`             | Message definitions and encode/decode (shared)             |
| `led_ws2812`               | WS2812 status LED driver (shared)                          |
| `stepper_protocol_uim2852` | UIM2852 SimpleCAN protocol library (shared)                |
| `control_logic`            | Extracted state machine decision logic (shared, tested)    |

### Hardware Detection Limitations

Not all components can detect physical absence at init or runtime. Components that are output-only GPIOs have no feedback path from the external hardware -- the ESP32 drives pins but receives no acknowledgment.

| Component               | Detects absence? | Why                                                                                                                                                                                                                                          |
|-------------------------|------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `multiplexer_dg408djz`  | No               | Output-only GPIO. Init readback tests the MCU output latch, not the external IC. The DG408DJZ is a purely analog device with no feedback path.                                                                                               |
| `relay_jd2912`          | No               | Output-only GPIO. Same readback pattern as the mux -- verifies the MCU register, not whether the relay/MOSFET is physically present.                                                                                                         |
| `adc_12bitsar`          | No               | ADC init configures an internal ESP32 peripheral. A floating/disconnected pin reads ~0 mV, which is indistinguishable from "pedal not pressed." Safe direction (override never triggers), but pedal override detection is silently disabled. |
| `optocoupler_pc817`     | Yes              | Pull-ups + active-low signaling: disconnected = both HIGH = NEUTRAL. `init_fr_inputs()` rejects NEUTRAL as invalid, triggering FAULT.                                                                                                        |
| `stepper_motor_uim2852` | Yes              | Init performs a CAN handshake (query status). No response = timeout = init failure, triggering FAULT.                                                                                                                                        |

For the mux and relay, detecting hardware absence would require board-level changes (e.g., adding a sense/feedback line). For the pedal ADC, a plausibility range check on the idle reading could improve detection but is not yet implemented.

## Throttle Control

The throttle system uses an 8-channel analog multiplexer to select 8 levels from a 7-tap resistor ladder (top tap duplicated on CH7):
- Level 0: Idle (minimum throttle)
- Level 7: Maximum throttle
- Slew rate limited: max 1 level change per 100ms

Enable sequence (READY -> ACTIVE):
1. Set mux to level 0
2. Energize pedal bypass relay (GPIO10) - JD-2912 bypasses pedal microswitch
3. Wait 200ms enable dwell
4. Enable steering and braking motors
5. Energize throttle relay (GPIO9) - switches to mux output, then enable mux autonomous path

## Test Bypasses

Compile-time Kconfig flags for bench testing without the full system connected. All default to off (disabled). Enable via `idf.py menuconfig` under **Test Bypasses** (top-level), or add to `sdkconfig.defaults`:

| Flag                                         | Effect                                                          |
|----------------------------------------------|-----------------------------------------------------------------|
| `CONFIG_BYPASS_SAFETY_TARGET_MIRROR`         | Force target_state to ACTIVE (ignore Safety target mirror)      |
| `CONFIG_BYPASS_SAFETY_ESTOP_MIRROR`          | Force Safety estop_fault_code to NONE in Control inputs/logging |
| `CONFIG_BYPASS_SAFETY_LIVENESS_CHECKS`       | Ignore Safety heartbeat timeout (test without Safety on bus)    |
| `CONFIG_BYPASS_PLANNER_COMMAND_INPUTS`       | Force Planner command inputs to zero throttle/steering/braking  |
| `CONFIG_BYPASS_PLANNER_COMMAND_STALE_CHECKS` | Disable Planner command timeout/stale checks                    |
| `CONFIG_BYPASS_INPUT_PEDAL_ADC`              | Skip pedal ADC readings (always not pressed, always re-armed)   |
| `CONFIG_BYPASS_INPUT_FR_SENSOR`              | Force F/R state to FORWARD (skip optocoupler channel reading)   |
| `CONFIG_BYPASS_ACTUATOR_MULTIPLEXER`         | Skip multiplexer and throttle relay control                     |
| `CONFIG_BYPASS_ACTUATOR_PEDAL_RELAY`         | Skip pedal bypass relay energize/de-energize                    |
| `CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING`    | Skip steering stepper motor (node 5) init/configure/commands    |
| `CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING`     | Skip braking stepper motor (node 6) init/configure/commands     |

## Debug Logging

Compile-time Kconfig flags for verbose logging. Enable via `idf.py menuconfig` under **Debug Logging** (top-level):

### CAN Traffic & Planner I/O

| Flag                                | Default | Effect                                                |
|-------------------------------------|---------|-------------------------------------------------------|
| `CONFIG_LOG_CAN_HEARTBEAT_TX`       | off     | Log every periodic heartbeat TX (very verbose)        |
| `CONFIG_LOG_CAN_HEARTBEAT_RX`       | off     | Log every received Safety heartbeat, not just changes |
| `CONFIG_LOG_CAN_PLANNER_COMMAND_RX` | off     | Log every Planner command RX + timeout/stale warnings |

### Component Health & Recovery

| Flag                                       | Default | Effect                                                        |
|--------------------------------------------|---------|---------------------------------------------------------------|
| `CONFIG_LOG_COMPONENT_HEALTH_TRANSITIONS`  | on      | Log component LOST/REGAINED edge transitions                  |
| `CONFIG_LOG_HEARTBEAT_MONITOR_TRANSITIONS` | on      | Log Safety heartbeat monitor lost/regained transitions        |
| `CONFIG_LOG_CAN_RECOVERY`                  | off     | Log CAN bus recovery events (stop/start, reinstall, bus-off)  |
| `CONFIG_LOG_RETRY_TWAI`                    | off     | Log repeated TWAI retry attempts (startup and runtime faults) |
| `CONFIG_LOG_RETRY_MULTIPLEXER`             | off     | Log multiplexer retry attempts while faulted                  |
| `CONFIG_LOG_RETRY_PEDAL_RELAY`             | off     | Log pedal relay retry attempts while faulted                  |
| `CONFIG_LOG_RETRY_PEDAL_INPUT`             | off     | Log pedal ADC retry attempts while faulted                    |
| `CONFIG_LOG_RETRY_FR_INPUT`                | off     | Log F/R input retry attempts while faulted                    |
| `CONFIG_LOG_RETRY_STEPPER_STEERING`        | off     | Log steering stepper retry attempts                           |
| `CONFIG_LOG_RETRY_STEPPER_BRAKING`         | off     | Log braking stepper retry attempts                            |

Retries are unbounded for failed required components, paced at 500ms intervals (no maximum attempt limit). HEARTBEAT_LED is non-critical for FAULT gating and is initialized once at startup.

### Control Logic

| Flag                                       | Default | Effect                                                           |
|--------------------------------------------|---------|------------------------------------------------------------------|
| `CONFIG_LOG_CONTROL_STATE_CHANGES`         | on      | Log control state transitions and transition reasons             |
| `CONFIG_LOG_CONTROL_FAULT_CHANGES`         | off     | Log control fault code changes                                   |
| `CONFIG_LOG_CONTROL_STATE_TICK`            | off     | Log state-machine evaluation every 20ms cycle (very verbose)     |
| `CONFIG_LOG_CONTROL_ENABLE_SEQUENCE`       | off     | Log enable/disable sequence steps (start, complete, abort)       |
| `CONFIG_LOG_CONTROL_OVERRIDE`              | off     | Log driver override trigger events (pedal/F/R)                   |
| `CONFIG_LOG_CONTROL_THROTTLE_CHANGES`      | off     | Log throttle level changes                                       |
| `CONFIG_LOG_CONTROL_THROTTLE_TICK`         | off     | Log throttle current/target every 20ms cycle (very verbose)      |
| `CONFIG_LOG_CONTROL_PRECONDITION_BLOCKED`  | on      | Log when enable preconditions block readiness/ENABLE transitions |
| `CONFIG_LOG_CONTROL_SAFETY_TARGET_CHANGES` | on      | Log received Safety target/fault changes seen by Control         |

### Inputs

| Flag                           | Default | Effect                                               |
|--------------------------------|---------|------------------------------------------------------|
| `CONFIG_LOG_INPUT_PEDAL_ADC`   | off     | Log pedal ADC millivolt readings (extremely verbose) |
| `CONFIG_LOG_INPUT_FR_DEBOUNCE` | off     | Log F/R switch debounce transitions                  |
| `CONFIG_LOG_INPUT_FR_STATE`    | off     | Log F/R selector state changes                       |

### Actuators

| Flag                                     | Default | Effect                                              |
|------------------------------------------|---------|-----------------------------------------------------|
| `CONFIG_LOG_ACTUATOR_MUX_LEVEL`          | off     | Log multiplexer level changes with A2/A1/A0 values  |
| `CONFIG_LOG_ACTUATOR_PEDAL_RELAY`        | off     | Log pedal bypass relay energize/de-energize         |
| `CONFIG_LOG_ACTUATOR_STEPPER_COMMAND_TX` | off     | Log stepper position commands sent over CAN         |
| `CONFIG_LOG_ACTUATOR_STEPPER_MOTION_TX`  | off     | Log stepper motion commands (PA/PR/ST)              |
| `CONFIG_LOG_ACTUATOR_STEPPER_RX`         | off     | Log parsed stepper CAN RX frames (MS, params, ACKs) |

### HEARTBEAT_LED

| Flag                                     | Default | Effect                                                                |
|------------------------------------------|---------|-----------------------------------------------------------------------|
| `CONFIG_LOG_HEARTBEAT_LED_STATE_CHANGES` | off     | Log HEARTBEAT_LED color/reason mode changes                           |
| `CONFIG_LOG_HEARTBEAT_LED_COLOR_UPDATES` | off     | Log every HEARTBEAT_LED color update with color/reason (very verbose) |

## Build

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p PORT flash monitor
```
