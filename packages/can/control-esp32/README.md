# control-esp32

ESP-IDF firmware for the Control ESP32-C6. Receives commands from the Planner (Jetson AGX Orin) over CAN and controls/monitors throttle, steering, and braking actuators. Follows the system target state broadcast by Safety ESP32.

## State Machine

Control follows Safety's target state (`NOT_READY`/`READY`/`ENABLE`/`ACTIVE`) and reports retreat causes via heartbeat `stop_flags` (non-fault interventions) and `fault_code` (issues).

High-level rules:

- `READY` means local preconditions are currently satisfied.
- `NOT_READY` means local preconditions are not satisfied (not a fault).
- Driver intervention retreats to `NOT_READY` with operator bits in `stop_flags`.
- Runtime issues retreat to `NOT_READY` with issue codes in `fault_code`.
- Safety only commands target states `NOT_READY`/`READY`/`ENABLE`/`ACTIVE`.
- Issue/intervention causes clear back to `NODE_FAULT_NONE` when conditions recover, allowing `NOT_READY -> READY` when preconditions pass.

### States

| State     | Description                                                                                                           |
| --------- | --------------------------------------------------------------------------------------------------------------------- |
| INIT      | Hardware initializing.                                                                                                |
| NOT_READY | Manual mode, but local autonomy preconditions are not satisfied.                                                      |
| READY     | Manual mode with local autonomy preconditions satisfied. Waiting for Safety to advance.                               |
| ENABLE    | Enable relay energized, waiting to switch throttle source. Sets `NODE_STATUS_FLAG_ENABLE_COMPLETE` when done.           |
| ACTIVE    | Autonomous mode. Executing throttle/steering/braking commands from Planner.                                           |

### Transitions

**`INIT -> NOT_READY/READY`** occurs after init dwell. If local preconditions pass at that moment, transition to `READY`; otherwise transition to `NOT_READY`.

**`NOT_READY <-> READY`** is driven by local preconditions:

- F/R switch not in Reverse (FORWARD or NEUTRAL both satisfy this)
- Pedal not pressed
- Pedal re-armed (released for 500ms after any press)
- No active issue code and no active stop flags

Note: The anti-arcing microswitch is in series after the throttle microswitch. Before the DPDT relay (MY5NJ) is energized during ENABLE, the anti-arcing switch cannot conduct. Pre-bypass, FORWARD reads as NEUTRAL and REVERSE reads as INVALID. The precondition check only requires "not in reverse" — NEUTRAL is accepted because it is indistinguishable from FORWARD pre-bypass.

**`READY -> ENABLE`** requires all:

- Safety target state is ENABLE or ACTIVE (from Safety heartbeat `state` field)
- F/R switch not in Reverse
- Pedal not pressed
- Pedal re-armed (released for 500ms after any press)
- No active issue code and no active stop flags

**`ENABLE -> ACTIVE`** advances when Safety target reaches `ACTIVE`. It can abort back to `NOT_READY`/`READY` if Safety retreats, pedal is pressed, or F/R moves to Reverse. NEUTRAL during ENABLE is allowed.

**`ACTIVE -> NOT_READY`** is triggered immediately by any of these operator intervention causes (encoded in `stop_flags`):

- Pedal press (no debounce - immediate response)
- F/R switch moved to Reverse (debounced)
- Steering position error (actual encoder diverged >200 pulses from commanded target after motion complete)
- Braking position error (actual encoder diverged >200 pulses from commanded target after motion complete)

Note: F/R NEUTRAL while ACTIVE does NOT trigger override. Instead, throttle is zeroed (cart can't drive in neutral anyway) while steering/braking continue. The system resumes normal throttle when FORWARD is engaged.

**`ACTIVE -> NOT_READY/READY`** occurs when Safety target retreats (e-stop, node fault/timeout, or Planner/Orin autonomy halt / error).
This is a commanded retreat from Safety, not a human override event.

**Issue/stop clear conditions** (while in `NOT_READY`) include:

- F/R switch not in Reverse or Invalid (FORWARD or NEUTRAL)
- Pedal re-armed (released for 500ms)

Recovery target is recomputed from current preconditions (`NOT_READY` or `READY`).

## CAN Messages

### Receives (Standard 11-bit Frames)

| ID    | Name             | Description                                                             |
| ----- | ---------------- | ----------------------------------------------------------------------- |
| 0x100 | SAFETY_HEARTBEAT | System target state + Safety `fault_code` and `stop_flags` (same `node_heartbeat_t` format) |
| 0x111 | PLANNER_COMMAND  | Commands (sequence, throttle 0-255, steering_pos, braking_pos)          |

### Sends (Standard 11-bit Frames)

| ID    | Name              | Rate                              | Description                                  |
| ----- | ----------------- | --------------------------------- | -------------------------------------------- |
| 0x120 | CONTROL_HEARTBEAT | 100ms + immediate on state change | Alive signal (seq, state, fault_code, stop_flags, status_flags) |

### Stale Planner Command Detection

Control tracks the Planner command sequence number. If the same sequence is seen 10 consecutive times (`PLANNER_CMD_STALE_COUNT`), Control zeros the throttle as a safety measure.

### UIM2852CA Motors (Extended 29-bit Frames)

| Motor    | Node ID | Description                         |
| -------- | ------- | ----------------------------------- |
| Steering | 5       | Linear actuator for steering column |
| Braking  | 6       | Linear actuator for brake pedal     |

Master controller ID: 4. See `stepper_protocol_uim2852.h` for CAN ID encoding.

## Pin Configuration

| GPIO | Function        | Direction | Notes                                                           |
| ---- | --------------- | --------- | --------------------------------------------------------------- |
| 0    | Pedal ADC       | Analog In | ADC1_CH0, voltage divider (220k/100k), threshold 500mV          |
| 2    | Digipot MOSI    | Output    | MCP41HVX1 SPI data                                              |
| 3    | Digipot SCK     | Output    | MCP41HVX1 SPI clock                                             |
| 4    | CAN TX          | Output    | TWAI peripheral (SN65HVD230 transceiver)                        |
| 5    | CAN RX          | Input     | TWAI peripheral (SN65HVD230 transceiver)                        |
| 6    | Digipot CS      | Output    | MCP41HVX1 SPI chip select                                       |
| 8    | Status LED      | Output    | Onboard WS2812 RGB LED (no external wiring)                     |
| 10   | DPDT Relay      | Output    | 2N5551 base via 680R (10k pull-down), MY5NJ 24V coil             |
| 22   | F/R Forward     | Input     | PC817 optocoupler, pull-up, active LOW                          |
| 23   | F/R Reverse     | Input     | PC817 optocoupler, pull-up, active LOW                          |

### LED Behavior

| Color        | State                                                |
| ------------ | ---------------------------------------------------- |
| Solid green  | Local Control state READY                            |
| Solid blue   | Local Control state ACTIVE                           |
| Solid red    | Local Control state NOT_READY or fault overlay active |
| Solid yellow | Local Control state INIT/ENABLE                      |

## Wiring

Each subsection covers production (on-cart) and bench wiring for Control ESP32 interfaces where applicable. F/R hardware wiring in this revision is cart-only; on bench, use the F/R input bypass flag if cart switch wiring is not present. For CAN bus topology and stepper motor wiring shared across all nodes, see [CAN Bus Wiring](../README.md#can-bus-wiring).

### Wiring Summary

| Interface                                    | Bench vs Production                                            |
| -------------------------------------------- | -------------------------------------------------------------- |
| CAN bus (SN65HVD230)                         | Same — see [root README](../README.md#can-bus-wiring)          |
| Throttle system (MCP41HVX1 + MY5NJ relay)    | Same hardware — bench output unloaded (no Curtis controller)   |
| Pedal ADC                                    | Same — bench divider floating reads ~0mV (safe default)        |
| F/R optocouplers (PC817)                     | Cart wiring only — production-style 48V switch wiring via 4.7k |
| Status LED (WS2812)                          | Same                                                           |

### CAN Bus (SN65HVD230)

GPIO 4 (TX) and GPIO 5 (RX) connect to a WAVESHARE SN65HVD230 CAN transceiver module via a 4-pin JST-PH connector. Control ESP32's Waveshare board has the onboard termination resistor **removed** (termination is on Safety ESP32 and Planner/Orin). See the [root README](../README.md#can-bus-wiring) for the full 5-node bus topology including stepper motors.

**ESP32-to-transceiver connector (4-pin JST-PH):**

| Pin | Wire Color | Signal       |
| --- | ---------- | ------------ |
| 1   | Yellow     | TXD (GPIO 4) |
| 2   | Green      | RXD (GPIO 5) |
| 3   | Red        | VCC (3.3V)   |
| 4   | Black      | GND          |

### Throttle Box Connectors

All throttle-related components (MCP41HVX1 digipot, MY5NJ DPDT relay, 2N5551, PC817 optocouplers, pedal ADC voltage divider) are housed in a separate throttle box perfboard powered from the cart's 24V rail. The Control ESP32 connects to the throttle box via four cables:

**ESP32-side connectors:**

| Label  | Connector    | Pin 1                      | Pin 2                       | Pin 3                    | Pin 4 |
| ------ | ------------ | -------------------------- | --------------------------- | ------------------------ | ----- |
| **J1** | 3-pin JST-PH | Digipot MOSI (GPIO 2) WHITE | Digipot SCK (GPIO 3) YELLOW | Digipot CS (GPIO 6) GREEN |       |
| **J2** | 2-pin JST-PH | DPDT Relay Base (GPIO 10) WHITE | Pedal ADC (GPIO 0) YELLOW |                          |       |
| **J3** | 2-pin JST-PH | F/R Fwd (GPIO 22) WHITE    | F/R Rev (GPIO 23) GREEN     |                          |       |
| **J9** | Single wire  | ESP32 GND BLACK            |                             |                          |       |

J9 is a single black wire connecting an ESP32 GND pin to the throttle box GND bus rail. Required for signal reference between the ESP32 and the throttle box.

**Cart-side connectors (on the throttle box):**

| Label  | Connector          | Pin 1                  | Pin 2                 | Pin 3          | Pin 4                 |
| ------ | ------------------ | ---------------------- | --------------------- | -------------- | --------------------- |
| **J4** | Anderson Powerpole | 24V+ RED               | GND BLACK             |                |                       |
| **J5** | 4-pin JST-PH       | Curtis Pin 2 RED       | Curtis Pin 3 YELLOW   | Curtis B- BLUE | Pedal pot wiper GREEN |
| **J6** | 2-pin JST-PH       | Bypass wire A YELLOW   | Bypass wire B WHITE   |                |                       |
| **J7** | 2-pin JST-PH       | Anti-arc signal YELLOW | Anti-arc return GREEN |                |                       |
| **J8** | 2-pin JST-PH       | Buzzer supply BLUE     | Buzzer signal WHITE   |                |                       |

**Important:** J5 pin 3 (Curtis B-, blue wire) connects to the digipot Terminal B — it is NOT connected to the throttle box GND bus. J7/J8 wires are galvanically isolated 48V circuits — they do NOT connect to GND bus or any ESP32 signal. See [F/R Optocouplers (PC817)](#fr-optocouplers-pc817) for the exact cart-side connection points for J7 and J8.

**J5 internal throttle box connections:**

The MY5NJ DPDT relay Pole 1 on the throttle box perfboard switches the Curtis throttle input (Pin 3) between the manual pedal pot and the MCP41HVX1 digipot wiper. J5 connects the relay and digipot to the cart-side Curtis controller and pedal pot:

| J5 Pin | Cart-Side Signal                     | Throttle Box Internal Connection          |
| ------ | ------------------------------------ | ----------------------------------------- |
| Pin 1  | Curtis Pin 2 (pot high ref) RED      | Digipot Terminal A                         |
| Pin 2  | Curtis Pin 3 (throttle input) YELLOW | DPDT Relay Pole 1 COM (output)             |
| Pin 3  | Curtis B- BLUE                       | Digipot Terminal B                          |
| Pin 4  | Pedal pot wiper GREEN                | DPDT Relay Pole 1 NC (input)               |

The relay switches Curtis Pin 3 between manual pedal pot (NC, de-energized) and digipot wiper (NO, energized). In manual mode, the pot wiper signal passes through NC → COM → Curtis Pin 3. In autonomous mode, the digipot wiper output passes through NO → COM → Curtis Pin 3.

**Curtis controller pinout (1999 Club Car DS 48V):**

| Curtis Pin | Function              | Used by J5? |
| ---------- | --------------------- | ----------- |
| Pin 1      | Key switch            | No          |
| Pin 2      | Pot high reference    | Yes (Pin 1) |
| Pin 3      | Pot wiper / throttle input | Yes (Pin 2) |
| B-         | Battery negative      | Yes (Pin 3) |

**J5 cart-side wiring procedure:**

The original cart wiring connects the pedal pot wiper directly to Curtis Pin 3. To intercept this signal for the DPDT relay:

1. **CUT** the original pedal pot wiper → Curtis Pin 3 wire:
   - Curtis Pin 3 side of the cut → J5 Pin 2 (YELLOW)
   - Pedal pot wiper side of the cut → J5 Pin 4 (GREEN)
2. **SPLICE** (do not cut) the Curtis Pin 2 wire:
   - Original connection to pedal pot high terminal stays intact
   - New branch → J5 Pin 1 (RED) for the digipot Terminal A
3. **SPLICE** (do not cut) the Curtis B- wire:
   - Original connection stays intact
   - New branch → J5 Pin 3 (BLUE) for the digipot Terminal B

Curtis Pin 2 must remain connected to the pedal pot high terminal so the pot retains its reference voltage for manual mode.

### Throttle System (MCP41HVX1 Digipot + MY5NJ Relay)

MCP41HVX1 high-voltage digital potentiometer provides 256 wiper positions (0-255) for continuous throttle level control. Terminal A connects to Curtis Pin 2 (8.5V reference), Terminal B to Curtis B- (ground reference), wiper output to the MY5NJ DPDT relay Pole 1 NO terminal.

**Power supply:** V+ powered from 24V rail (max 36V). VL (digital logic) powered from ESP32 3.3V. SPI interface: MOSI (GPIO 2), SCK (GPIO 3), CS (GPIO 6). SPI clock: 1 MHz, mode 0.

**Safe state:** Wiper position 0 (minimum throttle, near Terminal B). At wiper 0 the output voltage is approximately 0V — well within the Curtis controller's deadband, producing no movement.

**MY5NJ DPDT relay** (24V coil, driven via 2N5551 NPN transistor on GPIO 10):
- Pole 1 switches Curtis Pin 3 between manual pedal pot (NC) and digipot wiper (NO)
- Pole 2 bypasses the pedal microswitch (NC = normal pedal operation, NO = bypassed)
- Both poles switch simultaneously (DPDT)
- De-energized = safe state (manual pedal control, no bypass)

**Production:** Digipot wiper feeds into the Curtis motor controller throttle input through the DPDT relay. The 256 wiper positions span the Curtis controller's full throttle range with fine granularity.

**Bench:** Same MCP41HVX1 + MY5NJ relay hardware. The digipot output is unloaded (no Curtis controller connected). Useful for verifying SPI communication with a DMM on the wiper output. The relay will audibly click when energized — verifies GPIO 10 output.

### Pedal ADC

GPIO 0 (ADC1_CH0) reads the accelerator pedal position through a voltage divider (220k/100k). Threshold for "pressed" is 500 mV.

**Production:** Voltage divider connected to the cart's accelerator pedal potentiometer wiper. Pedal pressed produces >500 mV; released reads ~0 mV.

**Bench:** Same voltage divider. With no pedal connected, the floating input reads ~0 mV (treated as "not pressed" — safe default).

### F/R Optocouplers (PC817)

The F/R decode is wired for a **1999 Club Car DS 48V** forward/reverse switch assembly. The switch has three microswitches; two are used for direction sensing:

| Microswitch    | GPIO              | Active When                                                     | Circuit Position |
| -------------- | ----------------- | --------------------------------------------------------------- | ---------------- |
| Anti-arcing    | 22 (forward_gpio) | Forward **and** Reverse (opens solenoid coil during transition) | High-side switch (output → solenoid coil → B-) |
| Reverse buzzer | 23 (reverse_gpio) | Reverse only (activates backup buzzer)                          | Low-side switch (supply → buzzer → input, output → B-) |

State decode (PC817 optocouplers, active LOW with internal pull-up):

| GPIO 22 (Anti-arc) | GPIO 23 (Buzzer) | State           | Meaning                                |
| ------------------ | ---------------- | --------------- | -------------------------------------- |
| LOW                | HIGH             | Forward         | Anti-arc ON, buzzer OFF                |
| LOW                | LOW              | Reverse         | Both ON                                |
| HIGH               | HIGH             | Neutral         | Neither ON (handle in center detent)   |
| HIGH               | LOW              | Invalid (fault) | Buzzer without anti-arc = wiring fault |

Each PC817 provides galvanic isolation between the cart's 48V signal circuits and the ESP32. A 4.7k ohm current-limiting resistor on pin 1 of each PC817 LED limits current at 48V to ~10 mA. Keep the cart-side F/R switch wiring isolated from the ESP32 GPIO side (do not tie any cart-side LED wire to ESP32 GND or the throttle box GND bus).

The two microswitches sit in **different circuit positions** (high-side vs low-side) in the cart's 48V wiring, so their optocoupler LED connections differ. In both cases, the PC817 LED must be wired so current flows through the LED **only** when the microswitch is closed. Wiring the LED across the microswitch contacts (in parallel with the switch) is **incorrect** — it creates a sneak current path that activates the cart load (buzzer/solenoid) regardless of switch position and inverts the sensing logic.

#### Anti-arcing microswitch — high-side switch (J7)

The anti-arc microswitch is a high-side switch: its output feeds the solenoid coil, which returns to B-. The output terminal toggles between supply voltage (switch closed) and ~B- (switch open, pulled through coil impedance).

```
Supply → Anti-arc microswitch → OUTPUT ──┬── Solenoid coil → B-
                                         │
                                         └── J7 Pin1 (YELLOW) → [4.7k → PC817 LED] → J7 Pin2 (GREEN) → B-
```

- **J7 Pin 1 (YELLOW, "Anti-arc signal")**: Connect to the microswitch **output** terminal (the terminal that goes toward the solenoid coil — hot only when the switch is closed).
- **J7 Pin 2 (GREEN, "Anti-arc return")**: Connect to **B-** (battery negative / chassis ground).

| Switch State | OUTPUT Voltage | LED   | GPIO 22 | Software     |
| ------------ | -------------- | ----- | ------- | ------------ |
| Closed       | ~Supply        | ON    | LOW     | active ✓     |
| Open         | ~B- (via coil) | OFF   | HIGH    | inactive ✓   |

#### Reverse buzzer microswitch — low-side switch (J8)

The reverse buzzer microswitch is a low-side switch: its output connects directly to Curtis B-. The buzzer sits between the supply and the microswitch input. Because the output is always at B-, tapping the output is useless for sensing. Tapping the input with a return to B- creates a sneak path through the buzzer (current flows `supply → buzzer → 4.7k → LED → B-` even when the switch is open, audibly activating the buzzer and inverting the logic).

Instead, wire the LED from the **supply** (before the buzzer) to the microswitch **input** (after the buzzer). When the switch is open, both ends of the LED are at supply potential (the input is pulled to supply through the buzzer with no current flowing), so 0V appears across the LED and it stays off. When the switch is closed, the input is pulled to B- through the switch, and current flows through the LED.

```
Supply ──┬── Buzzer → Microswitch INPUT ──┬── Microswitch → OUTPUT (B-)
         │                                │
         └── J8 Pin1 (BLUE) → [4.7k → PC817 LED] → J8 Pin2 (WHITE) ─┘
```

- **J8 Pin 1 (BLUE, "Buzzer supply")**: Connect to the **supply wire** that feeds the buzzer circuit (the hot wire in the 48V domain, before the buzzer). This is upstream of the buzzer — find it with a multimeter: keyed on, F/R in neutral, the wire that reads ~48V against B- regardless of switch position.
- **J8 Pin 2 (WHITE, "Buzzer signal")**: Connect to the microswitch **input** terminal (the terminal connected to the buzzer, NOT the terminal that goes to B-).

| Switch State | Microswitch INPUT | LED Voltage          | LED   | GPIO 23 | Software     |
| ------------ | ----------------- | -------------------- | ----- | ------- | ------------ |
| Closed       | ~B- (via switch)  | Supply − B- ≈ 48V    | ON    | LOW     | active ✓     |
| Open         | ~Supply (via buzzer, no current) | Supply − Supply ≈ 0V | OFF   | HIGH    | inactive ✓   |

**Bench:** If cart F/R switch wiring is absent on bench, disconnected optocouplers read NEUTRAL (both HIGH). Pre-bypass, NEUTRAL passes preconditions (not-in-reverse), so the system can reach READY and ENABLE without the F/R sensor bypass. Use `CONFIG_BYPASS_INPUT_FR_SENSOR` only if you need to force FORWARD readings for testing post-bypass ACTIVE behavior.

### Status LED (WS2812)

GPIO 8 is configured as an RMT TX output driving the onboard WS2812 RGB status LED data line (no external wiring required). LED color is determined by software state logic, not GPIO level — the GPIO serves only as the RMT data output.

## Components

| Component                  | Description                                                  |
| -------------------------- | ------------------------------------------------------------ |
| `digipot_mcp41hvx1`       | MCP41HVX1 SPI digital potentiometer for throttle level selection |
| `stepper_motor_uim2852`    | UIM2852CA closed-loop stepper motor control API              |
| `relay_dpdt_my5nj`         | MY5NJ DPDT relay driver (24V coil via 2N5551 NPN transistor)      |
| `adc_12bitsar`             | Dedicated ESP32-C6 12-bit SAR ADC read/calibration helper    |
| `optocoupler_pc817`        | Dedicated F/R PC817 decode + debounce helper                 |
| `can_twai`                 | CAN bus driver wrapper (shared)                              |
| `can_protocol`             | Message definitions and encode/decode (shared)               |
| `led_ws2812`               | WS2812 status LED driver (shared)                            |
| `stepper_protocol_uim2852` | UIM2852 SimpleCAN protocol library (shared)                  |
| `control_logic`            | Extracted state machine decision logic (shared, tested)      |

### Hardware Detection Limitations

Not all components can detect physical absence at init or runtime. Components that are output-only GPIOs have no feedback path from the external hardware -- the ESP32 drives pins but receives no acknowledgment.

| Component               | Detects absence? | Why                                                                                                                                                                                                                                          |
| ----------------------- | ---------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `digipot_mcp41hvx1`    | Partial          | SPI transaction success is verified, but no readback of actual wiper position. A disconnected digipot will report SPI success with no actual output.                                                                                          |
| `relay_dpdt_my5nj`     | No               | Output-only GPIO. Same readback pattern as before — verifies the MCU register, not whether the relay/transistor is physically present.                                                                                                        |
| `adc_12bitsar`          | No               | ADC init configures an internal ESP32 peripheral. A floating/disconnected pin reads ~0 mV, which is indistinguishable from "pedal not pressed." Safe direction (override never triggers), but pedal override detection is silently disabled. |
| `optocoupler_pc817`     | No (pre-bypass)  | Pull-ups + active-low signaling: disconnected = both HIGH = NEUTRAL. Pre-bypass, NEUTRAL is expected (FORWARD reads as NEUTRAL when anti-arc switch can't conduct). Hardware absence is indistinguishable from "cart in FORWARD/NEUTRAL." Post-bypass (ACTIVE), the full truth table is readable and a disconnected optocoupler would read NEUTRAL, which zeroes throttle but does not fault. |
| `stepper_motor_uim2852` | Yes              | Init performs a CAN handshake (query status). No response = timeout = init failure, triggering FAULT.                                                                                                                                        |

## Throttle Control

The throttle system uses an MCP41HVX1 digital potentiometer to provide 256 wiper positions (0-255) for continuous throttle level control:

- Wiper 0: Off (near Terminal B, in Curtis deadband, no movement)
- Low wiper values: Fine low-speed control
- Wiper 255: Maximum throttle (near Terminal A, full reference voltage)
- Slew rate limited: max 12 wiper steps per 100ms

Enable sequence (READY -> ACTIVE):

1. Set digipot wiper to 0
2. Wait 200ms enable dwell
3. Enable steering and braking motors
4. Energize DPDT relay (GPIO 10) — both poles switch: throttle source to digipot, pedal bypass active
5. Enable digipot autonomous mode

## Test Bypasses

Compile-time Kconfig flags for bench testing without the full system connected. All default to off (disabled). Enable via `idf.py menuconfig` under **Test Bypasses** (top-level), or add to `sdkconfig.defaults`:

| Flag                                         | Effect                                                          |
| -------------------------------------------- | --------------------------------------------------------------- |
| `CONFIG_BYPASS_SAFETY_TARGET_MIRROR`         | Force target_state to ACTIVE (ignore Safety target mirror)      |
| `CONFIG_BYPASS_SAFETY_ESTOP_MIRROR`          | Force Safety mirror channels clear (`safety_fault_code=NONE`, `safety_stop_flags=NONE`) |
| `CONFIG_BYPASS_SAFETY_LIVENESS_CHECKS`       | Ignore Safety heartbeat timeout (test without Safety on bus)    |
| `CONFIG_BYPASS_PLANNER_COMMAND_INPUTS`       | Force Planner command inputs to zero throttle/steering/braking  |
| `CONFIG_BYPASS_PLANNER_COMMAND_STALE_CHECKS` | Disable Planner command timeout/stale checks                    |
| `CONFIG_BYPASS_INPUT_PEDAL_ADC`              | Skip pedal ADC readings (always not pressed, always re-armed)   |
| `CONFIG_BYPASS_INPUT_FR_SENSOR`              | Force F/R state to FORWARD (skip optocoupler channel reading)   |
| `CONFIG_BYPASS_ACTUATOR_THROTTLE`          | Skip digipot and DPDT relay control                            |
| `CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING`    | Skip steering stepper motor (node 5) init/configure/commands    |
| `CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING`     | Skip braking stepper motor (node 6) init/configure/commands     |

## Debug Logging

Compile-time Kconfig flags for verbose logging. Enable via `idf.py menuconfig` under **Debug Logging** (top-level):

### CAN Traffic & Planner I/O

| Flag                                | Default | Effect                                                |
| ----------------------------------- | ------- | ----------------------------------------------------- |
| `CONFIG_LOG_CAN_HEARTBEAT_TX`       | off     | Log every periodic heartbeat TX (very verbose)        |
| `CONFIG_LOG_CAN_HEARTBEAT_RX`       | off     | Log every received Safety heartbeat, not just changes |
| `CONFIG_LOG_CAN_PLANNER_COMMAND_RX` | off     | Log every Planner command RX + timeout/stale warnings |

### Component Health & Recovery

| Flag                                       | Default | Effect                                                        |
| ------------------------------------------ | ------- | ------------------------------------------------------------- |
| `CONFIG_LOG_COMPONENT_HEALTH_TRANSITIONS`  | on      | Log component LOST/REGAINED edge transitions                  |
| `CONFIG_LOG_HEARTBEAT_MONITOR_TRANSITIONS` | on      | Log Safety heartbeat monitor lost/regained transitions        |
| `CONFIG_LOG_CAN_RECOVERY`                  | off     | Log CAN bus recovery events (stop/start, reinstall, bus-off)  |
| `CONFIG_LOG_RETRY_TWAI`                    | off     | Log repeated TWAI retry attempts (startup and runtime faults) |
| `CONFIG_LOG_RETRY_DIGIPOT`                 | off     | Log digipot retry attempts while faulted                  |
| `CONFIG_LOG_RETRY_DPDT_RELAY`              | off     | Log DPDT relay retry attempts while faulted               |
| `CONFIG_LOG_RETRY_PEDAL_INPUT`             | off     | Log pedal ADC retry attempts while faulted                    |
| `CONFIG_LOG_RETRY_FR_INPUT`                | off     | Log F/R input retry attempts while faulted                    |
| `CONFIG_LOG_RETRY_STEPPER_STEERING`        | off     | Log steering stepper retry attempts                           |
| `CONFIG_LOG_RETRY_STEPPER_BRAKING`         | off     | Log braking stepper retry attempts                            |

Retries are unbounded for failed required components, paced at 500ms intervals (no maximum attempt limit). HEARTBEAT_LED is non-critical for FAULT gating and is initialized once at startup.

### Control Logic

| Flag                                       | Default | Effect                                                           |
| ------------------------------------------ | ------- | ---------------------------------------------------------------- |
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
| ------------------------------ | ------- | ---------------------------------------------------- |
| `CONFIG_LOG_INPUT_PEDAL_ADC`   | off     | Log all pedal ADC millivolt readings every cycle (including below threshold; extremely verbose) |
| `CONFIG_LOG_INPUT_PEDAL_EVENTS` | off     | Log edge-triggered pedal threshold/rearm events     |
| `CONFIG_LOG_INPUT_FR_DEBOUNCE` | off     | Log F/R switch debounce transitions                  |
| `CONFIG_LOG_INPUT_FR_STATE`    | off     | Log F/R selector state changes                       |

### Actuators

| Flag                                     | Default | Effect                                              |
| ---------------------------------------- | ------- | --------------------------------------------------- |
| `CONFIG_LOG_ACTUATOR_DIGIPOT_WIPER`        | off     | Log digipot wiper position changes                       |
| `CONFIG_LOG_ACTUATOR_DPDT_RELAY`           | off     | Log DPDT relay energize/de-energize                      |
| `CONFIG_LOG_ACTUATOR_STEPPER_COMMAND_TX` | off     | Log stepper position commands sent over CAN         |
| `CONFIG_LOG_ACTUATOR_STEPPER_MOTION_TX`  | off     | Log stepper motion commands (PA/PR/ST)              |
| `CONFIG_LOG_ACTUATOR_STEPPER_RX`         | off     | Log parsed stepper CAN RX frames (MS, params, ACKs) |

### HEARTBEAT_LED

| Flag                                     | Default | Effect                                                                |
| ---------------------------------------- | ------- | --------------------------------------------------------------------- |
| `CONFIG_LOG_HEARTBEAT_LED_STATE_CHANGES` | off     | Log HEARTBEAT_LED color/reason mode changes                           |
| `CONFIG_LOG_HEARTBEAT_LED_COLOR_UPDATES` | off     | Log every HEARTBEAT_LED color update with color/reason (very verbose) |

## Build

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p PORT flash monitor
```
