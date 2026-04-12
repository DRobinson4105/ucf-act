# control-esp32

ESP-IDF firmware for the Control ESP32-C6-DevKitM-1. Receives planner commands from the Jetson AGX Orin over UART0 (the board's COM USB-C port, wired through the onboard CP2102N bridge) at 1 Mbaud, and controls/monitors throttle, steering, and braking actuators. Follows the system target state broadcast by Safety ESP32 over CAN.

The `main/` sources are organized into subdirectories by concern: `config/` (compile-time constants), `state/` (global state declarations), `health/` (component health and recovery), `actuators/` (enable/disable/override action helpers), `tasks/` (FreeRTOS task entry points — CAN RX, 50Hz control loop, heartbeat), `init/` (peripheral bring-up and task creation), `transport/` (CAN RX decode, heartbeat TX, Orin UART link), `inputs/` (pedal ADC + F/R sensor polling), and `bench/` (standalone actuator bench-test mode). `main.cpp` contains only the production build guard and `app_main` entry point.

## State Machine

Control follows Safety's target state (`NOT_READY`/`READY`/`ENABLE`/`ACTIVE`) and reports retreat causes via heartbeat `stop_flags` (non-fault interventions) and `fault_flags` (issues).

High-level rules:

- `READY` means local preconditions are currently satisfied.
- `NOT_READY` means local preconditions are not satisfied (not a fault).
- Driver intervention retreats to `NOT_READY` with operator bits in `stop_flags`.
- Runtime issues retreat to `NOT_READY` with issue codes in `fault_flags`.
- Safety only commands target states `NOT_READY`/`READY`/`ENABLE`/`ACTIVE`.
- Issue/intervention causes clear back to `NODE_FAULT_NONE` when conditions recover, allowing `NOT_READY -> READY` when preconditions pass.

### States

| State     | Description                                                                                                   |
| --------- | ------------------------------------------------------------------------------------------------------------- |
| INIT      | Hardware initializing.                                                                                        |
| NOT_READY | Manual mode, but local autonomy preconditions are not satisfied.                                              |
| READY     | Manual mode with local autonomy preconditions satisfied. Waiting for Safety to advance.                       |
| ENABLE    | Enable relay energized, waiting to switch throttle source. Sets `NODE_STATUS_FLAG_ENABLE_COMPLETE` when done. |
| ACTIVE    | Autonomous mode. Executing throttle/steering/braking commands from Planner.                                   |

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

**`ACTIVE -> NOT_READY/READY`** occurs when Safety target retreats (stop/fault active, node timeout, or Planner/Orin autonomy halt).
This is a commanded retreat from Safety, not a human override event.

**Issue/stop clear conditions** (while in `NOT_READY`) include:

- F/R switch not in Reverse or Invalid (FORWARD or NEUTRAL)
- Pedal re-armed (released for 500ms)

Recovery target is recomputed from current preconditions (`NOT_READY` or `READY`).

## External Interfaces

### Orin UART Link (COM USB-C port → UART0)

The Orin plugs into the DevKitM-1's **COM** USB-C connector, which routes through the CP2102N bridge to UART0. `usb_serial_link_init()` owns UART0 at 1 Mbaud; `control_orin_link_rx_task` pulls framed messages off the link and `control_heartbeat_tx` pushes heartbeats the other way. Frames carry an `ORIN_LINK_SYNC_BYTE` (0xAA) + 1-byte type + 1-byte length header — see [common/protocol/README.md](../common/protocol/README.md) for the wire format.

| Direction | Message            | Description                                                            |
| --------- | ------------------ | ---------------------------------------------------------------------- |
| RX        | PLANNER_COMMAND    | Planner command from Orin (sequence, throttle, steering_pos, braking_pos) |
| TX        | CONTROL_HEARTBEAT  | Local Control heartbeat mirrored to Orin for liveness/status reporting |

Planner command and heartbeat payloads reuse the shared `can_protocol` byte layouts and are framed on the UART by `orin_link_protocol`. The separate **USB** USB-C connector (native USB Serial JTAG) stays free for `idf.py flash` / `idf.py monitor` from the development laptop.

### CAN Messages

### Receives (Standard 11-bit Frames)

| ID    | Name             | Description                                                                                  |
| ----- | ---------------- | -------------------------------------------------------------------------------------------- |
| 0x100 | SAFETY_HEARTBEAT | System target state + Safety `fault_flags` and `stop_flags` (same `node_heartbeat_t` format) |

### Sends (Standard 11-bit Frames)

| ID    | Name              | Rate                              | Description                                                      |
| ----- | ----------------- | --------------------------------- | ---------------------------------------------------------------- |
| 0x120 | CONTROL_HEARTBEAT | 100ms + immediate on state change | Alive signal (seq, state, fault_flags, stop_flags, status_flags) |

### Stale Planner Command Detection

Control tracks the Planner command sequence number. If the same sequence is seen 10 consecutive times, Control zeros the throttle as a safety measure.

### UIM2852CA Motors (Extended 29-bit Frames)

| Motor    | Node ID | Description                         |
| -------- | ------- | ----------------------------------- |
| Steering | 7       | Linear actuator for steering column |
| Braking  | 6       | Linear actuator for brake pedal     |

See `motor_protocol.h` in `motor_uim2852` for CAN ID encoding.

## Pin Configuration

| GPIO | Function    | Direction | Notes                                                  |
| ---- | ----------- | --------- | ------------------------------------------------------ |
| 0    | Pedal ADC   | Analog In | ADC1_CH0, voltage divider (220k/100k), threshold 500mV |
| 4    | CAN TX      | Output    | TWAI peripheral (SN65HVD230 transceiver)               |
| 5    | CAN RX      | Input     | TWAI peripheral (SN65HVD230 transceiver)               |
| 6    | DAC SDA     | I/O       | MCP4728 I2C data (4.7kΩ pull-up to 3.3V at throttle box) |
| 7    | DAC SCL     | I/O       | MCP4728 I2C clock (4.7kΩ pull-up to 3.3V at throttle box) |
| 8    | Status LED  | Output    | Onboard WS2812 RGB LED (no external wiring)            |
| 10   | DPDT Relay  | Output    | 2N5551 base via 680R (10k pull-down), MY5NJ 24V coil   |
| 22   | F/R Forward | Input     | PC817 optocoupler, pull-up, active LOW                 |
| 23   | F/R Reverse | Input     | PC817 optocoupler, pull-up, active LOW                 |

### LED Behavior

| Color        | State                                                 |
| ------------ | ----------------------------------------------------- |
| Solid green  | Local Control state READY                             |
| Solid blue   | Local Control state ACTIVE                            |
| Solid red    | Local Control state NOT_READY                         |
| Solid yellow | Local Control state INIT/ENABLE                       |

## Wiring

Each subsection covers production (on-cart) and bench wiring for Control ESP32 interfaces where applicable. F/R hardware wiring in this revision is cart-only; on bench, use the F/R input bypass flag if cart switch wiring is not present. For CAN bus topology and motor wiring shared across all nodes, see [CAN Bus Wiring](../README.md#can-bus-wiring).

### Wiring Summary

| Interface                                 | Bench vs Production                                            |
| ----------------------------------------- | -------------------------------------------------------------- |
| CAN bus (SN65HVD230)                      | Same — see [root README](../README.md#can-bus-wiring)          |
| COM USB-C → Orin (UART0, 1 Mbaud)         | Same                                                           |
| Throttle system (MCP4728 + LM358 + MY5NJ) | Same hardware — bench: measure VOUTA directly (no op-amp needed) |
| Pedal ADC                                 | Same — bench divider floating reads ~0mV (safe default)        |
| F/R optocouplers (PC817)                  | Cart wiring only — production-style 48V switch wiring via 4.7k |
| Status LED (WS2812)                       | Same                                                           |

### CAN Bus (SN65HVD230)

GPIO 4 (TX) and GPIO 5 (RX) connect to a WAVESHARE SN65HVD230 CAN transceiver module via a 4-pin JST-PH connector. Control ESP32's Waveshare board has the onboard termination resistor **removed**. See the [root README](../README.md#can-bus-wiring) for the current CAN topology shared with Safety and the UIM2852 motors.

**ESP32-to-transceiver connector (4-pin JST-PH):**

| Pin | Wire Color | Signal       |
| --- | ---------- | ------------ |
| 1   | Yellow     | TXD (GPIO 4) |
| 2   | Green      | RXD (GPIO 5) |
| 3   | Red        | VCC (3.3V)   |
| 4   | Black      | GND          |

### Throttle Box Connectors

All throttle-related components (MCP4728 DAC, LM358 op-amp, MY5NJ DPDT relay, 2N5551, PC817 optocouplers, pedal ADC voltage divider) are housed in a separate throttle box perfboard powered from the cart's 24V rail. The Control ESP32 connects to the throttle box via four cables:

**ESP32-side connectors:**

| Label  | Connector    | Pin 1                           | Pin 2                       |
| ------ | ------------ | ------------------------------- | --------------------------- |
| **J1** | 2-pin JST-PH | DAC SDA (GPIO 6) GREEN          | DAC SCL (GPIO 7) WHITE      |
| **J2** | 2-pin JST-PH | DPDT Relay Base (GPIO 10) WHITE | Pedal ADC (GPIO 0) YELLOW   |
| **J3** | 2-pin JST-PH | F/R Fwd (GPIO 22) WHITE         | F/R Rev (GPIO 23) GREEN     |
| **J9** | 2-pin JST-PH | ESP32 GND BLACK                 | ESP32 3.3V RED              |

J9 carries ESP32 GND (signal reference for the throttle box GND bus) and ESP32 3.3V (MCP4728 VDD + I2C pull-up supply). On the perfboard, connect J9 3.3V to the MCP4728 VDD pin. Place a 100nF (0.1µF) ceramic decoupling cap from VDD to GND close to the chip (parallel bypass, not in series with the supply). Add **4.7kΩ pull-up resistors** from J9 3.3V to each I2C line (SDA and SCL) at the throttle box end of the cable — these are required for reliable I2C over the ~7ft cable run. Tie the MCP4728 LDAC pin to GND for immediate output updates.

**I2C pull-up placement:** The 4.7kΩ pull-ups go on the **perfboard** (throttle box end), not at the ESP32. Connect one 4.7kΩ resistor from the SDA pad (J1 Pin 1) to the 3.3V rail, and another 4.7kΩ from the SCL pad (J1 Pin 2) to the 3.3V rail. The ESP32's internal pull-ups (~45kΩ) are too weak for the cable capacitance. Twist the SDA and SCL wires together along the run to reduce EMI pickup.

**Cart-side connectors (on the throttle box):**

| Label  | Connector          | Pin 1                  | Pin 2                 | Pin 3 | Pin 4 |
| ------ | ------------------ | ---------------------- | --------------------- | ----- | ----- |
| **J4** | Anderson Powerpole | 24V+ RED               | GND BLACK             |       |       |
| **J5** | 2-pin JST-PH       | Pedal pot wiper GREEN  | Curtis Pin 3 YELLOW   |       |       |
| **J6** | 2-pin JST-PH       | Bypass wire A YELLOW   | Bypass wire B WHITE   |       |       |
| **J7** | 2-pin JST-PH       | Anti-arc signal YELLOW | Anti-arc return GREEN |       |       |
| **J8** | 2-pin JST-PH       | Buzzer supply BLUE     | Buzzer signal WHITE   |       |       |

J7/J8 wires are galvanically isolated 48V circuits — they do NOT connect to GND bus or any ESP32 signal. See [F/R Optocouplers (PC817)](#fr-optocouplers-pc817) for the exact cart-side connection points for J7 and J8.

**J5 internal throttle box connections:**

The MY5NJ DPDT relay Pole 1 on the throttle box perfboard switches the Curtis throttle input (Pin 3) between the manual pedal pot and the DAC/op-amp output. J5 connects the relay to the cart-side Curtis controller and pedal pot:

| J5 Pin | Cart-Side Signal                     | Throttle Box Internal Connection    |
| ------ | ------------------------------------ | ----------------------------------- |
| Pin 1  | Pedal pot wiper GREEN                | DPDT Relay Pole 1 NC (input)        |
| Pin 2  | Curtis Pin 3 (throttle input) YELLOW | DPDT Relay Pole 1 COM (output)      |

The relay switches Curtis Pin 3 between manual pedal pot (NC, de-energized) and DAC/op-amp output (NO, energized). In manual mode, the pot wiper signal passes through NC → COM → Curtis Pin 3. In autonomous mode, the op-amp output passes through NO → COM → Curtis Pin 3.

Curtis Pin 2 and B- are no longer connected to the throttle box — they remain in their original cart wiring (Pin 2 to pedal pot high terminal, B- to pedal pot low terminal). The DAC + op-amp generates the throttle voltage independently.

**Curtis controller pinout (1999 Club Car DS 48V):**

| Curtis Pin | Function                   | Used by J5? |
| ---------- | -------------------------- | ----------- |
| Pin 1      | Key switch                 | No          |
| Pin 2      | Pot high reference         | No (original pot wiring only) |
| Pin 3      | Pot wiper / throttle input | Yes (Pin 1) |
| B-         | Battery negative           | No (original pot wiring only) |

**J5 cart-side wiring procedure:**

The original cart wiring connects the pedal pot wiper directly to Curtis Pin 3. To intercept this signal for the DPDT relay:

1. **CUT** the original pedal pot wiper → Curtis Pin 3 wire:
    - Pedal pot wiper side of the cut → J5 Pin 1 (GREEN)
    - Curtis Pin 3 side of the cut → J5 Pin 2 (YELLOW)
2. Curtis Pin 2 and B- remain in their **original unmodified wiring** — no splices needed

### Throttle System (MCP4728 DAC + LM358 Op-Amp + MY5NJ Relay)

MCP4728 12-bit I2C DAC provides 4096 output levels (0-4095) on channel A using VDD as the voltage reference, producing 0-VDD (~3.3V) output. An LM358 op-amp in non-inverting configuration (gain ~2.47) scales this to 0-8.5V for the Curtis 1204 throttle input. The op-amp output connects to the MY5NJ DPDT relay Pole 1 NO terminal.

**Signal chain:**

```
ESP32 (I2C) → MCP4728 Ch.A (0-VDD, ~3.3V) → LM358 (gain 2.47) → 0-8.5V → Relay NO → Curtis Pin 3
```

**MCP4728 DAC circuit (GY-MCP4728 breakout board):**

The GY-MCP4728 breakout has onboard 8.2kΩ I2C pull-ups and a decoupling cap. External 4.7kΩ pull-ups are added in parallel for stronger drive over the ~7ft I2C cable run (effective combined pull-up: ~3kΩ). No external decoupling cap needed on VDD.

Board pin labels: V G CL DA L R A B C D

```
J9 3.3V ──┬── V (VDD)
           │
           ├── R3 (4.7kΩ) ── J1 Pin 1 (SDA) ── DA
           │
           └── R4 (4.7kΩ) ── J1 Pin 2 (SCL) ── CL

GND bus ──┬── G (VSS)
           │
           └── L (LDAC)

A (VOUTA) ── LM358 Pin 3 (non-inverting input)
R, B, C, D ── (floating)
```

| Board Pin | Full Name | Connect to |
|-----------|-----------|-----------|
| V | VDD | J9 3.3V rail |
| G | GND | GND bus |
| DA | SDA | J1 Pin 1 (GPIO 6) |
| CL | SCL | J1 Pin 2 (GPIO 7) |
| L | LDAC | GND bus (immediate update) |
| A | VOUTA | LM358 pin 3 |
| R | RDY/BSY | Floating |
| B, C, D | VOUTB-D | Floating |

| Component | Value | Connection |
|-----------|-------|-----------|
| R3 (4.7kΩ) | External I2C SDA pull-up | J9 3.3V → J1 Pin 1 (SDA), on perfboard |
| R4 (4.7kΩ) | External I2C SCL pull-up | J9 3.3V → J1 Pin 2 (SCL), on perfboard |

Internal DAC config (set by driver): channel A, VDD reference, 1x gain → 0-VDD (~3.3V) output range (~0.8 mV/step, 4096 levels). Power-on reset outputs 0V (safe default).

**LM358 op-amp circuit (non-inverting amplifier):**

```
MCP4728 VOUTA ── LM358 Pin 3 (IN+)

                    LM358 Pin 1 (OUT) ──┬── Relay Pole 1 NO
                                        │
                                        ├── R5 (100kΩ) ── GND    (failsafe)
                                        │
                                        └── R1 (10kΩ + 4.7kΩ) ──┬── LM358 Pin 2 (IN-)
                                                                 │
                                                                 R2 (10kΩ) ── GND

24V ──┬── LM358 Pin 8 (V+)
      │
      ├── C2 (100nF) ── GND             (decoupling, close to chip)
      │
      └── C3 (10µF) ── GND              (bulk decoupling, near chip)

GND ──── LM358 Pin 4 (GND)

LM358 Pins 5, 6, 7 ── (floating, unused channel B)
```

| Component | Value | Connection |
|-----------|-------|-----------|
| LM358 Pin 1 (OUT_A) | Op-amp output | Relay Pole 1 NO + R5 + R1 |
| LM358 Pin 2 (IN-_A) | Inverting input | Junction of R1 and R2 |
| LM358 Pin 3 (IN+_A) | Non-inverting input | MCP4728 VOUTA |
| LM358 Pin 4 (GND) | Ground | GND bus |
| LM358 Pin 8 (V+) | Positive supply | 24V rail |
| LM358 Pins 5-7 | Unused channel B | Floating |
| R1 (10kΩ + 4.7kΩ) | Feedback resistor | LM358 pin 1 → LM358 pin 2 |
| R2 (10kΩ) | Gain-set to GND | LM358 pin 2 → GND |
| R5 (100kΩ) | Failsafe pull-down | LM358 pin 1 → GND (pulls output to 0V if DAC loses power) |
| C2 (100nF ceramic) | HF decoupling | 24V → GND, close to LM358 pin 8 |
| C3 (10µF electrolytic) | Bulk decoupling | 24V → GND, near LM358 (stripe/minus to GND) |

Gain = 1 + (R1/R2) = 1 + (14.7kΩ / 10kΩ) = **2.47**. Max output = 3.3V × 2.47 = **8.15V** (varies slightly with VDD, software-capped at 8.5V).

**Safe state:** DAC level 0 produces 0V on the op-amp output — well within the Curtis controller's deadband, producing no movement.

**MY5NJ DPDT relay** (24V coil, driven via 2N5551 NPN transistor on GPIO 10):

- Pole 1 switches Curtis Pin 3 between manual pedal pot (NC) and op-amp output (NO)
- Pole 2 bypasses the pedal microswitch (NC = normal pedal operation, NO = bypassed)
- Both poles switch simultaneously (DPDT)
- De-energized = safe state (manual pedal control, no bypass)

**2N5551 relay driver circuit:**

```
J2 Pin 1 (GPIO 10) ── 680Ω ──┬── 2N5551 Base
                               │
                             10kΩ
                               │
                              GND

                    24V ──┬── MY5NJ Coil+ ── Coil- ──┬── 2N5551 Collector
                          │                           │
                          │    1N4007 (flyback)       │
                          └──── Cathode ── Anode ─────┘
                                (stripe)

                              2N5551 Emitter ── GND
```

| Component | Value | Connection |
|-----------|-------|-----------|
| 680Ω resistor | Base current limiter | J2 Pin 1 (GPIO 10) → 2N5551 base |
| 10kΩ resistor | Base pull-down (ensures relay off when GPIO floats) | 2N5551 base → GND |
| 2N5551 NPN | Relay coil switch | Collector → relay coil low side, Emitter → GND |
| MY5NJ coil | 24V DPDT relay | High side → 24V rail, Low side → 2N5551 collector |
| 1N4007 diode | Flyback clamp | Cathode (stripe) → 24V rail, Anode → 2N5551 collector |

When GPIO 10 goes HIGH, ~3.8mA flows through the 680Ω into the 2N5551 base, saturating the transistor and energizing the relay coil. The 10kΩ pull-down ensures the base is LOW (relay off) during ESP32 boot when GPIOs are floating. The 1N4007 clamps the inductive back-EMF spike when the coil de-energizes.

**Production:** Op-amp output feeds into the Curtis motor controller throttle input through the DPDT relay. The 4096 DAC levels span the Curtis controller's full throttle range with ~2.0 mV per step.

**Bench:** Without the op-amp, measure MCP4728 VOUTA directly (0-VDD, ~3.3V). With the op-amp, measure the output (0-8.5V). The relay will audibly click when energized — verifies GPIO 10 output.

### Pedal ADC

GPIO 0 (ADC1_CH0) reads the accelerator pedal position through a voltage divider (220kΩ/100kΩ). Threshold for "pressed" is 500 mV.

**Pedal ADC voltage divider circuit:**

```
Pedal pot wiper (0-8.5V) ── 220kΩ ──┬── GPIO 0 (ADC1_CH0)
                                      │
                                    100kΩ
                                      │
                                     GND
```

| Component | Value | Connection |
|-----------|-------|-----------|
| 220kΩ resistor | High-side divider | Pedal pot wiper (from relay Pole 1 NC / J5 Pin 2) → divider junction |
| 100kΩ resistor | Low-side divider | Divider junction → GND bus |

The divider scales the pedal pot's 0-8.5V range to 0-2.66V for the ESP32's 3.3V ADC. The divider junction connects to GPIO 0 via J2 Pin 2. The pedal pot wiper comes from the relay Pole 1 NC terminal (same wire as J5 Pin 2, pedal pot wiper side) — this reads the pedal position regardless of relay state since the divider taps before the relay.

Note: the divider has high source impedance (~69kΩ = 220k||100k). The ESP32-C6 ADC handles this but readings may have slight noise. The 8-sample oversampling in the driver smooths this out.

**Production:** Pedal pressed produces >500 mV; released reads ~0 mV.

**Bench:** With no pedal connected, the floating input reads ~0 mV (treated as "not pressed" — safe default).

### F/R Optocouplers (PC817)

The F/R decode is wired for a **1999 Club Car DS 48V** forward/reverse switch assembly. The switch has three microswitches; two are used for direction sensing:

| Microswitch    | GPIO              | Active When                                                     | Circuit Position                                       |
| -------------- | ----------------- | --------------------------------------------------------------- | ------------------------------------------------------ |
| Anti-arcing    | 22 (forward_gpio) | Forward **and** Reverse (opens solenoid coil during transition) | High-side switch (output → solenoid coil → B-)         |
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

| Switch State | OUTPUT Voltage | LED | GPIO 22 | Software   |
| ------------ | -------------- | --- | ------- | ---------- |
| Closed       | ~Supply        | ON  | LOW     | active ✓   |
| Open         | ~B- (via coil) | OFF | HIGH    | inactive ✓ |

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

| Switch State | Microswitch INPUT                | LED Voltage          | LED | GPIO 23 | Software   |
| ------------ | -------------------------------- | -------------------- | --- | ------- | ---------- |
| Closed       | ~B- (via switch)                 | Supply − B- ≈ 48V    | ON  | LOW     | active ✓   |
| Open         | ~Supply (via buzzer, no current) | Supply − Supply ≈ 0V | OFF | HIGH    | inactive ✓ |

**Bench:** If cart F/R switch wiring is absent on bench, disconnected optocouplers read NEUTRAL (both HIGH). Pre-bypass, NEUTRAL passes preconditions (not-in-reverse), so the system can reach READY and ENABLE without the F/R sensor bypass. Use `CONFIG_BYPASS_INPUT_FR_SENSOR` only if you need to force FORWARD readings for testing post-bypass ACTIVE behavior.

### Status LED (WS2812)

GPIO 8 is configured as an RMT TX output driving the onboard WS2812 RGB status LED data line (no external wiring required). LED color is determined by software state logic, not GPIO level — the GPIO serves only as the RMT data output.

## Components

Local (`control-esp32/components/`):

| Component           | Description                                                                      |
| ------------------- | -------------------------------------------------------------------------------- |
| `dac_mcp4728`       | MCP4728 I2C DAC for throttle level (12-bit, 4096 levels)                          |
| `motor_uim2852`     | UIM2852CA motor protocol, dispatch, exec, setup, and diagnostics (unified)        |
| `relay_dpdt_my5nj`  | MY5NJ DPDT relay driver (24V coil via 2N5551 NPN transistor)                      |
| `adc_12bitsar`      | Dedicated ESP32-C6 12-bit SAR ADC read/calibration helper                         |
| `optocoupler_pc817` | Dedicated F/R PC817 decode + debounce helper                                      |

Local (`control-esp32/main/logic/`):

| Component       | Description                                      |
| --------------- | ------------------------------------------------ |
| `control_logic` | Extracted state machine decision logic (tested)  |
| `control_types` | Shared type definitions for control logic        |

Shared (`common/`):

| Component                  | Location             | Description                                                              |
| -------------------------- | -------------------- | ------------------------------------------------------------------------ |
| `can_twai`                 | `common/drivers`     | CAN bus (TWAI) driver wrapper                                            |
| `led_ws2812`               | `common/drivers`     | Raw WS2812 RGB transport driver                                          |
| `can_protocol`             | `common/protocol`    | Shared CAN message struct + encode/decode                                |
| `orin_link_protocol`       | `common/protocol`    | Typed message helpers for the Orin UART links                            |
| `usb_serial_link`          | `common/services`    | Framed UART0 transport used by both Control and Safety for the Orin link |
| `status_led`               | `common/services`    | Node-state → RGB color policy adapter                                    |
| `node_support`             | `common/services`    | Task watchdog + boot logging helpers                                     |
| `can_runtime`              | `common/services`    | CAN TX/RX task plumbing and component-health tracking                    |
| `heartbeat_monitor`        | `common/services`    | Node liveness tracking (per-source timeout masks)                        |

### Hardware Detection Limitations

Not all components can detect physical absence at init or runtime. Components that are output-only GPIOs have no feedback path from the external hardware -- the ESP32 drives pins but receives no acknowledgment.

| Component               | Detects absence? | Why                                                                                                                                                                                                                                                                                                                                                                                           |
| ----------------------- | ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `dac_mcp4728`           | Yes              | Every I2C write is followed by a read-back verification. I2C NACK detects disconnected device immediately.                                                                                                                                                                                                                                          |
| `relay_dpdt_my5nj`      | No               | Output-only GPIO. Same readback pattern as before — verifies the MCU register, not whether the relay/transistor is physically present.                                                                                                                                                                                                                                                        |
| `adc_12bitsar`          | No               | ADC init configures an internal ESP32 peripheral. A floating/disconnected pin reads ~0 mV, which is indistinguishable from "pedal not pressed." Safe direction (override never triggers), but pedal override detection is silently disabled.                                                                                                                                                  |
| `optocoupler_pc817`     | No (pre-bypass)  | Pull-ups + active-low signaling: disconnected = both HIGH = NEUTRAL. Pre-bypass, NEUTRAL is expected (FORWARD reads as NEUTRAL when anti-arc switch can't conduct). Hardware absence is indistinguishable from "cart in FORWARD/NEUTRAL." Post-bypass (ACTIVE), the full truth table is readable and a disconnected optocoupler would read NEUTRAL, which zeroes throttle but does not fault. |
| `motor_uim2852`          | Yes             | Init performs a CAN handshake (MO=0 disable). No response = timeout = init failure, triggering FAULT.                                                                                                                                                                                                                                                                                         |

## Throttle Control

The throttle system uses an MCP4728 DAC + LM358 op-amp to provide 4096 output levels (0-4095) for continuous throttle level control:

- Level 0: Off (0V output, in Curtis deadband, no movement)
- Levels 0-~800: Within Curtis deadband, no wheel movement
- Level ~800: First wheel movement (deadband exit, ~1.6V at op-amp output)
- Levels ~800-4095: Proportional speed control (~2.0 mV per step)
- Level 4095: Maximum throttle (~8.24V output)
- Slew rate limited: max 200 DAC steps per 100ms

Enable sequence (READY -> ACTIVE):

1. Set DAC output to level 0
2. Wait 200ms enable dwell
3. Enable steering and braking motors
4. Configure and arm PT FIFO mode for both motors
5. Braking PT targets are bounded per update so large planner jumps are applied gradually
6. Energize DPDT relay (GPIO 10) — both poles switch: throttle source to DAC/op-amp, pedal bypass active
7. Enable DAC autonomous mode

In ACTIVE, both steering and braking are fed as PT streams. Steering repeats the latest clamped target to keep the FIFO primed. Braking also repeats its latest target, but each commanded target step is additionally bounded before it is queued so abrupt planner changes do not turn into a single large brake move.

## Test Bypasses

Compile-time Kconfig flags for bench testing without the full system connected. All default to off (disabled). Enable via `idf.py menuconfig` under **Test Bypasses** (top-level), or add to `sdkconfig.defaults`:

| Flag                                         | Effect                                                                                   |
| -------------------------------------------- | ---------------------------------------------------------------------------------------- |
| `CONFIG_BYPASS_SAFETY_TARGET_MIRROR`         | Force target_state to ACTIVE (ignore Safety target mirror)                               |
| `CONFIG_BYPASS_SAFETY_ESTOP_MIRROR`          | Force Safety mirror channels clear (`safety_fault_flags=NONE`, `safety_stop_flags=NONE`) |
| `CONFIG_BYPASS_SAFETY_LIVENESS_CHECKS`       | Ignore Safety heartbeat timeout (test without Safety on bus)                             |
| `CONFIG_BYPASS_PLANNER_COMMAND_INPUTS`       | Force Orin Planner command inputs to zero throttle/steering/braking                      |
| `CONFIG_BYPASS_PLANNER_COMMAND_STALE_CHECKS` | Disable Orin Planner command timeout/stale checks                                        |
| `CONFIG_BYPASS_INPUT_PEDAL_ADC`              | Skip pedal ADC readings (always not pressed, always re-armed)                            |
| `CONFIG_BYPASS_INPUT_FR_SENSOR`              | Force F/R state to FORWARD (skip optocoupler channel reading)                            |
| `CONFIG_BYPASS_ACTUATOR_THROTTLE`            | Skip DAC and DPDT relay control                                                          |
| `CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING`    | Skip steering motor (node 7) init/configure/commands                             |
| `CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING`     | Skip braking motor (node 6) init/configure/commands                              |
| `CONFIG_BYPASS_MOTOR_UIM2852_LIMITS_STEERING`     | Skip programming steering LM[0-7] software limits during motor configure            |
| `CONFIG_BYPASS_MOTOR_UIM2852_LIMITS_BRAKING`      | Skip programming braking LM[0-7] software limits during motor configure             |

## Motor Configuration

Compile-time Kconfig defaults for the UIM2852CA motors, programmed during `motor_uim2852 init sequence`. Menu: `idf.py menuconfig` → **Motor Limits**.

| Menu         | Flag                               | Description                                                                   |
| ------------ | ---------------------------------- | ----------------------------------------------------------------------------- |
| **Steering** | `CONFIG_STEERING_LM0_MAX_SPEED`    | Max working speed (pulses/sec)                                                |
|              | `CONFIG_STEERING_LM1_LOWER_LIMIT`  | Lower working position limit (pulses, signed)                                 |
|              | `CONFIG_STEERING_LM2_UPPER_LIMIT`  | Upper working position limit (pulses, signed)                                 |
|              | `CONFIG_STEERING_LM7_MAX_ACCEL`    | Max acceleration (pulses/sec²)                                                |
|              | `CONFIG_STEERING_AC_DEFAULT_ACCEL` | Default AC acceleration                                                       |
|              | `CONFIG_STEERING_DC_DEFAULT_DECEL` | Default DC deceleration                                                       |
|              | `CONFIG_STEERING_SD_STOP_DECEL`    | Emergency SD stop deceleration                                                |
| **Braking**  | `CONFIG_BRAKING_LM0_MAX_SPEED`     | Max working speed (pulses/sec)                                                |
|              | `CONFIG_BRAKING_LM1_LOWER_LIMIT`   | Lower working position limit (pulses, signed)                                 |
|              | `CONFIG_BRAKING_LM2_UPPER_LIMIT`   | Upper working position limit (pulses, signed)                                 |
|              | `CONFIG_BRAKING_LM7_MAX_ACCEL`     | Max acceleration (pulses/sec²)                                                |
|              | `CONFIG_BRAKE_RELEASE_POSITION`    | Target position used when the braking motor should be fully released          |
|              | `CONFIG_BRAKING_AC_DEFAULT_ACCEL`  | Default AC acceleration                                                       |
|              | `CONFIG_BRAKING_DC_DEFAULT_DECEL`  | Default DC deceleration                                                       |
|              | `CONFIG_BRAKING_SD_STOP_DECEL`     | Emergency SD stop deceleration                                                |
| **PT FIFO**  | `CONFIG_MOTOR_UIM2852_PT_FRAME_TIME_MS`  | PT-mode frame interval in ms (MP[4])                                          |
|              | `CONFIG_MOTOR_UIM2852_PT_LOW_WATER_MARK` | FIFO low-water threshold for `PVT_FIFO_EMPTY` notifications (MP[5])           |
|              | `CONFIG_BRAKING_PT_STEP_LIMIT_PULSES_PER_500MS` | Max braking PT target change per 500 ms of commanded motion |

## Control Test Mode

`CONFIG_CONTROL_TEST_MODE` replaces the normal Orin+CAN control loop with a standalone bench-test task for one selected actuator. Choose the actuator with `CONFIG_CONTROL_TEST_ACTUATOR_*`.

- `CONFIG_CONTROL_TEST_ACTUATOR_THROTTLE`: validates throttle hardware (DAC/op-amp/relay/F/R/pedal). Commands: `0-4095 + space/enter` to set DAC, `r` to toggle the DPDT relay, `q` to quit. Requires F/R not in Reverse to arm; disables DAC if F/R moves to Reverse or the pedal is pressed.
- `CONFIG_CONTROL_TEST_ACTUATOR_STEERING`: validates the steering motor over CAN/PT mode. Commands: `0-720 + space/enter`, `q` to quit.
- `CONFIG_CONTROL_TEST_ACTUATOR_BRAKING`: validates the braking motor over CAN/PT mode. Commands: `0-3 + space/enter`, `q` to quit.

Only one actuator is driven in a test build. `CONFIG_CONTROL_TEST_MODE` cannot be enabled alongside `CONFIG_PRODUCTION_BUILD`.

## Debug Logging

Compile-time Kconfig flags for verbose logging. Enable via `idf.py menuconfig` under **Debug Logging** (top-level). Shared options (Transport, Health) live in `common/kconfig/` and are `rsource`'d by both firmware Kconfigs.

### Transport

| Flag | Default | Effect |
| --- | --- | --- |
| `CONFIG_LOG_TRANSPORT_CAN_FRAMES` | off | Log raw CAN frames (TX and RX) at TWAI driver layer |
| `CONFIG_LOG_TRANSPORT_CAN_FRAMES_MOTORS_ONLY` | off | Log motor frames only (suppress standard) |
| `CONFIG_LOG_TRANSPORT_CAN_FRAMES_SUPPRESS_SAFETY_HB` | off | Suppress Safety heartbeat from raw frame log |
| `CONFIG_LOG_TRANSPORT_CAN_FRAMES_SUPPRESS_CONTROL_HB` | off | Suppress Control heartbeat from raw frame log |
| `CONFIG_LOG_TRANSPORT_CAN_HEARTBEAT_TX` | off | Log every CAN heartbeat TX |
| `CONFIG_LOG_TRANSPORT_CAN_HEARTBEAT_RX` | off | Log every CAN heartbeat RX |
| `CONFIG_LOG_TRANSPORT_ORIN_LINK_FRAMING` | off | Log Orin UART sync/resync/malformed frame events |
| `CONFIG_LOG_TRANSPORT_ORIN_PLANNER_COMMAND_RX` | off | Log every Orin Planner command RX |

### Health & Recovery

| Flag | Default | Effect |
| --- | --- | --- |
| `CONFIG_LOG_HEALTH_COMPONENT_CHANGES` | on | Log component LOST/REGAINED edge transitions |
| `CONFIG_LOG_HEALTH_HEARTBEAT_MONITOR_CHANGES` | on | Log Safety heartbeat monitor lost/regained |
| `CONFIG_LOG_HEALTH_CAN_RECOVERY` | off | Log CAN bus recovery events (stop/start, reinstall, bus-off) |
| `CONFIG_LOG_HEALTH_RETRY_TWAI` | off | Log TWAI retry attempts |
| `CONFIG_LOG_HEALTH_RETRY_COMPONENTS` | off | Log all component retry attempts (DAC, relay, pedal, F/R, motors) |

Retries are unbounded for failed required components, paced at 500ms intervals.

### Control Logic

| Flag | Default | Effect |
| --- | --- | --- |
| `CONFIG_LOG_CONTROL_STATE_CHANGES` | on | Log control state transitions and transition reasons |
| `CONFIG_LOG_CONTROL_FAULT_CHANGES` | off | Log control fault code changes |
| `CONFIG_LOG_CONTROL_ENABLE_SEQUENCE` | off | Log enable/disable sequence steps (start, complete, abort) |
| `CONFIG_LOG_CONTROL_OVERRIDE_CHANGES` | off | Log driver override trigger events |
| `CONFIG_LOG_CONTROL_THROTTLE_CHANGES` | off | Log throttle level changes (edge-triggered) |
| `CONFIG_LOG_CONTROL_PRECONDITION_BLOCKED` | on | Log when enable preconditions block readiness/ENABLE |
| `CONFIG_LOG_CONTROL_SAFETY_TARGET_CHANGES` | on | Log received Safety target/fault changes |

### Inputs

| Flag | Default | Effect |
| --- | --- | --- |
| `CONFIG_LOG_INPUT_PEDAL_ADC_ERRORS` | off | Log pedal ADC read errors (I/O failures, out-of-range) |
| `CONFIG_LOG_INPUT_PEDAL_CHANGES` | off | Log pedal threshold crossed / re-armed transitions |
| `CONFIG_LOG_INPUT_FR_DEBOUNCE` | off | Log F/R switch debounce transitions |
| `CONFIG_LOG_INPUT_FR_STATE_CHANGES` | off | Log debounced F/R state changes |

### Actuators

| Flag | Default | Effect |
| --- | --- | --- |
| `CONFIG_LOG_ACTUATOR_DAC_WRITE` | off | Log every MCP4728 I2C register write with DAC level |
| `CONFIG_LOG_ACTUATOR_DPDT_RELAY_CHANGES` | off | Log DPDT relay energize/de-energize |
| `CONFIG_LOG_ACTUATOR_MOTOR_UIM2852_INIT` | off | Log motor init/configure handshake |
| `CONFIG_LOG_ACTUATOR_MOTOR_UIM2852_ENABLE_CHANGES` | off | Log motor MO enable/disable transitions |
| `CONFIG_LOG_ACTUATOR_MOTOR_UIM2852_COMMAND_TX` | off | Log motor PT feed commands sent over CAN |
| `CONFIG_LOG_ACTUATOR_MOTOR_UIM2852_MOTION_TX` | off | Log motor motion commands (PA/PR/ST) |
| `CONFIG_LOG_ACTUATOR_MOTOR_UIM2852_RX` | off | Log parsed motor CAN RX frames |

## Build

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p PORT flash monitor
```
