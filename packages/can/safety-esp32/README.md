# safety-esp32

ESP-IDF firmware for the Safety ESP32-C6. Acts as the **system state authority** — it is the only node that can advance the system forward (NOT_READY -> READY -> ENABLE -> ACTIVE). Monitors all safety inputs, controls the 24V power relay, and broadcasts the system target state via a unified heartbeat.

## State Machine

Safety owns the system target state and broadcasts it as `state` in heartbeat `0x100`. All nodes observe Safety's heartbeat to track the system target. Safety commands `NOT_READY`/`READY`/`ENABLE`/`ACTIVE`; Planner and Control report local causes via heartbeat `fault_flags` (issues) and `stop_flags` (non-fault stop inputs). Safety advances only when nodes are healthy and readiness conditions are met, and retreats on hard safety conditions.

### Target Transitions

Safety evaluates transitions in this order:

1. **INIT dwell gate**
    - Stay `INIT` until dwell expires.
    - On dwell expiry, go to `READY` when both nodes are alive and `READY` with no stop/fault active; otherwise `NOT_READY`.
2. **Problem retreat**
    - From any non-INIT target, retreat to `NOT_READY` on problems (stop/fault active or node timeout/liveness loss).
3. **Autonomy hold drop behavior (ENABLE/ACTIVE only)**
    - If Planner/Orin drops autonomy hold (`NODE_STATUS_FLAG_AUTONOMY_REQUEST`), retreat to `READY` when both nodes are still `READY`; otherwise `NOT_READY`.
4. **Forward path**
    - `NOT_READY -> READY`: both nodes report `READY`, both alive, no stop/fault active.
    - `READY -> ENABLE`: Planner/Orin request edge is latched in the accept window.
    - `ENABLE -> ACTIVE`: both nodes report `ENABLE` and set `NODE_STATUS_FLAG_ENABLE_COMPLETE`, no stop/fault active.
    - `ACTIVE` stays active only while both nodes report `ACTIVE` (a short active-entry grace allows transient `ENABLE`).

Note: During `INIT` dwell, Safety intentionally holds target state at `INIT` for deterministic startup sequencing.

Autonomy request handling is one-shot for entry and level-hold for run:

- Safety latches a rising request edge only while in the accept window (`READY`, clear, both nodes alive, both node states `READY`).
- Safety consumes it when entering `ENABLE`.
- Safety requires request drop before re-arm.
- If request drops during `ENABLE`/`ACTIVE`, Safety retreats out of autonomy.

## Safety Logic

The Safety ESP32 continuously monitors:

1. **Hardware stop inputs**: Push button (HB2-ES544), RF remote (EV1527)
2. **Obstacle detection**: Ultrasonic sensor A02YYUW (threshold: 2000mm ~6.6ft)
3. **Node liveness**: Planner and Control heartbeats (timeout: 500ms)
4. **Node causes**: Planner and Control heartbeat cause channels
    - `fault_flags`: unexpected issues (hardware/software faults)
    - `stop_flags`: operator/manual stop inputs (throttle/reverse/steering/braking, app request)

**Power relay behavior:**

- ENABLED only in target `ENABLE` or `ACTIVE`, and only when all stop/fault inputs are clear
- DISABLED in `INIT`/`NOT_READY`/`READY`, or immediately when any stop or fault condition is detected

**Input debounce:**

- Push-button and RF remote engage is immediate (safety-critical, no debounce)
- Push-button and RF remote disengage requires 3 consecutive clear reads (~150ms at 50ms loop) to filter contact bounce
- Ultrasonic obstacle engage requires 4 consecutive close reads (~200ms) to filter noise/multipath
- Ultrasonic obstacle disengage requires 6 consecutive clear reads (~300ms) to filter ghost echoes
- Ultrasonic health transitions require 3 consecutive agreeing samples (~150ms) to prevent flap around the 500ms timeout boundary

**Cause channels produced by Safety:**

1. `stop_flags` bitmask for non-fault stop inputs (push button, RF remote, ultrasonic obstacle, planner app stop, control operator stop)
2. `fault_flags` bitmask for issues/timeouts (ultrasonic unhealthy, planner issue/timeout, control issue/timeout, relay unavailable)

## CAN Messages

### Receives

| ID    | Name              | Description                                                       |
| ----- | ----------------- | ----------------------------------------------------------------- |
| 0x110 | PLANNER_HEARTBEAT | Planner alive (seq, state, fault_flags, stop_flags, status_flags) |
| 0x120 | CONTROL_HEARTBEAT | Control alive (seq, state, fault_flags, stop_flags, status_flags) |

### Sends

| ID    | Name                  | Rate                              | Description                                                                                           |
| ----- | --------------------- | --------------------------------- | ----------------------------------------------------------------------------------------------------- |
| 0x100 | SAFETY_HEARTBEAT      | 100ms + immediate on state change | System target state + Safety `fault_flags`/`stop_flags` (same `node_heartbeat_t` format as all nodes) |
| 0x101 | SAFETY_BATTERY_STATUS | 1 Hz (every 10th heartbeat tick)  | Pack voltage, current, SOC, and battery flags (`battery_status_t`)                                    |

Safety's heartbeat `state` field = system target state (`NODE_STATE_*`). Its `fault_flags` field = Safety fault bitmask (`NODE_FAULT_SAFETY_*`), and `stop_flags` field = non-fault stop bitmask (`NODE_STOP_*`).

### Heartbeat Monitoring

| Node    | Timeout | Tracked Fields                                         |
| ------- | ------- | ------------------------------------------------------ |
| Planner | 500ms   | sequence, state, fault_flags, stop_flags, status_flags |
| Control | 500ms   | sequence, state, fault_flags, stop_flags, status_flags |

## Safety Fault Bitmask (`NODE_FAULT_SAFETY_*`)

The `fault_flags` byte in Safety heartbeat is a bitmask. Multiple fault bits may be set at once.

| Bit | Code | Constant                    | Trigger                                      |
| --- | ---- | --------------------------- | -------------------------------------------- |
| -   | 0x00 | NONE                        | No fault                                     |
| 0   | 0x01 | SAFETY_ULTRASONIC_UNHEALTHY | Ultrasonic sensor unhealthy / timeout        |
| 1   | 0x02 | SAFETY_PLANNER_ISSUE        | Planner `fault_flags` in planner fault range |
| 2   | 0x04 | SAFETY_PLANNER_TIMEOUT      | No Planner heartbeat for 500ms               |
| 3   | 0x08 | SAFETY_CONTROL_ISSUE        | Control `fault_flags` in control fault range |
| 4   | 0x10 | SAFETY_CONTROL_TIMEOUT      | No Control heartbeat for 500ms               |
| 5   | 0x20 | SAFETY_RELAY_UNAVAILABLE    | Safety relay path unavailable                |

## Pin Configuration

| GPIO | Function              | Direction | Notes                                              |
| ---- | --------------------- | --------- | -------------------------------------------------- |
| 0    | Battery Voltage       | Input     | ADC1_CH0, 220kΩ/10kΩ divider (pack voltage)        |
| 1    | Battery Current       | Input     | ADC1_CH1, HTFS-200-P via 6.8kΩ/10kΩ divider        |
| 2    | Power Relay           | Output    | Active HIGH, SRD-05VDC-SL-C module                 |
| 4    | CAN TX                | Output    | TWAI peripheral (SN65HVD230 transceiver)           |
| 5    | CAN RX                | Input     | TWAI peripheral (SN65HVD230 transceiver)           |
| 6    | Push Button HB2-ES544 | Input     | Pull-up, active HIGH (NC switch opens on press)    |
| 7    | RF Remote EV1527      | Input     | Pull-up, active HIGH (NC relay output, COM to GND) |
| 8    | Status LED            | Output    | Onboard WS2812 RGB LED (no external wiring)        |
| 10   | Ultrasonic A02YYUW TX | Output    | UART1 TX (sensor RX, mode select)                  |
| 11   | Ultrasonic A02YYUW RX | Input     | UART1 RX, 9600 baud (sensor TX)                    |

### LED Behavior

| Color        | State                                         |
| ------------ | --------------------------------------------- |
| Solid green  | Target state READY (no stop/fault active)     |
| Solid orange | Target state ENABLE                           |
| Solid blue   | Target state ACTIVE                           |
| Solid red    | Target state NOT_READY or local fault overlay |

## Wiring

Each subsection covers production (on-cart) and bench wiring for Safety ESP32 interfaces. For CAN bus topology shared across all nodes, see [CAN Bus Wiring](../README.md#can-bus-wiring).

### Wiring Summary

| Interface                    | Bench vs Production                                         |
| ---------------------------- | ----------------------------------------------------------- |
| CAN bus (SN65HVD230)         | Same — see [root README](../README.md#can-bus-wiring)       |
| Push button (HB2-ES544)      | Same                                                        |
| RF remote (EV1527)           | **Different** — receiver needs separate 12V supply on bench |
| Ultrasonic (A02YYUW)         | Same                                                        |
| Power relay (SRD-05VDC)      | Same hardware — bench relay switches with no 24V load       |
| Battery monitor (HTFS-200-P) | Same — voltage divider and current sensor wired identically |
| Status LED (WS2812)          | Same                                                        |

### Ground Distribution (Safety ESP32)

Safety wiring uses all three ESP32 GND pins, grouped by noise profile:

- **GND pin A (analog/battery monitor):** HTFS `0V`, voltage divider 10kΩ low-side, current output divider 10kΩ low-side — keeping the sensor and ADC divider grounds on the same pin cancels common-mode offset
- **GND pin B (digital/power return):** ESP32 5V supply return, CAN transceiver, push button, ultrasonic
- **GND pin C (noisy/relay):** Power relay module and RF receiver relay return

Power rail distribution across ESP32 pins:

- **5V pin 1:** ESP32 power supply input (from 5V fuse block)
- **5V pin 2 (splice):** Power relay VCC + HTFS U_C + Ultrasonic VCC
- **3.3V pin 1:** CAN transceiver VCC

Use a small harness splice/star point for each branch (or a small terminal block), then run one wire per ESP32 GND pin. Do not stack multiple wires into a single ESP32 header hole.

### CAN Bus (SN65HVD230)

GPIO 4 (TX) and GPIO 5 (RX) connect to a WAVESHARE SN65HVD230 CAN transceiver module via a 4-pin JST-PH connector. Safety ESP32's Waveshare board has onboard termination **enabled** (120 ohm). See the [root README](../README.md#can-bus-wiring) for the full 5-node bus topology including stepper motors.

**ESP32-to-transceiver connector (4-pin JST-PH):**

| Wire Color | Signal                      |
| ---------- | --------------------------- |
| Black      | GND (digital branch, pin B) |
| Red        | VCC (3.3V)                  |
| Yellow     | TXD (GPIO 4)                |
| Green      | RXD (GPIO 5)                |

### Push Button Stop (HB2-ES544)

GPIO 6 reads a normally-closed (NC) mxuteek HB2-ES544 22mm emergency stop push button. Internal pull-up enabled, active HIGH — the NC switch holds GPIO LOW in normal operation; pressing the button opens the switch and GPIO reads HIGH (stop active). NC design is fail-safe: a broken wire also triggers stop.

**Wiring (2-pin):**

| Wire Color | ESP32 Pin                   | Button Terminal |
| ---------- | --------------------------- | --------------- |
| Black      | GND (digital branch, pin B) | COM terminal    |
| White      | GPIO 6                      | NC terminal     |

**Same wiring for bench and production.** On the bench, use the same HB2-ES544 switch or any NC momentary button wired between GPIO 6 and GND (pull-up keeps it HIGH when open).

### RF Remote Stop (EV1527)

GPIO 7 reads the NC (normally closed) relay output of a DieseRC 433MHz RF receiver module (DC 12V 1CH relay, EV1527 learning code). Internal pull-up enabled, active HIGH. The receiver's onboard relay NC and COM terminals connect to GPIO 7 and ESP32 GND respectively. When the remote button is not pressed (relay de-energized), NC-COM is closed, pulling GPIO 7 to GND (LOW, safe). When pressed (relay energizes), NC-COM opens, internal pull-up pulls GPIO 7 HIGH (stop active). This is fail-safe: receiver power loss or broken wire also opens NC-COM, triggering stop.

Use two 2-pin connectors instead of one mixed 4-pin harness:

**Connector A (RF power, 2-pin):**

| Wire Color | From           | To                 |
| ---------- | -------------- | ------------------ |
| Black      | 12V supply (-) | Receiver GND       |
| Red        | 12V supply (+) | Receiver VCC (12V) |

**Connector B (RF stop signal, 2-pin):**

| Wire Color | From                            | To                    |
| ---------- | ------------------------------- | --------------------- |
| Black      | GND (noisy/relay branch, pin C) | Receiver COM terminal |
| White      | ESP32 GPIO 7                    | Receiver NC terminal  |

Receiver power GND and ESP32 signal GND are spliced together in the noisy/relay ground branch (pin C) so NC/COM has a valid reference.

**Production:** The receiver module is powered from 12V on the cart and mounted near the Safety ESP32.

**Bench:** Same wiring. The receiver module requires its own 12V supply (bench power supply or 12V adapter).

### Ultrasonic Sensor (A02YYUW)

UART1: GPIO 10 (TX to sensor RX/mode select), GPIO 11 (RX from sensor TX). 9600 baud, 4-byte frames. The A02YYUW is a waterproof ultrasonic rangefinder powered from 5V (pin 2 splice) with a direct 3.3V-compatible UART output — no level shifting or isolation needed.

**Wiring (4-pin):**

| Wire Color | ESP32 Pin                   | Sensor Wire             |
| ---------- | --------------------------- | ----------------------- |
| Red        | 5V (pin 2 splice)           | VCC (red wire)          |
| Black      | GND (digital branch, pin B) | GND (black wire)        |
| Blue       | GPIO 10 (TX)                | Sensor RX (mode select) |
| Green      | GPIO 11 (RX)                | Sensor TX (data output) |

Note the TX/RX crossover: ESP32 TX -> Sensor RX, ESP32 RX -> Sensor TX.

**Same wiring for bench and production.** The sensor connects directly to the ESP32 UART pins in both environments. On the bench, point the sensor at a wall or object to test obstacle detection. Stop threshold is 2000mm (~6.6 ft), clear threshold is 2500mm (~8.2 ft).

### Power Relay (SRD-05VDC-SL-C)

GPIO 2 drives an AEDIKO 1-channel 5V relay module (SRD-05VDC-SL-C, optocoupler-isolated trigger). The relay's NO (normally open) terminal switches the 24V autonomous power rail that feeds the stepper motor drivers. The output latch is preloaded LOW before GPIO configuration to keep the relay de-energized on boot/reset.

**Wiring (3-pin):**

| Wire Color | From                            | To                        |
| ---------- | ------------------------------- | ------------------------- |
| White      | ESP32 GPIO 2                    | Relay module IN (trigger) |
| Red        | ESP32 5V pin 2 (splice)         | Relay module VCC          |
| Black      | GND (noisy/relay branch, pin C) | Relay module GND          |

**Production:** Relay NO terminal connects the 24V power rail to the stepper motor drivers. Energized = 24V flows to motors (vehicle can move). De-energized = 24V cut (fail-safe stop).

**Bench:** Same hardware. Relay clicks audibly when toggled — useful for verifying GPIO 2 output. Without a 24V load, the relay opens/closes its contacts with no effect.

### Status LED (WS2812)

GPIO 8 is configured as an RMT TX output driving the onboard WS2812 RGB status LED data line (no external wiring required). LED color is determined by software state logic, not GPIO level — the GPIO serves only as the RMT data output.

### Battery Monitor (LEM HTFS-200-P)

GPIO 0 and GPIO 1 read pack voltage and current via ADC1. Non-safety-critical — failure does not trigger stop; it only affects SOC reporting on CAN (`0x101 SAFETY_BATTERY_STATUS`). The battery monitor retries initialization in the background if the ADC fails at startup.

**Voltage sensing (3-wire, divider split across run):**

The 220kΩ resistor sits at the rear bus bar end; the 10kΩ resistor sits at the ESP32 end. This way the ~7ft yellow wire carries only the divided junction voltage (~2.2V at 0.22mA) rather than the full 48V, eliminating fault/fire risk on the long run.

| Wire Color | From                           | To                                 |
| ---------- | ------------------------------ | ---------------------------------- |
| Red        | Rear +48V distribution bus bar | 220kΩ resistor (rear, high-side)   |
| Yellow     | 220kΩ output (rear)            | 10kΩ resistor (ESP32 end) + GPIO 0 |
| Black      | 10kΩ resistor (low-side)       | GND (analog branch, pin A)         |

220kΩ (high-side, rear) and 10kΩ (low-side, ESP32 end) form a 1:23 divider, scaling 42–51.2V pack voltage to ~1.8–2.2V for the 12-bit ADC (3.3V reference). Voltage is measured from the rear distribution bus bars near the batteries for the most accurate open-circuit voltage reading during SOC recalibration.

**Current sensing (5-wire):**

| Wire Color | From                       | To                                             |
| ---------- | -------------------------- | ---------------------------------------------- |
| Red        | ESP32 5V pin 2 (splice)    | HTFS `U_C` (supply)                            |
| Black      | GND (analog branch, pin A) | HTFS `0V`                                      |
| —          | Pack main power cable      | Through HTFS 22mm aperture (primary conductor) |
| Blue       | HTFS `U_out`               | 6.8kΩ resistor (high-side)                     |
| Yellow     | 6.8kΩ/10kΩ junction        | ESP32 GPIO 1 (ADC1_CH1)                        |

The HTFS-200-P is a hall-effect sensor — the pack power cable passes through the 22mm aperture with no electrical contact to the sense circuit. Output is VCC/2 (2.5V) at 0A with 6.25 mV/A sensitivity (200A nominal, 300A measuring range). The 6.8kΩ/10kΩ output divider scales the sensor output by ~0.595x before it reaches the ESP32 ADC input. The divider resistors sit at the ESP32 end; 10kΩ low-side to ESP32 GND pin A (analog branch). HTFS 0V also connects to pin A, so the sensor and ADC divider grounds share the same reference, cancelling common-mode offset. HTFS U_C is spliced into the ESP32 5V pin 2 (shared with power relay VCC).

**Same wiring for bench and production.** On the bench, use `CONFIG_BYPASS_INPUT_BATTERY_MONITOR` to skip ADC init and force 50% SOC if no sensors are connected.

## Components

| Component            | Description                                                          |
| -------------------- | -------------------------------------------------------------------- |
| `ultrasonic_a02yyuw` | A02YYUW waterproof UART ultrasonic sensor                            |
| `relay_srd05vdc`     | AEDIKO SRD-05VDC-SL-C 5V relay module for 24V autonomous power rail  |
| `can_twai`           | CAN bus driver wrapper (shared)                                      |
| `can_protocol`       | Message definitions (shared)                                         |
| `led_ws2812`         | WS2812 status LED driver (shared)                                    |
| `heartbeat_monitor`  | CAN node liveness tracking (shared)                                  |
| `battery_monitor`    | Pack voltage/current ADC sensing and SOC estimation (LEM HTFS-200-P) |
| `safety_logic`       | Extracted stop/fault evaluation logic (shared, tested)               |
| `system_state`       | System state machine — target state advancement (shared, tested)     |

## Ultrasonic Sensor

- **Protocol**: 4-byte UART frames at 9600 baud
- **Frame format**: `0xFF | HIGH_BYTE | LOW_BYTE | CHECKSUM`
- **Checksum**: `(0xFF + HIGH + LOW) & 0xFF`
- **Stop threshold**: 2000mm (~6.6 ft)
- **Clear threshold**: 2500mm (~8.2 ft)
- **Engage debounce**: 4 consecutive close reads (~200ms)
- **Disengage debounce**: 6 consecutive clear reads (~300ms)
- **Sample timeout**: 200ms (stale readings ignored)
- **Health timeout**: 500ms based on checksum-valid frame reception (out-of-range/no-echo frames remain healthy)

## Test Bypasses

Compile-time Kconfig flags for bench testing without the full system connected. All default to off (disabled). Enable via `idf.py menuconfig` under **Test Bypasses** (top-level), or add to `sdkconfig.defaults`:

| Flag                                    | Effect                                                                                                |
| --------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| `CONFIG_BYPASS_PLANNER_AUTONOMY_GATE`   | Force autonomy request true (bench mode without Orin)                                                 |
| `CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS` | Ignore Planner heartbeat timeout + Planner fault checks                                               |
| `CONFIG_BYPASS_PLANNER_STATE_MIRROR`    | Simulate Planner state: force READY in NOT_READY/READY, force ENABLE+enable_complete in ENABLE/ACTIVE |
| `CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS` | Ignore Control heartbeat timeout + Control fault checks                                               |
| `CONFIG_BYPASS_CONTROL_STATE_MIRROR`    | Simulate Control state: force READY in NOT_READY/READY, force ENABLE+enable_complete in ENABLE/ACTIVE |
| `CONFIG_BYPASS_CAN_TWAI`                | Disable CAN/TWAI entirely (skip TWAI init/recovery and heartbeat TX/RX)                               |
| `CONFIG_BYPASS_INPUT_PUSH_BUTTON`       | Force push-button stop to inactive (not pressed)                                                      |
| `CONFIG_BYPASS_INPUT_RF_REMOTE`         | Force RF remote stop to inactive (not engaged)                                                        |
| `CONFIG_BYPASS_INPUT_ULTRASONIC`        | Force ultrasonic clear and healthy (skip sensor)                                                      |
| `CONFIG_BYPASS_INPUT_BATTERY_MONITOR`   | Skip battery ADC init, force 50% SOC (bench without sensors)                                          |

## Debug Logging

Compile-time Kconfig flags for verbose logging. Enable via `idf.py menuconfig` under **Debug Logging** (top-level):

### CAN Traffic

| Flag                                        | Default | Effect                                               |
| ------------------------------------------- | ------- | ---------------------------------------------------- |
| `CONFIG_LOG_CAN_FRAMES`                     | off     | Log raw CAN frames (TX and RX) at TWAI driver layer  |
| `CONFIG_LOG_CAN_FRAMES_MOTORS_ONLY`         | off     | Log stepper motor frames only (suppress standard)    |
| `CONFIG_LOG_CAN_FRAMES_SUPPRESS_SAFETY_HB`  | off     | Suppress Safety heartbeat from raw frame log         |
| `CONFIG_LOG_CAN_FRAMES_SUPPRESS_PLANNER_HB` | off     | Suppress Planner heartbeat from raw frame log        |
| `CONFIG_LOG_CAN_FRAMES_SUPPRESS_CONTROL_HB` | off     | Suppress Control heartbeat from raw frame log        |
| `CONFIG_LOG_CAN_HEARTBEAT_TX`               | off     | Log every periodic heartbeat TX (very verbose)       |
| `CONFIG_LOG_CAN_HEARTBEAT_RX`               | off     | Log every received heartbeat, not just state changes |

### Component Health & Recovery

| Flag                                       | Default | Effect                                                       |
| ------------------------------------------ | ------- | ------------------------------------------------------------ |
| `CONFIG_LOG_COMPONENT_HEALTH_TRANSITIONS`  | on      | Log component LOST/REGAINED edge transitions                 |
| `CONFIG_LOG_HEARTBEAT_MONITOR_TRANSITIONS` | on      | Log Planner/Control heartbeat monitor lost/regained events   |
| `CONFIG_LOG_CAN_RECOVERY`                  | off     | Log CAN bus recovery events (stop/start, reinstall, bus-off) |
| `CONFIG_LOG_RETRY_TWAI`                    | off     | Log TWAI retry attempts                                      |
| `CONFIG_LOG_RETRY_PUSH_BUTTON`             | off     | Log push-button retry attempts                               |
| `CONFIG_LOG_RETRY_RF_REMOTE`               | off     | Log RF-remote retry attempts                                 |
| `CONFIG_LOG_RETRY_ULTRASONIC`              | off     | Log ultrasonic retry attempts                                |
| `CONFIG_LOG_RETRY_POWER_RELAY`             | off     | Log power-relay retry attempts                               |

Safety retries failed GPIO-attached safety components indefinitely at 500ms intervals until they are healthy again (no maximum attempt limit).
HEARTBEAT_LED is non-critical and initialized once at startup.

### Safety Logic

| Flag                              | Default | Effect                                                         |
| --------------------------------- | ------- | -------------------------------------------------------------- |
| `CONFIG_LOG_SAFETY_ESTOP_INPUTS`  | off     | Log safety input snapshot every 50ms cycle (extremely verbose) |
| `CONFIG_LOG_SAFETY_STATE_CHANGES` | on      | Log target-state transitions and autonomy-request gate events  |
| `CONFIG_LOG_SAFETY_FAULT_CHANGES` | on      | Log Safety fault/stop channel transitions                      |
| `CONFIG_LOG_SAFETY_STATE_TICK`    | off     | Log state-machine evaluation every 50ms cycle (very verbose)   |

### Inputs

| Flag                                       | Default | Effect                                                                   |
| ------------------------------------------ | ------- | ------------------------------------------------------------------------ |
| `CONFIG_LOG_INPUT_PUSH_BUTTON`             | off     | Log push-button state changes (pressed/released)                         |
| `CONFIG_LOG_INPUT_RF_REMOTE`               | off     | Log RF remote state changes (engaged/disengaged)                         |
| `CONFIG_LOG_INPUT_ULTRASONIC_DISTANCE`     | off     | Log every valid ultrasonic distance reading (~5-10/sec)                  |
| `CONFIG_LOG_INPUT_ULTRASONIC_RX`           | off     | Log raw ultrasonic UART RX bytes (extremely verbose)                     |
| `CONFIG_LOG_INPUT_ULTRASONIC_PARSE_ERRORS` | off     | Log ultrasonic UART parse failures                                       |
| `CONFIG_LOG_INPUT_BATTERY_VOLTAGE`         | off     | Log battery voltage, current, and SOC every update (very verbose, 20 Hz) |
| `CONFIG_LOG_INPUT_BATTERY_SOC_CHANGES`     | on      | Log SOC percentage changes (>=1%) and voltage-based recalibration events |

### Actuators

| Flag                                    | Default | Effect                                     |
| --------------------------------------- | ------- | ------------------------------------------ |
| `CONFIG_LOG_ACTUATOR_POWER_RELAY_STATE` | off     | Log power relay enable/disable transitions |

### Heartbeat LED

| Flag                                     | Default | Effect                                                                |
| ---------------------------------------- | ------- | --------------------------------------------------------------------- |
| `CONFIG_LOG_HEARTBEAT_LED_COLOR_UPDATES` | off     | Log every heartbeat LED color update with color/reason (very verbose) |

## Build

```bash
idf.py set-target esp32c6
idf.py build
idf.py -p PORT flash monitor
```
