# safety-esp32

ESP-IDF firmware for the Safety ESP32-C6. Acts as the **system state authority** — it is the only node that can advance the system forward (NOT_READY -> READY -> ENABLE -> ACTIVE). Monitors all safety inputs, controls the 24V power relay , and broadcasts the system target state via a unified heartbeat.

## State Machine

Safety owns the system target state and broadcasts it as `state` in heartbeat `0x100`. All nodes observe Safety's heartbeat to track the system target. Safety commands `NOT_READY`/`READY`/`ENABLE`/`ACTIVE`; Planner and Control still report local `OVERRIDE`/`FAULT` as live states. Safety advances only when nodes are healthy and readiness conditions are met, and retreats to `NOT_READY` on hard safety conditions.

### Target Transitions

- **Startup target**: Safety starts in `INIT`; after `INIT` dwell it transitions to `NOT_READY`.
- **`NOT_READY -> READY`**: Planner and Control both report `READY`, both are alive, no e-stop.
- **`READY -> ENABLE`**: Planner/Orin request edge (`HEARTBEAT_FLAG_AUTONOMY_REQUEST`) is latched when Safety is in the accept window (target `READY`, no e-stop, both nodes alive, both node states `READY`).
- **`ENABLE -> ACTIVE`**: Both nodes report `ENABLE` with `HEARTBEAT_FLAG_ENABLE_COMPLETE`, no e-stop.
- **`ENABLE/ACTIVE -> READY`**: Planner/Orin drops autonomy request while nodes remain `READY`.
- **`ENABLE/ACTIVE -> NOT_READY`**: Planner/Orin drops autonomy request while either node is not `READY`.
- **`ANY (post-INIT) -> NOT_READY`**: e-stop active, node `FAULT`, node `OVERRIDE`, or node timeout.

Note: During `INIT` dwell, Safety intentionally holds target state at `INIT` for deterministic startup sequencing.

Autonomy request handling is one-shot for entry and level-hold for run:

- Safety latches a rising request edge only while in the accept window (`READY`, clear, both nodes alive, both node states `READY`).
- Safety consumes it when entering `ENABLE`.
- Safety requires request drop before re-arm.
- If request drops during `ENABLE`/`ACTIVE`, Safety retreats out of autonomy.

## Safety Logic

The Safety ESP32 continuously monitors:
1. **Hardware e-stops**: Push button (HB2-ES544), RF remote (EV1527)
2. **Obstacle detection**: Ultrasonic sensor A02YYUW (threshold: 1000mm ~3.3ft)
3. **Node liveness**: Planner and Control heartbeats (timeout: 500ms)
4. **Node health**: Planner and Control state fields (FAULT/OVERRIDE triggers retreat)

**Power relay behavior:**
- ENABLED when all inputs are clear (no e-stop)
- DISABLED immediately when any e-stop condition detected

**Input debounce:**
- Push-button and RF remote engage is immediate (safety-critical, no debounce)
- Push-button and RF remote disengage requires 3 consecutive clear reads (~150ms at 50ms loop) to filter contact bounce
- Ultrasonic health transitions require 3 consecutive agreeing samples (~150ms) to prevent flap around the 500ms timeout boundary

**E-stop fault sources** (OR'ed into a bitmask):
1. Push button pressed (HB2-ES544)
2. RF remote kill (EV1527)
3. Ultrasonic triggered (A02YYUW obstacle detected OR sensor not responding)
4. Planner heartbeat state == FAULT
5. Planner heartbeat timeout (500ms)
6. Control heartbeat state == FAULT
7. Control heartbeat timeout (500ms)

## CAN Messages

### Receives

| ID    | Name              | Description                                   |
|-------|-------------------|-----------------------------------------------|
| 0x110 | PLANNER_HEARTBEAT | Planner alive (seq, state, fault_code, flags) |
| 0x120 | CONTROL_HEARTBEAT | Control alive (seq, state, fault_code, flags) |

### Sends

| ID    | Name             | Rate                              | Description                                                                           |
|-------|------------------|-----------------------------------|---------------------------------------------------------------------------------------|
| 0x100 | SAFETY_HEARTBEAT | 100ms + immediate on state change | System target state + e-stop fault code (same `node_heartbeat_t` format as all nodes) |

Safety's heartbeat `state` field = system target state (NODE_STATE_*). Its `fault_code` field = e-stop reason (NODE_FAULT_ESTOP_*, 0 when safe).

### Heartbeat Monitoring

| Node    | Timeout | Tracked Fields                          |
|---------|---------|-----------------------------------------|
| Planner | 500ms   | sequence, state (FAULT triggers e-stop) |
| Control | 500ms   | sequence, state (FAULT triggers e-stop) |

## E-stop Fault Bitmask (NODE_FAULT_ESTOP_*)

The fault_code byte in Safety's heartbeat is a **bitmask** — multiple bits can be set simultaneously when multiple e-stop conditions are active. For example, if both the push button and RF remote are active, fault_code = 0x03 (0x01 | 0x02).

| Bit | Code | Constant              | Trigger                                                 |
|-----|------|-----------------------|---------------------------------------------------------|
| -   | 0x00 | NONE                  | System OK                                               |
| 0   | 0x01 | ESTOP_BUTTON          | Push button HB2-ES544 pressed                           |
| 1   | 0x02 | ESTOP_REMOTE          | RF remote EV1527 kill signal active                     |
| 2   | 0x04 | ESTOP_ULTRASONIC      | Ultrasonic A02YYUW obstacle (<1000mm) or not responding |
| 3   | 0x08 | ESTOP_PLANNER         | Planner heartbeat state == FAULT                        |
| 4   | 0x10 | ESTOP_PLANNER_TIMEOUT | No Planner heartbeat for 500ms                          |
| 5   | 0x20 | ESTOP_CONTROL         | Control heartbeat state == FAULT                        |
| 6   | 0x40 | ESTOP_CONTROL_TIMEOUT | No Control heartbeat for 500ms                          |
| all | 0x7F | ESTOP_ANY             | Power relay unavailable (all bits set = total e-stop)   |

## Pin Configuration

| GPIO | Function              | Direction | Notes                                           |
|------|-----------------------|-----------|-------------------------------------------------|
| 2    | Power Relay           | Output    | Active HIGH, pull-down, SRD-05VDC-SL-C module   |
| 4    | CAN TX                | Output    | TWAI peripheral (SN65HVD230 transceiver)        |
| 5    | CAN RX                | Input     | TWAI peripheral (SN65HVD230 transceiver)        |
| 6    | Push Button HB2-ES544 | Input     | Pull-up, active HIGH (NC switch opens on press) |
| 7    | RF Remote EV1527      | Input     | Pull-up, active HIGH (NC relay output, COM to GND) |
| 8    | Status LED            | Output    | Onboard WS2812 RGB LED (no external wiring)     |
| 10   | Ultrasonic A02YYUW TX | Output    | UART1 TX (sensor RX, mode select)               |
| 11   | Ultrasonic A02YYUW RX | Input     | UART1 RX, 9600 baud (sensor TX)                 |

### LED Behavior

| Color        | State                                 |
|--------------|---------------------------------------|
| Solid green  | Target state READY (no e-stop active) |
| Solid orange | Target state ENABLE                   |
| Solid blue   | Target state ACTIVE                   |
| Solid red    | Non-nominal condition (e-stop active) |

## Wiring

Each subsection covers production (on-cart) and bench wiring for Safety ESP32 interfaces. For CAN bus topology shared across all nodes, see [CAN Bus Wiring](../README.md#can-bus-wiring).

### Wiring Summary

| Interface               | Bench vs Production                                         |
|-------------------------|-------------------------------------------------------------|
| CAN bus (SN65HVD230)    | Same — see [root README](../README.md#can-bus-wiring)       |
| Push button (HB2-ES544) | Same                                                        |
| RF remote (EV1527)      | **Different** — receiver needs separate 12V supply on bench |
| Ultrasonic (A02YYUW)    | Same                                                        |
| Power relay (SRD-05VDC) | Same hardware — bench relay switches with no 24V load       |
| Status LED (WS2812)     | Same                                                        |

### CAN Bus (SN65HVD230)

GPIO 4 (TX) and GPIO 5 (RX) connect to a WAVESHARE SN65HVD230 CAN transceiver module via a 4-pin JST-PH connector. Safety ESP32's Waveshare board has onboard termination **enabled** (120 ohm). See the [root README](../README.md#can-bus-wiring) for the full 5-node bus topology including stepper motors.

**ESP32-to-transceiver connector (4-pin JST-PH):**

| Pin | Wire Color | Signal |
|-----|------------|--------|
| 1   | Yellow     | TXD (GPIO 4) |
| 2   | Green      | RXD (GPIO 5) |
| 3   | Red        | VCC (3.3V)   |
| 4   | Black      | GND          |

### Push Button E-Stop (HB2-ES544)

GPIO 6 reads a normally-closed (NC) mxuteek HB2-ES544 22mm emergency stop push button. Internal pull-up enabled, active HIGH — the NC switch holds GPIO LOW in normal operation; pressing the button opens the switch and GPIO reads HIGH (e-stop active). NC design is fail-safe: a broken wire also triggers e-stop.

**Wiring (2-pin):**

| Wire Color | ESP32 Pin | Button Terminal |
|------------|-----------|-----------------|
| White      | GPIO 6    | NC terminal     |
| Black      | GND       | COM terminal    |

**Same wiring for bench and production.** On the bench, use the same HB2-ES544 switch or any NC momentary button wired between GPIO 6 and GND (pull-up keeps it HIGH when open).

### RF Remote E-Stop (EV1527)

GPIO 7 reads the NC (normally closed) relay output of a DieseRC 433MHz RF receiver module (DC 12V 1CH relay, EV1527 learning code). Internal pull-up enabled, active HIGH. The receiver's onboard relay NC and COM terminals connect to GPIO 7 and ESP32 GND respectively. When the remote button is not pressed (relay de-energized), NC-COM is closed, pulling GPIO 7 to GND (LOW, safe). When pressed (relay energizes), NC-COM opens, internal pull-up pulls GPIO 7 HIGH (e-stop active). This is fail-safe: receiver power loss or broken wire also opens NC-COM, triggering e-stop.

**Wiring:**

| Wire Color | From | To |
|------------|------|----|
| White      | ESP32 GPIO 7 | Receiver NC terminal |
| Black      | ESP32 GND | Receiver COM terminal |
| Red        | 12V supply (+) | Receiver VCC (12V) |
| Black      | 12V supply (-) | Receiver GND |

Receiver GND and ESP32 GND must share a common ground reference.

**Production:** The receiver module is powered from 12V on the cart and mounted near the Safety ESP32.

**Bench:** Same wiring. The receiver module requires its own 12V supply (bench power supply or 12V adapter).

### Ultrasonic Sensor (A02YYUW)

UART1: GPIO 10 (TX to sensor RX/mode select), GPIO 11 (RX from sensor TX). 9600 baud, 4-byte frames. The A02YYUW is a waterproof ultrasonic rangefinder operating at 3.3-5V with a direct UART output — no level shifting or isolation needed.

**Wiring (4-pin):**

| Wire Color | ESP32 Pin | Sensor Wire |
|------------|-----------|-------------|
| Red        | 3.3V      | VCC (red wire)           |
| Black      | GND       | GND (black wire)         |
| Blue       | GPIO 10 (TX) | Sensor RX (mode select) |
| Green      | GPIO 11 (RX) | Sensor TX (data output) |

Note the TX/RX crossover: ESP32 TX -> Sensor RX, ESP32 RX -> Sensor TX.

**Same wiring for bench and production.** The sensor connects directly to the ESP32 UART pins in both environments. On the bench, point the sensor at a wall or object to test obstacle detection. Stop threshold is 1000mm (~3.3 ft).

### Power Relay (SRD-05VDC-SL-C)

GPIO 2 drives an AEDIKO 1-channel 5V relay module (SRD-05VDC-SL-C, optocoupler-isolated trigger). The relay's NO (normally open) terminal switches the 24V autonomous power rail that feeds the stepper motor drivers. Pull-down on GPIO 2 ensures the relay stays de-energized (24V cut, vehicle stopped) on boot/reset.

**Wiring (3-pin):**

| Wire Color | From | To |
|------------|------|----|
| White      | ESP32 GPIO 2 | Relay module IN (trigger) |
| Red        | ESP32 5V (USB) or cart 5V rail | Relay module VCC |
| Black      | ESP32 GND | Relay module GND |

**Production:** Relay NO terminal connects the 24V power rail to the stepper motor drivers. Energized = 24V flows to motors (vehicle can move). De-energized = 24V cut (fail-safe stop).

**Bench:** Same hardware. Relay clicks audibly when toggled — useful for verifying GPIO 2 output. Without a 24V load, the relay opens/closes its contacts with no effect.

### Status LED (WS2812)

GPIO 8 is configured as an RMT TX output driving the onboard WS2812 RGB status LED data line (no external wiring required). LED color is determined by software state logic, not GPIO level — the GPIO serves only as the RMT data output.

## Components

| Component            | Description                                                         |
|----------------------|---------------------------------------------------------------------|
| `gpio_input`         | Generic GPIO digital input driver (push button, RF remote)          |
| `ultrasonic_a02yyuw` | A02YYUW waterproof UART ultrasonic sensor                           |
| `relay_srd05vdc`     | AEDIKO SRD-05VDC-SL-C 5V relay module for 24V autonomous power rail |
| `can_twai`           | CAN bus driver wrapper (shared)                                     |
| `can_protocol`       | Message definitions (shared)                                        |
| `led_ws2812`         | WS2812 status LED driver (shared)                                   |
| `heartbeat_monitor`  | CAN node liveness tracking (shared)                                 |
| `safety_logic`       | Extracted e-stop evaluation logic (shared, tested)                  |
| `system_state`       | System state machine — target state advancement (shared, tested)    |

## Ultrasonic Sensor

- **Protocol**: 4-byte UART frames at 9600 baud
- **Frame format**: `0xFF | HIGH_BYTE | LOW_BYTE | CHECKSUM`
- **Checksum**: `(0xFF + HIGH + LOW) & 0xFF`
- **Stop threshold**: 1000mm (~3.3 ft)
- **Sample timeout**: 200ms (stale readings ignored)

## Test Bypasses

Compile-time Kconfig flags for bench testing without the full system connected. All default to off (disabled). Enable via `idf.py menuconfig` under **Test Bypasses** (top-level), or add to `sdkconfig.defaults`:

| Flag                                    | Effect                                                            |
|-----------------------------------------|-------------------------------------------------------------------|
| `CONFIG_BYPASS_PLANNER_AUTONOMY_GATE`   | Force autonomy request true (bench mode without Orin)             |
| `CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS` | Ignore Planner heartbeat timeout + Planner FAULT checks           |
| `CONFIG_BYPASS_PLANNER_STATE_MIRROR`    | Simulate Planner state/enable_complete by mirroring Safety target |
| `CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS` | Ignore Control heartbeat timeout + Control FAULT checks           |
| `CONFIG_BYPASS_CONTROL_STATE_MIRROR`    | Simulate Control state/enable_complete by mirroring Safety target |
| `CONFIG_BYPASS_INPUT_PUSH_BUTTON`       | Force push-button e-stop to inactive (not pressed)                |
| `CONFIG_BYPASS_INPUT_RF_REMOTE`         | Force RF remote e-stop to inactive (not engaged)                  |
| `CONFIG_BYPASS_INPUT_ULTRASONIC`        | Force ultrasonic clear and healthy (skip sensor)                  |

## Debug Logging

Compile-time Kconfig flags for verbose logging. Enable via `idf.py menuconfig` under **Debug Logging** (top-level):

### CAN I/O

| Flag                          | Default | Effect                                               |
|-------------------------------|---------|------------------------------------------------------|
| `CONFIG_LOG_CAN_HEARTBEAT_TX` | off     | Log every periodic heartbeat TX (very verbose)       |
| `CONFIG_LOG_CAN_HEARTBEAT_RX` | off     | Log every received heartbeat, not just state changes |

### Component Health & Recovery

| Flag                                       | Default | Effect                                                       |
|--------------------------------------------|---------|--------------------------------------------------------------|
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

| Flag                              | Default | Effect                                                        |
|-----------------------------------|---------|---------------------------------------------------------------|
| `CONFIG_LOG_SAFETY_ESTOP_INPUTS`  | off     | Log all e-stop inputs every 50ms cycle (extremely verbose)    |
| `CONFIG_LOG_SAFETY_STATE_CHANGES` | on      | Log target-state transitions and autonomy-request gate events |
| `CONFIG_LOG_SAFETY_FAULT_CHANGES` | on      | Log e-stop fault code transitions                              |
| `CONFIG_LOG_SAFETY_STATE_TICK`    | off     | Log state-machine evaluation every 50ms cycle (very verbose)  |

### Inputs

| Flag                                       | Default | Effect                                                  |
|--------------------------------------------|---------|---------------------------------------------------------|
| `CONFIG_LOG_INPUT_PUSH_BUTTON`             | off     | Log push-button state changes (pressed/released)        |
| `CONFIG_LOG_INPUT_RF_REMOTE`               | off     | Log RF remote state changes (engaged/disengaged)        |
| `CONFIG_LOG_INPUT_ULTRASONIC_DISTANCE`     | off     | Log every valid ultrasonic distance reading (~5-10/sec) |
| `CONFIG_LOG_INPUT_ULTRASONIC_RX`           | off     | Log raw ultrasonic UART RX bytes (extremely verbose)    |
| `CONFIG_LOG_INPUT_ULTRASONIC_PARSE_ERRORS` | off     | Log ultrasonic UART parse failures                      |

### Actuators

| Flag                                    | Default | Effect                                     |
|-----------------------------------------|---------|--------------------------------------------|
| `CONFIG_LOG_ACTUATOR_POWER_RELAY_STATE` | off     | Log power relay enable/disable transitions |

### HEARTBEAT_LED

| Flag                                     | Default | Effect                                                                |
|------------------------------------------|---------|-----------------------------------------------------------------------|
| `CONFIG_LOG_HEARTBEAT_LED_STATE_CHANGES` | off     | Log HEARTBEAT_LED color/reason mode changes                           |
| `CONFIG_LOG_HEARTBEAT_LED_COLOR_UPDATES` | off     | Log every HEARTBEAT_LED color update with color/reason (very verbose) |
