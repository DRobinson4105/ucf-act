# Unit Tests

Host-native unit tests for the CAN bus shared protocol, logic, and component libraries. These compile and run on any dev machine — no ESP-IDF, FreeRTOS, or embedded toolchain required.

The tests cover:

- Pure protocol logic (`common/protocol/can_protocol`, `common/protocol/orin_link_protocol`)
- Motor protocol and driver component (`control-esp32/components/motor_uim2852`) via mocked ESP-IDF APIs
- Driver input hardware components (`control-esp32/components/adc_12bitsar`, `control-esp32/components/optocoupler_pc817`)
- Actuator components (`control-esp32/components/relay_dpdt_my5nj`) via mocked ESP-IDF APIs
- Obstacle filter logic (now part of `safety-esp32/components/ultrasonic_a02yyuw`) — pure functions with no hardware deps
- Extracted decision logic (`control-esp32/main/logic/control_logic`, `safety-esp32/main/logic/safety_logic`, `safety-esp32/main/logic/system_state_machine`) — pure functions with no hardware deps
- Integration tests combining Safety and Control state machines for full round-trip verification

## Prerequisites

- `gcc` (any recent version)
- `g++` with C++17 support (for C++ test suites)
- `make`

## How to Run

Run all tests:

```bash
cd tests
make
```

`make` compiles all test binaries and runs them immediately. Exit code is `0` if all pass, `1` if any fail — so it works in CI pipelines via `make -C tests`.

### Other Commands

| Command                               | Description                                    |
| ------------------------------------- | ---------------------------------------------- |
| `make`                                | Compile and run all test suites                |
| `make test_motor_uim2852_protocol`    | Compile only the motor protocol test binary    |
| `make test_can_protocol`              | Compile only the CAN protocol test binary      |
| `make test_orin_link_protocol`        | Compile only the Orin link protocol test binary |
| `make test_safety_logic`              | Compile only the safety logic test binary      |
| `make test_obstacle_filter`           | Compile only the obstacle filter test binary   |
| `make test_control_logic`             | Compile only the control logic test binary     |
| `make test_system_state`              | Compile only the system state test binary      |
| `make test_heartbeat_monitor`         | Compile only the heartbeat monitor test binary |
| `make test_driver_inputs`             | Compile only the driver inputs test binary     |
| `make test_control_driver_inputs`     | Compile only the control driver inputs binary  |
| `make test_relay`                     | Compile only the relay test binary             |
| `make test_integration_state_machine` | Compile only the integration test binary       |
| `make clean`                          | Delete compiled binaries                       |

To run a single suite after building:

```bash
make test_motor_uim2852_protocol
./build/test_motor_uim2852_protocol
```

## Test Suites

### `test_motor_uim2852_protocol.cpp`

Tests the motor protocol layer of `motor_uim2852` (CAN frame building, codec, and RX parsing for UIM2852 motors).

| Category              | What it covers                                                                                                                                       |
| --------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| CAN ID encode/decode  | Round-trip for node IDs 0-31, ACK bit manipulation, NULL output safety, invalid ID rejection                                                         |
| CW utility            | `cw_with_ack()`, `cw_ack_requested()`, `cw_base()`                                                                                                   |
| Frame builders        | Every command type: MO, BG, ST, SD, PA, SP, MS, PP, AC, DC, JV, PR, IC/LM/QE set, OG, brake. Verifies byte layout matches the UIM2852 SimpleCAN spec |
| MS[0] parse           | Status flags (driver_on, stopped, in_position, stall), relative position including negative values, short-data rejection, NULL safety                |
| MS[1] parse           | Speed + absolute position parsing, negative 24-bit speed, short-data and NULL rejection                                                              |
| Notification parse    | PTP complete, alarm/stall detection, short-data rejection                                                                                            |
| Error parse           | Error code + related CW extraction, short-data rejection                                                                                             |
| Param response parse  | 8/16/32-bit values, negative values, short-data rejection                                                                                            |
| Set param round-trips | IC, LM, QE build-then-parse verifying values survive encoding                                                                                        |
| Edge cases            | Zero, INT32_MIN, INT32_MAX for position commands; OG zeroes entire buffer                                                                            |

### `test_can_protocol.cpp`

Tests the `can_protocol` header (shared protocol definitions used by all nodes).

| Category                       | What it covers                                                                                                            |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------- |
| LE16 pack/unpack               | Unsigned and signed 16-bit little-endian: zero, max, arbitrary value, INT16_MIN, INT16_MAX                                |
| Planner command encode/decode  | Round-trip with typical values and boundary values, short-DLC reject, throttle clamp, wire format byte order verification |
| Safety heartbeat encode/decode | Round-trip advancing/retreating states, reserved byte zeroing                                                             |
| Heartbeat encode/decode        | Round-trip basic, with fault, with enable_complete flag, with autonomy_request flag, reserved byte zeroing                |
| String helpers                 | `node_state_to_string()` all values + unknown, `node_fault_to_string()` all ranges + unknown                              |
| CAN ID constants               | `CAN_ID_SAFETY_HEARTBEAT` in 0x100-0x10F and `CAN_ID_CONTROL_HEARTBEAT` in 0x120-0x12F, no collisions                       |

### `test_orin_link_protocol.cpp`

Tests the `orin_link_protocol` helpers used on the Orin UART links to Safety and Control.

| Category               | What it covers                                                                                  |
| ---------------------- | ----------------------------------------------------------------------------------------------- |
| Message encode/decode  | Planner command and Planner heartbeat round-trip using the shared payload layouts               |
| Heartbeat wrappers     | Control and Safety heartbeat message typing                                                     |
| Message metadata       | Expected payload lengths and message-type string helpers                                        |

### `test_obstacle_filter.cpp`

Tests the pure `obstacle_filter` helper (now part of `ultrasonic_a02yyuw`) used by Safety's ultrasonic stop path.

| Category            | What it covers                                                                   |
| ------------------- | -------------------------------------------------------------------------------- |
| Engage hold         | Requires a sustained true input for `engage_hold_ms` before latching             |
| Engage bounce       | Resets the pending-assert timer if the input goes false before the hold expires  |
| Disengage count     | Requires `disengage_count` consecutive clear samples before unlatching           |
| Disengage bounce    | Clear-count resets to zero on any re-assertion                                   |
| Initial-state edges | NULL input guard, zero-duration configs, latched-from-boot behavior              |

### `test_safety_logic.cpp`

Tests the pure `safety_logic` module (stop/fault bitmask evaluation, ultrasonic fail-safe).

| Category              | What it covers                                                                             |
| --------------------- | ------------------------------------------------------------------------------------------ |
| Ultrasonic trigger    | Clear path, obstacle detected, sensor unhealthy (fail-safe), both bad                      |
| Stop-only inputs      | Local stop inputs (push button, remote, ultrasonic) produce stop_flags with no fault_flags |
| Stop forwarding       | Planner/Control stop_flags forwarded into Safety stop_flags                                |
| Fault-only inputs     | Timeouts and issues produce fault_flags with no stop_flags                                 |
| Combined stop + fault | Multiple simultaneous causes in both channels (all bits preserved)                         |
| NULL input guard      | `safety_evaluate(NULL)` returns fail-safe defaults (stop_active=true)                      |

### `test_control_logic.cpp`

Tests the pure `control_logic` module (state machine, throttle slew, preconditions, fault injection, envelope clamping). 87 tests covering all public functions and every state machine branch.

| Category                       | What it covers                                                                                                          |
| ------------------------------ | ----------------------------------------------------------------------------------------------------------------------- |
| State transitions (16)         | INIT/NOT_READY/READY/ENABLE/ACTIVE transitions, precondition-driven readiness, operator-stop retreat/recovery           |
| Precondition checker (7)       | NULL returns ALL, FR reverse/invalid/neutral, pedal not rearmed, active fault, multiple combine as bitmask              |
| Throttle slew (6)              | At target (no change), step up/down by 1, rate limited, NULL inputs, uint32 timer overflow                              |
| Command clamping (4)           | Below min, above max, in range, misconfigured (min > max) returns neutral                                               |
| INIT edge cases (2)            | Stays when dwell not expired, uint32 timer wrap still transitions                                                       |
| ENABLE abort (6)               | Safety retreat, FR reverse, neutral doesn't abort, timer not expired (stays), complete fires once, exact timer boundary |
| ACTIVE override (6)            | FR changed, neutral zeros throttle, neutral at zero no change, steering error, braking error, safety retreat priority   |
| ACTIVE throttle + steering (9) | Slew applies APPLY_THROTTLE, at target no action, clamps to envelope, unconfigured forces neutral,                      |
|                                | steering dedup (same skips, changed sends), braking dedup reset always sends,                                           |
|                                | envelope unconfigured forces neutral, envelope clamps out-of-range                                                      |
| Motor fault injection (4)      | From ACTIVE (DISABLE_AUTONOMY), from ENABLE (ABORT_ENABLE), from READY (FAULT only), ignored if faulted                 |
| FR_INVALID sensor fault (5)    | From ACTIVE, from ENABLE, from NOT_READY (no fault), from READY (no fault), no re-trigger when faulted                  |
| OVERRIDE recovery (3)          | Stays when pedal not rearmed, stays when FR reverse, recovers when FR neutral                                           |
| FAULT recovery (3)             | MOTOR_COMM clears when motor OK, SENSOR_INVALID clears when FR valid, unknown fault stays with RECOVERY                 |
| Target sanitization (1)        | Invalid target (0xFF) treated as NOT_READY, triggers safety retreat from ACTIVE                                         |
| Safe outputs (1)               | Override zeros throttle, resets motor dedup trackers to sentinel                                                      |
| NULL inputs (1)                | `control_compute_step(state, fault, NULL)` returns safe zero-init defaults                                              |

### `test_system_state.cpp`

Tests the pure `system_state_machine` module (Safety's target state advancement logic).

| Category              | What it covers                                                                    |
| --------------------- | --------------------------------------------------------------------------------- |
| INIT -> NOT_READY     | Transitions after boot dwell                                                      |
| NOT_READY -> READY    | Both nodes READY, both alive, no stop/fault active                                |
| READY -> ENABLE       | Both nodes READY and autonomy_request asserted                                    |
| ENABLE -> ACTIVE      | Both nodes ENABLE with enable_complete flag                                       |
| ENABLE stays          | One node complete, no nodes complete                                              |
| ACTIVE stays          | Normal operation                                                                  |
| Autonomy halt retreat | ENABLE/ACTIVE retreat to READY/NOT_READY when Planner drops autonomy hold request |
| Problem retreat       | From READY, ENABLE, ACTIVE -- stop/fault active retreats to NOT_READY             |
| Timeout retreat       | Planner/Control liveness loss retreats target to NOT_READY                        |
| Edge cases            | Unknown state retreats to NOT_READY                                               |

### `test_heartbeat_monitor.cpp`

Tests the `heartbeat_monitor` component (name handling, timeout transitions, and mask reporting).

| Category          | What it covers                                                                                                            |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------- |
| Init/tag behavior | Config-derived tag (`<name>_HB`) and null-config fallback (`HEARTBEAT`)                                                   |
| Node registration | Null name fallback to `"unknown"`, copied-name lifetime safety                                                            |
| Timeout/liveness  | Alive -> timeout -> alive transition and timeout mask bit behavior                                                        |
| Capacity limits   | Registration failure when max nodes reached                                                                               |
| all_alive         | True when all updated, false when one timed out, false when never seen, vacuous truth (zero nodes), recovers after update |
| Tick wrap         | Timeout detection works correctly when uint32 tick counter wraps past UINT32_MAX                                          |

### `test_driver_inputs.cpp`

Tests the split hardware input components used by Control: `adc_12bitsar` and `optocoupler_pc817`.

| Category         | What it covers                                                       |
| ---------------- | -------------------------------------------------------------------- |
| Pedal ADC        | Raw conversion path, calibration path, ADC init failure handling     |
| F/R debounce     | Debounced transition requires stable `FR_PC817_DEBOUNCE_MS` interval |
| F/R raw mapping  | PC817 active-low mapping for Forward/Reverse/Neutral/Invalid         |
| F/R init failure | GPIO setup failure propagates init failure                           |

### `test_control_driver_inputs.cpp`

Tests the higher-level Control input aggregation helper used by `control-esp32/main`.

| Category             | What it covers                                                         |
| -------------------- | ---------------------------------------------------------------------- |
| Pedal init           | Seeds ready/pressed state from the first ADC sample                    |
| Pedal trigger        | Requires consecutive above-threshold samples before reporting pressed  |
| Pedal rearm          | Requires sustained below-threshold hold before reporting re-armed      |
| Pedal runtime loss   | Reports ADC read failure back to the caller for normal loss handling   |
| F/R fallback         | Returns NEUTRAL until the F/R input has been initialized               |

### `test_relay.cpp`

Tests the `relay_dpdt_my5nj` driver component (MY5NJ DPDT relay with 2N5551 transistor driver) using mocked ESP-IDF APIs.

| Category            | What it covers                                          |
| ------------------- | ------------------------------------------------------- |
| Pre-init state      | `is_energized` returns false before init                |
| Init                | Success, NULL config, GPIO failure, preloads safe level |
| Energize/deenergize | GPIO level changes, cycle, set_level failure handling   |
| State query         | `is_energized` reflects actual GPIO level               |

### `test_integration_state_machine.cpp`

Integration tests exercising both `system_state_step` (Safety) and `control_compute_step` (Control) together, simulating the CAN message exchange across state transitions.

| Category                 | What it covers                                                       |
| ------------------------ | -------------------------------------------------------------------- |
| Full round-trip          | INIT -> NOT_READY -> READY -> ENABLE -> ACTIVE -> NOT_READY (e-stop) |
| ENABLE timeout           | Safety retreats when nodes fail to complete within 5s window         |
| Override during ACTIVE   | Pedal press triggers override, Safety sees retreat                   |
| Autonomy halt            | Planner drops autonomy hold, Safety retreats                         |
| FR_INVALID during ACTIVE | Wiring fault triggers override with SENSOR_INVALID fault             |
| Stale planner command    | Zeroed throttle slews down, steering preserved via dedup             |

## Mock Infrastructure

The `mocks/` directory provides stub implementations of ESP-IDF and FreeRTOS APIs so that the motor component can be compiled and tested on the host:

| File                      | Purpose                                                                                                                                         |
| ------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| `esp_idf_mock.h` / `.cpp` | Core types, error codes, mock state variables (`mock_sent_frames[]`, `mock_tick_count`, etc.), semaphore callback injection, `mock_reset_all()` |
| `can_twai.h`              | Mock CAN send that captures frames to `mock_sent_frames[]`, `can_twai_bus_ok()` stub                                                            |
| `freertos/FreeRTOS.h`     | FreeRTOS type stubs and tick macros                                                                                                             |
| `driver/twai.h`           | TWAI message type, status info struct, driver stub functions                                                                                    |
| `esp_log.h`               | No-op logging macros                                                                                                                            |
| `esp_err.h`               | Error code definitions                                                                                                                          |

The mock include path (`-Imocks`) is placed **first** so that `#include "esp_log.h"` etc. in the component source resolves to mock headers instead of real ESP-IDF ones.

## How the Harness Works

There is no external test framework. Each file uses a minimal `assert`-based harness:

```c
#define TEST(name) do { \
    tests_run++; \
    printf("  [TEST] %-50s ", #name); \
    name(); \
    tests_passed++; \
    printf("PASS\n"); \
} while (0)
```

Each test is a `static void` function containing `assert()` calls. If an assertion fails, the program aborts immediately with the file and line number of the failure. If the function returns normally, it counts as passed.

Compiler flags are strict: `-Wall -Wextra -Werror -std=c11 -g -O0` (debug, no optimization, warnings as errors).

## Adding New Tests

1. Write a test function in the appropriate file:

```c
static void test_my_new_feature(void) {
    uint8_t data[8];
    uint8_t dl = motor_cmd_pa_set(data, 12345);
    assert(dl == 4);
    // ... more assertions
}
```

2. Add it to `main()`:

```c
TEST(test_my_new_feature);
```

3. Run `make` to verify.
