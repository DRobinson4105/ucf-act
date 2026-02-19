# Unit Tests

Host-native unit tests for the CAN bus shared protocol, logic, and component libraries. These compile and run on any dev machine — no ESP-IDF, FreeRTOS, or embedded toolchain required.

The tests cover:
- Pure protocol logic (`common/protocol/stepper_protocol_uim2852`, `common/protocol/can_protocol`)
- Motor driver component (`control-esp32/components/stepper_motor_uim2852`) via mocked ESP-IDF APIs
- Driver input hardware components (`control-esp32/components/adc_12bitsar`, `control-esp32/components/optocoupler_pc817`)
- Extracted decision logic (`common/logic/safety_logic`, `common/logic/control_logic`, `common/logic/system_state`) — pure functions with no hardware deps

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

| Command | Description |
|---------|-------------|
| `make` | Compile and run all test suites |
| `make test_stepper_protocol` | Compile only the stepper protocol test binary |
| `make test_can_protocol` | Compile only the CAN protocol test binary |
| `make test_motor_component` | Compile only the motor component test binary |
| `make test_safety_logic` | Compile only the safety logic test binary |
| `make test_control_logic` | Compile only the control logic test binary |
| `make test_system_state` | Compile only the system state test binary |
| `make test_heartbeat_monitor` | Compile only the heartbeat monitor test binary |
| `make test_driver_inputs` | Compile only the driver inputs test binary |
| `make clean` | Delete compiled binaries |

To run a single suite after building:

```bash
make test_stepper_protocol
./test_stepper_protocol
```

## Test Suites

### `test_stepper_protocol.c`

Tests the `stepper_protocol_uim2852` library (CAN frame building and parsing for UIM2852 stepper motors).

| Category | What it covers |
|----------|----------------|
| CAN ID encode/decode | Round-trip for node IDs 0-31, ACK bit manipulation, NULL output safety, invalid ID rejection |
| CW utility | `cw_with_ack()`, `cw_ack_requested()`, `cw_base()` |
| Frame builders | Every command type: MO, BG, ST, SD, PA, SP, MS, PP, AC, DC, JV, PR, IC/LM/QE set, OG, brake. Verifies byte layout matches the UIM2852 SimpleCAN spec |
| MS[0] parse | Status flags (driver_on, stopped, in_position, stall), relative position including negative values, short-data rejection, NULL safety |
| MS[1] parse | Speed + absolute position parsing, negative 24-bit speed, short-data and NULL rejection |
| Notification parse | PTP complete, alarm/stall detection, short-data rejection |
| Error parse | Error code + related CW extraction, short-data rejection |
| Param response parse | 8/16/32-bit values, negative values, short-data rejection |
| Set param round-trips | IC, LM, QE build-then-parse verifying values survive encoding |
| Edge cases | Zero, INT32_MIN, INT32_MAX for position commands; OG zeroes entire buffer |

### `test_can_protocol.c`

Tests the `can_protocol` header (shared protocol definitions used by all nodes).

| Category | What it covers |
|----------|----------------|
| LE16 pack/unpack | Unsigned and signed 16-bit little-endian: zero, max, arbitrary value, INT16_MIN, INT16_MAX |
| Planner command encode/decode | Round-trip with typical values, zeroes, INT16 extremes, wire format byte order verification |
| Safety heartbeat encode/decode | Round-trip advancing/retreating states, reserved byte zeroing |
| Heartbeat encode/decode | Round-trip basic, with fault, with enable_complete flag, with autonomy_request flag, reserved byte zeroing |
| String helpers | `node_state_to_string()` all values + unknown, `node_fault_to_string()` all ranges + unknown |
| CAN ID constants | IDs fall within assigned ranges (0x100-0x10F, 0x110-0x11F, 0x120-0x12F), no collisions |

### `test_motor_component.c`

Tests the `stepper_motor_uim2852` driver component using mocked ESP-IDF/FreeRTOS APIs (see `mocks/` directory).

| Category | What it covers |
|----------|----------------|
| Init | Default/custom config, NULL motor, semaphore create failure |
| Enable/disable | MO frame send, TX failure handling, uninitialized motor guard |
| Stop | ST command, emergency stop (SD+ST sequence) |
| Go absolute/relative | PA+BG sequence, negative positions, PA/BG failure, PR+BG, motion flag |
| Set origin | OG frame send, position clear |
| Set speed/accel/decel | SP, AC, DC parameter frames |
| Query status/position/clear | MS query, PP query, MS clear |
| Process frame filtering | NULL motor/msg, uninitialized, standard (non-extended) frame, wrong node ID |
| Process frame MS | MS[0] status/position/driver_enabled, MS[1] speed/position, negative values |
| Process frame NOTIFY | PTP complete, PTP callback, stall detection, stall callback |
| Process frame ER | Error flag, short data handling |
| Process frame MO ACK | Enable/disable acknowledgment |
| Process frame param response | Query unblock, wrong idx/cw ignored, no pending query |
| Process frame general | Timestamp update, ACK pending clear |
| Query param | Send+block, timeout, NULL value, uninitialized |
| Set param | Frame send, uninitialized guard |
| Configure | Timeout uses defaults, uninitialized guard, success updates microstep |
| Notify callback | Set callback, NULL motor guard |

### `test_safety_logic.c`

Tests the pure `safety_logic` module (e-stop bitmask evaluation, ultrasonic fail-safe, relay decisions).

| Category | What it covers |
|----------|----------------|
| Ultrasonic trigger | Clear path, obstacle detected, sensor unhealthy (fail-safe), both bad |
| E-stop bitmask | All clear, each individual source, and combined conditions represented as OR'ed fault bits |
| Ultrasonic fail-safe in evaluate | Unhealthy sensor triggers estop, healthy+clear passes |
| Relay output | Enabled when safe, disabled on estop, disabled on timeout |
| Combined scenarios | Multiple simultaneous faults (all bits preserved), estop on/off transition, ultrasonic fault alone blocks |

### `test_control_logic.c`

Tests the pure `control_logic` module (state machine, throttle slew, preconditions, CAN TX tracking).

| Category | What it covers |
|----------|----------------|
| Preconditions | All good, FR not forward, pedal pressed, pedal not rearmed, active fault, NULL input |
| Throttle slew | At target (no change), step up, step down, too soon (rate limited) |
| INIT -> READY | Always transitions, regardless of Safety target state |
| READY -> ENABLING | Preconditions met, blocked by pedal/auto/FR |
| ENABLING -> ACTIVE | Timer expired, timer not expired, exact boundary, enable_complete stays in ENABLING |
| ENABLING -> READY abort | Auto blocked, pedal pressed, FR wrong, abort priority (auto checked first) |
| ACTIVE override | Pedal override, FR changed, safety retreat disable, priority (safety retreat > pedal > FR) |
| ACTIVE throttle + steering | Slew triggers APPLY_THROTTLE, at target, steering/braking dedup |
| OVERRIDE -> READY | Conditions met, stays (no auto/pedal not rearmed), resets dedup trackers |
| FAULT -> recovery | Signals ATTEMPT_RECOVERY, resets trackers, preserves fault code |
| Motor fault injection | From ACTIVE (disables autonomy + FAULT), from READY (FAULT only) |
| FR INVALID sensor fault | From ACTIVE (disables autonomy + fault), already faulted (no double-fault) |
| Motor/FR fault from ENABLING | Motor fault during ENABLING aborts, FR_INVALID during ENABLING aborts |
| CAN TX tracking | OK resets count, fail below threshold, fail at threshold triggers recovery |
| Timer overflow | Slew timer and enable timer handle uint32 wrap correctly |
| Full lifecycle | Walks READY -> ENABLING -> ACTIVE -> OVERRIDE -> READY |

### `test_system_state.c`

Tests the pure `system_state` module (Safety's target state advancement logic).

| Category | What it covers |
|----------|----------------|
| INIT -> READY | Always transitions after boot |
| READY -> ENABLING | Both nodes READY, both alive, no e-stop, autonomy_request asserted |
| READY stays | No autonomy request, Control not ready, Planner not ready |
| ENABLING -> ACTIVE | Both nodes ENABLING with enable_complete flag |
| ENABLING stays | One node complete, no nodes complete |
| ACTIVE stays | Normal operation |
| Autonomy halt retreat | ENABLING/ACTIVE retreat to READY when Planner drops autonomy hold request |
| E-stop retreat | From READY, ENABLING, ACTIVE -- all retreat to READY |
| Fault retreat | Planner fault, Control fault from ACTIVE |
| Override retreat | Planner override from ACTIVE, Control override from ENABLING |
| Timeout retreat | Planner timeout from ACTIVE, Control timeout from ENABLING |
| Edge cases | Already READY with e-stop (no change), unknown state retreats |

### `test_heartbeat_monitor.cpp`

Tests the `heartbeat_monitor` component (name handling, timeout transitions, and mask reporting).

| Category | What it covers |
|----------|----------------|
| Init/tag behavior | Config-derived tag (`<name>_HB`) and null-config fallback (`HEARTBEAT`) |
| Node registration | Null name fallback to `"unknown"`, copied-name lifetime safety |
| Timeout/liveness | Alive -> timeout -> alive transition and timeout mask bit behavior |
| Capacity limits | Registration failure when max nodes reached |

### `test_driver_inputs.cpp`

Tests the split hardware input components used by Control: `adc_12bitsar` and `optocoupler_pc817`.

| Category | What it covers |
|----------|----------------|
| Pedal ADC | Raw conversion path, calibration path, ADC init failure handling |
| F/R debounce | Debounced transition requires stable `FR_PC817_DEBOUNCE_MS` interval |
| F/R raw mapping | PC817 active-low mapping for Forward/Reverse/Neutral/Invalid |
| F/R init failure | GPIO setup failure propagates init failure |

## Mock Infrastructure

The `mocks/` directory provides stub implementations of ESP-IDF and FreeRTOS APIs so that the motor component can be compiled and tested on the host:

| File | Purpose |
|------|---------|
| `esp_idf_mock.h` / `.c` | Core types, error codes, mock state variables (`mock_sent_frames[]`, `mock_tick_count`, etc.), semaphore callback injection, `mock_reset_all()` |
| `can_twai.hh` | Mock CAN send that captures frames to `mock_sent_frames[]`, `can_twai_bus_ok()` / `can_twai_recover_bus_off()` stubs |
| `freertos/FreeRTOS.h` | FreeRTOS type stubs and tick macros |
| `driver/twai.h` | TWAI message type, status info struct, driver stub functions |
| `esp_log.h` | No-op logging macros |
| `esp_err.h` | Error code definitions |

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
    uint8_t dl = stepper_uim2852_build_pa(data, 12345);
    assert(dl == 4);
    // ... more assertions
}
```

2. Add it to `main()`:

```c
TEST(test_my_new_feature);
```

3. Run `make` to verify.
