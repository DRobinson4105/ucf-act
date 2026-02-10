# Unit Tests

Host-native unit tests for the CAN bus protocol libraries. These compile and run with plain `gcc` on any dev machine — no ESP-IDF, FreeRTOS, or embedded toolchain required.

The tests cover:
- Pure protocol logic (`common/stepper_protocol_uim2852`, `common/can_protocol`)
- Motor driver component (`control-esp32/components/stepper_motor_uim2852`) via mocked ESP-IDF APIs
- Extracted decision logic (`common/safety_logic`, `common/control_logic`) — pure functions with no hardware deps

## Prerequisites

- `gcc` (any recent version)
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
| `make` | Compile and run all 204 tests |
| `make test_stepper_protocol` | Compile only the stepper protocol test binary |
| `make test_can_protocol` | Compile only the CAN protocol test binary |
| `make test_motor_component` | Compile only the motor component test binary |
| `make test_safety_logic` | Compile only the safety logic test binary |
| `make test_control_logic` | Compile only the control logic test binary |
| `make clean` | Delete compiled binaries |

To run a single suite after building:

```bash
make test_stepper_protocol
./test_stepper_protocol
```

## Test Suites

### `test_stepper_protocol.c` — 44 tests

Tests the `stepper_protocol_uim2852` library (CAN frame building and parsing for UIM2852 stepper motors).

| Category | Tests | What it covers |
|----------|-------|----------------|
| CAN ID encode/decode | 6 | Round-trip for node IDs 0-31, ACK bit manipulation, NULL output safety, invalid ID rejection |
| CW utility | 1 | `cw_with_ack()`, `cw_ack_requested()`, `cw_base()` |
| Frame builders | 13 | Every command type: MO, BG, ST, SD, PA, SP, MS, PP, IC/LM/QE set, OG, brake. Verifies byte layout matches the UIM2852 SimpleCAN spec |
| MS[0] parse | 4 | Status flags (driver_on, stopped, in_position, stall), relative position including negative values, short-data rejection, NULL safety |
| MS[1] parse | 3 | Speed + absolute position parsing, negative 24-bit speed, short-data and NULL rejection |
| Notification parse | 3 | PTP complete, alarm/stall detection, short-data rejection |
| Error parse | 2 | Error code + related CW extraction, short-data rejection |
| Param response parse | 5 | 8/16/32-bit values, negative values, short-data rejection |
| Set param round-trips | 3 | IC, LM, QE build-then-parse verifying values survive encoding |
| Edge cases | 4 | Zero, INT32_MIN, INT32_MAX for position commands; OG zeroes entire buffer |

### `test_can_protocol.c` — 20 tests

Tests the `can_protocol` header (shared protocol definitions used by both ESP32s and the Jetson).

| Category | Tests | What it covers |
|----------|-------|----------------|
| LE16 pack/unpack | 7 | Unsigned and signed 16-bit little-endian: zero, max, arbitrary value, INT16_MIN, INT16_MAX |
| Orin command encode/decode | 4 | Round-trip with typical values, zeroes, INT16 extremes, reserved byte zeroing |
| Safety auto_allowed | 3 | Round-trip allowed/blocked, estop reasons, reserved byte zeroing |
| String helpers | 4 | Every enum value + unknown fallback for `estop_reason`, `control_state`, `override_reason`, `control_fault` |
| CAN ID constants | 2 | IDs fall within assigned ranges (0x100-0x10F, 0x110-0x11F, 0x120-0x12F), no collisions |

### `test_motor_component.c` — 63 tests

Tests the `stepper_motor_uim2852` driver component using mocked ESP-IDF/FreeRTOS APIs (see `mocks/` directory).

| Category | Tests | What it covers |
|----------|-------|----------------|
| Init | 4 | Default/custom config, NULL motor, semaphore create failure |
| Enable/disable | 4 | MO frame send, TX failure handling, uninitialized motor guard |
| Stop | 2 | ST command, emergency stop (SD+ST sequence) |
| Go absolute/relative | 7 | PA+BG sequence, negative positions, PA/BG failure, PR+BG, motion flag |
| Set origin | 2 | OG frame send, position clear |
| Set speed/accel/decel | 3 | SP, AC, DC parameter frames |
| Query status/position/clear | 3 | MS query, PP query, MS clear |
| Process frame filtering | 5 | NULL motor/msg, uninitialized, standard (non-extended) frame, wrong node ID |
| Process frame MS | 8 | MS[0] status/position/driver_enabled, MS[1] speed/position, negative values |
| Process frame NOTIFY | 5 | PTP complete, PTP callback, stall detection, stall callback |
| Process frame ER | 2 | Error flag, short data handling |
| Process frame MO ACK | 2 | Enable/disable acknowledgment |
| Process frame param response | 4 | Query unblock, wrong idx/cw ignored, no pending query |
| Process frame general | 2 | Timestamp update, ACK pending clear |
| Query param | 4 | Send+block, timeout, NULL value, uninitialized |
| Set param | 2 | Frame send, uninitialized guard |
| Configure | 2 | Timeout uses defaults, uninitialized guard |
| Notify callback | 2 | Set callback, NULL motor guard |

### `test_safety_logic.c` — 30 tests

Tests the pure `safety_logic` module (e-stop priority chain, ultrasonic fail-safe, broadcast timing).

| Category | Tests | What it covers |
|----------|-------|----------------|
| Ultrasonic trigger | 4 | Clear path, obstacle detected, sensor unhealthy (fail-safe), both bad |
| E-stop priority chain | 8 | All clear, push_button highest priority, RF remote, ultrasonic, orin_error, orin_timeout, control_timeout, control_error lowest |
| Ultrasonic fail-safe | 2 | Unhealthy sensor triggers estop, healthy+clear passes |
| Auto-allowed logic | 4 | All clear, blocked by estop, blocked by orin/control timeout |
| Relay output | 3 | Enabled when safe, disabled on estop, disabled on timeout |
| NULL safety | 1 | NULL input returns safe defaults (relay off, auto blocked) |
| Broadcast timing | 5 | State change activate/deactivate, periodic during estop, no broadcast when stable, too-soon periodic |
| Combined scenarios | 3 | Multiple faults (highest wins), estop on/off transition, ultrasonic fault alone blocks |

### `test_control_logic.c` — 47 tests

Tests the pure `control_logic` module (state machine, throttle slew, preconditions, CAN TX tracking).

| Category | Tests | What it covers |
|----------|-------|----------------|
| Preconditions | 6 | All good, FR not forward, pedal pressed, pedal not rearmed, active fault, NULL input |
| Throttle slew | 4 | At target (no change), step up, step down, too soon (rate limited) |
| INIT → READY | 2 | Always transitions, regardless of auto_allowed |
| READY → ENABLING | 4 | Preconditions met, blocked by pedal/auto/FR |
| ENABLING → ACTIVE | 3 | Timer expired, timer not expired, exact boundary |
| ENABLING → READY abort | 4 | Auto blocked, pedal pressed, FR wrong, abort priority (auto checked first) |
| ACTIVE override | 4 | Pedal override, FR changed, auto blocked, priority (pedal > FR > auto) |
| ACTIVE throttle + steering | 4 | Slew triggers APPLY_THROTTLE, at target, steering/braking dedup |
| OVERRIDE → READY | 4 | Conditions met, stays (no auto/pedal not rearmed), resets dedup trackers |
| FAULT → recovery | 3 | Signals ATTEMPT_RECOVERY, resets trackers, preserves fault code |
| Motor fault injection | 2 | From ACTIVE (triggers override), from READY (no override) |
| FR INVALID | 2 | From ACTIVE (override + fault), already faulted (no double-fault) |
| CAN TX tracking | 3 | OK resets count, fail below threshold, fail at threshold triggers recovery |
| NULL safety | 1 | NULL inputs returns safe defaults |
| Full lifecycle | 1 | Walks READY → ENABLING → ACTIVE → OVERRIDE → READY |

## Mock Infrastructure

The `mocks/` directory provides stub implementations of ESP-IDF and FreeRTOS APIs so that the motor component can be compiled and tested on the host:

| File | Purpose |
|------|---------|
| `esp_idf_mock.h` / `.c` | Core types, error codes, mock state variables (`mock_sent_frames[]`, `mock_tick_count`, etc.), `mock_reset_all()` |
| `can_twai.hh` | Mock CAN send that captures frames to `mock_sent_frames[]` |
| `freertos/FreeRTOS.h` | FreeRTOS type stubs and tick macros |
| `driver/twai.h` | TWAI message type definitions |
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
