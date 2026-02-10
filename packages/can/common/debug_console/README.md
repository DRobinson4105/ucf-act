# Debug Console

Interactive UART command-line interface for bench testing with partial hardware. Allows bypassing heartbeat checks, simulating sensor inputs, and injecting Planner commands without requiring the full system to be connected.

## Enabling

The debug console is **disabled by default** and gated by `CONFIG_ENABLE_DEBUG_CONSOLE`. When disabled, all code is compiled out — zero overhead in production.

### Via sdkconfig.defaults

```
CONFIG_ENABLE_DEBUG_CONSOLE=y
```

### Via menuconfig

```bash
idf.py menuconfig
# Navigate to: Debug Console → Enable debug serial console
```

**WARNING**: Must be OFF for production deployments. The console prints a warning banner at startup when enabled.

## Safety ESP32 Commands

| Command | Description |
|---------|-------------|
| `bypass planner` | Treat Planner heartbeat as always alive |
| `bypass control` | Treat Control heartbeat as always alive |
| `bypass all` | Bypass both heartbeat checks |
| `unbypass planner` | Re-enable Planner heartbeat checking |
| `unbypass control` | Re-enable Control heartbeat checking |
| `unbypass all` | Re-enable all heartbeat checks |
| `status` | Print current safety state (e-stop, fault code, target state, relay, liveness) |

### Typical Safety Bench Session

```
can> bypass all          # Skip heartbeat timeouts
can> status              # Check if relay enabled, see target state
can> unbypass planner    # Re-enable Planner timeout to test that path
```

## Control ESP32 Commands

| Command | Description |
|---------|-------------|
| `sim fr forward` | Override F/R sensor to FORWARD |
| `sim fr neutral` | Override F/R sensor to NEUTRAL |
| `sim fr reverse` | Override F/R sensor to REVERSE |
| `sim fr off` | Clear F/R override (use real sensor) |
| `sim auto 1` | Force target_state to ENABLING (auto allowed) |
| `sim auto 0` | Force target_state to READY (auto blocked) |
| `sim auto off` | Clear auto override (use real CAN) |
| `sim planner <thr> <steer> <brake>` | Inject Planner commands (throttle 0-7, steering/braking int16) |
| `sim planner stop` | Stop injecting Planner commands |
| `sim off` | Clear all simulation overrides |
| `status` | Print current control state, fault code, sensors |

### Typical Control Bench Session

```
can> sim fr forward      # Pretend F/R is in forward
can> sim auto 1          # Pretend Safety says ENABLING
can> status              # Should show ENABLING -> ACTIVE transition
can> sim planner 3 0 0   # Inject throttle=3, steering=0, braking=0
can> sim planner stop    # Stop injection
can> sim off             # Clear everything
```

## Architecture

The console uses the ESP-IDF `esp_console` REPL over UART0. Each board calls its own init function from `app_main`:

- **Safety**: `debug_console_init_safety()` registers `bypass`, `unbypass`, `status`
- **Control**: `debug_console_init_control()` registers `sim`, `status`

Shared state structs (`debug_safety_state_t`, `debug_control_state_t`) use `volatile` fields so the main task loop picks up changes from the console task without explicit synchronization.

## Files

| File | Purpose |
|------|---------|
| `Kconfig` | menuconfig entry (default n) |
| `CMakeLists.txt` | Component definition, depends on `console`, `can_protocol` |
| `include/debug_console.h` | Shared state structs, init function declarations |
| `debug_console.cpp` | REPL startup, all command handlers |
