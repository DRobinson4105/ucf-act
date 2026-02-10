/**
 * @file debug_console.h
 * @brief Interactive debug console for partial-hardware testing.
 *
 * Gated by CONFIG_ENABLE_DEBUG_CONSOLE (menuconfig).  When disabled, all
 * declarations compile to nothing and no code is linked.
 *
 * Safety ESP32 commands:  bypass/unbypass heartbeat checks, status.
 * Control ESP32 commands: simulate FR sensor, target_state, Planner commands, status.
 */
#pragma once

#ifdef CONFIG_ENABLE_DEBUG_CONSOLE

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Safety ESP32 — shared debug state
// ============================================================================

typedef struct {
    // Written by console commands, read by safety_task
    volatile bool bypass_planner;
    volatile bool bypass_control;

    // Written by safety_task, read by status command
    volatile bool estop_active;
    volatile uint8_t fault_code;
    volatile uint8_t target_state;      // NODE_STATE_* — current system target
    volatile bool relay_on;
    volatile bool planner_alive;
    volatile bool control_alive;
} debug_safety_state_t;

extern debug_safety_state_t g_dbg_safety;

// ============================================================================
// Control ESP32 — shared debug state
// ============================================================================

typedef struct {
    // Written by console commands, read by control_task / CAN RX
    volatile bool sim_fr_active;
    volatile uint8_t sim_fr_value;          // FR_STATE_* constant

    volatile bool sim_auto_active;
    volatile bool sim_auto_value;           // true = ENABLING, false = READY

    volatile bool sim_planner_active;
    volatile uint8_t sim_planner_throttle;
    volatile int16_t sim_planner_steering;
    volatile int16_t sim_planner_braking;

    // Written by control_task, read by status command
    volatile uint8_t control_state;         // NODE_STATE_*
    volatile bool target_enabling;           // target_state >= ENABLING
    volatile uint8_t fr_sensor;
    volatile bool pedal_pressed;
    volatile uint8_t fault_code;            // NODE_FAULT_*
} debug_control_state_t;

extern debug_control_state_t g_dbg_control;

// ============================================================================
// Initialization — call from app_main / main_task
// ============================================================================

/**
 * @brief Start the debug REPL and register Safety ESP32 commands.
 *
 * Registers: bypass, unbypass, status.
 * Must be called after UART is ready (i.e., after app_main entry).
 */
void debug_console_init_safety(void);

/**
 * @brief Start the debug REPL and register Control ESP32 commands.
 *
 * Registers: sim, status.
 * Must be called after UART is ready (i.e., after app_main entry).
 */
void debug_console_init_control(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ENABLE_DEBUG_CONSOLE */
