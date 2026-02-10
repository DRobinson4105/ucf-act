/**
 * @file debug_console.cpp
 * @brief Interactive debug console — bypass/simulate commands for bench testing.
 *
 * Compiled only when CONFIG_ENABLE_DEBUG_CONSOLE=y (menuconfig).
 * Provides an esp_console REPL over UART0 with commands tailored to
 * whichever board calls the init function (safety or control).
 */

#include "sdkconfig.h"

#ifdef CONFIG_ENABLE_DEBUG_CONSOLE

#include "debug_console.h"
#include "can_protocol.hh"

#include "esp_console.h"
#include "esp_log.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <climits>

static const char *TAG = "DBG_CON";

// ============================================================================
// Global shared state instances
// ============================================================================

debug_safety_state_t  g_dbg_safety  = {};
debug_control_state_t g_dbg_control = {};

// ============================================================================
// Safety ESP32 command handlers
// ============================================================================

static int cmd_bypass(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: bypass <planner|control|all>\n");
        return 1;
    }
    if (strcmp(argv[1], "planner") == 0) {
        g_dbg_safety.bypass_planner = true;
        printf("OK: Planner heartbeat bypassed (always alive)\n");
    } else if (strcmp(argv[1], "control") == 0) {
        g_dbg_safety.bypass_control = true;
        printf("OK: Control heartbeat bypassed (always alive)\n");
    } else if (strcmp(argv[1], "all") == 0) {
        g_dbg_safety.bypass_planner = true;
        g_dbg_safety.bypass_control = true;
        printf("OK: All heartbeats bypassed\n");
    } else {
        printf("Unknown target '%s'. Use: planner, control, all\n", argv[1]);
        return 1;
    }
    return 0;
}

static int cmd_unbypass(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: unbypass <planner|control|all>\n");
        return 1;
    }
    if (strcmp(argv[1], "planner") == 0) {
        g_dbg_safety.bypass_planner = false;
        printf("OK: Planner heartbeat check re-enabled\n");
    } else if (strcmp(argv[1], "control") == 0) {
        g_dbg_safety.bypass_control = false;
        printf("OK: Control heartbeat check re-enabled\n");
    } else if (strcmp(argv[1], "all") == 0) {
        g_dbg_safety.bypass_planner = false;
        g_dbg_safety.bypass_control = false;
        printf("OK: All heartbeat checks re-enabled\n");
    } else {
        printf("Unknown target '%s'. Use: planner, control, all\n", argv[1]);
        return 1;
    }
    return 0;
}

static int cmd_safety_status(int argc, char **argv) {
    (void)argc; (void)argv;
    printf("\n--- Safety ESP32 Status ---\n");
    printf("E-stop active:  %s\n", g_dbg_safety.estop_active ? "YES" : "NO");
    printf("Fault code:     %s\n", node_fault_to_string(g_dbg_safety.fault_code));
    printf("Target state:   %s\n", node_state_to_string(g_dbg_safety.target_state));
    printf("Relay:          %s\n", g_dbg_safety.relay_on ? "ON" : "OFF");
    printf("Planner alive:  %s%s\n",
           g_dbg_safety.planner_alive ? "YES" : "NO",
           g_dbg_safety.bypass_planner ? " (BYPASSED)" : "");
    printf("Control alive:  %s%s\n",
           g_dbg_safety.control_alive ? "YES" : "NO",
           g_dbg_safety.bypass_control ? " (BYPASSED)" : "");
    printf("\n");
    return 0;
}

// ============================================================================
// Control ESP32 command handlers
// ============================================================================

static const char *fr_state_name(uint8_t fr) {
    switch (fr) {
        case FR_STATE_NEUTRAL:  return "NEUTRAL";
        case FR_STATE_FORWARD:  return "FORWARD";
        case FR_STATE_REVERSE:  return "REVERSE";
        case FR_STATE_INVALID:  return "INVALID";
        default:                return "???";
    }
}

static int cmd_sim(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage:\n");
        printf("  sim fr <forward|neutral|reverse|off>  Override F/R sensor\n");
        printf("  sim auto <0|1>                        Override target_state\n");
        printf("  sim planner <throttle> <steering> <brake> Inject Planner commands\n");
        printf("  sim planner stop                      Stop injecting\n");
        printf("  sim off                               Clear all overrides\n");
        return 1;
    }

    if (strcmp(argv[1], "fr") == 0) {
        if (argc < 3) { printf("Usage: sim fr <forward|neutral|reverse|off>\n"); return 1; }
        if (strcmp(argv[2], "forward") == 0) {
            g_dbg_control.sim_fr_active = true;
            g_dbg_control.sim_fr_value = FR_STATE_FORWARD;
            printf("OK: F/R overridden to FORWARD\n");
        } else if (strcmp(argv[2], "neutral") == 0) {
            g_dbg_control.sim_fr_active = true;
            g_dbg_control.sim_fr_value = FR_STATE_NEUTRAL;
            printf("OK: F/R overridden to NEUTRAL\n");
        } else if (strcmp(argv[2], "reverse") == 0) {
            g_dbg_control.sim_fr_active = true;
            g_dbg_control.sim_fr_value = FR_STATE_REVERSE;
            printf("OK: F/R overridden to REVERSE\n");
        } else if (strcmp(argv[2], "off") == 0) {
            g_dbg_control.sim_fr_active = false;
            printf("OK: F/R override cleared (using real sensor)\n");
        } else {
            printf("Unknown FR state '%s'\n", argv[2]);
            return 1;
        }
    } else if (strcmp(argv[1], "auto") == 0) {
        if (argc < 3) { printf("Usage: sim auto <0|1|off>\n"); return 1; }
        if (strcmp(argv[2], "off") == 0) {
            g_dbg_control.sim_auto_active = false;
            printf("OK: target_state override cleared (using real CAN)\n");
        } else {
            g_dbg_control.sim_auto_active = true;
            g_dbg_control.sim_auto_value = (atoi(argv[2]) != 0);
            printf("OK: target_state forced to %s\n",
                   g_dbg_control.sim_auto_value ? "ENABLING" : "READY");
        }
    } else if (strcmp(argv[1], "planner") == 0) {
        if (argc >= 3 && strcmp(argv[2], "stop") == 0) {
            g_dbg_control.sim_planner_active = false;
            printf("OK: Planner command injection stopped\n");
        } else if (argc >= 5) {
            int thr_val = atoi(argv[2]);
            if (thr_val < 0) thr_val = 0;
            if (thr_val > 7) thr_val = 7;
            g_dbg_control.sim_planner_throttle = (uint8_t)thr_val;
            int steer_val = atoi(argv[3]);
            if (steer_val < INT16_MIN) steer_val = INT16_MIN;
            if (steer_val > INT16_MAX) steer_val = INT16_MAX;
            g_dbg_control.sim_planner_steering = (int16_t)steer_val;
            int brake_val = atoi(argv[4]);
            if (brake_val < INT16_MIN) brake_val = INT16_MIN;
            if (brake_val > INT16_MAX) brake_val = INT16_MAX;
            g_dbg_control.sim_planner_braking  = (int16_t)brake_val;
            g_dbg_control.sim_planner_active = true;
            printf("OK: Injecting Planner commands: throttle=%u steering=%d braking=%d\n",
                   g_dbg_control.sim_planner_throttle,
                   g_dbg_control.sim_planner_steering,
                   g_dbg_control.sim_planner_braking);
        } else {
            printf("Usage: sim planner <throttle> <steering> <braking>\n");
            printf("       sim planner stop\n");
            return 1;
        }
    } else if (strcmp(argv[1], "off") == 0) {
        g_dbg_control.sim_fr_active = false;
        g_dbg_control.sim_auto_active = false;
        g_dbg_control.sim_planner_active = false;
        printf("OK: All simulation overrides cleared\n");
    } else {
        printf("Unknown sim target '%s'\n", argv[1]);
        return 1;
    }
    return 0;
}

static int cmd_control_status(int argc, char **argv) {
    (void)argc; (void)argv;
    printf("\n--- Control ESP32 Status ---\n");
    printf("State:          %s\n", node_state_to_string(g_dbg_control.control_state));
    printf("Fault:          %s\n", node_fault_to_string(g_dbg_control.fault_code));
    printf("Target >= ENABLING: %s%s\n",
           g_dbg_control.target_enabling ? "YES" : "NO",
           g_dbg_control.sim_auto_active ? " (SIMULATED)" : "");
    printf("F/R sensor:     %s%s\n",
           fr_state_name(g_dbg_control.fr_sensor),
           g_dbg_control.sim_fr_active ? " (SIMULATED)" : "");
    printf("Pedal pressed:  %s\n", g_dbg_control.pedal_pressed ? "YES" : "NO");
    printf("Planner inject: %s\n", g_dbg_control.sim_planner_active ? "ACTIVE" : "off");
    if (g_dbg_control.sim_planner_active) {
        printf("  throttle=%u  steering=%d  braking=%d\n",
               g_dbg_control.sim_planner_throttle,
               g_dbg_control.sim_planner_steering,
               g_dbg_control.sim_planner_braking);
    }
    printf("\n");
    return 0;
}

// ============================================================================
// REPL startup helper
// ============================================================================

static void start_repl(void) {
    esp_console_repl_t *repl = nullptr;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "can> ";
    repl_config.max_cmdline_length = 128;

    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();

    esp_err_t err = esp_console_new_repl_uart(&uart_config, &repl_config, &repl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create REPL: %s", esp_err_to_name(err));
        return;
    }
    err = esp_console_start_repl(repl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start REPL: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Debug console started — type 'help' for commands");
}

// ============================================================================
// Public init functions
// ============================================================================

void debug_console_init_safety(void) {
    ESP_LOGW(TAG, "*** DEBUG CONSOLE ENABLED — NOT FOR PRODUCTION ***");

    const esp_console_cmd_t cmds[] = {
        {
            .command = "bypass",
            .help = "Bypass heartbeat: bypass <planner|control|all>",
            .hint = nullptr,
            .func = &cmd_bypass,
            .argtable = nullptr,
        },
        {
            .command = "unbypass",
            .help = "Re-enable heartbeat: unbypass <planner|control|all>",
            .hint = nullptr,
            .func = &cmd_unbypass,
            .argtable = nullptr,
        },
        {
            .command = "status",
            .help = "Show current safety state",
            .hint = nullptr,
            .func = &cmd_safety_status,
            .argtable = nullptr,
        },
    };

    for (const auto &cmd : cmds) {
        esp_err_t reg_err = esp_console_cmd_register(&cmd);
        if (reg_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register command '%s': %s", cmd.command, esp_err_to_name(reg_err));
        }
    }

    start_repl();
}

void debug_console_init_control(void) {
    ESP_LOGW(TAG, "*** DEBUG CONSOLE ENABLED — NOT FOR PRODUCTION ***");

    const esp_console_cmd_t cmds[] = {
        {
            .command = "sim",
            .help = "Simulate: sim <fr|auto|planner|off> [args...]",
            .hint = nullptr,
            .func = &cmd_sim,
            .argtable = nullptr,
        },
        {
            .command = "status",
            .help = "Show current control state",
            .hint = nullptr,
            .func = &cmd_control_status,
            .argtable = nullptr,
        },
    };

    for (const auto &cmd : cmds) {
        esp_err_t reg_err = esp_console_cmd_register(&cmd);
        if (reg_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register command '%s': %s", cmd.command, esp_err_to_name(reg_err));
        }
    }

    start_repl();
}

#endif /* CONFIG_ENABLE_DEBUG_CONSOLE */
