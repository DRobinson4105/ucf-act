/**
 * @file main.cpp
 * @brief Safety ESP32 entry point — production build guards and app_main.
 *
 * All runtime logic lives in focused modules under subdirectories:
 *   config/    — compile-time constants (safety_config.h)
 *   state/     — global state declarations (safety_globals.h/.cpp)
 *   health/    — component health, recovery orchestration
 *   tasks/     — FreeRTOS task entry points (CAN RX, safety loop, heartbeat)
 *   init/      — peripheral bring-up and task creation (main_task)
 *   transport/ — CAN RX decode, heartbeat TX, Orin UART link
 *   inputs/    — local stop-input polling (push button, RF remote, ultrasonic)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "safety_init.h"

// ============================================================================
// Production Build Guard
// ============================================================================
// If CONFIG_PRODUCTION_BUILD is set, all BYPASS_* flags must be disabled.
// This prevents accidental deployment of bench-test firmware onto the vehicle.
#ifdef CONFIG_PRODUCTION_BUILD
#ifdef CONFIG_BYPASS_PLANNER_AUTONOMY_GATE
#error "PRODUCTION_BUILD: CONFIG_BYPASS_PLANNER_AUTONOMY_GATE must be disabled"
#endif
#ifdef CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS
#error "PRODUCTION_BUILD: CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS must be disabled"
#endif
#ifdef CONFIG_BYPASS_PLANNER_STATE_MIRROR
#error "PRODUCTION_BUILD: CONFIG_BYPASS_PLANNER_STATE_MIRROR must be disabled"
#endif
#ifdef CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS
#error "PRODUCTION_BUILD: CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS must be disabled"
#endif
#ifdef CONFIG_BYPASS_CONTROL_STATE_MIRROR
#error "PRODUCTION_BUILD: CONFIG_BYPASS_CONTROL_STATE_MIRROR must be disabled"
#endif
#ifdef CONFIG_BYPASS_INPUT_PUSH_BUTTON
#error "PRODUCTION_BUILD: CONFIG_BYPASS_INPUT_PUSH_BUTTON must be disabled"
#endif
#ifdef CONFIG_BYPASS_INPUT_RF_REMOTE
#error "PRODUCTION_BUILD: CONFIG_BYPASS_INPUT_RF_REMOTE must be disabled"
#endif
#ifdef CONFIG_BYPASS_INPUT_ULTRASONIC
#error "PRODUCTION_BUILD: CONFIG_BYPASS_INPUT_ULTRASONIC must be disabled"
#endif
#ifdef CONFIG_BYPASS_INPUT_BATTERY_MONITOR
#error "PRODUCTION_BUILD: CONFIG_BYPASS_INPUT_BATTERY_MONITOR must be disabled"
#endif
#ifdef CONFIG_BYPASS_CAN_TWAI
#error "PRODUCTION_BUILD: CONFIG_BYPASS_CAN_TWAI must be disabled"
#endif
#endif // CONFIG_PRODUCTION_BUILD

/**
 * @brief ESP-IDF application entry point for the Safety ESP32.
 *
 * Creates the main_task on a dedicated stack to perform peripheral
 * initialization and spawn runtime tasks.  Restarts the system if
 * task creation fails.
 */
extern "C" void app_main(void)
{
	if (xTaskCreate(main_task, "main_task", 8192, nullptr, 5, nullptr) != pdPASS)
	{
		ESP_LOGE("SAFETY_INIT", "Failed to create main_task, restarting");
		esp_restart();
	}
}
