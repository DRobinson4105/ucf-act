/**
 * @file main.cpp
 * @brief Control ESP32 entry point — production build guards and app_main.
 *
 * All runtime logic lives in focused modules under subdirectories:
 *   config/   — compile-time constants (control_config.h)
 *   state/    — global state declarations (control_globals.h/.cpp)
 *   health/   — component health, fault gating, recovery
 *   actuators/— enable/disable/override action helpers
 *   tasks/    — FreeRTOS task entry points (CAN RX, control loop, heartbeat)
 *   init/     — peripheral bring-up and task creation (main_task)
 *   transport/— CAN RX decode, heartbeat TX, Orin UART link
 *   inputs/   — driver input polling (pedal ADC, F/R sensor)
 *   bench/    — standalone actuator bench-test mode
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "control_init.h"

// ============================================================================
// Production Build Guard
// ============================================================================
// If CONFIG_PRODUCTION_BUILD is set, all BYPASS_* flags must be disabled.
// This prevents accidental deployment of bench-test firmware onto the vehicle.
#ifdef CONFIG_PRODUCTION_BUILD
#ifdef CONFIG_BYPASS_SAFETY_TARGET_MIRROR
#error "PRODUCTION_BUILD: CONFIG_BYPASS_SAFETY_TARGET_MIRROR must be disabled"
#endif
#ifdef CONFIG_BYPASS_SAFETY_ESTOP_MIRROR
#error "PRODUCTION_BUILD: CONFIG_BYPASS_SAFETY_ESTOP_MIRROR must be disabled"
#endif
#ifdef CONFIG_BYPASS_SAFETY_LIVENESS_CHECKS
#error "PRODUCTION_BUILD: CONFIG_BYPASS_SAFETY_LIVENESS_CHECKS must be disabled"
#endif
#ifdef CONFIG_BYPASS_PLANNER_COMMAND_INPUTS
#error "PRODUCTION_BUILD: CONFIG_BYPASS_PLANNER_COMMAND_INPUTS must be disabled"
#endif
#ifdef CONFIG_BYPASS_PLANNER_COMMAND_STALE_CHECKS
#error "PRODUCTION_BUILD: CONFIG_BYPASS_PLANNER_COMMAND_STALE_CHECKS must be disabled"
#endif
#ifdef CONFIG_BYPASS_INPUT_PEDAL_ADC
#error "PRODUCTION_BUILD: CONFIG_BYPASS_INPUT_PEDAL_ADC must be disabled"
#endif
#ifdef CONFIG_BYPASS_INPUT_FR_SENSOR
#error "PRODUCTION_BUILD: CONFIG_BYPASS_INPUT_FR_SENSOR must be disabled"
#endif
#ifdef CONFIG_BYPASS_ACTUATOR_THROTTLE
#error "PRODUCTION_BUILD: CONFIG_BYPASS_ACTUATOR_THROTTLE must be disabled"
#endif
#ifdef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
#error "PRODUCTION_BUILD: CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING must be disabled"
#endif
#ifdef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
#error "PRODUCTION_BUILD: CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING must be disabled"
#endif
#ifdef CONFIG_CONTROL_TEST_MODE
#error "PRODUCTION_BUILD: CONFIG_CONTROL_TEST_MODE must be disabled"
#endif
#endif // CONFIG_PRODUCTION_BUILD

/**
 * @brief ESP-IDF application entry point for the Control ESP32.
 *
 * Creates the main_task on a dedicated stack to perform peripheral
 * initialization and spawn runtime tasks.  Restarts the system if
 * task creation fails.
 */
extern "C" void app_main(void)
{
	if (xTaskCreate(main_task, "main_task", 8192, nullptr, 5, nullptr) != pdPASS)
	{
		ESP_LOGE("CONTROL_INIT", "Failed to create main_task, restarting");
		esp_restart();
	}
}
