/**
 * @file node_utils.h
 * @brief Shared utility functions for ESP32 node main applications.
 *
 * Header-only (static inline) so that each translation unit picks up its own
 * TAG for ESP_LOG calls. Both control-esp32 and safety-esp32 use these
 * identically; keeping a single source of truth avoids copy-paste divergence.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// Time
// ============================================================================

/**
 * @brief 32-bit millisecond uptime (wraps at ~49.7 days).
 *
 * All elapsed-time checks should use unsigned subtraction, which remains
 * correct across a single wrap for intervals << 2^31 ms.
 */
static inline uint32_t node_get_time_ms(void)
{
	return (uint32_t)(esp_timer_get_time() / 1000);
}

// ============================================================================
// Task Watchdog Helpers
// ============================================================================

/**
 * @brief Add the calling task to the task watchdog, logging on failure.
 * @param tag  ESP_LOG tag (typically the file-scope TAG)
 * @param task_name  Human-readable task name for the log message
 */
static inline void node_task_wdt_add_self_or_log(const char *tag, const char *task_name)
{
	esp_err_t err = esp_task_wdt_add(NULL);
	if (err != ESP_OK)
	{
		ESP_LOGE(tag, "%s: esp_task_wdt_add failed: %s", task_name, esp_err_to_name(err));
	}
}

/**
 * @brief Reset the task watchdog, logging on first failure and on recovery.
 * @param tag  ESP_LOG tag
 * @param task_name  Human-readable task name for the log message
 * @param had_failure  Pointer to a bool tracking whether previous reset failed
 */
static inline void node_task_wdt_reset_or_log(const char *tag, const char *task_name, bool *had_failure)
{
	if (!had_failure)
		return;
	esp_err_t err = esp_task_wdt_reset();
	if (err != ESP_OK)
	{
		if (!*had_failure)
		{
			ESP_LOGE(tag, "%s: esp_task_wdt_reset failed: %s", task_name, esp_err_to_name(err));
			*had_failure = true;
		}
	}
	else if (*had_failure)
	{
		ESP_LOGI(tag, "%s: esp_task_wdt_reset recovered", task_name);
		*had_failure = false;
	}
}

// ============================================================================
// Component Health Logging
// ============================================================================

/**
 * @brief Log that a component has been lost (transition to unhealthy).
 * @param tag     ESP_LOG tag
 * @param name    Component name (e.g. "DAC")
 * @param detail  Optional detail string (may be NULL or empty)
 */
static inline void node_log_component_lost(const char *tag, const char *name, const char *detail)
{
#ifdef CONFIG_LOG_COMPONENT_HEALTH_TRANSITIONS
	if (detail && detail[0] != '\0')
	{
		ESP_LOGE(tag, "%s: LOST: %s", name, detail);
	}
	else
	{
		ESP_LOGE(tag, "%s: LOST", name);
	}
#else
	(void)tag;
	(void)name;
	(void)detail;
#endif
}

/**
 * @brief Log that a component has been regained (transition to healthy).
 * @param tag   ESP_LOG tag
 * @param name  Component name
 */
static inline void node_log_component_regained(const char *tag, const char *name)
{
#ifdef CONFIG_LOG_COMPONENT_HEALTH_TRANSITIONS
	ESP_LOGI(tag, "%s: REGAINED", name);
#else
	(void)tag;
	(void)name;
#endif
}

/**
 * @brief Mark a component as lost: clear its ready flag and log the transition.
 * @param ready   Pointer to the volatile bool ready flag
 * @param tag     ESP_LOG tag
 * @param name    Component name
 * @param detail  Optional detail string
 */
static inline void node_mark_component_lost(volatile bool *ready, const char *tag, const char *name, const char *detail)
{
	if (!ready)
		return;
	if (*ready)
	{
		node_log_component_lost(tag, name, detail);
	}
	*ready = false;
}

#ifdef __cplusplus
}
#endif
