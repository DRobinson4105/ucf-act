/**
 * @file heartbeat_monitor.cpp
 * @brief Heartbeat monitor implementation with thread-safe timeout tracking.
 */

#include "heartbeat_monitor.h"

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "freertos/portmacro.h"

// ============================================================================
// Initialization
// ============================================================================

void heartbeat_monitor_init(heartbeat_monitor_t *mon, const heartbeat_monitor_config_t *config)
{
	if (!mon)
		return;

	if (config && config->name && config->name[0] != '\0')
		snprintf(mon->tag, sizeof(mon->tag), "%s", config->name);
	else
		snprintf(mon->tag, sizeof(mon->tag), "HEARTBEAT");

	memset(mon->nodes, 0, sizeof(mon->nodes));
	mon->node_count = 0;
	portMUX_INITIALIZE(&mon->lock);
}

// ============================================================================
// Node Registration
// ============================================================================

int heartbeat_monitor_register(heartbeat_monitor_t *mon, const char *name, uint32_t timeout_ms)
{
	if (!mon)
		return -1;
	const char *effective_name = (name && name[0] != '\0') ? name : "unknown";

	taskENTER_CRITICAL(&mon->lock);
	if (mon->node_count >= HEARTBEAT_MONITOR_MAX_NODES)
	{
		taskEXIT_CRITICAL(&mon->lock);
		return -1;
	}
	int node_id = mon->node_count++;
	strncpy(mon->nodes[node_id].name, effective_name, sizeof(mon->nodes[node_id].name) - 1);
	mon->nodes[node_id].name[sizeof(mon->nodes[node_id].name) - 1] = '\0';
	mon->nodes[node_id].timeout_ticks = pdMS_TO_TICKS(timeout_ms);
	mon->nodes[node_id].last_seen = 0;
	mon->nodes[node_id].seen_heartbeat = false;
	mon->nodes[node_id].last_sequence = 0;
	mon->nodes[node_id].last_state = 0;
	mon->nodes[node_id].active = true;
	mon->nodes[node_id].alive = false; // Not alive until first heartbeat received
	taskEXIT_CRITICAL(&mon->lock);

	return node_id;
}

// ============================================================================
// Runtime Updates
// ============================================================================

void heartbeat_monitor_update(heartbeat_monitor_t *mon, int node_id, node_seq_t sequence, node_state_t state)
{
	if (!mon || node_id < 0)
		return;

	TickType_t now = xTaskGetTickCount();

	taskENTER_CRITICAL(&mon->lock);
	if (node_id >= mon->node_count)
	{
		taskEXIT_CRITICAL(&mon->lock);
		return;
	}
	if (!mon->nodes[node_id].active)
	{
		taskEXIT_CRITICAL(&mon->lock);
		return;
	}

#ifdef CONFIG_LOG_HEARTBEAT_MONITOR_TRANSITIONS
	bool was_alive = mon->nodes[node_id].alive;
	bool had_seen_heartbeat = mon->nodes[node_id].seen_heartbeat;
	char name[HEARTBEAT_MONITOR_NODE_NAME_MAX_LEN] = {};
	strncpy(name, mon->nodes[node_id].name, sizeof(name) - 1);
	name[sizeof(name) - 1] = '\0';
#endif

	mon->nodes[node_id].last_seen = now;
	mon->nodes[node_id].seen_heartbeat = true;
	mon->nodes[node_id].last_sequence = sequence;
	mon->nodes[node_id].last_state = state;
	mon->nodes[node_id].alive = true;
	taskEXIT_CRITICAL(&mon->lock);

#ifdef CONFIG_LOG_HEARTBEAT_MONITOR_TRANSITIONS
	if (!was_alive && had_seen_heartbeat)
		ESP_LOGI(mon->tag, "%s regained", name);
#endif
}

void heartbeat_monitor_check_timeouts(heartbeat_monitor_t *mon)
{
	if (!mon)
		return;

	TickType_t now = xTaskGetTickCount();

	// Buffer timeout events inside the critical section, log outside.
	// This avoids releasing/re-acquiring the lock mid-loop which could
	// allow another task to mutate node_count or node state.
	struct
	{
		char name[HEARTBEAT_MONITOR_NODE_NAME_MAX_LEN];
		unsigned long elapsed_ms;
	} timeouts[HEARTBEAT_MONITOR_MAX_NODES];
	int timeout_count = 0;

	taskENTER_CRITICAL(&mon->lock);
	for (int i = 0; i < mon->node_count; i++)
	{
		if (!mon->nodes[i].active)
			continue;

		TickType_t elapsed = now - mon->nodes[i].last_seen;
		bool was_alive = mon->nodes[i].alive;

		// Only timeout if we've seen at least one heartbeat.
		if (mon->nodes[i].seen_heartbeat && elapsed > mon->nodes[i].timeout_ticks)
		{
			mon->nodes[i].alive = false;
			if (was_alive && timeout_count < HEARTBEAT_MONITOR_MAX_NODES)
			{
				strncpy(timeouts[timeout_count].name, mon->nodes[i].name, sizeof(timeouts[timeout_count].name) - 1);
				timeouts[timeout_count].name[sizeof(timeouts[timeout_count].name) - 1] = '\0';
				timeouts[timeout_count].elapsed_ms = (unsigned long)(elapsed * portTICK_PERIOD_MS);
				timeout_count++;
			}
		}
	}
	taskEXIT_CRITICAL(&mon->lock);

	// Log outside critical section
#ifdef CONFIG_LOG_HEARTBEAT_MONITOR_TRANSITIONS
	for (int i = 0; i < timeout_count; i++)
	{
		ESP_LOGE(mon->tag, "%s lost (no response for %lu ms)", timeouts[i].name, timeouts[i].elapsed_ms);
	}
#else
	(void)timeout_count;
#endif
}

// ============================================================================
// Status Queries
// ============================================================================

bool heartbeat_monitor_is_alive(heartbeat_monitor_t *mon, int node_id)
{
	if (!mon || node_id < 0)
		return false;

	bool alive;
	taskENTER_CRITICAL(&mon->lock);
	if (node_id >= mon->node_count)
	{
		taskEXIT_CRITICAL(&mon->lock);
		return false;
	}
	alive = mon->nodes[node_id].active && mon->nodes[node_id].alive;
	taskEXIT_CRITICAL(&mon->lock);
	return alive;
}
