#include "heartbeat_monitor.hh"

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "freertos/portmacro.h"

// =============================================================================
// Initialization
// =============================================================================

void heartbeat_monitor_init(heartbeat_monitor_t *mon, const heartbeat_monitor_config_t *config) {
    if (!mon || !config || !config->name) return;

    snprintf(mon->tag, sizeof(mon->tag), "%s_HB_MONITOR", config->name);
    memset(mon->nodes, 0, sizeof(mon->nodes));
    mon->node_count = 0;
    portMUX_INITIALIZE(&mon->lock);

    ESP_LOGI(mon->tag, "Heartbeat monitor initialized");
}

// =============================================================================
// Node Registration
// =============================================================================

// Register a CAN node to monitor - returns node_id for future reference
int heartbeat_monitor_register(heartbeat_monitor_t *mon,
                                const char *name,
                                uint32_t timeout_ms) {
    if (!mon || mon->node_count >= HEARTBEAT_MONITOR_MAX_NODES) return -1;

    taskENTER_CRITICAL(&mon->lock);
    int node_id = mon->node_count++;
    mon->nodes[node_id].name = name ? name : "unknown";
    mon->nodes[node_id].timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    mon->nodes[node_id].last_seen = 0;
    mon->nodes[node_id].last_sequence = 0;
    mon->nodes[node_id].last_state = 0;
    mon->nodes[node_id].active = true;
    mon->nodes[node_id].alive = false;  // Not alive until first heartbeat received
    taskEXIT_CRITICAL(&mon->lock);

    ESP_LOGI(mon->tag, "Registered %s (timeout %lu ms)", name, (unsigned long)timeout_ms);
    return node_id;
}

// =============================================================================
// Runtime Updates
// =============================================================================

// Call when heartbeat CAN frame received - updates timestamp and marks node alive
void heartbeat_monitor_update(heartbeat_monitor_t *mon,
                               int node_id,
                               uint8_t sequence,
                               uint8_t state) {
    if (!mon || node_id < 0 || node_id >= mon->node_count) return;

    TickType_t now = xTaskGetTickCount();
    bool was_alive;
    bool ever_seen;

    taskENTER_CRITICAL(&mon->lock);
    if (!mon->nodes[node_id].active) {
        taskEXIT_CRITICAL(&mon->lock);
        return;
    }

    was_alive = mon->nodes[node_id].alive;
    ever_seen = mon->nodes[node_id].last_seen > 0;
    mon->nodes[node_id].last_seen = now;
    mon->nodes[node_id].last_sequence = sequence;
    mon->nodes[node_id].last_state = state;
    mon->nodes[node_id].alive = true;
    const char *name = mon->nodes[node_id].name;
    taskEXIT_CRITICAL(&mon->lock);

    // Log state transitions for debugging
    if (!was_alive) {
        if (!ever_seen) ESP_LOGI(mon->tag, "%s ALIVE", name);
        else ESP_LOGI(mon->tag, "%s regained", name);
    }
}

// Check all nodes for timeout - call periodically (e.g., every 50-100ms)
void heartbeat_monitor_check_timeouts(heartbeat_monitor_t *mon) {
    if (!mon) return;

    TickType_t now = xTaskGetTickCount();

    taskENTER_CRITICAL(&mon->lock);
    for (int i = 0; i < mon->node_count; i++) {
        if (!mon->nodes[i].active) continue;

        TickType_t elapsed = now - mon->nodes[i].last_seen;
        bool was_alive = mon->nodes[i].alive;

        // Only timeout if we've seen at least one heartbeat (last_seen > 0)
        if (mon->nodes[i].last_seen > 0 && elapsed > mon->nodes[i].timeout_ticks) {
            mon->nodes[i].alive = false;
            if (was_alive) {
                const char *name = mon->nodes[i].name;
                taskEXIT_CRITICAL(&mon->lock);
                ESP_LOGW(mon->tag, "%s lost (no response for %lu ms)",
                         name, (unsigned long)(elapsed * portTICK_PERIOD_MS));
                taskENTER_CRITICAL(&mon->lock);
            }
        }
    }
    taskEXIT_CRITICAL(&mon->lock);
}

// =============================================================================
// Status Queries
// =============================================================================

bool heartbeat_monitor_is_alive(heartbeat_monitor_t *mon, int node_id) {
    if (!mon || node_id < 0 || node_id >= mon->node_count) return false;

    bool alive;
    taskENTER_CRITICAL(&mon->lock);
    alive = mon->nodes[node_id].active && mon->nodes[node_id].alive;
    taskEXIT_CRITICAL(&mon->lock);
    return alive;
}

bool heartbeat_monitor_all_alive(heartbeat_monitor_t *mon) {
    if (!mon) return false;

    bool all_alive = true;
    taskENTER_CRITICAL(&mon->lock);
    for (int i = 0; i < mon->node_count; i++) {
        if (mon->nodes[i].active && !mon->nodes[i].alive) {
            all_alive = false;
            break;
        }
    }
    taskEXIT_CRITICAL(&mon->lock);
    return all_alive;
}

bool heartbeat_monitor_get_status(heartbeat_monitor_t *mon,
                                   int node_id,
                                   heartbeat_monitor_node_t *out_status) {
    if (!mon || node_id < 0 || node_id >= mon->node_count || !out_status) return false;

    taskENTER_CRITICAL(&mon->lock);
    if (!mon->nodes[node_id].active) {
        taskEXIT_CRITICAL(&mon->lock);
        return false;
    }
    *out_status = mon->nodes[node_id];
    taskEXIT_CRITICAL(&mon->lock);
    return true;
}

// Returns bitmask where bit N set = node N has timed out
uint8_t heartbeat_monitor_get_timeout_mask(heartbeat_monitor_t *mon) {
    if (!mon) return 0xFF;

    uint8_t mask = 0;
    taskENTER_CRITICAL(&mon->lock);
    for (int i = 0; i < mon->node_count && i < 8; i++)
        if (mon->nodes[i].active && !mon->nodes[i].alive) mask |= (1 << i);
	
    taskEXIT_CRITICAL(&mon->lock);
    return mask;
}

// =============================================================================
// Debugging
// =============================================================================

void heartbeat_monitor_log_status(heartbeat_monitor_t *mon) {
    if (!mon) return;

    ESP_LOGI(mon->tag, "--- Node Status ---");

    int alive_count = 0;
    int dead_count = 0;
    int never_seen_count = 0;

    taskENTER_CRITICAL(&mon->lock);
    for (int i = 0; i < mon->node_count; i++) {
        if (!mon->nodes[i].active) continue;

        const char *name = mon->nodes[i].name;
        bool alive = mon->nodes[i].alive;
        TickType_t last_seen = mon->nodes[i].last_seen;

        taskEXIT_CRITICAL(&mon->lock);

        if (alive) {
            ESP_LOGI(mon->tag, "  [OK]  %s", name);
            alive_count++;
        } else if (last_seen == 0) {
            ESP_LOGW(mon->tag, "  [--]  %s (never seen)", name);
            never_seen_count++;
        } else {
            TickType_t elapsed = xTaskGetTickCount() - last_seen;
            ESP_LOGW(mon->tag, "  [!!]  %s (dead, last seen %lu ms ago)",
                     name, (unsigned long)(elapsed * portTICK_PERIOD_MS));
            dead_count++;
        }

        taskENTER_CRITICAL(&mon->lock);
    }
    taskEXIT_CRITICAL(&mon->lock);

    ESP_LOGI(mon->tag, "Summary: %d alive, %d dead, %d never seen",
             alive_count, dead_count, never_seen_count);
}
