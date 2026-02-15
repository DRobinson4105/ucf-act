/**
 * @file heartbeat_monitor.cpp
 * @brief Heartbeat monitor implementation with thread-safe timeout tracking.
 */

#include "heartbeat_monitor.hh"

#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "freertos/portmacro.h"

// ============================================================================
// Initialization
// ============================================================================

void heartbeat_monitor_init(heartbeat_monitor_t *mon, const heartbeat_monitor_config_t *config) {
    if (!mon) return;

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

// Register a CAN node to monitor - returns node_id for future reference
int heartbeat_monitor_register(heartbeat_monitor_t *mon,
                                 const char *name,
                                 uint32_t timeout_ms) {
    if (!mon) return -1;
    const char *effective_name = (name && name[0] != '\0') ? name : "unknown";

    taskENTER_CRITICAL(&mon->lock);
    if (mon->node_count >= HEARTBEAT_MONITOR_MAX_NODES) {
        taskEXIT_CRITICAL(&mon->lock);
        return -1;
    }
    int node_id = mon->node_count++;
    strncpy(mon->nodes[node_id].name, effective_name, sizeof(mon->nodes[node_id].name) - 1);
    mon->nodes[node_id].name[sizeof(mon->nodes[node_id].name) - 1] = '\0';
    mon->nodes[node_id].timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    mon->nodes[node_id].last_seen = 0;
    mon->nodes[node_id].last_sequence = 0;
    mon->nodes[node_id].last_state = 0;
    mon->nodes[node_id].active = true;
    mon->nodes[node_id].alive = false;  // Not alive until first heartbeat received
    taskEXIT_CRITICAL(&mon->lock);

    return node_id;
}

// ============================================================================
// Runtime Updates
// ============================================================================

// Call when heartbeat CAN frame received - updates timestamp and marks node alive
void heartbeat_monitor_update(heartbeat_monitor_t *mon,
                               int node_id,
                               uint8_t sequence,
                               uint8_t state) {
    if (!mon || node_id < 0 || node_id >= mon->node_count) return;

    TickType_t now = xTaskGetTickCount();
    bool was_alive;
    bool ever_seen;
    char name[HEARTBEAT_MONITOR_NODE_NAME_MAX_LEN] = {0};

    taskENTER_CRITICAL(&mon->lock);
    if (!mon->nodes[node_id].active) {
        taskEXIT_CRITICAL(&mon->lock);
        return;
    }

    was_alive = mon->nodes[node_id].alive;
    ever_seen = mon->nodes[node_id].last_seen > 0;
    mon->nodes[node_id].last_seen = (now == 0) ? 1 : now;
    mon->nodes[node_id].last_sequence = sequence;
    mon->nodes[node_id].last_state = state;
    mon->nodes[node_id].alive = true;
    strncpy(name, mon->nodes[node_id].name, sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
    taskEXIT_CRITICAL(&mon->lock);

    // Log lost/regained transitions independently from frame RX logs.
#ifdef CONFIG_LOG_HEARTBEAT_MONITOR_TRANSITIONS
    if (!was_alive) {
        if (ever_seen) ESP_LOGI(mon->tag, "%s regained", name);
    }
#else
    (void)was_alive;
    (void)ever_seen;
    (void)name;
#endif
}

// Check all nodes for timeout - call periodically (e.g., every 50-100ms)
void heartbeat_monitor_check_timeouts(heartbeat_monitor_t *mon) {
    if (!mon) return;

    TickType_t now = xTaskGetTickCount();

    // Buffer timeout events inside the critical section, log outside.
    // This avoids releasing/re-acquiring the lock mid-loop which could
    // allow another task to mutate node_count or node state.
    struct {
        char name[HEARTBEAT_MONITOR_NODE_NAME_MAX_LEN];
        unsigned long elapsed_ms;
    } timeouts[HEARTBEAT_MONITOR_MAX_NODES];
    int timeout_count = 0;

    taskENTER_CRITICAL(&mon->lock);
    for (int i = 0; i < mon->node_count; i++) {
        if (!mon->nodes[i].active) continue;

        TickType_t elapsed = now - mon->nodes[i].last_seen;
        bool was_alive = mon->nodes[i].alive;

        // Only timeout if we've seen at least one heartbeat (last_seen > 0)
        if (mon->nodes[i].last_seen > 0 && elapsed > mon->nodes[i].timeout_ticks) {
            mon->nodes[i].alive = false;
            if (was_alive && timeout_count < HEARTBEAT_MONITOR_MAX_NODES) {
                strncpy(timeouts[timeout_count].name, mon->nodes[i].name,
                        sizeof(timeouts[timeout_count].name) - 1);
                timeouts[timeout_count].name[sizeof(timeouts[timeout_count].name) - 1] = '\0';
                timeouts[timeout_count].elapsed_ms = (unsigned long)(elapsed * portTICK_PERIOD_MS);
                timeout_count++;
            }
        }
    }
    taskEXIT_CRITICAL(&mon->lock);

    // Log outside critical section
#ifdef CONFIG_LOG_HEARTBEAT_MONITOR_TRANSITIONS
    for (int i = 0; i < timeout_count; i++) {
        ESP_LOGI(mon->tag, "%s lost (no response for %lu ms)",
                 timeouts[i].name, timeouts[i].elapsed_ms);
    }
#else
    (void)timeout_count;
#endif
}

// ============================================================================
// Status Queries
// ============================================================================

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

// ============================================================================
// Debugging
// ============================================================================

void heartbeat_monitor_log_status(heartbeat_monitor_t *mon) {
    if (!mon) return;

#ifdef CONFIG_LOG_CAN_HEARTBEAT_RX
    ESP_LOGI(mon->tag, "--- Node Status ---");

    // Snapshot all node data inside a single critical section, log outside.
    struct {
        char name[HEARTBEAT_MONITOR_NODE_NAME_MAX_LEN];
        bool alive;
        TickType_t last_seen;
    } snaps[HEARTBEAT_MONITOR_MAX_NODES];
    int snap_count = 0;

    taskENTER_CRITICAL(&mon->lock);
    for (int i = 0; i < mon->node_count && i < HEARTBEAT_MONITOR_MAX_NODES; i++) {
        if (!mon->nodes[i].active) continue;
        strncpy(snaps[snap_count].name, mon->nodes[i].name, sizeof(snaps[snap_count].name) - 1);
        snaps[snap_count].name[sizeof(snaps[snap_count].name) - 1] = '\0';
        snaps[snap_count].alive = mon->nodes[i].alive;
        snaps[snap_count].last_seen = mon->nodes[i].last_seen;
        snap_count++;
    }
    taskEXIT_CRITICAL(&mon->lock);

    int alive_count = 0, dead_count = 0, never_seen_count = 0;
    TickType_t now = xTaskGetTickCount();

    for (int i = 0; i < snap_count; i++) {
        if (snaps[i].alive) {
            ESP_LOGI(mon->tag, "  [OK]  %s", snaps[i].name);
            alive_count++;
        } else if (snaps[i].last_seen == 0) {
            ESP_LOGI(mon->tag, "  [--]  %s (never seen)", snaps[i].name);
            never_seen_count++;
        } else {
            TickType_t elapsed = now - snaps[i].last_seen;
            ESP_LOGI(mon->tag, "  [!!]  %s (dead, last seen %lu ms ago)",
                     snaps[i].name, (unsigned long)(elapsed * portTICK_PERIOD_MS));
            dead_count++;
        }
    }

    ESP_LOGI(mon->tag, "Summary: %d alive, %d dead, %d never seen",
             alive_count, dead_count, never_seen_count);
#endif
}
