/**
 * @file heartbeat_monitor.hh
 * @brief Multi-node heartbeat timeout monitor with thread-safe access.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Heartbeat Monitor
// ============================================================================
// Allows Safety ESP32 to track liveness of CAN nodes by monitoring their heartbeat
// messages. Each node sends periodic heartbeats with sequence number and state.
// If a node's heartbeat is not received within its timeout, it's marked dead.
//
// Usage:
//   1. Initialize monitor with heartbeat_monitor_init()
//   2. Register nodes with heartbeat_monitor_register()
//   3. Call heartbeat_monitor_update() when heartbeat CAN frame received
//   4. Call heartbeat_monitor_check_timeouts() periodically
//   5. Query status with heartbeat_monitor_is_alive() or _all_alive()
// ============================================================================

// ============================================================================
// Constants
// ============================================================================

#define HEARTBEAT_MONITOR_MAX_NODES   8    // Maximum nodes per monitor
#define HEARTBEAT_MONITOR_TAG_MAX_LEN 32   // Maximum length of monitor tag

// ============================================================================
// Data Structures
// ============================================================================

// heartbeat_monitor_node_t - per-node tracking state
//   name:          Human-readable node identifier
//   timeout_ticks: Maximum time between heartbeats before node is dead
//   last_seen:     Tick count when last heartbeat received
//   last_sequence: Most recent sequence number from node
//   last_state:    Most recent state value from node
//   active:        true if node is registered
//   alive:         true if node is responding within timeout
typedef struct {
    const char *name;
    TickType_t timeout_ticks;
    TickType_t last_seen;
    uint8_t last_sequence;
    uint8_t last_state;
    bool active;
    bool alive;
} heartbeat_monitor_node_t;

// heartbeat_monitor_t - monitor instance containing all tracked nodes
//   tag:        Debug label (e.g., "SAFETY_HB_MONITOR")
//   nodes:      Array of tracked nodes
//   node_count: Number of registered nodes
//   lock:       Spinlock for thread-safe access
typedef struct {
    char tag[HEARTBEAT_MONITOR_TAG_MAX_LEN];
    heartbeat_monitor_node_t nodes[HEARTBEAT_MONITOR_MAX_NODES];
    uint8_t node_count;
    portMUX_TYPE lock;
} heartbeat_monitor_t;

// heartbeat_monitor_config_t - initialization configuration
//   name: Base name for monitor (e.g., "SAFETY" -> "SAFETY_HB_MONITOR")
typedef struct {
    const char *name;
} heartbeat_monitor_config_t;

// ============================================================================
// Initialization
// ============================================================================

// Initialize a heartbeat monitor instance
void heartbeat_monitor_init(heartbeat_monitor_t *mon, const heartbeat_monitor_config_t *config);

// ============================================================================
// Node Registration
// ============================================================================

// Register a node to monitor
// Returns node index (0 to MAX_NODES-1), or -1 if registration failed
int heartbeat_monitor_register(heartbeat_monitor_t *mon,
                                const char *name,
                                uint32_t timeout_ms);

// ============================================================================
// Runtime Updates
// ============================================================================

// Call when heartbeat CAN frame received from a node
// Updates last_seen timestamp, sequence, and state
void heartbeat_monitor_update(heartbeat_monitor_t *mon,
                               int node_id,
                               uint8_t sequence,
                               uint8_t state);

// Check all nodes for timeout - call periodically (e.g., every 100ms)
// Marks nodes as dead if last_seen exceeds their timeout
void heartbeat_monitor_check_timeouts(heartbeat_monitor_t *mon);

// ============================================================================
// Status Queries
// ============================================================================

// Check if a specific node is alive (responding within timeout)
bool heartbeat_monitor_is_alive(heartbeat_monitor_t *mon, int node_id);

// Check if all registered nodes are alive
bool heartbeat_monitor_all_alive(heartbeat_monitor_t *mon);

// Get detailed status of a specific node
// Returns false if node_id is invalid or not registered
bool heartbeat_monitor_get_status(heartbeat_monitor_t *mon,
                                   int node_id,
                                   heartbeat_monitor_node_t *out_status);

// Get bitmask of timed-out nodes (bit N set = node N timed out)
uint8_t heartbeat_monitor_get_timeout_mask(heartbeat_monitor_t *mon);

// ============================================================================
// Debugging
// ============================================================================

// Log status of all monitored nodes (for debugging)
void heartbeat_monitor_log_status(heartbeat_monitor_t *mon);

#ifdef __cplusplus
}
#endif
