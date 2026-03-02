/**
 * @file heartbeat_monitor.h
 * @brief Multi-node heartbeat timeout monitor with thread-safe access.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "can_protocol.h"

#ifdef __cplusplus
extern "C"
{
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

#define HEARTBEAT_MONITOR_MAX_NODES         8  // Maximum nodes per monitor
#define HEARTBEAT_MONITOR_TAG_MAX_LEN       32 // Maximum length of monitor tag
#define HEARTBEAT_MONITOR_NODE_NAME_MAX_LEN 16 // Maximum tracked node name length

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
typedef struct
{
	char name[HEARTBEAT_MONITOR_NODE_NAME_MAX_LEN];
	TickType_t timeout_ticks;
	TickType_t last_seen;
	heartbeat_seq_t last_sequence;
	node_state_t last_state;
	bool active;
	bool alive;
} heartbeat_monitor_node_t;

// heartbeat_monitor_t - monitor instance containing all tracked nodes
//   tag:        Debug label (e.g., "SAFETY_HB_MONITOR")
//   nodes:      Array of tracked nodes
//   node_count: Number of registered nodes
//   lock:       Spinlock for thread-safe access
typedef struct
{
	char tag[HEARTBEAT_MONITOR_TAG_MAX_LEN];
	heartbeat_monitor_node_t nodes[HEARTBEAT_MONITOR_MAX_NODES];
	uint8_t node_count;
	portMUX_TYPE lock;
} heartbeat_monitor_t;

// heartbeat_monitor_config_t - initialization configuration
//   name: Optional base name for monitor tag (e.g., "SAFETY" -> "SAFETY_HB")
typedef struct
{
	const char *name;
} heartbeat_monitor_config_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize a heartbeat monitor instance.
 *
 * Zeroes all node slots, sets the monitor tag from the config name,
 * and initializes the spinlock for thread-safe access.
 *
 * @param mon     Pointer to the monitor instance to initialize
 * @param config  Initialization configuration (monitor name)
 */
void heartbeat_monitor_init(heartbeat_monitor_t *mon, const heartbeat_monitor_config_t *config);

// ============================================================================
// Node Registration
// ============================================================================

/**
 * @brief Register a node to monitor.
 *
 * Adds a node to the monitor's tracking list. The node is initially
 * marked as not-alive until its first heartbeat is received.
 *
 * @param mon         Pointer to the monitor instance
 * @param name        Human-readable node name (truncated to NODE_NAME_MAX_LEN)
 * @param timeout_ms  Maximum interval (ms) between heartbeats before timeout
 * @return Node index (0 to MAX_NODES-1), or -1 if registration failed
 */
int heartbeat_monitor_register(heartbeat_monitor_t *mon, const char *name, uint32_t timeout_ms);

// ============================================================================
// Runtime Updates
// ============================================================================

/**
 * @brief Update a node's status on heartbeat reception.
 *
 * Records the current tick as last_seen, stores the latest sequence
 * number and node state, and marks the node as alive.
 *
 * @param mon       Pointer to the monitor instance
 * @param node_id   Index returned by heartbeat_monitor_register()
 * @param sequence  Heartbeat sequence number from the CAN frame
 * @param state     Node state reported in the heartbeat
 */
void heartbeat_monitor_update(heartbeat_monitor_t *mon, int node_id, heartbeat_seq_t sequence, node_state_t state);

/**
 * @brief Check all registered nodes for heartbeat timeout.
 *
 * Should be called periodically (e.g., every 100 ms). Marks any node
 * whose last_seen timestamp exceeds its configured timeout as dead.
 *
 * @param mon  Pointer to the monitor instance
 */
void heartbeat_monitor_check_timeouts(heartbeat_monitor_t *mon);

// ============================================================================
// Status Queries
// ============================================================================

/**
 * @brief Check if a specific node is alive.
 *
 * Returns whether the node identified by @p node_id has sent a heartbeat
 * within its configured timeout window.
 *
 * @param mon      Pointer to the monitor instance
 * @param node_id  Index returned by heartbeat_monitor_register()
 * @return true if the node is alive, false if timed out or invalid
 */
bool heartbeat_monitor_is_alive(heartbeat_monitor_t *mon, int node_id);

/**
 * @brief Check if all registered nodes are alive.
 *
 * Convenience function that returns true only when every registered
 * node has sent a heartbeat within its configured timeout.
 *
 * @param mon  Pointer to the monitor instance
 * @return true if all nodes are alive, false if any node has timed out
 */
bool heartbeat_monitor_all_alive(heartbeat_monitor_t *mon);

/**
 * @brief Get detailed status of a specific node.
 *
 * Copies the node's full tracking state into @p out_status, including
 * last_seen time, sequence number, reported state, and alive flag.
 *
 * @param mon         Pointer to the monitor instance
 * @param node_id     Index returned by heartbeat_monitor_register()
 * @param out_status  Pointer to structure to fill with node status
 * @return true on success, false if node_id is invalid or not registered
 */
bool heartbeat_monitor_get_status(heartbeat_monitor_t *mon, int node_id, heartbeat_monitor_node_t *out_status);

/**
 * @brief Get a bitmask of timed-out nodes.
 *
 * Returns a bitmask where bit N is set if node N has timed out.
 * Useful for compact status reporting and fault aggregation.
 *
 * @param mon  Pointer to the monitor instance
 * @return Bitmask of timed-out nodes (bit N set = node N timed out)
 */
uint8_t heartbeat_monitor_get_timeout_mask(heartbeat_monitor_t *mon);

// ============================================================================
// Debugging
// ============================================================================

/**
 * @brief Log the status of all monitored nodes.
 *
 * Prints each registered node's name, alive/dead state, last sequence
 * number, and last reported state via ESP_LOG. Intended for debugging.
 *
 * @param mon  Pointer to the monitor instance
 */
void heartbeat_monitor_log_status(heartbeat_monitor_t *mon);

#ifdef __cplusplus
}
#endif
