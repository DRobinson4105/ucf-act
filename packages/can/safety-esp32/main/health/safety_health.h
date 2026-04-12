#pragma once

/**
 * @file safety_health.h
 * @brief Health-related functions for the Safety ESP32 node.
 */

#include "can_protocol.h"

/** @brief Log that a component has been lost (forwarded from node_support). */
void log_component_lost(const char *name, const char *detail);

/** @brief Log that a component has been recovered (forwarded from node_support). */
void log_component_regained(const char *name);

/** @brief Mark a component as lost and log the failure (forwarded from node_support). */
void mark_component_lost(volatile bool *ready, const char *name, const char *detail);

/**
 * @brief Convert Safety fault flags to a logging string.
 *
 * @param fault_flags  Fault flags to convert
 * @return Static string label for logging
 */
const char *safety_fault_to_log_string(node_fault_t fault_flags);

/**
 * @brief Wait for CAN RX and heartbeat tasks to finish in-flight TWAI calls.
 *
 * Must be called after setting g_twai_ready = false, before touching
 * the TWAI driver for recovery. Forwarded from node_support.
 */
void quiesce_can_rx(void);

/**
 * @brief Re-initialize any failed components at a rate-limited cadence.
 *
 * Called every safety loop tick, but paced at RETRY_INTERVAL_MS to
 * avoid hammering init calls.  Retries are infinite with no cap.
 * Covers push button, RF remote, relay, ultrasonic, and TWAI.
 * TWAI recovery uses a full teardown/reinit cycle with an active
 * probe heartbeat to verify bus connectivity.
 */
void retry_failed_components(void);

/**
 * @brief Encode and transmit the Safety heartbeat frame on CAN.
 *
 * Atomically snapshots the current target state, e-stop fault code,
 * and sequence number, encodes them into the CAN heartbeat format,
 * and sends via TWAI.  Called from safety_task on state change
 * (immediate) and from heartbeat_task on the 100ms periodic timer.
 * Guards against concurrent recovery by checking g_twai_ready and
 * tracking in-flight TX count under spinlock.
 *
 * @param log_as_change  true for state-change-triggered sends (logged as "change"),
 *                       false for periodic sends (logged as "periodic")
 */
void send_safety_heartbeat(bool log_as_change);
