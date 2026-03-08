/**
 * @file can_tx_track.h
 * @brief Pure-function CAN TX failure tracking.
 *
 * Header-only (static inline) so both control-esp32 and safety-esp32 share
 * the same logic without creating a library dependency.  No FreeRTOS or
 * ESP-IDF includes — safe for host-side unit tests.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// CAN TX Failure Tracking
// ============================================================================

typedef struct
{
	uint8_t fail_count; /**< current consecutive failure count */
	uint8_t threshold;  /**< trigger recovery at this count */
	bool tx_ok;         /**< result of last TX */
} can_tx_track_inputs_t;

typedef struct
{
	uint8_t new_fail_count; /**< updated count */
	bool trigger_recovery;  /**< true if threshold reached */
} can_tx_track_result_t;

/**
 * @brief Track CAN TX success/failure and determine if recovery is needed.
 *
 * Pure function — no side effects, no global state.
 *
 * @param inputs  Current TX result and failure tracking state
 * @return Updated count and whether recovery should be triggered
 */
static inline can_tx_track_result_t can_tx_track(const can_tx_track_inputs_t *inputs)
{
	can_tx_track_result_t r = {.new_fail_count = 0, .trigger_recovery = false};

	if (!inputs)
		return r;

	if (inputs->tx_ok)
	{
		r.new_fail_count = 0;
		return r;
	}

	r.new_fail_count = (inputs->fail_count < 255) ? (uint8_t)(inputs->fail_count + 1) : 255;
	if (r.new_fail_count >= inputs->threshold)
	{
		r.trigger_recovery = true;
	}

	return r;
}

#ifdef __cplusplus
}
#endif
