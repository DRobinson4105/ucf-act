/**
 * @file obstacle_filter.h
 * @brief Pure obstacle filtering helpers for sustained-presence checks.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	uint32_t engage_hold_ms; // required sustained asserted time before latching
	uint8_t disengage_count; // consecutive clear samples required to unlatch
	uint32_t assert_since_ms; // timestamp when the current asserted streak began
	uint8_t clear_count;      // consecutive clear samples while latched
	bool assert_pending;      // true once an asserted streak has started
	bool latched;             // filtered output
} obstacle_filter_t;

/**
 * @brief Update an obstacle sustained-presence filter.
 *
 * Latches the output only after raw_asserted stays true for at least
 * engage_hold_ms. Once latched, requires disengage_count consecutive clear
 * samples to release.
 *
 * @param filter        Mutable filter state/config
 * @param raw_asserted  Raw input state for the latest sample
 * @param now_ms        Monotonic timestamp in milliseconds
 * @return Filtered latched state after the update
 */
bool obstacle_filter_update(obstacle_filter_t *filter, bool raw_asserted, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
