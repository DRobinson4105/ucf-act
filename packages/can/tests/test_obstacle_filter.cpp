/**
 * @file test_obstacle_filter.cpp
 * @brief Unit tests for pure obstacle filtering helpers.
 */

#include "test_harness.h"

#include <stdint.h>

#include "obstacle_filter.h"

static void test_sustain_filter_requires_one_second_sustained_presence(void)
{
	obstacle_filter_t filter = {
		.engage_hold_ms = 1000,
		.disengage_count = 6,
		.assert_since_ms = 0,
		.clear_count = 0,
		.assert_pending = false,
		.latched = false,
	};

	assert(!obstacle_filter_update(&filter, true, 0));
	assert(!obstacle_filter_update(&filter, true, 950));
	assert(obstacle_filter_update(&filter, true, 1000));
}

static void test_sustain_filter_resets_pending_assert_on_bounce(void)
{
	obstacle_filter_t filter = {
		.engage_hold_ms = 1000,
		.disengage_count = 6,
		.assert_since_ms = 0,
		.clear_count = 0,
		.assert_pending = false,
		.latched = false,
	};

	assert(!obstacle_filter_update(&filter, true, 0));
	assert(!obstacle_filter_update(&filter, false, 500));
	assert(!obstacle_filter_update(&filter, true, 600));
	assert(!obstacle_filter_update(&filter, true, 1500));
	assert(obstacle_filter_update(&filter, true, 1600));
}

static void test_sustain_filter_requires_consecutive_clear_samples_to_release(void)
{
	obstacle_filter_t filter = {
		.engage_hold_ms = 1000,
		.disengage_count = 6,
		.assert_since_ms = 0,
		.clear_count = 0,
		.assert_pending = false,
		.latched = true,
	};

	for (uint32_t i = 0; i < 5; ++i)
	{
		assert(obstacle_filter_update(&filter, false, i * 50));
	}
	assert(!obstacle_filter_update(&filter, false, 250));
}

static void test_zero_engage_hold_ms_latches_immediately(void)
{
	obstacle_filter_t filter = {
		.engage_hold_ms = 0,
		.disengage_count = 6,
		.assert_since_ms = 0,
		.clear_count = 0,
		.assert_pending = false,
		.latched = false,
	};

	// With zero hold time, first asserted sample starts pending, second latches
	assert(!obstacle_filter_update(&filter, true, 0));
	assert(obstacle_filter_update(&filter, true, 1));
}

static void test_engaged_to_disengaged_with_disengage_count(void)
{
	obstacle_filter_t filter = {
		.engage_hold_ms = 0,
		.disengage_count = 3,
		.assert_since_ms = 0,
		.clear_count = 0,
		.assert_pending = false,
		.latched = true,
	};

	// Should remain latched for fewer than disengage_count clear samples
	assert(obstacle_filter_update(&filter, false, 100));
	assert(obstacle_filter_update(&filter, false, 200));
	// Third clear sample meets the count and unlatches
	assert(!obstacle_filter_update(&filter, false, 300));
}

static void test_engaged_to_disengaged_interrupted_by_assert(void)
{
	obstacle_filter_t filter = {
		.engage_hold_ms = 0,
		.disengage_count = 3,
		.assert_since_ms = 0,
		.clear_count = 0,
		.assert_pending = false,
		.latched = true,
	};

	// Two clear samples, then an asserted sample resets the clear count
	assert(obstacle_filter_update(&filter, false, 100));
	assert(obstacle_filter_update(&filter, false, 200));
	assert(obstacle_filter_update(&filter, true, 300));
	// Need three more consecutive clears after the interruption
	assert(obstacle_filter_update(&filter, false, 400));
	assert(obstacle_filter_update(&filter, false, 500));
	assert(!obstacle_filter_update(&filter, false, 600));
}

static void test_exact_engage_hold_ms_boundary(void)
{
	obstacle_filter_t filter = {
		.engage_hold_ms = 500,
		.disengage_count = 6,
		.assert_since_ms = 0,
		.clear_count = 0,
		.assert_pending = false,
		.latched = false,
	};

	// Start the asserted streak
	assert(!obstacle_filter_update(&filter, true, 0));
	// Just under the threshold
	assert(!obstacle_filter_update(&filter, true, 499));
	// Exactly at the threshold
	assert(obstacle_filter_update(&filter, true, 500));
}

static void test_null_filter_returns_raw(void)
{
	assert(obstacle_filter_update(nullptr, true, 100) == true);
	assert(obstacle_filter_update(nullptr, false, 200) == false);
}

int main(void)
{
	printf("\n=== obstacle_filter unit tests ===\n\n");

	TEST(test_sustain_filter_requires_one_second_sustained_presence);
	TEST(test_sustain_filter_resets_pending_assert_on_bounce);
	TEST(test_sustain_filter_requires_consecutive_clear_samples_to_release);
	TEST(test_zero_engage_hold_ms_latches_immediately);
	TEST(test_engaged_to_disengaged_with_disengage_count);
	TEST(test_engaged_to_disengaged_interrupted_by_assert);
	TEST(test_exact_engage_hold_ms_boundary);
	TEST(test_null_filter_returns_raw);

	TEST_REPORT();
	TEST_EXIT();
}
