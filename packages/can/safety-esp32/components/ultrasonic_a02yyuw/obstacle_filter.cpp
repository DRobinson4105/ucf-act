/**
 * @file obstacle_filter.cpp
 * @brief Pure obstacle filtering helpers for sustained-presence checks.
 */

#include "obstacle_filter.h"

bool obstacle_filter_update(obstacle_filter_t *filter, bool raw_asserted, uint32_t now_ms)
{
	if (!filter)
		return raw_asserted;

	if (raw_asserted)
	{
		filter->clear_count = 0;
		if (!filter->latched)
		{
			if (!filter->assert_pending)
			{
				filter->assert_pending = true;
				filter->assert_since_ms = now_ms;
			}
			else if ((uint32_t)(now_ms - filter->assert_since_ms) >= filter->engage_hold_ms)
			{
				filter->latched = true;
			}
		}
	}
	else
	{
		filter->assert_pending = false;
		filter->assert_since_ms = now_ms;
		if (filter->clear_count < filter->disengage_count)
			filter->clear_count++;
		if (filter->clear_count >= filter->disengage_count)
			filter->latched = false;
	}

	return filter->latched;
}
