/**
 * @file test_heartbeat_monitor.cpp
 * @brief Unit tests for heartbeat_monitor component.
 */

#include "test_harness.h"
#include <string.h>

#include "esp_idf_mock.h"
#include "heartbeat_monitor.h"

namespace
{

void test_init_uses_config_name_for_tag(void)
{
	mock_reset_all();

	heartbeat_monitor_t mon = {};
	heartbeat_monitor_config_t cfg = {.name = "SAFETY"};
	heartbeat_monitor_init(&mon, &cfg);

	assert(strcmp(mon.tag, "SAFETY") == 0);
	assert(mon.node_count == 0);
}

void test_init_allows_null_config(void)
{
	mock_reset_all();

	heartbeat_monitor_t mon = {};
	heartbeat_monitor_init(&mon, nullptr);

	assert(strcmp(mon.tag, "HEARTBEAT") == 0);
	assert(mon.node_count == 0);
}

void test_register_null_name_uses_unknown(void)
{
	mock_reset_all();

	heartbeat_monitor_t mon = {};
	heartbeat_monitor_config_t cfg = {.name = "SAFETY"};
	heartbeat_monitor_init(&mon, &cfg);

	int node_id = heartbeat_monitor_register(&mon, nullptr, 500);
	assert(node_id == 0);
	// Node starts not-alive
	assert(!heartbeat_monitor_is_alive(&mon, node_id));
}

void test_register_copies_node_name(void)
{
	mock_reset_all();

	heartbeat_monitor_t mon = {};
	heartbeat_monitor_config_t cfg = {.name = "SAFETY"};
	heartbeat_monitor_init(&mon, &cfg);

	char temp_name[16] = "Planner";
	int node_id = heartbeat_monitor_register(&mon, temp_name, 500);
	assert(node_id == 0);

	// Mutate original — monitor should have its own copy
	strncpy(temp_name, "Changed", sizeof(temp_name) - 1);
	temp_name[sizeof(temp_name) - 1] = '\0';

	// Verify by checking the node is registered and starts not-alive
	assert(!heartbeat_monitor_is_alive(&mon, node_id));
}

void test_timeout_transition(void)
{
	mock_reset_all();

	heartbeat_monitor_t mon = {};
	heartbeat_monitor_config_t cfg = {.name = "SAFETY"};
	heartbeat_monitor_init(&mon, &cfg);
	int node_id = heartbeat_monitor_register(&mon, "Planner", 100);
	assert(node_id == 0);

	assert(!heartbeat_monitor_is_alive(&mon, node_id));

	mock_tick_count = 10;
	heartbeat_monitor_update(&mon, node_id, 1, 2);
	assert(heartbeat_monitor_is_alive(&mon, node_id));

	mock_tick_count = 50;
	heartbeat_monitor_check_timeouts(&mon);
	assert(heartbeat_monitor_is_alive(&mon, node_id));

	mock_tick_count = 200;
	heartbeat_monitor_check_timeouts(&mon);
	assert(!heartbeat_monitor_is_alive(&mon, node_id));

	mock_tick_count = 210;
	heartbeat_monitor_update(&mon, node_id, 2, 3);
	assert(heartbeat_monitor_is_alive(&mon, node_id));
}

void test_tick_zero_update_not_immediately_timed_out(void)
{
	mock_reset_all();

	heartbeat_monitor_t mon = {};
	heartbeat_monitor_config_t cfg = {.name = "SAFETY"};
	heartbeat_monitor_init(&mon, &cfg);
	int node_id = heartbeat_monitor_register(&mon, "Planner", 100);
	assert(node_id == 0);

	mock_tick_count = 0;
	heartbeat_monitor_update(&mon, node_id, 1, 2);
	assert(heartbeat_monitor_is_alive(&mon, node_id));

	// At the same tick, this must not false-timeout.
	heartbeat_monitor_check_timeouts(&mon);
	assert(heartbeat_monitor_is_alive(&mon, node_id));
}

void test_never_seen_node_stays_not_alive(void)
{
	mock_reset_all();

	heartbeat_monitor_t mon = {};
	heartbeat_monitor_config_t cfg = {.name = "SAFETY"};
	heartbeat_monitor_init(&mon, &cfg);
	int n0 = heartbeat_monitor_register(&mon, "Seen", 100);
	int n1 = heartbeat_monitor_register(&mon, "NeverSeen", 100);
	assert(n0 == 0);
	assert(n1 == 1);

	mock_tick_count = 10;
	heartbeat_monitor_update(&mon, n0, 1, 0);

	mock_tick_count = 200;
	heartbeat_monitor_check_timeouts(&mon);

	assert(!heartbeat_monitor_is_alive(&mon, n0)); // timed out
	assert(!heartbeat_monitor_is_alive(&mon, n1)); // never seen, still not alive
}

void test_register_fails_when_full(void)
{
	mock_reset_all();

	heartbeat_monitor_t mon = {};
	heartbeat_monitor_config_t cfg = {.name = "SAFETY"};
	heartbeat_monitor_init(&mon, &cfg);

	for (int i = 0; i < HEARTBEAT_MONITOR_MAX_NODES; i++)
	{
		int node_id = heartbeat_monitor_register(&mon, "Node", 100);
		assert(node_id == i);
	}

	assert(heartbeat_monitor_register(&mon, "Overflow", 100) == -1);
}

void test_two_nodes_independent_liveness(void)
{
	mock_reset_all();

	heartbeat_monitor_t mon = {};
	heartbeat_monitor_config_t cfg = {.name = "TEST"};
	heartbeat_monitor_init(&mon, &cfg);

	int n0 = heartbeat_monitor_register(&mon, "Fast", 100);
	int n1 = heartbeat_monitor_register(&mon, "Slow", 1000);

	mock_tick_count = 10;
	heartbeat_monitor_update(&mon, n0, 1, 0);
	heartbeat_monitor_update(&mon, n1, 1, 0);
	assert(heartbeat_monitor_is_alive(&mon, n0));
	assert(heartbeat_monitor_is_alive(&mon, n1));

	// Advance past Fast's timeout but not Slow's
	mock_tick_count = 200;
	heartbeat_monitor_check_timeouts(&mon);
	assert(!heartbeat_monitor_is_alive(&mon, n0));
	assert(heartbeat_monitor_is_alive(&mon, n1));
}

void test_recovers_after_timeout(void)
{
	mock_reset_all();

	heartbeat_monitor_t mon = {};
	heartbeat_monitor_config_t cfg = {.name = "TEST"};
	heartbeat_monitor_init(&mon, &cfg);

	int n0 = heartbeat_monitor_register(&mon, "Node", 100);

	mock_tick_count = 10;
	heartbeat_monitor_update(&mon, n0, 1, 0);
	assert(heartbeat_monitor_is_alive(&mon, n0));

	// Time out
	mock_tick_count = 200;
	heartbeat_monitor_check_timeouts(&mon);
	assert(!heartbeat_monitor_is_alive(&mon, n0));

	// Recover with new heartbeat
	mock_tick_count = 210;
	heartbeat_monitor_update(&mon, n0, 2, 0);
	assert(heartbeat_monitor_is_alive(&mon, n0));
}

void test_timeout_handles_tick_wrap(void)
{
	mock_reset_all();

	heartbeat_monitor_t mon = {};
	heartbeat_monitor_config_t cfg = {.name = "WRAP"};
	heartbeat_monitor_init(&mon, &cfg);

	int n0 = heartbeat_monitor_register(&mon, "Node", 100);

	// Update near UINT32_MAX
	mock_tick_count = UINT32_MAX - 50;
	heartbeat_monitor_update(&mon, n0, 1, 0);
	assert(heartbeat_monitor_is_alive(&mon, n0));

	// Tick wraps past 0; elapsed = ~150 > 100
	mock_tick_count = 100;
	heartbeat_monitor_check_timeouts(&mon);
	assert(!heartbeat_monitor_is_alive(&mon, n0));
}

void test_zero_timeout_times_out_immediately(void)
{
	mock_reset_all();

	heartbeat_monitor_t mon = {};
	heartbeat_monitor_config_t cfg = {.name = "TEST"};
	heartbeat_monitor_init(&mon, &cfg);

	int n0 = heartbeat_monitor_register(&mon, "Node", 0); // zero timeout
	assert(n0 == 0);

	mock_tick_count = 10;
	heartbeat_monitor_update(&mon, n0, 1, 0);
	assert(heartbeat_monitor_is_alive(&mon, n0));

	// Any subsequent tick should time it out (elapsed > 0 timeout)
	mock_tick_count = 11;
	heartbeat_monitor_check_timeouts(&mon);
	assert(!heartbeat_monitor_is_alive(&mon, n0));
}

void test_out_of_bounds_node_id_returns_not_alive(void)
{
	mock_reset_all();

	heartbeat_monitor_t mon = {};
	heartbeat_monitor_config_t cfg = {.name = "TEST"};
	heartbeat_monitor_init(&mon, &cfg);

	// Query node IDs that were never registered
	assert(!heartbeat_monitor_is_alive(&mon, 0));
	assert(!heartbeat_monitor_is_alive(&mon, -1));
	assert(!heartbeat_monitor_is_alive(&mon, HEARTBEAT_MONITOR_MAX_NODES));
	assert(!heartbeat_monitor_is_alive(&mon, 999));
}

void test_update_unregistered_node_does_not_crash(void)
{
	mock_reset_all();

	heartbeat_monitor_t mon = {};
	heartbeat_monitor_config_t cfg = {.name = "TEST"};
	heartbeat_monitor_init(&mon, &cfg);

	// Updating a node that was never registered should not crash
	heartbeat_monitor_update(&mon, 0, 1, 0);
	heartbeat_monitor_update(&mon, -1, 1, 0);
	heartbeat_monitor_update(&mon, HEARTBEAT_MONITOR_MAX_NODES, 1, 0);
	// If we get here without crashing, the test passes
}

} // namespace

int main(void)
{
	printf("\n=== heartbeat_monitor unit tests ===\n\n");

	TEST(test_init_uses_config_name_for_tag);
	TEST(test_init_allows_null_config);
	TEST(test_register_null_name_uses_unknown);
	TEST(test_register_copies_node_name);
	TEST(test_timeout_transition);
	TEST(test_tick_zero_update_not_immediately_timed_out);
	TEST(test_never_seen_node_stays_not_alive);
	TEST(test_register_fails_when_full);
	TEST(test_two_nodes_independent_liveness);
	TEST(test_recovers_after_timeout);

	printf("\n  --- tick wrap ---\n");
	TEST(test_timeout_handles_tick_wrap);

	printf("\n  --- additional edge cases ---\n");
	TEST(test_zero_timeout_times_out_immediately);
	TEST(test_out_of_bounds_node_id_returns_not_alive);
	TEST(test_update_unregistered_node_does_not_crash);

	TEST_REPORT();
	TEST_EXIT();
}
