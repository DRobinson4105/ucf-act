/**
 * @file test_heartbeat_monitor.cpp
 * @brief Unit tests for heartbeat_monitor component.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "esp_idf_mock.h"
#include "heartbeat_monitor.hh"

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { \
    tests_run++; \
    printf("  [TEST] %-55s ", #name); \
    name(); \
    tests_passed++; \
    printf("PASS\n"); \
} while (0)

static void test_init_uses_config_name_for_tag(void) {
    mock_reset_all();

    heartbeat_monitor_t mon = {};
    heartbeat_monitor_config_t cfg = {.name = "SAFETY"};
    heartbeat_monitor_init(&mon, &cfg);

    assert(strcmp(mon.tag, "SAFETY") == 0);
    assert(mon.node_count == 0);
}

static void test_init_allows_null_config(void) {
    mock_reset_all();

    heartbeat_monitor_t mon = {};
    heartbeat_monitor_init(&mon, nullptr);

    assert(strcmp(mon.tag, "HEARTBEAT") == 0);
    assert(mon.node_count == 0);
}

static void test_register_null_name_uses_unknown(void) {
    mock_reset_all();

    heartbeat_monitor_t mon = {};
    heartbeat_monitor_config_t cfg = {.name = "SAFETY"};
    heartbeat_monitor_init(&mon, &cfg);

    int node_id = heartbeat_monitor_register(&mon, nullptr, 500);
    assert(node_id == 0);

    heartbeat_monitor_node_t st = {};
    assert(heartbeat_monitor_get_status(&mon, node_id, &st));
    assert(strcmp(st.name, "unknown") == 0);
}

static void test_register_copies_node_name(void) {
    mock_reset_all();

    heartbeat_monitor_t mon = {};
    heartbeat_monitor_config_t cfg = {.name = "SAFETY"};
    heartbeat_monitor_init(&mon, &cfg);

    char temp_name[16] = "Planner";
    int node_id = heartbeat_monitor_register(&mon, temp_name, 500);
    assert(node_id == 0);

    strncpy(temp_name, "Changed", sizeof(temp_name) - 1);
    temp_name[sizeof(temp_name) - 1] = '\0';

    heartbeat_monitor_node_t st = {};
    assert(heartbeat_monitor_get_status(&mon, node_id, &st));
    assert(strcmp(st.name, "Planner") == 0);
}

static void test_timeout_transition_and_mask(void) {
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
    assert((heartbeat_monitor_get_timeout_mask(&mon) & 0x01) != 0);

    mock_tick_count = 210;
    heartbeat_monitor_update(&mon, node_id, 2, 3);
    assert(heartbeat_monitor_is_alive(&mon, node_id));
    assert((heartbeat_monitor_get_timeout_mask(&mon) & 0x01) == 0);
}

static void test_register_fails_when_full(void) {
    mock_reset_all();

    heartbeat_monitor_t mon = {};
    heartbeat_monitor_config_t cfg = {.name = "SAFETY"};
    heartbeat_monitor_init(&mon, &cfg);

    for (int i = 0; i < HEARTBEAT_MONITOR_MAX_NODES; i++) {
        int node_id = heartbeat_monitor_register(&mon, "Node", 100);
        assert(node_id == i);
    }

    assert(heartbeat_monitor_register(&mon, "Overflow", 100) == -1);
}

// ============================================================================
// all_alive tests — critical safety-chain function, previously untested
// ============================================================================

static void test_all_alive_true_when_all_updated(void) {
    mock_reset_all();

    heartbeat_monitor_t mon = {};
    heartbeat_monitor_config_t cfg = {.name = "TEST"};
    heartbeat_monitor_init(&mon, &cfg);

    int n0 = heartbeat_monitor_register(&mon, "NodeA", 500);
    int n1 = heartbeat_monitor_register(&mon, "NodeB", 500);
    assert(n0 == 0);
    assert(n1 == 1);

    // Before any updates, nodes start not-alive
    assert(heartbeat_monitor_all_alive(&mon) == false);

    // Update both
    mock_tick_count = 10;
    heartbeat_monitor_update(&mon, n0, 1, 0);
    heartbeat_monitor_update(&mon, n1, 1, 0);

    assert(heartbeat_monitor_all_alive(&mon) == true);
}

static void test_all_alive_false_when_one_timed_out(void) {
    mock_reset_all();

    heartbeat_monitor_t mon = {};
    heartbeat_monitor_config_t cfg = {.name = "TEST"};
    heartbeat_monitor_init(&mon, &cfg);

    int n0 = heartbeat_monitor_register(&mon, "Fast", 100);
    int n1 = heartbeat_monitor_register(&mon, "Slow", 1000);

    mock_tick_count = 10;
    heartbeat_monitor_update(&mon, n0, 1, 0);
    heartbeat_monitor_update(&mon, n1, 1, 0);
    assert(heartbeat_monitor_all_alive(&mon) == true);

    // Advance past Fast's timeout but not Slow's
    mock_tick_count = 200;
    heartbeat_monitor_check_timeouts(&mon);
    assert(heartbeat_monitor_all_alive(&mon) == false);
    assert(heartbeat_monitor_is_alive(&mon, n0) == false);
    assert(heartbeat_monitor_is_alive(&mon, n1) == true);
}

static void test_all_alive_false_when_never_seen(void) {
    mock_reset_all();

    heartbeat_monitor_t mon = {};
    heartbeat_monitor_config_t cfg = {.name = "TEST"};
    heartbeat_monitor_init(&mon, &cfg);

    heartbeat_monitor_register(&mon, "Ghost", 500);
    // Never updated — alive starts false
    assert(heartbeat_monitor_all_alive(&mon) == false);
}

static void test_all_alive_true_with_zero_nodes(void) {
    mock_reset_all();

    heartbeat_monitor_t mon = {};
    heartbeat_monitor_config_t cfg = {.name = "EMPTY"};
    heartbeat_monitor_init(&mon, &cfg);

    // Vacuous truth: no registered nodes
    assert(heartbeat_monitor_all_alive(&mon) == true);
}

static void test_all_alive_recovers_after_update(void) {
    mock_reset_all();

    heartbeat_monitor_t mon = {};
    heartbeat_monitor_config_t cfg = {.name = "TEST"};
    heartbeat_monitor_init(&mon, &cfg);

    int n0 = heartbeat_monitor_register(&mon, "Node", 100);

    mock_tick_count = 10;
    heartbeat_monitor_update(&mon, n0, 1, 0);
    assert(heartbeat_monitor_all_alive(&mon) == true);

    // Time out
    mock_tick_count = 200;
    heartbeat_monitor_check_timeouts(&mon);
    assert(heartbeat_monitor_all_alive(&mon) == false);

    // Recover with new heartbeat
    mock_tick_count = 210;
    heartbeat_monitor_update(&mon, n0, 2, 0);
    assert(heartbeat_monitor_all_alive(&mon) == true);
}

int main(void) {
    printf("\n=== heartbeat_monitor unit tests ===\n\n");

    TEST(test_init_uses_config_name_for_tag);
    TEST(test_init_allows_null_config);
    TEST(test_register_null_name_uses_unknown);
    TEST(test_register_copies_node_name);
    TEST(test_timeout_transition_and_mask);
    TEST(test_register_fails_when_full);

    printf("\n  --- all_alive ---\n");
    TEST(test_all_alive_true_when_all_updated);
    TEST(test_all_alive_false_when_one_timed_out);
    TEST(test_all_alive_false_when_never_seen);
    TEST(test_all_alive_true_with_zero_nodes);
    TEST(test_all_alive_recovers_after_update);

    printf("\n=== %d / %d tests passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
