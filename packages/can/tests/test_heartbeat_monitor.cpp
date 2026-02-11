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

    assert(strcmp(mon.tag, "SAFETY_HB") == 0);
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

int main(void) {
    printf("\n=== heartbeat_monitor unit tests ===\n\n");

    TEST(test_init_uses_config_name_for_tag);
    TEST(test_init_allows_null_config);
    TEST(test_register_null_name_uses_unknown);
    TEST(test_register_copies_node_name);
    TEST(test_timeout_transition_and_mask);
    TEST(test_register_fails_when_full);

    printf("\n=== %d / %d tests passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
