/**
 * @file test_override_sensors.cpp
 * @brief Unit tests for override_sensors component.
 */

#include <assert.h>
#include <stdio.h>

#include "esp_idf_mock.h"
#include "override_sensors.hh"

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { \
    tests_run++; \
    printf("  [TEST] %-55s ", #name); \
    name(); \
    tests_passed++; \
    printf("PASS\n"); \
} while (0)

static override_sensors_config_t default_cfg(void) {
    override_sensors_config_t cfg = {
        .adc_unit = ADC_UNIT_1,
        .pedal_adc_channel = ADC_CHANNEL_0,
        .fr_dir_gpio = 14,
        .fr_rev_gpio = 15,
    };
    return cfg;
}

static void set_fr_forward(void) {
    mock_gpio_levels[14] = 0;  // dir active
    mock_gpio_levels[15] = 1;  // rev inactive
}

static void set_fr_reverse(void) {
    mock_gpio_levels[14] = 0;  // dir active
    mock_gpio_levels[15] = 0;  // rev active
}

static void test_init_latches_initial_pedal_above_threshold(void) {
    mock_reset_all();
    mock_adc_cali_create_result = ESP_FAIL;  // use raw conversion path
    set_fr_forward();

    // Initial read above threshold (~805 mV)
    mock_adc_read_raw_value = 1000;

    override_sensors_config_t cfg = default_cfg();
    assert(override_sensors_init(&cfg) == ESP_OK);

    // Drop below threshold after init
    mock_adc_read_raw_value = 0;

    // Should still be NOT re-armed until update() clears the latch.
    assert(override_sensors_pedal_rearmed() == false);
}

static void test_pedal_rearm_after_release_window(void) {
    mock_reset_all();
    mock_adc_cali_create_result = ESP_FAIL;
    set_fr_forward();

    // Start with pedal pressed (> threshold)
    mock_adc_read_raw_value = 1000;

    override_sensors_config_t cfg = default_cfg();
    assert(override_sensors_init(&cfg) == ESP_OK);

    // Release pedal and advance time.
    mock_adc_read_raw_value = 0;
    override_sensors_update(100);
    assert(override_sensors_pedal_rearmed() == false);

    override_sensors_update(700);  // >= 500ms below threshold
    assert(override_sensors_pedal_rearmed() == true);
}

static void test_fr_debounce_requires_stable_interval(void) {
    mock_reset_all();
    mock_adc_cali_create_result = ESP_FAIL;
    set_fr_forward();
    mock_adc_read_raw_value = 0;

    override_sensors_config_t cfg = default_cfg();
    assert(override_sensors_init(&cfg) == ESP_OK);
    assert(override_sensors_get_fr_state() == FR_STATE_FORWARD);

    // Raw switch changes to reverse, but debounced state should not change yet.
    set_fr_reverse();
    override_sensors_update(100);
    assert(override_sensors_get_fr_state() == FR_STATE_FORWARD);

    override_sensors_update(119);
    assert(override_sensors_get_fr_state() == FR_STATE_FORWARD);

    // At debounce boundary, change should commit.
    override_sensors_update(120);
    assert(override_sensors_get_fr_state() == FR_STATE_REVERSE);
}

static void test_init_fails_when_adc_setup_fails(void) {
    mock_reset_all();
    set_fr_forward();
    mock_adc_new_unit_result = ESP_FAIL;

    override_sensors_config_t cfg = default_cfg();
    assert(override_sensors_init(&cfg) == ESP_FAIL);
}

int main(void) {
    printf("\n=== override_sensors unit tests ===\n\n");

    TEST(test_init_latches_initial_pedal_above_threshold);
    TEST(test_pedal_rearm_after_release_window);
    TEST(test_fr_debounce_requires_stable_interval);
    TEST(test_init_fails_when_adc_setup_fails);

    printf("\n=== %d / %d tests passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
