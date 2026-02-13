/**
 * @file test_driver_inputs.cpp
 * @brief Unit tests for adc_12bitsar and optocoupler_pc817 components.
 */

#include <assert.h>
#include <stdio.h>

#include "esp_idf_mock.h"
#include "adc_12bitsar.hh"
#include "optocoupler_pc817.hh"

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { \
    tests_run++; \
    printf("  [TEST] %-55s ", #name); \
    name(); \
    tests_passed++; \
    printf("PASS\n"); \
} while (0)

static adc_12bitsar_config_t default_pedal_cfg(void) {
    adc_12bitsar_config_t cfg = {
        .adc_unit = ADC_UNIT_1,
        .adc_channel = ADC_CHANNEL_0,
    };
    return cfg;
}

static optocoupler_pc817_config_t default_fr_cfg(void) {
    optocoupler_pc817_config_t cfg = {
        .forward_gpio = 14,
        .reverse_gpio = 15,
    };
    return cfg;
}

static void set_fr_forward(void) {
    mock_gpio_levels[14] = 0;  // forward active
    mock_gpio_levels[15] = 1;  // reverse inactive
}

static void set_fr_reverse(void) {
    mock_gpio_levels[14] = 1;  // forward inactive
    mock_gpio_levels[15] = 0;  // reverse active
}

static void set_fr_neutral(void) {
    mock_gpio_levels[14] = 1;  // forward inactive
    mock_gpio_levels[15] = 1;  // reverse inactive
}

static void set_fr_invalid(void) {
    mock_gpio_levels[14] = 0;  // forward active
    mock_gpio_levels[15] = 0;  // reverse active
}

static void test_adc_12bitsar_raw_conversion_without_calibration(void) {
    mock_reset_all();
    mock_adc_cali_create_result = ESP_FAIL;
    mock_adc_read_raw_value = 1000;

    adc_12bitsar_config_t cfg = default_pedal_cfg();
    assert(adc_12bitsar_init(&cfg) == ESP_OK);
    assert(adc_12bitsar_is_calibrated() == false);
    assert(adc_12bitsar_read_mv() == (uint16_t)((1000 * 3300) / 4095));

    adc_12bitsar_deinit();
}

static void test_adc_12bitsar_uses_calibration_when_available(void) {
    mock_reset_all();
    mock_adc_cali_create_result = ESP_OK;
    mock_adc_cali_voltage_mv = 777;
    mock_adc_read_raw_value = 1234;

    adc_12bitsar_config_t cfg = default_pedal_cfg();
    assert(adc_12bitsar_init(&cfg) == ESP_OK);
    assert(adc_12bitsar_is_calibrated() == true);
    assert(adc_12bitsar_read_mv() == 777);

    adc_12bitsar_deinit();
}

static void test_adc_12bitsar_init_fails_when_adc_setup_fails(void) {
    mock_reset_all();
    mock_adc_new_unit_result = ESP_FAIL;

    adc_12bitsar_config_t cfg = default_pedal_cfg();
    assert(adc_12bitsar_init(&cfg) == ESP_FAIL);
}

static void test_fr_raw_mapping_matches_pc817_wiring(void) {
    mock_reset_all();
    set_fr_forward();

    optocoupler_pc817_config_t cfg = default_fr_cfg();
    assert(optocoupler_pc817_init(&cfg) == ESP_OK);

    set_fr_forward();
    assert(optocoupler_pc817_get_state_raw() == FR_STATE_FORWARD);

    set_fr_reverse();
    assert(optocoupler_pc817_get_state_raw() == FR_STATE_REVERSE);

    set_fr_neutral();
    assert(optocoupler_pc817_get_state_raw() == FR_STATE_NEUTRAL);

    set_fr_invalid();
    assert(optocoupler_pc817_get_state_raw() == FR_STATE_INVALID);
}

static void test_fr_debounce_requires_stable_interval(void) {
    mock_reset_all();
    set_fr_forward();

    optocoupler_pc817_config_t cfg = default_fr_cfg();
    assert(optocoupler_pc817_init(&cfg) == ESP_OK);
    assert(optocoupler_pc817_get_state() == FR_STATE_FORWARD);

    set_fr_reverse();
    optocoupler_pc817_update(100);
    assert(optocoupler_pc817_get_state() == FR_STATE_FORWARD);

    optocoupler_pc817_update(119);
    assert(optocoupler_pc817_get_state() == FR_STATE_FORWARD);

    optocoupler_pc817_update(120);
    assert(optocoupler_pc817_get_state() == FR_STATE_REVERSE);
}

static void test_fr_init_fails_when_gpio_setup_fails(void) {
    mock_reset_all();
    mock_gpio_config_result = ESP_FAIL;

    optocoupler_pc817_config_t cfg = default_fr_cfg();
    assert(optocoupler_pc817_init(&cfg) == ESP_FAIL);
}

// ============================================================================
// ADC boundary tests
// ============================================================================

static void test_adc_raw_zero_returns_zero_mv(void) {
    mock_reset_all();
    mock_adc_cali_create_result = ESP_FAIL;  // use fallback formula
    mock_adc_read_raw_value = 0;

    adc_12bitsar_config_t cfg = default_pedal_cfg();
    assert(adc_12bitsar_init(&cfg) == ESP_OK);
    assert(adc_12bitsar_read_mv() == 0);
    adc_12bitsar_deinit();
}

static void test_adc_raw_4095_returns_3300_mv(void) {
    mock_reset_all();
    mock_adc_cali_create_result = ESP_FAIL;  // use fallback formula
    mock_adc_read_raw_value = 4095;

    adc_12bitsar_config_t cfg = default_pedal_cfg();
    assert(adc_12bitsar_init(&cfg) == ESP_OK);
    assert(adc_12bitsar_read_mv() == 3300);
    adc_12bitsar_deinit();
}

// ============================================================================
// FR debounce edge-case tests
// ============================================================================

static void test_fr_debounce_resets_on_bounce_back(void) {
    mock_reset_all();
    set_fr_forward();

    optocoupler_pc817_config_t cfg = default_fr_cfg();
    assert(optocoupler_pc817_init(&cfg) == ESP_OK);
    assert(optocoupler_pc817_get_state() == FR_STATE_FORWARD);

    // Signal changes to REVERSE at t=100 — starts debounce
    set_fr_reverse();
    optocoupler_pc817_update(100);
    assert(optocoupler_pc817_get_state() == FR_STATE_FORWARD);

    // Signal bounces BACK to FORWARD at t=110 (mid-debounce) — cancels debounce
    set_fr_forward();
    optocoupler_pc817_update(110);
    assert(optocoupler_pc817_get_state() == FR_STATE_FORWARD);

    // Even well past the original debounce window, state stays FORWARD
    optocoupler_pc817_update(130);
    assert(optocoupler_pc817_get_state() == FR_STATE_FORWARD);
}

static void test_fr_debounce_resets_on_different_state(void) {
    mock_reset_all();
    set_fr_forward();

    optocoupler_pc817_config_t cfg = default_fr_cfg();
    assert(optocoupler_pc817_init(&cfg) == ESP_OK);
    assert(optocoupler_pc817_get_state() == FR_STATE_FORWARD);

    // Signal changes to REVERSE at t=100 — starts debounce toward REVERSE
    set_fr_reverse();
    optocoupler_pc817_update(100);
    assert(optocoupler_pc817_get_state() == FR_STATE_FORWARD);

    // Signal changes to NEUTRAL at t=110 — pending resets to NEUTRAL, timer restarts
    set_fr_neutral();
    optocoupler_pc817_update(110);
    assert(optocoupler_pc817_get_state() == FR_STATE_FORWARD);

    // At t=129 (19ms from NEUTRAL start) — still debouncing
    optocoupler_pc817_update(129);
    assert(optocoupler_pc817_get_state() == FR_STATE_FORWARD);

    // At t=130 (20ms from NEUTRAL start) — debounce completes to NEUTRAL
    optocoupler_pc817_update(130);
    assert(optocoupler_pc817_get_state() == FR_STATE_NEUTRAL);
}

int main(void) {
    printf("\n=== driver_inputs unit tests ===\n\n");

    TEST(test_adc_12bitsar_raw_conversion_without_calibration);
    TEST(test_adc_12bitsar_uses_calibration_when_available);
    TEST(test_adc_12bitsar_init_fails_when_adc_setup_fails);
    TEST(test_adc_raw_zero_returns_zero_mv);
    TEST(test_adc_raw_4095_returns_3300_mv);
    TEST(test_fr_raw_mapping_matches_pc817_wiring);
    TEST(test_fr_debounce_requires_stable_interval);
    TEST(test_fr_debounce_resets_on_bounce_back);
    TEST(test_fr_debounce_resets_on_different_state);
    TEST(test_fr_init_fails_when_gpio_setup_fails);

    printf("\n=== %d / %d tests passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
