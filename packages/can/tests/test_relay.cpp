/**
 * @file test_relay.cpp
 * @brief Unit tests for MY5NJ DPDT relay driver component.
 */

#include "test_harness.h"

#include "esp_idf_mock.h"
#include "relay_dpdt_my5nj.h"

namespace
{

relay_dpdt_my5nj_config_t default_cfg(void)
{
	relay_dpdt_my5nj_config_t cfg = {
		.gpio = 10,
	};
	return cfg;
}

// Helper: perform a clean successful init.
void clean_init(void)
{
	mock_reset_all();
	relay_dpdt_my5nj_config_t cfg = default_cfg();
	esp_err_t err = relay_dpdt_my5nj_init(&cfg);
	assert(err == ESP_OK);
}

// ============================================================================
// Before-init tests (module starts uninitialized)
// ============================================================================

void test_energize_before_init_fails(void)
{
	assert(relay_dpdt_my5nj_energize() == ESP_ERR_INVALID_STATE);
}

void test_deenergize_before_init_fails(void)
{
	assert(relay_dpdt_my5nj_deenergize() == ESP_ERR_INVALID_STATE);
}

void test_is_energized_false_before_init(void)
{
	assert(relay_dpdt_my5nj_is_energized() == false);
}

// ============================================================================
// Init tests
// ============================================================================

void test_init_null_config_fails(void)
{
	assert(relay_dpdt_my5nj_init(NULL) == ESP_ERR_INVALID_ARG);
}

void test_init_success(void)
{
	clean_init();
	// Should start de-energized (GPIO LOW)
	assert(mock_gpio_levels[10] == 0);
	assert(relay_dpdt_my5nj_is_energized() == false);
}

void test_init_gpio_config_fails(void)
{
	mock_reset_all();
	mock_gpio_config_result = ESP_FAIL;
	relay_dpdt_my5nj_config_t cfg = default_cfg();
	assert(relay_dpdt_my5nj_init(&cfg) == ESP_FAIL);
}

void test_init_preloads_safe_level(void)
{
	mock_reset_all();
	// Set GPIO 10 high before init to verify init drives it LOW
	mock_gpio_levels[10] = 1;
	relay_dpdt_my5nj_config_t cfg = default_cfg();
	assert(relay_dpdt_my5nj_init(&cfg) == ESP_OK);
	assert(mock_gpio_levels[10] == 0);
}

void test_init_set_level_failure(void)
{
	mock_reset_all();
	mock_gpio_set_level_result = ESP_FAIL;
	relay_dpdt_my5nj_config_t cfg = default_cfg();
	assert(relay_dpdt_my5nj_init(&cfg) == ESP_FAIL);
}

// ============================================================================
// Energize / deenergize tests
// ============================================================================

void test_energize_success(void)
{
	clean_init();
	assert(relay_dpdt_my5nj_energize() == ESP_OK);
	assert(mock_gpio_levels[10] == 1);
	assert(relay_dpdt_my5nj_is_energized() == true);
}

void test_deenergize_success(void)
{
	clean_init();
	relay_dpdt_my5nj_energize();
	assert(relay_dpdt_my5nj_is_energized() == true);

	assert(relay_dpdt_my5nj_deenergize() == ESP_OK);
	assert(mock_gpio_levels[10] == 0);
	assert(relay_dpdt_my5nj_is_energized() == false);
}

void test_energize_deenergize_cycle(void)
{
	clean_init();

	for (int i = 0; i < 5; i++)
	{
		assert(relay_dpdt_my5nj_energize() == ESP_OK);
		assert(relay_dpdt_my5nj_is_energized() == true);
		assert(mock_gpio_levels[10] == 1);

		assert(relay_dpdt_my5nj_deenergize() == ESP_OK);
		assert(relay_dpdt_my5nj_is_energized() == false);
		assert(mock_gpio_levels[10] == 0);
	}
}

void test_energize_set_level_failure(void)
{
	clean_init();
	mock_gpio_set_level_result = ESP_FAIL;
	assert(relay_dpdt_my5nj_energize() != ESP_OK);
}

void test_deenergize_set_level_failure(void)
{
	clean_init();
	relay_dpdt_my5nj_energize();
	mock_gpio_set_level_result = ESP_FAIL;
	assert(relay_dpdt_my5nj_deenergize() != ESP_OK);
}

// ============================================================================
// State query tests
// ============================================================================

void test_is_energized_reflects_gpio(void)
{
	clean_init();
	assert(relay_dpdt_my5nj_is_energized() == false);

	relay_dpdt_my5nj_energize();
	assert(relay_dpdt_my5nj_is_energized() == true);

	relay_dpdt_my5nj_deenergize();
	assert(relay_dpdt_my5nj_is_energized() == false);
}

void test_different_gpio_pin(void)
{
	mock_reset_all();
	relay_dpdt_my5nj_config_t cfg = {.gpio = 15};
	assert(relay_dpdt_my5nj_init(&cfg) == ESP_OK);
	assert(mock_gpio_levels[15] == 0);

	relay_dpdt_my5nj_energize();
	assert(mock_gpio_levels[15] == 1);
	assert(relay_dpdt_my5nj_is_energized() == true);
}

} // namespace

int main(void)
{
	printf("\n=== relay_dpdt_my5nj unit tests ===\n\n");

	printf("  --- before init ---\n");
	TEST(test_energize_before_init_fails);
	TEST(test_deenergize_before_init_fails);
	TEST(test_is_energized_false_before_init);

	printf("\n  --- init ---\n");
	TEST(test_init_null_config_fails);
	TEST(test_init_success);
	TEST(test_init_gpio_config_fails);
	TEST(test_init_preloads_safe_level);
	TEST(test_init_set_level_failure);

	printf("\n  --- energize / deenergize ---\n");
	TEST(test_energize_success);
	TEST(test_deenergize_success);
	TEST(test_energize_deenergize_cycle);
	TEST(test_energize_set_level_failure);
	TEST(test_deenergize_set_level_failure);

	printf("\n  --- state query ---\n");
	TEST(test_is_energized_reflects_gpio);
	TEST(test_different_gpio_pin);

	TEST_REPORT();
	TEST_EXIT();
}
