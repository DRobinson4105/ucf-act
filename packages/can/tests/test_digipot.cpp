/**
 * @file test_digipot.cpp
 * @brief Unit tests for MCP41HV51 digital potentiometer component.
 */

#include "test_harness.h"

#include "esp_idf_mock.h"
#include "digipot_mcp41hv51.h"

namespace
{

digipot_mcp41hv51_config_t default_cfg(void)
{
	digipot_mcp41hv51_config_t cfg = {
		.spi_host = SPI2_HOST,
		.sdi = 2,
		.sck = 3,
		.cs = 6,
	};
	return cfg;
}

// Helper: perform a clean successful init.
void clean_init(void)
{
	mock_reset_all();
	digipot_mcp41hv51_config_t cfg = default_cfg();
	esp_err_t err = digipot_mcp41hv51_init(&cfg);
	assert(err == ESP_OK);
}

// ============================================================================
// Before-init tests (module starts uninitialized at process start)
// ============================================================================

void test_set_wiper_before_init_fails(void)
{
	assert(digipot_mcp41hv51_set_wiper(100) == ESP_ERR_INVALID_STATE);
}

void test_disable_before_init_fails(void)
{
	assert(digipot_mcp41hv51_disable() == ESP_ERR_INVALID_STATE);
}

void test_get_wiper_returns_zero_before_init(void)
{
	assert(digipot_mcp41hv51_get_wiper() == 0);
}

void test_enable_autonomous_before_init_fails(void)
{
	assert(digipot_mcp41hv51_enable_autonomous() == ESP_ERR_INVALID_STATE);
}

void test_emergency_stop_before_init_fails(void)
{
	assert(digipot_mcp41hv51_emergency_stop() == ESP_ERR_INVALID_STATE);
}

void test_is_autonomous_false_before_init(void)
{
	assert(digipot_mcp41hv51_is_autonomous() == false);
}

// ============================================================================
// Init tests
// ============================================================================

void test_init_null_config_fails(void)
{
	assert(digipot_mcp41hv51_init(NULL) == ESP_ERR_INVALID_ARG);
}

void test_init_spi_bus_fails(void)
{
	mock_reset_all();
	mock_spi_bus_init_results[0] = ESP_FAIL;
	digipot_mcp41hv51_config_t cfg = default_cfg();
	assert(digipot_mcp41hv51_init(&cfg) == ESP_FAIL);
	// Module should remain uninitialized
	assert(digipot_mcp41hv51_set_wiper(1) == ESP_ERR_INVALID_STATE);
}

void test_init_spi_device_add_fails(void)
{
	mock_reset_all();
	mock_spi_add_device_result = ESP_FAIL;
	digipot_mcp41hv51_config_t cfg = default_cfg();
	assert(digipot_mcp41hv51_init(&cfg) == ESP_FAIL);
	assert(digipot_mcp41hv51_set_wiper(1) == ESP_ERR_INVALID_STATE);
}

void test_init_spi_transmit_fails_on_initial_wiper_zero(void)
{
	mock_reset_all();
	mock_spi_transmit_result = ESP_FAIL;
	digipot_mcp41hv51_config_t cfg = default_cfg();
	assert(digipot_mcp41hv51_init(&cfg) == ESP_FAIL);
	assert(digipot_mcp41hv51_set_wiper(1) == ESP_ERR_INVALID_STATE);
}

void test_init_spi_bus_stale_retry_succeeds(void)
{
	mock_reset_all();
	mock_spi_bus_init_results[0] = ESP_ERR_INVALID_STATE; // first call: stale
	mock_spi_bus_init_results[1] = ESP_OK;                // retry: OK
	digipot_mcp41hv51_config_t cfg = default_cfg();
	assert(digipot_mcp41hv51_init(&cfg) == ESP_OK);
	assert(digipot_mcp41hv51_get_wiper() == 0);
}

void test_init_success(void)
{
	clean_init();
	assert(digipot_mcp41hv51_get_wiper() == 0);
	assert(digipot_mcp41hv51_is_autonomous() == false);
	// Init should transmit wiper=0
	assert(mock_spi_transmit_count == 1);
	assert(mock_spi_last_tx_data[0] == 0x00); // CMD_WRITE_VOLATILE_WIPER
	assert(mock_spi_last_tx_data[1] == 0);    // position 0
}

void test_reinit_releases_resources(void)
{
	// First init
	clean_init();
	assert(digipot_mcp41hv51_set_wiper(100) == ESP_OK);
	assert(digipot_mcp41hv51_get_wiper() == 100);

	// Re-init should reset state
	clean_init();
	assert(digipot_mcp41hv51_get_wiper() == 0);
	assert(digipot_mcp41hv51_is_autonomous() == false);
}

// ============================================================================
// Wiper control tests
// ============================================================================

void test_set_wiper_success(void)
{
	clean_init();
	assert(digipot_mcp41hv51_set_wiper(128) == ESP_OK);
	assert(digipot_mcp41hv51_get_wiper() == 128);
}

void test_set_wiper_min_max(void)
{
	clean_init();
	assert(digipot_mcp41hv51_set_wiper(0) == ESP_OK);
	assert(digipot_mcp41hv51_get_wiper() == 0);
	assert(digipot_mcp41hv51_set_wiper(255) == ESP_OK);
	assert(digipot_mcp41hv51_get_wiper() == 255);
}

void test_set_wiper_spi_failure(void)
{
	clean_init();
	mock_spi_transmit_result = ESP_FAIL;
	assert(digipot_mcp41hv51_set_wiper(50) == ESP_FAIL);
	// Wiper position should NOT be updated on failure
	assert(digipot_mcp41hv51_get_wiper() == 0);
}

void test_set_wiper_sends_correct_spi_data(void)
{
	clean_init();
	digipot_mcp41hv51_set_wiper(200);
	assert(mock_spi_last_tx_data[0] == 0x00); // write volatile wiper command
	assert(mock_spi_last_tx_data[1] == 200);  // position
	assert(mock_spi_last_tx_len == 2);
}

void test_disable_zeros_wiper(void)
{
	clean_init();
	digipot_mcp41hv51_set_wiper(100);
	assert(digipot_mcp41hv51_get_wiper() == 100);

	assert(digipot_mcp41hv51_disable() == ESP_OK);
	assert(digipot_mcp41hv51_get_wiper() == 0);
}

void test_disable_spi_failure(void)
{
	clean_init();
	digipot_mcp41hv51_set_wiper(100);
	mock_spi_transmit_result = ESP_FAIL;
	assert(digipot_mcp41hv51_disable() == ESP_FAIL);
	// Wiper should still be 100 since SPI failed
	assert(digipot_mcp41hv51_get_wiper() == 100);
}

// ============================================================================
// Autonomous mode tests
// ============================================================================

void test_enable_autonomous_success(void)
{
	clean_init();
	digipot_mcp41hv51_set_wiper(50);
	assert(digipot_mcp41hv51_enable_autonomous() == ESP_OK);
	assert(digipot_mcp41hv51_is_autonomous() == true);
	assert(digipot_mcp41hv51_get_wiper() == 0); // reset to idle
}

void test_enable_autonomous_spi_failure(void)
{
	clean_init();
	mock_spi_transmit_result = ESP_FAIL;
	assert(digipot_mcp41hv51_enable_autonomous() == ESP_FAIL);
	assert(digipot_mcp41hv51_is_autonomous() == false); // should NOT be set
}

void test_emergency_stop_zeros_and_clears_autonomous(void)
{
	clean_init();
	digipot_mcp41hv51_enable_autonomous();
	digipot_mcp41hv51_set_wiper(200);
	assert(digipot_mcp41hv51_is_autonomous() == true);
	assert(digipot_mcp41hv51_get_wiper() == 200);

	assert(digipot_mcp41hv51_emergency_stop() == ESP_OK);
	assert(digipot_mcp41hv51_get_wiper() == 0);
	assert(digipot_mcp41hv51_is_autonomous() == false);
}

void test_emergency_stop_spi_failure(void)
{
	clean_init();
	digipot_mcp41hv51_enable_autonomous();
	digipot_mcp41hv51_set_wiper(100);
	mock_spi_transmit_result = ESP_FAIL;
	assert(digipot_mcp41hv51_emergency_stop() == ESP_FAIL);
	// State should be unchanged on SPI failure
	assert(digipot_mcp41hv51_is_autonomous() == true);
	assert(digipot_mcp41hv51_get_wiper() == 100);
}

void test_disable_preserves_autonomous_flag(void)
{
	clean_init();
	digipot_mcp41hv51_enable_autonomous();
	digipot_mcp41hv51_set_wiper(128);
	assert(digipot_mcp41hv51_is_autonomous() == true);

	assert(digipot_mcp41hv51_disable() == ESP_OK);
	assert(digipot_mcp41hv51_get_wiper() == 0);
	// disable() zeros wiper but must NOT clear autonomous flag
	assert(digipot_mcp41hv51_is_autonomous() == true);
}

void test_autonomous_lifecycle(void)
{
	clean_init();
	assert(digipot_mcp41hv51_is_autonomous() == false);

	digipot_mcp41hv51_enable_autonomous();
	assert(digipot_mcp41hv51_is_autonomous() == true);

	digipot_mcp41hv51_set_wiper(128);
	assert(digipot_mcp41hv51_get_wiper() == 128);
	assert(digipot_mcp41hv51_is_autonomous() == true);

	digipot_mcp41hv51_emergency_stop();
	assert(digipot_mcp41hv51_is_autonomous() == false);
	assert(digipot_mcp41hv51_get_wiper() == 0);

	// Can re-enable after estop
	assert(digipot_mcp41hv51_enable_autonomous() == ESP_OK);
	assert(digipot_mcp41hv51_is_autonomous() == true);
}

// ============================================================================
// SPI transaction counting
// ============================================================================

void test_each_wiper_set_transmits(void)
{
	clean_init();
	int before = mock_spi_transmit_count;
	digipot_mcp41hv51_set_wiper(10);
	digipot_mcp41hv51_set_wiper(20);
	digipot_mcp41hv51_set_wiper(30);
	assert(mock_spi_transmit_count == before + 3);
}

} // namespace

int main(void)
{
	printf("\n=== digipot_mcp41hv51 unit tests ===\n\n");

	printf("  --- before init ---\n");
	TEST(test_set_wiper_before_init_fails);
	TEST(test_disable_before_init_fails);
	TEST(test_get_wiper_returns_zero_before_init);
	TEST(test_enable_autonomous_before_init_fails);
	TEST(test_emergency_stop_before_init_fails);
	TEST(test_is_autonomous_false_before_init);

	printf("\n  --- init ---\n");
	TEST(test_init_null_config_fails);
	TEST(test_init_spi_bus_fails);
	TEST(test_init_spi_device_add_fails);
	TEST(test_init_spi_transmit_fails_on_initial_wiper_zero);
	TEST(test_init_spi_bus_stale_retry_succeeds);
	TEST(test_init_success);
	TEST(test_reinit_releases_resources);

	printf("\n  --- wiper control ---\n");
	TEST(test_set_wiper_success);
	TEST(test_set_wiper_min_max);
	TEST(test_set_wiper_spi_failure);
	TEST(test_set_wiper_sends_correct_spi_data);
	TEST(test_disable_zeros_wiper);
	TEST(test_disable_spi_failure);

	printf("\n  --- autonomous mode ---\n");
	TEST(test_enable_autonomous_success);
	TEST(test_enable_autonomous_spi_failure);
	TEST(test_emergency_stop_zeros_and_clears_autonomous);
	TEST(test_emergency_stop_spi_failure);
	TEST(test_disable_preserves_autonomous_flag);
	TEST(test_autonomous_lifecycle);

	printf("\n  --- SPI transactions ---\n");
	TEST(test_each_wiper_set_transmits);

	TEST_REPORT();
	TEST_EXIT();
}
