/**
 * @file multiplexer_dg408djz.cpp
 * @brief DG408DJZ 8-channel analog multiplexer throttle control implementation.
 */
#include "multiplexer_dg408djz.h"

#include "esp_log.h"
#include "rom/ets_sys.h"

namespace
{

const char *TAG = "MULTIPLEXER";

// ============================================================================
// Timing Constants
// ============================================================================

constexpr uint32_t ADDRESS_SETTLE_US = 10; // Wait for address lines before enabling mux

// ============================================================================
// Module State
// ============================================================================

multiplexer_dg408djz_config_t s_config = {};
bool s_initialized = false;
int8_t s_current_level = -1; // -1 = mux disabled, 0-7 = active channel
bool s_autonomous = false;   // true = autonomous mode active (mux output to Curtis)

/**
 * @brief Set a GPIO level and verify the output matches the requested value.
 *
 * Writes the specified level to a multiplexer address or enable GPIO,
 * then reads it back to confirm the MCU output register holds the
 * expected state.  Used for all address line and enable pin writes
 * to detect GPIO misconfiguration or hardware faults.
 *
 * @param gpio   GPIO pin to set
 * @param level  Desired output level (0 or 1)
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if readback mismatch
 */
esp_err_t set_and_verify(gpio_num_t gpio, int level)
{
	esp_err_t err = gpio_set_level(gpio, level);
	if (err != ESP_OK)
		return err;
	if (gpio_get_level(gpio) != level)
		return ESP_ERR_INVALID_STATE;
	return ESP_OK;
}

} // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t multiplexer_dg408djz_init(const multiplexer_dg408djz_config_t *config)
{
	if (!config)
		return ESP_ERR_INVALID_ARG;

	s_config = *config;

	gpio_config_t io_conf = {
		.pin_bit_mask = (1ULL << config->a0) | (1ULL << config->a1) | (1ULL << config->a2) | (1ULL << config->en),
		.mode = GPIO_MODE_INPUT_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	esp_err_t err = gpio_config(&io_conf);
	if (err != ESP_OK)
		return err;

	// Safe state: mux disabled
	gpio_set_level(config->a0, 0);
	gpio_set_level(config->a1, 0);
	gpio_set_level(config->a2, 0);
	gpio_set_level(config->en, 0);

	// Verify MCU-side output levels reached expected safe defaults.
	if (gpio_get_level(config->a0) != 0 || gpio_get_level(config->a1) != 0 || gpio_get_level(config->a2) != 0 ||
	    gpio_get_level(config->en) != 0)
	{
		return ESP_ERR_INVALID_STATE;
	}

	s_current_level = -1;
	s_autonomous = false;
	s_initialized = true;

	return ESP_OK;
}

// ============================================================================
// Level Control
// ============================================================================

esp_err_t multiplexer_dg408djz_set_level(int8_t level)
{
	if (!s_initialized)
	{
		ESP_LOGE(TAG, "Not initialized");
		return ESP_ERR_INVALID_STATE;
	}

	if (level < 0 || level > 7)
	{
		esp_err_t err = set_and_verify(s_config.en, 0);
		if (err != ESP_OK)
			return err;
		s_current_level = -1;
#ifdef CONFIG_LOG_ACTUATOR_MUX_LEVEL
		ESP_LOGI(TAG, "Disabled (level=%d)", level);
#endif
		return ESP_OK;
	}

	// Set address lines BEFORE enabling - prevents glitching through wrong channel
	esp_err_t err = set_and_verify(s_config.a0, (level >> 0) & 1);
	if (err != ESP_OK)
		return err;
	err = set_and_verify(s_config.a1, (level >> 1) & 1);
	if (err != ESP_OK)
		return err;
	err = set_and_verify(s_config.a2, (level >> 2) & 1);
	if (err != ESP_OK)
		return err;

	ets_delay_us(ADDRESS_SETTLE_US);

	err = set_and_verify(s_config.en, 1);
	if (err != ESP_OK)
		return err;
	s_current_level = level;

#ifdef CONFIG_LOG_ACTUATOR_MUX_LEVEL
	ESP_LOGI(TAG, "Level set to %d (A2=%d A1=%d A0=%d)", level, (level >> 2) & 1, (level >> 1) & 1, (level >> 0) & 1);
#endif
	return ESP_OK;
}

esp_err_t multiplexer_dg408djz_disable(void)
{
	if (!s_initialized)
		return ESP_ERR_INVALID_STATE;

	esp_err_t err = set_and_verify(s_config.en, 0);
	if (err != ESP_OK)
		return err;
	s_current_level = -1;
#ifdef CONFIG_LOG_ACTUATOR_MUX_LEVEL
	ESP_LOGI(TAG, "Mux disabled");
#endif
	return ESP_OK;
}

int8_t multiplexer_dg408djz_get_level(void)
{
	return s_current_level;
}

// ============================================================================
// Autonomous Mode Control
// ============================================================================

esp_err_t multiplexer_dg408djz_enable_autonomous(void)
{
	if (!s_initialized)
	{
		ESP_LOGE(TAG, "Not initialized");
		return ESP_ERR_INVALID_STATE;
	}

#ifdef CONFIG_LOG_ACTUATOR_MUX_LEVEL
	ESP_LOGI(TAG, "Enabling autonomous mode");
#endif

	// Start at idle throttle
	esp_err_t err = multiplexer_dg408djz_set_level(0);
	if (err != ESP_OK)
		return err;

	s_autonomous = true;

#ifdef CONFIG_LOG_ACTUATOR_MUX_LEVEL
	ESP_LOGI(TAG, "Autonomous mode ENABLED");
#endif
	return ESP_OK;
}

esp_err_t multiplexer_dg408djz_emergency_stop(void)
{
	if (!s_initialized)
		return ESP_ERR_INVALID_STATE;

#ifdef CONFIG_LOG_ACTUATOR_MUX_LEVEL
	ESP_LOGI(TAG, "EMERGENCY STOP");
#endif

	// Immediately disable mux output
	esp_err_t err = set_and_verify(s_config.en, 0);
	if (err != ESP_OK)
		return err;
	s_current_level = -1;

	s_autonomous = false;
	return ESP_OK;
}

bool multiplexer_dg408djz_is_autonomous(void)
{
	return s_autonomous;
}
