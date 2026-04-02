/**
 * @file battery_monitor.cpp
 * @brief Battery pack voltage, current, and SOC monitoring implementation.
 *
 * Reads two ADC1 channels (pack voltage via resistor divider, pack current
 * via a hall-effect sensor) and estimates state of charge using a
 * voltage lookup table with coulomb-counting refinement.
 *
 * Design:
 *   - Both channels share one ADC1 unit handle (ESP-IDF oneshot API supports
 *     multiple channels per unit).
 *   - Exponential moving average (EMA) filters on both readings.
 *   - SOC from open-circuit voltage table (linear interpolation) as baseline.
 *   - Coulomb counting (current integration) tracks delta-SOC during driving.
 *   - Auto-recalibrates SOC from voltage when cart is idle (current ≈ 0) for
 *     an extended period.
 *   - Zero-offset auto-calibration on the current sensor at boot.
 */

#include "battery_monitor.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

namespace
{

[[maybe_unused]] const char *TAG = "BATTERY_MON";

// ============================================================================
// ADC Constants
// ============================================================================

constexpr int ADC_RAW_MIN = 0;
constexpr int ADC_RAW_MAX = 4095;
constexpr int ADC_MV_MAX = 3300;

// ============================================================================
// Filtering Constants
// ============================================================================

// EMA alpha for voltage: ~1s time constant at 20 Hz (alpha ≈ 1/(20*1) = 0.05)
constexpr float VOLTAGE_EMA_ALPHA = 0.05f;

// EMA alpha for current: ~250ms time constant at 20 Hz (alpha ≈ 1/(20*0.25) = 0.2)
constexpr float CURRENT_EMA_ALPHA = 0.2f;

// Voltage-SOC blend alpha: continuously corrects coulomb counting drift.
// At 20 Hz with alpha=0.001, time constant ≈ 50s (~2-3 min to converge).
constexpr float VOLTAGE_SOC_BLEND_ALPHA = 0.001f;

// ============================================================================
// SOC Constants
// ============================================================================

// Idle detection: cart must draw < this current (mA) for IDLE_RECAL_MS
// before SOC recalibrates from voltage.
constexpr int32_t IDLE_CURRENT_THRESHOLD_MA = 500;

// Time the cart must be idle before voltage-based SOC recalibration (ms).
constexpr uint32_t IDLE_RECAL_MS = 300000; // 5 minutes

// Zero-offset calibration tolerance: at boot, if measured current is within
// this range of expected zero, accept the reading as the true zero offset.
constexpr int32_t ZERO_CAL_TOLERANCE_MA = 2000; // ±2A

// ============================================================================
// Voltage-to-SOC Lookup Table (48V lead-acid, 24 cells)
// ============================================================================
// Open-circuit voltages for a generic 6 × 8V deep-cycle lead-acid golf cart
// battery pack (24 cells in series, 48V nominal).  Based on standard flooded
// lead-acid OCV curve at ~2.00 V/cell = 50% SOC, ~2.13 V/cell = 100% SOC.
// These are approximate and may need tuning for specific battery brands.

struct soc_entry_t
{
	uint16_t mv; // Pack voltage in mV
	uint8_t pct; // SOC percentage
};

constexpr soc_entry_t SOC_TABLE[] = {
	{42000, 0},   // 1.75 V/cell — dead
	{44900, 10},  // 1.87 V/cell
	{45800, 20},  // 1.91 V/cell
	{46600, 30},  // 1.94 V/cell
	{47300, 40},  // 1.97 V/cell
	{48000, 50},  // 2.00 V/cell — nominal
	{48700, 60},  // 2.03 V/cell
	{49400, 70},  // 2.06 V/cell
	{49900, 80},  // 2.08 V/cell
	{50400, 90},  // 2.10 V/cell
	{51200, 100}, // 2.13 V/cell — fully charged (resting)
};

constexpr size_t SOC_TABLE_LEN = sizeof(SOC_TABLE) / sizeof(SOC_TABLE[0]);

// ============================================================================
// Module State
// ============================================================================

bool s_initialized = false;
battery_monitor_config_t s_config = {};

// ADC handles
adc_oneshot_unit_handle_t s_adc_handle = nullptr;
adc_channel_t s_voltage_channel = ADC_CHANNEL_0;
adc_channel_t s_current_channel = ADC_CHANNEL_1;
adc_cali_handle_t s_voltage_cali = nullptr;
adc_cali_handle_t s_current_cali = nullptr;
bool s_voltage_cali_ok = false;
bool s_current_cali_ok = false;

// Filtered readings
float s_voltage_filtered_mv = 0.0f; // Pack voltage in mV (after divider undo)
float s_current_filtered_ma = 0.0f; // Pack current in mA (positive = discharge)
bool s_filters_primed = false;      // First sample initializes filters directly

// SOC state
uint8_t s_soc_pct = 0;
float s_coulomb_mah_used = 0.0f; // mAh consumed since last recalibration
float s_soc_coulomb_base = 0.0f; // SOC% at last recalibration point

// Idle tracking for voltage-based SOC recalibration
uint32_t s_idle_start_ms = 0;
bool s_is_idle = false;

// Zero-offset calibration
int32_t s_current_zero_offset_mv = 0; // Actual sensor zero in mV (after output divider)
bool s_zero_calibrated = false;

// Health tracking
uint8_t s_voltage_fail_count = 0;
uint8_t s_current_fail_count = 0;
constexpr uint8_t FAIL_COUNT_THRESHOLD = 10; // consecutive failures before unhealthy

// Timing
uint32_t s_last_update_ms = 0;

// ============================================================================
// Internal Helpers
// ============================================================================

/**
 * @brief Look up SOC percentage from pack voltage using the lookup table.
 *
 * Linearly interpolates between table entries. Clamps to 0-100%.
 *
 * @param voltage_mv  Pack voltage in millivolts
 * @return SOC percentage (0-100)
 */
uint8_t voltage_to_soc(uint16_t voltage_mv)
{
	// Below minimum
	if (voltage_mv <= SOC_TABLE[0].mv)
		return 0;

	// Above maximum
	if (voltage_mv >= SOC_TABLE[SOC_TABLE_LEN - 1].mv)
		return 100;

	// Find the bracketing entries and interpolate
	for (size_t i = 1; i < SOC_TABLE_LEN; i++)
	{
		if (voltage_mv <= SOC_TABLE[i].mv)
		{
			uint16_t v_low = SOC_TABLE[i - 1].mv;
			uint16_t v_high = SOC_TABLE[i].mv;
			uint8_t pct_low = SOC_TABLE[i - 1].pct;
			uint8_t pct_high = SOC_TABLE[i].pct;

			// Linear interpolation
			float fraction = (float)(voltage_mv - v_low) / (float)(v_high - v_low);
			float pct = pct_low + fraction * (pct_high - pct_low);
			if (pct < 0.0f)
				pct = 0.0f;
			if (pct > 100.0f)
				pct = 100.0f;
			return (uint8_t)(pct + 0.5f);
		}
	}

	return 100; // Should not reach here
}

/**
 * @brief Read a calibrated millivolt value from an ADC channel.
 *
 * @param channel    ADC channel to read
 * @param cali       Calibration handle (may be NULL for uncalibrated)
 * @param cali_ok    Whether calibration is available
 * @param out_mv     Output millivolt value
 * @return ESP_OK on success
 */
esp_err_t read_channel_mv(adc_channel_t channel, adc_cali_handle_t cali, bool cali_ok, int *out_mv)
{
	int raw = 0;
	esp_err_t err = adc_oneshot_read(s_adc_handle, channel, &raw);
	if (err != ESP_OK)
		return err;

	if (raw < ADC_RAW_MIN || raw > ADC_RAW_MAX)
		return ESP_ERR_INVALID_RESPONSE;

	if (cali_ok && cali)
	{
		err = adc_cali_raw_to_voltage(cali, raw, out_mv);
		if (err != ESP_OK)
			return err;
	}
	else
	{
		*out_mv = (raw * ADC_MV_MAX) / ADC_RAW_MAX;
	}

	return ESP_OK;
}

/**
 * @brief Set up calibration for a single ADC channel.
 *
 * @param unit       ADC unit
 * @param channel    ADC channel
 * @param out_cali   Output calibration handle
 * @return true if calibration was set up successfully
 */
bool setup_calibration(adc_unit_t unit, adc_channel_t channel, adc_cali_handle_t *out_cali)
{
	adc_cali_curve_fitting_config_t cali_config = {
		.unit_id = unit,
		.chan = channel,
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_12,
	};

	esp_err_t err = adc_cali_create_scheme_curve_fitting(&cali_config, out_cali);
	return (err == ESP_OK);
}

} // namespace

// ============================================================================
// Public API
// ============================================================================

esp_err_t battery_monitor_init(const battery_monitor_config_t *config)
{
	if (!config)
		return ESP_ERR_INVALID_ARG;
	if (config->current_output_scale <= 0.0f || config->current_sens_uv <= 0.0f)
		return ESP_ERR_INVALID_ARG;

	battery_monitor_deinit();

	s_config = *config;

	// Resolve GPIOs to ADC channels
	adc_unit_t unit_v = ADC_UNIT_1;
	adc_unit_t unit_c = ADC_UNIT_1;
	esp_err_t err = adc_oneshot_io_to_channel(config->voltage_gpio, &unit_v, &s_voltage_channel);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "voltage GPIO %d is not a valid ADC pin: %s", config->voltage_gpio, esp_err_to_name(err));
		return err;
	}

	err = adc_oneshot_io_to_channel(config->current_gpio, &unit_c, &s_current_channel);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "current GPIO %d is not a valid ADC pin: %s", config->current_gpio, esp_err_to_name(err));
		return err;
	}

	if (unit_v != ADC_UNIT_1 || unit_c != ADC_UNIT_1)
	{
		ESP_LOGE(TAG, "Both GPIOs must be on ADC1 (voltage unit=%d, current unit=%d)", unit_v, unit_c);
		return ESP_ERR_INVALID_ARG;
	}

	// Create ADC1 unit handle (shared by both channels)
	adc_oneshot_unit_init_cfg_t adc_config = {
		.unit_id = ADC_UNIT_1,
		.clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};

	err = adc_oneshot_new_unit(&adc_config, &s_adc_handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(err));
		return err;
	}

	// Configure both channels
	adc_oneshot_chan_cfg_t chan_config = {
		.atten = ADC_ATTEN_DB_12,
		.bitwidth = ADC_BITWIDTH_12,
	};

	err = adc_oneshot_config_channel(s_adc_handle, s_voltage_channel, &chan_config);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Voltage channel config failed: %s", esp_err_to_name(err));
		adc_oneshot_del_unit(s_adc_handle);
		s_adc_handle = nullptr;
		return err;
	}

	err = adc_oneshot_config_channel(s_adc_handle, s_current_channel, &chan_config);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Current channel config failed: %s", esp_err_to_name(err));
		adc_oneshot_del_unit(s_adc_handle);
		s_adc_handle = nullptr;
		return err;
	}

	// Set up calibration for both channels
	s_voltage_cali_ok = setup_calibration(ADC_UNIT_1, s_voltage_channel, &s_voltage_cali);
	s_current_cali_ok = setup_calibration(ADC_UNIT_1, s_current_channel, &s_current_cali);

	if (!s_voltage_cali_ok)
		ESP_LOGW(TAG, "Voltage channel calibration unavailable — using linear approximation");
	if (!s_current_cali_ok)
		ESP_LOGW(TAG, "Current channel calibration unavailable — using linear approximation");

	// Reset state
	s_filters_primed = false;
	s_voltage_filtered_mv = 0.0f;
	s_current_filtered_ma = 0.0f;
	s_soc_pct = 0;
	s_coulomb_mah_used = 0.0f;
	s_soc_coulomb_base = 0.0f;
	s_is_idle = false;
	s_idle_start_ms = 0;
	s_zero_calibrated = false;
	s_current_zero_offset_mv = 0;
	s_voltage_fail_count = 0;
	s_current_fail_count = 0;
	s_last_update_ms = 0;

	s_initialized = true;

	ESP_LOGI(TAG, "Initialized: voltage_gpio=%d (ch%d) current_gpio=%d (ch%d) divider=%.2f capacity=%lumAh",
	         config->voltage_gpio, s_voltage_channel, config->current_gpio, s_current_channel,
	         (double)config->divider_ratio, (unsigned long)config->capacity_mah);

	return ESP_OK;
}

void battery_monitor_deinit(void)
{
	if (s_voltage_cali)
	{
		adc_cali_delete_scheme_curve_fitting(s_voltage_cali);
		s_voltage_cali = nullptr;
	}
	if (s_current_cali)
	{
		adc_cali_delete_scheme_curve_fitting(s_current_cali);
		s_current_cali = nullptr;
	}
	if (s_adc_handle)
	{
		adc_oneshot_del_unit(s_adc_handle);
		s_adc_handle = nullptr;
	}

	s_voltage_cali_ok = false;
	s_current_cali_ok = false;
	s_initialized = false;
}

void battery_monitor_update(uint32_t now_ms)
{
	if (!s_initialized || !s_adc_handle)
		return;

	// ── Compute dt ──────────────────────────────────────────────────────
	float dt_s = 0.0f;
	if (s_last_update_ms != 0 && (now_ms - s_last_update_ms) > 0)
	{
		dt_s = (float)(now_ms - s_last_update_ms) / 1000.0f;
	}
	s_last_update_ms = now_ms;

	// ── Read voltage channel ────────────────────────────────────────────
	int voltage_adc_mv = 0;
	esp_err_t v_err = read_channel_mv(s_voltage_channel, s_voltage_cali, s_voltage_cali_ok, &voltage_adc_mv);
	if (v_err == ESP_OK)
	{
		s_voltage_fail_count = 0;
		// Undo divider: pack_voltage = adc_mv × divider_ratio
		float pack_mv = (float)voltage_adc_mv * (float)s_config.divider_ratio;

		if (!s_filters_primed)
		{
			s_voltage_filtered_mv = pack_mv;
		}
		else
		{
			s_voltage_filtered_mv += VOLTAGE_EMA_ALPHA * (pack_mv - s_voltage_filtered_mv);
		}
	}
	else
	{
		if (s_voltage_fail_count < FAIL_COUNT_THRESHOLD)
			s_voltage_fail_count++;
	}

	// ── Read current channel ────────────────────────────────────────────
	int current_adc_mv = 0;
	esp_err_t c_err = read_channel_mv(s_current_channel, s_current_cali, s_current_cali_ok, &current_adc_mv);
	if (c_err == ESP_OK)
	{
		s_current_fail_count = 0;

		// Undo the output divider to get the actual sensor output voltage.
		// sensor_mv = adc_mv / output_scale
		float sensor_mv = (float)current_adc_mv / s_config.current_output_scale;

		// Zero-offset auto-calibration on first good reading:
		// At boot the cart should be idle, so the sensor output represents 0A.
		// Record this as the true zero offset instead of using the nominal value.
		if (!s_zero_calibrated)
		{
			// Check if the reading is close to the expected zero
			int32_t expected_zero = (int32_t)s_config.current_zero_mv;
			float deviation_ma_f = (sensor_mv - (float)expected_zero) * 1000.0f / s_config.current_sens_uv;
			int32_t deviation_ma = (int32_t)deviation_ma_f;

			if (deviation_ma > -ZERO_CAL_TOLERANCE_MA && deviation_ma < ZERO_CAL_TOLERANCE_MA)
			{
				s_current_zero_offset_mv = (int32_t)(sensor_mv + 0.5f);
				s_zero_calibrated = true;
				ESP_LOGI(TAG, "Current sensor zero calibrated: %ldmV (nominal: %umV, deviation: %ldmA)",
				         (long)s_current_zero_offset_mv, s_config.current_zero_mv, (long)deviation_ma);
			}
			else
			{
				// Deviation too large — cart may not be idle, use nominal zero
				s_current_zero_offset_mv = expected_zero;
				s_zero_calibrated = true;
				ESP_LOGW(TAG, "Current sensor zero too far from nominal (%ldmA deviation), using nominal %umV",
				         (long)deviation_ma, s_config.current_zero_mv);
			}
		}

		// Convert sensor voltage to current:
		// current_ma = (sensor_mv - zero_mv) × 1000 / sensitivity_uv
		// Positive = discharge (current flowing out of battery)
		float current_ma = (sensor_mv - (float)s_current_zero_offset_mv) * 1000.0f / s_config.current_sens_uv;

		if (!s_filters_primed)
		{
			s_current_filtered_ma = current_ma;
		}
		else
		{
			s_current_filtered_ma += CURRENT_EMA_ALPHA * (current_ma - s_current_filtered_ma);
		}
	}
	else
	{
		if (s_current_fail_count < FAIL_COUNT_THRESHOLD)
			s_current_fail_count++;
	}

	// Mark filters as primed after first successful reads
	if (!s_filters_primed && v_err == ESP_OK && c_err == ESP_OK)
	{
		s_filters_primed = true;

		// Initialize SOC from voltage on first reading
		s_soc_pct = voltage_to_soc((uint16_t)(s_voltage_filtered_mv + 0.5f));
		s_soc_coulomb_base = (float)s_soc_pct;
		s_coulomb_mah_used = 0.0f;
		s_idle_start_ms = now_ms;

		ESP_LOGI(TAG, "Initial reading: voltage=%umV current=%dmA soc=%u%%", (unsigned)(s_voltage_filtered_mv + 0.5f),
		         (int)(s_current_filtered_ma + 0.5f), s_soc_pct);
	}

	if (!s_filters_primed)
		return;

	// ── Coulomb counting ────────────────────────────────────────────────
	if (dt_s > 0.0f && dt_s < 1.0f) // Guard against absurd dt (>1s means we missed ticks)
	{
		// mAh = mA × hours = mA × (seconds / 3600)
		float delta_mah = s_current_filtered_ma * (dt_s / 3600.0f);
		s_coulomb_mah_used += delta_mah;
	}

	// ── Idle detection and voltage-based SOC recalibration ──────────────
	float abs_current = s_current_filtered_ma;
	if (abs_current < 0.0f)
		abs_current = -abs_current;

	if (abs_current < (float)IDLE_CURRENT_THRESHOLD_MA)
	{
		if (!s_is_idle)
		{
			s_is_idle = true;
			s_idle_start_ms = now_ms;
		}
		else if ((now_ms - s_idle_start_ms) >= IDLE_RECAL_MS)
		{
			// Cart has been idle long enough — recalibrate SOC from voltage.
			// Open-circuit voltage is reliable for lead-acid when resting.
			uint8_t voltage_soc = voltage_to_soc((uint16_t)(s_voltage_filtered_mv + 0.5f));
			s_soc_coulomb_base = (float)voltage_soc;
			s_coulomb_mah_used = 0.0f;
			s_soc_pct = voltage_soc;

			// Reset idle timer so we don't recalibrate every tick
			s_idle_start_ms = now_ms;

#ifdef CONFIG_LOG_INPUT_BATTERY_SOC_CHANGES
			ESP_LOGI(TAG, "SOC recalibrated from voltage: %u%% (idle >%lus)", voltage_soc,
			         (unsigned long)(IDLE_RECAL_MS / 1000));
#endif
		}
	}
	else
	{
		s_is_idle = false;
	}

	// ── Compute SOC from coulomb counting ───────────────────────────────
	// soc = base_soc - (mAh_used / capacity_mAh) × 100
	float soc_delta_pct = (s_coulomb_mah_used / (float)s_config.capacity_mah) * 100.0f;
	float soc_estimate = s_soc_coulomb_base - soc_delta_pct;

	// ── Blend voltage-derived SOC to correct coulomb counting drift ─────
	// Nudge coulomb base toward voltage SOC each tick so drift is
	// corrected over ~2-3 minutes without requiring an idle period.
	float voltage_soc = (float)voltage_to_soc((uint16_t)(s_voltage_filtered_mv + 0.5f));
	s_soc_coulomb_base += VOLTAGE_SOC_BLEND_ALPHA * (voltage_soc - soc_estimate);
	soc_estimate = s_soc_coulomb_base - soc_delta_pct;

	// Clamp to 0-100
	if (soc_estimate < 0.0f)
		soc_estimate = 0.0f;
	if (soc_estimate > 100.0f)
		soc_estimate = 100.0f;

	uint8_t new_soc = (uint8_t)(soc_estimate + 0.5f);

#ifdef CONFIG_LOG_INPUT_BATTERY_SOC_CHANGES
	if (new_soc != s_soc_pct)
	{
		ESP_LOGI(TAG, "SOC: %u%% -> %u%% (voltage=%umV current=%dmA coulomb_used=%.1fmAh)", s_soc_pct, new_soc,
		         (unsigned)(s_voltage_filtered_mv + 0.5f), (int)(s_current_filtered_ma + 0.5f), s_coulomb_mah_used);
	}
#endif

	s_soc_pct = new_soc;

#ifdef CONFIG_LOG_INPUT_BATTERY_VOLTAGE
	ESP_LOGI(TAG, "V=%umV I=%dmA SOC=%u%% idle=%d zero_cal=%ldmV", (unsigned)(s_voltage_filtered_mv + 0.5f),
	         (int)(s_current_filtered_ma + 0.5f), s_soc_pct, s_is_idle, (long)s_current_zero_offset_mv);
#endif
}

void battery_monitor_get_status(battery_status_t *out)
{
	if (!out)
		return;

	out->voltage_mv = s_filters_primed ? (uint16_t)(s_voltage_filtered_mv + 0.5f) : 0;

	// Convert current mA to 10mA units (round toward nearest, matching voltage rounding)
	out->current_10ma =
		s_filters_primed ? (int16_t)(s_current_filtered_ma / 10.0f + (s_current_filtered_ma >= 0 ? 0.5f : -0.5f)) : 0;

	out->soc_pct = s_soc_pct;

	// Build flags
	out->flags = 0;
	if (s_current_filtered_ma < -500.0f)
		out->flags |= BATTERY_FLAG_CHARGING;
	if (s_soc_pct <= 20)
		out->flags |= BATTERY_FLAG_LOW_WARNING;
	if (s_soc_pct <= 10)
		out->flags |= BATTERY_FLAG_CRITICAL;
	if (!battery_monitor_is_healthy())
		out->flags |= BATTERY_FLAG_SENSOR_FAULT;
}

bool battery_monitor_is_healthy(void)
{
	if (!s_initialized)
		return false;
	return (s_voltage_fail_count < FAIL_COUNT_THRESHOLD) && (s_current_fail_count < FAIL_COUNT_THRESHOLD);
}
