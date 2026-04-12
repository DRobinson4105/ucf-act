/**
 * @file led_ws2812.cpp
 * @brief Raw WS2812 RGB transport driver implementation.
 */
#include "led_ws2812.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"

namespace
{

const char *TAG = "LED";

// ============================================================================
// Hardware Constants
// ============================================================================

// Onboard WS2812 RGB LED pin on ESP32-C6 dev board
constexpr gpio_num_t LED_GPIO = GPIO_NUM_8;

// ============================================================================
// Module State
// ============================================================================

// Init is called once from main_task before runtime tasks start — no lock needed.
bool s_initialized = false;

// ============================================================================
// WS2812 RMT Driver
// ============================================================================

// Init is called once from main_task before runtime tasks start — no lock needed.
rmt_channel_handle_t s_rmt_channel = nullptr;
rmt_encoder_handle_t s_rmt_encoder = nullptr;

// WS2812 timing at 10 MHz RMT clock (0.1 us per tick)
constexpr uint32_t WS2812_RESOLUTION_HZ = 10 * 1000 * 1000;
constexpr uint16_t WS2812_T0H = 4; // 0.4us high for '0' bit
constexpr uint16_t WS2812_T0L = 8; // 0.8us low for '0' bit
constexpr uint16_t WS2812_T1H = 7; // 0.7us high for '1' bit
constexpr uint16_t WS2812_T1L = 6; // 0.6us low for '1' bit

/**
 * @brief Transmit a 24-bit color to the WS2812 LED via RMT.
 *
 * Converts the RGB struct into 24 RMT symbols in GRB byte order
 * (MSB first) and transmits them through the configured RMT channel.
 * No-op if the RMT channel or encoder has not been initialized.
 *
 * @param color  RGB color to display
 */
void ws2812_write_rgb(led_ws2812_rgb_t color)
{
	if (!s_rmt_channel || !s_rmt_encoder)
		return;

	rmt_symbol_word_t symbols[24];
	rmt_symbol_word_t one = {.duration0 = WS2812_T1H, .level0 = 1, .duration1 = WS2812_T1L, .level1 = 0};
	rmt_symbol_word_t zero = {.duration0 = WS2812_T0H, .level0 = 1, .duration1 = WS2812_T0L, .level1 = 0};

	// WS2812 expects GRB order, MSB first
	int idx = 0;
	uint8_t grb[3] = {color.g, color.r, color.b};
	for (int ch = 0; ch < 3; ++ch)
		for (int bit = 7; bit >= 0; --bit)
			symbols[idx++] = (grb[ch] & (1 << bit)) ? one : zero;

	rmt_transmit_config_t tx_cfg = {};
	tx_cfg.loop_count = 0;
	tx_cfg.flags.eot_level = 0;

	esp_err_t err = rmt_transmit(s_rmt_channel, s_rmt_encoder, symbols, sizeof(symbols), &tx_cfg);
	if (err != ESP_OK)
		ESP_LOGW(TAG, "RMT transmit failed: %s", esp_err_to_name(err));
}

} // namespace

// ============================================================================
// Public API
// ============================================================================

esp_err_t led_ws2812_init(void)
{
	if (s_initialized)
	{
		ESP_LOGW(TAG, "Already initialized, skipping re-init");
		return ESP_OK;
	}

	rmt_tx_channel_config_t tx_cfg = {};
	tx_cfg.gpio_num = LED_GPIO;
	tx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
	tx_cfg.resolution_hz = WS2812_RESOLUTION_HZ;
	tx_cfg.mem_block_symbols = 64;
	tx_cfg.trans_queue_depth = 4;
	tx_cfg.intr_priority = 0;
	tx_cfg.flags.invert_out = 0;
	tx_cfg.flags.with_dma = 0;
	tx_cfg.flags.io_loop_back = 0;
	tx_cfg.flags.io_od_mode = 0;
	tx_cfg.flags.allow_pd = 0;

	esp_err_t err = rmt_new_tx_channel(&tx_cfg, &s_rmt_channel);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "RMT channel init failed: %s", esp_err_to_name(err));
		return err;
	}

	rmt_copy_encoder_config_t encoder_cfg = {};
	err = rmt_new_copy_encoder(&encoder_cfg, &s_rmt_encoder);
	if (err != ESP_OK)
	{
		rmt_del_channel(s_rmt_channel);
		s_rmt_channel = nullptr;
		ESP_LOGE(TAG, "RMT encoder init failed: %s", esp_err_to_name(err));
		return err;
	}

	err = rmt_enable(s_rmt_channel);
	if (err != ESP_OK)
	{
		rmt_del_encoder(s_rmt_encoder);
		s_rmt_encoder = nullptr;
		rmt_del_channel(s_rmt_channel);
		s_rmt_channel = nullptr;
		ESP_LOGE(TAG, "RMT enable failed: %s", esp_err_to_name(err));
		return err;
	}

	ws2812_write_rgb({0, 0, 0});

	s_initialized = true;

	return ESP_OK;
}

void led_ws2812_set_rgb(led_ws2812_rgb_t color)
{
	if (!s_initialized)
		return;
	ws2812_write_rgb(color);
}
