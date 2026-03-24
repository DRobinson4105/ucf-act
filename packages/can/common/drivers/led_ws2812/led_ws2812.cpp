/**
 * @file led_ws2812.cpp
 * @brief WS2812 status LED driver implementation.
 */
#include "led_ws2812.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

// Minimum interval between hardware updates (avoids flooding the RMT peripheral)
constexpr TickType_t UPDATE_INTERVAL = pdMS_TO_TICKS(500);

// ============================================================================
// State-to-Color Mapping
// ============================================================================

constexpr uint8_t LED_BRIGHTNESS = 16;

struct rgb_t
{
	uint8_t r, g, b;
};

/**
 * @brief Map a node state to its corresponding LED color.
 *
 * Returns the RGB values used to indicate the current system state
 * on the onboard WS2812 LED: green for READY, blue for ACTIVE,
 * red for NOT_READY, and yellow for INIT/ENABLE.
 *
 * @param state  Current node state
 * @return RGB color struct at LED_BRIGHTNESS intensity
 */
rgb_t state_to_rgb(node_state_t state)
{
	switch (state)
	{
	case NODE_STATE_READY:
		return {0, LED_BRIGHTNESS, 0}; // Green
	case NODE_STATE_ACTIVE:
		return {0, 0, LED_BRIGHTNESS}; // Blue
	case NODE_STATE_NOT_READY:
		return {LED_BRIGHTNESS, 0, 0}; // Red
	case NODE_STATE_INIT:
	case NODE_STATE_ENABLE:
	default:
		return {LED_BRIGHTNESS, LED_BRIGHTNESS, 0}; // Yellow
	}
}

// ============================================================================
// Module State
// ============================================================================

bool s_initialized = false;
volatile TickType_t s_last_update = 0;
volatile node_state_t s_node_state = NODE_STATE_INIT;
volatile bool s_fault_overlay = false;

// ============================================================================
// WS2812 RMT Driver
// ============================================================================

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
 * @param c  RGB color to display
 */
void ws2812_set_rgb(const rgb_t &c)
{
	if (!s_rmt_channel || !s_rmt_encoder)
		return;

	rmt_symbol_word_t symbols[24];
	rmt_symbol_word_t one = {.duration0 = WS2812_T1H, .level0 = 1, .duration1 = WS2812_T1L, .level1 = 0};
	rmt_symbol_word_t zero = {.duration0 = WS2812_T0H, .level0 = 1, .duration1 = WS2812_T0L, .level1 = 0};

	// WS2812 expects GRB order, MSB first
	int idx = 0;
	uint8_t grb[3] = {c.g, c.r, c.b};
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

	ws2812_set_rgb({0, 0, 0});

	s_last_update = xTaskGetTickCount();
	s_initialized = true;

	return ESP_OK;
}

void led_ws2812_set_state(node_state_t node_state)
{
	if (!s_initialized)
		return;

	bool state_changed = (node_state != s_node_state);
	s_node_state = node_state;

	TickType_t now = xTaskGetTickCount();
	if (!state_changed && (now - s_last_update) < UPDATE_INTERVAL)
		return;

	rgb_t c = s_fault_overlay ? rgb_t{LED_BRIGHTNESS, 0, 0} : state_to_rgb(node_state);
	ws2812_set_rgb(c);
	s_last_update = now;

#ifdef CONFIG_LOG_HEARTBEAT_LED_COLOR_UPDATES
	ESP_LOGI(TAG, "LED: state=%s rgb=(%u,%u,%u)", node_state_to_string(node_state), c.r, c.g, c.b);
#endif
}

void led_ws2812_set_fault_overlay(bool enabled)
{
	if (!s_initialized)
		return;

	if (s_fault_overlay == enabled)
		return;

	s_fault_overlay = enabled;
	// Force immediate color update.
	led_ws2812_set_state(s_node_state);
}
