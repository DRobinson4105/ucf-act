/**
 * @file led_ws2812.cpp
 * @brief WS2812 status LED driver implementation.
 */
#include "led_ws2812.hh"

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"

#include <string.h>

namespace {

static const char *TAG = "HEARTBEAT_LED";

// ============================================================================
// Module State
// ============================================================================

static bool s_initialized = false;
static TickType_t s_last_update = 0;
static volatile TickType_t s_last_activity = 0;
static TickType_t s_activity_window = 0;
static volatile bool s_error = false;
static bool s_manual_mode = false;

// Color configuration
static uint8_t s_idle_red = 0, s_idle_green = 0, s_idle_blue = 0;
static uint8_t s_activity_red = 0, s_activity_green = 0, s_activity_blue = 0;
static uint8_t s_error_red = 0, s_error_green = 0, s_error_blue = 0;
static uint8_t s_manual_red = 0, s_manual_green = 0, s_manual_blue = 0;

constexpr size_t LED_REASON_MAX_LEN = 48;
static char s_manual_reason[LED_REASON_MAX_LEN] = "manual";

#if defined(CONFIG_LOG_HEARTBEAT_LED_COLOR_UPDATES) || defined(CONFIG_LOG_HEARTBEAT_LED_STATE_CHANGES)
static const char *led_color_name(uint8_t red, uint8_t green, uint8_t blue) {
    if (red == 0 && green == 0 && blue == 0) return "OFF";
    if (red > 0 && green == 0 && blue == 0) return "RED";
    if (red == 0 && green > 0 && blue == 0) return "GREEN";
    if (red > 0 && green > 0 && blue == 0) return "ORANGE";
    if (red == 0 && green == 0 && blue > 0) return "BLUE";
    return "CUSTOM";
}
#endif

// ============================================================================
// WS2812 RMT Driver
// ============================================================================

static rmt_channel_handle_t s_rmt_channel = nullptr;
static rmt_encoder_handle_t s_rmt_encoder = nullptr;

// WS2812 timing at 10 MHz RMT clock (0.1 us per tick)
constexpr uint32_t WS2812_RESOLUTION_HZ = 10 * 1000 * 1000;
constexpr uint16_t WS2812_T0H = 4;   // 0.4us high for '0' bit
constexpr uint16_t WS2812_T0L = 8;   // 0.8us low for '0' bit
constexpr uint16_t WS2812_T1H = 7;   // 0.7us high for '1' bit
constexpr uint16_t WS2812_T1L = 6;   // 0.6us low for '1' bit

// ============================================================================
// WS2812 Internal Functions
// ============================================================================

// Initialize RMT peripheral for WS2812 single-wire protocol
static esp_err_t ws2812_init(gpio_num_t gpio) {
    rmt_tx_channel_config_t tx_cfg = {};
    tx_cfg.gpio_num = gpio;
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
    if (err != ESP_OK) return err;

    rmt_copy_encoder_config_t encoder_cfg = {};
    err = rmt_new_copy_encoder(&encoder_cfg, &s_rmt_encoder);
    if (err != ESP_OK) {
        rmt_del_channel(s_rmt_channel);
        s_rmt_channel = nullptr;
        return err;
    }

    err = rmt_enable(s_rmt_channel);
    if (err != ESP_OK) {
        rmt_del_encoder(s_rmt_encoder);
        s_rmt_encoder = nullptr;
        rmt_del_channel(s_rmt_channel);
        s_rmt_channel = nullptr;
        return err;
    }
    return ESP_OK;
}

// Transmit 24-bit color to WS2812 LED (GRB byte order)
static esp_err_t ws2812_set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
    if (!s_rmt_channel || !s_rmt_encoder) return ESP_ERR_INVALID_STATE;

    // Build RMT symbols for each bit - WS2812 uses pulse width encoding
    rmt_symbol_word_t symbols[24];
    rmt_symbol_word_t one = {.duration0 = WS2812_T1H, .level0 = 1, .duration1 = WS2812_T1L, .level1 = 0};
    rmt_symbol_word_t zero = {.duration0 = WS2812_T0H, .level0 = 1, .duration1 = WS2812_T0L, .level1 = 0};

    // WS2812 expects GRB order, MSB first
    int idx = 0;
    uint8_t grb[3] = {green, red, blue};
    for (int c = 0; c < 3; ++c) 
        for (int bit = 7; bit >= 0; --bit)
        	symbols[idx++] = (grb[c] & (1 << bit)) ? one : zero;

    rmt_transmit_config_t tx_cfg = {};
    tx_cfg.loop_count = 0;
    tx_cfg.flags.eot_level = 0;

    esp_err_t err = rmt_transmit(s_rmt_channel, s_rmt_encoder, symbols, sizeof(symbols), &tx_cfg);
    if (err != ESP_OK) return err;
    return ESP_OK;
}

}  // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t led_ws2812_init(const led_ws2812_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized, skipping re-init");
        return ESP_OK;
    }

    // Store color configuration
    s_idle_red = config->idle_red;
    s_idle_green = config->idle_green;
    s_idle_blue = config->idle_blue;
    s_activity_red = config->activity_red;
    s_activity_green = config->activity_green;
    s_activity_blue = config->activity_blue;
    s_error_red = config->error_red;
    s_error_green = config->error_green;
    s_error_blue = config->error_blue;

    // Initialize WS2812 via RMT peripheral
    esp_err_t err = ws2812_init(config->gpio);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WS2812 init failed: %s", esp_err_to_name(err));
        return err;
    }

    ws2812_set_rgb(0, 0, 0);

    s_last_update = xTaskGetTickCount();
    s_last_activity = s_last_update;
    s_activity_window = config->activity_window_ticks;
    s_initialized = true;

    return ESP_OK;
}

// ============================================================================
// Runtime Control
// ============================================================================

void led_ws2812_tick(const led_ws2812_config_t *config, TickType_t now_ticks) {
    if (!config || !s_initialized) return;
    if (now_ticks - s_last_update < config->interval_ticks) return;

    bool activity = s_activity_window > 0 && (now_ticks - s_last_activity <= s_activity_window);

    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    const char *reason = "idle";

    if (s_manual_mode) {
        red = s_manual_red;
        green = s_manual_green;
        blue = s_manual_blue;
        reason = s_manual_reason;
    } else if (s_error) {
        red = s_error_red;
        green = s_error_green;
        blue = s_error_blue;
        reason = "error";
    } else if (activity) {
        red = s_activity_red;
        green = s_activity_green;
        blue = s_activity_blue;
        reason = "activity";
    } else {
        red = s_idle_red;
        green = s_idle_green;
        blue = s_idle_blue;
        reason = "idle";
    }

    ws2812_set_rgb(red, green, blue);

#ifdef CONFIG_LOG_HEARTBEAT_LED_COLOR_UPDATES
    ESP_LOGI(TAG, "LED: color=%s rgb=(%u,%u,%u) reason=%s",
             led_color_name(red, green, blue),
             red, green, blue, reason);
#else
    (void)reason;
#endif

    s_last_update = now_ticks;
}

// Record CAN activity - triggers activity color for activity_window duration
void led_ws2812_mark_activity(TickType_t now_ticks) {
    if (!s_initialized) return;
    s_last_activity = now_ticks;
}

// Set error state - shows error color until cleared
void led_ws2812_set_error(bool active) {
    if (!s_initialized) return;
    s_error = active;
}

void led_ws2812_set_manual_mode(bool enabled) {
    if (!s_initialized) return;
    if (s_manual_mode == enabled) return;
    s_manual_mode = enabled;

#ifdef CONFIG_LOG_HEARTBEAT_LED_STATE_CHANGES
    ESP_LOGI(TAG, "Manual mode %s", enabled ? "ENABLED" : "DISABLED");
#endif
}

void led_ws2812_set_manual_color(uint8_t red,
                                 uint8_t green,
                                 uint8_t blue,
                                 const char *reason) {
    if (!s_initialized) return;

    const char *new_reason = (reason && reason[0] != '\0') ? reason : "manual";

#ifdef CONFIG_LOG_HEARTBEAT_LED_STATE_CHANGES
    bool changed = (s_manual_red != red) ||
                   (s_manual_green != green) ||
                   (s_manual_blue != blue) ||
                   (strncmp(s_manual_reason, new_reason, sizeof(s_manual_reason)) != 0);
#endif

    s_manual_red = red;
    s_manual_green = green;
    s_manual_blue = blue;
    strncpy(s_manual_reason, new_reason, sizeof(s_manual_reason) - 1);
    s_manual_reason[sizeof(s_manual_reason) - 1] = '\0';

#ifdef CONFIG_LOG_HEARTBEAT_LED_STATE_CHANGES
    if (changed) {
        ESP_LOGI(TAG, "Mode: color=%s rgb=(%u,%u,%u) reason=%s",
                 led_color_name(red, green, blue),
                 red, green, blue, s_manual_reason);
    }
#endif
}
