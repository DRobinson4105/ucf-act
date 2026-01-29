#include "heartbeat.hh"

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"

namespace {
static const char *kTag = "HEARTBEAT";

// heartbeat state
static bool s_initialized = false;
static bool s_state = false;
static TickType_t s_last_toggle = 0;
static TickType_t s_last_activity = 0;
static TickType_t s_activity_window = 0;
static bool s_error = false;

// ws2812 driver state
static bool s_use_ws2812 = false;
static uint8_t s_idle_red = 0;
static uint8_t s_idle_green = 0;
static uint8_t s_idle_blue = 0;
static uint8_t s_activity_red = 0;
static uint8_t s_activity_green = 0;
static uint8_t s_activity_blue = 0;
static uint8_t s_error_red = 0;
static uint8_t s_error_green = 0;
static uint8_t s_error_blue = 0;
static rmt_channel_handle_t s_rmt_channel = nullptr;
static rmt_encoder_handle_t s_rmt_encoder = nullptr;

// ws2812 timing (10 MHz resolution -> 0.1 us per tick)
constexpr uint32_t kWs2812ResolutionHz = 10 * 1000 * 1000;
constexpr uint16_t kWs2812T0H = 4;
constexpr uint16_t kWs2812T0L = 8;
constexpr uint16_t kWs2812T1H = 7;
constexpr uint16_t kWs2812T1L = 6;

// initialize RMT for WS2812 output
static esp_err_t ws2812_init(gpio_num_t gpio) {
    rmt_tx_channel_config_t tx_cfg = {};
    tx_cfg.gpio_num = gpio;
    tx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz = kWs2812ResolutionHz;
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
    if (err != ESP_OK) return err;

    return rmt_enable(s_rmt_channel);
}

// send a single RGB value (GRB order for WS2812)
static esp_err_t ws2812_set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
    if (!s_rmt_channel || !s_rmt_encoder) return ESP_ERR_INVALID_STATE;

    rmt_symbol_word_t symbols[24];
    rmt_symbol_word_t one = {};
    one.duration0 = kWs2812T1H;
    one.level0 = 1;
    one.duration1 = kWs2812T1L;
    one.level1 = 0;

    rmt_symbol_word_t zero = {};
    zero.duration0 = kWs2812T0H;
    zero.level0 = 1;
    zero.duration1 = kWs2812T0L;
    zero.level1 = 0;

    int idx = 0;
    uint8_t grb[3] = {green, red, blue};
    for (int c = 0; c < 3; ++c) {
        for (int bit = 7; bit >= 0; --bit) {
            symbols[idx++] = (grb[c] & (1 << bit)) ? one : zero;
        }
    }

    rmt_transmit_config_t tx_cfg = {};
    tx_cfg.loop_count = 0;
    tx_cfg.flags.eot_level = 0;

    esp_err_t err = rmt_transmit(s_rmt_channel, s_rmt_encoder, symbols, sizeof(symbols), &tx_cfg);
    if (err != ESP_OK) return err;
    return ESP_OK;
}
}

// initialize heartbeat GPIO or WS2812 output
esp_err_t heartbeat_init(const heartbeat_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    s_use_ws2812 = config->use_ws2812;
    s_idle_red = config->idle_red;
    s_idle_green = config->idle_green;
    s_idle_blue = config->idle_blue;
    s_activity_red = config->activity_red;
    s_activity_green = config->activity_green;
    s_activity_blue = config->activity_blue;
    s_error_red = config->error_red;
    s_error_green = config->error_green;
    s_error_blue = config->error_blue;

    esp_err_t err = ESP_OK;
    if (s_use_ws2812) {
        err = ws2812_init(config->ws2812_gpio);
        if (err != ESP_OK) {
            s_use_ws2812 = false;
        }
    }

    if (!s_use_ws2812) {
        gpio_reset_pin(config->gpio);
        err = gpio_set_direction(config->gpio, GPIO_MODE_OUTPUT);
        if (err != ESP_OK) return err;
        gpio_set_pull_mode(config->gpio, GPIO_FLOATING);
        gpio_set_level(config->gpio, config->active_high ? 0 : 1);
    } else {
        ws2812_set_rgb(0, 0, 0);
    }

    s_state = false;
    s_last_toggle = xTaskGetTickCount();
    s_last_activity = s_last_toggle;
    s_activity_window = config->activity_window_ticks;
    s_initialized = true;
    const char *label = config->label ? config->label : "device";
    ESP_LOGI(kTag, "heartbeat init %s gpio %d", label, config->gpio);
    return ESP_OK;
}

// update heartbeat state and toggle LED when interval elapses
void heartbeat_tick(const heartbeat_config_t *config, TickType_t now_ticks) {
    if (!config || !s_initialized) return;

    if (now_ticks - s_last_toggle < config->interval_ticks) return;

    TickType_t elapsed = now_ticks - s_last_toggle;
    bool activity = s_activity_window > 0 && (now_ticks - s_last_activity <= s_activity_window);
    s_state = !s_state;
    if (s_use_ws2812) {
        if (s_state) {
            if (s_error) {
                ws2812_set_rgb(s_error_red, s_error_green, s_error_blue);
            } else if (activity) {
                ws2812_set_rgb(s_activity_red, s_activity_green, s_activity_blue);
            } else {
                ws2812_set_rgb(s_idle_red, s_idle_green, s_idle_blue);
            }
        } else {
            ws2812_set_rgb(0, 0, 0);
        }
    } else {
        gpio_set_level(config->gpio, config->active_high ? (s_state ? 1 : 0) : (s_state ? 0 : 1));
    }

    const char *label = config->label ? config->label : "device";
    const char *status = s_error ? "error" : (activity ? "activity" : "idle");
    ESP_LOGI(kTag, "heartbeat %s %s dt=%lu ms", label, status, (unsigned long)(elapsed * portTICK_PERIOD_MS));
    s_last_toggle = now_ticks;
}

// mark recent activity so heartbeat can show activity state
void heartbeat_mark_activity(TickType_t now_ticks) {
    if (!s_initialized) return;
    s_last_activity = now_ticks;
}

// set or clear an error state for heartbeat indication
void heartbeat_set_error(bool active) {
    if (!s_initialized) return;
    s_error = active;
}
