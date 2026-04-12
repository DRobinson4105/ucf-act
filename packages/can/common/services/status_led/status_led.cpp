/**
 * @file status_led.cpp
 * @brief State-driven status LED adapter above the raw WS2812 transport.
 */

#include "status_led.h"

#include "led_ws2812.h"

namespace
{

constexpr uint8_t STATUS_LED_BRIGHTNESS = 16;

bool s_initialized = false;
node_state_t s_state = NODE_STATE_INIT;
bool s_color_valid = false;
led_ws2812_rgb_t s_last_color = {0, 0, 0};

led_ws2812_rgb_t state_to_rgb(node_state_t state)
{
	switch (state)
	{
	case NODE_STATE_READY:
		return {0, STATUS_LED_BRIGHTNESS, 0};
	case NODE_STATE_ACTIVE:
		return {0, 0, STATUS_LED_BRIGHTNESS};
	case NODE_STATE_NOT_READY:
		return {STATUS_LED_BRIGHTNESS, 0, 0};
	case NODE_STATE_INIT:
	case NODE_STATE_ENABLE:
	default:
		return {STATUS_LED_BRIGHTNESS, STATUS_LED_BRIGHTNESS, 0};
	}
}

void apply_visible_color(void)
{
	if (!s_initialized)
		return;

	led_ws2812_rgb_t color = state_to_rgb(s_state);
	if (s_color_valid && color.r == s_last_color.r && color.g == s_last_color.g && color.b == s_last_color.b)
		return;

	led_ws2812_set_rgb(color);
	s_last_color = color;
	s_color_valid = true;
}

} // namespace

esp_err_t status_led_init(void)
{
	esp_err_t err = led_ws2812_init();
	if (err != ESP_OK)
		return err;

	s_initialized = true;
	s_state = NODE_STATE_INIT;
	s_color_valid = false;
	return ESP_OK;
}

void status_led_set_state(node_state_t state)
{
	s_state = state;
	apply_visible_color();
}
