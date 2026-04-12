/**
 * @file control_startup_log.cpp
 * @brief Startup device status logging helpers for Control ESP32.
 */

#include "control_startup_log.h"

#include "esp_log.h"

void control_log_startup_device_status(const char *tag, bool twai_ready, gpio_num_t twai_tx_gpio, gpio_num_t twai_rx_gpio,
                                       const dac_mcp4728_config_t *dac_cfg, bool dac_ready,
                                       const relay_dpdt_my5nj_config_t *relay_cfg, bool relay_ready,
                                       const adc_12bitsar_config_t *pedal_cfg, unsigned pedal_threshold_mv,
                                       unsigned initial_pedal_mv, bool pedal_ready, bool pedal_calibrated,
                                       gpio_num_t fr_forward_gpio, gpio_num_t fr_reverse_gpio, bool fr_ready,
                                       unsigned steering_node_id, bool steering_ready, unsigned braking_node_id,
                                       bool braking_ready, bool heartbeat_ready, bool orin_link_ready)
{
	if (twai_ready)
		ESP_LOGI(tag, "TWAI: CONFIGURED: tx_gpio=%d rx_gpio=%d", twai_tx_gpio, twai_rx_gpio);
	else
		ESP_LOGE(tag, "TWAI: FAILED: tx_gpio=%d rx_gpio=%d", twai_tx_gpio, twai_rx_gpio);

#ifdef CONFIG_BYPASS_ACTUATOR_THROTTLE
	ESP_LOGW(tag, "DAC: BYPASSED: disabled (sda=%d scl=%d)", dac_cfg->sda, dac_cfg->scl);
	ESP_LOGW(tag, "DPDT_RELAY: BYPASSED: disabled (gpio=%d)", relay_cfg->gpio);
#else
	if (dac_ready)
		ESP_LOGI(tag, "DAC: CONFIGURED: sda=%d scl=%d addr=0x%02X start=LEVEL_0", dac_cfg->sda,
		         dac_cfg->scl, dac_cfg->device_addr);
	else
		ESP_LOGE(tag, "DAC: FAILED: sda=%d scl=%d addr=0x%02X", dac_cfg->sda, dac_cfg->scl, dac_cfg->device_addr);

	if (relay_ready)
		ESP_LOGI(tag, "DPDT_RELAY: CONFIGURED: gpio=%d active_level=HIGH start=DE-ENERGIZED", relay_cfg->gpio);
	else
		ESP_LOGE(tag, "DPDT_RELAY: FAILED: gpio=%d active_level=HIGH", relay_cfg->gpio);
#endif

#ifdef CONFIG_BYPASS_INPUT_PEDAL_ADC
	ESP_LOGW(tag, "PEDAL_INPUT: BYPASSED: pedal override disabled (forced released, threshold_mv=%u)", pedal_threshold_mv);
#else
	if (pedal_ready)
	{
		ESP_LOGI(tag, "PEDAL_INPUT: CONFIGURED: adc_channel=ADC%d_CH%d adc_mode=%s threshold_mv=%u initial_mv=%u",
		         pedal_cfg->adc_unit + 1, pedal_cfg->adc_channel, pedal_calibrated ? "curve_fit" : "raw",
		         pedal_threshold_mv, initial_pedal_mv);
	}
	else
	{
		ESP_LOGE(tag, "PEDAL_INPUT: FAILED: adc_channel=ADC%d_CH%d threshold_mv=%u", pedal_cfg->adc_unit + 1,
		         pedal_cfg->adc_channel, pedal_threshold_mv);
	}
#endif

#ifdef CONFIG_BYPASS_INPUT_FR_SENSOR
	ESP_LOGW(tag, "FR_INPUT_FORWARD: BYPASSED: input forced (input_gpio=%d)", fr_forward_gpio);
	ESP_LOGW(tag, "FR_INPUT_REVERSE: BYPASSED: input forced (input_gpio=%d)", fr_reverse_gpio);
#else
	if (fr_ready)
	{
		ESP_LOGI(tag, "FR_INPUT_FORWARD: OK: input_gpio=%d", fr_forward_gpio);
		ESP_LOGI(tag, "FR_INPUT_REVERSE: OK: input_gpio=%d", fr_reverse_gpio);
	}
	else
	{
		ESP_LOGE(tag, "FR_INPUT_FORWARD: FAILED: input_gpio=%d", fr_forward_gpio);
		ESP_LOGE(tag, "FR_INPUT_REVERSE: FAILED: input_gpio=%d", fr_reverse_gpio);
	}
#endif

#ifdef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
	ESP_LOGW(tag, "MOTOR_STEERING: BYPASSED: actuator disabled (node_id=%u)", steering_node_id);
#else
	if (steering_ready)
		ESP_LOGI(tag, "MOTOR_STEERING: OK: node_id=%u init_check=passed", steering_node_id);
	else
		ESP_LOGE(tag, "MOTOR_STEERING: FAILED: node_id=%u init_check=failed", steering_node_id);
#endif

#ifdef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
	ESP_LOGW(tag, "MOTOR_BRAKING: BYPASSED: actuator disabled (node_id=%u)", braking_node_id);
#else
	if (braking_ready)
		ESP_LOGI(tag, "MOTOR_BRAKING: OK: node_id=%u init_check=passed", braking_node_id);
	else
		ESP_LOGE(tag, "MOTOR_BRAKING: FAILED: node_id=%u init_check=failed", braking_node_id);
#endif

	if (heartbeat_ready)
		ESP_LOGI(tag, "HEARTBEAT_LED: CONFIGURED");
	else
		ESP_LOGW(tag, "HEARTBEAT_LED: UNAVAILABLE (non-critical)");

	if (orin_link_ready)
		ESP_LOGI(tag, "ORIN_USB_LINK: CONFIGURED: rx=planner_command tx=control_heartbeat");
	else
		ESP_LOGE(tag, "ORIN_USB_LINK: FAILED");
}
