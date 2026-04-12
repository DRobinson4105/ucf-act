/**
 * @file safety_startup_log.cpp
 * @brief Startup device status logging helpers for Safety ESP32.
 */

#include "safety_startup_log.h"

#include "esp_log.h"

void safety_log_startup_device_status(const char *tag, bool twai_ready, gpio_num_t twai_tx_gpio, gpio_num_t twai_rx_gpio,
                                      int push_button_gpio, int push_button_active_level, bool push_button_ready,
                                      int rf_remote_gpio, int rf_remote_active_level, bool rf_remote_ready,
                                      const ultrasonic_a02yyuw_config_t *ultrasonic_cfg, bool ultrasonic_ready,
                                      const relay_srd05vdc_config_t *relay_cfg, bool relay_ready, int battery_voltage_gpio,
                                      int battery_current_gpio, bool battery_monitor_ready, bool heartbeat_ready,
                                      bool orin_link_ready)
{
#ifdef CONFIG_BYPASS_CAN_TWAI
	ESP_LOGW(tag, "TWAI: BYPASSED: disabled");
#else
	if (twai_ready)
		ESP_LOGI(tag, "TWAI: CONFIGURED: tx_gpio=%d rx_gpio=%d", twai_tx_gpio, twai_rx_gpio);
	else
		ESP_LOGE(tag, "TWAI: FAILED: tx_gpio=%d rx_gpio=%d", twai_tx_gpio, twai_rx_gpio);
#endif

#ifdef CONFIG_BYPASS_INPUT_PUSH_BUTTON
	ESP_LOGW(tag, "PUSH_BUTTON: BYPASSED: disabled (input_gpio=%d active_level=%s)", push_button_gpio,
	         push_button_active_level ? "HIGH" : "LOW");
#else
	if (push_button_ready)
		ESP_LOGI(tag, "PUSH_BUTTON: CONFIGURED: input_gpio=%d active_level=%s", push_button_gpio,
		         push_button_active_level ? "HIGH" : "LOW");
	else
		ESP_LOGE(tag, "PUSH_BUTTON: FAILED: input_gpio=%d", push_button_gpio);
#endif

#ifdef CONFIG_BYPASS_INPUT_RF_REMOTE
	ESP_LOGW(tag, "RF_REMOTE: BYPASSED: disabled (input_gpio=%d active_level=%s)", rf_remote_gpio,
	         rf_remote_active_level ? "HIGH" : "LOW");
#else
	if (rf_remote_ready)
		ESP_LOGI(tag, "RF_REMOTE: CONFIGURED: input_gpio=%d active_level=%s", rf_remote_gpio,
		         rf_remote_active_level ? "HIGH" : "LOW");
	else
		ESP_LOGE(tag, "RF_REMOTE: FAILED: input_gpio=%d", rf_remote_gpio);
#endif

#ifdef CONFIG_BYPASS_INPUT_ULTRASONIC
	ESP_LOGW(tag, "ULTRASONIC: BYPASSED: disabled (uart=%d tx_gpio=%d rx_gpio=%d)", ultrasonic_cfg->uart_num,
	         ultrasonic_cfg->tx_gpio, ultrasonic_cfg->rx_gpio);
#else
	if (ultrasonic_ready)
		ESP_LOGI(tag, "ULTRASONIC: CONFIGURED: uart=%d tx_gpio=%d rx_gpio=%d baud=%d", ultrasonic_cfg->uart_num,
		         ultrasonic_cfg->tx_gpio, ultrasonic_cfg->rx_gpio, ultrasonic_cfg->baud_rate);
	else
		ESP_LOGE(tag, "ULTRASONIC: FAILED: uart=%d", ultrasonic_cfg->uart_num);
#endif

	if (relay_ready)
		ESP_LOGI(tag, "RELAY: CONFIGURED: gpio=%d active_level=HIGH start=DISABLED", relay_cfg->gpio);
	else
		ESP_LOGE(tag, "RELAY: FAILED: gpio=%d", relay_cfg->gpio);

#ifdef CONFIG_BYPASS_INPUT_BATTERY_MONITOR
	ESP_LOGW(tag, "BATTERY_MONITOR: BYPASSED: disabled (voltage_gpio=%d current_gpio=%d)", battery_voltage_gpio,
	         battery_current_gpio);
#else
	if (battery_monitor_ready)
		ESP_LOGI(tag, "BATTERY_MONITOR: CONFIGURED: voltage_gpio=%d current_gpio=%d", battery_voltage_gpio,
		         battery_current_gpio);
	else
		ESP_LOGE(tag, "BATTERY_MONITOR: FAILED: voltage_gpio=%d current_gpio=%d", battery_voltage_gpio,
		         battery_current_gpio);
#endif

#ifdef CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS
	ESP_LOGW(tag, "PLANNER: BYPASSED: liveness checks disabled (alive forced true, faults/stops cleared)");
#endif
#ifdef CONFIG_BYPASS_PLANNER_STATE_MIRROR
	ESP_LOGW(tag, "PLANNER: BYPASSED: state mirror active (planner state simulated)");
#endif
#ifdef CONFIG_BYPASS_PLANNER_AUTONOMY_GATE
	ESP_LOGW(tag, "PLANNER: BYPASSED: autonomy gate disabled (request/hold forced true)");
#endif

#ifdef CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS
	ESP_LOGW(tag, "CONTROL: BYPASSED: liveness checks disabled (alive forced true, faults/stops cleared)");
#endif
#ifdef CONFIG_BYPASS_CONTROL_STATE_MIRROR
	ESP_LOGW(tag, "CONTROL: BYPASSED: state mirror active (control state simulated)");
#endif

	if (heartbeat_ready)
		ESP_LOGI(tag, "HEARTBEAT_LED: CONFIGURED");
	else
		ESP_LOGW(tag, "HEARTBEAT_LED: UNAVAILABLE (non-critical)");

	if (orin_link_ready)
		ESP_LOGI(tag, "ORIN_USB_LINK: CONFIGURED: rx=planner_heartbeat tx=safety_heartbeat");
	else
		ESP_LOGE(tag, "ORIN_USB_LINK: FAILED");
}
