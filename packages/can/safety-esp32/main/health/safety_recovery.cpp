/**
 * @file safety_recovery.cpp
 * @brief Safety-local component retry and CAN recovery helpers.
 */

#include "safety_recovery.h"

#include "can_runtime.h"
#include "esp_log.h"
#include "node_support.h"

namespace
{

static inline bool should_log_retry(uint16_t retry_count, uint16_t retry_log_every_n)
{
	return retry_count == 1 || (retry_count % retry_log_every_n) == 0;
}

static inline const char *recovery_verbose_tag(const char *tag, uint16_t retry_count, uint16_t retry_log_every_n)
{
#ifdef CONFIG_LOG_HEALTH_CAN_RECOVERY
	return (retry_count == 0 || ((retry_count + 1) % retry_log_every_n) == 0) ? tag : nullptr;
#else
	(void)tag;
	(void)retry_count;
	(void)retry_log_every_n;
	return nullptr;
#endif
}

} // namespace

void safety_retry_failed_components(safety_recovery_context_t *ctx)
{
	if (!ctx || !ctx->last_retry_ms || !ctx->can_tx_lock || !ctx->can_recovery_in_progress || !ctx->twai_ready ||
	    !ctx->quiesce_can_rx || !ctx->push_button_cfg || !ctx->push_button_init_ok || !ctx->rf_remote_cfg ||
	    !ctx->rf_remote_init_ok || !ctx->relay_cfg || !ctx->relay_init_ok || !ctx->ultrasonic_cfg ||
	    !ctx->ultrasonic_init_ok || !ctx->ultrasonic_init_tick || !ctx->local_inputs || !ctx->local_inputs_cfg ||
	    !ctx->battery_cfg || !ctx->battery_monitor_init_ok)
	{
		return;
	}

	uint32_t now_ms = node_get_time_ms();
	if ((now_ms - *ctx->last_retry_ms) < ctx->retry_interval_ms)
		return;
	*ctx->last_retry_ms = now_ms;

#ifndef CONFIG_BYPASS_INPUT_PUSH_BUTTON
	if (!*ctx->push_button_init_ok)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = estop_input_init(ctx->push_button_cfg);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_HEALTH_RETRY_COMPONENTS
		if (!recovered)
		{
			s_retry_count++;
			if (should_log_retry(s_retry_count, ctx->retry_log_every_n))
				ESP_LOGW(ctx->tag, "PUSH_BUTTON retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			*ctx->push_button_init_ok = true;
			node_log_component_regained(ctx->tag, "PUSH_BUTTON");
		}
	}
#endif

#ifndef CONFIG_BYPASS_INPUT_RF_REMOTE
	if (!*ctx->rf_remote_init_ok)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = estop_input_init(ctx->rf_remote_cfg);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_HEALTH_RETRY_COMPONENTS
		if (!recovered)
		{
			s_retry_count++;
			if (should_log_retry(s_retry_count, ctx->retry_log_every_n))
				ESP_LOGW(ctx->tag, "RF_REMOTE retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			*ctx->rf_remote_init_ok = true;
			node_log_component_regained(ctx->tag, "RF_REMOTE");
		}
	}
#endif

	if (!*ctx->relay_init_ok)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = relay_srd05vdc_init(ctx->relay_cfg);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_HEALTH_RETRY_COMPONENTS
		if (!recovered)
		{
			s_retry_count++;
			if (should_log_retry(s_retry_count, ctx->retry_log_every_n))
				ESP_LOGW(ctx->tag, "RELAY retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			*ctx->relay_init_ok = true;
			node_log_component_regained(ctx->tag, "RELAY (driver-level)");
		}
	}

#ifndef CONFIG_BYPASS_INPUT_ULTRASONIC
	if (!*ctx->ultrasonic_init_ok)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = ultrasonic_a02yyuw_init(ctx->ultrasonic_cfg);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_HEALTH_RETRY_COMPONENTS
		if (!recovered)
		{
			s_retry_count++;
			if (should_log_retry(s_retry_count, ctx->retry_log_every_n))
				ESP_LOGW(ctx->tag, "ULTRASONIC retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			*ctx->ultrasonic_init_ok = true;
			safety_local_inputs_reset_ultrasonic(ctx->local_inputs, ctx->local_inputs_cfg);
			*ctx->ultrasonic_init_tick = xTaskGetTickCount();
		}
	}
#endif

#ifndef CONFIG_BYPASS_INPUT_BATTERY_MONITOR
	if (!*ctx->battery_monitor_init_ok)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = battery_monitor_init(ctx->battery_cfg);
		bool recovered = (err == ESP_OK);
		if (!recovered)
		{
			s_retry_count++;
			if (should_log_retry(s_retry_count, ctx->retry_log_every_n))
			{
				ESP_LOGW(ctx->tag, "BATTERY_MONITOR retry failed (%u attempts): %s", s_retry_count,
				         esp_err_to_name(err));
			}
		}
		if (recovered)
		{
			s_retry_count = 0;
			*ctx->battery_monitor_init_ok = true;
			node_log_component_regained(ctx->tag, "BATTERY_MONITOR");
		}
	}
#endif

#ifndef CONFIG_BYPASS_CAN_TWAI
	if (!*ctx->twai_ready)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		if (!can_runtime_try_begin_recovery(ctx->can_tx_lock, ctx->can_recovery_in_progress))
			return;

		ctx->quiesce_can_rx();

		can_runtime_recovery_probe_t probe = {
			.tx_gpio = ctx->twai_tx_gpio,
			.rx_gpio = ctx->twai_rx_gpio,
			.log_tag = recovery_verbose_tag(ctx->tag, s_retry_count, ctx->retry_log_every_n),
			.probe_can_id = CAN_ID_SAFETY_HEARTBEAT,
			.probe_heartbeat =
				{
					.sequence = 0,
					.state = ctx->probe_state,
					.fault_flags = ctx->probe_fault_flags,
					.status_flags = 0,
					.stop_flags = ctx->probe_stop_flags,
					.soc_pct = ctx->probe_soc_pct,
				},
			.probe_send_timeout = pdMS_TO_TICKS(50),
			.probe_settle_delay = pdMS_TO_TICKS(50),
		};

		esp_err_t err = ESP_FAIL;
		bool recovered = can_runtime_recover_with_probe(&probe, &err);
#ifdef CONFIG_LOG_HEALTH_RETRY_TWAI
		if (!recovered)
		{
			s_retry_count++;
			if (should_log_retry(s_retry_count, ctx->retry_log_every_n))
			{
				if (err != ESP_OK)
					ESP_LOGW(ctx->tag, "TWAI retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
				else
					ESP_LOGW(ctx->tag, "TWAI retry failed (%u attempts): bus unstable after reinit", s_retry_count);
			}
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			*ctx->twai_ready = true;
		}

		can_runtime_finish_recovery(ctx->can_tx_lock, ctx->can_recovery_in_progress);
	}
#endif
}
