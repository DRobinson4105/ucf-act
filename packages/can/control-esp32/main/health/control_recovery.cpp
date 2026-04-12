/**
 * @file control_recovery.cpp
 * @brief Control-local component retry and CAN recovery helpers.
 */

#include "control_recovery.h"

#include <stdio.h>

#include "can_runtime.h"
#include "control_globals.h"
#include "control_health.h"
#include "esp_log.h"
#include "motor_dispatch.h"
#include "motor_protocol.h"
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

void control_retry_failed_components(control_recovery_context_t *ctx)
{
	if (!ctx || !ctx->last_retry_ms || !ctx->can_tx_lock || !ctx->can_recovery_in_progress || !ctx->twai_ready ||
	    !ctx->quiesce_can_rx || !ctx->driver_inputs || !ctx->driver_inputs_cfg || !ctx->fr_pc817_cfg ||
	    !ctx->motor_uim2852_braking_ready || !ctx->motor_uim2852_steering_ready)
	{
		return;
	}

	uint32_t now_ms = node_get_time_ms();
	if ((now_ms - *ctx->last_retry_ms) < ctx->retry_interval_ms)
		return;
	*ctx->last_retry_ms = now_ms;

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
			.probe_can_id = CAN_ID_CONTROL_HEARTBEAT,
			.probe_heartbeat =
				{
					.sequence = 0,
					.state = ctx->probe_state,
					.fault_flags = ctx->probe_fault_flags,
					.status_flags = ctx->probe_status_flags,
					.stop_flags = ctx->probe_stop_flags,
					.soc_pct = 0,
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

#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	if (ctx->dac_ready && !*ctx->dac_ready)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = dac_mcp4728_init(ctx->dac_cfg);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_HEALTH_RETRY_COMPONENTS
		if (!recovered)
		{
			s_retry_count++;
			if (should_log_retry(s_retry_count, ctx->retry_log_every_n))
				ESP_LOGW(ctx->tag, "DAC retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			node_log_component_regained(ctx->tag, "DAC (driver-level)");
		}
		*ctx->dac_ready = recovered;
	}

	if (ctx->dpdt_relay_ready && !*ctx->dpdt_relay_ready)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = relay_dpdt_my5nj_init(ctx->dpdt_relay_cfg);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_HEALTH_RETRY_COMPONENTS
		if (!recovered)
		{
			s_retry_count++;
			if (should_log_retry(s_retry_count, ctx->retry_log_every_n))
				ESP_LOGW(ctx->tag, "DPDT_RELAY retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			node_log_component_regained(ctx->tag, "DPDT_RELAY (driver-level)");
		}
		*ctx->dpdt_relay_ready = recovered;
	}
#endif

#ifndef CONFIG_BYPASS_INPUT_PEDAL_ADC
	if (!ctx->driver_inputs->pedal_ready)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = control_driver_inputs_init_pedal(ctx->driver_inputs, ctx->driver_inputs_cfg);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_HEALTH_RETRY_COMPONENTS
		if (!recovered)
		{
			s_retry_count++;
			if (should_log_retry(s_retry_count, ctx->retry_log_every_n))
				ESP_LOGW(ctx->tag, "PEDAL_INPUT retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			node_log_component_regained(ctx->tag, "PEDAL_INPUT");
		}
	}
#endif

#ifndef CONFIG_BYPASS_INPUT_FR_SENSOR
	if (!ctx->driver_inputs->fr_ready)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = control_driver_inputs_init_fr(ctx->driver_inputs, ctx->driver_inputs_cfg);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_HEALTH_RETRY_COMPONENTS
		if (!recovered)
		{
			s_retry_count++;
			if (should_log_retry(s_retry_count, ctx->retry_log_every_n))
			{
				ESP_LOGW(ctx->tag, "FR_INPUT_FORWARD retry failed (%u attempts): %s (input_gpio=%d)", s_retry_count,
				         esp_err_to_name(err), ctx->fr_pc817_cfg->forward_gpio);
				ESP_LOGW(ctx->tag, "FR_INPUT_REVERSE retry failed (%u attempts): %s (input_gpio=%d)", s_retry_count,
				         esp_err_to_name(err), ctx->fr_pc817_cfg->reverse_gpio);
			}
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			fr_state_t recovered_state = control_driver_inputs_fr_state(ctx->driver_inputs);
			char fr_regained[48];
			snprintf(fr_regained, sizeof(fr_regained), "FR_INPUT (fr=%s)", fr_state_to_string(recovered_state));
			node_log_component_regained(ctx->tag, fr_regained);
		}
	}
#endif

	[[maybe_unused]] bool braking_needs_limits = false;
	[[maybe_unused]] bool steering_needs_limits = false;

#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
	if (!*ctx->motor_uim2852_braking_ready)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		motor_cmd_t cmd;
		// Send MO=0 to verify motor is reachable
		motor_cmd_mo_set(MOTOR_NODE_BRAKING, true, MOTOR_MO_STATE_DISABLE, &cmd);
		motor_dispatch_result_t res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(200), pdMS_TO_TICKS(50),
		                                                  nullptr, nullptr, nullptr);
		bool init_ok = (res == MOTOR_DISPATCH_RESULT_OK);
#ifdef CONFIG_LOG_HEALTH_RETRY_COMPONENTS
		if (!init_ok)
		{
			s_retry_count++;
			if (should_log_retry(s_retry_count, ctx->retry_log_every_n))
			{
				ESP_LOGW(ctx->tag, "MOTOR_BRAKING retry failed (%u attempts): dispatch result %d", s_retry_count,
				         (int)res);
			}
		}
#endif
		if (init_ok)
			s_retry_count = 0;
		braking_needs_limits = init_ok;
	}
#endif

#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
	if (!*ctx->motor_uim2852_steering_ready)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		motor_cmd_t cmd;
		// Send MO=0 to verify motor is reachable
		motor_cmd_mo_set(MOTOR_NODE_STEERING, true, MOTOR_MO_STATE_DISABLE, &cmd);
		motor_dispatch_result_t res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(200), pdMS_TO_TICKS(50),
		                                                  nullptr, nullptr, nullptr);
		bool init_ok = (res == MOTOR_DISPATCH_RESULT_OK);
#ifdef CONFIG_LOG_HEALTH_RETRY_COMPONENTS
		if (!init_ok)
		{
			s_retry_count++;
			if (should_log_retry(s_retry_count, ctx->retry_log_every_n))
			{
				ESP_LOGW(ctx->tag, "MOTOR_STEERING retry failed (%u attempts): dispatch result %d", s_retry_count,
				         (int)res);
			}
		}
#endif
		if (init_ok)
			s_retry_count = 0;
		steering_needs_limits = init_ok;
	}
#endif

#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
	if (steering_needs_limits)
	{
		*ctx->motor_uim2852_steering_ready = true;
		node_log_component_regained(ctx->tag, "MOTOR_UIM2852_STEERING");
	}
#endif

#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
	if (braking_needs_limits)
	{
		*ctx->motor_uim2852_braking_ready = true;
		node_log_component_regained(ctx->tag, "MOTOR_UIM2852_BRAKING");
	}
#endif
}
