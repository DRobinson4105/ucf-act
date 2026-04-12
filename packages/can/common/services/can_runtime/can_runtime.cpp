/**
 * @file can_runtime.cpp
 * @brief Shared CAN runtime helpers for TX reservation, recovery, and bus-failure handling.
 */

#include "can_runtime.h"

#include <cstring>

#include "can_twai.h"
#include "freertos/task.h"

namespace
{

bool try_request_recovery(portMUX_TYPE *lock, volatile bool *twai_ready, volatile bool *recovery_in_progress)
{
	if (!lock || !twai_ready || !recovery_in_progress)
		return false;

	bool requested = false;
	taskENTER_CRITICAL(lock);
	if (*twai_ready && !*recovery_in_progress)
	{
		*twai_ready = false;
		requested = true;
	}
	taskEXIT_CRITICAL(lock);
	return requested;
}

} // namespace

bool can_runtime_try_reserve_tx(portMUX_TYPE *lock, volatile bool *twai_ready, volatile bool *recovery_in_progress,
                                volatile uint8_t *in_flight)
{
	if (!lock || !twai_ready || !recovery_in_progress || !in_flight)
		return false;

	taskENTER_CRITICAL(lock);
	if (!*twai_ready || *recovery_in_progress || *in_flight == UINT8_MAX)
	{
		taskEXIT_CRITICAL(lock);
		return false;
	}

	uint8_t reserved = *in_flight;
	*in_flight = (uint8_t)(reserved + 1);
	taskEXIT_CRITICAL(lock);
	return true;
}

void can_runtime_release_tx(portMUX_TYPE *lock, volatile uint8_t *in_flight)
{
	if (!lock || !in_flight)
		return;

	taskENTER_CRITICAL(lock);
	if (*in_flight > 0)
	{
		uint8_t remaining = *in_flight;
		*in_flight = (uint8_t)(remaining - 1);
	}
	taskEXIT_CRITICAL(lock);
}

bool can_runtime_try_begin_recovery(portMUX_TYPE *lock, volatile bool *recovery_in_progress)
{
	if (!lock || !recovery_in_progress)
		return false;

	taskENTER_CRITICAL(lock);
	bool already_running = *recovery_in_progress;
	if (!already_running)
		*recovery_in_progress = true;
	taskEXIT_CRITICAL(lock);
	return !already_running;
}

void can_runtime_finish_recovery(portMUX_TYPE *lock, volatile bool *recovery_in_progress)
{
	if (!lock || !recovery_in_progress)
		return;

	taskENTER_CRITICAL(lock);
	*recovery_in_progress = false;
	taskEXIT_CRITICAL(lock);
}

bool can_runtime_note_tx_result(portMUX_TYPE *lock, volatile bool *twai_ready, volatile bool *recovery_in_progress,
                                uint8_t *fail_count, uint8_t threshold, esp_err_t tx_err)
{
	if (!lock || !twai_ready || !recovery_in_progress || !fail_count || threshold == 0)
		return false;

	bool trigger_recovery = false;
	bool check_bus = false;

	taskENTER_CRITICAL(lock);
	can_tx_track_inputs_t inputs = {
		.fail_count = *fail_count,
		.threshold = threshold,
		.tx_ok = (tx_err == ESP_OK),
	};
	can_tx_track_result_t result = can_tx_track(&inputs);
	*fail_count = result.new_fail_count;
	if (result.trigger_recovery)
	{
		*fail_count = 0;
		check_bus = true;
	}
	taskEXIT_CRITICAL(lock);

	if (check_bus && !can_twai_bus_ok())
	{
		trigger_recovery = try_request_recovery(lock, twai_ready, recovery_in_progress);
	}

	return trigger_recovery;
}

bool can_runtime_recover_with_probe(const can_runtime_recovery_probe_t *probe, esp_err_t *recover_err)
{
	if (!probe)
		return false;

	esp_err_t err = can_twai_recover(probe->tx_gpio, probe->rx_gpio, probe->log_tag);
	if (recover_err)
		*recover_err = err;
	if (err != ESP_OK)
		return false;

	uint8_t probe_data[8] = {};
	can_encode_heartbeat(probe_data, &probe->probe_heartbeat);
	twai_message_t probe_msg = {};
	probe_msg.identifier = probe->probe_can_id;
	probe_msg.data_length_code = 8;
	memcpy(probe_msg.data, probe_data, 8);
	(void)can_twai_send(&probe_msg, probe->probe_send_timeout);
	vTaskDelay(probe->probe_settle_delay);

	twai_status_info_t probe_status;
	if (twai_get_status_info(&probe_status) != ESP_OK)
		return false;

	bool bus_running = (probe_status.state == TWAI_STATE_RUNNING);
	bool no_tx_errors = (probe_status.tx_error_counter == 0);
	return bus_running && no_tx_errors;
}
