/**
 * @file can_twai.cpp
 * @brief CAN bus TWAI driver wrapper implementation.
 */
#include "can_twai.h"
#include "esp_log.h"

namespace
{

const char *TAG = "CAN_TWAI";

// Recovery timing constants
// ============================================================================
// Frame Log Helpers
// ============================================================================
#ifdef CONFIG_LOG_CAN_FRAMES

// Returns a short type label for an RX extended frame based on the command word.
// CW_ER (0x0F) = error report, CW_NOTIFY (0x5A) = real-time notification,
// ACK bit (0x80) set = motor ACK for a commanded CW, else status/query response.
static inline const char *can_rx_ext_type(uint8_t cw)
{
	uint8_t cw_base = cw & 0x7F;
	if (cw_base == 0x0F) return "ERR  ";
	if (cw_base == 0x5A) return "NOTIF";
	if (cw & 0x80)       return "ACK  ";
	return "RSP  ";
}

// Returns true if a frame should be suppressed from the frame log.
static inline bool can_log_suppressed(uint32_t id, bool extd)
{
	if (extd)
		return false;
#ifdef CONFIG_LOG_CAN_FRAMES_MOTORS_ONLY
	return true; // suppress all standard frames
#endif
#ifdef CONFIG_LOG_CAN_FRAMES_SUPPRESS_SAFETY_HB
	if (id == 0x100) return true;
#endif
#ifdef CONFIG_LOG_CAN_FRAMES_SUPPRESS_PLANNER_HB
	if (id == 0x110) return true;
#endif
#ifdef CONFIG_LOG_CAN_FRAMES_SUPPRESS_CONTROL_HB
	if (id == 0x120) return true;
#endif
	return false;
}

// Decode a SimpleCAN 29-bit extended ID into producer_id and command word.
// Mirrors stepper_uim2852_parse_can_id() without pulling in that header.
// Returns true if the SID format is valid (bit 8 of SID must be set).
static inline bool can_parse_ext_id(uint32_t can_id, uint8_t *producer_id, uint8_t *cw)
{
	uint16_t sid = (can_id >> 18) & 0x07FF;
	uint32_t eid = can_id & 0x3FFFF;
	if ((sid & 0x0100) == 0)
		return false;
	*producer_id = (uint8_t)(((eid >> 11) & 0x0060) | ((sid >> 6) & 0x001F));
	*cw          = (uint8_t)(eid & 0x00FF);
	return true;
}

#endif // CONFIG_LOG_CAN_FRAMES
constexpr int RECOVERY_POLL_ITERATIONS = 25; // 25 iterations x 10ms = 250ms max
constexpr int RECOVERY_POLL_INTERVAL_MS = 10;
constexpr int RECOVERY_STOP_SETTLE_MS = 100;      // settle after twai_stop()
constexpr int RECOVERY_REINSTALL_SETTLE_MS = 200; // settle after twai_driver_uninstall()

} // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t can_twai_init_default(gpio_num_t tx_gpio, gpio_num_t rx_gpio)
{
	twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_gpio, rx_gpio, TWAI_MODE_NORMAL);
	twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

	esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "TWAI driver install failed: %s", esp_err_to_name(err));
		return err;
	}

	err = twai_start();
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "TWAI start failed: %s", esp_err_to_name(err));
		twai_driver_uninstall();
		return err;
	}

	return ESP_OK;
}

// ============================================================================
// Transmit
// ============================================================================

esp_err_t can_twai_send(uint32_t identifier, const uint8_t data[8], TickType_t timeout)
{
	if (!data)
		return ESP_ERR_INVALID_ARG;

	twai_message_t msg = {};
	msg.identifier = identifier;
	msg.extd = 0;
	msg.rtr = 0;
	msg.ss = 0;
	msg.self = 0;
	msg.dlc_non_comp = 0;
	msg.data_length_code = 8;
	for (int i = 0; i < 8; ++i)
		msg.data[i] = data[i];

	esp_err_t err = twai_transmit(&msg, timeout);
#ifdef CONFIG_LOG_CAN_FRAMES
	if (!can_log_suppressed(identifier, false))
		ESP_LOGI(TAG, "TX std  id=0x%03lX dlc=8 [%02X %02X %02X %02X %02X %02X %02X %02X] %s",
		         (unsigned long)identifier,
		         data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
		         err == ESP_OK ? "ok" : esp_err_to_name(err));
#endif
	return err;
}

esp_err_t can_twai_send_extended(uint32_t identifier, const uint8_t *data, uint8_t dlc, TickType_t timeout)
{
	if (!data && dlc > 0)
		return ESP_ERR_INVALID_ARG;

	twai_message_t msg = {};
	msg.identifier = identifier;
	msg.extd = 1;
	msg.rtr = 0;
	msg.ss = 0;
	msg.self = 0;
	msg.dlc_non_comp = 0;
	msg.data_length_code = (dlc > 8) ? 8 : dlc;
	for (int i = 0; i < msg.data_length_code; ++i)
		msg.data[i] = data[i];

	esp_err_t err = twai_transmit(&msg, timeout);
#ifdef CONFIG_LOG_CAN_FRAMES
	{
		uint8_t prod = 0, cw = 0;
		if (can_parse_ext_id(identifier, &prod, &cw))
			ESP_LOGI(TAG, "TX ext  id=0x%08lX node=%u cw=0x%02X dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X] %s",
			         (unsigned long)identifier, prod, cw, msg.data_length_code,
			         msg.data[0], msg.data[1], msg.data[2], msg.data[3],
			         msg.data[4], msg.data[5], msg.data[6], msg.data[7],
			         err == ESP_OK ? "ok" : esp_err_to_name(err));
		else
			ESP_LOGI(TAG, "TX ext  id=0x%08lX dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X] %s",
			         (unsigned long)identifier, msg.data_length_code,
			         msg.data[0], msg.data[1], msg.data[2], msg.data[3],
			         msg.data[4], msg.data[5], msg.data[6], msg.data[7],
			         err == ESP_OK ? "ok" : esp_err_to_name(err));
	}
#endif
	return err;
}

// ============================================================================
// Receive
// ============================================================================

esp_err_t can_twai_receive(twai_message_t *msg, TickType_t timeout)
{
	esp_err_t err = twai_receive(msg, timeout);
#ifdef CONFIG_LOG_CAN_FRAMES
	if (err == ESP_OK && !can_log_suppressed(msg->identifier, msg->extd))
	{
		if (msg->extd)
		{
			uint8_t prod = 0, cw = 0;
			if (can_parse_ext_id(msg->identifier, &prod, &cw))
				ESP_LOGI(TAG, "RX ext  id=0x%08lX node=%u cw=0x%02X %s dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X]",
				         (unsigned long)msg->identifier, prod, cw, can_rx_ext_type(cw), msg->data_length_code,
				         msg->data[0], msg->data[1], msg->data[2], msg->data[3],
				         msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
			else
				ESP_LOGI(TAG, "RX ext  id=0x%08lX dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X]",
				         (unsigned long)msg->identifier, msg->data_length_code,
				         msg->data[0], msg->data[1], msg->data[2], msg->data[3],
				         msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
		}
		else
		{
			ESP_LOGI(TAG, "RX std  id=0x%03lX dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X]",
			         (unsigned long)msg->identifier, msg->data_length_code,
			         msg->data[0], msg->data[1], msg->data[2], msg->data[3],
			         msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
		}
	}
#endif
	return err;
}

// ============================================================================
// Bus Health
// ============================================================================

bool can_twai_bus_ok(void)
{
	twai_status_info_t status;
	if (twai_get_status_info(&status) != ESP_OK)
		return false;

	return status.state == TWAI_STATE_RUNNING;
}

esp_err_t can_twai_recover(gpio_num_t tx_gpio, gpio_num_t rx_gpio, const char *log_tag)
{
#ifdef CONFIG_LOG_CAN_RECOVERY
	// NULL log_tag = suppress internal logs (caller handles its own logging).
	const char *tag = log_tag;
#else
	(void)log_tag;
#endif

	// --- Tier 1: CAN-spec bus-off recovery ---
#ifdef CONFIG_LOG_CAN_RECOVERY
	if (tag)
		ESP_LOGI(tag, "CAN recovery: attempting bus-off recovery");
#endif
	esp_err_t err = twai_initiate_recovery();
	if (err == ESP_OK)
	{
		for (int i = 0; i < RECOVERY_POLL_ITERATIONS; i++)
		{
			vTaskDelay(pdMS_TO_TICKS(RECOVERY_POLL_INTERVAL_MS));
			twai_status_info_t status;
			if (twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_STOPPED)
			{
				err = twai_start();
				if (err == ESP_OK)
				{
#ifdef CONFIG_LOG_CAN_RECOVERY
					if (tag)
						ESP_LOGI(tag, "CAN recovery: bus-off recovery succeeded");
#endif
					return ESP_OK;
				}
				break; // twai_start failed, escalate
			}
		}
	}

	// --- Tier 2: soft driver reset (stop/start) ---
#ifdef CONFIG_LOG_CAN_RECOVERY
	if (tag)
		ESP_LOGI(tag, "CAN recovery: bus-off failed, attempting stop/start");
#endif
	(void)twai_stop();
	vTaskDelay(pdMS_TO_TICKS(RECOVERY_STOP_SETTLE_MS));

	err = twai_start();
	if (err == ESP_OK)
	{
#ifdef CONFIG_LOG_CAN_RECOVERY
		if (tag)
			ESP_LOGI(tag, "CAN recovery: stop/start succeeded");
#endif
		return ESP_OK;
	}

	// --- Tier 3: full driver uninstall/reinstall ---
#ifdef CONFIG_LOG_CAN_RECOVERY
	if (tag)
		ESP_LOGI(tag, "CAN recovery: stop/start failed (%s), reinstalling TWAI driver", esp_err_to_name(err));
#endif

	// Ensure driver is stopped before uninstall. Ignore invalid-state here:
	// it can mean already stopped or not installed.
	(void)twai_stop();

	esp_err_t uninstall_err = twai_driver_uninstall();
	if (uninstall_err == ESP_ERR_INVALID_STATE)
	{
		// Distinguish "already uninstalled" from "still installed in wrong state".
		// If status query succeeds, driver is still present; attempt one more
		// stop/uninstall cycle before giving up.
		twai_status_info_t status;
		if (twai_get_status_info(&status) == ESP_OK)
		{
			(void)twai_stop();
			vTaskDelay(pdMS_TO_TICKS(RECOVERY_STOP_SETTLE_MS));
			uninstall_err = twai_driver_uninstall();
			if (uninstall_err != ESP_OK)
				return uninstall_err;
		}
	}
	else if (uninstall_err != ESP_OK)
	{
		return uninstall_err;
	}

	vTaskDelay(pdMS_TO_TICKS(RECOVERY_REINSTALL_SETTLE_MS));
	err = can_twai_init_default(tx_gpio, rx_gpio);

#ifdef CONFIG_LOG_CAN_RECOVERY
	if (err == ESP_OK)
	{
		if (tag)
			ESP_LOGI(tag, "CAN recovery: driver reinstall succeeded");
	}
	else
	{
		if (tag)
			ESP_LOGI(tag, "CAN recovery: driver reinstall failed (%s)", esp_err_to_name(err));
	}
#endif

	return err;
}
