#include "twai_port.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TWAI_PORT_MAX_DLC 8U

static const char *TAG = "twai_port";

static esp_err_t driver_install_default(const twai_general_config_t *g_config,
                                        const twai_timing_config_t *t_config,
                                        const twai_filter_config_t *f_config)
{
    return twai_driver_install(g_config, t_config, f_config);
}

static esp_err_t driver_uninstall_default(void)
{
    return twai_driver_uninstall();
}

static esp_err_t start_default(void)
{
    return twai_start();
}

static esp_err_t stop_default(void)
{
    return twai_stop();
}

static esp_err_t transmit_default(twai_message_t *message, TickType_t ticks_to_wait)
{
    return twai_transmit(message, ticks_to_wait);
}

static esp_err_t receive_default(twai_message_t *message, TickType_t ticks_to_wait)
{
    return twai_receive(message, ticks_to_wait);
}

static esp_err_t read_alerts_default(uint32_t *alerts, TickType_t ticks_to_wait)
{
    return twai_read_alerts(alerts, ticks_to_wait);
}

static esp_err_t get_status_info_default(twai_status_info_t *status_info)
{
    return twai_get_status_info(status_info);
}

static const twai_port_driver_ops_t s_default_driver_ops = {
    .driver_install = driver_install_default,
    .driver_uninstall = driver_uninstall_default,
    .start = start_default,
    .stop = stop_default,
    .transmit = transmit_default,
    .receive = receive_default,
    .read_alerts = read_alerts_default,
    .get_status_info = get_status_info_default,
};

static twai_port_cfg_t s_cfg;
static bool s_initialized = false;
static twai_port_state_t s_state = TWAI_PORT_STATE_UNINITIALIZED;
static bool s_recovery_in_progress = false;
static uint32_t s_active_receivers = 0;
static portMUX_TYPE s_rx_count_lock = portMUX_INITIALIZER_UNLOCKED;
static const twai_port_driver_ops_t *s_driver_ops = &s_default_driver_ops;

/*
 * s_send_lock:
 * Guards the TX critical boundary. Quiesce takes this lock and holds it until resume,
 * which guarantees no send is inside the driver while the bus is quiesced.
 *
 * s_state_lock:
 * Guards state/config transitions and quiesced/running checks.
 *
 * s_rx_idle_sem:
 * Signaled when the last in-flight receive exits twai_receive(). State transitions that
 * stop or reconfigure the driver wait on this semaphore after blocking new receives.
 *
 * s_rx_count_lock:
 * Guards s_active_receivers so receive accounting remains balanced even if state-lock
 * re-entry fails on an exceptional path.
 * */
static SemaphoreHandle_t s_send_lock = NULL;
static SemaphoreHandle_t s_state_lock = NULL;
static SemaphoreHandle_t s_rx_idle_sem = NULL;

static bool twai_port_state_is_active_locked(void) {
    return s_initialized && (
            s_state == TWAI_PORT_STATE_RUNNING ||
            s_state == TWAI_PORT_STATE_QUIESCED ||
            s_state == TWAI_PORT_STATE_RECOVERING
            );
}

static uint32_t active_receivers_get(void)
{
    uint32_t count;

    portENTER_CRITICAL(&s_rx_count_lock);
    count = s_active_receivers;
    portEXIT_CRITICAL(&s_rx_count_lock);

    return count;
}

static void active_receivers_inc(void)
{
    portENTER_CRITICAL(&s_rx_count_lock);
    s_active_receivers++;
    portEXIT_CRITICAL(&s_rx_count_lock);
}

static void active_receivers_dec(void)
{
    bool became_idle = false;

    portENTER_CRITICAL(&s_rx_count_lock);
    if (s_active_receivers > 0) {
        s_active_receivers--;
        became_idle = (s_active_receivers == 0);
    }
    portEXIT_CRITICAL(&s_rx_count_lock);

    if (became_idle) {
        xSemaphoreGive(s_rx_idle_sem);
    }
}

static bool frame_matches_filter_mode(const twai_port_cfg_t *cfg, const twai_message_t *msg)
{
    if (cfg == NULL || msg == NULL) {
        return false;
    }

    switch (cfg->filter_mode) {
        case TWAI_PORT_FILTER_ACCEPT_ALL:
            return true;
        case TWAI_PORT_FILTER_STD_ONLY:
            return !msg->extd;
        case TWAI_PORT_FILTER_EXT_ONLY:
            return msg->extd;
        default:
            return false;
    }
}

static bool should_log_frame(const twai_message_t *msg, bool is_tx) {
    if (msg == NULL) {
        return false;
    }

    if (is_tx && !s_cfg.log_tx) {
        return false;
    }

    if (!is_tx && !s_cfg.log_rx) {
        return false;
    }

    if (s_cfg.log_ext_only && !msg->extd) {
        return false;
    }

    return true;
}

static void log_frame(const char *dir, const twai_message_t *msg, esp_err_t result, bool include_result) {
    if (msg == NULL) {
        return;
    }

    uint8_t dlc = msg->data_length_code;
    if (dlc > TWAI_PORT_MAX_DLC) {
        dlc = TWAI_PORT_MAX_DLC;
    }

    char data_buf[3 * TWAI_PORT_MAX_DLC + 1];
    size_t offset = 0;

    for (uint8_t i = 0; i < dlc; ++i) {
        int written = snprintf(&data_buf[offset], sizeof(data_buf) - offset,
                               (i == 0) ? "%02X" : " %02X", msg->data[i]);
        if (written < 0) {
            return;
        }
        if ((size_t)written >= sizeof(data_buf) - offset) {
            offset = sizeof(data_buf) - 1;
            break;
        }
        offset += (size_t)written;
    }
    data_buf[offset] = '\0';

    uint32_t id = msg->identifier;
    const char *id_type = msg->extd ? "ext" : "std";
    const char *rtr = msg->rtr ? " RTR" : "";

    if (include_result){
        ESP_LOGI(TAG,
                "%s %s id=0x%08" PRIX32 " dlc=%u data=[%s]%s rc=%s",
                dir,
                id_type,
                id,
                (unsigned)msg->data_length_code,
                data_buf,
                rtr,
                esp_err_to_name(result));

    } else {
        ESP_LOGI(TAG,
                "%s %s id=0x%08" PRIX32 " dlc=%u data=[%s]%s",
                dir,
                id_type,
                id,
                (unsigned)msg->data_length_code,
                data_buf,
                rtr);
    }
}

static esp_err_t build_timing_cfg(uint32_t bitrate, twai_timing_config_t *out_cfg) {
    if (out_cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (bitrate) {
        case 1000000:
            *out_cfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
            return ESP_OK;
        case 800000:
            *out_cfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_800KBITS();
            return ESP_OK;
        case 500000:
            *out_cfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
            return ESP_OK;
        case 250000:
            *out_cfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
            return ESP_OK;
        case 125000:
            *out_cfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
            return ESP_OK;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

static twai_filter_config_t build_filter_cfg(const twai_port_cfg_t *cfg) {
    twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (cfg == NULL) {
        return f_cfg;
    }

    /*
     * Keep the hardware filter permissive and enforce frame-format filtering in
     * twai_port_receive(). ESP-IDF's single-filter layout can express ID-based
     * filters cleanly, but format-only filtering is awkward enough that a
     * software check is easier to reason about and keeps API behavior correct.
     */
    switch (cfg->filter_mode) {
        case TWAI_PORT_FILTER_ACCEPT_ALL:
        case TWAI_PORT_FILTER_STD_ONLY:
        case TWAI_PORT_FILTER_EXT_ONLY:
        default:
            return f_cfg;
    }
}

static esp_err_t install_driver_from_cfg(const twai_port_cfg_t *cfg){
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_timing_config_t t_cfg;
    esp_err_t err = build_timing_cfg(cfg->bitrate, &t_cfg);
    if (err != ESP_OK) {
        return err;
    }

    twai_mode_t hw_mode;
    switch (cfg->mode) {
        case TWAI_PORT_MODE_LISTEN_ONLY: hw_mode = TWAI_MODE_LISTEN_ONLY; break;
        case TWAI_PORT_MODE_NO_ACK:      hw_mode = TWAI_MODE_NO_ACK;      break;
        default:                         hw_mode = TWAI_MODE_NORMAL;       break;
    }

    twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)cfg->tx_gpio, (gpio_num_t)cfg->rx_gpio, hw_mode);

    g_cfg.tx_queue_len = cfg->tx_queue_len;
    g_cfg.rx_queue_len = cfg->rx_queue_len;
    g_cfg.alerts_enabled = cfg->alerts_enabled;

    twai_filter_config_t f_cfg = build_filter_cfg(cfg);

    return s_driver_ops->driver_install(&g_cfg, &t_cfg, &f_cfg);
}

static esp_err_t stop_and_uninstall_locked(void){
    esp_err_t err_stop = ESP_OK;
    esp_err_t err_uninstall = ESP_OK;

    if(!s_initialized){
        return ESP_OK;
    }

    if(s_state == TWAI_PORT_STATE_RUNNING || s_state == TWAI_PORT_STATE_QUIESCED || s_state == TWAI_PORT_STATE_RECOVERING){
        err_stop = s_driver_ops->stop();
        if (err_stop != ESP_OK && err_stop != ESP_ERR_INVALID_STATE) {
            return err_stop;
        }
    }

    err_uninstall = s_driver_ops->driver_uninstall();
    if (err_uninstall != ESP_OK && err_uninstall != ESP_ERR_INVALID_STATE) {
        return err_uninstall;
    }

    s_initialized = false;
    s_state = TWAI_PORT_STATE_UNINITIALIZED;
    s_recovery_in_progress = false;
    return ESP_OK;
}

static esp_err_t wait_for_receivers_to_drain(TickType_t timeout_ticks)
{
    TickType_t deadline = 0;
    if (timeout_ticks != portMAX_DELAY) {
        deadline = xTaskGetTickCount() + timeout_ticks;
    }

    while (true) {
        bool idle = (active_receivers_get() == 0);

        if (idle) {
            return ESP_OK;
        }

        TickType_t wait_ticks = portMAX_DELAY;
        if (timeout_ticks != portMAX_DELAY) {
            TickType_t now = xTaskGetTickCount();
            if (now >= deadline) {
                return ESP_ERR_TIMEOUT;
            }
            wait_ticks = deadline - now;
        }

        if (xSemaphoreTake(s_rx_idle_sem, wait_ticks) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }
}

static esp_err_t validate_cfg(const twai_port_cfg_t *cfg){
    if (cfg == NULL){
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->tx_queue_len == 0 || cfg->rx_queue_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->mode != TWAI_PORT_MODE_NORMAL &&
        cfg->mode != TWAI_PORT_MODE_LISTEN_ONLY &&
        cfg->mode != TWAI_PORT_MODE_NO_ACK) {
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->filter_mode != TWAI_PORT_FILTER_ACCEPT_ALL &&
        cfg->filter_mode != TWAI_PORT_FILTER_STD_ONLY &&
        cfg->filter_mode != TWAI_PORT_FILTER_EXT_ONLY) {
        return ESP_ERR_INVALID_ARG;
    }

    if (build_timing_cfg(cfg->bitrate, &(twai_timing_config_t){0}) != ESP_OK) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_OK;
}

esp_err_t twai_port_init(const twai_port_cfg_t *cfg){
    esp_err_t err = validate_cfg(cfg);
    if (err != ESP_OK) {
        return err;
    }

    SemaphoreHandle_t new_send_lock = NULL;
    SemaphoreHandle_t new_state_lock = NULL;
    SemaphoreHandle_t new_rx_idle_sem = NULL;

    if (s_send_lock == NULL) {
        new_send_lock = xSemaphoreCreateMutex();
        if (new_send_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_state_lock == NULL) {
        new_state_lock = xSemaphoreCreateMutex();
        if (new_state_lock == NULL) {
            if (new_send_lock != NULL) {
                vSemaphoreDelete(new_send_lock);
            }
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_rx_idle_sem == NULL) {
        new_rx_idle_sem = xSemaphoreCreateBinary();
        if (new_rx_idle_sem == NULL) {
            if (new_send_lock != NULL) {
                vSemaphoreDelete(new_send_lock);
            }
            if (new_state_lock != NULL) {
                vSemaphoreDelete(new_state_lock);
            }
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_send_lock == NULL) {
        s_send_lock = new_send_lock;
    }
    if (s_state_lock == NULL) {
        s_state_lock = new_state_lock;
    }
    if (s_rx_idle_sem == NULL) {
        s_rx_idle_sem = new_rx_idle_sem;
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_initialized) {
        xSemaphoreGive(s_state_lock);
        return ESP_ERR_INVALID_STATE;
    }

    err = install_driver_from_cfg(cfg);
    if (err != ESP_OK) {
        xSemaphoreGive(s_state_lock);
        return err;
    }

    s_cfg = *cfg;
    s_initialized = true;
    s_state = TWAI_PORT_STATE_STOPPED;
    s_recovery_in_progress = false;
    portENTER_CRITICAL(&s_rx_count_lock);
    s_active_receivers = 0;
    portEXIT_CRITICAL(&s_rx_count_lock);

    xSemaphoreGive(s_state_lock);

    if (s_cfg.log_alerts){
        ESP_LOGI(TAG, "initialized bitrate=%" PRIu32 " tx_gpio=%d rx_gpio=%d", s_cfg.bitrate, s_cfg.tx_gpio, s_cfg.rx_gpio);
    }

    return ESP_OK;
}

esp_err_t twai_port_start(void)
{
    if (s_state_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_initialized) {
        xSemaphoreGive(s_state_lock);
        return ESP_ERR_INVALID_STATE;
    }

    if (s_state == TWAI_PORT_STATE_RUNNING) {
        xSemaphoreGive(s_state_lock);
        return ESP_OK;
    }

    if (s_state == TWAI_PORT_STATE_QUIESCED || s_state == TWAI_PORT_STATE_RECOVERING) {
        xSemaphoreGive(s_state_lock);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = s_driver_ops->start();
    if (err == ESP_OK) {
        s_state = TWAI_PORT_STATE_RUNNING;
    }

    xSemaphoreGive(s_state_lock);

    if (err == ESP_OK && s_cfg.log_alerts) {
        ESP_LOGI(TAG, "started bitrate=%" PRIu32, s_cfg.bitrate);
    }

    return err;
}

esp_err_t twai_port_stop(void)
{
    if (s_state_lock == NULL || s_send_lock == NULL || s_rx_idle_sem == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_initialized) {
        xSemaphoreGive(s_state_lock);
        return ESP_ERR_INVALID_STATE;
    }

    if (s_state == TWAI_PORT_STATE_STOPPED) {
        xSemaphoreGive(s_state_lock);
        return ESP_OK;
    }

    if (s_state == TWAI_PORT_STATE_QUIESCED || s_state == TWAI_PORT_STATE_RECOVERING) {
        xSemaphoreGive(s_state_lock);
        return ESP_ERR_INVALID_STATE;
    }

    twai_port_state_t prev_state = s_state;
    s_state = TWAI_PORT_STATE_STOPPED;
    xSemaphoreGive(s_state_lock);

    if (xSemaphoreTake(s_send_lock, portMAX_DELAY) != pdTRUE) {
        if (xSemaphoreTake(s_state_lock, portMAX_DELAY) == pdTRUE) {
            s_state = prev_state;
            xSemaphoreGive(s_state_lock);
        }
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = wait_for_receivers_to_drain(portMAX_DELAY);
    if (err == ESP_OK) {
        err = s_driver_ops->stop();
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
            err = ESP_OK;
        }
    }

    xSemaphoreGive(s_send_lock);

    if (err != ESP_OK && xSemaphoreTake(s_state_lock, portMAX_DELAY) == pdTRUE) {
        s_state = prev_state;
        xSemaphoreGive(s_state_lock);
    }

    if (err == ESP_OK && s_cfg.log_alerts) {
        ESP_LOGI(TAG, "stopped");
    }

    return err;
}

esp_err_t twai_port_deinit(void)
{
    if (s_state_lock == NULL || s_send_lock == NULL || s_rx_idle_sem == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_initialized) {
        xSemaphoreGive(s_state_lock);
        return ESP_ERR_INVALID_STATE;
    }

    twai_port_state_t prev_state = s_state;
    if (s_state == TWAI_PORT_STATE_RUNNING || s_state == TWAI_PORT_STATE_RECOVERING) {
        s_state = TWAI_PORT_STATE_STOPPED;
    }
    xSemaphoreGive(s_state_lock);

    if (xSemaphoreTake(s_send_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = wait_for_receivers_to_drain(portMAX_DELAY);
    if (err == ESP_OK) {
        if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
            xSemaphoreGive(s_send_lock);
            return ESP_ERR_TIMEOUT;
        }

        err = stop_and_uninstall_locked();
        xSemaphoreGive(s_state_lock);
    }

    xSemaphoreGive(s_send_lock);

    if (err != ESP_OK && xSemaphoreTake(s_state_lock, portMAX_DELAY) == pdTRUE) {
        if (s_initialized) {
            s_state = prev_state;
        }
        xSemaphoreGive(s_state_lock);
    }

    if (err == ESP_OK && s_cfg.log_alerts) {
        ESP_LOGI(TAG, "deinitialized");
    }

    return err;
}

twai_port_state_t twai_port_get_state(void)
{
    if (s_state_lock == NULL) {
        return TWAI_PORT_STATE_UNINITIALIZED;
    }

    twai_port_state_t state_copy;
    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return TWAI_PORT_STATE_UNINITIALIZED;
    }

    state_copy = s_state;
    xSemaphoreGive(s_state_lock);
    return state_copy;
}

bool twai_port_is_initialized(void)
{
    if (s_state_lock == NULL) {
        return false;
    }

    bool out;
    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    out = s_initialized;
    xSemaphoreGive(s_state_lock);
    return out;
}

bool twai_port_is_running(void)
{
    if (s_state_lock == NULL) {
        return false;
    }

    bool out;
    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    out = (s_initialized && s_state == TWAI_PORT_STATE_RUNNING);
    xSemaphoreGive(s_state_lock);
    return out;
}

bool twai_port_is_quiesced(void)
{
    if (s_state_lock == NULL) {
        return false;
    }

    bool out;
    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    out = (s_initialized && s_state == TWAI_PORT_STATE_QUIESCED);
    xSemaphoreGive(s_state_lock);
    return out;
}

esp_err_t twai_port_get_runtime_info(twai_port_runtime_info_t *out_info)
{
    if (out_info == NULL || s_state_lock == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    out_info->state = s_state;
    out_info->initialized = s_initialized;
    out_info->running = (s_initialized && s_state == TWAI_PORT_STATE_RUNNING);
    out_info->quiesced = (s_initialized && s_state == TWAI_PORT_STATE_QUIESCED);
    out_info->recovery_in_progress = s_recovery_in_progress;

    xSemaphoreGive(s_state_lock);
    return ESP_OK;
}

esp_err_t twai_port_get_cfg(twai_port_cfg_t *out_cfg)
{
    if (out_cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_state_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_initialized) {
        xSemaphoreGive(s_state_lock);
        return ESP_ERR_INVALID_STATE;
    }

    *out_cfg = s_cfg;
    xSemaphoreGive(s_state_lock);
    return ESP_OK;
}

esp_err_t twai_port_send(const twai_message_t *msg, TickType_t timeout_ticks)
{
    if (msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (msg->data_length_code > TWAI_PORT_MAX_DLC) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_send_lock == NULL || s_state_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    bool allowed = s_initialized && (s_state == TWAI_PORT_STATE_RUNNING);
    xSemaphoreGive(s_state_lock);

    if (!allowed) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_send_lock, timeout_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        xSemaphoreGive(s_send_lock);
        return ESP_ERR_TIMEOUT;
    }

    allowed = s_initialized && (s_state == TWAI_PORT_STATE_RUNNING);
    xSemaphoreGive(s_state_lock);

    if (!allowed) {
        xSemaphoreGive(s_send_lock);
        return ESP_ERR_INVALID_STATE;
    }

    if (should_log_frame(msg, true)) {
        log_frame("TX", msg, ESP_OK, false);
    }

    esp_err_t err = s_driver_ops->transmit((twai_message_t *)msg, timeout_ticks);

    if (should_log_frame(msg, true)) {
        log_frame("TXR", msg, err, true);
    }

    xSemaphoreGive(s_send_lock);
    return err;
}

esp_err_t twai_port_receive(twai_message_t *out_msg, TickType_t timeout_ticks)
{
    if (out_msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_state_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    bool allowed = s_initialized &&
                   (s_state == TWAI_PORT_STATE_RUNNING || s_state == TWAI_PORT_STATE_RECOVERING);
    twai_port_cfg_t cfg = s_cfg;
    xSemaphoreGive(s_state_lock);

    if (!allowed) {
        return ESP_ERR_INVALID_STATE;
    }

    active_receivers_inc();

    /*
     * twai_receive() is task-safe; no external TX lock is needed here.
     * Quiesce blocks new receives via the state transition and then waits for
     * the in-flight receiver count to drain before touching the driver.
     */
    TickType_t start_ticks = 0;
    if (timeout_ticks != portMAX_DELAY) {
        start_ticks = xTaskGetTickCount();
    }

    esp_err_t err = ESP_ERR_TIMEOUT;
    while (true) {
        TickType_t wait_ticks = timeout_ticks;
        if (timeout_ticks != portMAX_DELAY) {
            TickType_t now = xTaskGetTickCount();
            TickType_t elapsed = now - start_ticks;
            if (elapsed >= timeout_ticks) {
                err = ESP_ERR_TIMEOUT;
                break;
            }
            wait_ticks = timeout_ticks - elapsed;
        }

        err = s_driver_ops->receive(out_msg, wait_ticks);
        if (err != ESP_OK) {
            break;
        }

        if (frame_matches_filter_mode(&cfg, out_msg)) {
            if (should_log_frame(out_msg, false)) {
                log_frame("RX", out_msg, ESP_OK, false);
            }
            break;
        }
    }

    active_receivers_dec();

    return err;
}

esp_err_t twai_port_send_std(uint16_t std_id,
                             const uint8_t *data,
                             uint8_t dlc,
                             TickType_t timeout_ticks)
{
    if (dlc > TWAI_PORT_MAX_DLC) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_message_t msg = {0};
    msg.identifier = std_id;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = dlc;

    if (data != NULL && dlc > 0) {
        memcpy(msg.data, data, dlc);
    }

    return twai_port_send(&msg, timeout_ticks);
}

esp_err_t twai_port_send_ext(uint32_t ext_id,
                             const uint8_t *data,
                             uint8_t dlc,
                             TickType_t timeout_ticks)
{
    if (dlc > TWAI_PORT_MAX_DLC) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_message_t msg = {0};
    msg.identifier = ext_id;
    msg.extd = 1;
    msg.rtr = 0;
    msg.data_length_code = dlc;

    if (data != NULL && dlc > 0) {
        memcpy(msg.data, data, dlc);
    }

    return twai_port_send(&msg, timeout_ticks);
}

esp_err_t twai_port_read_alerts(uint32_t *out_alerts, TickType_t timeout_ticks)
{
    if (out_alerts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_state_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    bool allowed = twai_port_state_is_active_locked();
    xSemaphoreGive(s_state_lock);

    if (!allowed) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = s_driver_ops->read_alerts(out_alerts, timeout_ticks);

    if (err == ESP_OK && s_cfg.log_alerts) {
        ESP_LOGI(TAG, "alerts=0x%08" PRIX32, *out_alerts);
    }

    return err;
}

esp_err_t twai_port_get_status(twai_status_info_t *out_status)
{
    if (out_status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_state_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    bool allowed = s_initialized;
    xSemaphoreGive(s_state_lock);

    if (!allowed) {
        return ESP_ERR_INVALID_STATE;
    }

    return s_driver_ops->get_status_info(out_status);
}

bool twai_port_is_bus_off(void)
{
    twai_status_info_t status = {0};
    if (twai_port_get_status(&status) != ESP_OK) {
        return false;
    }

    return status.state == TWAI_STATE_BUS_OFF;
}

esp_err_t twai_port_quiesce(TickType_t timeout_ticks)
{
    if (s_send_lock == NULL || s_state_lock == NULL || s_rx_idle_sem == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_initialized || s_state != TWAI_PORT_STATE_RUNNING) {
        xSemaphoreGive(s_state_lock);
        return ESP_ERR_INVALID_STATE;
    }

    s_state = TWAI_PORT_STATE_QUIESCED;
    xSemaphoreGive(s_state_lock);

    /*
     * Hold the send lock until resume(). This guarantees:
     * - no new sends enter the driver
     * - no new receives are admitted after the state flips to QUIESCED
     * - quiesce waits for any in-flight send/receive call to leave the driver
     */
    if (xSemaphoreTake(s_send_lock, timeout_ticks) != pdTRUE) {
        if (xSemaphoreTake(s_state_lock, portMAX_DELAY) == pdTRUE) {
            if (s_initialized && s_state == TWAI_PORT_STATE_QUIESCED) {
                s_state = TWAI_PORT_STATE_RUNNING;
            }
            xSemaphoreGive(s_state_lock);
        }
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = wait_for_receivers_to_drain(timeout_ticks);
    if (err != ESP_OK) {
        xSemaphoreGive(s_send_lock);
        if (xSemaphoreTake(s_state_lock, portMAX_DELAY) == pdTRUE) {
            if (s_initialized && s_state == TWAI_PORT_STATE_QUIESCED) {
                s_state = TWAI_PORT_STATE_RUNNING;
            }
            xSemaphoreGive(s_state_lock);
        }
        return err;
    }

    if (s_cfg.log_alerts) {
        ESP_LOGI(TAG, "quiesced");
    }

    return ESP_OK;
}

esp_err_t twai_port_resume(void)
{
    if (s_send_lock == NULL || s_state_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_initialized || s_state != TWAI_PORT_STATE_QUIESCED || s_recovery_in_progress) {
        xSemaphoreGive(s_state_lock);
        return ESP_ERR_INVALID_STATE;
    }

    s_state = TWAI_PORT_STATE_RUNNING;
    xSemaphoreGive(s_state_lock);

    xSemaphoreGive(s_send_lock);

    if (s_cfg.log_alerts) {
        ESP_LOGI(TAG, "resumed");
    }

    return ESP_OK;
}

esp_err_t twai_port_reconfigure_blocking(const twai_port_cfg_t *new_cfg,
                                         TickType_t timeout_ticks)
{
    esp_err_t err = validate_cfg(new_cfg);
    if (err != ESP_OK) {
        return err;
    }

    if (s_state_lock == NULL || s_send_lock == NULL || s_rx_idle_sem == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_cfg.log_alerts || new_cfg->log_alerts) {
        ESP_LOGI(TAG, "reconfiguring bitrate=%" PRIu32 " -> %" PRIu32,
                 s_initialized ? s_cfg.bitrate : 0U, new_cfg->bitrate);
    }

    bool release_send_lock = false;
    bool reconfigure_in_progress = false;
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    bool initialized = s_initialized;
    twai_port_state_t state = s_state;
    if (initialized && (state == TWAI_PORT_STATE_RUNNING || state == TWAI_PORT_STATE_QUIESCED)) {
        if (state == TWAI_PORT_STATE_QUIESCED &&
            xSemaphoreGetMutexHolder(s_send_lock) != current_task) {
            xSemaphoreGive(s_state_lock);
            return ESP_ERR_INVALID_STATE;
        }
        s_recovery_in_progress = true;
        reconfigure_in_progress = true;
    }
    xSemaphoreGive(s_state_lock);

    if (initialized && state == TWAI_PORT_STATE_RUNNING) {
        err = twai_port_quiesce(timeout_ticks);
        if (err != ESP_OK) {
            if (reconfigure_in_progress && xSemaphoreTake(s_state_lock, portMAX_DELAY) == pdTRUE) {
                s_recovery_in_progress = false;
                xSemaphoreGive(s_state_lock);
            }
            return err;
        }
        release_send_lock = true;
    } else if (initialized && state == TWAI_PORT_STATE_QUIESCED) {
        release_send_lock = true;
        err = wait_for_receivers_to_drain(timeout_ticks);
        if (err != ESP_OK) {
            if (reconfigure_in_progress && xSemaphoreTake(s_state_lock, portMAX_DELAY) == pdTRUE) {
                s_recovery_in_progress = false;
                xSemaphoreGive(s_state_lock);
            }
            return err;
        }
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        if (release_send_lock) {
            xSemaphoreGive(s_send_lock);
        }
        return ESP_ERR_TIMEOUT;
    }

    if (s_initialized) {
        if (s_state == TWAI_PORT_STATE_RUNNING ||
            s_state == TWAI_PORT_STATE_QUIESCED ||
            s_state == TWAI_PORT_STATE_RECOVERING) {
            err = s_driver_ops->stop();
            if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
                s_recovery_in_progress = false;
                xSemaphoreGive(s_state_lock);
                if (release_send_lock) {
                    xSemaphoreGive(s_send_lock);
                }
                return err;
            }
        }

        err = s_driver_ops->driver_uninstall();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            s_recovery_in_progress = false;
            xSemaphoreGive(s_state_lock);
            if (release_send_lock) {
                xSemaphoreGive(s_send_lock);
            }
            return err;
        }
    }

    err = install_driver_from_cfg(new_cfg);
    if (err != ESP_OK) {
        s_initialized = false;
        s_state = TWAI_PORT_STATE_UNINITIALIZED;
        s_recovery_in_progress = false;
        xSemaphoreGive(s_state_lock);
        if (release_send_lock) {
            xSemaphoreGive(s_send_lock);
        }
        return err;
    }

    s_cfg = *new_cfg;
    s_initialized = true;
    s_state = TWAI_PORT_STATE_STOPPED;
    s_recovery_in_progress = false;

    err = s_driver_ops->start();
    if (err != ESP_OK) {
        (void)s_driver_ops->driver_uninstall();
        s_initialized = false;
        s_state = TWAI_PORT_STATE_UNINITIALIZED;
        s_recovery_in_progress = false;
        xSemaphoreGive(s_state_lock);
        if (release_send_lock) {
            xSemaphoreGive(s_send_lock);
        }
        return err;
    }

    s_state = TWAI_PORT_STATE_RUNNING;
    s_recovery_in_progress = false;
    xSemaphoreGive(s_state_lock);

    if (release_send_lock) {
        xSemaphoreGive(s_send_lock);
    }

    if (s_cfg.log_alerts) {
        ESP_LOGI(TAG, "reconfigured and restarted at bitrate=%" PRIu32, s_cfg.bitrate);
    }

    return ESP_OK;
}

esp_err_t twai_port_set_bitrate_blocking(uint32_t bitrate,
                                         TickType_t timeout_ticks)
{
    if (s_state_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_state_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!s_initialized) {
        xSemaphoreGive(s_state_lock);
        return ESP_ERR_INVALID_STATE;
    }

    twai_port_cfg_t new_cfg = s_cfg;
    xSemaphoreGive(s_state_lock);

    new_cfg.bitrate = bitrate;
    return twai_port_reconfigure_blocking(&new_cfg, timeout_ticks);
}

void twai_port_test_set_driver_ops(const twai_port_driver_ops_t *ops)
{
    s_driver_ops = (ops != NULL) ? ops : &s_default_driver_ops;
}

void twai_port_test_reset_state(void)
{
    if (s_send_lock != NULL) {
        vSemaphoreDelete(s_send_lock);
        s_send_lock = NULL;
    }

    if (s_state_lock != NULL) {
        vSemaphoreDelete(s_state_lock);
        s_state_lock = NULL;
    }

    if (s_rx_idle_sem != NULL) {
        vSemaphoreDelete(s_rx_idle_sem);
        s_rx_idle_sem = NULL;
    }

    memset(&s_cfg, 0, sizeof(s_cfg));
    s_initialized = false;
    s_state = TWAI_PORT_STATE_UNINITIALIZED;
    s_recovery_in_progress = false;

    portENTER_CRITICAL(&s_rx_count_lock);
    s_active_receivers = 0;
    portEXIT_CRITICAL(&s_rx_count_lock);

    s_driver_ops = &s_default_driver_ops;
}
