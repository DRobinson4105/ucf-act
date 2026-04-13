/**
 * @file dac_mcp4728.cpp
 * @brief MCP4728 4-channel 12-bit I2C DAC driver implementation.
 */

#include "dac_mcp4728.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "rom/ets_sys.h"

namespace
{

const char *TAG = "DAC_MCP4728";
constexpr uint8_t CMD_MULTI_WRITE_CH_A = 0x40;
constexpr uint8_t CH_A_CONFIG = 0x00;

dac_mcp4728_config_t s_config = {};
bool s_initialized = false;
uint16_t s_current_level = 0;
bool s_autonomous = false;
i2c_master_bus_handle_t s_bus_handle = nullptr;
i2c_master_dev_handle_t s_dev_handle = nullptr;

void recover_bus(gpio_num_t sda, gpio_num_t scl)
{
    gpio_set_direction(scl, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(sda, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level(scl, 1);
    gpio_set_level(sda, 1);
    ets_delay_us(5);

    for (int i = 0; i < 9; ++i) {
        gpio_set_level(scl, 0);
        ets_delay_us(5);
        gpio_set_level(scl, 1);
        ets_delay_us(5);

        if (gpio_get_level(sda)) {
            break;
        }
    }

    gpio_set_level(scl, 0);
    ets_delay_us(5);
    gpio_set_level(sda, 0);
    ets_delay_us(5);
    gpio_set_level(scl, 1);
    ets_delay_us(5);
    gpio_set_level(sda, 1);
    ets_delay_us(5);

    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
}

void release_resources(void)
{
    if (s_dev_handle != nullptr) {
        i2c_master_bus_rm_device(s_dev_handle);
        s_dev_handle = nullptr;
    }
    if (s_bus_handle != nullptr) {
        i2c_del_master_bus(s_bus_handle);
        s_bus_handle = nullptr;
    }
    s_initialized = false;
    s_autonomous = false;
    s_current_level = 0;
}

esp_err_t write_channel_a(uint16_t level)
{
    if (s_dev_handle == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    if (level > DAC_MCP4728_LEVEL_MAX) {
        level = DAC_MCP4728_LEVEL_MAX;
    }

    uint8_t tx[3] = {
        CMD_MULTI_WRITE_CH_A,
        static_cast<uint8_t>(CH_A_CONFIG | ((level >> 8) & 0x0F)),
        static_cast<uint8_t>(level & 0xFF),
    };

    return i2c_master_transmit(s_dev_handle, tx, sizeof(tx), pdMS_TO_TICKS(100));
}

esp_err_t read_channel_a(uint16_t *out_level)
{
    if (s_dev_handle == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (out_level == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t rx[3] = {};
    esp_err_t err = i2c_master_receive(s_dev_handle, rx, sizeof(rx), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        return err;
    }

    *out_level = static_cast<uint16_t>(((rx[1] & 0x0F) << 8) | rx[2]);
    return ESP_OK;
}

} // namespace

esp_err_t dac_mcp4728_init(const dac_mcp4728_config_t *config)
{
    if (config == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_initialized || s_bus_handle != nullptr || s_dev_handle != nullptr) {
        release_resources();
    }

    s_config = *config;
    recover_bus(config->sda, config->scl);

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.i2c_port = config->i2c_port;
    bus_cfg.scl_io_num = config->scl;
    bus_cfg.sda_io_num = config->sda;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = config->device_addr;
    dev_cfg.scl_speed_hz = config->clk_speed_hz;

    err = i2c_master_bus_add_device(s_bus_handle, &dev_cfg, &s_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C add device failed: %s", esp_err_to_name(err));
        release_resources();
        return err;
    }

    err = write_channel_a(0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Initial write failed: %s", esp_err_to_name(err));
        release_resources();
        return err;
    }

    s_current_level = 0;
    s_autonomous = false;
    s_initialized = true;

    ESP_LOGI(TAG, "Initialized: sda=%d scl=%d addr=0x%02X", config->sda, config->scl, config->device_addr);
    return ESP_OK;
}

esp_err_t dac_mcp4728_set_level(uint16_t level)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (level > DAC_MCP4728_LEVEL_MAX) {
        level = DAC_MCP4728_LEVEL_MAX;
    }

    esp_err_t err = write_channel_a(level);
    if (err != ESP_OK) {
        return err;
    }

#ifdef CONFIG_LOG_ACTUATOR_DAC_WRITE
    ESP_LOGI(TAG, "DAC write: level=%u", level);
#endif

    uint16_t readback = 0xFFFF;
    err = read_channel_a(&readback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Readback failed: %s", esp_err_to_name(err));
        return err;
    }

    static uint16_t s_verify_count = 0;
    if (readback != level || (++s_verify_count % 50U) == 0U) {
        ESP_LOGI(TAG, "Level: wrote=%u read=%u %s", level, readback,
                 (readback == level) ? "OK" : "MISMATCH");
    }

    if (readback != level) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    s_current_level = level;
    return ESP_OK;
}

esp_err_t dac_mcp4728_disable(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = write_channel_a(0);
    if (err != ESP_OK) {
        return err;
    }

    s_current_level = 0;
    return ESP_OK;
}

esp_err_t dac_mcp4728_enable_autonomous(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = write_channel_a(0);
    if (err != ESP_OK) {
        return err;
    }

    s_current_level = 0;
    s_autonomous = true;

    ESP_LOGI(TAG, "Autonomous mode ENABLED");
    return ESP_OK;
}

esp_err_t dac_mcp4728_emergency_stop(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    (void)write_channel_a(0);
    s_current_level = 0;
    s_autonomous = false;

    ESP_LOGI(TAG, "EMERGENCY STOP");
    return ESP_OK;
}

uint16_t dac_mcp4728_get_level(void)
{
    return s_current_level;
}

bool dac_mcp4728_is_autonomous(void)
{
    return s_autonomous;
}
