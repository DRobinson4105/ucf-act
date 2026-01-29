#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "can_twai.hh"
#include "heartbeat.hh"
#include "mcp4728.hh"
#include "uim342.hh"

namespace {
// logging tag
static const char *kTag = "CONTROL";

// task and timing configuration
constexpr int kCanTaskStack = 4096;
constexpr int kMainTaskStack = 4096;
constexpr UBaseType_t kCanTaskPrio = 6;
constexpr UBaseType_t kMainTaskPrio = 5;
constexpr TickType_t kRxPollInterval = pdMS_TO_TICKS(20);
constexpr TickType_t kHeartbeatInterval = pdMS_TO_TICKS(500);

// TWAI (CAN) transceiver GPIO assignment
constexpr gpio_num_t kTwaiTxGpio = GPIO_NUM_5;
constexpr gpio_num_t kTwaiRxGpio = GPIO_NUM_4;

// I2C bus configuration for MCP4728 (DAC)
constexpr gpio_num_t kI2cSdaGpio = GPIO_NUM_10;
constexpr gpio_num_t kI2cSclGpio = GPIO_NUM_11;
constexpr uint32_t kI2cClockHz = 400000;
constexpr uint8_t kMcp4728Addr = 0x60;
constexpr uint8_t kThrottleDacChannel = 0;

// motor node IDs
constexpr uint8_t kSteeringMotorNodeId = 1;
constexpr uint8_t kBrakingMotorNodeId = 2;

// heartbeat LED
constexpr gpio_num_t kHeartbeatLedGpio = GPIO_NUM_8;

// i2c bus and dac handle
static i2c_master_bus_handle_t i2c_bus = nullptr;
static mcp4728_handle_t throttle_dac = {};

// can rx task: parses motor status frames and logs telemetry
void can_rx_task(void *param) {
    (void)param;
    twai_message_t msg{};

    while (true) {
        if (can_twai_receive(&msg, kRxPollInterval) == ESP_OK) {
            if (msg.extd || msg.rtr || msg.data_length_code != 8) continue;
            
            uint8_t node_id = 0;
            if (!motor_can_id_to_node(msg.identifier, &node_id)) continue;

            heartbeat_mark_activity(xTaskGetTickCount());
            
            MotorStatus status{};
            motor_parse_status_frame(&msg, &status);
            ESP_LOGI(kTag,
                     "Node %u: T=%.1fC, Speed=%d rpm, Current=%.2f A, Fault=0x%04X",
                     node_id,
                     static_cast<float>(status.temperature_c10) / 10.0f,
                     status.speed_rpm,
                     static_cast<float>(status.current_a100) / 100.0f,
                     status.fault_code);
        }
    }
}

// main task: initializes CAN, I2C, DAC, and enables motors
void main_task(void *param) {
    (void)param;

    ESP_LOGI(kTag, "Control ESP32 startup");
#ifdef CONFIG_IDF_TARGET
    ESP_LOGI(kTag, "Target: %s", CONFIG_IDF_TARGET);
#endif

    // initialize TWAI for 1 mbps, standard frames
    esp_err_t err = can_twai_init_default(kTwaiTxGpio, kTwaiRxGpio);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "TWAI init failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }

    // initialize I2C master bus for the throttle DAC
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = kI2cSdaGpio,
        .scl_io_num = kI2cSclGpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
            .allow_pd = 0,
        },
    };

    err = i2c_new_master_bus(&bus_cfg, &i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "I2C bus init failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }

    heartbeat_config_t heartbeat_cfg = {
        .gpio = kHeartbeatLedGpio,
        .interval_ticks = kHeartbeatInterval,
        .activity_window_ticks = pdMS_TO_TICKS(250),
        .active_high = true,
        .label = "control",
        .use_ws2812 = true,
        .ws2812_gpio = kHeartbeatLedGpio,
        .idle_red = 0,
        .idle_green = 16,
        .idle_blue = 0,
        .activity_red = 0,
        .activity_green = 0,
        .activity_blue = 16,
        .error_red = 16,
        .error_green = 0,
        .error_blue = 0,
    };

    err = heartbeat_init(&heartbeat_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(kTag, "Heartbeat init failed: %s", esp_err_to_name(err));
    }

    err = mcp4728_init(i2c_bus, kI2cClockHz, kMcp4728Addr, &throttle_dac);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "MCP4728 init failed: %s", esp_err_to_name(err));
        heartbeat_set_error(true);
        while (true) {
            heartbeat_tick(&heartbeat_cfg, xTaskGetTickCount());
            vTaskDelay(kHeartbeatInterval / 2);
        }
    }

    // default throttle output to 0 (safe state)
    mcp4728_set_channel(&throttle_dac, kThrottleDacChannel, 0);

    // start CAN RX processing task
    xTaskCreate(can_rx_task, "can_rx_task", kCanTaskStack, nullptr, kCanTaskPrio, nullptr);

    // initialization sequence: clear faults, then enable
    motor_clear_fault(kSteeringMotorNodeId);
    heartbeat_mark_activity(xTaskGetTickCount());
    motor_clear_fault(kBrakingMotorNodeId);
    heartbeat_mark_activity(xTaskGetTickCount());
    vTaskDelay(pdMS_TO_TICKS(10));
    motor_enable(kSteeringMotorNodeId);
    heartbeat_mark_activity(xTaskGetTickCount());
    motor_enable(kBrakingMotorNodeId);
    heartbeat_mark_activity(xTaskGetTickCount());

    ESP_LOGI(kTag, "TWAI initialized, motor control ready (DAC OK)");

    while (true) {
        heartbeat_tick(&heartbeat_cfg, xTaskGetTickCount());
        vTaskDelay(kHeartbeatInterval / 2);
    }
}
}

extern "C" void app_main(void) {
    xTaskCreate(main_task, "main_task", kMainTaskStack, nullptr, kMainTaskPrio, nullptr);
}
