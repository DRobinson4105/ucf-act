#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "heartbeat.hh"

#include "nc_mushroom.hh"
#include "safety_can.hh"
#include "ultrasonic.hh"
#include "wireless_remote.hh"

namespace {
// logging tag
static const char *kTag = "SAFETY";

// task and timing configuration
constexpr int kMainTaskStack = 4096;
constexpr UBaseType_t kMainTaskPrio = 5;
constexpr TickType_t kSafetyPollInterval = pdMS_TO_TICKS(100);
constexpr TickType_t kCanPollInterval = pdMS_TO_TICKS(5);
constexpr TickType_t kHeartbeatInterval = pdMS_TO_TICKS(500);

// GPIO assignments
constexpr gpio_num_t kNcMushroomGpio = GPIO_NUM_6;
constexpr int kNcMushroomActiveLevel = 1;

constexpr gpio_num_t kRemoteGpio = GPIO_NUM_7;
constexpr int kRemoteActiveLevel = 1;

// ultrasonic sensor UART configuration
constexpr uart_port_t kUltrasonicUart = UART_NUM_1;
constexpr int kUltrasonicTxGpio = UART_PIN_NO_CHANGE;
constexpr int kUltrasonicRxGpio = GPIO_NUM_9;
constexpr int kUltrasonicBaudRate = 9600;
constexpr uint16_t kUltrasonicStopDistanceMm = 300;

// TWAI pins
constexpr gpio_num_t kTwaiTxGpio = GPIO_NUM_5;
constexpr gpio_num_t kTwaiRxGpio = GPIO_NUM_4;

// CAN message identifiers
constexpr uint32_t kAppEstopMsgId = 0x120;
constexpr uint32_t kControlFaultMsgId = 0x121;
constexpr uint32_t kSafetyEstopMsgId = 0x122;

// heartbeat LED
constexpr gpio_num_t kHeartbeatLedGpio = GPIO_NUM_8;

// snapshot of current safety inputs
struct SafetyInputs {
    bool mushroom = false;
    bool remote = false;
    bool app = false;
    bool fault = false;
    bool obstacle = false;
};

// log changes in safety inputs
void log_input_change(const char *name, bool value) {
    ESP_LOGI(kTag, "%s changed -> %s", name, value ? "ACTIVE" : "CLEAR");
}

// main safety task: read inputs, parse CAN, publish E-stop state
void main_task(void *param) {
    (void)param;
    ESP_LOGI(kTag, "Safety ESP32 startup");
#ifdef CONFIG_IDF_TARGET
    ESP_LOGI(kTag, "Target: %s", CONFIG_IDF_TARGET);
#endif

    nc_mushroom_config_t mushroom_cfg = {
        .gpio = kNcMushroomGpio,
        .active_level = kNcMushroomActiveLevel,
        .enable_pullup = true,
        .enable_pulldown = false,
    };

    wireless_remote_config_t remote_cfg = {
        .gpio = kRemoteGpio,
        .active_level = kRemoteActiveLevel,
        .enable_pullup = true,
        .enable_pulldown = false,
    };

    safety_can_config_t can_cfg = {
        .tx_gpio = kTwaiTxGpio,
        .rx_gpio = kTwaiRxGpio,
        .app_estop_id = kAppEstopMsgId,
        .control_fault_id = kControlFaultMsgId,
        .safety_estop_id = kSafetyEstopMsgId,
    };

    ultrasonic_config_t ultrasonic_cfg = {
        .uart_num = kUltrasonicUart,
        .tx_gpio = kUltrasonicTxGpio,
        .rx_gpio = kUltrasonicRxGpio,
        .baud_rate = kUltrasonicBaudRate,
    };

    esp_err_t err = nc_mushroom_init(&mushroom_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "NC mushroom init failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }

    err = wireless_remote_init(&remote_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Wireless remote init failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }

    err = safety_can_init(&can_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "TWAI init failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }

    err = ultrasonic_init(&ultrasonic_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(kTag, "Ultrasonic init failed: %s", esp_err_to_name(err));
    }

    heartbeat_config_t heartbeat_cfg = {
        .gpio = kHeartbeatLedGpio,
        .interval_ticks = kHeartbeatInterval,
        .activity_window_ticks = pdMS_TO_TICKS(250),
        .active_high = true,
        .label = "safety",
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

    SafetyInputs last{};
    bool last_estop = false;
    TickType_t last_can_poll = xTaskGetTickCount();

    while (true) {
        SafetyInputs current = last;
        current.mushroom = nc_mushroom_read_active(&mushroom_cfg);
        current.remote = wireless_remote_is_active(&remote_cfg);

        if (xTaskGetTickCount() - last_can_poll >= kCanPollInterval) {
            twai_message_t msg{};
            while (safety_can_receive(&msg, 0) == ESP_OK) {
                safety_can_parse_frame(&can_cfg, &msg, &current.app, &current.fault);
                heartbeat_mark_activity(xTaskGetTickCount());
            }
            last_can_poll = xTaskGetTickCount();
        }

        uint16_t distance_mm = 0;
        current.obstacle = ultrasonic_is_too_close(kUltrasonicStopDistanceMm, &distance_mm);

        if (current.mushroom != last.mushroom) {
            log_input_change("NC mushroom", current.mushroom);
        }
        if (current.remote != last.remote) {
            log_input_change("Wireless remote", current.remote);
        }
        if (current.app != last.app) {
            log_input_change("App command", current.app);
        }
        if (current.fault != last.fault) {
            log_input_change("Control fault", current.fault);
        }
        if (current.obstacle != last.obstacle) {
            log_input_change("Ultrasonic obstacle", current.obstacle);
        }

        heartbeat_tick(&heartbeat_cfg, xTaskGetTickCount());

        bool estop_active = current.mushroom || current.remote || current.app || current.fault || current.obstacle;
        if (estop_active != last_estop) {
            ESP_LOGI(kTag, "Emergency stop state -> %s", estop_active ? "ACTIVE" : "CLEAR");
            safety_can_publish_estop(&can_cfg, estop_active, pdMS_TO_TICKS(50));
            heartbeat_mark_activity(xTaskGetTickCount());
            last_estop = estop_active;
        }

        last = current;
        vTaskDelay(kSafetyPollInterval);
    }
}
}

extern "C" void app_main(void) {
    xTaskCreate(main_task, "main_task", kMainTaskStack, nullptr, kMainTaskPrio, nullptr);
}
