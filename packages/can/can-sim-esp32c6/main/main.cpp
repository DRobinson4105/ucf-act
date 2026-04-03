/**
 * @file main.cpp
 * @brief CAN simulator — sends planner command frames (0x111) every 100 ms.
 *
 * Steering bounces back and forth between 0 and 720 in steps of 5, taking
 * ~14 seconds per sweep direction. Braking follows a triangle wave between 0
 * and 3 (3→2→1→0→1→2→3→…), holding each value for 5 seconds. Throttle is
 * always 0. Frame encoding matches the planner command wire format in
 * can_protocol.h.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#include "can_protocol.h"
#include "can_twai.h"

namespace
{

const char *TAG = "CAN_SIM";

// CAN bus (WAVESHARE SN65HVD230 transceiver)
constexpr gpio_num_t TWAI_TX_GPIO = GPIO_NUM_4;
constexpr gpio_num_t TWAI_RX_GPIO = GPIO_NUM_5;

// Steering bounces 0↔720; step=5 → 144 frames per direction ≈ 14.4 s
constexpr uint16_t STEERING_STEP = 5;
constexpr uint16_t STEERING_MAX  = 720;

// Braking triangle wave 3→0→3→…; each value held for 5 s (50 frames)
constexpr uint8_t  BRAKING_PEAK          = 3;
constexpr uint32_t BRAKING_HOLD_FRAMES   = 50;

} // namespace

extern "C" void app_main(void)
{
    esp_err_t err = can_twai_init_default(TWAI_TX_GPIO, TWAI_RX_GPIO);
    if (err == ESP_OK)
        ESP_LOGI(TAG, "TWAI ready: tx_gpio=%d rx_gpio=%d", TWAI_TX_GPIO, TWAI_RX_GPIO);
    else
        ESP_LOGE(TAG, "TWAI init failed (%s): tx_gpio=%d rx_gpio=%d", esp_err_to_name(err), TWAI_TX_GPIO, TWAI_RX_GPIO);

    node_seq_t  seq           = 0;
    uint16_t    steering      = 0;
    int         steering_dir  = 1;   // +1 = increasing, -1 = decreasing
    uint8_t     braking       = BRAKING_PEAK;
    int         braking_dir   = -1;  // start: 3→2→1→0
    uint32_t    braking_hold  = 0;   // frames elapsed at current braking value

    while (true)
    {
        uint8_t frame[8];
        planner_command_t cmd = {
            .sequence          = seq,
            .throttle          = 0,
            .steering_position = steering,
            .braking_position  = braking,
        };
        can_encode_planner_command(frame, &cmd);

        esp_err_t tx_err = can_twai_send(CAN_ID_PLANNER_COMMAND, frame, pdMS_TO_TICKS(10));
        if (tx_err != ESP_OK)
            ESP_LOGW(TAG, "TX failed: %s", esp_err_to_name(tx_err));

        ESP_LOGI(TAG, "TX seq=%u throttle=0 steering=%u braking=%u",
                 seq, steering, braking);

        seq++;

        // Steering: bounce between 0 and STEERING_MAX
        int next_steering = (int)steering + steering_dir * (int)STEERING_STEP;
        if (next_steering >= (int)STEERING_MAX)
        {
            steering     = STEERING_MAX;
            steering_dir = -1;
        }
        else if (next_steering <= 0)
        {
            steering     = 0;
            steering_dir = 1;
        }
        else
        {
            steering = (uint16_t)next_steering;
        }

        // Braking: triangle wave 3→2→1→0→1→2→3→2→…, 5 s per step
        braking_hold++;
        if (braking_hold >= BRAKING_HOLD_FRAMES)
        {
            braking_hold = 0;
            int next_braking = (int)braking + braking_dir;
            if (next_braking <= 0)
            {
                braking     = 0;
                braking_dir = 1;
            }
            else if (next_braking >= (int)BRAKING_PEAK)
            {
                braking     = BRAKING_PEAK;
                braking_dir = -1;
            }
            else
            {
                braking = (uint8_t)next_braking;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
