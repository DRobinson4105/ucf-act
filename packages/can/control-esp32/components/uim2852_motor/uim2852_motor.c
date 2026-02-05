/**
 * @file uim2852_motor.c
 * @brief UIM2852CA Motor Control Implementation
 */

#include "uim2852_motor.h"

#include <string.h>

#include "esp_log.h"
#include "can_twai.hh"

static const char *TAG = "UIM2852";

// Default timeout for CAN transmit
static const TickType_t TX_TIMEOUT = pdMS_TO_TICKS(20);

// Notification callback (per-motor, stored externally for now)
static uim2852_motor_notify_cb_t s_notify_callback = NULL;

// ----------------------------------------------------------------------------
// Internal Helpers
// ----------------------------------------------------------------------------

static esp_err_t send_instruction(uim2852_motor_t *motor, uint8_t cw, 
                                   const uint8_t *data, uint8_t dl) {
    if (!motor || !motor->initialized) return ESP_ERR_INVALID_STATE;
    
    // Add ACK bit if configured
    uint8_t cw_to_send = motor->config.request_ack ? (cw | UIM2852_CW_ACK_BIT) : cw;
    
    // Calculate 29-bit CAN ID
    uint32_t can_id = uim2852_make_can_id(motor->config.node_id, cw_to_send);
    
    // Send extended frame
    esp_err_t err = can_twai_send_extended(can_id, data, dl, TX_TIMEOUT);
    
    if (err == ESP_OK) {
        motor->last_command_tick = xTaskGetTickCount();
        motor->last_cw_sent = cw;
        if (motor->config.request_ack) motor->ack_pending = true;
    } else
        ESP_LOGW(TAG, "Node %u: TX failed for CW=0x%02X: %s", 
            motor->config.node_id, cw, esp_err_to_name(err));
    
    return err;
}

// ----------------------------------------------------------------------------
// Initialization
// ----------------------------------------------------------------------------

esp_err_t uim2852_motor_init(uim2852_motor_t *motor, const uim2852_motor_config_t *config) {
    if (!motor) return ESP_ERR_INVALID_ARG;
    
    memset(motor, 0, sizeof(uim2852_motor_t));
    
    if (config) motor->config = *config;
    else {
        uim2852_motor_config_t defaults = UIM2852_MOTOR_CONFIG_DEFAULT();
        motor->config = defaults;
    }
    
    motor->initialized = true;
    motor->microstep_resolution = 16;  // Assume 16 until queried
    motor->pulses_per_rev = 3200;      // 16 * 200
    
    ESP_LOGI(TAG, "Motor initialized: node_id=%u, producer_id=%u", 
        motor->config.node_id, motor->config.producer_id);
    
    return ESP_OK;
}

esp_err_t uim2852_motor_configure(uim2852_motor_t *motor) {
    if (!motor || !motor->initialized) return ESP_ERR_INVALID_STATE;
    
    int32_t microstep = 0;
    esp_err_t err = uim2852_motor_query_param(motor, UIM2852_CW_PP, 
                                               UIM2852_PP_MICROSTEP, &microstep);
    
    if (err == ESP_OK && microstep > 0) {
        motor->microstep_resolution = (uint8_t)microstep;
        motor->pulses_per_rev = microstep * 200;
        ESP_LOGI(TAG, "Node %u: microstep=%d, pulses_per_rev=%ld",
            motor->config.node_id, motor->microstep_resolution, 
            (long)motor->pulses_per_rev);
    } else
        ESP_LOGW(TAG, "Node %u: Failed to query microstep, using default 16",
            motor->config.node_id);
    
    // Set default motion parameters
    err = uim2852_motor_set_speed(motor, motor->config.default_speed);
    if (err != ESP_OK) return err;
    
    err = uim2852_motor_set_accel(motor, motor->config.default_accel);
    if (err != ESP_OK) return err;
    
    err = uim2852_motor_set_decel(motor, motor->config.default_decel);
    if (err != ESP_OK) return err;
    
    // Set emergency stop deceleration
    uint8_t data[8];
    uint8_t dl = uim2852_build_sd(data, motor->config.stop_decel);
    err = send_instruction(motor, UIM2852_CW_SD, data, dl);
    
    return err;
}

// ----------------------------------------------------------------------------
// Basic Control
// ----------------------------------------------------------------------------

esp_err_t uim2852_motor_enable(uim2852_motor_t *motor) {
    uint8_t data[8];
    uint8_t dl = uim2852_build_mo(data, true);
    
    esp_err_t err = send_instruction(motor, UIM2852_CW_MO, data, dl);
    if (err == ESP_OK) {
        motor->driver_enabled = true;
        ESP_LOGI(TAG, "Node %u: Motor enabled", motor->config.node_id);
    }
    return err;
}

esp_err_t uim2852_motor_disable(uim2852_motor_t *motor) {
    uint8_t data[8];
    uint8_t dl = uim2852_build_mo(data, false);
    
    esp_err_t err = send_instruction(motor, UIM2852_CW_MO, data, dl);
    if (err == ESP_OK) {
        motor->driver_enabled = false;
        motor->motion_in_progress = false;
        ESP_LOGI(TAG, "Node %u: Motor disabled", motor->config.node_id);
    }
    return err;
}

esp_err_t uim2852_motor_stop(uim2852_motor_t *motor) {
    uint8_t data[8];
    uint8_t dl = uim2852_build_st(data);
    
    esp_err_t err = send_instruction(motor, UIM2852_CW_ST, data, dl);
    if (err == ESP_OK) {
        motor->motion_in_progress = false;
        ESP_LOGD(TAG, "Node %u: Stop commanded", motor->config.node_id);
    }
    return err;
}

esp_err_t uim2852_motor_emergency_stop(uim2852_motor_t *motor) {
    // First set high stop deceleration, then stop
    uint8_t data[8];
    uint8_t dl;
    
    // Set very high SD rate for emergency
    dl = uim2852_build_sd(data, motor->config.stop_decel);
    send_instruction(motor, UIM2852_CW_SD, data, dl);
    
    // Issue stop command
    dl = uim2852_build_st(data);
    esp_err_t err = send_instruction(motor, UIM2852_CW_ST, data, dl);
    
    if (err == ESP_OK) {
        motor->motion_in_progress = false;
        ESP_LOGW(TAG, "Node %u: Emergency stop!", motor->config.node_id);
    }
    return err;
}

// ----------------------------------------------------------------------------
// Position Control
// ----------------------------------------------------------------------------

esp_err_t uim2852_motor_set_speed(uim2852_motor_t *motor, int32_t speed_pps) {
    uint8_t data[8];
    uint8_t dl = uim2852_build_sp(data, speed_pps);
    return send_instruction(motor, UIM2852_CW_SP, data, dl);
}

esp_err_t uim2852_motor_set_accel(uim2852_motor_t *motor, uint32_t accel_rate) {
    uint8_t data[8];
    uint8_t dl = uim2852_build_ac(data, accel_rate);
    return send_instruction(motor, UIM2852_CW_AC, data, dl);
}

esp_err_t uim2852_motor_set_decel(uim2852_motor_t *motor, uint32_t decel_rate) {
    uint8_t data[8];
    uint8_t dl = uim2852_build_dc(data, decel_rate);
    return send_instruction(motor, UIM2852_CW_DC, data, dl);
}

esp_err_t uim2852_motor_go_absolute(uim2852_motor_t *motor, int32_t position) {
    uint8_t data[8];
    uint8_t dl;
    esp_err_t err;
    
    // Set absolute position
    dl = uim2852_build_pa(data, position);
    err = send_instruction(motor, UIM2852_CW_PA, data, dl);
    if (err != ESP_OK) return err;
    
    // Small delay to ensure PA is processed
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Begin motion
    dl = uim2852_build_bg(data);
    err = send_instruction(motor, UIM2852_CW_BG, data, dl);
    
    if (err == ESP_OK) {
        motor->motion_in_progress = true;
        ESP_LOGD(TAG, "Node %u: Moving to PA=%ld", motor->config.node_id, (long)position);
    }
    return err;
}

esp_err_t uim2852_motor_go_relative(uim2852_motor_t *motor, int32_t displacement) {
    uint8_t data[8];
    uint8_t dl;
    esp_err_t err;
    
    // Set relative position
    dl = uim2852_build_pr(data, displacement);
    err = send_instruction(motor, UIM2852_CW_PR, data, dl);
    if (err != ESP_OK) return err;
    
    // Small delay to ensure PR is processed
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Begin motion
    dl = uim2852_build_bg(data);
    err = send_instruction(motor, UIM2852_CW_BG, data, dl);
    
    if (err == ESP_OK) {
        motor->motion_in_progress = true;
        ESP_LOGD(TAG, "Node %u: Moving PR=%ld", motor->config.node_id, (long)displacement);
    }
    return err;
}

esp_err_t uim2852_motor_set_origin(uim2852_motor_t *motor) {
    uint8_t data[8];
    uint8_t dl = uim2852_build_og(data);
    
    esp_err_t err = send_instruction(motor, UIM2852_CW_OG, data, dl);
    if (err == ESP_OK) {
        motor->absolute_position = 0;
        ESP_LOGI(TAG, "Node %u: Origin set", motor->config.node_id);
    }
    return err;
}

// ----------------------------------------------------------------------------
// Status Query
// ----------------------------------------------------------------------------

esp_err_t uim2852_motor_query_status(uim2852_motor_t *motor) {
    uint8_t data[8];
    uint8_t dl = uim2852_build_ms(data, UIM2852_MS_FLAGS_RELPOS);
    return send_instruction(motor, UIM2852_CW_MS, data, dl);
}

esp_err_t uim2852_motor_query_position(uim2852_motor_t *motor) {
    uint8_t data[8];
    uint8_t dl = uim2852_build_ms(data, UIM2852_MS_SPEED_ABSPOS);
    return send_instruction(motor, UIM2852_CW_MS, data, dl);
}

esp_err_t uim2852_motor_clear_status(uim2852_motor_t *motor) {
    uint8_t data[8];
    uint8_t dl = uim2852_build_ms_clear(data);
    return send_instruction(motor, UIM2852_CW_MS, data, dl);
}

// ----------------------------------------------------------------------------
// CAN Frame Processing
// ----------------------------------------------------------------------------

bool uim2852_motor_process_frame(uim2852_motor_t *motor, const twai_message_t *msg) {
    if (!motor || !motor->initialized || !msg) return false;
    
    // Must be extended frame
    if (!msg->extd) return false;
    
    // Parse CAN ID
    uint8_t producer_id, cw;
    if (!uim2852_parse_can_id(msg->identifier, &producer_id, &cw)) return false;
    
    // Check if this frame is from our motor
    if (producer_id != motor->config.node_id) return false;
    
    // Update response timestamp
    motor->last_response_tick = xTaskGetTickCount();
    motor->ack_pending = false;
    
    uint8_t cw_base = uim2852_cw_base(cw);
    uint8_t dl = msg->data_length_code;
    const uint8_t *data = msg->data;
    
    // Handle different response types
    switch (cw_base) {
        case UIM2852_CW_MS:
            // Motion status response
            if (dl >= 1) {
                uint8_t index = data[0];
                if (index == UIM2852_MS_FLAGS_RELPOS) {
                    uim2852_parse_ms0(data, dl, &motor->status);
                    
                    // Update motion state from flags
                    motor->driver_enabled = motor->status.driver_on;
                    if (motor->status.in_position || motor->status.stopped)
                        motor->motion_in_progress = false;
                    
                    ESP_LOGD(TAG, "Node %u: MS[0] drv=%d stop=%d inpos=%d stall=%d",
                        motor->config.node_id,
                        motor->status.driver_on,
                        motor->status.stopped,
                        motor->status.in_position,
                        motor->status.stall_detected);
                } else if (index == UIM2852_MS_SPEED_ABSPOS) {
                    uim2852_parse_ms1(data, dl, &motor->current_speed, 
                        &motor->absolute_position);
                    
                    ESP_LOGD(TAG, "Node %u: MS[1] speed=%ld pos=%ld",
                        motor->config.node_id,
                        (long)motor->current_speed,
                        (long)motor->absolute_position);
                }
            }
            break;
            
        case UIM2852_CW_NOTIFY:
            // Real-time notification
            {
                uim2852_notification_t notif = {0};
                notif.node_id = motor->config.node_id;
                
                if (uim2852_parse_notification(data, dl, &notif)) {
                    // Handle specific notifications
                    if (notif.type == UIM2852_STATUS_PTP_COMPLETE) {
                        motor->motion_in_progress = false;
                        motor->status.in_position = true;
                        motor->absolute_position = notif.position;
                        ESP_LOGI(TAG, "Node %u: PTP complete at pos=%ld",
                            motor->config.node_id, (long)notif.position);
                    } else if (notif.type == UIM2852_ALARM_STALL) {
                        motor->status.stall_detected = true;
                        motor->motion_in_progress = false;
                        ESP_LOGW(TAG, "Node %u: STALL DETECTED!", motor->config.node_id);
                    } else if (notif.is_alarm)
                        ESP_LOGW(TAG, "Node %u: Alarm 0x%02X", 
                            motor->config.node_id, notif.type);
                    
                    // Call user callback if set
                    if (s_notify_callback) s_notify_callback(motor, &notif);
                }
            }
            break;
            
        case UIM2852_CW_ER:
            // Error report
            {
                uim2852_error_t error = {0};
                if (uim2852_parse_error(data, dl, &error)) {
                    motor->status.error_detected = true;
                    ESP_LOGE(TAG, "Node %u: Error 0x%02X on CW=0x%02X[%u]",
                        motor->config.node_id,
                        error.error_code,
                        error.related_cw,
                        error.subindex);
                }
            }
            break;
            
        case UIM2852_CW_MO:
            // ACK for motor enable/disable
            if (dl >= 1) motor->driver_enabled = (data[0] != 0);
            break;
        case UIM2852_CW_PP:
        case UIM2852_CW_IC:
        case UIM2852_CW_IE:
        case UIM2852_CW_LM:
        case UIM2852_CW_QE:
            // Parameter response - handled by query_param
            break;
            
        default:
            // ACK for other commands
            ESP_LOGD(TAG, "Node %u: ACK for CW=0x%02X", motor->config.node_id, cw_base);
            break;
    }
    
    return true;
}

// ----------------------------------------------------------------------------
// Notification Callback
// ----------------------------------------------------------------------------

void uim2852_motor_set_notify_callback(uim2852_motor_t *motor, 
                                        uim2852_motor_notify_cb_t callback) {
    (void)motor;  // Currently global callback
    s_notify_callback = callback;
}

// ----------------------------------------------------------------------------
// Low-Level Access
// ----------------------------------------------------------------------------

esp_err_t uim2852_motor_send_instruction(uim2852_motor_t *motor, uint8_t cw,
                                          const uint8_t *data, uint8_t dl) {
    return send_instruction(motor, cw, data, dl);
}

esp_err_t uim2852_motor_query_param(uim2852_motor_t *motor, uint8_t cw, 
                                     uint8_t index, int32_t *value) {
    if (!motor || !motor->initialized) return ESP_ERR_INVALID_STATE;
    
    uint8_t data[8] = {0};
    data[0] = index;
    
    esp_err_t err = send_instruction(motor, cw, data, 1);
    if (err != ESP_OK) return err;
    
    // Note: Response will be processed asynchronously via process_frame
    // For synchronous operation, caller should wait and check motor state
    // This is a simplified implementation
    
    if (value) *value = 0;  // Caller should wait for response and re-check
    
    return ESP_OK;
}

esp_err_t uim2852_motor_set_param(uim2852_motor_t *motor, uint8_t cw,
                                   uint8_t index, int32_t value) {
    if (!motor || !motor->initialized) return ESP_ERR_INVALID_STATE;
    
    uint8_t data[8] = {0};
    data[0] = index;
    
    // Pack 32-bit value in little-endian
    data[1] = (uint8_t)(value & 0xFF);
    data[2] = (uint8_t)((value >> 8) & 0xFF);
    data[3] = (uint8_t)((value >> 16) & 0xFF);
    data[4] = (uint8_t)((value >> 24) & 0xFF);
    
    return send_instruction(motor, cw, data, 5);
}
