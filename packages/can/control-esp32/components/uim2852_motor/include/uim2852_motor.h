/**
 * @file uim2852_motor.h
 * @brief UIM2852CA Motor Control API
 *
 * High-level API for controlling UIM2852CA integrated servo stepper motors
 * over CAN 2.0B using the SimpleCAN protocol.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "driver/twai.h"
#include "esp_err.h"

#include "uim2852_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

typedef struct {
    uint8_t node_id;            // Motor's CAN node ID (default 5)
    uint8_t producer_id;        // Host controller ID (default 4)
    int32_t default_speed;      // Default PTP speed in pulses/sec
    uint32_t default_accel;     // Default acceleration in pulses/sec^2
    uint32_t default_decel;     // Default deceleration in pulses/sec^2
    uint32_t stop_decel;        // Emergency stop deceleration rate
    bool request_ack;           // Request ACK for commands (recommended)
} uim2852_motor_config_t;

// Default configuration
#define UIM2852_MOTOR_CONFIG_DEFAULT() { \
    .node_id = 5, \
    .producer_id = 4, \
    .default_speed = 3200, \
    .default_accel = 50000, \
    .default_decel = 50000, \
    .stop_decel = 200000, \
    .request_ack = true, \
}

// ----------------------------------------------------------------------------
// Motor State
// ----------------------------------------------------------------------------

typedef struct {
    uim2852_motor_config_t config;
    
    // Status from last MS[0] query
    uim2852_status_t status;
    
    // Speed and position from last MS[1] query
    int32_t current_speed;      // pulses/sec
    int32_t absolute_position;  // pulses
    
    // Tracking
    TickType_t last_response_tick;
    TickType_t last_command_tick;
    
    // Configuration (queried from motor)
    uint8_t microstep_resolution;  // From PP[5]
    int32_t pulses_per_rev;        // microstep * 200
    
    // Flags
    bool initialized;
    bool driver_enabled;
    bool motion_in_progress;
    bool ack_pending;
    uint8_t last_cw_sent;       // Last control word sent
} uim2852_motor_t;

// ----------------------------------------------------------------------------
// Initialization
// ----------------------------------------------------------------------------

/**
 * @brief Initialize motor structure with configuration
 * @param motor Motor instance
 * @param config Configuration (NULL for defaults)
 * @return ESP_OK on success
 */
esp_err_t uim2852_motor_init(uim2852_motor_t *motor, const uim2852_motor_config_t *config);

/**
 * @brief Query motor configuration (microstepping) and initialize parameters
 * @param motor Motor instance
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if motor doesn't respond
 * @note Call this after CAN bus is initialized
 */
esp_err_t uim2852_motor_configure(uim2852_motor_t *motor);

// ----------------------------------------------------------------------------
// Basic Control
// ----------------------------------------------------------------------------

/**
 * @brief Enable motor driver (MO=1)
 */
esp_err_t uim2852_motor_enable(uim2852_motor_t *motor);

/**
 * @brief Disable motor driver (MO=0)
 */
esp_err_t uim2852_motor_disable(uim2852_motor_t *motor);

/**
 * @brief Stop motion with deceleration (ST)
 */
esp_err_t uim2852_motor_stop(uim2852_motor_t *motor);

/**
 * @brief Emergency stop - high deceleration rate
 */
esp_err_t uim2852_motor_emergency_stop(uim2852_motor_t *motor);

// ----------------------------------------------------------------------------
// Position Control (PTP Mode)
// ----------------------------------------------------------------------------

/**
 * @brief Set speed for position moves
 * @param speed_pps Speed in pulses/sec (positive value, direction from position)
 */
esp_err_t uim2852_motor_set_speed(uim2852_motor_t *motor, int32_t speed_pps);

/**
 * @brief Set acceleration rate
 * @param accel_rate Acceleration in pulses/sec^2
 */
esp_err_t uim2852_motor_set_accel(uim2852_motor_t *motor, uint32_t accel_rate);

/**
 * @brief Set deceleration rate
 * @param decel_rate Deceleration in pulses/sec^2
 */
esp_err_t uim2852_motor_set_decel(uim2852_motor_t *motor, uint32_t decel_rate);

/**
 * @brief Move to absolute position
 * @param position Target position in pulses
 * @note Automatically calls BG to begin motion
 */
esp_err_t uim2852_motor_go_absolute(uim2852_motor_t *motor, int32_t position);

/**
 * @brief Move by relative displacement
 * @param displacement Distance to move in pulses (signed for direction)
 * @note Automatically calls BG to begin motion
 */
esp_err_t uim2852_motor_go_relative(uim2852_motor_t *motor, int32_t displacement);

/**
 * @brief Set current position as origin (zero)
 */
esp_err_t uim2852_motor_set_origin(uim2852_motor_t *motor);

// ----------------------------------------------------------------------------
// Status Query
// ----------------------------------------------------------------------------

/**
 * @brief Query motor status (MS[0] - flags and relative position)
 * @note Results stored in motor->status
 */
esp_err_t uim2852_motor_query_status(uim2852_motor_t *motor);

/**
 * @brief Query motor position and speed (MS[1])
 * @note Results stored in motor->current_speed and motor->absolute_position
 */
esp_err_t uim2852_motor_query_position(uim2852_motor_t *motor);

/**
 * @brief Clear status flags (PAIF, TLIF, ERR)
 */
esp_err_t uim2852_motor_clear_status(uim2852_motor_t *motor);

// ----------------------------------------------------------------------------
// Status Accessors
// ----------------------------------------------------------------------------

/**
 * @brief Check if motor is at target position (PAIF flag)
 */
static inline bool uim2852_motor_in_position(const uim2852_motor_t *motor) {
    return motor->status.in_position;
}

/**
 * @brief Check if motor is stopped
 */
static inline bool uim2852_motor_is_stopped(const uim2852_motor_t *motor) {
    return motor->status.stopped;
}

/**
 * @brief Check if stall was detected (TLIF flag)
 */
static inline bool uim2852_motor_stall_detected(const uim2852_motor_t *motor) {
    return motor->status.stall_detected;
}

/**
 * @brief Check if motor driver is enabled
 */
static inline bool uim2852_motor_is_enabled(const uim2852_motor_t *motor) {
    return motor->status.driver_on;
}

/**
 * @brief Check if any error flag is set
 */
static inline bool uim2852_motor_has_error(const uim2852_motor_t *motor) {
    return motor->status.error_detected;
}

/**
 * @brief Get current absolute position (from last query)
 */
static inline int32_t uim2852_motor_get_position(const uim2852_motor_t *motor) {
    return motor->absolute_position;
}

/**
 * @brief Get current speed (from last query)
 */
static inline int32_t uim2852_motor_get_speed(const uim2852_motor_t *motor) {
    return motor->current_speed;
}

// ----------------------------------------------------------------------------
// CAN Frame Processing
// ----------------------------------------------------------------------------

/**
 * @brief Process incoming CAN frame from motor
 * @param motor Motor instance
 * @param msg Received CAN message (must be extended frame)
 * @return true if frame was processed (was from this motor)
 */
bool uim2852_motor_process_frame(uim2852_motor_t *motor, const twai_message_t *msg);

// ----------------------------------------------------------------------------
// Notification Callback
// ----------------------------------------------------------------------------

/**
 * @brief Notification callback type
 * @param motor Motor that generated the notification
 * @param notif Notification details
 */
typedef void (*uim2852_motor_notify_cb_t)(uim2852_motor_t *motor, 
                                           const uim2852_notification_t *notif);

/**
 * @brief Set notification callback for a motor
 */
void uim2852_motor_set_notify_callback(uim2852_motor_t *motor, 
                                        uim2852_motor_notify_cb_t callback);

// ----------------------------------------------------------------------------
// Low-Level Access
// ----------------------------------------------------------------------------

/**
 * @brief Send raw instruction to motor
 * @param motor Motor instance
 * @param cw Control word (mnemonic)
 * @param data Data bytes
 * @param dl Data length
 * @return ESP_OK on successful transmit
 */
esp_err_t uim2852_motor_send_instruction(uim2852_motor_t *motor, uint8_t cw,
                                          const uint8_t *data, uint8_t dl);

/**
 * @brief Query a parameter value
 * @param motor Motor instance
 * @param cw Parameter control word (PP, IC, IE, LM, QE)
 * @param index Parameter index
 * @param value Output value (NULL to skip)
 * @return ESP_OK on success
 */
esp_err_t uim2852_motor_query_param(uim2852_motor_t *motor, uint8_t cw, 
                                     uint8_t index, int32_t *value);

/**
 * @brief Set a parameter value
 * @param motor Motor instance
 * @param cw Parameter control word (PP, IC, IE, LM, QE)
 * @param index Parameter index
 * @param value Value to set
 * @return ESP_OK on success
 */
esp_err_t uim2852_motor_set_param(uim2852_motor_t *motor, uint8_t cw,
                                   uint8_t index, int32_t value);

#ifdef __cplusplus
}
#endif
