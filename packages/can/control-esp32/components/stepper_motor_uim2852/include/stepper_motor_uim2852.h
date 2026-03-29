/**
 * @file stepper_motor_uim2852.h
 * @brief UIM2852CA Stepper Motor Control API
 *
 * High-level API for controlling UIM2852CA integrated servo stepper motors
 * over CAN 2.0B using the SimpleCAN protocol.
 *
 * ## Safe Power-On Sequence
 *
 * The motor driver can be in an unknown state when 24V power is applied
 * (hot-start scenario: ESP32 rebooted while relay was on, or motor flash
 * has auto-enable IC[0]=1 from factory). The required init sequence is:
 *
 *   1. CAN bus must be initialized and running (TWAI driver started).
 *   2. **MO=0** (driver off) — immediately de-energize coils.
 *   3. **IC[0]=0** (disable auto-enable) — prevent driver from enabling
 *      on future power cycles before host configures the motor.
 *   4. Configure motion parameters: SP, AC, DC, SD.
 *   5. Configure software limits: LM[1], LM[2], IC[7]=1.
 *   6. **MO=0** (redundant disable) — ensure driver stays off.
 *
 * Steps 2-6 are performed by init_stepper_checked() in main.cpp.
 * The ENABLE transition later performs:
 *
 *   7. **MO=1** (driver enable)
 *   8. Begin accepting position commands.
 *
 * The DISABLE / E-STOP sequence reverses this:
 *
 *   9. **ST** (deceleration stop)
 *  10. **MO=0** (driver off)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

#include "freertos/FreeRTOS.h"
#include "driver/twai.h"
#include "esp_err.h"

#include "stepper_protocol_uim2852.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// Configuration
// ============================================================================

typedef struct
{
	uint8_t node_id;          // Motor's CAN node ID — used as consumer ID when sending commands
	uint8_t producer_id;      // Motor's response producer ID — used to match incoming frames
	uint32_t default_accel;   // Default acceleration in pulses/sec^2
	uint32_t default_decel;   // Default deceleration in pulses/sec^2
	uint32_t stop_decel;      // Emergency stop deceleration rate
	uint16_t working_current; // Working current in 0.1A units (e.g. 8 = 0.8A)
	bool request_ack;         // Request ACK for commands (recommended)
} stepper_motor_uim2852_config_t;

// Default configuration
#define STEPPER_MOTOR_UIM2852_CONFIG_DEFAULT() \
	{                                          \
		.node_id = 5,                          \
		.producer_id = 1,                      \
		.default_accel = 5000,                 \
		.default_decel = 5000,                 \
		.stop_decel = 8000,                    \
		.working_current = 8,                  \
		.request_ack = true,                   \
	}

// ============================================================================
// Motor State
// ============================================================================

typedef struct stepper_motor_uim2852
{
	stepper_motor_uim2852_config_t config;

	// Status from last MS[0] query
	stepper_uim2852_status_t status;

	// Last error reported by motor (valid when status.error_detected is true)
	stepper_uim2852_error_t last_error;

	// Speed and position from last MS[1] query
	int32_t current_speed;     // pulses/sec
	int32_t absolute_position; // pulses
	int32_t target_position;   // last commanded absolute position (from pt_feed)

	// Tracking
	TickType_t last_response_tick;
	TickType_t last_command_tick;

	// Configuration (queried from motor)
	uint16_t microstep_resolution; // From PP[5] (uint16_t to support 256 microsteps)
	int32_t pulses_per_rev;        // microstep * 200

	// Flags
	bool initialized;
	bool driver_enabled;
	bool motion_in_progress;
	bool ack_pending;
	uint8_t last_cw_sent; // Last control word sent

	// PT (Position-Time) interpolation mode state
	bool pt_mode_active;       // true after PT FIFO mode is armed with PV=start_row
	bool pt_motion_started;    // true after BG has been sent for the current PT FIFO run
	bool pt_fifo_empty;        // set by PVT FIFO empty notification (0x2A)
	bool pt_fifo_low;          // set by PVT FIFO low warning notification (0x2B)
	uint16_t pt_frame_time_ms; // fixed PT frame time configured via MP[4]
	uint16_t pt_write_index;   // next PT table row to write (0-511)
	uint8_t pt_prefill_count;  // number of PT rows queued before BG start
	uint8_t pt_low_water_mark; // queue-low threshold configured via MP[5]

	// Per-motor notification callback (set via stepper_motor_uim2852_set_notify_callback)
	// Forward-declared as void* to avoid circular typedef; actual type matches
	// stepper_motor_uim2852_notify_cb_t which takes (stepper_motor_uim2852_t *, notif *).
	void (*notify_callback)(struct stepper_motor_uim2852 *motor, const stepper_uim2852_notification_t *notif);

	// Synchronous query support (used by query_param to block until response)
	SemaphoreHandle_t query_sem; // Binary semaphore signaled when param response arrives
	uint8_t query_pending_cw;    // CW of the pending query (0 = none)
	uint8_t query_pending_idx;   // Subindex of the pending query
	int32_t query_result;        // Value parsed from param response

	// Thread-safety: protects status/motion_in_progress/absolute_position
	// which are written by process_frame (CAN RX task) and read by control task.
	portMUX_TYPE lock;
} stepper_motor_uim2852_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize motor structure with configuration
 * @param motor Motor instance
 * @param config Configuration (NULL for defaults)
 * @return ESP_OK on success
 * @note Safe to call on a previously initialized motor — internally deinits first.
 */
esp_err_t stepper_motor_uim2852_init(stepper_motor_uim2852_t *motor, const stepper_motor_uim2852_config_t *config);

/**
 * @brief Deinitialize motor structure and free resources
 *
 * Releases the query semaphore and marks the motor as uninitialized.
 * Safe to call on an already-deinitialized or zero-initialized motor.
 *
 * @param motor Motor instance
 */
void stepper_motor_uim2852_deinit(stepper_motor_uim2852_t *motor);

/**
 * @brief Query motor configuration (microstepping) and initialize parameters
 * @param motor Motor instance
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if motor doesn't respond
 * @note Call this after CAN bus is initialized
 */
esp_err_t stepper_motor_uim2852_configure(stepper_motor_uim2852_t *motor);

/**
 * @brief Program hardware software-limits (LM) on the motor controller.
 *
 * Sets lower/upper working limits and enables the limit system.
 * These act as a secondary safety net — even if the host sends an
 * out-of-range position, the motor controller will reject it.
 *
 * @param motor       Motor instance
 * @param lower_limit Lower working limit in pulses (signed)
 * @param upper_limit Upper working limit in pulses (signed)
 * @return ESP_OK on success
 */
esp_err_t stepper_motor_uim2852_set_limits(stepper_motor_uim2852_t *motor, int32_t lower_limit, int32_t upper_limit);

// ============================================================================
// Basic Control
// ============================================================================

/**
 * @brief Enable motor driver (MO=1)
 */
esp_err_t stepper_motor_uim2852_enable(stepper_motor_uim2852_t *motor);

/**
 * @brief Disable motor driver (MO=0)
 */
esp_err_t stepper_motor_uim2852_disable(stepper_motor_uim2852_t *motor);

/**
 * @brief Stop motion with deceleration (ST)
 */
esp_err_t stepper_motor_uim2852_stop(stepper_motor_uim2852_t *motor);

/**
 * @brief Emergency stop - high deceleration rate
 */
esp_err_t stepper_motor_uim2852_emergency_stop(stepper_motor_uim2852_t *motor);

// ============================================================================
// Motion Parameters
// ============================================================================

/**
 * @brief Set acceleration rate
 * @param accel_rate Acceleration in pulses/sec^2
 */
esp_err_t stepper_motor_uim2852_set_accel(stepper_motor_uim2852_t *motor, uint32_t accel_rate);

/**
 * @brief Set deceleration rate
 * @param decel_rate Deceleration in pulses/sec^2
 */
esp_err_t stepper_motor_uim2852_set_decel(stepper_motor_uim2852_t *motor, uint32_t decel_rate);

/**
 * @brief Set current position as origin (zero)
 */
esp_err_t stepper_motor_uim2852_set_origin(stepper_motor_uim2852_t *motor);

// ============================================================================
// Position-Time (PT) Interpolation Mode
// ============================================================================
// PT mode queues absolute position waypoints in the motor's internal FIFO.
// The motor uses MP[4] as the fixed segment time for every PT row.
// Each PT frame interval feeds one PT row via pt_feed().

/**
 * @brief Configure PT FIFO mode and enable FIFO notifications.
 *
 * Resets the PT/PVT queue, selects FIFO mode, programs the fixed PT frame
 * time (MP[4]), sets a queue-low threshold (MP[5]), and enables the FIFO
 * empty / low notifications. Call once during enable sequence, before
 * pt_start().
 *
 * @param motor Motor instance
 * @return ESP_OK on success
 */
esp_err_t stepper_motor_uim2852_pt_configure(stepper_motor_uim2852_t *motor);

/**
 * @brief Arm PT FIFO mode at row 0.
 *
 * Sends PV=0 to select the starting PT row.  Actual motion begins later
 * when pt_feed() has preloaded a small number of PT rows and issues BG.
 *
 * @param motor Motor instance
 * @return ESP_OK on success
 */
esp_err_t stepper_motor_uim2852_pt_start(stepper_motor_uim2852_t *motor);

/**
 * @brief Stop PT interpolation with a deceleration stop.
 *
 * Issues ST, clears PT FIFO tracking state, and marks the PT stream as
 * inactive in software.
 *
 * @param motor Motor instance
 * @return ESP_OK on success
 */
esp_err_t stepper_motor_uim2852_pt_stop(stepper_motor_uim2852_t *motor);

/**
 * @brief Feed a PT row into the PT FIFO table.
 *
 * Writes the next PT table row using the desired absolute position.  The
 * time argument is retained for API compatibility and must match the fixed
 * PT frame time configured via MP[4].  After a small startup prefill, the
 * function automatically sends BG once to start the PT engine.
 *
 * @param motor    Motor instance
 * @param position Absolute target position in pulses (signed)
 * @param time_ms  Time to reach position in milliseconds
 * @return ESP_OK on success
 */
esp_err_t stepper_motor_uim2852_pt_feed(stepper_motor_uim2852_t *motor, int32_t position, uint32_t time_ms);

// ============================================================================
// Status Query
// ============================================================================

/**
 * @brief Query motor status (MS[0] - flags and relative position)
 * @note Results stored in motor->status
 */
esp_err_t stepper_motor_uim2852_query_status(stepper_motor_uim2852_t *motor);

/**
 * @brief Query motor position and speed (MS[1])
 * @note Results stored in motor->current_speed and motor->absolute_position
 */
esp_err_t stepper_motor_uim2852_query_position(stepper_motor_uim2852_t *motor);

/**
 * @brief Clear status flags (PAIF, TLIF, ERR)
 */
esp_err_t stepper_motor_uim2852_clear_status(stepper_motor_uim2852_t *motor);

// ============================================================================
// Status Accessors
// ============================================================================

/**
 * @brief Check if motor is at target position (PAIF flag)
 */
static inline bool stepper_motor_uim2852_in_position(const stepper_motor_uim2852_t *motor)
{
	return motor->status.in_position;
}

/**
 * @brief Check if motor is stopped
 */
static inline bool stepper_motor_uim2852_is_stopped(const stepper_motor_uim2852_t *motor)
{
	return motor->status.stopped;
}

/**
 * @brief Check if stall was detected (TLIF flag)
 */
static inline bool stepper_motor_uim2852_stall_detected(const stepper_motor_uim2852_t *motor)
{
	return motor->status.stall_detected;
}

/**
 * @brief Check if motor driver is enabled
 */
static inline bool stepper_motor_uim2852_is_enabled(const stepper_motor_uim2852_t *motor)
{
	return motor->status.driver_on;
}

/**
 * @brief Check if any error flag is set
 */
static inline bool stepper_motor_uim2852_has_error(const stepper_motor_uim2852_t *motor)
{
	return motor->status.error_detected;
}

/**
 * @brief Get current absolute position (from last query)
 */
static inline int32_t stepper_motor_uim2852_get_position(const stepper_motor_uim2852_t *motor)
{
	return motor->absolute_position;
}

/**
 * @brief Get current speed (from last query)
 */
static inline int32_t stepper_motor_uim2852_get_speed(const stepper_motor_uim2852_t *motor)
{
	return motor->current_speed;
}

/**
 * @brief Get last commanded target position (from pt_feed)
 */
static inline int32_t stepper_motor_uim2852_get_target_position(const stepper_motor_uim2852_t *motor)
{
	return motor->target_position;
}

/**
 * @brief Check if motion is currently in progress
 */
static inline bool stepper_motor_uim2852_is_motion_in_progress(const stepper_motor_uim2852_t *motor)
{
	if (!motor)
		return false;
	stepper_motor_uim2852_t *m = (stepper_motor_uim2852_t *)motor;
	taskENTER_CRITICAL(&m->lock);
	bool in_progress = m->motion_in_progress;
	taskEXIT_CRITICAL(&m->lock);
	return in_progress;
}

/**
 * @brief Compute absolute position error (|target - actual|)
 * @return Position error in pulses (always >= 0)
 */
static inline int32_t stepper_motor_uim2852_position_error(const stepper_motor_uim2852_t *motor)
{
	if (!motor)
		return 0;
	stepper_motor_uim2852_t *m = (stepper_motor_uim2852_t *)motor;
	taskENTER_CRITICAL(&m->lock);
	int32_t target = m->target_position;
	int32_t actual = m->absolute_position;
	taskEXIT_CRITICAL(&m->lock);
	int64_t err = (int64_t)target - (int64_t)actual;
	if (err < 0)
		err = -err;
	if (err > INT32_MAX)
		return INT32_MAX;
	return (int32_t)err;
}

// ============================================================================
// Liveness Watchdog
// ============================================================================

/**
 * @brief Check motor liveness based on response timeout.
 *
 * Compares the current tick against last_response_tick.  If the motor has
 * been initialized, the driver is enabled, and no response has arrived
 * within @p timeout_ticks, the motor is considered unresponsive.
 *
 * @param motor         Motor instance
 * @param now_tick      Current FreeRTOS tick count (xTaskGetTickCount)
 * @param timeout_ticks Maximum allowed silence (e.g. pdMS_TO_TICKS(500))
 * @return true if the motor is unresponsive (timed out), false otherwise
 */
bool stepper_motor_uim2852_check_liveness(const stepper_motor_uim2852_t *motor, TickType_t now_tick,
                                          TickType_t timeout_ticks);

// ============================================================================
// CAN Frame Processing
// ============================================================================

/**
 * @brief Process incoming CAN frame from motor
 * @param motor Motor instance
 * @param msg Received CAN message (must be extended frame)
 * @return true if frame was processed (was from this motor)
 */
bool stepper_motor_uim2852_process_frame(stepper_motor_uim2852_t *motor, const twai_message_t *msg);

// ============================================================================
// Notification Callback
// ============================================================================

/**
 * @brief Notification callback type
 * @param motor Motor that generated the notification
 * @param notif Notification details
 */
typedef void (*stepper_motor_uim2852_notify_cb_t)(stepper_motor_uim2852_t *motor,
                                                  const stepper_uim2852_notification_t *notif);

/**
 * @brief Set notification callback for a motor
 */
void stepper_motor_uim2852_set_notify_callback(stepper_motor_uim2852_t *motor,
                                               stepper_motor_uim2852_notify_cb_t callback);

// ============================================================================
// Low-Level Access
// ============================================================================

/**
 * @brief Send raw instruction to motor
 * @param motor Motor instance
 * @param cw Control word (mnemonic)
 * @param data Data bytes
 * @param dl Data length
 * @return ESP_OK on successful transmit
 */
esp_err_t stepper_motor_uim2852_send_instruction(stepper_motor_uim2852_t *motor, uint8_t cw, const uint8_t *data,
                                                 uint8_t dl);

/**
 * @brief Query a parameter value
 * @param motor Motor instance
 * @param cw Parameter control word (PP, IC, IE, LM, QE)
 * @param index Parameter index
 * @param value Output value (NULL to skip)
 * @return ESP_OK on success
 */
esp_err_t stepper_motor_uim2852_query_param(stepper_motor_uim2852_t *motor, uint8_t cw, uint8_t index, int32_t *value);

/**
 * @brief Set a parameter value
 * @param motor Motor instance
 * @param cw Parameter control word (PP, IC, IE, LM, QE)
 * @param index Parameter index
 * @param value Value to set
 * @return ESP_OK on success
 */
esp_err_t stepper_motor_uim2852_set_param(stepper_motor_uim2852_t *motor, uint8_t cw, uint8_t index, int32_t value);

#ifdef __cplusplus
}
#endif