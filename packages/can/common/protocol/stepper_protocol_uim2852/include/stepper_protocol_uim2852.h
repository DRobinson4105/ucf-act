/**
 * @file stepper_protocol_uim2852.h
 * @brief UIM2852 stepper motor SimpleCAN3.0 protocol frame encoding and parsing.
 *
 * All CW (Control Word) hex codes and data formats match the UIM342CA CAN
 * Interface Reference specification.  Byte ordering is little-endian (LSB
 * first) throughout.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// CAN ID Encoding (Extended 29-bit CAN 2.0B) for UIM2852CA Stepper Motor
// ============================================================================
// For instructions (Host -> Motor):
//   SID = ((ConsumerID << 1) & 0x003F) | 0x0100
//   EID = (((ConsumerID << 1) & 0x00C0) << 8) | CW
//   CAN_ID = (SID << 18) | EID
//
// For responses (Motor -> Host):
//   ProducerID = ((EID >> 11) & 0x0060) | ((SID >> 6) & 0x001F)
//   CW = EID & 0x00FF
// ============================================================================

// Common node IDs used by this project.
#define UIM2852_MASTER_ID     4 // Producer ID for host controller
#define UIM2852_NODE_STEERING 7 // Steering motor node ID
#define UIM2852_NODE_BRAKING  6 // Braking motor node ID
#define UIM2852_GLOBAL_ID     0 // Broadcast to all motors

/**
 * @brief Calculate the 29-bit extended CAN ID for sending an instruction to a motor.
 *
 * Encodes the consumer (motor) node ID and control word into a CAN 2.0B
 * extended identifier following the UIM2852 SimpleCAN3.0 addressing scheme.
 *
 * @param consumer_id  Target motor node ID (5-126)
 * @param cw           Control word (instruction opcode, optionally OR'd with ACK bit)
 * @return 29-bit CAN ID suitable for a CAN extended frame
 */
static inline uint32_t stepper_uim2852_make_can_id(uint8_t consumer_id, uint8_t cw)
{
	uint16_t sid = ((consumer_id << 1) & 0x003F) | 0x0100;
	uint32_t eid = (((consumer_id << 1) & 0x00C0) << 8) | cw;
	return ((uint32_t)sid << 18) | eid;
}

/**
 * @brief Parse a 29-bit extended CAN ID from a motor response frame.
 *
 * Extracts the producer (motor) node ID and control word from a received
 * CAN 2.0B extended identifier.  Validates the SID format before extracting
 * fields.
 *
 * @param can_id       Received 29-bit CAN ID
 * @param producer_id  [out] Pointer to store the producer node ID (may be NULL)
 * @param cw           [out] Pointer to store the control word byte (may be NULL)
 * @return true if the CAN ID has a valid SID format, false otherwise
 */
static inline bool stepper_uim2852_parse_can_id(uint32_t can_id, uint8_t *producer_id, uint8_t *cw)
{
	uint16_t sid = (can_id >> 18) & 0x07FF;
	uint32_t eid = can_id & 0x3FFFF;

	// Validate SID format (should have bit 8 set)
	if ((sid & 0x0100) == 0)
		return false;

	if (producer_id)
		*producer_id = ((eid >> 11) & 0x0060) | ((sid >> 6) & 0x001F);
	if (cw)
		*cw = eid & 0x00FF;
	return true;
}

// ============================================================================
// Control Word (CW) Constants - Instruction Mnemonics
// ============================================================================
// bit7 of CW: 0 = no ACK requested, 1 = ACK requested
// For instructions, add 0x80 to request acknowledgment
//
// CW values below are from the UIM342CA CAN Interface Reference spec, section 15.

#define STEPPER_UIM2852_CW_ACK_BIT 0x80 // OR with CW to request ACK

// Protocol / configuration
#define STEPPER_UIM2852_CW_PP 0x01 // System parameters (Node ID, Group ID, bit rate)
#define STEPPER_UIM2852_CW_IC 0x06 // Initial configuration
#define STEPPER_UIM2852_CW_IE 0x07 // Information enable (notifications)
#define STEPPER_UIM2852_CW_ML 0x0B // Model string (read-only)
#define STEPPER_UIM2852_CW_SN 0x0C // Serial number (read-only)
#define STEPPER_UIM2852_CW_ER 0x0F // Error query/report

// Motor driver
#define STEPPER_UIM2852_CW_MT 0x10 // Motor driver parameters (microstep, current, brake)
#define STEPPER_UIM2852_CW_MS 0x11 // Motion status query

// Motion control
#define STEPPER_UIM2852_CW_MO 0x15 // Motor driver on/off
#define STEPPER_UIM2852_CW_BG 0x16 // Begin motion
#define STEPPER_UIM2852_CW_ST 0x17 // Stop motion (decel)
#define STEPPER_UIM2852_CW_MF 0x18 // Motion parameter frame (read-only)
#define STEPPER_UIM2852_CW_AC 0x19 // Acceleration
#define STEPPER_UIM2852_CW_DC 0x1A // Deceleration
#define STEPPER_UIM2852_CW_SS 0x1B // Cut-in speed
#define STEPPER_UIM2852_CW_SD 0x1C // Stop deceleration rate
#define STEPPER_UIM2852_CW_JV 0x1D // Jog velocity
#define STEPPER_UIM2852_CW_SP 0x1E // Speed for PTP mode
#define STEPPER_UIM2852_CW_PR 0x1F // Relative position
#define STEPPER_UIM2852_CW_PA 0x20 // Absolute position
#define STEPPER_UIM2852_CW_OG 0x21 // Set origin

// PVT/PT interpolated motion
#define STEPPER_UIM2852_CW_MP 0x22 // PVT motion parameters
#define STEPPER_UIM2852_CW_PV 0x23 // PVT mode select / start
#define STEPPER_UIM2852_CW_PT 0x24 // PT position
#define STEPPER_UIM2852_CW_QP 0x25 // PVT queue position
#define STEPPER_UIM2852_CW_QV 0x26 // PVT queue velocity
#define STEPPER_UIM2852_CW_QT 0x27 // PVT queue time
#define STEPPER_UIM2852_CW_QF 0x29 // PVT quick feed (8 bytes packed)

// Limits and compensation
#define STEPPER_UIM2852_CW_LM 0x2C // Software limits
#define STEPPER_UIM2852_CW_BL 0x2D // Backlash compensation
#define STEPPER_UIM2852_CW_DV 0x2E // Desired values (read-only)

// I/O control
#define STEPPER_UIM2852_CW_IL 0x34 // Input logic action
#define STEPPER_UIM2852_CW_TG 0x35 // Trigger type/filter
#define STEPPER_UIM2852_CW_DI 0x37 // Digital I/O

// Encoder / stall
#define STEPPER_UIM2852_CW_QE 0x3D // Quadrature encoder / stall parameters

// Real-time notification (from motor, asynchronous)
#define STEPPER_UIM2852_CW_NOTIFY 0x5A // Real-time notification

// System operation
#define STEPPER_UIM2852_CW_SY 0x7E // System operation (reboot, factory reset)

// High-speed functions
#define STEPPER_UIM2852_CW_D1 0xD1 // High-speed reciprocating / fixed-angle pulse

// ============================================================================
// Parameter Indices
// ============================================================================

// PP[i] - System parameters (CW: 0x01)
//   GET: DL=1, d0=index  ->  ACK: DL=2, d0=index, d1=value (u8)
//   SET: DL=2, d0=index, d1=value  ->  ACK echoes
#define STEPPER_UIM2852_PP_BITRATE  5 // CAN bit rate (0:1M 1:800K 2:500K 3:250K 4:125K)
#define STEPPER_UIM2852_PP_NODE_ID  7 // Node ID (5-126)
#define STEPPER_UIM2852_PP_GROUP_ID 8 // Group ID (5-126)

// IC[i] - Initial configuration (CW: 0x06)
//   GET: DL=1, d0=index  ->  ACK: DL=3, d0=index, d1-d2=value (u16 LE)
//   SET: DL=3, d0=index, d1-d2=value  ->  ACK echoes
#define STEPPER_UIM2852_IC_AUTO_ENABLE     0  // Auto-enable motor driver on power-up (0/1)
#define STEPPER_UIM2852_IC_DIR_POLARITY    1  // Positive direction (0:CW, 1:CCW)
#define STEPPER_UIM2852_IC_LOCKDOWN        3  // Enable lockdown system (0/1)
#define STEPPER_UIM2852_IC_ACDC_UNITS      4  // AC/DC units (0:pulse/sec^2, 1:milliseconds)
#define STEPPER_UIM2852_IC_CLOSED_LOOP     6  // Closed-loop control (0/1)
#define STEPPER_UIM2852_IC_SOFTWARE_LIMITS 7  // Software limits (0/1)
#define STEPPER_UIM2852_IC_STALL_REACTION  16 // Stall event reaction

// IE[i] - Information enable / notifications (CW: 0x07)
//   GET/SET same format as IC (DL=1 query, DL=3 set, 16-bit value)
#define STEPPER_UIM2852_IE_INPUT1         0  // Input 1 edge notification (0/1)
#define STEPPER_UIM2852_IE_INPUT2         1  // Input 2 edge notification (0/1)
#define STEPPER_UIM2852_IE_INPUT3         2  // Input 3 edge notification (0/1)
#define STEPPER_UIM2852_IE_PTP_COMPLETE   8  // PTP positioning finished notification (0/1)
#define STEPPER_UIM2852_IE_PVT_FIFO_EMPTY 10 // PVT/PT FIFO empty (dry out) notification (0/1)
#define STEPPER_UIM2852_IE_PVT_FIFO_LOW   11 // PVT/PT FIFO low warning notification (0/1)

// MT[i] - Motor driver parameters (CW: 0x10)
//   GET: DL=1, d0=index  ->  ACK: DL=3, d0=index, d1-d2=value (u16 LE)
//   SET: DL=3, d0=index, d1-d2=value  ->  ACK echoes
#define STEPPER_UIM2852_MT_MICROSTEP         0 // Micro-stepping (1, 2, 4, 8, 16, 32, 64)
#define STEPPER_UIM2852_MT_WORKING_CURRENT   1 // Working current (5-80 = 0.5-8.0A in 0.1A steps)
#define STEPPER_UIM2852_MT_IDLE_CURRENT      2 // Idle current (0-100% of working current)
#define STEPPER_UIM2852_MT_AUTO_ENABLE_DELAY 3 // Auto-enable delay (0-60000 ms)

// MS[i] - Motion status query (CW: 0x11)
#define STEPPER_UIM2852_MS_FLAGS_RELPOS 0 // Status flags + relative position
#define STEPPER_UIM2852_MS_SPEED_ABSPOS 1 // Speed + absolute position

// LM[i] - Software limits (CW: 0x2C)
//   GET: DL=1, d0=index  ->  ACK varies
//   SET: DL=5, d0=index, d1-d4=value (s32 LE)  ->  ACK echoes
#define STEPPER_UIM2852_LM_MAX_SPEED   0   // Maximum working speed
#define STEPPER_UIM2852_LM_LOWER_WORK  1   // Lower working limit
#define STEPPER_UIM2852_LM_UPPER_WORK  2   // Upper working limit
#define STEPPER_UIM2852_LM_LOWER_BUMP  3   // Lower bumping limit
#define STEPPER_UIM2852_LM_UPPER_BUMP  4   // Upper bumping limit
#define STEPPER_UIM2852_LM_MAX_POS_ERR 6   // Maximum position error (stall)
#define STEPPER_UIM2852_LM_MAX_ACCEL   7   // Maximum acceleration
#define STEPPER_UIM2852_LM_RESET       254 // Reset limits to defaults
#define STEPPER_UIM2852_LM_ENABLE      255 // Enable/disable limits (0/1)

// QE[i] - Quadrature encoder / stall parameters (CW: 0x3D)
//   GET/SET same format as IC/MT (DL=1 query, DL=3 set, 16-bit value)
#define STEPPER_UIM2852_QE_LPR             0 // Lines per revolution (1-65535)
#define STEPPER_UIM2852_QE_STALL_TOLERANCE 1 // Max position error / stall tolerance (10-65535 pulses)
#define STEPPER_UIM2852_QE_BATTERY_VOLTAGE 3 // Abs encoder battery voltage (0-3300 mV, read-only)
#define STEPPER_UIM2852_QE_CPR             4 // Counts per revolution (microstep * 200, 200-25600)

// DV[i] - Desired values (CW: 0x2E, read-only)
//   GET: DL=1, d0=index  ->  ACK: DL=5, d0=index, d1-d4=value
#define STEPPER_UIM2852_DV_MOTION_MODE     0 // Current motion mode (0:JOG, 1:PTP)
#define STEPPER_UIM2852_DV_WORKING_CURRENT 1 // Desired working current (0-80)
#define STEPPER_UIM2852_DV_SPEED           2 // Desired speed (s32)
#define STEPPER_UIM2852_DV_REL_POSITION    3 // Desired relative position (s32)
#define STEPPER_UIM2852_DV_ABS_POSITION    4 // Desired absolute position (s32)

// ============================================================================
// Notification Types (d0/d1 values when CW=0x5A)
// ============================================================================

typedef enum
{
	// Alarm notifications (d0=0x00, d1=alarm code)
	STEPPER_UIM2852_ALARM_ESTOP_LOCK = 0x0A,
	STEPPER_UIM2852_ALARM_ACCEL_OVERLIMIT = 0x16,
	STEPPER_UIM2852_ALARM_SPEED_OVERLIMIT = 0x17,
	STEPPER_UIM2852_ALARM_LOWER_BUMP = 0x18,
	STEPPER_UIM2852_ALARM_UPPER_BUMP = 0x19,
	STEPPER_UIM2852_ALARM_LOWER_WORK = 0x1A,
	STEPPER_UIM2852_ALARM_UPPER_WORK = 0x1B,
	STEPPER_UIM2852_ALARM_STALL = 0x1D,
	STEPPER_UIM2852_ALARM_ENCODER_ERROR = 0x1E,
	STEPPER_UIM2852_ALARM_ENCODER_BATTERY = 0x1F,

	// Status notifications (d0=type)
	STEPPER_UIM2852_STATUS_IN1_FALL = 0x01,
	STEPPER_UIM2852_STATUS_IN1_RISE = 0x02,
	STEPPER_UIM2852_STATUS_IN2_FALL = 0x03,
	STEPPER_UIM2852_STATUS_IN2_RISE = 0x04,
	STEPPER_UIM2852_STATUS_IN3_FALL = 0x05,
	STEPPER_UIM2852_STATUS_IN3_RISE = 0x06,
	STEPPER_UIM2852_STATUS_PTP_COMPLETE = 0x29,
	STEPPER_UIM2852_STATUS_PVT_FIFO_EMPTY = 0x2A, // PVT/PT FIFO empty (dry-out)
	STEPPER_UIM2852_STATUS_PVT_FIFO_LOW = 0x2B,   // PVT/PT FIFO low warning
} stepper_uim2852_notification_type_t;

// ============================================================================
// Error Codes (from ER instruction response)
// ============================================================================

typedef enum
{
	STEPPER_UIM2852_ERR_SYNTAX = 0x32,            // Instruction syntax error
	STEPPER_UIM2852_ERR_DATA = 0x33,              // Instruction data error
	STEPPER_UIM2852_ERR_SUBINDEX = 0x34,          // Instruction sub-index error
	STEPPER_UIM2852_ERR_SD_LESS_DC = 0x3C,        // SD value < DC value
	STEPPER_UIM2852_ERR_NOT_WHILE_RUNNING = 0x3D, // Instruction not allowed while running
	STEPPER_UIM2852_ERR_BG_DRIVER_OFF = 0x3E,     // BG not allowed when driver OFF
	STEPPER_UIM2852_ERR_BG_ESTOP = 0x3F,          // BG not allowed during e-stop
	STEPPER_UIM2852_ERR_OG_WHILE_RUNNING = 0x41,  // OG not allowed while running
} stepper_uim2852_error_code_t;

// ============================================================================
// Status Structures
// ============================================================================

// MS[0] response - Status flags and relative position
typedef struct
{
	// d1 flags
	uint8_t mode;    // bits 1:0 - 0=JOG, 1=PTP
	bool driver_on;  // bit 2 - SON, motor driver enabled
	bool in1_level;  // bit 3 - Input 1 logic level
	bool in2_level;  // bit 4 - Input 2 logic level
	bool in3_level;  // bit 5 - Input 3 logic level
	bool out1_level; // bit 6 - Output 1 logic level

	// d2 flags
	bool stopped;        // bit 0 - STOP, motor is stationary
	bool in_position;    // bit 1 - PAIF, motor reached target position
	bool pvt_stopped;    // bit 2 - PSIF, PVT stopped
	bool stall_detected; // bit 3 - TLIF, stall detected
	bool system_locked;  // bit 5 - LOCK, system locked down
	bool error_detected; // bit 7 - ERR, error flag

	// d4-d7: relative position (signed 32-bit LE)
	int32_t relative_position;
} stepper_uim2852_status_t;

// Notification structure
typedef struct
{
	uint8_t node_id;
	bool is_alarm;    // true if alarm (d0==0x00), false if status
	uint8_t type;     // notification type code (alarm code or status code)
	int32_t position; // For PTP_COMPLETE, current position from d4-d7
} stepper_uim2852_notification_t;

// Error report structure (from ER response)
typedef struct
{
	uint8_t error_code; // d1: error code
	uint8_t related_cw; // d2: CW that caused the error
	uint8_t subindex;   // d3: sub-index of the CW
} stepper_uim2852_error_t;

// ============================================================================
// Frame Building Functions
// ============================================================================
// All functions return the data length (DL) to use.
// The 'data' array should be at least 8 bytes.

/**
 * @brief Build a Motor driver On/Off frame (MO instruction, CW 0x15).
 *
 * Enables or disables the motor driver.  The motor must be enabled before
 * any motion command (BG) can be issued.
 *
 * @param data    [out] Frame data buffer (at least 8 bytes)
 * @param enable  true to enable (driver ON), false to disable (driver OFF)
 * @return Data length (DL) for the CAN frame (1)
 */
uint8_t stepper_uim2852_build_mo(uint8_t *data, bool enable);

/**
 * @brief Build a Begin Motion frame (BG instruction, CW 0x16).
 *
 * Starts motion using the currently configured motion parameters (speed,
 * position, acceleration, etc.).  The motor driver must be enabled first.
 *
 * @param data  [out] Frame data buffer (at least 8 bytes)
 * @return Data length (DL) for the CAN frame (0)
 */
uint8_t stepper_uim2852_build_bg(uint8_t *data);

/**
 * @brief Build a Stop Motion frame (ST instruction, CW 0x17).
 *
 * Commands the motor to decelerate to a stop using the configured stop
 * deceleration rate (SD).
 *
 * @param data  [out] Frame data buffer (at least 8 bytes)
 * @return Data length (DL) for the CAN frame (0)
 */
uint8_t stepper_uim2852_build_st(uint8_t *data);

/**
 * @brief Build a Set Stop Deceleration frame (SD instruction, CW 0x1C).
 *
 * Sets the deceleration rate used when a stop (ST) command is issued.
 * Units depend on the IC[4] AC/DC units setting (pulses/sec^2 or ms).
 *
 * @param data        [out] Frame data buffer (at least 8 bytes)
 * @param decel_rate  Stop deceleration rate (unsigned 32-bit)
 * @return Data length (DL) for the CAN frame (4)
 */
uint8_t stepper_uim2852_build_sd(uint8_t *data, uint32_t decel_rate);

/**
 * @brief Build a Set Acceleration frame (AC instruction, CW 0x19).
 *
 * Sets the acceleration rate for motion profiling.  Units depend on the
 * IC[4] AC/DC units setting (pulses/sec^2 or ms).
 *
 * @param data        [out] Frame data buffer (at least 8 bytes)
 * @param accel_rate  Acceleration rate (unsigned 32-bit)
 * @return Data length (DL) for the CAN frame (4)
 */
uint8_t stepper_uim2852_build_ac(uint8_t *data, uint32_t accel_rate);

/**
 * @brief Build a Set Deceleration frame (DC instruction, CW 0x1A).
 *
 * Sets the deceleration rate for motion profiling.  Units depend on the
 * IC[4] AC/DC units setting (pulses/sec^2 or ms).
 *
 * @param data        [out] Frame data buffer (at least 8 bytes)
 * @param decel_rate  Deceleration rate (unsigned 32-bit)
 * @return Data length (DL) for the CAN frame (4)
 */
uint8_t stepper_uim2852_build_dc(uint8_t *data, uint32_t decel_rate);

/**
 * @brief Build a Set Cut-in Speed frame (SS instruction, CW 0x1B).
 *
 * Sets the cut-in (start) speed for the motion profile.  The motor begins
 * moving at this speed before accelerating to the target speed.
 *
 * @param data       [out] Frame data buffer (at least 8 bytes)
 * @param speed_pps  Cut-in speed in pulses per second (unsigned 32-bit)
 * @return Data length (DL) for the CAN frame (4)
 */
uint8_t stepper_uim2852_build_ss(uint8_t *data, uint32_t speed_pps);

/**
 * @brief Build a Set Speed for PTP frame (SP instruction, CW 0x1E).
 *
 * Sets the target speed for point-to-point (PTP) positioning mode.
 * Sign indicates direction.
 *
 * @param data       [out] Frame data buffer (at least 8 bytes)
 * @param speed_pps  Target speed in pulses per second (signed 32-bit)
 * @return Data length (DL) for the CAN frame (4)
 */
uint8_t stepper_uim2852_build_sp(uint8_t *data, int32_t speed_pps);

/**
 * @brief Build a Set Jog Velocity frame (JV instruction, CW 0x1D).
 *
 * Sets the velocity for jog (continuous) motion mode.  Sign indicates
 * direction: positive = forward, negative = reverse.
 *
 * @param data          [out] Frame data buffer (at least 8 bytes)
 * @param velocity_pps  Jog velocity in pulses per second (signed 32-bit)
 * @return Data length (DL) for the CAN frame (4)
 */
uint8_t stepper_uim2852_build_jv(uint8_t *data, int32_t velocity_pps);

/**
 * @brief Build a Set Absolute Position frame (PA instruction, CW 0x20).
 *
 * Sets the target absolute position for point-to-point (PTP) positioning.
 * Issue a BG command after this to begin the move.
 *
 * @param data      [out] Frame data buffer (at least 8 bytes)
 * @param position  Target absolute position in pulses (signed 32-bit)
 * @return Data length (DL) for the CAN frame (4)
 */
uint8_t stepper_uim2852_build_pa(uint8_t *data, int32_t position);

/**
 * @brief Build a Set Relative Position frame (PR instruction, CW 0x1F).
 *
 * Sets a relative displacement for point-to-point (PTP) positioning.
 * Issue a BG command after this to begin the move.
 *
 * @param data          [out] Frame data buffer (at least 8 bytes)
 * @param displacement  Relative displacement in pulses (signed 32-bit)
 * @return Data length (DL) for the CAN frame (4)
 */
uint8_t stepper_uim2852_build_pr(uint8_t *data, int32_t displacement);

/**
 * @brief Build a Set Origin frame (OG instruction, CW 0x21).
 *
 * Resets the motor's internal position counter to zero at the current
 * physical position.  Must not be called while the motor is in motion.
 *
 * @param data  [out] Frame data buffer (at least 8 bytes)
 * @return Data length (DL) for the CAN frame (0)
 */
uint8_t stepper_uim2852_build_og(uint8_t *data);

/**
 * @brief Build a Query Motion Status frame (MS instruction, CW 0x11).
 *
 * Queries the motor's current motion status.  Index 0 returns status flags
 * and relative position; index 1 returns speed and absolute position.
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Status query index (0 = flags + rel pos, 1 = speed + abs pos)
 * @return Data length (DL) for the CAN frame (1)
 */
uint8_t stepper_uim2852_build_ms(uint8_t *data, uint8_t index);

/**
 * @brief Build a Clear Status Flags frame (MS[0]=0, CW 0x11).
 *
 * Clears latched status flags (e.g. in-position, stall detected) by
 * writing zero to MS[0].
 *
 * @param data  [out] Frame data buffer (at least 8 bytes)
 * @return Data length (DL) for the CAN frame (2)
 */
uint8_t stepper_uim2852_build_ms_clear(uint8_t *data);

/**
 * @brief Build a Query System Parameter frame (PP instruction, CW 0x01).
 *
 * Queries a system parameter by index.  The motor responds with the
 * parameter value as an 8-bit unsigned integer.
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Parameter index (e.g. STEPPER_UIM2852_PP_NODE_ID)
 * @return Data length (DL) for the CAN frame (1)
 */
uint8_t stepper_uim2852_build_pp_query(uint8_t *data, uint8_t index);

/**
 * @brief Build a Set System Parameter frame (PP instruction, CW 0x01).
 *
 * Sets a system parameter to an 8-bit value.  Changes to node ID or
 * bit rate take effect after the next power cycle.
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Parameter index (e.g. STEPPER_UIM2852_PP_NODE_ID)
 * @param value  New parameter value (unsigned 8-bit)
 * @return Data length (DL) for the CAN frame (2)
 */
uint8_t stepper_uim2852_build_pp_set(uint8_t *data, uint8_t index, uint8_t value);

/**
 * @brief Build a Query Initial Configuration frame (IC instruction, CW 0x06).
 *
 * Queries an initial configuration parameter by index.  The motor responds
 * with the value as a 16-bit unsigned integer (little-endian).
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Configuration index (e.g. STEPPER_UIM2852_IC_CLOSED_LOOP)
 * @return Data length (DL) for the CAN frame (1)
 */
uint8_t stepper_uim2852_build_ic_query(uint8_t *data, uint8_t index);

/**
 * @brief Build a Set Initial Configuration frame (IC instruction, CW 0x06).
 *
 * Sets an initial configuration parameter to a 16-bit value (little-endian).
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Configuration index (e.g. STEPPER_UIM2852_IC_CLOSED_LOOP)
 * @param value  New configuration value (unsigned 16-bit)
 * @return Data length (DL) for the CAN frame (3)
 */
uint8_t stepper_uim2852_build_ic_set(uint8_t *data, uint8_t index, uint16_t value);

/**
 * @brief Build a Query Information Enable frame (IE instruction, CW 0x07).
 *
 * Queries a notification enable setting by index.  The motor responds
 * with the value as a 16-bit unsigned integer (little-endian).
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Notification index (e.g. STEPPER_UIM2852_IE_PTP_COMPLETE)
 * @return Data length (DL) for the CAN frame (1)
 */
uint8_t stepper_uim2852_build_ie_query(uint8_t *data, uint8_t index);

/**
 * @brief Build a Set Information Enable frame (IE instruction, CW 0x07).
 *
 * Enables or disables a specific notification by index.
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Notification index (e.g. STEPPER_UIM2852_IE_PTP_COMPLETE)
 * @param value  Enable value (unsigned 16-bit; typically 0 = disable, 1 = enable)
 * @return Data length (DL) for the CAN frame (3)
 */
uint8_t stepper_uim2852_build_ie_set(uint8_t *data, uint8_t index, uint16_t value);

/**
 * @brief Build a Query Motor Driver Parameter frame (MT instruction, CW 0x10).
 *
 * Queries a motor driver parameter by index.  The motor responds with
 * the value as a 16-bit unsigned integer (little-endian).
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Motor parameter index (e.g. STEPPER_UIM2852_MT_MICROSTEP)
 * @return Data length (DL) for the CAN frame (1)
 */
uint8_t stepper_uim2852_build_mt_query(uint8_t *data, uint8_t index);

/**
 * @brief Build a Set Motor Driver Parameter frame (MT instruction, CW 0x10).
 *
 * Sets a motor driver parameter to a 16-bit value (little-endian).
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Motor parameter index (e.g. STEPPER_UIM2852_MT_MICROSTEP)
 * @param value  New parameter value (unsigned 16-bit)
 * @return Data length (DL) for the CAN frame (3)
 */
uint8_t stepper_uim2852_build_mt_set(uint8_t *data, uint8_t index, uint16_t value);

/**
 * @brief Build a Query Software Limit frame (LM instruction, CW 0x2C).
 *
 * Queries a software limit parameter by index.  The motor responds with
 * the value as a 32-bit signed integer (little-endian).
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Limit parameter index (e.g. STEPPER_UIM2852_LM_MAX_SPEED)
 * @return Data length (DL) for the CAN frame (1)
 */
uint8_t stepper_uim2852_build_lm_query(uint8_t *data, uint8_t index);

/**
 * @brief Build a Set Software Limit frame (LM instruction, CW 0x2C).
 *
 * Sets a software limit parameter to a 32-bit signed value (little-endian).
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Limit parameter index (e.g. STEPPER_UIM2852_LM_MAX_SPEED)
 * @param value  New limit value (signed 32-bit)
 * @return Data length (DL) for the CAN frame (5)
 */
uint8_t stepper_uim2852_build_lm_set(uint8_t *data, uint8_t index, int32_t value);

/**
 * @brief Build a Query Encoder/Stall Parameter frame (QE instruction, CW 0x3D).
 *
 * Queries a quadrature encoder or stall detection parameter by index.
 * The motor responds with the value as a 16-bit unsigned integer
 * (little-endian).
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Encoder/stall parameter index (e.g. STEPPER_UIM2852_QE_LPR)
 * @return Data length (DL) for the CAN frame (1)
 */
uint8_t stepper_uim2852_build_qe_query(uint8_t *data, uint8_t index);

/**
 * @brief Build a Set Encoder/Stall Parameter frame (QE instruction, CW 0x3D).
 *
 * Sets a quadrature encoder or stall detection parameter to a 16-bit value
 * (little-endian).
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Encoder/stall parameter index (e.g. STEPPER_UIM2852_QE_LPR)
 * @param value  New parameter value (unsigned 16-bit)
 * @return Data length (DL) for the CAN frame (3)
 */
uint8_t stepper_uim2852_build_qe_set(uint8_t *data, uint8_t index, uint16_t value);


// ============================================================================
// PT/PVT Interpolated Motion Frame Builders
// ============================================================================
// Position-Time (PT) mode queues (position, time) waypoints in the motor's
// internal FIFO.  The motor smoothly interpolates between waypoints,
// eliminating the jerk caused by repeated PTP (PA+BG) re-profiling.
//
// Typical usage:
//   1. build_pv(mode=1, start) — select PT mode, start interpolation
//   2. build_pt(position, time_ms) — feed waypoints at control loop rate
//      OR build_qf(position, time_ms) — packed single-frame variant
//   3. build_pv(mode=1, stop) — stop PT interpolation
//   4. build_st() — stop motion if needed

/**
 * @brief Build a PVT/PT Mode Select and Start/Stop frame (PV instruction, CW 0x23).
 *
 * Selects the interpolation mode and starts or stops interpolated motion.
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param mode   Interpolation mode (0 = PVT cubic spline, 1 = PT linear, 2 = PT circular)
 * @param start  true to begin interpolation, false to stop
 * @return Data length (DL) for the CAN frame (1)
 */
uint8_t stepper_uim2852_build_pv(uint8_t *data, uint8_t mode, bool start);

/**
 * @brief Build a PT Position + Time feed frame (PT instruction, CW 0x24).
 *
 * Feeds an absolute position and time waypoint into the motor's PT FIFO.
 * The motor will move to the specified position within the given time.
 *
 * @param data      [out] Frame data buffer (at least 8 bytes)
 * @param position  Absolute target position in pulses (signed 32-bit LE, bytes 0-3)
 * @param time_ms   Time to reach the position in milliseconds (unsigned 32-bit LE, bytes 4-7)
 * @return Data length (DL) for the CAN frame (8)
 */
uint8_t stepper_uim2852_build_pt(uint8_t *data, int32_t position, uint32_t time_ms);

/**
 * @brief Build a PVT Queue Position frame (QP instruction, CW 0x25).
 *
 * Queues an absolute position component for a PVT waypoint.
 *
 * @param data      [out] Frame data buffer (at least 8 bytes)
 * @param position  Absolute position in pulses (signed 32-bit LE)
 * @return Data length (DL) for the CAN frame (4)
 */
uint8_t stepper_uim2852_build_qp(uint8_t *data, int32_t position);

/**
 * @brief Build a PVT Queue Velocity frame (QV instruction, CW 0x26).
 *
 * Queues a velocity component for a PVT waypoint.
 *
 * @param data      [out] Frame data buffer (at least 8 bytes)
 * @param velocity  Velocity in pulses per second (signed 32-bit LE)
 * @return Data length (DL) for the CAN frame (4)
 */
uint8_t stepper_uim2852_build_qv(uint8_t *data, int32_t velocity);

/**
 * @brief Build a PVT Queue Time frame (QT instruction, CW 0x27).
 *
 * Queues a time component for a PVT waypoint.
 *
 * @param data     [out] Frame data buffer (at least 8 bytes)
 * @param time_ms  Time segment duration in milliseconds (unsigned 32-bit LE)
 * @return Data length (DL) for the CAN frame (4)
 */
uint8_t stepper_uim2852_build_qt(uint8_t *data, uint32_t time_ms);

/**
 * @brief Build a Quick Feed frame (QF instruction, CW 0x29).
 *
 * Packs an absolute position and time into a single 8-byte CAN frame,
 * equivalent to issuing QP + QT but using only one bus transaction.
 *
 * @param data      [out] Frame data buffer (at least 8 bytes)
 * @param position  Absolute position in pulses (signed 32-bit LE, bytes 0-3)
 * @param time_ms   Time segment in milliseconds (unsigned 32-bit LE, bytes 4-7)
 * @return Data length (DL) for the CAN frame (8)
 */
uint8_t stepper_uim2852_build_qf(uint8_t *data, int32_t position, uint32_t time_ms);

/**
 * @brief Build a Query Model String frame (ML instruction, CW 0x0B).
 *
 * Requests the motor's model identification string.
 *
 * @param data  [out] Frame data buffer (at least 8 bytes)
 * @return Data length (DL) for the CAN frame (0)
 */
uint8_t stepper_uim2852_build_ml(uint8_t *data);

/**
 * @brief Build a Query Serial Number frame (SN instruction, CW 0x0C).
 *
 * Requests the motor's serial number.
 *
 * @param data  [out] Frame data buffer (at least 8 bytes)
 * @return Data length (DL) for the CAN frame (0)
 */
uint8_t stepper_uim2852_build_sn(uint8_t *data);

/**
 * @brief Build a Set Backlash Compensation frame (BL instruction, CW 0x2D).
 *
 * Configures the backlash compensation value.  The motor adds this many
 * extra pulses when reversing direction to compensate for mechanical
 * backlash.
 *
 * @param data    [out] Frame data buffer (at least 8 bytes)
 * @param pulses  Backlash compensation in pulses (unsigned 16-bit LE)
 * @return Data length (DL) for the CAN frame (2)
 */
uint8_t stepper_uim2852_build_bl(uint8_t *data, uint16_t pulses);

// ============================================================================
// Response Parsing Functions
// ============================================================================

/**
 * @brief Parse an MS[0] response (status flags + relative position).
 *
 * Decodes the response from an MS[0] query into structured status flags
 * (driver state, I/O levels, motion flags) and the current relative
 * position.
 *
 * @param data    Response frame data bytes
 * @param dl      Data length of the response frame
 * @param status  [out] Pointer to status structure to populate
 * @return true if the response was parsed successfully, false on format error
 */
bool stepper_uim2852_parse_ms0(const uint8_t *data, uint8_t dl, stepper_uim2852_status_t *status);

/**
 * @brief Parse an MS[1] response (speed + absolute position).
 *
 * Decodes the response from an MS[1] query into the current speed and
 * absolute position.
 *
 * @param data          Response frame data bytes
 * @param dl            Data length of the response frame
 * @param speed_pps     [out] Pointer to store current speed in pulses/sec (signed 32-bit)
 * @param abs_position  [out] Pointer to store current absolute position (signed 32-bit)
 * @return true if the response was parsed successfully, false on format error
 */
bool stepper_uim2852_parse_ms1(const uint8_t *data, uint8_t dl, int32_t *speed_pps, int32_t *abs_position);

/**
 * @brief Parse a real-time notification frame (CW 0x5A).
 *
 * Decodes an asynchronous notification from the motor, which may be an
 * alarm (e.g. stall, limit hit) or a status event (e.g. PTP complete,
 * input edge).
 *
 * @param data   Response frame data bytes
 * @param dl     Data length of the response frame
 * @param notif  [out] Pointer to notification structure to populate
 * @return true if the notification was parsed successfully, false on format error
 */
bool stepper_uim2852_parse_notification(const uint8_t *data, uint8_t dl, stepper_uim2852_notification_t *notif);

/**
 * @brief Parse an error report frame (CW 0x0F).
 *
 * Decodes an error response from the motor, extracting the error code,
 * the CW that triggered the error, and the associated sub-index.
 *
 * @param data   Response frame data bytes
 * @param dl     Data length of the response frame
 * @param error  [out] Pointer to error structure to populate
 * @return true if the error was parsed successfully, false on format error
 */
bool stepper_uim2852_parse_error(const uint8_t *data, uint8_t dl, stepper_uim2852_error_t *error);

/**
 * @brief Parse a parameter query response (PP, IC, IE, MT, LM, QE).
 *
 * Automatically handles 8-bit (DL=2), 16-bit (DL=3), and 32-bit (DL>=5)
 * parameter values based on the response data length.
 *
 * @param data   Response frame data bytes
 * @param dl     Data length of the response frame
 * @param index  [out] Pointer to store the parameter sub-index (d0)
 * @param value  [out] Pointer to store the parameter value (widened to signed 32-bit)
 * @return true if the response was parsed successfully, false on format error
 */
bool stepper_uim2852_parse_param_response(const uint8_t *data, uint8_t dl, uint8_t *index, int32_t *value);

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Get a control word with the ACK request bit set.
 *
 * ORs the ACK bit (0x80) into the given control word so the motor will
 * send an acknowledgment response after processing the instruction.
 *
 * @param cw  Base control word value
 * @return Control word with bit 7 (ACK) set
 */
static inline uint8_t stepper_uim2852_cw_with_ack(uint8_t cw)
{
	return cw | STEPPER_UIM2852_CW_ACK_BIT;
}

/**
 * @brief Check if a control word has the ACK request bit set.
 *
 * @param cw  Control word to check
 * @return true if bit 7 (ACK) is set, false otherwise
 */
static inline bool stepper_uim2852_cw_ack_requested(uint8_t cw)
{
	return (cw & STEPPER_UIM2852_CW_ACK_BIT) != 0;
}

/**
 * @brief Get the base control word with the ACK bit cleared.
 *
 * Strips the ACK request bit (0x80) from a control word to obtain the
 * underlying instruction opcode.
 *
 * @param cw  Control word (possibly with ACK bit set)
 * @return Control word with bit 7 cleared
 */
static inline uint8_t stepper_uim2852_cw_base(uint8_t cw)
{
	return cw & ~STEPPER_UIM2852_CW_ACK_BIT;
}

#ifdef __cplusplus
}
#endif
