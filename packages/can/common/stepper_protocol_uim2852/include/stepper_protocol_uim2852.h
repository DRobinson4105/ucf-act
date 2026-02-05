#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------------
// CAN ID Encoding (Extended 29-bit CAN 2.0B) for UIM2852CA Stepper Motor
// ----------------------------------------------------------------------------
// For instructions (Host -> Motor):
//   SID = ((ConsumerID << 1) & 0x003F) | 0x0100
//   EID = (((ConsumerID << 1) & 0x00C0) << 8) | CW
//   CAN_ID = (SID << 18) | EID
//
// For responses (Motor -> Host):
//   ProducerID = ((EID >> 11) & 0x0060) | ((SID >> 6) & 0x001F)
//   CW = EID & 0x00FF
// ----------------------------------------------------------------------------

#define STEPPER_UIM2852_PRODUCER_ID_HOST    4   // Master controller ID

// Calculate 29-bit CAN ID for sending instruction to motor
static inline uint32_t stepper_uim2852_make_can_id(uint8_t consumer_id, uint8_t cw) {
    uint16_t sid = ((consumer_id << 1) & 0x003F) | 0x0100;
    uint32_t eid = (((consumer_id << 1) & 0x00C0) << 8) | cw;
    return ((uint32_t)sid << 18) | eid;
}

// Parse 29-bit CAN ID from motor response
static inline bool stepper_uim2852_parse_can_id(uint32_t can_id, uint8_t *producer_id, uint8_t *cw) {
    uint16_t sid = (can_id >> 18) & 0x07FF;
    uint32_t eid = can_id & 0x3FFFF;
    
    // Validate SID format (should have bit 8 set)
    if ((sid & 0x0100) == 0) return false;

    if (producer_id) *producer_id = ((eid >> 11) & 0x0060) | ((sid >> 1) & 0x001F);
    if (cw) *cw = eid & 0x00FF;
    return true;
}

// ----------------------------------------------------------------------------
// Control Word (CW) Constants - Instruction Mnemonics
// ----------------------------------------------------------------------------
// bit7 of CW: 0 = no ACK requested, 1 = ACK requested
// For instructions, add 0x80 to request acknowledgment

#define STEPPER_UIM2852_CW_ACK_BIT          0x80    // OR with CW to request ACK

// Parameter instructions
#define STEPPER_UIM2852_CW_PP               0x01    // System parameters (Node ID, Group ID, etc.)
#define STEPPER_UIM2852_CW_IC               0x0C    // Internal configuration
#define STEPPER_UIM2852_CW_IE               0x0D    // Information enable
#define STEPPER_UIM2852_CW_ER               0x0F    // Error query/report

// Motion status
#define STEPPER_UIM2852_CW_MS               0x11    // Motion status query

// Motion control
#define STEPPER_UIM2852_CW_MO               0x15    // Motor driver on/off
#define STEPPER_UIM2852_CW_BG               0x16    // Begin motion
#define STEPPER_UIM2852_CW_ST               0x17    // Stop motion (decel)
#define STEPPER_UIM2852_CW_SD               0x18    // Stop deceleration rate
#define STEPPER_UIM2852_CW_AC               0x19    // Acceleration
#define STEPPER_UIM2852_CW_DC               0x1A    // Deceleration
#define STEPPER_UIM2852_CW_OG               0x1C    // Set origin
#define STEPPER_UIM2852_CW_JV               0x1D    // Jog velocity
#define STEPPER_UIM2852_CW_SP               0x1E    // Speed for PTP mode
#define STEPPER_UIM2852_CW_PR               0x1F    // Relative position
#define STEPPER_UIM2852_CW_PA               0x20    // Absolute position

// Limits
#define STEPPER_UIM2852_CW_LM               0x21    // Software limits

// Stall/encoder
#define STEPPER_UIM2852_CW_QE               0x22    // Encoder/stall parameters

// Input logic
#define STEPPER_UIM2852_CW_IL               0x23    // Input logic action
#define STEPPER_UIM2852_CW_TG               0x24    // Trigger type/filter

// Brake control
#define STEPPER_UIM2852_CW_MT               0x25    // Brake control

// PVT/PT motion
#define STEPPER_UIM2852_CW_MP               0x26    // PVT motion parameters
#define STEPPER_UIM2852_CW_QP               0x27    // PVT position
#define STEPPER_UIM2852_CW_QV               0x28    // PVT velocity
#define STEPPER_UIM2852_CW_QT               0x29    // PVT time
#define STEPPER_UIM2852_CW_QF               0x2A    // PVT quick feed
#define STEPPER_UIM2852_CW_PV               0x2B    // Select PVT mode
#define STEPPER_UIM2852_CW_PT               0x2C    // PT position

// Real-time notification (from motor)
#define STEPPER_UIM2852_CW_NOTIFY           0x5A    // Real-time notification

// High-speed functions
#define STEPPER_UIM2852_CW_D1               0xD1    // High-speed reciprocating / fixed-angle pulse

// ----------------------------------------------------------------------------
// Parameter Indices
// ----------------------------------------------------------------------------

// PP[i] - System parameters
#define STEPPER_UIM2852_PP_MICROSTEP        5       // Microstepping resolution
#define STEPPER_UIM2852_PP_NODE_ID          7       // Node ID
#define STEPPER_UIM2852_PP_GROUP_ID         8       // Group ID

// IC[i] - Internal configuration
#define STEPPER_UIM2852_IC_DIR_POLARITY     1       // Position counter direction
#define STEPPER_UIM2852_IC_CLOSED_LOOP      6       // Closed loop enable
#define STEPPER_UIM2852_IC_BRAKE_LOGIC      8       // Brake control logic enable
#define STEPPER_UIM2852_IC_STALL_FREEWHEEL  16      // Freewheel on stall (vs lock)

// IE[i] - Information enable (real-time notifications)
#define STEPPER_UIM2852_IE_INPUT1           0       // Input 1 edge notification
#define STEPPER_UIM2852_IE_INPUT2           1       // Input 2 edge notification
#define STEPPER_UIM2852_IE_INPUT3           2       // Input 3 edge notification

// MS[i] - Motion status query
#define STEPPER_UIM2852_MS_FLAGS_RELPOS     0       // Status flags + relative position
#define STEPPER_UIM2852_MS_SPEED_ABSPOS     1       // Speed + absolute position

// LM[i] - Software limits
#define STEPPER_UIM2852_LM_MAX_SPEED        0       // Maximum working speed
#define STEPPER_UIM2852_LM_LOWER_WORK       1       // Lower working limit
#define STEPPER_UIM2852_LM_UPPER_WORK       2       // Upper working limit
#define STEPPER_UIM2852_LM_LOWER_BUMP       3       // Lower bumping limit
#define STEPPER_UIM2852_LM_UPPER_BUMP       4       // Upper bumping limit
#define STEPPER_UIM2852_LM_MAX_POS_ERR      6       // Maximum position error (stall)
#define STEPPER_UIM2852_LM_MAX_ACCEL        7       // Maximum acceleration
#define STEPPER_UIM2852_LM_RESET            254     // Reset limits to defaults
#define STEPPER_UIM2852_LM_ENABLE           255     // Enable/disable limits

// QE[i] - Encoder/stall parameters
#define STEPPER_UIM2852_QE_STALL_TOLERANCE  1       // Stall tolerance in pulses

// MT[i] - Brake control
#define STEPPER_UIM2852_MT_BRAKE            5       // 0=release, 1=engage

// ----------------------------------------------------------------------------
// Notification Types (d0 values when CW=0x5A)
// ----------------------------------------------------------------------------

typedef enum {
    // Alarm notifications (d0=0x00, d1=alarm code)
    STEPPER_UIM2852_ALARM_ESTOP_LOCK        = 0x0A,
    STEPPER_UIM2852_ALARM_ACCEL_OVERLIMIT   = 0x16,
    STEPPER_UIM2852_ALARM_SPEED_OVERLIMIT   = 0x17,
    STEPPER_UIM2852_ALARM_LOWER_BUMP        = 0x18,
    STEPPER_UIM2852_ALARM_UPPER_BUMP        = 0x19,
    STEPPER_UIM2852_ALARM_LOWER_WORK        = 0x1A,
    STEPPER_UIM2852_ALARM_UPPER_WORK        = 0x1B,
    STEPPER_UIM2852_ALARM_STALL             = 0x1D,
    STEPPER_UIM2852_ALARM_ENCODER_ERROR     = 0x1E,
    STEPPER_UIM2852_ALARM_ENCODER_BATTERY   = 0x1F,
    
    // Status notifications (d0=type, d1=0)
    STEPPER_UIM2852_STATUS_IN1_FALL         = 0x01,
    STEPPER_UIM2852_STATUS_IN1_RISE         = 0x02,
    STEPPER_UIM2852_STATUS_IN2_FALL         = 0x03,
    STEPPER_UIM2852_STATUS_IN2_RISE         = 0x04,
    STEPPER_UIM2852_STATUS_IN3_FALL         = 0x05,
    STEPPER_UIM2852_STATUS_IN3_RISE         = 0x06,
    STEPPER_UIM2852_STATUS_PTP_COMPLETE     = 0x29,
} stepper_uim2852_notification_type_t;

// ----------------------------------------------------------------------------
// Error Codes (from ER instruction response)
// ----------------------------------------------------------------------------

typedef enum {
    STEPPER_UIM2852_ERR_SYNTAX              = 0x32, // Instruction syntax error
    STEPPER_UIM2852_ERR_DATA                = 0x33, // Instruction data error
    STEPPER_UIM2852_ERR_SUBINDEX            = 0x34, // Instruction sub-index error
    STEPPER_UIM2852_ERR_SD_LESS_DC          = 0x3C, // SD value < DC value
    STEPPER_UIM2852_ERR_NOT_WHILE_RUNNING   = 0x3D, // Instruction not allowed while running
    STEPPER_UIM2852_ERR_BG_DRIVER_OFF       = 0x3E, // BG not allowed when driver OFF
    STEPPER_UIM2852_ERR_BG_ESTOP            = 0x3F, // BG not allowed during e-stop
    STEPPER_UIM2852_ERR_OG_WHILE_RUNNING    = 0x41, // OG not allowed while running
} stepper_uim2852_error_code_t;

// ----------------------------------------------------------------------------
// Status Structures
// ----------------------------------------------------------------------------

// MS[0] response - Status flags and relative position
typedef struct {
    // d1 flags
    uint8_t mode;           // 0=JOG, 1=PTP
    bool driver_on;         // Motor driver enabled
    bool in1_level;         // Input 1 logic level
    bool in2_level;         // Input 2 logic level
    bool in3_level;         // Input 3 logic level
    bool out1_level;        // Output 1 logic level
    
    // d2 flags
    bool stopped;           // Motor is stationary
    bool in_position;       // PAIF - motor reached target position
    bool pvt_stopped;       // PSIF - PVT stopped
    bool stall_detected;    // TLIF - stall detected
    bool system_locked;     // System locked down
    bool error_detected;    // Error flag
    
    // d4-d7: relative position
    int32_t relative_position;
} stepper_uim2852_status_t;

// Notification structure
typedef struct {
    uint8_t node_id;
    bool is_alarm;          // true if alarm, false if status
    uint8_t type;           // notification type code
    int32_t position;       // For PTP_COMPLETE, current position
} stepper_uim2852_notification_t;

// Error report structure
typedef struct {
    uint8_t error_code;
    uint8_t related_cw;     // CW that caused the error
    uint8_t subindex;       // Sub-index of the CW
} stepper_uim2852_error_t;

// ----------------------------------------------------------------------------
// Frame Building Functions
// ----------------------------------------------------------------------------
// All functions return the data length (DL) to use
// The 'data' array should be at least 8 bytes

// Motor driver on/off: MO = value (0=off, 1=on)
uint8_t stepper_uim2852_build_mo(uint8_t *data, bool enable);

// Begin motion: BG
uint8_t stepper_uim2852_build_bg(uint8_t *data);

// Stop motion (decelerate): ST
uint8_t stepper_uim2852_build_st(uint8_t *data);

// Emergency stop: ST with SD rate (or use high SD value)
uint8_t stepper_uim2852_build_emergency_stop(uint8_t *data);

// Set stop deceleration: SD = value (pulses/sec^2)
uint8_t stepper_uim2852_build_sd(uint8_t *data, uint32_t decel_rate);

// Set acceleration: AC = value (pulses/sec^2)
uint8_t stepper_uim2852_build_ac(uint8_t *data, uint32_t accel_rate);

// Set deceleration: DC = value (pulses/sec^2)
uint8_t stepper_uim2852_build_dc(uint8_t *data, uint32_t decel_rate);

// Set speed for PTP: SP = value (pulses/sec, signed for direction)
uint8_t stepper_uim2852_build_sp(uint8_t *data, int32_t speed_pps);

// Set jog velocity: JV = value (pulses/sec, signed for direction)
uint8_t stepper_uim2852_build_jv(uint8_t *data, int32_t velocity_pps);

// Set absolute position: PA = value (pulses)
uint8_t stepper_uim2852_build_pa(uint8_t *data, int32_t position);

// Set relative position: PR = value (pulses)
uint8_t stepper_uim2852_build_pr(uint8_t *data, int32_t displacement);

// Set origin: OG
uint8_t stepper_uim2852_build_og(uint8_t *data);

// Query motion status: MS[index]
uint8_t stepper_uim2852_build_ms(uint8_t *data, uint8_t index);

// Clear status flags: MS[0] = 0
uint8_t stepper_uim2852_build_ms_clear(uint8_t *data);

// Query/set parameter: PP[index] or PP[index] = value
uint8_t stepper_uim2852_build_pp_query(uint8_t *data, uint8_t index);
uint8_t stepper_uim2852_build_pp_set(uint8_t *data, uint8_t index, int32_t value);

// Query/set internal config: IC[index] or IC[index] = value
uint8_t stepper_uim2852_build_ic_query(uint8_t *data, uint8_t index);
uint8_t stepper_uim2852_build_ic_set(uint8_t *data, uint8_t index, int32_t value);

// Query/set information enable: IE[index] or IE[index] = value
uint8_t stepper_uim2852_build_ie_query(uint8_t *data, uint8_t index);
uint8_t stepper_uim2852_build_ie_set(uint8_t *data, uint8_t index, int32_t value);

// Query/set software limits: LM[index] or LM[index] = value
uint8_t stepper_uim2852_build_lm_query(uint8_t *data, uint8_t index);
uint8_t stepper_uim2852_build_lm_set(uint8_t *data, uint8_t index, int32_t value);

// Query/set stall tolerance: QE[index] or QE[index] = value
uint8_t stepper_uim2852_build_qe_query(uint8_t *data, uint8_t index);
uint8_t stepper_uim2852_build_qe_set(uint8_t *data, uint8_t index, int32_t value);

// Brake control: MT[5] = value (0=release, 1=engage)
uint8_t stepper_uim2852_build_brake(uint8_t *data, bool engage);

// ----------------------------------------------------------------------------
// Response Parsing Functions
// ----------------------------------------------------------------------------

// Parse MS[0] response (status flags + relative position)
bool stepper_uim2852_parse_ms0(const uint8_t *data, uint8_t dl, stepper_uim2852_status_t *status);

// Parse MS[1] response (speed + absolute position)
bool stepper_uim2852_parse_ms1(const uint8_t *data, uint8_t dl, int32_t *speed_pps, int32_t *abs_position);

// Parse real-time notification (CW = 0x5A)
bool stepper_uim2852_parse_notification(const uint8_t *data, uint8_t dl, stepper_uim2852_notification_t *notif);

// Parse error report (CW = 0x0F)
bool stepper_uim2852_parse_error(const uint8_t *data, uint8_t dl, stepper_uim2852_error_t *error);

// Parse parameter query response (PP, IC, IE, LM, QE)
bool stepper_uim2852_parse_param_response(const uint8_t *data, uint8_t dl, uint8_t *index, int32_t *value);

// ----------------------------------------------------------------------------
// Utility Functions
// ----------------------------------------------------------------------------

// Get CW with ACK bit set
static inline uint8_t stepper_uim2852_cw_with_ack(uint8_t cw) {
    return cw | STEPPER_UIM2852_CW_ACK_BIT;
}

// Check if CW indicates ACK was requested
static inline bool stepper_uim2852_cw_ack_requested(uint8_t cw) {
    return (cw & STEPPER_UIM2852_CW_ACK_BIT) != 0;
}

// Get base CW without ACK bit
static inline uint8_t stepper_uim2852_cw_base(uint8_t cw) {
    return cw & ~STEPPER_UIM2852_CW_ACK_BIT;
}

#ifdef __cplusplus
}
#endif
