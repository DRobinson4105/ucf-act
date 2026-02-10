/**
 * @file stepper_protocol_uim2852.c
 * @brief UIM2852 stepper motor SimpleCAN protocol implementation.
 */
#include "stepper_protocol_uim2852.h"
#include <string.h>

// ============================================================================
// Helper Functions
// ============================================================================

// Pack 32-bit signed value into little-endian bytes
static void pack_le32(uint8_t *dst, int32_t value) {
    uint32_t u = (uint32_t)value;
    dst[0] = (uint8_t)(u & 0xFF);
    dst[1] = (uint8_t)((u >> 8) & 0xFF);
    dst[2] = (uint8_t)((u >> 16) & 0xFF);
    dst[3] = (uint8_t)((u >> 24) & 0xFF);
}

// Unpack 32-bit signed value from little-endian bytes
static int32_t unpack_le32(const uint8_t *src) {
    uint32_t u = (uint32_t)src[0] |
                 ((uint32_t)src[1] << 8) |
                 ((uint32_t)src[2] << 16) |
                 ((uint32_t)src[3] << 24);
    return (int32_t)u;
}

// Unpack 24-bit signed value from little-endian bytes (sign extend to 32-bit)
static int32_t unpack_le24_signed(const uint8_t *src) {
    uint32_t u = (uint32_t)src[0] |
                 ((uint32_t)src[1] << 8) |
                 ((uint32_t)src[2] << 16);
    // Sign extend if bit 23 is set
    if (u & 0x800000) {
        u |= 0xFF000000;
    }
    return (int32_t)u;
}

// ============================================================================
// Frame Building Functions
// ============================================================================

uint8_t stepper_uim2852_build_mo(uint8_t *data, bool enable) {
    memset(data, 0, 8);
    data[0] = enable ? 1 : 0;
    return 1;
}

uint8_t stepper_uim2852_build_bg(uint8_t *data) {
    memset(data, 0, 8);
    return 0;  // No data bytes
}

uint8_t stepper_uim2852_build_st(uint8_t *data) {
    memset(data, 0, 8);
    return 0;  // No data bytes
}

uint8_t stepper_uim2852_build_emergency_stop(uint8_t *data) {
    // Emergency stop: set very high SD rate then stop
    // Or simply stop - the motor will use current SD
    memset(data, 0, 8);
    return 0;
}

uint8_t stepper_uim2852_build_sd(uint8_t *data, uint32_t decel_rate) {
    memset(data, 0, 8);
    pack_le32(data, (int32_t)decel_rate);
    return 4;
}

uint8_t stepper_uim2852_build_ac(uint8_t *data, uint32_t accel_rate) {
    memset(data, 0, 8);
    pack_le32(data, (int32_t)accel_rate);
    return 4;
}

uint8_t stepper_uim2852_build_dc(uint8_t *data, uint32_t decel_rate) {
    memset(data, 0, 8);
    pack_le32(data, (int32_t)decel_rate);
    return 4;
}

uint8_t stepper_uim2852_build_sp(uint8_t *data, int32_t speed_pps) {
    memset(data, 0, 8);
    pack_le32(data, speed_pps);
    return 4;
}

uint8_t stepper_uim2852_build_jv(uint8_t *data, int32_t velocity_pps) {
    memset(data, 0, 8);
    pack_le32(data, velocity_pps);
    return 4;
}

uint8_t stepper_uim2852_build_pa(uint8_t *data, int32_t position) {
    memset(data, 0, 8);
    pack_le32(data, position);
    return 4;
}

uint8_t stepper_uim2852_build_pr(uint8_t *data, int32_t displacement) {
    memset(data, 0, 8);
    pack_le32(data, displacement);
    return 4;
}

uint8_t stepper_uim2852_build_og(uint8_t *data) {
    memset(data, 0, 8);
    return 0;  // No data bytes
}

uint8_t stepper_uim2852_build_ms(uint8_t *data, uint8_t index) {
    memset(data, 0, 8);
    data[0] = index;
    return 1;
}

uint8_t stepper_uim2852_build_ms_clear(uint8_t *data) {
    memset(data, 0, 8);
    data[0] = 0;  // Index
    data[1] = 0;  // Value = 0
    return 2;
}

uint8_t stepper_uim2852_build_pp_query(uint8_t *data, uint8_t index) {
    memset(data, 0, 8);
    data[0] = index;
    return 1;
}

uint8_t stepper_uim2852_build_pp_set(uint8_t *data, uint8_t index, int32_t value) {
    memset(data, 0, 8);
    data[0] = index;
    pack_le32(&data[1], value);
    return 5;
}

uint8_t stepper_uim2852_build_ic_query(uint8_t *data, uint8_t index) {
    memset(data, 0, 8);
    data[0] = index;
    return 1;
}

uint8_t stepper_uim2852_build_ic_set(uint8_t *data, uint8_t index, int32_t value) {
    memset(data, 0, 8);
    data[0] = index;
    pack_le32(&data[1], value);
    return 5;
}

uint8_t stepper_uim2852_build_ie_query(uint8_t *data, uint8_t index) {
    memset(data, 0, 8);
    data[0] = index;
    return 1;
}

uint8_t stepper_uim2852_build_ie_set(uint8_t *data, uint8_t index, int32_t value) {
    memset(data, 0, 8);
    data[0] = index;
    pack_le32(&data[1], value);
    return 5;
}

uint8_t stepper_uim2852_build_lm_query(uint8_t *data, uint8_t index) {
    memset(data, 0, 8);
    data[0] = index;
    return 1;
}

uint8_t stepper_uim2852_build_lm_set(uint8_t *data, uint8_t index, int32_t value) {
    memset(data, 0, 8);
    data[0] = index;
    pack_le32(&data[1], value);
    return 5;
}

uint8_t stepper_uim2852_build_qe_query(uint8_t *data, uint8_t index) {
    memset(data, 0, 8);
    data[0] = index;
    return 1;
}

uint8_t stepper_uim2852_build_qe_set(uint8_t *data, uint8_t index, int32_t value) {
    memset(data, 0, 8);
    data[0] = index;
    pack_le32(&data[1], value);
    return 5;
}

uint8_t stepper_uim2852_build_brake(uint8_t *data, bool engage) {
    memset(data, 0, 8);
    data[0] = STEPPER_UIM2852_MT_BRAKE;  // Index = 5
    data[1] = engage ? 1 : 0;
    return 2;
}

// ============================================================================
// Response Parsing Functions
// ============================================================================

bool stepper_uim2852_parse_ms0(const uint8_t *data, uint8_t dl, stepper_uim2852_status_t *status) {
    if (!data || !status || dl < 8) return false;
    
    // d1: status flags byte 1
    uint8_t d1 = data[1];
    status->mode = d1 & 0x03;               // bits 0-1
    status->driver_on = (d1 & 0x04) != 0;   // bit 2
    status->in1_level = (d1 & 0x08) != 0;   // bit 3
    status->in2_level = (d1 & 0x10) != 0;   // bit 4
    status->in3_level = (d1 & 0x20) != 0;   // bit 5
    status->out1_level = (d1 & 0x40) != 0;  // bit 6
    
    // d2: status flags byte 2
    uint8_t d2 = data[2];
    status->stopped = (d2 & 0x01) != 0;         // bit 0
    status->in_position = (d2 & 0x02) != 0;     // bit 1 (PAIF)
    status->pvt_stopped = (d2 & 0x04) != 0;     // bit 2 (PSIF)
    status->stall_detected = (d2 & 0x08) != 0;  // bit 3 (TLIF)
    status->system_locked = (d2 & 0x20) != 0;   // bit 5
    status->error_detected = (d2 & 0x80) != 0;  // bit 7
    
    // d4-d7: relative position (32-bit signed, little-endian)
    status->relative_position = unpack_le32(&data[4]);
    
    return true;
}

bool stepper_uim2852_parse_ms1(const uint8_t *data, uint8_t dl, int32_t *speed_pps, int32_t *abs_position) {
    if (!data || dl < 8) return false;
    
    // d1-d3: current speed (24-bit signed, little-endian)
    if (speed_pps) *speed_pps = unpack_le24_signed(&data[1]);
    
    // d4-d7: absolute position (32-bit signed, little-endian)
    if (abs_position) *abs_position = unpack_le32(&data[4]);
    
    return true;
}

bool stepper_uim2852_parse_notification(const uint8_t *data, uint8_t dl, stepper_uim2852_notification_t *notif) {
    if (!data || !notif || dl < 2) return false;
    
    uint8_t d0 = data[0];
    uint8_t d1 = data[1];
    
    // Check if this is an alarm (d0 == 0x00) or status notification
    if (d0 == 0x00) {
        notif->is_alarm = true;
        notif->type = d1;  // Alarm code in d1
    } else {
        notif->is_alarm = false;
        notif->type = d0;  // Status type in d0
        
        // For PTP complete, extract position from d4-d7
        if (d0 == STEPPER_UIM2852_STATUS_PTP_COMPLETE && dl >= 8)
            notif->position = unpack_le32(&data[4]);
        else notif->position = 0;
    }
    
    return true;
}

bool stepper_uim2852_parse_error(const uint8_t *data, uint8_t dl, stepper_uim2852_error_t *error) {
    if (!data || !error || dl < 4) return false;
    
    error->error_code = data[1];
    error->related_cw = data[2];
    error->subindex = data[3];
    
    return true;
}

bool stepper_uim2852_parse_param_response(const uint8_t *data, uint8_t dl, uint8_t *index, int32_t *value) {
    if (!data || dl < 2) return false;
    
    if (index) *index = data[0];
    
    if (value) {
        if (dl >= 5) *value = unpack_le32(&data[1]);
        else if (dl >= 3)
            // 16-bit value
            *value = (int32_t)(int16_t)((uint16_t)data[1] | ((uint16_t)data[2] << 8));
        else
            // 8-bit value
            *value = (int32_t)(int8_t)data[1];
    }
    
    return true;
}
