/**
 * @file stepper_protocol_uim2852.cpp
 * @brief UIM2852 stepper motor SimpleCAN3.0 protocol implementation.
 *
 * Frame data lengths and formats match the UIM342CA CAN Interface Reference.
 */
#include "stepper_protocol_uim2852.h"
#include <string.h>

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Pack a 32-bit signed value into a byte buffer as little-endian.
 *
 * Stores the four bytes of value at dst[0]..dst[3] in little-endian
 * order (least significant byte first).
 *
 * @param dst    [out] Destination buffer (must have room for at least 4 bytes)
 * @param value  The 32-bit signed value to pack
 */
static void pack_le32(uint8_t *dst, int32_t value)
{
	uint32_t u = (uint32_t)value;
	dst[0] = (uint8_t)(u & 0xFF);
	dst[1] = (uint8_t)((u >> 8) & 0xFF);
	dst[2] = (uint8_t)((u >> 16) & 0xFF);
	dst[3] = (uint8_t)((u >> 24) & 0xFF);
}

/**
 * @brief Unpack a 32-bit signed value from a little-endian byte buffer.
 *
 * Reads four bytes at src[0]..src[3] in little-endian order and
 * returns them as a signed 32-bit integer.
 *
 * @param src  Source buffer (must contain at least 4 bytes)
 * @return The unpacked 32-bit signed value
 */
static int32_t unpack_le32(const uint8_t *src)
{
	uint32_t u = (uint32_t)src[0] | ((uint32_t)src[1] << 8) | ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
	return (int32_t)u;
}

/**
 * @brief Unpack a 24-bit signed value from little-endian bytes with sign extension.
 *
 * Reads three bytes at src[0]..src[2] in little-endian order and
 * sign-extends the 24-bit result to a full 32-bit signed integer.
 * Used for parsing speed values in MS[1] responses.
 *
 * @param src  Source buffer (must contain at least 3 bytes)
 * @return The unpacked and sign-extended 32-bit value
 */
static int32_t unpack_le24_signed(const uint8_t *src)
{
	uint32_t u = (uint32_t)src[0] | ((uint32_t)src[1] << 8) | ((uint32_t)src[2] << 16);
	// Sign extend if bit 23 is set
	if (u & 0x800000)
	{
		u |= 0xFF000000;
	}
	return (int32_t)u;
}

/**
 * @brief Pack a 16-bit unsigned value into a byte buffer as little-endian.
 *
 * Stores the low byte at dst[0] and the high byte at dst[1].
 *
 * @param dst    [out] Destination buffer (must have room for at least 2 bytes)
 * @param value  The 16-bit unsigned value to pack
 */
static void pack_le16(uint8_t *dst, uint16_t value)
{
	dst[0] = (uint8_t)(value & 0xFF);
	dst[1] = (uint8_t)((value >> 8) & 0xFF);
}

/**
 * @brief Pack a signed 24-bit value into a byte buffer as little-endian.
 *
 * Stores the least-significant 24 bits of @p value in dst[0]..dst[2].
 * Used by the QF instruction, whose velocity field is 24-bit signed.
 *
 * @param dst    [out] Destination buffer (must have room for at least 3 bytes)
 * @param value  Signed value to store
 */
static void pack_le24_signed(uint8_t *dst, int32_t value)
{
	uint32_t u = (uint32_t)value;
	dst[0] = (uint8_t)(u & 0xFF);
	dst[1] = (uint8_t)((u >> 8) & 0xFF);
	dst[2] = (uint8_t)((u >> 16) & 0xFF);
}

// ============================================================================
// Frame Building Helpers (shared packing patterns)
// ============================================================================

/**
 * @brief Build a zero-payload CAN frame (DL=0).
 *
 * Clears the 8-byte data buffer and returns data length 0.
 * Used for commands that carry no parameters (BG, ST, OG, etc.).
 *
 * @param data  [out] Frame data buffer (at least 8 bytes, zeroed on return)
 * @return Data length (0)
 */
static uint8_t build_nodata(uint8_t *data)
{
	memset(data, 0, 8);
	return 0;
}

/**
 * @brief Build a frame with a single 32-bit value at d0-d3 (DL=4).
 *
 * Used for motion parameters that carry one 32-bit argument
 * (AC, DC, SD, SS, SP, JV, PA, PR, QP, QV, QT).
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param value  The 32-bit value to pack at d0-d3
 * @return Data length (4)
 */
static uint8_t build_val32(uint8_t *data, int32_t value)
{
	memset(data, 0, 8);
	pack_le32(data, value);
	return 4;
}

/**
 * @brief Build an index-only query frame (DL=1, d0=index).
 *
 * Used to query the value of an indexed parameter (MS, PP, IC, IE, MT, LM, QE).
 * The motor responds with the current value at the given subindex.
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Subindex to query
 * @return Data length (1)
 */
static uint8_t build_idx_query(uint8_t *data, uint8_t index)
{
	memset(data, 0, 8);
	data[0] = index;
	return 1;
}

/**
 * @brief Build an indexed 16-bit set frame (DL=3, d0=index, d1-d2=value LE).
 *
 * Used to write a 16-bit unsigned parameter at a given subindex
 * (IC, IE, MT, QE configuration registers).
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Subindex to write
 * @param value  16-bit value to set
 * @return Data length (3)
 */
static uint8_t build_idx_u16(uint8_t *data, uint8_t index, uint16_t value)
{
	memset(data, 0, 8);
	data[0] = index;
	pack_le16(&data[1], value);
	return 3;
}

/**
 * @brief Build an indexed 32-bit set frame (DL=5, d0=index, d1-d4=value LE).
 *
 * Used to write a 32-bit signed parameter at a given subindex
 * (LM software limits).
 *
 * @param data   [out] Frame data buffer (at least 8 bytes)
 * @param index  Subindex to write
 * @param value  32-bit signed value to set
 * @return Data length (5)
 */
static uint8_t build_idx_s32(uint8_t *data, uint8_t index, int32_t value)
{
	memset(data, 0, 8);
	data[0] = index;
	pack_le32(&data[1], value);
	return 5;
}

/**
 * @brief Build a frame with two packed 32-bit values (DL=8, d0-d3 + d4-d7).
 *
 * Used for PT/QF commands that carry a position-time pair.
 *
 * @param data  [out] Frame data buffer (at least 8 bytes)
 * @param a     First 32-bit value (packed at d0-d3)
 * @param b     Second 32-bit value (packed at d4-d7)
 * @return Data length (8)
 */
static uint8_t build_val32_pair(uint8_t *data, int32_t a, int32_t b)
{
	memset(data, 0, 8);
	pack_le32(&data[0], a);
	pack_le32(&data[4], b);
	return 8;
}

// ============================================================================
// Frame Building Functions
// ============================================================================

uint8_t stepper_uim2852_build_mo(uint8_t *data, bool enable)
{
	memset(data, 0, 8);
	data[0] = enable ? 1 : 0;
	return 1;
}

uint8_t stepper_uim2852_build_bg(uint8_t *data)
{
	return build_nodata(data);
}

uint8_t stepper_uim2852_build_st(uint8_t *data)
{
	return build_nodata(data);
}

uint8_t stepper_uim2852_build_sd(uint8_t *data, uint32_t decel_rate)
{
	return build_val32(data, (int32_t)decel_rate);
}

uint8_t stepper_uim2852_build_ac(uint8_t *data, uint32_t accel_rate)
{
	return build_val32(data, (int32_t)accel_rate);
}

uint8_t stepper_uim2852_build_dc(uint8_t *data, uint32_t decel_rate)
{
	return build_val32(data, (int32_t)decel_rate);
}

uint8_t stepper_uim2852_build_ss(uint8_t *data, uint32_t speed_pps)
{
	return build_val32(data, (int32_t)speed_pps);
}

uint8_t stepper_uim2852_build_sp(uint8_t *data, int32_t speed_pps)
{
	return build_val32(data, speed_pps);
}

uint8_t stepper_uim2852_build_jv(uint8_t *data, int32_t velocity_pps)
{
	return build_val32(data, velocity_pps);
}

uint8_t stepper_uim2852_build_pa(uint8_t *data, int32_t position)
{
	return build_val32(data, position);
}

uint8_t stepper_uim2852_build_pr(uint8_t *data, int32_t displacement)
{
	return build_val32(data, displacement);
}

uint8_t stepper_uim2852_build_og(uint8_t *data)
{
	return build_nodata(data);
}

uint8_t stepper_uim2852_build_ms(uint8_t *data, uint8_t index)
{
	return build_idx_query(data, index);
}

uint8_t stepper_uim2852_build_ms_clear(uint8_t *data)
{
	memset(data, 0, 8);
	data[0] = 0; // Index
	data[1] = 0; // Value = 0
	return 2;
}

uint8_t stepper_uim2852_build_pp_query(uint8_t *data, uint8_t index)
{
	return build_idx_query(data, index);
}
uint8_t stepper_uim2852_build_pp_set(uint8_t *data, uint8_t index, uint8_t value)
{
	memset(data, 0, 8);
	data[0] = index;
	data[1] = value;
	return 2;
}

uint8_t stepper_uim2852_build_ic_query(uint8_t *data, uint8_t index)
{
	return build_idx_query(data, index);
}
uint8_t stepper_uim2852_build_ic_set(uint8_t *data, uint8_t index, uint16_t value)
{
	return build_idx_u16(data, index, value);
}

uint8_t stepper_uim2852_build_ie_query(uint8_t *data, uint8_t index)
{
	return build_idx_query(data, index);
}
uint8_t stepper_uim2852_build_ie_set(uint8_t *data, uint8_t index, uint16_t value)
{
	return build_idx_u16(data, index, value);
}

uint8_t stepper_uim2852_build_mt_query(uint8_t *data, uint8_t index)
{
	return build_idx_query(data, index);
}
uint8_t stepper_uim2852_build_mt_set(uint8_t *data, uint8_t index, uint16_t value)
{
	return build_idx_u16(data, index, value);
}

uint8_t stepper_uim2852_build_lm_query(uint8_t *data, uint8_t index)
{
	return build_idx_query(data, index);
}
uint8_t stepper_uim2852_build_lm_set(uint8_t *data, uint8_t index, int32_t value)
{
	return build_idx_s32(data, index, value);
}

uint8_t stepper_uim2852_build_qe_query(uint8_t *data, uint8_t index)
{
	return build_idx_query(data, index);
}
uint8_t stepper_uim2852_build_qe_set(uint8_t *data, uint8_t index, uint16_t value)
{
	return build_idx_u16(data, index, value);
}


uint8_t stepper_uim2852_build_ml(uint8_t *data)
{
	return build_nodata(data);
}

uint8_t stepper_uim2852_build_sn(uint8_t *data)
{
	return build_nodata(data);
}

// ============================================================================
// PT/PVT Interpolated Motion Frame Builders
// ============================================================================

uint8_t stepper_uim2852_build_pv(uint8_t *data, uint16_t start_row)
{
	memset(data, 0, 8);
	pack_le16(data, start_row);
	return 2;
}

uint8_t stepper_uim2852_build_pt(uint8_t *data, uint16_t row, int32_t position)
{
	memset(data, 0, 8);
	pack_le16(&data[0], row);
	pack_le32(&data[2], position);
	return 8;
}

uint8_t stepper_uim2852_build_qp(uint8_t *data, int32_t position)
{
	return build_val32(data, position);
}

uint8_t stepper_uim2852_build_qv(uint8_t *data, int32_t velocity)
{
	return build_val32(data, velocity);
}

uint8_t stepper_uim2852_build_qt(uint8_t *data, uint32_t time_ms)
{
	return build_val32(data, (int32_t)time_ms);
}

uint8_t stepper_uim2852_build_qf(uint8_t *data, uint8_t time_ms, int32_t velocity, int32_t position)
{
	memset(data, 0, 8);
	data[0] = time_ms;
	pack_le24_signed(&data[1], velocity);
	pack_le32(&data[4], position);
	return 8;
}

uint8_t stepper_uim2852_build_bl(uint8_t *data, uint16_t pulses)
{
	memset(data, 0, 8);
	pack_le16(data, pulses);
	return 2;
}

// ============================================================================
// Response Parsing Functions
// ============================================================================

bool stepper_uim2852_parse_ms0(const uint8_t *data, uint8_t dl, stepper_uim2852_status_t *status)
{
	if (!data || !status || dl < 8)
		return false;

	// Verify this is an MS[0] response (index must be 0)
	if (data[0] != STEPPER_UIM2852_MS_FLAGS_RELPOS)
		return false;

	// d1: status flags byte 1
	uint8_t d1 = data[1];
	status->mode = d1 & 0x03;              // bits 0-1
	status->driver_on = (d1 & 0x04) != 0;  // bit 2
	status->in1_level = (d1 & 0x08) != 0;  // bit 3
	status->in2_level = (d1 & 0x10) != 0;  // bit 4
	status->in3_level = (d1 & 0x20) != 0;  // bit 5
	status->out1_level = (d1 & 0x40) != 0; // bit 6

	// d2: status flags byte 2
	uint8_t d2 = data[2];
	status->stopped = (d2 & 0x01) != 0;        // bit 0
	status->in_position = (d2 & 0x02) != 0;    // bit 1 (PAIF)
	status->pvt_stopped = (d2 & 0x04) != 0;    // bit 2 (PSIF)
	status->stall_detected = (d2 & 0x08) != 0; // bit 3 (TLIF)
	status->system_locked = (d2 & 0x20) != 0;  // bit 5
	status->error_detected = (d2 & 0x80) != 0; // bit 7

	// d4-d7: relative position (32-bit signed, little-endian)
	status->relative_position = unpack_le32(&data[4]);

	return true;
}

bool stepper_uim2852_parse_ms1(const uint8_t *data, uint8_t dl, int32_t *speed_pps, int32_t *abs_position)
{
	if (!data || dl < 8)
		return false;

	// Verify this is an MS[1] response (index must be 1)
	if (data[0] != STEPPER_UIM2852_MS_SPEED_ABSPOS)
		return false;

	// d1-d3: current speed (24-bit signed, little-endian)
	if (speed_pps)
		*speed_pps = unpack_le24_signed(&data[1]);

	// d4-d7: absolute position (32-bit signed, little-endian)
	if (abs_position)
		*abs_position = unpack_le32(&data[4]);

	return true;
}

bool stepper_uim2852_parse_notification(const uint8_t *data, uint8_t dl, stepper_uim2852_notification_t *notif)
{
	if (!data || !notif || dl < 2)
		return false;

	uint8_t d0 = data[0];
	uint8_t d1 = data[1];

	// Check if this is an alarm (d0 == 0x00) or status notification
	if (d0 == 0x00)
	{
		notif->is_alarm = true;
		notif->type = d1; // Alarm code in d1
	}
	else
	{
		notif->is_alarm = false;
		notif->type = d0; // Status type in d0

		// For PTP complete, extract position from d4-d7
		if (d0 == STEPPER_UIM2852_STATUS_PTP_COMPLETE && dl >= 8)
			notif->position = unpack_le32(&data[4]);
		else
			notif->position = 0;
	}

	return true;
}

bool stepper_uim2852_parse_error(const uint8_t *data, uint8_t dl, stepper_uim2852_error_t *error)
{
	if (!data || !error || dl < 4)
		return false;

	error->error_code = data[1];
	error->related_cw = data[2];
	error->subindex = data[3];

	return true;
}

bool stepper_uim2852_parse_param_response(const uint8_t *data, uint8_t dl, uint8_t *index, int32_t *value)
{
	if (!data || dl < 2)
		return false;

	if (index)
		*index = data[0];

	if (value)
	{
		if (dl >= 5)
			*value = unpack_le32(&data[1]);
		else if (dl >= 3)
			// 16-bit unsigned value (IC, IE, MT, QE params are all unsigned)
			*value = (int32_t)((uint16_t)data[1] | ((uint16_t)data[2] << 8));
		else
			// 8-bit value
			*value = (int32_t)(uint8_t)data[1];
	}

	return true;
}
