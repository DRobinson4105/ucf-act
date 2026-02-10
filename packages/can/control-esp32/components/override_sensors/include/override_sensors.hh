/**
 * @file override_sensors.hh
 * @brief Human override detection via pedal ADC and forward/reverse optocouplers.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "can_protocol.hh"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Override Sensors Driver
// ============================================================================
// Monitors driver override inputs: accelerator pedal and Forward/Reverse switch.
// Used to detect when the human driver is taking manual control.
//
// Pedal Detection:
//   - ADC reads voltage from throttle pedal potentiometer
//   - Voltage divider (270k/150k) scales pedal voltage to ADC range
//   - Above threshold = pedal pressed = override active
//   - Re-arm requires pedal released for 500ms
//
// F/R Switch Detection:
//   - Two PC817 optocouplers read Forward/Reverse switch position
//   - Debounced to avoid spurious transitions during switch bounce
//   - Change from FORWARD while autonomous = override triggered
// ============================================================================

// ============================================================================
// Type Aliases (from can_protocol.hh)
// ============================================================================

typedef uint8_t fr_state_t;        // FR_STATE_* values
typedef uint8_t override_reason_t; // OVERRIDE_REASON_* values

// ============================================================================
// Pedal ADC Thresholds
// ============================================================================
// Voltage divider: 270k/150k ratio ~0.36
// 1.0V at pedal pot wiper -> ~360mV at ADC input
// 12-bit ADC, 3.3V full scale: 360mV = ~450 counts

#define PEDAL_ADC_THRESHOLD_MV   360   // Millivolt threshold at ADC pin
#define PEDAL_ADC_THRESHOLD_RAW  450   // Raw ADC count threshold (12-bit)

// ============================================================================
// Timing Constants
// ============================================================================

#define PEDAL_REARM_MS           500   // Pedal must be released this long to re-arm
#define FR_DEBOUNCE_MS           20    // F/R switch debounce time

// ============================================================================
// Configuration
// ============================================================================

// override_sensors_config_t - hardware pin assignments
//   adc_unit:            ADC peripheral (typically ADC_UNIT_1)
//   pedal_adc_channel:   ADC channel connected to pedal voltage divider
//   fr_dir_gpio:         GPIO for direction enable optocoupler (PC817 #1)
//   fr_rev_gpio:         GPIO for reverse optocoupler (PC817 #2)
typedef struct {
    adc_unit_t adc_unit;
    adc_channel_t pedal_adc_channel;
    gpio_num_t fr_dir_gpio;
    gpio_num_t fr_rev_gpio;
} override_sensors_config_t;

// ============================================================================
// Initialization
// ============================================================================

// Initialize ADC for pedal sensing and GPIOs for F/R optocouplers
// Optocoupler inputs configured with internal pull-ups
esp_err_t override_sensors_init(const override_sensors_config_t *config);

// ============================================================================
// Pedal Detection
// ============================================================================

// Immediate pedal check (no debounce) - true if ADC above threshold
// Use for safety-critical override detection in main loop
bool override_sensors_pedal_pressed(void);

// Check if pedal has been released long enough to re-arm (500ms below threshold)
// Must be true before allowing transition back to autonomous mode
bool override_sensors_pedal_rearmed(void);

// Get raw pedal ADC value in millivolts (for diagnostics/telemetry)
uint16_t override_sensors_get_pedal_mv(void);

// ============================================================================
// F/R Switch Detection
// ============================================================================
// Optocoupler logic (active-low: LOW = switch closed = LED on = conducting):
//   fr_dir_gpio LOW  + fr_rev_gpio HIGH = Forward
//   fr_dir_gpio LOW  + fr_rev_gpio LOW  = Reverse
//   fr_dir_gpio HIGH + fr_rev_gpio HIGH = Neutral
//   fr_dir_gpio HIGH + fr_rev_gpio LOW  = Invalid (fault condition)

// Get debounced F/R state (use for control logic)
// Returns FR_STATE_FORWARD, FR_STATE_REVERSE, FR_STATE_NEUTRAL, or FR_STATE_INVALID
fr_state_t override_sensors_get_fr_state(void);

// Get raw (non-debounced) F/R state (for diagnostics only)
fr_state_t override_sensors_get_fr_state_raw(void);

// ============================================================================
// Runtime Update
// ============================================================================

// Call every 10-20ms to update debounce state and re-arm tracking
// Must be called regularly from main loop
void override_sensors_update(uint32_t now_ms);

// ============================================================================
// Status Flags
#ifdef __cplusplus
}
#endif
