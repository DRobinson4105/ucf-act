#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Push Button E-Stop Driver (mxuteek HB2-ES544)
// =============================================================================
// Reads the state of a normally-closed (NC) mushroom-style emergency stop switch.
//
// Hardware:
//   - mxuteek HB2-ES544: 22mm NC red mushroom emergency stop push button
//   - Rated: AC 660V 10A (using low-voltage logic level sensing)
//   - NC switch: closed (conducting) in normal operation
//   - When pressed/activated: switch opens, breaks circuit
//   - Wired to GPIO with configurable pull-up/pull-down
//
// Safety:
//   - NC design is fail-safe: broken wire = e-stop triggered
//   - Active state means e-stop is engaged (autonomy blocked)
//   - Must return to inactive state before autonomy can resume
// =============================================================================

// =============================================================================
// Configuration
// =============================================================================

// push_button_hb2es544_config_t - switch input configuration
//   gpio:            GPIO pin connected to switch
//   active_level:    Logic level when e-stop is active (0 or 1)
//   enable_pullup:   true to enable internal pull-up resistor
//   enable_pulldown: true to enable internal pull-down resistor
typedef struct {
    gpio_num_t gpio;
    int active_level;
    bool enable_pullup;
    bool enable_pulldown;
} push_button_hb2es544_config_t;

// =============================================================================
// Initialization
// =============================================================================

// Initialize GPIO for mushroom switch input
esp_err_t push_button_hb2es544_init(const push_button_hb2es544_config_t *config);

// =============================================================================
// State Reading
// =============================================================================

// Read current e-stop state
// Returns true when switch is in active (e-stop engaged) state
// Returns false when switch is in normal (safe to operate) state
bool push_button_hb2es544_read_active(const push_button_hb2es544_config_t *config);

#ifdef __cplusplus
}
#endif
