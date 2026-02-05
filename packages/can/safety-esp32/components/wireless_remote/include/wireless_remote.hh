#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Wireless Remote E-Stop Driver
// =============================================================================
// Reads the state of a wireless remote control e-stop receiver.
//
// Hardware:
//   - Wireless receiver module (e.g., 433MHz or 2.4GHz)
//   - Output pin goes to active level when remote button pressed
//   - Wired to GPIO with configurable pull-up/pull-down
//
// Operation:
//   - Remote is carried by safety operator during autonomous testing
//   - Pressing remote button triggers e-stop (blocks autonomy)
//   - Releasing button allows autonomy to resume (after conditions met)
//
// Safety:
//   - Provides wireless "dead man's switch" capability
//   - Loss of signal should be treated as e-stop (fail-safe)
//   - Range-limited to keep operator in visual contact with vehicle
// =============================================================================

// =============================================================================
// Configuration
// =============================================================================

// wireless_remote_config_t - receiver input configuration
//   gpio:            GPIO pin connected to receiver output
//   active_level:    Logic level when remote button is pressed (0 or 1)
//   enable_pullup:   true to enable internal pull-up resistor
//   enable_pulldown: true to enable internal pull-down resistor
typedef struct {
    gpio_num_t gpio;
    int active_level;
    bool enable_pullup;
    bool enable_pulldown;
} wireless_remote_config_t;

// =============================================================================
// Initialization
// =============================================================================

// Initialize GPIO for wireless remote receiver input
esp_err_t wireless_remote_init(const wireless_remote_config_t *config);

// =============================================================================
// State Reading
// =============================================================================

// Check if wireless remote e-stop is active
// Returns true when remote button is pressed (e-stop engaged)
// Returns false when remote button is released (safe to operate)
bool wireless_remote_is_active(const wireless_remote_config_t *config);

#ifdef __cplusplus
}
#endif
