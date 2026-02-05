#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// NC Mushroom E-Stop Switch Driver
// =============================================================================
// Reads the state of a normally-closed (NC) mushroom-style emergency stop switch.
//
// Hardware:
//   - Physical red mushroom button mounted on vehicle
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

// nc_mushroom_config_t - switch input configuration
//   gpio:            GPIO pin connected to switch
//   active_level:    Logic level when e-stop is active (0 or 1)
//   enable_pullup:   true to enable internal pull-up resistor
//   enable_pulldown: true to enable internal pull-down resistor
typedef struct {
    gpio_num_t gpio;
    int active_level;
    bool enable_pullup;
    bool enable_pulldown;
} nc_mushroom_config_t;

// =============================================================================
// Initialization
// =============================================================================

// Initialize GPIO for mushroom switch input
esp_err_t nc_mushroom_init(const nc_mushroom_config_t *config);

// =============================================================================
// State Reading
// =============================================================================

// Read current e-stop state
// Returns true when switch is in active (e-stop engaged) state
// Returns false when switch is in normal (safe to operate) state
bool nc_mushroom_read_active(const nc_mushroom_config_t *config);

#ifdef __cplusplus
}
#endif
