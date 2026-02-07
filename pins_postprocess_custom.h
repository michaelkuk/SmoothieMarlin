/**
 * Custom pin overrides for Smoothieboard
 *
 * This file is appended to pins_postprocess.h during the build process.
 * Add any board-specific pin overrides here.
 */

// Disable FAN1_PIN to free the pin for spindle control
#undef FAN1_PIN
#define FAN1_PIN -1
