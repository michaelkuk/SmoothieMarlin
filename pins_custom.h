/**
 * Custom pin overrides for Smoothieboard
 * This file is included AFTER the board pins file, so #undef works here
 */

// Free up P2_04 for spindle PWM by disabling FAN1
#undef FAN1_PIN
#define FAN1_PIN -1
