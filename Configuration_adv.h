/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * Configuration_adv.h
 *
 * Advanced settings.
 * Only change these if you know exactly what you're doing.
 * Some of these settings can damage your printer if improperly set!
 *
 * Basic settings can be found in Configuration.h
 */
#define CONFIGURATION_ADV_H_VERSION 02010300

// @section develop

//===========================================================================
//============================= Thermal Settings ============================
//===========================================================================
// @section temperature

/**
 * Thermocouple sensors are quite sensitive to noise.  Any noise induced in
 * the sensor wires, such as by stepper motor wires run in parallel to them,
 * may result in the thermocouple sensor reporting spurious errors.  This
 * value is the number of errors which can occur in a row before the error
 * is reported.  This allows us to ignore intermittent error conditions while
 * still detecting an actual failure, which should result in a continuous
 * stream of errors from the sensor.
 *
 * Set this value to 0 to fail on the first error to occur.
 */
#define THERMOCOUPLE_MAX_ERRORS 15

#if TEMP_SENSOR_0 == 1000
  #define HOTEND0_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND0_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND0_BETA                    3950 // Beta value
  #define HOTEND0_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_1 == 1000
  #define HOTEND1_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND1_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND1_BETA                    3950 // Beta value
  #define HOTEND1_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_2 == 1000
  #define HOTEND2_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND2_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND2_BETA                    3950 // Beta value
  #define HOTEND2_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_3 == 1000
  #define HOTEND3_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND3_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND3_BETA                    3950 // Beta value
  #define HOTEND3_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_4 == 1000
  #define HOTEND4_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND4_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND4_BETA                    3950 // Beta value
  #define HOTEND4_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_5 == 1000
  #define HOTEND5_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND5_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND5_BETA                    3950 // Beta value
  #define HOTEND5_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_6 == 1000
  #define HOTEND6_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND6_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND6_BETA                    3950 // Beta value
  #define HOTEND6_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_7 == 1000
  #define HOTEND7_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define HOTEND7_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define HOTEND7_BETA                    3950 // Beta value
  #define HOTEND7_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_BED == 1000
  #define BED_PULLUP_RESISTOR_OHMS        4700 // Pullup resistor
  #define BED_RESISTANCE_25C_OHMS       100000 // Resistance at 25C
  #define BED_BETA                        3950 // Beta value
  #define BED_SH_C_COEFF                     0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_CHAMBER == 1000
  #define CHAMBER_PULLUP_RESISTOR_OHMS    4700 // Pullup resistor
  #define CHAMBER_RESISTANCE_25C_OHMS   100000 // Resistance at 25C
  #define CHAMBER_BETA                    3950 // Beta value
  #define CHAMBER_SH_C_COEFF                 0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_COOLER == 1000
  #define COOLER_PULLUP_RESISTOR_OHMS     4700 // Pullup resistor
  #define COOLER_RESISTANCE_25C_OHMS    100000 // Resistance at 25C
  #define COOLER_BETA                     3950 // Beta value
  #define COOLER_SH_C_COEFF                  0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_PROBE == 1000
  #define PROBE_PULLUP_RESISTOR_OHMS      4700 // Pullup resistor
  #define PROBE_RESISTANCE_25C_OHMS     100000 // Resistance at 25C
  #define PROBE_BETA                      3950 // Beta value
  #define PROBE_SH_C_COEFF                   0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_BOARD == 1000
  #define BOARD_PULLUP_RESISTOR_OHMS      4700 // Pullup resistor
  #define BOARD_RESISTANCE_25C_OHMS     100000 // Resistance at 25C
  #define BOARD_BETA                      3950 // Beta value
  #define BOARD_SH_C_COEFF                   0 // Steinhart-Hart C coefficient
#endif

#if TEMP_SENSOR_REDUNDANT == 1000
  #define REDUNDANT_PULLUP_RESISTOR_OHMS  4700 // Pullup resistor
  #define REDUNDANT_RESISTANCE_25C_OHMS 100000 // Resistance at 25C
  #define REDUNDANT_BETA                  3950 // Beta value
  #define REDUNDANT_SH_C_COEFF               0 // Steinhart-Hart C coefficient
#endif

#if ENABLED(HEPHESTOS2_HEATED_BED_KIT)
  #define HEATER_BED_INVERTING true
#endif

#if DISABLED(PIDTEMPBED)
  #define BED_CHECK_INTERVAL 5000   // (ms) Interval between checks in bang-bang control
  #if ANY(BED_LIMIT_SWITCHING, PELTIER_BED)
    #define BED_HYSTERESIS 2        // (°C) Only set the relevant heater state when ABS(T-target) > BED_HYSTERESIS
  #endif
#endif

#if TEMP_SENSOR_CHAMBER

  #if DISABLED(PIDTEMPCHAMBER)
    #define CHAMBER_CHECK_INTERVAL 5000   // (ms) Interval between checks in bang-bang control
    #if ENABLED(CHAMBER_LIMIT_SWITCHING)
      #define CHAMBER_HYSTERESIS 2        // (°C) Only set the relevant heater state when ABS(T-target) > CHAMBER_HYSTERESIS
    #endif
  #endif

  #if ENABLED(CHAMBER_FAN)
    #define CHAMBER_FAN_MODE      2   // Fan control mode: 0=Static; 1=Linear increase when temp is higher than target; 2=V-shaped curve; 3=similar to 1 but fan is always on.
    #if CHAMBER_FAN_MODE == 0
      #define CHAMBER_FAN_BASE  255   // Chamber fan PWM (0-255)
    #elif CHAMBER_FAN_MODE == 1
      #define CHAMBER_FAN_BASE  128   // Base chamber fan PWM (0-255); turns on when chamber temperature is above the target
      #define CHAMBER_FAN_FACTOR 25   // PWM increase per °C above target
    #elif CHAMBER_FAN_MODE == 2
      #define CHAMBER_FAN_BASE  128   // Minimum chamber fan PWM (0-255)
      #define CHAMBER_FAN_FACTOR 25   // PWM increase per °C difference from target
    #elif CHAMBER_FAN_MODE == 3
      #define CHAMBER_FAN_BASE  128   // Base chamber fan PWM (0-255)
      #define CHAMBER_FAN_FACTOR 25   // PWM increase per °C above target
    #endif
  #endif

  #if ENABLED(CHAMBER_VENT)
    #define CHAMBER_VENT_SERVO_NR  1  // Index of the vent servo
    #define HIGH_EXCESS_HEAT_LIMIT 5  // How much above target temp to consider there is excess heat in the chamber
    #define LOW_EXCESS_HEAT_LIMIT  3
    #define MIN_COOLING_SLOPE_TIME_CHAMBER_VENT 20
    #define MIN_COOLING_SLOPE_DEG_CHAMBER_VENT 1.5
  #endif
#endif

#if TEMP_SENSOR_COOLER
  #define COOLER_MINTEMP           8  // (°C)
  #define COOLER_MAXTEMP          26  // (°C)
  #define COOLER_DEFAULT_TEMP     16  // (°C)
  #define TEMP_COOLER_HYSTERESIS   1  // (°C) Temperature proximity considered "close enough" to the target
  #define COOLER_PIN               8  // Laser cooler on/off pin used to control power to the cooling element (e.g., TEC, External chiller via relay)
  #define COOLER_INVERTING     false
  #define TEMP_COOLER_PIN         15  // Laser/Cooler temperature sensor pin. ADC is required.
  #define COOLER_FAN                  // Enable a fan on the cooler, Fan# 0,1,2,3 etc.
  #define COOLER_FAN_INDEX         0  // FAN number 0, 1, 2 etc. e.g.
  #if ENABLED(COOLER_FAN)
    #define COOLER_FAN_BASE      100  // Base Cooler fan PWM (0-255); turns on when Cooler temperature is above the target
    #define COOLER_FAN_FACTOR     25  // PWM increase per °C above target
  #endif
#endif

#if TEMP_SENSOR_BOARD
  #define THERMAL_PROTECTION_BOARD   // Halt the printer if the board sensor leaves the temp range below.
  #define BOARD_MINTEMP           8  // (°C)
  #define BOARD_MAXTEMP          70  // (°C)
#endif

#if TEMP_SENSOR_SOC
  #define THERMAL_PROTECTION_SOC     // Halt the printer if the SoC sensor leaves the temp range below.
  #define SOC_MAXTEMP            85  // (°C)
#endif

/**
 * Thermal Protection provides additional protection to your printer from damage
 * and fire. Marlin always includes safe min and max temperature ranges which
 * protect against a broken or disconnected thermistor wire.
 *
 * The issue: If a thermistor falls out, it will report the much lower
 * temperature of the air in the room, and the firmware will keep
 * the heater on.
 *
 * The solution: Once the temperature reaches the target, start observing.
 * If the temperature stays too far below the target (hysteresis) for too
 * long (period), the firmware will halt the machine as a safety precaution.
 *
 * If you get false positives for "Thermal Runaway", increase
 * THERMAL_PROTECTION_HYSTERESIS and/or THERMAL_PROTECTION_PERIOD
 */
#if ALL(HAS_HOTEND, THERMAL_PROTECTION_HOTENDS)
  #define THERMAL_PROTECTION_PERIOD        40 // (seconds)
  #define THERMAL_PROTECTION_HYSTERESIS     4 // (°C)

  /**
   * Whenever an M104, M109, or M303 increases the target temperature, the
   * firmware will wait for the WATCH_TEMP_PERIOD to expire. If the temperature
   * hasn't increased by WATCH_TEMP_INCREASE degrees, the machine is halted and
   * requires a hard reset. This test restarts with any M104/M109/M303, but only
   * if the current temperature is far enough below the target for a reliable
   * test.
   *
   * If you get false positives for "Heating failed", increase WATCH_TEMP_PERIOD
   * and/or decrease WATCH_TEMP_INCREASE. WATCH_TEMP_INCREASE should not be set
   * below 2.
   */
  #define WATCH_TEMP_PERIOD  40               // (seconds)
  #define WATCH_TEMP_INCREASE 2               // (°C)
#endif

/**
 * Thermal Protection parameters for the bed are just as above for hotends.
 */
#if TEMP_SENSOR_BED && ENABLED(THERMAL_PROTECTION_BED)
  #define THERMAL_PROTECTION_BED_PERIOD        20 // (seconds)
  #define THERMAL_PROTECTION_BED_HYSTERESIS     2 // (°C)

  /**
   * As described above, except for the bed (M140/M190/M303).
   */
  #define WATCH_BED_TEMP_PERIOD                60 // (seconds)
  #define WATCH_BED_TEMP_INCREASE               2 // (°C)
#endif

/**
 * Thermal Protection parameters for the heated chamber.
 */
#if TEMP_SENSOR_CHAMBER && ENABLED(THERMAL_PROTECTION_CHAMBER)
  #define THERMAL_PROTECTION_CHAMBER_PERIOD    20 // (seconds)
  #define THERMAL_PROTECTION_CHAMBER_HYSTERESIS 2 // (°C)

  /**
   * Heated chamber watch settings (M141/M191).
   */
  #define WATCH_CHAMBER_TEMP_PERIOD            60 // (seconds)
  #define WATCH_CHAMBER_TEMP_INCREASE           2 // (°C)
#endif

/**
 * Thermal Protection parameters for the laser cooler.
 */
#if TEMP_SENSOR_COOLER && ENABLED(THERMAL_PROTECTION_COOLER)
  #define THERMAL_PROTECTION_COOLER_PERIOD     10 // (seconds)
  #define THERMAL_PROTECTION_COOLER_HYSTERESIS  3 // (°C)

  /**
   * Laser cooling watch settings (M143/M193).
   */
  #define WATCH_COOLER_TEMP_PERIOD             60 // (seconds)
  #define WATCH_COOLER_TEMP_INCREASE            3 // (°C)
#endif

#if ENABLED(PIDTEMP)
  #if ENABLED(PID_EXTRUSION_SCALING)
    #define LPQ_MAX_LEN 50
    #define DEFAULT_KC 100  // heating power = Kc * e_speed
    #if ENABLED(PID_PARAMS_PER_HOTEND)
      #define DEFAULT_KC_LIST { DEFAULT_KC, DEFAULT_KC }  // heating power = Kc * e_speed
    #endif
  #endif

  #if ENABLED(PID_FAN_SCALING)
    #if ENABLED(PID_FAN_SCALING_ALTERNATIVE_DEFINITION)

      #define PID_FAN_SCALING_AT_FULL_SPEED 13.0        //=PID_FAN_SCALING_LIN_FACTOR*255+DEFAULT_KF
      #define PID_FAN_SCALING_AT_MIN_SPEED   6.0        //=PID_FAN_SCALING_LIN_FACTOR*PID_FAN_SCALING_MIN_SPEED+DEFAULT_KF
      #define PID_FAN_SCALING_MIN_SPEED     10.0        // Minimum fan speed at which to enable PID_FAN_SCALING

      #define DEFAULT_KF (255.0*PID_FAN_SCALING_AT_MIN_SPEED-PID_FAN_SCALING_AT_FULL_SPEED*PID_FAN_SCALING_MIN_SPEED)/(255.0-PID_FAN_SCALING_MIN_SPEED)
      #define PID_FAN_SCALING_LIN_FACTOR (PID_FAN_SCALING_AT_FULL_SPEED-DEFAULT_KF)/255.0

    #else
      #define PID_FAN_SCALING_LIN_FACTOR (0)             // Power-loss due to cooling = Kf * (fan_speed)
      #define DEFAULT_KF 10                              // A constant value added to the PID-tuner
      #define PID_FAN_SCALING_MIN_SPEED 10               // Minimum fan speed at which to enable PID_FAN_SCALING
    #endif
  #endif
  #if ENABLED(PID_PARAMS_PER_HOTEND)
    #define DEFAULT_KF_LIST { DEFAULT_KF, DEFAULT_KF }
  #endif
#endif

/**
 * Automatic Temperature Mode
 *
 * Dynamically adjust the hotend target temperature based on planned E moves.
 *
 * (Contrast with PID_EXTRUSION_SCALING, which tracks E movement and adjusts PID
 *  behavior using an additional kC value.)
 *
 * Autotemp is calculated by (mintemp + factor * mm_per_sec), capped to maxtemp.
 *
 * Enable Autotemp Mode with M104/M109 F<factor> S<mintemp> B<maxtemp>.
 * Disable by sending M104/M109 with no F parameter (or F0 with AUTOTEMP_PROPORTIONAL).
 */
#define AUTOTEMP
#if ENABLED(AUTOTEMP)
  #define AUTOTEMP_OLDWEIGHT    0.98  // Factor used to weight previous readings (0.0 < value < 1.0)
  #define AUTOTEMP_MIN        210
  #define AUTOTEMP_MAX        250
  #define AUTOTEMP_FACTOR       0.1f
  #if ENABLED(AUTOTEMP_PROPORTIONAL)
    #define AUTOTEMP_MIN_P      0     // (°C) Added to the target temperature
    #define AUTOTEMP_MAX_P      5     // (°C) Added to the target temperature
    #define AUTOTEMP_FACTOR_P   1     // Apply this F parameter by default (overridden by M104/M109 F)
  #endif
#endif

// @section extruder

#if ENABLED(EXTRUDER_RUNOUT_PREVENT)
  #define EXTRUDER_RUNOUT_MINTEMP 190
  #define EXTRUDER_RUNOUT_SECONDS 30
  #define EXTRUDER_RUNOUT_SPEED 1500  // (mm/min)
  #define EXTRUDER_RUNOUT_EXTRUDE 5   // (mm)
#endif

#if ENABLED(HOTEND_IDLE_TIMEOUT)
  #define HOTEND_IDLE_TIMEOUT_SEC (5*60)    // (seconds) Time without extruder movement to trigger protection
  #define HOTEND_IDLE_MIN_TRIGGER   180     // (°C) Minimum temperature to enable hotend protection
  #define HOTEND_IDLE_NOZZLE_TARGET   0     // (°C) Safe temperature for the nozzle after timeout
  #define HOTEND_IDLE_BED_TARGET      0     // (°C) Safe temperature for the bed after timeout
#endif

// @section temperature

#define TEMP_SENSOR_AD595_OFFSET  0.0
#define TEMP_SENSOR_AD595_GAIN    1.0
#define TEMP_SENSOR_AD8495_OFFSET 0.0
#define TEMP_SENSOR_AD8495_GAIN   1.0

// @section fans

#if ENABLED(USE_CONTROLLER_FAN)
  #define CONTROLLERFAN_SPEED_MIN         0 // (0-255) Minimum speed. (If set below this value the fan is turned off.)
  #define CONTROLLERFAN_SPEED_ACTIVE    255 // (0-255) Active speed, used when any motor is enabled
  #define CONTROLLERFAN_SPEED_IDLE        0 // (0-255) Idle speed, used when motors are disabled
  #define CONTROLLERFAN_IDLE_TIME        60 // (seconds) Extra time to keep the fan running after disabling motors

  #define CONTROLLER_FAN_BED_HEATING        // Turn on the fan when heating the bed

  #if ENABLED(CONTROLLER_FAN_EDITABLE)
    #define CONTROLLER_FAN_MENU             // Enable the Controller Fan submenu
  #endif
#endif

#if ENABLED(FAST_PWM_FAN)
  #ifndef FAST_PWM_FAN_FREQUENCY
    #ifdef __AVR__
      #define FAST_PWM_FAN_FREQUENCY ((F_CPU) / (2 * 255 * 1))
    #else
      #define FAST_PWM_FAN_FREQUENCY 1000U
    #endif
  #endif
#endif

/**
 * Extruder cooling fans
 *
 * Extruder auto fans automatically turn on when their extruders'
 * temperatures go above EXTRUDER_AUTO_FAN_TEMPERATURE.
 *
 * Your board's pins file specifies the recommended pins. Override those here
 * or set to -1 to disable completely.
 *
 * Multiple extruders can be assigned to the same pin in which case
 * the fan will turn on when any selected extruder is above the threshold.
 */
#define E0_AUTO_FAN_PIN -1
#define E1_AUTO_FAN_PIN -1
#define E2_AUTO_FAN_PIN -1
#define E3_AUTO_FAN_PIN -1
#define E4_AUTO_FAN_PIN -1
#define E5_AUTO_FAN_PIN -1
#define E6_AUTO_FAN_PIN -1
#define E7_AUTO_FAN_PIN -1
#define CHAMBER_AUTO_FAN_PIN -1
#define COOLER_AUTO_FAN_PIN -1

#define EXTRUDER_AUTO_FAN_TEMPERATURE 50
#define EXTRUDER_AUTO_FAN_SPEED 255   // 255 == full speed
#define CHAMBER_AUTO_FAN_TEMPERATURE 30
#define CHAMBER_AUTO_FAN_SPEED 255
#define COOLER_AUTO_FAN_TEMPERATURE 18
#define COOLER_AUTO_FAN_SPEED 255

/**
 * Part-Cooling Fan Multiplexer
 *
 * This feature allows you to digitally multiplex the fan output.
 * The multiplexer is automatically switched at tool-change.
 * Set FANMUX[012]_PINs below for up to 2, 4, or 8 multiplexed fans.
 */
#define FANMUX0_PIN -1
#define FANMUX1_PIN -1
#define FANMUX2_PIN -1

#if ENABLED(CASE_LIGHT_ENABLE)
  #define INVERT_CASE_LIGHT false             // Set true if Case Light is ON when pin is LOW
  #define CASE_LIGHT_DEFAULT_ON true          // Set default power-up state on
  #define CASE_LIGHT_DEFAULT_BRIGHTNESS 105   // Set default power-up brightness (0-255, requires PWM pin)
  #if ANY(CASE_LIGHT_USE_NEOPIXEL, CASE_LIGHT_USE_RGB_LED)
    #define CASE_LIGHT_DEFAULT_COLOR { 255, 255, 255, 255 } // { Red, Green, Blue, White }
  #endif
#endif

// @section endstops

// @section extras

// @section idex

#if ENABLED(DUAL_X_CARRIAGE)
  #define X1_MIN_POS X_MIN_POS    // Set to X_MIN_POS
  #define X1_MAX_POS X_BED_SIZE   // A max coordinate so the X1 carriage can't hit the parked X2 carriage
  #define X2_MIN_POS    80        // A min coordinate so the X2 carriage can't hit the parked X1 carriage
  #define X2_MAX_POS   353        // The max position of the X2 carriage, typically also the home position
  #define X2_HOME_POS X2_MAX_POS  // Default X2 home position. Set to X2_MAX_POS.

  #define DEFAULT_DUAL_X_CARRIAGE_MODE DXC_AUTO_PARK_MODE

  #define DEFAULT_DUPLICATION_X_OFFSET 100

#endif

// @section multi stepper

/**
 * Multi-Stepper / Multi-Endstop
 *
 * When X2_DRIVER_TYPE is defined, this indicates that the X and X2 motors work in tandem.
 * The following explanations for X also apply to Y and Z multi-stepper setups.
 * Endstop offsets may be changed by 'M666 X<offset> Y<offset> Z<offset>' and stored to EEPROM.
 *
 * - Enable INVERT_X2_VS_X_DIR if the X2 motor requires an opposite DIR signal from X.
 *
 * - Enable X_DUAL_ENDSTOPS if the second motor has its own endstop, with adjustable offset.
 *
 *   - Extra endstops are included in the output of 'M119'.
 *
 *   - Set X_DUAL_ENDSTOP_ADJUSTMENT to the known error in the X2 endstop.
 *     Applied to the X2 motor on 'G28' / 'G28 X'.
 *     Get the offset by homing X and measuring the error.
 *     Also set with 'M666 X<offset>' and stored to EEPROM with 'M500'.
 *
 *   - Define the extra endstop pins here to override defaults. No auto-assignment.
 */
#if HAS_X2_STEPPER && DISABLED(DUAL_X_CARRIAGE)
  #define INVERT_X2_VS_X_DIR          // X2 motor faces opposite direction on MPCNC
  #if ENABLED(X_DUAL_ENDSTOPS)
    #define X2_ENDSTOP_ADJUSTMENT  0  // X2 offset relative to X endstop
  #endif
#endif

#if HAS_Y2_STEPPER
  //#define INVERT_Y2_VS_Y_DIR          // Y2 motor faces opposite direction on MPCNC
  #if ENABLED(Y_DUAL_ENDSTOPS)
    #define Y2_ENDSTOP_ADJUSTMENT  0  // Y2 offset relative to Y endstop
  #endif
#endif

#ifdef Z2_DRIVER_TYPE

  #if ENABLED(Z_MULTI_ENDSTOPS)
    #define Z2_ENDSTOP_ADJUSTMENT 0   // Z2 offset relative to Z endstop
  #endif
  #ifdef Z3_DRIVER_TYPE
    #if ENABLED(Z_MULTI_ENDSTOPS)
      #define Z3_ENDSTOP_ADJUSTMENT 0 // Z3 offset relative to Z endstop
    #endif
  #endif
  #ifdef Z4_DRIVER_TYPE
    #if ENABLED(Z_MULTI_ENDSTOPS)
      #define Z4_ENDSTOP_ADJUSTMENT 0 // Z4 offset relative to Z endstop
    #endif
  #endif
#endif

// @section extruder

// @section homing

#define HOMING_BUMP_MM      { 5, 5, 2 }       // (linear=mm, rotational=°) Backoff from endstops after first bump
#define HOMING_BUMP_DIVISOR { 2, 2, 4 }       // Re-Bump Speed Divisor (Divides the Homing Feedrate)

// @section bltouch

#if ENABLED(BLTOUCH)

  #ifdef BLTOUCH_HS_MODE
    #define BLTOUCH_HS_EXTRA_CLEARANCE    7 // Extra Z Clearance
  #endif

#endif // BLTOUCH

// @section calibrate

#if ENABLED(Z_STEPPER_AUTO_ALIGN)

  /**
   * Orientation for the automatically-calculated probe positions.
   * Override Z stepper align points with 'M422 S<index> X<pos> Y<pos>'
   *
   * 2 Steppers:  (0)     (1)
   *               |       |   2   |
   *               | 1   2 |       |
   *               |       |   1   |
   *
   * 3 Steppers:  (0)     (1)     (2)     (3)
   *               |   3   | 1     | 2   1 |     2 |
   *               |       |     3 |       | 3     |
   *               | 1   2 | 2     |   3   |     1 |
   *
   * 4 Steppers:  (0)     (1)     (2)     (3)
   *               | 4   3 | 1   4 | 2   1 | 3   2 |
   *               |       |       |       |       |
   *               | 1   2 | 2   3 | 3   4 | 4   1 |
   */

  #ifndef Z_STEPPER_ALIGN_STEPPER_XY
    #define Z_STEPPER_ALIGN_AMP 1.0       // Use a value > 1.0 NOTE: This may cause instability!
  #endif

  #define G34_MAX_GRADE              5    // (%) Maximum incline that G34 will handle
  #define Z_STEPPER_ALIGN_ITERATIONS 5    // Number of iterations to apply during alignment
  #define Z_STEPPER_ALIGN_ACC        0.02 // Stop iterating early if the accuracy is better than this

  #define RESTORE_LEVELING_AFTER_G34      // Restore leveling after G34 is done?

  #define HOME_AFTER_G34

#endif // Z_STEPPER_AUTO_ALIGN

#if ENABLED(ASSISTED_TRAMMING)

  #define TRAMMING_POINT_XY { {  20, 20 }, { 180,  20 }, { 180, 180 }, { 20, 180 } }

  #define TRAMMING_POINT_NAME_1 "Front-Left"
  #define TRAMMING_POINT_NAME_2 "Front-Right"
  #define TRAMMING_POINT_NAME_3 "Back-Right"
  #define TRAMMING_POINT_NAME_4 "Back-Left"

  #define RESTORE_LEVELING_AFTER_G35    // Enable to restore leveling setup after operation

  /**
   * Screw Thread. Use one of the following defines:
   *
   *   M3_CW = M3 Clockwise, M3_CCW = M3 Counter-Clockwise
   *   M4_CW = M4 Clockwise, M4_CCW = M4 Counter-Clockwise
   *   M5_CW = M5 Clockwise, M5_CCW = M5 Counter-Clockwise
   *
   * :{'M3_CW':'M3 Clockwise','M3_CCW':'M3 Counter-Clockwise','M4_CW':'M4 Clockwise','M4_CCW':'M4 Counter-Clockwise','M5_CW':'M5 Clockwise','M5_CCW':'M5 Counter-Clockwise'}
   */
  #define TRAMMING_SCREW_THREAD M3_CW

#endif

// @section motion control

#if ENABLED(FT_MOTION)

  #if ENABLED(FTM_DYNAMIC_FREQ)
    #define FTM_DEFAULT_DYNFREQ_MODE dynFreqMode_DISABLED // Default mode of dynamic frequency calculation. (DISABLED, Z_BASED, MASS_BASED)
  #endif

  #define FTM_SHAPER_ZV
  #define FTM_SHAPER_ZVD
  #define FTM_SHAPER_ZVDD
  #define FTM_SHAPER_ZVDDD
  #define FTM_SHAPER_EI
  #define FTM_SHAPER_2HEI
  #define FTM_SHAPER_3HEI
  #define FTM_SHAPER_MZV

  #define FTM_DEFAULT_SHAPER_X      ftMotionShaper_NONE // Default shaper mode on X axis (NONE, ZV, ZVD, ZVDD, ZVDDD, EI, 2HEI, 3HEI, MZV)
  #define FTM_SHAPING_DEFAULT_FREQ_X   37.0f    // (Hz) Default peak frequency used by input shapers
  #define FTM_SHAPING_ZETA_X            0.1f    // Zeta used by input shapers for X axis
  #define FTM_SHAPING_V_TOL_X           0.05f   // Vibration tolerance used by EI input shapers for X axis

  #define FTM_DEFAULT_SHAPER_Y      ftMotionShaper_NONE // Default shaper mode on Y axis
  #define FTM_SHAPING_DEFAULT_FREQ_Y   37.0f    // (Hz) Default peak frequency used by input shapers
  #define FTM_SHAPING_ZETA_Y            0.1f    // Zeta used by input shapers for Y axis
  #define FTM_SHAPING_V_TOL_Y           0.05f   // Vibration tolerance used by EI input shapers for Y axis

  #define FTM_DEFAULT_SHAPER_Z      ftMotionShaper_NONE // Default shaper mode on Z axis
  #define FTM_SHAPING_DEFAULT_FREQ_Z   21.0f    // (Hz) Default peak frequency used by input shapers
  #define FTM_SHAPING_ZETA_Z            0.03f   // Zeta used by input shapers for Z axis
  #define FTM_SHAPING_V_TOL_Z           0.05f   // Vibration tolerance used by EI input shapers for Z axis

  #define FTM_DEFAULT_SHAPER_E      ftMotionShaper_NONE // Default shaper mode on Extruder axis
  #define FTM_SHAPING_DEFAULT_FREQ_E   21.0f    // (Hz) Default peak frequency used by input shapers
  #define FTM_SHAPING_ZETA_E            0.03f   // Zeta used by input shapers for E axis
  #define FTM_SHAPING_V_TOL_E           0.05f   // Vibration tolerance used by EI input shapers for E axis

  #if ENABLED(FTM_SMOOTHING)
    #define FTM_MAX_SMOOTHING_TIME      0.10f   // (s) Maximum smoothing time. Higher values consume more RAM.
    #define FTM_SMOOTHING_TIME_X        0.00f   // (s) Smoothing time for X axis. Zero means disabled.
    #define FTM_SMOOTHING_TIME_Y        0.00f   // (s) Smoothing time for Y axis
    #define FTM_SMOOTHING_TIME_Z        0.00f   // (s) Smoothing time for Z axis
    #define FTM_SMOOTHING_TIME_E        0.02f   // (s) Smoothing time for E axis. Prevents noise/skipping from LA by
  #endif

  #define FTM_POLYS                             // Disable POLY5/6 to save ~3k of Flash. Preserves TRAPEZOIDAL.
  #if ENABLED(FTM_POLYS)
    #define FTM_TRAJECTORY_TYPE TRAPEZOIDAL     // Block acceleration profile (TRAPEZOIDAL, POLY5, POLY6)

    #define FTM_POLY6_ACCELERATION_OVERSHOOT 1.875f // Max acceleration overshoot factor for POLY6 (1.25 to 1.875)
  #endif

  /**
   * Advanced configuration
   */
  #define FTM_BUFFER_SIZE             128   // Window size for trajectory generation, must be a power of 2 (e.g 64, 128, 256, ...)
  #define FTM_FS                     1000   // (Hz) Frequency for trajectory generation.
  #define FTM_MIN_SHAPE_FREQ           20   // (Hz) Minimum shaping frequency, lower consumes more RAM

  /**
   * TMC2208 / TMC2208_STANDALONE drivers require a brief pause after a DIR change
   * to prevent a standstill shutdown when using StealthChop (the standalone default).
   * These options cause FT Motion to delay for > 750µs after a DIR change on a given axis.
   * Disable only if you are certain that this can never happen with your TMC2208s.
   */
  #if AXIS_DRIVER_TYPE_X(TMC2208) || AXIS_DRIVER_TYPE_X(TMC2208_STANDALONE)
    #define FTM_DIR_CHANGE_HOLD_X
  #endif
  #if AXIS_DRIVER_TYPE_Y(TMC2208) || AXIS_DRIVER_TYPE_Y(TMC2208_STANDALONE)
    #define FTM_DIR_CHANGE_HOLD_Y
  #endif
  #if AXIS_DRIVER_TYPE_Z(TMC2208) || AXIS_DRIVER_TYPE_Z(TMC2208_STANDALONE)
    #define FTM_DIR_CHANGE_HOLD_Z
  #endif
  #if HAS_E_DRIVER(TMC2208) || HAS_E_DRIVER(TMC2208_STANDALONE)
    #define FTM_DIR_CHANGE_HOLD_E
  #endif

#endif // FT_MOTION

#if ANY(INPUT_SHAPING_X, INPUT_SHAPING_Y, INPUT_SHAPING_Z)
  #if ENABLED(INPUT_SHAPING_X)
    #define SHAPING_FREQ_X  40.0        // (Hz) The default dominant resonant frequency on the X axis.
    #define SHAPING_ZETA_X   0.15       // Damping ratio of the X axis (range: 0.0 = no damping to 1.0 = critical damping).
  #endif
  #if ENABLED(INPUT_SHAPING_Y)
    #define SHAPING_FREQ_Y  40.0        // (Hz) The default dominant resonant frequency on the Y axis.
    #define SHAPING_ZETA_Y   0.15       // Damping ratio of the Y axis (range: 0.0 = no damping to 1.0 = critical damping).
  #endif
  #if ENABLED(INPUT_SHAPING_Z)
    #define SHAPING_FREQ_Z  40.0        // (Hz) The default dominant resonant frequency on the Z axis.
    #define SHAPING_ZETA_Z   0.15       // Damping ratio of the Z axis (range: 0.0 = no damping to 1.0 = critical damping).
  #endif
#endif

// @section motion

#define AXIS_RELATIVE_MODES { false, false, false }

#define STEP_STATE_X HIGH
#define STEP_STATE_Y HIGH
#define STEP_STATE_Z HIGH
#define STEP_STATE_I HIGH
#define STEP_STATE_J HIGH
#define STEP_STATE_K HIGH
#define STEP_STATE_U HIGH
#define STEP_STATE_V HIGH
#define STEP_STATE_W HIGH
#define STEP_STATE_E HIGH

/**
 * Idle Stepper Shutdown
 * Enable DISABLE_IDLE_* to shut down axis steppers after an idle period.
 * The default timeout duration can be overridden with M18 and M84. Set to 0 for No Timeout.
 */
#define DEFAULT_STEPPER_TIMEOUT_SEC 120
#define DISABLE_IDLE_X
#define DISABLE_IDLE_Y
#define DISABLE_IDLE_Z    // Disable if the nozzle could fall onto your printed part!
#define DISABLE_IDLE_E    // Shut down all idle extruders

#define DEFAULT_MINIMUMFEEDRATE       0.0     // (mm/s) Minimum feedrate. Set with M205 S.
#define DEFAULT_MINTRAVELFEEDRATE     0.0     // (mm/s) Minimum travel feedrate. Set with M205 T.

#define DEFAULT_MINSEGMENTTIME        20000   // (µs) Set with M205 B.

#define SLOWDOWN
#if ENABLED(SLOWDOWN)
  #define SLOWDOWN_DIVISOR 2
#endif

#ifdef XY_FREQUENCY_LIMIT
  #define XY_FREQUENCY_MIN_PERCENT 5 // (%) Minimum FR percentage to apply. Set with M201 S<min%>.
#endif

#if ENABLED(BACKLASH_COMPENSATION)
  #define BACKLASH_DISTANCE_MM { 0, 0, 0 } // (linear=mm, rotational=°) One value for each linear axis
  #define BACKLASH_CORRECTION    0.0       // 0.0 = no correction; 1.0 = full correction

  #if ENABLED(BACKLASH_GCODE)
    #define MEASURE_BACKLASH_WHEN_PROBING

    #if ENABLED(MEASURE_BACKLASH_WHEN_PROBING)
      #define BACKLASH_MEASUREMENT_LIMIT       0.5   // (mm)
      #define BACKLASH_MEASUREMENT_RESOLUTION  0.005 // (mm)
      #define BACKLASH_MEASUREMENT_FEEDRATE    Z_PROBE_FEEDRATE_SLOW // (mm/min)
    #endif
  #endif
#endif

#if ENABLED(CALIBRATION_GCODE)

  #define CALIBRATION_FEEDRATE_SLOW             60    // (mm/min)
  #define CALIBRATION_FEEDRATE_FAST           1200    // (mm/min)
  #define CALIBRATION_FEEDRATE_TRAVEL         3000    // (mm/min)

  #define CALIBRATION_NOZZLE_TIP_HEIGHT          1.0  // (mm)
  #define CALIBRATION_NOZZLE_OUTER_DIAMETER      2.0  // (mm)

  #define CALIBRATION_OBJECT_CENTER     { 264.0, -22.0,  -2.0 } // (mm)
  #define CALIBRATION_OBJECT_DIMENSIONS {  10.0,  10.0,  10.0 } // (mm)

  #define CALIBRATION_MEASURE_RIGHT
  #define CALIBRATION_MEASURE_FRONT
  #define CALIBRATION_MEASURE_LEFT
  #define CALIBRATION_MEASURE_BACK

  #ifndef CALIBRATION_PIN
    #define CALIBRATION_PIN_INVERTING false // Set to true to invert the custom pin
    #define CALIBRATION_PIN_PULLUP
  #endif
#endif

/**
 * Multi-stepping sends steps in bursts to reduce MCU usage for high step-rates.
 * This allows higher feedrates than the MCU could otherwise support.
 */
#define MULTISTEPPING_LIMIT   16  // :[1, 2, 4, 8, 16, 32, 64, 128]

#define MICROSTEP_MODES { 16, 16, 16, 16, 16, 16 } // [1,2,4,8,16]

#if ANY(DIGIPOT_MCP4018, DIGIPOT_MCP4451)
  #define DIGIPOT_I2C_NUM_CHANNELS 5  // 5DPRINT:4   AZTEEG_X3_PRO:8   MKS_SBASE:5   MIGHTYBOARD_REVE:5

  #define DIGIPOT_I2C_MOTOR_CURRENTS { 1.0, 1.0, 1.6, 1.0, 1.0 } // AZTEEG_X3_PRO

#endif

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

// @section lcd

#if HAS_MANUAL_MOVE_MENU
  #define MANUAL_FEEDRATE { 50*60, 50*60, 4*60, 2*60 } // (mm/min) Feedrates for manual moves along X, Y, Z, E from panel
  #define FINE_MANUAL_MOVE 0.025    // (mm) Smallest manual move (< 0.1mm) applying to Z on most machines
  #if IS_ULTIPANEL
    #define MANUAL_E_MOVES_RELATIVE // Display extruder move distance rather than "position"
    #define ULTIPANEL_FEEDMULTIPLY  // Encoder sets the feedrate multiplier on the Status Screen
  #endif
#endif

#define ENCODER_RATE_MULTIPLIER
#if ENABLED(ENCODER_RATE_MULTIPLIER)
  #define ENCODER_10X_STEPS_PER_SEC   30  // (steps/s) Encoder rate for 10x speed
  #define ENCODER_100X_STEPS_PER_SEC  80  // (steps/s) Encoder rate for 100x speed
#endif

#if ENABLED(BEEP_ON_FEEDRATE_CHANGE)
  #define FEEDRATE_CHANGE_BEEP_DURATION   10
  #define FEEDRATE_CHANGE_BEEP_FREQUENCY 440
#endif

/**
 * Probe Offset Wizard
 * Add a Probe Z Offset calibration option to the LCD menu.
 * Use this helper to get a perfect 'M851 Z' probe offset.
 * When launched this powerful wizard:
 *  - Measures the bed height at the configured position with the probe.
 *  - Moves the nozzle to the same position for a "paper" measurement.
 *  - The difference is used to set the probe Z offset.
 */

#if HAS_MARLINUI_MENU

  #if HAS_BED_PROBE

    #define PROBE_DEPLOY_STOW_MENU

    #if ENABLED(X_AXIS_TWIST_COMPENSATION)
      /**
       * Enable to init the Probe Z-Offset when starting the Wizard.
       * Use a height slightly above the estimated nozzle-to-probe Z offset.
       * For example, with an offset of -5, consider a starting height of -4.
       */
      #define XATC_START_Z 0.0
      #define XATC_MAX_POINTS 3             // Number of points to probe in the wizard
      #define XATC_Y_POSITION Y_CENTER      // (mm) Y position to probe
      #define XATC_Z_OFFSETS { 0, 0, 0 }    // Z offsets for X axis sample points
    #endif

  #endif

  /**
   * MarlinUI "Move Axis" menu distances. Comma-separated list.
   * Values are displayed as-defined, so always use plain numbers here.
   * Axis moves <= 1/2 the axis length and Extruder moves <= EXTRUDE_MAXLENGTH
   * will be shown in the move submenus.
   */

  #define MANUAL_MOVE_DISTANCE_MM                    10, 1.0, 0.1  // (mm)

  #define MANUAL_MOVE_DISTANCE_IN                          0.100, 0.010, 0.001  // (in)

  #define MANUAL_MOVE_DISTANCE_DEG             90, 45, 22.5, 5, 1  // (°)

#endif // HAS_MARLINUI_MENU

#if HAS_DISPLAY
  /**
   * *** VENDORS PLEASE READ ***
   *
   * Marlin allows you to add a custom boot image for Graphical LCDs.
   * With this option Marlin will first show your custom screen followed
   * by the standard Marlin logo with version number and web URL.
   *
   * We encourage you to take advantage of this new feature and we also
   * respectfully request that you retain the unmodified Marlin boot screen.
   */
  #define SHOW_BOOTSCREEN                 // Show the Marlin bootscreen on startup. ** ENABLE FOR PRODUCTION **
  #if ENABLED(SHOW_BOOTSCREEN)
    #define BOOTSCREEN_TIMEOUT 3000       // (ms) Total Duration to display the boot screen(s)
    #if ANY(HAS_MARLINUI_U8GLIB, TFT_COLOR_UI)
      #define BOOT_MARLIN_LOGO_SMALL      // Show a smaller Marlin logo on the Boot Screen (saving lots of flash)
    #endif
  #endif

  #define SOUND_ON_DEFAULT    // Buzzer/speaker default enabled state

  #if ENABLED(LED_CONTROL_MENU)
    #define LED_COLOR_PRESETS                 // Enable the Preset Color menu option
    #if ENABLED(LED_COLOR_PRESETS)
      #define LED_USER_PRESET_RED        255  // User defined RED value
      #define LED_USER_PRESET_GREEN      128  // User defined GREEN value
      #define LED_USER_PRESET_BLUE         0  // User defined BLUE value
      #define LED_USER_PRESET_WHITE      255  // User defined WHITE value
      #define LED_USER_PRESET_BRIGHTNESS 255  // User defined intensity
    #endif
    #if ENABLED(NEO2_COLOR_PRESETS)
      #define NEO2_USER_PRESET_RED        255 // User defined RED value
      #define NEO2_USER_PRESET_GREEN      128 // User defined GREEN value
      #define NEO2_USER_PRESET_BLUE         0 // User defined BLUE value
      #define NEO2_USER_PRESET_WHITE      255 // User defined WHITE value
      #define NEO2_USER_PRESET_BRIGHTNESS 255 // User defined intensity
    #endif
  #endif

#endif // HAS_DISPLAY

#if ANY(HAS_MARLINUI_MENU, DWIN_CREALITY_LCD, DWIN_LCD_PROUI, MALYAN_LCD, TOUCH_SCREEN, ULTIPANEL_FEEDMULTIPLY)
  #define SPEED_EDIT_MIN    10  // (%) Feedrate percentage edit range minimum
  #define SPEED_EDIT_MAX   999  // (%) Feedrate percentage edit range maximum
#endif
#if ANY(HAS_MARLINUI_MENU, DWIN_CREALITY_LCD, DWIN_LCD_PROUI, MALYAN_LCD, TOUCH_SCREEN)
  #define FLOW_EDIT_MIN     10  // (%) Flow percentage edit range minimum
  #define FLOW_EDIT_MAX    999  // (%) Flow percentage edit range maximum
#endif

#if ENABLED(SET_PROGRESS_MANUALLY)
  #define SET_PROGRESS_PERCENT            // Add 'P' parameter to set percentage done
  #define SET_REMAINING_TIME              // Add 'R' parameter to set remaining time
  #if ALL(M73_REPORT, HAS_MEDIA)
    #define M73_REPORT_SD_ONLY            // Report only when printing from SD
  #endif
#endif

#if HAS_DISPLAY && ANY(HAS_MEDIA, SET_PROGRESS_MANUALLY)
  #define SHOW_PROGRESS_PERCENT           // Show print progress percentage (doesn't affect progress bar)
  #define SHOW_ELAPSED_TIME               // Display elapsed printing time (prefix 'E')
  #if ENABLED(SET_INTERACTION_TIME)
    #define SHOW_INTERACTION_TIME         // Display time until next user interaction ('C' = filament change)
  #endif

  #if ANY(HAS_MARLINUI_HD44780, IS_TFTGLCD_PANEL)
    #if ENABLED(LCD_PROGRESS_BAR)
      #define PROGRESS_BAR_BAR_TIME 2000  // (ms) Amount of time to show the bar
      #define PROGRESS_BAR_MSG_TIME 3000  // (ms) Amount of time to show the status message
      #define PROGRESS_MSG_EXPIRE      0  // (ms) Amount of time to retain the status message (0=forever)
    #endif
  #endif
#endif

#if HAS_MEDIA

  /**
   * Reinit the LCD after SD Card insert/remove or when entering the menu.
   * Required for some LCDs that use shared SPI with an external SD Card reader.
   */
  #define REINIT_NOISY_LCD

  #define SD_PROCEDURE_DEPTH 1              // Increase if you need more nested M32 calls

  #define SD_FINISHED_STEPPERRELEASE true   // Disable steppers when SD Print is finished
  #define SD_FINISHED_RELEASECOMMAND "M84"  // Use "M84XYE" to keep Z enabled so your bed stays in place

  #define SDCARD_RATHERRECENTFIRST

  #define SD_MENU_CONFIRM_START             // Confirm the selected SD file before printing

  #define EVENT_GCODE_SD_ABORT "G28XY"      // G-code to run on SD Abort Print (e.g., "G28XY" or "G27")

  #if ENABLED(PRINTER_EVENT_LEDS)
    #define PE_LEDS_COMPLETED_TIME  (30*60) // (seconds) Time to keep the LED "done" color before restoring normal illumination
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    #define PLR_ENABLED_DEFAULT       false // Power-Loss Recovery enabled by default. (Set with 'M413 Sn' & M500)

    #define POWER_LOSS_MIN_Z_CHANGE    0.05 // (mm) Minimum Z change before saving power-loss data

  #endif

  #if ENABLED(SDCARD_SORT_ALPHA)
    #define SDSORT_QUICK           true   // Use Quick Sort as a sorting algorithm. Otherwise use Bubble Sort.
    #define SDSORT_REVERSE         false  // Default to sorting file names in reverse order.
    #define SDSORT_LIMIT           40     // Maximum number of sorted items (10-256). Costs 27 bytes each.
    #define SDSORT_FOLDERS        -1      // -1=above  0=none  1=below
    #define SDSORT_GCODE           false  // Enable G-code M34 to set sorting behaviors: M34 S<-1|0|1> F<-1|0|1>
    #define SDSORT_USES_STACK      false  // Prefer the stack for pre-sorting to give back some SRAM. (Negated by next 2 options.)
    #define SDSORT_USES_RAM        false  // Pre-allocate a static array for faster pre-sorting.
    #if ENABLED(SDSORT_USES_RAM)
      #define SDSORT_CACHE_NAMES   false  // Keep sorted items in RAM longer for speedy performance. Most expensive option.
      #if ENABLED(SDSORT_CACHE_NAMES)
        #define SDSORT_DYNAMIC_RAM false  // Use dynamic allocation (within SD menus). Least expensive option. Set SDSORT_LIMIT before use!
        #define SDSORT_CACHE_VFATS 2      // Maximum number of 13-byte VFAT entries to use for sorting.
      #endif
    #endif
  #endif

  #if ENABLED(USB_FLASH_DRIVE_SUPPORT)

    #define DISABLE_DUE_SD_MMC // Disable USB Host access to USB Drive to prevent hangs on block access for DUE platform

    #if DISABLED(USE_OTG_USB_HOST)
      #define USB_CS_PIN    SD_SS_PIN
      #define USB_INTR_PIN  SD_DETECT_PIN
    #endif
  #endif

  #if ENABLED(SD_FIRMWARE_UPDATE)
    #define SD_FIRMWARE_UPDATE_EEPROM_ADDR    0x1FF
    #define SD_FIRMWARE_UPDATE_ACTIVE_VALUE   0xF0
    #define SD_FIRMWARE_UPDATE_INACTIVE_VALUE 0xFF
  #endif

  #if ENABLED(MULTI_VOLUME)
    #define VOLUME_SD_ONBOARD
    #define VOLUME_USB_FLASH_DRIVE
    #define DEFAULT_VOLUME        SD_ONBOARD       // :[ 'SD_ONBOARD', 'USB_FLASH_DRIVE' ]
    #define DEFAULT_SHARED_VOLUME USB_FLASH_DRIVE  // :[ 'SD_ONBOARD', 'USB_FLASH_DRIVE' ]
  #endif

#endif // HAS_MEDIA

/**
 * Additional options for Graphical Displays
 *
 * Use the optimizations here to improve printing performance,
 * which can be adversely affected by graphical display drawing,
 * especially when doing several short moves, and when printing
 * on DELTA and SCARA machines.
 *
 * Some of these options may result in the display lagging behind
 * controller events, as there is a trade-off between reliable
 * printing performance versus fast display updates.
 */
#if HAS_MARLINUI_U8GLIB
  #define XYZ_HOLLOW_FRAME

  /**
   * ST7920-based LCDs can emulate a 16 x 4 character display using
   * the ST7920 character-generator for very fast screen updates.
   * Enable LIGHTWEIGHT_UI to use this special display mode.
   *
   * Since LIGHTWEIGHT_UI has limited space, the position and status
   * message occupy the same line. Set STATUS_EXPIRE_SECONDS to the
   * length of time to display the status message before clearing.
   *
   * Set STATUS_EXPIRE_SECONDS to zero to never clear the status.
   * This will prevent position updates from being displayed.
   */
  #if IS_U8GLIB_ST7920

    #if ENABLED(LIGHTWEIGHT_UI)
      #define STATUS_EXPIRE_SECONDS 20
    #endif
  #endif

  #define STATUS_HOTEND_INVERTED      // Show solid nozzle bitmaps when heating (Requires STATUS_HOTEND_ANIM for numbered hotends)
  #define STATUS_HOTEND_ANIM          // Use a second bitmap to indicate hotend heating
  #define STATUS_BED_ANIM             // Use a second bitmap to indicate bed heating
  #define STATUS_CHAMBER_ANIM         // Use a second bitmap to indicate chamber heating

#endif // HAS_MARLINUI_U8GLIB

#if HAS_MARLINUI_U8GLIB || IS_DWIN_MARLINUI
  #define MENU_HOLLOW_FRAME           // Enable to save many cycles by drawing a hollow frame on Menu Screens

#endif

#if HAS_DGUS_LCD
  #define LCD_BAUDRATE 115200

  #define DGUS_RX_BUFFER_SIZE 128
  #define DGUS_TX_BUFFER_SIZE 48

  #define DGUS_UPDATE_INTERVAL_MS  500    // (ms) Interval between automatic screen updates

  #if DGUS_UI_IS(FYSETC, MKS, HIPRECY)
    #define DGUS_PRINT_FILENAME           // Display the filename during printing
    #define DGUS_PREHEAT_UI               // Display a preheat screen during heatup

    #if DGUS_UI_IS(FYSETC, MKS)
    #else
      #define DGUS_UI_MOVE_DIS_OPTION     // Enabled by default for UI_HIPRECY
    #endif

    #define DGUS_FILAMENT_LOADUNLOAD
    #if ENABLED(DGUS_FILAMENT_LOADUNLOAD)
      #define DGUS_FILAMENT_PURGE_LENGTH 10
      #define DGUS_FILAMENT_LOAD_LENGTH_PER_TIME 0.5 // (mm) Adjust in proportion to DGUS_UPDATE_INTERVAL_MS
    #endif

    #define DGUS_UI_WAITING               // Show a "waiting" screen between some screens
    #if ENABLED(DGUS_UI_WAITING)
      #define DGUS_UI_WAITING_STATUS 10
      #define DGUS_UI_WAITING_STATUS_PERIOD 8 // Increase to slower waiting status looping
    #endif

  #elif DGUS_UI_IS(E3S1PRO)
    /**
     * The stock Ender-3 S1 Pro/Plus display firmware has rather poor SD file handling.
     *
     * The autoscroll is mainly useful for status messages, filenames, and the "About" page.
     *
     * NOTE: The Advanced SD Card option is affected by the stock touchscreen firmware, so
     *       pages 5 and up will display "4/4". This may get fixed in a screen firmware update.
     */
    #define DGUS_SOFTWARE_AUTOSCROLL        // Enable long text software auto-scroll
    #define DGUS_AUTOSCROLL_START_CYCLES 1  // Refresh cycles without scrolling at the beginning of text strings
    #define DGUS_AUTOSCROLL_END_CYCLES 1    // ... at the end of text strings

    #define DGUS_ADVANCED_SDCARD            // Allow more than 20 files and navigating directories
    #define DGUS_USERCONFIRM                // Reuse the SD Card page to show various messages
  #endif
#endif // HAS_DGUS_LCD

#if ENABLED(ANYCUBIC_LCD_CHIRON)

  /**
   * Display Folders
   * By default the file browser lists all G-code files (including those in subfolders) in a flat list.
   * Enable this option to display a hierarchical file browser.
   *
   * NOTES:
   * - Without this option it helps to enable SDCARD_SORT_ALPHA so files are sorted before/after folders.
   * - When used with the "new" panel, folder names will also have '.gcode' appended to their names.
   *   This hack is currently required to force the panel to show folders.
   */
  #define AC_SD_FOLDER_VIEW
#endif

#if ENABLED(TOUCH_UI_FTDI_EVE)

  #if ENABLED(OTHER_PIN_LAYOUT)
    #define CLCD_MOD_RESET  9
    #define CLCD_SPI_CS    10

    #if ENABLED(CLCD_USE_SOFT_SPI)
      #define CLCD_SOFT_SPI_MOSI 11
      #define CLCD_SOFT_SPI_MISO 12
      #define CLCD_SOFT_SPI_SCLK 13
    #endif
  #endif

  #if ENABLED(TOUCH_UI_USE_UTF8)
    #define TOUCH_UI_UTF8_WESTERN_CHARSET

  #endif

  #define TOUCH_UI_FIT_TEXT

#endif // TOUCH_UI_FTDI_EVE

#if defined(DISPLAY_SLEEP_MINUTES) || defined(LCD_BACKLIGHT_TIMEOUT_MINS)
  #define EDITABLE_DISPLAY_TIMEOUT      // Edit sleep / backlight timeout with M255 S<minutes> and a menu item
#endif

#if HAS_ADC_BUTTONS
  #define ADC_BUTTON_DEBOUNCE_DELAY 16  // (count) Increase if buttons bounce or repeat too fast
#endif

// @section safety

/**
 * The watchdog hardware timer will do a reset and disable all outputs
 * if the firmware gets too overloaded to read the temperature sensors.
 *
 * If you find that watchdog reboot causes your AVR board to hang forever,
 * enable WATCHDOG_RESET_MANUAL to use a custom timer instead of WDTO.
 * NOTE: This method is less reliable as it can only catch hangups while
 * interrupts are enabled.
 */
#define USE_WATCHDOG

// @section baby-stepping

#if ENABLED(BABYSTEPPING)
  #define BABYSTEP_MULTIPLICATOR_Z  1       // (steps or mm) Steps or millimeter distance for each Z babystep
  #define BABYSTEP_MULTIPLICATOR_XY 1       // (steps or mm) Steps or millimeter distance for each XY babystep

  #if ENABLED(DOUBLECLICK_FOR_Z_BABYSTEPPING)
    #define DOUBLECLICK_MAX_INTERVAL 1250   // (ms) Maximum interval between clicks.
    #if ENABLED(MOVE_Z_WHEN_IDLE)
      #define MOVE_Z_IDLE_MULTIPLICATOR 1   // Multiply 1mm by this factor for the move step size.
    #endif
  #endif

#endif

// @section extruder

#if ANY(LIN_ADVANCE, FT_MOTION)
  #if ENABLED(DISTINCT_E_FACTORS)
    #define ADVANCE_K { 0.22 }    // (mm) Compression length per 1mm/s extruder speed, per extruder. Override with 'M900 T<tool> K<mm>'.
  #else
    #define ADVANCE_K 0.22        // (mm) Compression length for all extruders. Override with 'M900 K<mm>'.
  #endif
#endif

#if ENABLED(LIN_ADVANCE)

  #if ENABLED(SMOOTH_LIN_ADVANCE)
    /**
     * ADVANCE_TAU is also the time ahead that the smoother needs to look
     * into the planner, so the planner needs to have enough blocks loaded.
     * For k=0.04 at 10k acceleration and an "Orbiter 2" extruder it can be as low as 0.0075.
     * Adjust by lowering the value until you observe the extruder skipping, then raise slightly.
     * Higher k and higher XY acceleration may require larger ADVANCE_TAU to avoid skipping steps.
     */
    #if ENABLED(DISTINCT_E_FACTORS)
      #define ADVANCE_TAU { 0.02 }   // (s) Smoothing time to reduce extruder acceleration, per extruder
    #else
      #define ADVANCE_TAU 0.02       // (s) Smoothing time to reduce extruder acceleration
    #endif
    #define SMOOTH_LIN_ADV_HZ 1000   // (Hz) How often to update extruder speed
    #define INPUT_SHAPING_E_SYNC     // Synchronize the extruder-shaped XY axes (to increase precision)
  #endif
#endif

// @section leveling

/**
 * Points to probe for all 3-point Leveling procedures.
 * Override if the automatically selected points are inadequate.
 */

/**
 * Probing Margins
 *
 * Override PROBING_MARGIN for each side of the build plate
 * Useful to get probe points to exact positions on targets or
 * to allow leveling to avoid plate clamps on only specific
 * sides of the bed. With NOZZLE_AS_PROBE negative values are
 * allowed, to permit probing outside the bed.
 *
 * If you are replacing the prior *_PROBE_BED_POSITION options,
 * LEFT and FRONT values in most cases will map directly over
 * RIGHT and REAR would be the inverse such as
 * (X/Y_BED_SIZE - RIGHT/BACK_PROBE_BED_POSITION)
 *
 * This will allow all positions to match at compilation, however
 * should the probe position be modified with M851XY then the
 * probe points will follow. This prevents any change from causing
 * the probe to be unable to reach any points.
 */

#if ENABLED(G29_RETRY_AND_RECOVER)
  #define G29_MAX_RETRIES 3
  #define G29_HALT_ON_FAILURE
  /**
   * Specify the GCODE commands that will be executed when leveling succeeds,
   * between attempts, and after the maximum number of retries have been tried.
   */
  #define G29_SUCCESS_COMMANDS "M117 Bed leveling done."
  #define G29_RECOVER_COMMANDS "M117 Probe failed. Rewiping.\nG28\nG12 P0 S12 T0"
  #define G29_FAILURE_COMMANDS "M117 Bed leveling failed.\nG0 Z10\nM300 P25 S880\nM300 P50 S0\nM300 P25 S880\nM300 P50 S0\nM300 P25 S880\nM300 P50 S0\nG4 S1"

#endif

// @section probes

#if ANY(PTC_PROBE, PTC_BED, PTC_HOTEND)

  #if ENABLED(PTC_PROBE)
    #define PTC_PROBE_START   30    // (°C)
    #define PTC_PROBE_RES      5    // (°C)
    #define PTC_PROBE_COUNT   10
    #define PTC_PROBE_ZOFFS   { 0 } // (µm) Z adjustments per sample
  #endif

  #if ENABLED(PTC_BED)
    #define PTC_BED_START     60    // (°C)
    #define PTC_BED_RES        5    // (°C)
    #define PTC_BED_COUNT     10
    #define PTC_BED_ZOFFS     { 0 } // (µm) Z adjustments per sample
  #endif

  #if ENABLED(PTC_HOTEND)
    #define PTC_HOTEND_START 180    // (°C)
    #define PTC_HOTEND_RES     5    // (°C)
    #define PTC_HOTEND_COUNT  20
    #define PTC_HOTEND_ZOFFS  { 0 } // (µm) Z adjustments per sample
  #endif

  #if ALL(PTC_PROBE, PTC_BED)
    #define PTC_PARK_POS   { 0, 0, 100 }

    #define PTC_PROBE_POS  { 90, 100 }

    #define PTC_PROBE_TEMP    30  // (°C)

    #define PTC_PROBE_HEATING_OFFSET 0.5  // (mm)
  #endif
#endif // PTC_PROBE || PTC_BED || PTC_HOTEND

// @section gcode

// @section motion

#define ARC_SUPPORT                   // Requires ~3226 bytes
#if ENABLED(ARC_SUPPORT)
  #define MIN_ARC_SEGMENT_MM      0.1 // (mm) Minimum length of each arc segment
  #define MAX_ARC_SEGMENT_MM      1.0 // (mm) Maximum length of each arc segment
  #define MIN_CIRCLE_SEGMENTS    72   // Minimum number of segments in a complete circle
  #define N_ARC_CORRECTION       25   // Number of interpolated segments between corrections
#endif

#if ANY(ARC_SUPPORT, BEZIER_CURVE_SUPPORT)
  #define CNC_WORKSPACE_PLANES        // Allow G2/G3/G5 to operate in XY, ZX, or YZ planes
#endif

// @section calibrate

#if ENABLED(G38_PROBE_TARGET)
  #define G38_MINIMUM_MOVE 0.0275 // (mm) Minimum distance that will produce a move.
#endif

// @section motion

#define MIN_STEPS_PER_SEGMENT 6

// @section temperature

//===========================================================================
//================================= Buffers =================================
//===========================================================================

// @section gcode

#if ALL(HAS_MEDIA, DIRECT_STEPPING)
  #define BLOCK_BUFFER_SIZE  8
#elif HAS_MEDIA
  #define BLOCK_BUFFER_SIZE 16
#else
  #define BLOCK_BUFFER_SIZE 16
#endif

// @section serial

#define MAX_CMD_SIZE 96
#define BUFSIZE 4

/**
 * Host Transmit Buffer Size
 *  - Costs 386 bytes of flash and TX_BUFFER_SIZE+3 bytes of SRAM (if not 0).
 *  - 4 bytes required to buffer a simple "ok".
 *  - 32 bytes for ADVANCED_OK (M105).
 *  - 128 bytes for the optimal speed of 'debug-echo:'
 *  - Other output doesn't need to be that speedy.
 * :[0, 2, 4, 8, 16, 32, 64, 128, 256]
 */
#define TX_BUFFER_SIZE 0

#define SERIAL_OVERRUN_PROTECTION

/**
 * Set the number of proportional font spaces required to fill up a typical character space.
 * This can help to better align the output of commands like 'G29 O' Mesh Output.
 *
 * For clients that use a fixed-width font (like OctoPrint), leave this set to 1.0.
 * Otherwise, adjust according to your client and font.
 */
#define PROPORTIONAL_FONT_RATIO 1.0

// @section extras

// @section firmware retraction

#if ENABLED(FWRETRACT)
  #define FWRETRACT_AUTORETRACT             // Override slicer retractions
  #if ENABLED(FWRETRACT_AUTORETRACT)
    #define MIN_AUTORETRACT             0.1 // (mm) Don't convert E moves under this length
    #define MAX_AUTORETRACT            10.0 // (mm) Don't convert E moves over this length
  #endif
  #define RETRACT_LENGTH                3   // (mm) Default retract length (positive value)
  #define RETRACT_LENGTH_SWAP          13   // (mm) Default swap retract length (positive value)
  #define RETRACT_FEEDRATE             45   // (mm/s) Default feedrate for retracting
  #define RETRACT_ZRAISE                0   // (mm) Default retract Z-raise
  #define RETRACT_RECOVER_LENGTH        0   // (mm) Default additional recover length (added to retract length on recover)
  #define RETRACT_RECOVER_LENGTH_SWAP   0   // (mm) Default additional swap recover length (added to retract length on recover from toolchange)
  #define RETRACT_RECOVER_FEEDRATE      8   // (mm/s) Default feedrate for recovering from retraction
  #define RETRACT_RECOVER_FEEDRATE_SWAP 8   // (mm/s) Default feedrate for recovering from swap retraction
#endif

// @section tool change

/**
 * Universal tool change settings.
 * Applies to all types of extruders except where explicitly noted.
 */
#if HAS_MULTI_EXTRUDER
  #define TOOLCHANGE_ZRAISE                 2 // (mm)

  #if ENABLED(TOOLCHANGE_FILAMENT_SWAP)
    #define TOOLCHANGE_FS_LENGTH              12  // (mm) Load / Unload length
    #define TOOLCHANGE_FS_EXTRA_RESUME_LENGTH  0  // (mm) Extra length for better restart. Adjust with LCD or M217 B.
    #define TOOLCHANGE_FS_RETRACT_SPEED   (50*60) // (mm/min) (Unloading)
    #define TOOLCHANGE_FS_UNRETRACT_SPEED (25*60) // (mm/min) (On SINGLENOZZLE or Bowden loading must be slowed down)

    #define TOOLCHANGE_FS_EXTRA_PRIME          0  // (mm) Extra priming length
    #define TOOLCHANGE_FS_PRIME_SPEED    (4.6*60) // (mm/min) Extra priming feedrate
    #define TOOLCHANGE_FS_WIPE_RETRACT         0  // (mm) Cutting retraction out of park, for less stringing, better wipe, etc. Adjust with LCD or M217 G.

    #define TOOLCHANGE_FS_FAN                 -1  // Fan index or -1 to skip
    #define TOOLCHANGE_FS_FAN_SPEED          255  // 0-255
    #define TOOLCHANGE_FS_FAN_TIME            10  // (seconds)

    /**
     * Tool Change Migration
     * This feature provides G-code and LCD options to switch tools mid-print.
     * All applicable tool properties are migrated so the print can continue.
     * Tools must be closely matching and other restrictions may apply.
     * Useful to:
     *   - Change filament color without interruption
     *   - Switch spools automatically on filament runout
     *   - Switch to a different nozzle on an extruder jam
     */
    #define TOOLCHANGE_MIGRATION_FEATURE
  #endif

  #if ENABLED(TOOLCHANGE_PARK)
    #define TOOLCHANGE_PARK_XY    { X_MIN_POS + 10, Y_MIN_POS + 10 }
    #define TOOLCHANGE_PARK_XY_FEEDRATE 6000  // (mm/min)
  #endif
#endif // HAS_MULTI_EXTRUDER

// @section advanced pause

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #define PAUSE_PARK_RETRACT_FEEDRATE         60  // (mm/s) Initial retract feedrate.
  #define PAUSE_PARK_RETRACT_LENGTH            2  // (mm) Initial retract.
  #define FILAMENT_CHANGE_UNLOAD_FEEDRATE     10  // (mm/s) Unload filament feedrate. This can be pretty fast.
  #define FILAMENT_CHANGE_UNLOAD_ACCEL        25  // (mm/s^2) Lower acceleration may allow a faster feedrate.
  #define FILAMENT_CHANGE_UNLOAD_LENGTH      100  // (mm) The length of filament for a complete unload.
  #define FILAMENT_CHANGE_SLOW_LOAD_FEEDRATE   6  // (mm/s) Slow move when starting load.
  #define FILAMENT_CHANGE_SLOW_LOAD_LENGTH     0  // (mm) Slow length, to allow time to insert material.
  #define FILAMENT_CHANGE_FAST_LOAD_FEEDRATE   6  // (mm/s) Load filament feedrate. This can be pretty fast.
  #define FILAMENT_CHANGE_FAST_LOAD_ACCEL     25  // (mm/s^2) Lower acceleration may allow a faster feedrate.
  #define FILAMENT_CHANGE_FAST_LOAD_LENGTH     0  // (mm) Load length of filament, from extruder gear to nozzle.
  #define ADVANCED_PAUSE_PURGE_FEEDRATE        3  // (mm/s) Extrude feedrate (after loading). Should be slower than load feedrate.
  #define ADVANCED_PAUSE_PURGE_LENGTH         50  // (mm) Length to extrude after loading.
  #define ADVANCED_PAUSE_RESUME_PRIME          0  // (mm) Extra distance to prime nozzle after returning from park.

  #define FILAMENT_UNLOAD_PURGE_RETRACT       13  // (mm) Unload initial retract length.
  #define FILAMENT_UNLOAD_PURGE_DELAY       5000  // (ms) Delay for the filament to cool after retract.
  #define FILAMENT_UNLOAD_PURGE_LENGTH         8  // (mm) An unretract is done, then this length is purged.
  #define FILAMENT_UNLOAD_PURGE_FEEDRATE      25  // (mm/s) feedrate to purge before unload

  #define PAUSE_PARK_NOZZLE_TIMEOUT           45  // (seconds) Time limit before the nozzle is turned off for safety.
  #define FILAMENT_CHANGE_ALERT_BEEPS         10  // Number of alert beeps to play when a response is needed.
  #define PAUSE_PARK_NO_STEPPER_TIMEOUT           // Enable for XYZ steppers to stay powered on during filament change.

  #define CONFIGURE_FILAMENT_CHANGE               // Add M603 G-code and menu items. Requires ~1.3K bytes of flash.
#endif

// @section tmc_smart

/**
 * Trinamic Smart Drivers
 *
 * To use TMC2130, TMC2160, TMC2240, TMC2660, TMC5130, TMC5160 stepper drivers in SPI mode:
 *  - Connect your SPI pins to the Hardware SPI interface on the board.
 *    Some boards have simple jumper connections! See your board's documentation.
 *  - Define the required Stepper CS pins in your `pins_MYBOARD.h` file.
 *    (See the RAMPS pins, for example.)
 *  - You can also use Software SPI with GPIO pins instead of Hardware SPI.
 *
 * To use TMC220x stepper drivers with Serial UART:
 *  - Connect PDN_UART to the #_SERIAL_TX_PIN through a 1K resistor.
 *    For reading capabilities also connect PDN_UART to #_SERIAL_RX_PIN with no resistor.
 *    Some boards have simple jumper connections! See your board's documentation.
 *  - These drivers can also be used with Hardware Serial.
 *
 * The TMCStepper library is required for other TMC stepper drivers.
 *   https://github.com/teemuatlut/TMCStepper
 *
 * @section tmc/config
 */
#if HAS_TRINAMIC_CONFIG

  #define HOLD_MULTIPLIER    0.5  // Scales down the holding current from run current

  /**
   * Interpolate microsteps to 256
   * Override for each driver with <driver>_INTERPOLATE settings below
   */
  #define INTERPOLATE      true

  #if HAS_DRIVER(TMC2240)
    #define TMC2240_RREF        12000   // (Ω) 12000 .. 60000. (FLY TMC2240 = 12300)
    #define TMC2240_CURRENT_RANGE   1   // :{ 0:'RMS=690mA PEAK=1A', 1:'RMS=1410mA PEAK=2A', 2:'RMS=2120mA PEAK=3A', 3:'RMS=2110mA PEAK=3A' }
    #define TMC2240_SLOPE_CONTROL   0   // :{ 0:'100V/µs', 1:'200V/µs', 2:'400V/µs', 3:'800V/µs' }
  #endif

  #if AXIS_IS_TMC_CONFIG(X)
    #define X_CURRENT       800        // (mA) RMS current. Multiply by 1.414 for peak current.
    #define X_CURRENT_HOME  X_CURRENT  // (mA) RMS current for homing. (Typically lower than *_CURRENT.)
    #define X_MICROSTEPS     16        // 0..256
    #define X_RSENSE          0.11
    #define X_CHAIN_POS      -1        // -1..0: Not chained. 1: MCU MOSI connected. 2: Next in chain, ...
  #endif

  #if AXIS_IS_TMC_CONFIG(X2)
    #define X2_CURRENT      X_CURRENT
    #define X2_CURRENT_HOME X_CURRENT_HOME
    #define X2_MICROSTEPS   X_MICROSTEPS
    #define X2_RSENSE       X_RSENSE
    #define X2_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(Y)
    #define Y_CURRENT       800
    #define Y_CURRENT_HOME  Y_CURRENT
    #define Y_MICROSTEPS     16
    #define Y_RSENSE          0.11
    #define Y_CHAIN_POS      -1
  #endif

  #if AXIS_IS_TMC_CONFIG(Y2)
    #define Y2_CURRENT      Y_CURRENT
    #define Y2_CURRENT_HOME Y_CURRENT_HOME
    #define Y2_MICROSTEPS   Y_MICROSTEPS
    #define Y2_RSENSE       Y_RSENSE
    #define Y2_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(Z)
    #define Z_CURRENT       800
    #define Z_CURRENT_HOME  Z_CURRENT
    #define Z_MICROSTEPS     16
    #define Z_RSENSE          0.11
    #define Z_CHAIN_POS      -1
  #endif

  #if AXIS_IS_TMC_CONFIG(Z2)
    #define Z2_CURRENT      Z_CURRENT
    #define Z2_CURRENT_HOME Z_CURRENT_HOME
    #define Z2_MICROSTEPS   Z_MICROSTEPS
    #define Z2_RSENSE       Z_RSENSE
    #define Z2_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(Z3)
    #define Z3_CURRENT      Z_CURRENT
    #define Z3_CURRENT_HOME Z_CURRENT_HOME
    #define Z3_MICROSTEPS   Z_MICROSTEPS
    #define Z3_RSENSE       Z_RSENSE
    #define Z3_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(Z4)
    #define Z4_CURRENT      Z_CURRENT
    #define Z4_CURRENT_HOME Z_CURRENT_HOME
    #define Z4_MICROSTEPS   Z_MICROSTEPS
    #define Z4_RSENSE       Z_RSENSE
    #define Z4_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(I)
    #define I_CURRENT      800
    #define I_CURRENT_HOME I_CURRENT
    #define I_MICROSTEPS    16
    #define I_RSENSE         0.11
    #define I_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(J)
    #define J_CURRENT      800
    #define J_CURRENT_HOME J_CURRENT
    #define J_MICROSTEPS    16
    #define J_RSENSE         0.11
    #define J_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(K)
    #define K_CURRENT      800
    #define K_CURRENT_HOME K_CURRENT
    #define K_MICROSTEPS    16
    #define K_RSENSE         0.11
    #define K_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(U)
    #define U_CURRENT      800
    #define U_CURRENT_HOME U_CURRENT
    #define U_MICROSTEPS     8
    #define U_RSENSE         0.11
    #define U_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(V)
    #define V_CURRENT      800
    #define V_CURRENT_HOME V_CURRENT
    #define V_MICROSTEPS     8
    #define V_RSENSE         0.11
    #define V_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(W)
    #define W_CURRENT      800
    #define W_CURRENT_HOME W_CURRENT
    #define W_MICROSTEPS     8
    #define W_RSENSE         0.11
    #define W_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(E0)
    #define E0_CURRENT      800
    #define E0_MICROSTEPS    16
    #define E0_RSENSE         0.11
    #define E0_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(E1)
    #define E1_CURRENT      E0_CURRENT
    #define E1_MICROSTEPS   E0_MICROSTEPS
    #define E1_RSENSE       E0_RSENSE
    #define E1_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(E2)
    #define E2_CURRENT      E0_CURRENT
    #define E2_MICROSTEPS   E0_MICROSTEPS
    #define E2_RSENSE       E0_RSENSE
    #define E2_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(E3)
    #define E3_CURRENT      E0_CURRENT
    #define E3_MICROSTEPS   E0_MICROSTEPS
    #define E3_RSENSE       E0_RSENSE
    #define E3_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(E4)
    #define E4_CURRENT      E0_CURRENT
    #define E4_MICROSTEPS   E0_MICROSTEPS
    #define E4_RSENSE       E0_RSENSE
    #define E4_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(E5)
    #define E5_CURRENT      E0_CURRENT
    #define E5_MICROSTEPS   E0_MICROSTEPS
    #define E5_RSENSE       E0_RSENSE
    #define E5_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(E6)
    #define E6_CURRENT      E0_CURRENT
    #define E6_MICROSTEPS   E0_MICROSTEPS
    #define E6_RSENSE       E0_RSENSE
    #define E6_CHAIN_POS     -1
  #endif

  #if AXIS_IS_TMC_CONFIG(E7)
    #define E7_CURRENT      E0_CURRENT
    #define E7_MICROSTEPS   E0_MICROSTEPS
    #define E7_RSENSE       E0_RSENSE
    #define E7_CHAIN_POS     -1
  #endif

  // @section tmc/spi

  // @section tmc/serial

  // @section tmc/smart

  // @section tmc/stealthchop

  /**
   * TMC2130, TMC2160, TMC2208, TMC2209, TMC2240, TMC5130 and TMC5160 only
   * Use Trinamic's ultra quiet stepping mode.
   * When disabled, Marlin will use spreadCycle stepping mode.
   */
  #if HAS_STEALTHCHOP
    #define STEALTHCHOP_XY
    #define STEALTHCHOP_Z
    #define STEALTHCHOP_I
    #define STEALTHCHOP_J
    #define STEALTHCHOP_K
    #define STEALTHCHOP_U
    #define STEALTHCHOP_V
    #define STEALTHCHOP_W
    #define STEALTHCHOP_E
  #endif

  /**
   * Optimize spreadCycle chopper parameters by using predefined parameter sets
   * or with the help of an example included in the library.
   * Provided parameter sets are
   * CHOPPER_DEFAULT_12V
   * CHOPPER_DEFAULT_19V
   * CHOPPER_DEFAULT_24V
   * CHOPPER_DEFAULT_36V
   * CHOPPER_09STEP_24V   // 0.9 degree steppers (24V)
   * CHOPPER_PRUSAMK3_24V // Imported parameters from the official Průša firmware for MK3 (24V)
   * CHOPPER_MARLIN_119   // Old defaults from Marlin v1.1.9
   *
   * Define your own with:
   * { <off_time[1..15]>, <hysteresis_end[-3..12]>, hysteresis_start[1..8] }
   */
  #define CHOPPER_TIMING CHOPPER_DEFAULT_12V        // All axes (override below)

  // @section tmc/status

  #if ENABLED(MONITOR_DRIVER_STATUS)
    #define CURRENT_STEP_DOWN     50  // [mA]
    #define REPORT_CURRENT_CHANGE
    #define STOP_ON_ERROR
  #endif

  // @section tmc/hybrid

  #define X_HYBRID_THRESHOLD     100  // [mm/s]
  #define X2_HYBRID_THRESHOLD    100
  #define Y_HYBRID_THRESHOLD     100
  #define Y2_HYBRID_THRESHOLD    100
  #define Z_HYBRID_THRESHOLD       3
  #define Z2_HYBRID_THRESHOLD      3
  #define Z3_HYBRID_THRESHOLD      3
  #define Z4_HYBRID_THRESHOLD      3
  #define I_HYBRID_THRESHOLD       3  // [linear=mm/s, rotational=°/s]
  #define J_HYBRID_THRESHOLD       3  // [linear=mm/s, rotational=°/s]
  #define K_HYBRID_THRESHOLD       3  // [linear=mm/s, rotational=°/s]
  #define U_HYBRID_THRESHOLD       3  // [mm/s]
  #define V_HYBRID_THRESHOLD       3
  #define W_HYBRID_THRESHOLD       3
  #define E0_HYBRID_THRESHOLD     30
  #define E1_HYBRID_THRESHOLD     30
  #define E2_HYBRID_THRESHOLD     30
  #define E3_HYBRID_THRESHOLD     30
  #define E4_HYBRID_THRESHOLD     30
  #define E5_HYBRID_THRESHOLD     30
  #define E6_HYBRID_THRESHOLD     30
  #define E7_HYBRID_THRESHOLD     30

  #if ANY(SENSORLESS_HOMING, SENSORLESS_PROBING)
    #define X_STALL_SENSITIVITY  8
    #define X2_STALL_SENSITIVITY X_STALL_SENSITIVITY
    #define Y_STALL_SENSITIVITY  8
    #define Y2_STALL_SENSITIVITY Y_STALL_SENSITIVITY
  #endif

  // @section tmc/config

  /**
   * Step on both rising and falling edge signals (as with a square wave).
   */
  #define EDGE_STEPPING

  /**
   * You can set your own advanced settings by filling in predefined functions.
   * A list of available functions can be found on the library github page
   * https://github.com/teemuatlut/TMCStepper
   *
   * Example:
   * #define TMC_ADV() { \
   *   stepperX.diag0_otpw(1); \
   *   stepperY.intpol(0); \
   * }
   */
  #define TMC_ADV() {  }

#endif // HAS_TRINAMIC_CONFIG

// @section i2cbus

#if ENABLED(EXPERIMENTAL_I2CBUS)
  #define I2C_SLAVE_ADDRESS  0  // Set a value from 8 to 127 to act as a slave
#endif

// @section photo

#if ENABLED(PHOTO_GCODE)

  #ifdef PHOTO_PULSES_US
    #define PHOTO_PULSE_DELAY_US 13 // (µs) Approximate duration of each HIGH and LOW pulse in the oscillation
  #endif
#endif

// @section cnc

/**
 * Spindle & Laser control
 *
 * Add the M3, M4, and M5 commands to turn the spindle/laser on and off, and
 * to set spindle speed, spindle direction, and laser power.
 *
 * SuperPID is a router/spindle speed controller used in the CNC milling community.
 * Marlin can be used to turn the spindle on and off. It can also be used to set
 * the spindle speed from 5,000 to 30,000 RPM.
 *
 * You'll need to select a pin for the ON/OFF function and optionally choose a 0-5V
 * hardware PWM pin for the speed control and a pin for the rotation direction.
 *
 * See https://marlinfw.org/docs/configuration/2.0.9/laser_spindle.html for more config details.
 */
#define SPINDLE_FEATURE
#if ANY(SPINDLE_FEATURE, LASER_FEATURE)
  #define SPINDLE_LASER_ACTIVE_STATE    LOW    // Set to "HIGH" if SPINDLE_LASER_ENA_PIN is active HIGH

  #define SPINDLE_LASER_USE_PWM                // Enable if your controller supports setting the speed/power
  #define SPINDLE_LASER_PWM_PIN        P2_04   // Smoothieboard PWM pin for spindle (FAN1 disabled in pins_postprocess.h)
  #if ENABLED(SPINDLE_LASER_USE_PWM)
    #define SPINDLE_LASER_PWM_INVERT    false  // Set to "true" if the speed/power goes up when you want it to go slower
    #define SPINDLE_LASER_FREQUENCY     2500   // (Hz) Spindle/laser frequency (only on supported HALs: AVR, ESP32, and LPC)
  #endif

  #if ENABLED(AIR_EVACUATION)
    #define AIR_EVACUATION_ACTIVE       LOW    // Set to "HIGH" if the on/off function is active HIGH
  #endif

  #if ENABLED(AIR_ASSIST)
    #define AIR_ASSIST_ACTIVE           LOW    // Active state on air assist pin
  #endif

  #ifdef SPINDLE_SERVO
    #define SPINDLE_SERVO_NR   0               // Index of servo used for spindle control
    #define SPINDLE_SERVO_MIN 10               // Minimum angle for servo spindle
  #endif

  /**
   * Speed / Power can be set ('M3 S') and displayed in terms of:
   *  - PWM255  (S0 - S255)
   *  - PERCENT (S0 - S100)
   *  - RPM     (S0 - S50000)  Best for use with a spindle
   *  - SERVO   (S0 - S180)
   */
  #define CUTTER_POWER_UNIT PWM255

  #if ENABLED(SPINDLE_FEATURE)
    #define SPINDLE_CHANGE_DIR_STOP            // Enable if the spindle should stop before changing spin direction
    #define SPINDLE_INVERT_DIR          false  // Set to "true" if the spin direction is reversed

    #define SPINDLE_LASER_POWERUP_DELAY   5000 // (ms) Delay to allow the spindle/laser to come up to speed/power
    #define SPINDLE_LASER_POWERDOWN_DELAY 5000 // (ms) Delay to allow the spindle to stop

    /**
     * M3/M4 Power Equation
     *
     * Each tool uses different value ranges for speed / power control.
     * These parameters are used to convert between tool power units and PWM.
     *
     * Speed/Power = (PWMDC / 255 * 100 - SPEED_POWER_INTERCEPT) / SPEED_POWER_SLOPE
     * PWMDC = (spdpwr - SPEED_POWER_MIN) / (SPEED_POWER_MAX - SPEED_POWER_MIN) / SPEED_POWER_SLOPE
     */
    #if ENABLED(SPINDLE_LASER_USE_PWM)
      #define SPEED_POWER_INTERCEPT       0    // (%) 0-100 i.e., Minimum power percentage
      #define SPEED_POWER_MIN         10000    // (RPM) 10k RPM min
      #define SPEED_POWER_MAX         30000    // (RPM) 30k RPM max
      #define SPEED_POWER_STARTUP     20000    // (RPM) Default startup RPM

    #endif

  #else

    #if ENABLED(SPINDLE_LASER_USE_PWM)
      #define SPEED_POWER_INTERCEPT       0    // (%) 0-100 i.e., Minimum power percentage
      #define SPEED_POWER_MIN             0    // (%) 0-100
      #define SPEED_POWER_MAX           100    // (%) 0-100
      #define SPEED_POWER_STARTUP        80    // (%) M3/M4 speed/power default (with no arguments)
    #endif

    #define LASER_TEST_PULSE_MIN           1   // (ms) Used with Laser Control Menu
    #define LASER_TEST_PULSE_MAX         999   // (ms) Caution: Menu may not show more than 3 characters

    #define SPINDLE_LASER_POWERUP_DELAY   50   // (ms) Delay to allow the spindle/laser to come up to speed/power
    #define SPINDLE_LASER_POWERDOWN_DELAY 50   // (ms) Delay to allow the spindle to stop

   /**
    * Laser Safety Timeout
    *
    * The laser should be turned off when there is no movement for a period of time.
    * Consider material flammability, cut rate, and G-code order when setting this
    * value. Too low and it could turn off during a very slow move; too high and
    * the material could ignite.
    */
    #define LASER_SAFETY_TIMEOUT_MS     1000   // (ms)

    #if ENABLED(I2C_AMMETER)
      #define I2C_AMMETER_IMAX            0.1    // (Amps) Calibration value for the expected current range
      #define I2C_AMMETER_SHUNT_RESISTOR  0.1    // (Ohms) Calibration shunt resistor value
    #endif

    #if ENABLED(LASER_COOLANT_FLOW_METER)
      #define FLOWMETER_PIN         20  // Requires an external interrupt-enabled pin (e.g., RAMPS 2,3,18,19,20,21)
      #define FLOWMETER_PPL       5880  // (pulses/liter) Flow meter pulses-per-liter on the input pin
      #define FLOWMETER_INTERVAL  1000  // (ms) Flow rate calculation interval in milliseconds
      #define FLOWMETER_SAFETY          // Prevent running the laser without the minimum flow rate set below
      #if ENABLED(FLOWMETER_SAFETY)
        #define FLOWMETER_MIN_LITERS_PER_MINUTE 1.5 // (liters/min) Minimum flow required when enabled
      #endif
    #endif

  #endif
#endif // SPINDLE_FEATURE || LASER_FEATURE

#if ENABLED(COOLANT_CONTROL)
  #define COOLANT_MIST                // Enable if mist coolant is present
  #define COOLANT_FLOOD               // Enable if flood coolant is present
  #define COOLANT_MIST_INVERT  false  // Set "true" if the on/off function is reversed
  #define COOLANT_FLOOD_INVERT false  // Set "true" if the on/off function is reversed
#endif

// @section filament width

#if ENABLED(FILAMENT_WIDTH_SENSOR)
  #define FILAMENT_SENSOR_EXTRUDER_NUM 0    // Index of the extruder that has the filament sensor. :[0,1,2,3,4]
  #define MEASUREMENT_DELAY_CM        14    // (cm) The distance from the filament sensor to the melting chamber

  #define FILWIDTH_ERROR_MARGIN        1.0  // (mm) If a measurement differs too much from nominal width ignore it
  #define MAX_MEASUREMENT_DELAY       20    // (bytes) Buffer size for stored measurements (1 byte per cm). Must be larger than MEASUREMENT_DELAY_CM.

  #define DEFAULT_MEASURED_FILAMENT_DIA DEFAULT_NOMINAL_FILAMENT_DIA // Set measured to nominal initially

#endif

// @section power

#if ENABLED(POWER_MONITOR_CURRENT)
  #define POWER_MONITOR_VOLTS_PER_AMP    0.05000  // Input voltage to the MCU analog pin per amp  - DO NOT apply more than ADC_VREF!
  #define POWER_MONITOR_CURRENT_OFFSET   0        // Offset (in amps) applied to the calculated current
  #define POWER_MONITOR_FIXED_VOLTAGE   13.6      // Voltage for a current sensor with no voltage sensor (for power display)
#endif

#if ENABLED(POWER_MONITOR_VOLTAGE)
  #define POWER_MONITOR_VOLTS_PER_VOLT  0.077933  // Input voltage to the MCU analog pin per volt - DO NOT apply more than ADC_VREF!
  #define POWER_MONITOR_VOLTAGE_OFFSET  0         // Offset (in volts) applied to the calculated voltage
#endif

// @section safety

// @section cnc

/**
 * CNC Coordinate Systems
 *
 * Enables G53 and G54-G59.3 commands to select coordinate systems
 * and G92.1 to reset the workspace to native machine space.
 */
#define CNC_COORDINATE_SYSTEMS

// @section security

// @section volumetrics

#if DISABLED(NO_VOLUMETRICS)

  #if ENABLED(VOLUMETRIC_EXTRUDER_LIMIT)
    /**
     * Default volumetric extrusion limit in cubic mm per second (mm^3/sec).
     * This factory setting applies to all extruders.
     * Use 'M200 [T<extruder>] L<limit>' to override and 'M502' to reset.
     * A non-zero value activates Volume-based Extrusion Limiting.
     */
    #define DEFAULT_VOLUMETRIC_EXTRUDER_LIMIT  0.00     // (mm^3/sec)
    #define VOLUMETRIC_EXTRUDER_LIMIT_MAX     20        // (mm^3/sec)
  #endif
#endif

// @section reporting

/**
 * Auto-report temperatures with M155 S<seconds>
 */
#define AUTO_REPORT_TEMPERATURES

/**
 * M115 - Report capabilities. Disable to save ~1150 bytes of flash.
 *        Some hosts (and serial TFT displays) rely on this feature.
 */
#define CAPABILITIES_REPORT
#if ENABLED(CAPABILITIES_REPORT)
  #define EXTENDED_CAPABILITIES_REPORT
#endif

// @section gcode

/**
 * Spend 28 bytes of SRAM to optimize the G-code parser
 */
#define FASTER_GCODE_PARSER

/**
 * Enable M111 debug flags 1=ECHO, 2=INFO, 4=ERRORS (unimplemented).
 * Disable to save some flash. Some hosts (Repetier Host) may rely on this feature.
 */
#define DEBUG_FLAGS_GCODE

#if ENABLED(GCODE_MACROS)
  #define GCODE_MACROS_SLOTS       5  // Up to 10 may be used
  #define GCODE_MACROS_SLOT_SIZE  50  // Maximum length of a single macro
#endif

/**
 * User-defined menu items to run custom G-code.
 * Up to 25 may be defined, but the actual number is LCD-dependent.
 */

// @section custom main menu

#if ENABLED(CUSTOM_MENU_MAIN)
  #define CUSTOM_MENU_MAIN_SCRIPT_DONE "M117 User Script Done"
  #define CUSTOM_MENU_MAIN_SCRIPT_AUDIBLE_FEEDBACK
  #define CUSTOM_MENU_MAIN_ONLY_IDLE         // Only show custom menu when the machine is idle

  #define MAIN_MENU_ITEM_1_DESC "Home & UBL Info"
  #define MAIN_MENU_ITEM_1_GCODE "G28\nG29 W"

  #define MAIN_MENU_ITEM_2_DESC "Preheat for " PREHEAT_1_LABEL
  #define MAIN_MENU_ITEM_2_GCODE "M140 S" STRINGIFY(PREHEAT_1_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND)

#endif

// @section custom config menu

#if ENABLED(CUSTOM_MENU_CONFIG)
  #define CUSTOM_MENU_CONFIG_SCRIPT_DONE "M117 Wireless Script Done"
  #define CUSTOM_MENU_CONFIG_SCRIPT_AUDIBLE_FEEDBACK
  #define CUSTOM_MENU_CONFIG_ONLY_IDLE        // Only show custom menu when the machine is idle

  #define CONFIG_MENU_ITEM_1_DESC "Wifi ON"
  #define CONFIG_MENU_ITEM_1_GCODE "M118 [ESP110] WIFI-STA pwd=12345678"

  #define CONFIG_MENU_ITEM_2_DESC "Bluetooth ON"
  #define CONFIG_MENU_ITEM_2_GCODE "M118 [ESP110] BT pwd=12345678"

#endif

// @section custom buttons

#if ENABLED(CUSTOM_USER_BUTTONS)
  #if PIN_EXISTS(BUTTON1)
    #define BUTTON1_HIT_STATE     LOW       // State of the triggered button. NC=LOW. NO=HIGH.
    #define BUTTON1_WHEN_PRINTING false     // Button allowed to trigger during printing?
    #define BUTTON1_GCODE         "G28"
    #define BUTTON1_DESC          "Homing"  // Optional string to set the LCD status
  #endif

  #if PIN_EXISTS(BUTTON2)
    #define BUTTON2_HIT_STATE     LOW
    #define BUTTON2_WHEN_PRINTING false
    #define BUTTON2_GCODE         "M140 S" STRINGIFY(PREHEAT_1_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_1_TEMP_HOTEND)
    #define BUTTON2_DESC          "Preheat for " PREHEAT_1_LABEL
  #endif

  #if PIN_EXISTS(BUTTON3)
    #define BUTTON3_HIT_STATE     LOW
    #define BUTTON3_WHEN_PRINTING false
    #define BUTTON3_GCODE         "M140 S" STRINGIFY(PREHEAT_2_TEMP_BED) "\nM104 S" STRINGIFY(PREHEAT_2_TEMP_HOTEND)
    #define BUTTON3_DESC          "Preheat for " PREHEAT_2_LABEL
  #endif
#endif

// @section host

// @section extras

#if ENABLED(CANCEL_OBJECTS)
  #define CANCEL_OBJECTS_REPORTING // Emit the current object as a status message
#endif

#if ENABLED(I2C_POSITION_ENCODERS)

  #define I2CPE_ENCODER_CNT         1                       // The number of encoders installed; max of 5

  #define I2CPE_ENC_1_ADDR          I2CPE_PRESET_ADDR_X     // I2C address of the encoder. 30-200.
  #define I2CPE_ENC_1_AXIS          X_AXIS                  // Axis the encoder module is installed on.  <X|Y|Z|E>_AXIS.
  #define I2CPE_ENC_1_TYPE          I2CPE_ENC_TYPE_LINEAR   // Type of encoder:  I2CPE_ENC_TYPE_LINEAR -or-
  #define I2CPE_ENC_1_TICKS_UNIT    2048                    // 1024 for magnetic strips with 2mm poles; 2048 for
  #define I2CPE_ENC_1_EC_METHOD     I2CPE_ECM_MICROSTEP     // Type of error correction.
  #define I2CPE_ENC_1_EC_THRESH     0.10                    // Threshold size for error (in mm) above which the

  #define I2CPE_ENC_2_ADDR          I2CPE_PRESET_ADDR_Y     // Same as above, but for encoder 2.
  #define I2CPE_ENC_2_AXIS          Y_AXIS
  #define I2CPE_ENC_2_TYPE          I2CPE_ENC_TYPE_LINEAR
  #define I2CPE_ENC_2_TICKS_UNIT    2048
  #define I2CPE_ENC_2_EC_METHOD     I2CPE_ECM_MICROSTEP
  #define I2CPE_ENC_2_EC_THRESH     0.10

  #define I2CPE_ENC_3_ADDR          I2CPE_PRESET_ADDR_Z     // Encoder 3.  Add additional configuration options
  #define I2CPE_ENC_3_AXIS          Z_AXIS                  // as above, or use defaults below.

  #define I2CPE_ENC_4_ADDR          I2CPE_PRESET_ADDR_E     // Encoder 4.
  #define I2CPE_ENC_4_AXIS          E_AXIS

  #define I2CPE_ENC_5_ADDR          34                      // Encoder 5.
  #define I2CPE_ENC_5_AXIS          E_AXIS

  #define I2CPE_DEF_TYPE            I2CPE_ENC_TYPE_LINEAR
  #define I2CPE_DEF_ENC_TICKS_UNIT  2048
  #define I2CPE_DEF_TICKS_REV       (16 * 200)
  #define I2CPE_DEF_EC_METHOD       I2CPE_ECM_NONE
  #define I2CPE_DEF_EC_THRESH       0.1

  #define I2CPE_TIME_TRUSTED        10000                   // After an encoder fault, there must be no further fault

  /**
   * Position is checked every time a new command is executed from the buffer but during long moves,
   * this setting determines the minimum update time between checks. A value of 100 works well with
   * error rolling average when attempting to correct only for skips and not for vibration.
   */
  #define I2CPE_MIN_UPD_TIME_MS     4                       // (ms) Minimum time between encoder checks.

  #define I2CPE_ERR_ROLLING_AVERAGE

#endif // I2C_POSITION_ENCODERS

#if ENABLED(JOYSTICK)
  #define JOY_X_PIN    5  // RAMPS: Suggested pin A5  on AUX2
  #define JOY_Y_PIN   10  // RAMPS: Suggested pin A10 on AUX2
  #define JOY_Z_PIN   12  // RAMPS: Suggested pin A12 on AUX2
  #define JOY_EN_PIN  44  // RAMPS: Suggested pin D44 on AUX2

  #define JOY_X_LIMITS { 5600, 8190-100, 8190+100, 10800 } // min, deadzone start, deadzone end, max
  #define JOY_Y_LIMITS { 5600, 8250-100, 8250+100, 11000 }
  #define JOY_Z_LIMITS { 4800, 8080-100, 8080+100, 11550 }
#endif

#if ENABLED(MECHANICAL_GANTRY_CALIBRATION)
  #define GANTRY_CALIBRATION_CURRENT          600     // Default calibration current in ma
  #define GANTRY_CALIBRATION_EXTRA_HEIGHT      15     // Extra distance in mm past Z_###_POS to move
  #define GANTRY_CALIBRATION_FEEDRATE         500     // Feedrate for correction move

  #define GANTRY_CALIBRATION_COMMANDS_POST  "G28"     // G28 highly recommended to ensure an accurate position
#endif

#if ENABLED(FREEZE_FEATURE)
  #define FREEZE_STATE  LOW   // State of pin indicating freeze
#endif

#if ANY(FREEZE_FEATURE, REALTIME_REPORTING_COMMANDS)
  #if ENABLED(SOFT_FEED_HOLD)
    #define FREEZE_JERK     2   // (mm/s) Completely halt when motion has decelerated below this value
  #endif
#endif

#if ENABLED(MAX7219_DEBUG)
  #define MAX7219_CLK_PIN   64
  #define MAX7219_DIN_PIN   57
  #define MAX7219_LOAD_PIN  44

  #define MAX7219_INIT_TEST    2   // Test pattern at startup: 0=none, 1=sweep, 2=spiral
  #define MAX7219_NUMBER_UNITS 1   // Number of Max7219 units in chain.
  #define MAX7219_ROTATE       0   // Rotate the display clockwise (in multiples of +/- 90°)

  /**
   * Sample debug features
   * If you add more debug displays, be careful to avoid conflicts!
   */
  #define MAX7219_DEBUG_PRINTER_ALIVE     // Blink corner LED of 8x8 matrix to show that the firmware is functioning
  #define MAX7219_DEBUG_PLANNER_HEAD  2   // Show the planner queue head position on this and the next LED matrix row
  #define MAX7219_DEBUG_PLANNER_TAIL  4   // Show the planner queue tail position on this and the next LED matrix row

  #define MAX7219_DEBUG_PLANNER_QUEUE 0   // Show the current planner queue depth on this and the next LED matrix row
  #define MAX7219_DEBUG_PROFILE       6   // Display the fraction of CPU time spent in profiled code on this LED matrix
#endif

/**
 * Ethernet. Use M552 to enable and set the IP address.
 * @section network
 */
#if HAS_ETHERNET
  #define MAC_ADDRESS { 0xDE, 0xAD, 0xBE, 0xEF, 0xF0, 0x0D }  // A MAC address unique to your network
#endif

/**
 * Extras for an ESP32-based motherboard with WIFISUPPORT
 * These options don't apply to add-on WiFi modules based on ESP32 WiFi101.
 */

// @section multi-material

/**
 * Průša Multi-Material Unit (MMU)
 * Enable in Configuration.h
 *
 * These devices allow a single stepper driver on the board to drive
 * multi-material feeders with any number of stepper motors.
 */
#if HAS_PRUSA_MMU1

#elif HAS_PRUSA_MMU2 || HAS_PRUSA_MMU3
  #define MMU_SERIAL_PORT 2
  #define MMU_BAUD 115200

  #if HAS_PRUSA_MMU2

    #define MMU2_FILAMENTCHANGE_EJECT_FEED 80.0

    #define MMU2_FILAMENT_RUNOUT_SCRIPT "M600"

    #define MMU2_LOAD_TO_NOZZLE_SEQUENCE \
      {  4.4,  871 }, \
      { 10.0, 1393 }, \
      {  4.4,  871 }, \
      { 10.0,  198 }

    #define MMU2_RAMMING_SEQUENCE \
      {   1.0, 1000 }, \
      {   1.0, 1500 }, \
      {   2.0, 2000 }, \
      {   1.5, 3000 }, \
      {   2.5, 4000 }, \
      { -15.0, 5000 }, \
      { -14.0, 1200 }, \
      {  -6.0,  600 }, \
      {  10.0,  700 }, \
      { -10.0,  400 }, \
      { -50.0, 2000 }

  #endif // HAS_PRUSA_MMU2

  /**
   * Options pertaining to MMU2S devices
   * Requires the MK3S extruder with a sensor at the extruder idler, like the MMU2S.
   * See https://help.prusa3d.com/guide/3b-mk3s-mk2-5s-extruder-upgrade_41560#42048, step 11
   */
  #if HAS_PRUSA_MMU2S
    #define MMU2_C0_RETRY   5             // Number of retries (total time = timeout*retries)

    /**
     * This is called after the filament runout sensor is triggered to check if
     * the filament has been loaded properly by moving the filament back and
     * forth to see if the filament runout sensor is going to get triggered
     * again, which should not occur if the filament is properly loaded.
     *
     * Thus, the MMU2_CAN_LOAD_SEQUENCE should contain some forward and
     * backward moves. The forward moves should be greater than the backward
     * moves.
     *
     * This is useless if your filament runout sensor is way behind the gears.
     * In that case use {0, MMU2_CAN_LOAD_FEEDRATE}
     *
     * Adjust MMU2_CAN_LOAD_SEQUENCE according to your setup.
     */
    #define MMU2_CAN_LOAD_FEEDRATE 800    // (mm/min)
    #define MMU2_CAN_LOAD_SEQUENCE \
      {   5.0, MMU2_CAN_LOAD_FEEDRATE }, \
      {  15.0, MMU2_CAN_LOAD_FEEDRATE }, \
      { -10.0, MMU2_CAN_LOAD_FEEDRATE }

    #define MMU2_CAN_LOAD_RETRACT   6.0   // (mm) Keep under the distance between Load Sequence values
    #define MMU2_CAN_LOAD_DEVIATION 0.8   // (mm) Acceptable deviation

    #define MMU2_CAN_LOAD_INCREMENT 0.2   // (mm) To reuse within MMU2 module
    #define MMU2_CAN_LOAD_INCREMENT_SEQUENCE \
      { -MMU2_CAN_LOAD_INCREMENT, MMU2_CAN_LOAD_FEEDRATE }

  #elif HAS_PRUSA_MMU3

    #define MMU3_HAS_CUTTER     // Enable cutter related functionality

    #define MMU3_MAX_RETRIES 3  // Number of retries (total time = timeout*retries)

    #define MMU3_FILAMENT_SENSOR_E_POSITION  0   // (mm)
    #define _MMU3_LOAD_DISTANCE_PAST_GEARS   5   // (mm)
    #define MMU3_TOOL_CHANGE_LOAD_LENGTH (MMU3_FILAMENT_SENSOR_E_POSITION + _MMU3_LOAD_DISTANCE_PAST_GEARS) // (mm)

    #define MMU3_LOAD_TO_NOZZLE_FEED_RATE        20.0 // (mm/s)

    #define MMU3_VERIFY_LOAD_TO_NOZZLE_FEED_RATE 50.0 // (mm/s)
    #define _MMU3_VERIFY_LOAD_TO_NOZZLE_TWEAK    -5.0 // (mm) Amount to adjust the length for verifying load-to-nozzle

    #define MMU3_RETRY_UNLOAD_TO_FINDA_LENGTH    80.0 // (mm)
    #define MMU3_RETRY_UNLOAD_TO_FINDA_FEED_RATE 80.0 // (mm/s)

    #define _MMU_EXTRUDER_PTFE_LENGTH            42.3 // (mm)
    #define _MMU_EXTRUDER_HEATBREAK_LENGTH       17.7 // (mm)
    #define MMU3_CHECK_FILAMENT_PRESENCE_EXTRUSION_LENGTH (MMU3_FILAMENT_SENSOR_E_POSITION + _MMU_EXTRUDER_PTFE_LENGTH + _MMU_EXTRUDER_HEATBREAK_LENGTH + _MMU3_VERIFY_LOAD_TO_NOZZLE_TWEAK) // (mm)

    #define MMU3_LOAD_TO_NOZZLE_SEQUENCE \
      { _MMU_EXTRUDER_PTFE_LENGTH,      MMM_TO_MMS(810) }, /* (13.5 mm/s) Fast load ahead of heatbreak */ \
      { _MMU_EXTRUDER_HEATBREAK_LENGTH, MMM_TO_MMS(198) }  /* ( 3.3 mm/s) Slow load after heatbreak */

    #define MMU3_RAMMING_SEQUENCE \
      { 0.2816,  MMM_TO_MMS(1339.0) }, \
      { 0.3051,  MMM_TO_MMS(1451.0) }, \
      { 0.3453,  MMM_TO_MMS(1642.0) }, \
      { 0.3990,  MMM_TO_MMS(1897.0) }, \
      { 0.4761,  MMM_TO_MMS(2264.0) }, \
      { 0.5767,  MMM_TO_MMS(2742.0) }, \
      { 0.5691,  MMM_TO_MMS(3220.0) }, \
      { 0.1081,  MMM_TO_MMS(3220.0) }, \
      { 0.7644,  MMM_TO_MMS(3635.0) }, \
      { 0.8248,  MMM_TO_MMS(3921.0) }, \
      { 0.8483,  MMM_TO_MMS(4033.0) }, \
      { -15.0,   MMM_TO_MMS(6000.0) }, \
      { -24.5,   MMM_TO_MMS(1200.0) }, \
      {  -7.0,   MMM_TO_MMS( 600.0) }, \
      {  -3.5,   MMM_TO_MMS( 360.0) }, \
      {  20.0,   MMM_TO_MMS( 454.0) }, \
      { -20.0,   MMM_TO_MMS( 303.0) }, \
      { -35.0,   MMM_TO_MMS(2000.0) }

  #else // MMU2 (not MMU2S)

    #if ENABLED(MMU2_EXTRUDER_SENSOR)
      #define MMU2_LOADING_ATTEMPTS_NR 5  // Number of times to try loading filament before failure
    #endif

  #endif

#endif // HAS_PRUSA_MMU2 || HAS_PRUSA_MMU3

/**
 * Advanced Print Counter settings
 * @section stats
 */
#if ENABLED(PRINTCOUNTER)
  #define SERVICE_WARNING_BUZZES  3
#endif

// @section develop
