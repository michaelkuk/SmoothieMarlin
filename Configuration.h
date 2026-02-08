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
 * Configuration.h
 *
 * Basic settings such as:
 *
 * - Type of electronics
 * - Type of temperature sensor
 * - Printer geometry
 * - Endstop configuration
 * - LCD controller
 * - Extra features
 *
 * Advanced settings can be found in Configuration_adv.h
 */
#define CONFIGURATION_H_VERSION 02010300

//===========================================================================
//============================= Getting Started =============================
//===========================================================================

/**
 * Here are some useful links to help get your machine configured and
 * calibrated:
 *
 * Example Configs:
 * https://github.com/MarlinFirmware/Configurations/branches/all
 *
 * Průša Calculator:    https://blog.prusa3d.com/calculator_3416/
 *
 * Calibration Guides:  https://reprap.org/wiki/Calibration
 *                      https://reprap.org/wiki/Triffid_Hunter%27s_Calibration_Guide
 *                      https://web.archive.org/web/20220907014303/sites.google.com/site/repraplogphase/calibration-of-your-reprap
 *                      https://youtu.be/wAL9d7FgInk
 *                      https://teachingtechyt.github.io/calibration.html
 *
 * Calibration Objects: https://www.thingiverse.com/thing:5573
 *                      https://www.thingiverse.com/thing:1278865
 */

// @section info

#define STRING_CONFIG_H_AUTHOR                                                 \
  "(MarlinFirmware)" // Original author or contributor.

// @section machine

#ifndef MOTHERBOARD
#define MOTHERBOARD BOARD_SMOOTHIEBOARD
#endif

// @section serial

/**
 * Select the serial port on the board to use for communication with the host.
 * This allows the connection of wireless adapters (for instance) to non-default
 * port pins. Serial port -1 is the USB emulated serial port, if available.
 * Note: The first serial port (-1 or 0) will always be used by the Arduino
 * bootloader.
 *
 * :[-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
 */
#define SERIAL_PORT -1

/**
 * Serial Port Baud Rate
 * This is the default communication speed for all serial ports.
 * Set the baud rate defaults for additional serial ports below.
 *
 * 250000 works in most cases, but you might try a lower speed if
 * you commonly experience drop-outs during host printing.
 * You may try up to 1000000 to speed up SD file transfer.
 *
 * :[2400, 9600, 19200, 38400, 57600, 115200, 250000, 500000, 1000000]
 */
#define BAUDRATE 250000

// @section stepper drivers

/**
 * Stepper Drivers
 *
 * These settings allow Marlin to tune stepper driver timing and enable advanced
 * options for stepper drivers that support them. You may also override timing
 * options in Configuration_adv.h.
 *
 * Use TMC2208/TMC2208_STANDALONE for TMC2225 drivers and
 * TMC2209/TMC2209_STANDALONE for TMC2226 drivers.
 *
 * Options: A4988, A5984, DRV8825, LV8729, TB6560, TB6600, TMC2100,
 *          TMC2130, TMC2130_STANDALONE, TMC2160, TMC2160_STANDALONE,
 *          TMC2208, TMC2208_STANDALONE, TMC2209, TMC2209_STANDALONE,
 *          TMC2240, TMC2660, TMC2660_STANDALONE,
 *          TMC5130, TMC5130_STANDALONE, TMC5160, TMC5160_STANDALONE
 * :['A4988', 'A5984', 'DRV8825', 'LV8729', 'TB6560', 'TB6600', 'TMC2100',
 * 'TMC2130', 'TMC2130_STANDALONE', 'TMC2160', 'TMC2160_STANDALONE', 'TMC2208',
 * 'TMC2208_STANDALONE', 'TMC2209', 'TMC2209_STANDALONE', 'TMC2240', 'TMC2660',
 * 'TMC2660_STANDALONE', 'TMC5130', 'TMC5130_STANDALONE', 'TMC5160',
 * 'TMC5160_STANDALONE']
 */
#define X_DRIVER_TYPE A4988
#define Y_DRIVER_TYPE A4988
#define Z_DRIVER_TYPE A4988
#define X2_DRIVER_TYPE A4988
#define Y2_DRIVER_TYPE A4988
#define E0_DRIVER_TYPE A4988

/**
 * Additional Axis Settings
 *
 * Define AXISn_ROTATES for all axes that rotate or pivot.
 * Rotational axis coordinates are expressed in degrees.
 *
 * AXISn_NAME defines the letter used to refer to the axis in (most) G-code
 * commands. By convention the names and roles are typically: 'A' : Rotational
 * axis parallel to X 'B' : Rotational axis parallel to Y 'C' : Rotational axis
 * parallel to Z 'U' : Secondary linear axis parallel to X 'V' : Secondary
 * linear axis parallel to Y 'W' : Secondary linear axis parallel to Z
 *
 * Regardless of these settings the axes are internally named I, J, K, U, V, W.
 */
#ifdef I_DRIVER_TYPE
#define AXIS4_NAME 'A' // :['A', 'B', 'C', 'U', 'V', 'W']
#define AXIS4_ROTATES
#endif
#ifdef J_DRIVER_TYPE
#define AXIS5_NAME 'B' // :['B', 'C', 'U', 'V', 'W']
#define AXIS5_ROTATES
#endif
#ifdef K_DRIVER_TYPE
#define AXIS6_NAME 'C' // :['C', 'U', 'V', 'W']
#define AXIS6_ROTATES
#endif
#ifdef U_DRIVER_TYPE
#define AXIS7_NAME 'U' // :['U', 'V', 'W']
#endif
#ifdef V_DRIVER_TYPE
#define AXIS8_NAME 'V' // :['V', 'W']
#endif
#ifdef W_DRIVER_TYPE
#define AXIS9_NAME 'W' // :['W']
#endif

// @section extruder

#define EXTRUDERS 0

#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75

#if ENABLED(SWITCHING_EXTRUDER)
#define SWITCHING_EXTRUDER_SERVO_NR 0
#define SWITCHING_EXTRUDER_SERVO_ANGLES {0, 90} // Angles for E0, E1[, E2, E3]
#if EXTRUDERS > 3
#define SWITCHING_EXTRUDER_E23_SERVO_NR 1
#endif
#endif

#if ENABLED(SWITCHING_NOZZLE)
#define SWITCHING_NOZZLE_SERVO_NR 0
#define SWITCHING_NOZZLE_SERVO_ANGLES                                          \
  {0, 90} // A pair of angles for { E0, E1 }.
#define SWITCHING_NOZZLE_SERVO_DWELL                                           \
  2500 // Dwell time to wait for servo to make physical move
#define SWITCHING_NOZZLE_LIFT_TO_PROBE // Lift toolheads out of the way while
#endif

#if ANY(PARKING_EXTRUDER, MAGNETIC_PARKING_EXTRUDER)

#define PARKING_EXTRUDER_PARKING_X                                             \
  {-78, 184} // X positions for parking the extruders
#define PARKING_EXTRUDER_GRAB_DISTANCE                                         \
  1 // (mm) Distance to move beyond the parking point to grab the extruder

#if ENABLED(PARKING_EXTRUDER)

#define PARKING_EXTRUDER_SOLENOIDS_INVERT // If enabled, the solenoid is NOT
#define PARKING_EXTRUDER_SOLENOIDS_PINS_ACTIVE                                 \
  LOW // LOW or HIGH pin signal energizes the coil
#define PARKING_EXTRUDER_SOLENOIDS_DELAY                                       \
  250 // (ms) Delay for magnetic field. No delay if 0 or not defined.

#elif ENABLED(MAGNETIC_PARKING_EXTRUDER)

#define MPE_FAST_SPEED                                                         \
  9000 // (mm/min) Speed for travel before last distance point
#define MPE_SLOW_SPEED                                                         \
  4500 // (mm/min) Speed for last distance travel to park and couple
#define MPE_TRAVEL_DISTANCE 10 // (mm) Last distance point
#define MPE_COMPENSATION                                                       \
  0 // Offset Compensation -1 , 0 , 1 (multiplier) only for coupling

#endif

#endif

#if ANY(SWITCHING_TOOLHEAD, MAGNETIC_SWITCHING_TOOLHEAD,                       \
        ELECTROMAGNETIC_SWITCHING_TOOLHEAD)
#define SWITCHING_TOOLHEAD_Y_POS 235     // (mm) Y position of the toolhead dock
#define SWITCHING_TOOLHEAD_Y_SECURITY 10 // (mm) Security distance Y axis
#define SWITCHING_TOOLHEAD_Y_CLEAR                                             \
  60 // (mm) Minimum distance from dock for unobstructed X axis
#define SWITCHING_TOOLHEAD_X_POS                                               \
  {215, 0} // (mm) X positions for parking the extruders
#if ENABLED(SWITCHING_TOOLHEAD)
#define SWITCHING_TOOLHEAD_SERVO_NR 2 // Index of the servo connector
#define SWITCHING_TOOLHEAD_SERVO_ANGLES                                        \
  {0, 180} // (degrees) Angles for Lock, Unlock
#elif ENABLED(MAGNETIC_SWITCHING_TOOLHEAD)
#define SWITCHING_TOOLHEAD_Y_RELEASE 5 // (mm) Security distance Y axis
#define SWITCHING_TOOLHEAD_X_SECURITY                                          \
  {90, 150} // (mm) Security distance X axis (T0,T1)
#if ENABLED(PRIME_BEFORE_REMOVE)
#define SWITCHING_TOOLHEAD_PRIME_MM 20   // (mm)   Extruder prime length
#define SWITCHING_TOOLHEAD_RETRACT_MM 10 // (mm)   Retract after priming length
#define SWITCHING_TOOLHEAD_PRIME_FEEDRATE                                      \
  300 // (mm/min) Extruder prime feedrate
#define SWITCHING_TOOLHEAD_RETRACT_FEEDRATE                                    \
  2400 // (mm/min) Extruder retract feedrate
#endif
#elif ENABLED(ELECTROMAGNETIC_SWITCHING_TOOLHEAD)
#define SWITCHING_TOOLHEAD_Z_HOP 2 // (mm) Z raise for switching
#endif
#endif

#if ENABLED(MIXING_EXTRUDER)
#define MIXING_STEPPERS 2 // Number of steppers in your mixing extruder
#define MIXING_VIRTUAL_TOOLS                                                   \
  16 // Use the Virtual Tool method with M163 and M164
#endif

// @section multi-material

// @section psu control

#if ENABLED(PSU_CONTROL)
#define PSU_ACTIVE_STATE LOW // Set 'LOW' for ATX, 'HIGH' for X-Box

#define PS_EDM_RESPONSE 250 // (ms) Time to allow for relay action

#if ENABLED(AUTO_POWER_CONTROL)
#define AUTO_POWER_FANS          // Turn on PSU for fans
#define AUTO_POWER_E_FANS        // Turn on PSU for E Fans
#define AUTO_POWER_CONTROLLERFAN // Turn on PSU for Controller Fan
#define AUTO_POWER_CHAMBER_FAN   // Turn on PSU for Chamber Fan
#define AUTO_POWER_COOLER_FAN    // Turn on PSU for Cooler Fan
#define AUTO_POWER_SPINDLE_LASER // Turn on PSU for Spindle/Laser
#define POWER_TIMEOUT                                                          \
  30 // (s) Turn off power if the machine is idle for this duration
#endif
#endif

//===========================================================================
//============================= Thermal Settings ============================
//===========================================================================
// @section temperature sensors

/**
 * Temperature Sensors:
 *
 * NORMAL IS 4.7kΩ PULLUP! Hotend sensors can use 1kΩ pullup with correct
 * resistor and table.
 *
 * ================================================================
 *  Analog Thermistors - 4.7kΩ pullup - Normal
 * ================================================================
 *     1 : 100kΩ EPCOS - Best choice for EPCOS thermistors
 *   331 : 100kΩ Same as #1, but 3.3V scaled for MEGA
 *   332 : 100kΩ Same as #1, but 3.3V scaled for DUE
 *     2 : 200kΩ ATC Semitec 204GT-2
 *   202 : 200kΩ Copymaster 3D
 *     3 : ???Ω  Mendel-parts thermistor
 *     4 : 10kΩ  Generic Thermistor !! DO NOT use for a hotend - it gives bad
 * resolution at high temp. !! 5 : 100kΩ ATC Semitec 104GT-2/104NT-4-R025H42G -
 * Used in ParCan, J-Head, and E3D, SliceEngineering 300°C 501 : 100kΩ Zonestar
 * - Tronxy X3A 502 : 100kΩ Zonestar - used by hot bed in Zonestar Průša P802M
 *   503 : 100kΩ Zonestar (Z8XM2) Heated Bed thermistor
 *   504 : 100kΩ Zonestar P802QR2 (Part# QWG-104F-B3950) Hotend Thermistor
 *   505 : 100kΩ Zonestar P802QR2 (Part# QWG-104F-3950) Bed Thermistor
 *   512 : 100kΩ RPW-Ultra hotend
 *     6 : 100kΩ EPCOS - Not as accurate as table #1 (created using a fluke
 * thermocouple) 7 : 100kΩ Honeywell 135-104LAG-J01 71 : 100kΩ Honeywell
 * 135-104LAF-J01 8 : 100kΩ Vishay 0603 SMD NTCS0603E3104FXT 9 : 100kΩ GE
 * Sensing AL03006-58.2K-97-G1 10 : 100kΩ RS PRO 198-961 11 : 100kΩ Keenovo AC
 * silicone mats, most Wanhao i3 machines - beta 3950, 1% 12 : 100kΩ Vishay 0603
 * SMD NTCS0603E3104FXT (#8) - calibrated for Makibox hot bed 13 : 100kΩ Hisense
 * up to 300°C - for "Simple ONE" & "All In ONE" hotend - beta 3950, 1% 14 :
 * 100kΩ  (R25), 4092K (beta25), 4.7kΩ pull-up, bed thermistor as used in
 * Ender-5 S1 15 : 100kΩ Calibrated for JGAurora A5 hotend 17 : 100kΩ Dagoma NTC
 * white thermistor 18 : 200kΩ ATC Semitec 204GT-2 Dagoma.Fr -
 * MKS_Base_DKU001327 22 : 100kΩ GTM32 Pro vB - hotend - 4.7kΩ pullup to 3.3V
 * and 220Ω to analog input 23 : 100kΩ GTM32 Pro vB - bed - 4.7kΩ pullup to 3.3v
 * and 220Ω to analog input 30 : 100kΩ Kis3d Silicone heating mat 200W/300W with
 * 6mm precision cast plate (EN AW 5083) NTC100K - beta 3950 60 : 100kΩ Maker's
 * Tool Works Kapton Bed Thermistor - beta 3950 61 : 100kΩ Formbot/Vivedino
 * 350°C Thermistor - beta 3950 66 : 4.7MΩ Dyze Design / Trianglelab T-D500
 * 500°C High Temperature Thermistor 67 : 500kΩ SliceEngineering 450°C
 * Thermistor 68 : PT100 Smplifier board from Dyze Design 70 : 100kΩ bq
 * Hephestos 2 75 : 100kΩ Generic Silicon Heat Pad with NTC100K
 * MGB18-104F39050L32 666 : 200kΩ Einstart S custom thermistor with 10k pullup.
 *  2000 : 100kΩ Ultimachine Rambo TDK NTCG104LH104KT1 NTC100K motherboard
 * Thermistor
 *
 * ================================================================
 *  Analog Thermistors - 1kΩ pullup
 *   Atypical, and requires changing out the 4.7kΩ pullup for 1kΩ.
 *   (but gives greater accuracy and more stable PID)
 * ================================================================
 *    51 : 100kΩ EPCOS (1kΩ pullup)
 *    52 : 200kΩ ATC Semitec 204GT-2 (1kΩ pullup)
 *    55 : 100kΩ ATC Semitec 104GT-2 - Used in ParCan & J-Head (1kΩ pullup)
 *
 * ================================================================
 *  Analog Thermistors - 10kΩ pullup - Atypical
 * ================================================================
 *    99 : 100kΩ Found on some Wanhao i3 machines with a 10kΩ pull-up resistor
 *
 * ================================================================
 *  Analog RTDs (Pt100/Pt1000)
 * ================================================================
 *   110 : Pt100  with 1kΩ pullup (atypical)
 *   147 : Pt100  with 4.7kΩ pullup
 *  1010 : Pt1000 with 1kΩ pullup (atypical)
 *  1022 : Pt1000 with 2.2kΩ pullup
 *  1047 : Pt1000 with 4.7kΩ pullup (E3D)
 *    20 : Pt100  with circuit in the Ultimainboard V2.x with mainboard ADC
 * reference voltage = INA826 amplifier-board supply voltage. NOTE: (1) Must use
 * an ADC input with no pullup. (2) Some INA826 amplifiers are unreliable
 * at 3.3V so consider using sensor 147, 110, or 21. 21 : Pt100  with circuit in
 * the Ultimainboard V2.x with 3.3v ADC reference voltage (STM32, LPC176x....)
 * and 5V INA826 amplifier board supply. NOTE: ADC pins are not 5V tolerant. Not
 * recommended because it's possible to damage the CPU by going over 500°C. 201
 * : Pt100  with circuit in Overlord, similar to Ultimainboard V2.x
 *
 * ================================================================
 *  SPI RTD/Thermocouple Boards
 * ================================================================
 *    -5 : MAX31865 with Pt100/Pt1000, 2, 3, or 4-wire  (only for sensors 0-2
 * and bed) NOTE: You must uncomment/set the MAX31865_*_OHMS_n defines below. -3
 * : MAX31855 with Thermocouple, -200°C to +700°C (only for sensors 0-2 and bed)
 *    -2 : MAX6675  with Thermocouple, 0°C to +700°C    (only for sensors 0-2
 * and bed)
 *
 *  NOTE: Ensure TEMP_n_CS_PIN is set in your pins file for each TEMP_SENSOR_n
 * using an SPI Thermocouple. By default, Hardware SPI on the default serial bus
 * is used. If you have also set TEMP_n_SCK_PIN and TEMP_n_MISO_PIN, Software
 * SPI will be used on those ports instead. You can force Hardware SPI on the
 * default bus in the Configuration_adv.h file. At this time, separate Hardware
 * SPI buses for sensors are not supported.
 *
 * ================================================================
 *  Analog Thermocouple Boards
 * ================================================================
 *    -4 : AD8495 with Thermocouple
 *    -1 : AD595  with Thermocouple
 *
 * ================================================================
 *  SoC internal sensor
 * ================================================================
 *   100 : SoC internal sensor
 *
 * ================================================================
 *  Custom/Dummy/Other Thermal Sensors
 * ================================================================
 *     0 : not used
 *  1000 : Custom - Specify parameters in Configuration_adv.h
 *
 *   !!! Use these for Testing or Development purposes. NEVER for production
 * machine. !!! 998 : Dummy Table that ALWAYS reads 25°C or the temperature
 * defined below. 999 : Dummy Table that ALWAYS reads 100°C or the temperature
 * defined below.
 */
#define TEMP_SENSOR_0 0
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_4 0
#define TEMP_SENSOR_5 0
#define TEMP_SENSOR_6 0
#define TEMP_SENSOR_7 0
#define TEMP_SENSOR_BED 0
#define TEMP_SENSOR_PROBE 0
#define TEMP_SENSOR_CHAMBER 0
#define TEMP_SENSOR_COOLER 0
#define TEMP_SENSOR_BOARD 0
#define TEMP_SENSOR_SOC 0
#define TEMP_SENSOR_REDUNDANT 0

#define DUMMY_THERMISTOR_998_VALUE 25
#define DUMMY_THERMISTOR_999_VALUE 100

#if TEMP_SENSOR_IS_MAX_TC(0)
#define MAX31865_SENSOR_OHMS_0                                                 \
  100 // (Ω) Typically 100 or 1000 (PT100 or PT1000)
#define MAX31865_CALIBRATION_OHMS_0                                            \
  430 // (Ω) Typically 430 for Adafruit PT100; 4300 for Adafruit PT1000
#endif
#if TEMP_SENSOR_IS_MAX_TC(1)
#define MAX31865_SENSOR_OHMS_1 100
#define MAX31865_CALIBRATION_OHMS_1 430
#endif
#if TEMP_SENSOR_IS_MAX_TC(2)
#define MAX31865_SENSOR_OHMS_2 100
#define MAX31865_CALIBRATION_OHMS_2 430
#endif
#if TEMP_SENSOR_IS_MAX_TC(BED)
#define MAX31865_SENSOR_OHMS_BED 100
#define MAX31865_CALIBRATION_OHMS_BED 430
#endif

#if HAS_E_TEMP_SENSOR
#define TEMP_RESIDENCY_TIME                                                    \
  10 // (seconds) Time to wait for hotend to "settle" in M109
#define TEMP_WINDOW                                                            \
  1 // (°C) Temperature proximity for the "temperature reached" timer
#define TEMP_HYSTERESIS                                                        \
  3 // (°C) Temperature proximity considered "close enough" to the target
#endif

#if TEMP_SENSOR_BED
#define TEMP_BED_RESIDENCY_TIME                                                \
  10 // (seconds) Time to wait for bed to "settle" in M190
#define TEMP_BED_WINDOW                                                        \
  1 // (°C) Temperature proximity for the "temperature reached" timer
#define TEMP_BED_HYSTERESIS                                                    \
  3 // (°C) Temperature proximity considered "close enough" to the target
#endif

#if TEMP_SENSOR_CHAMBER
#define TEMP_CHAMBER_RESIDENCY_TIME                                            \
  10 // (seconds) Time to wait for chamber to "settle" in M191
#define TEMP_CHAMBER_WINDOW                                                    \
  1 // (°C) Temperature proximity for the "temperature reached" timer
#define TEMP_CHAMBER_HYSTERESIS                                                \
  3 // (°C) Temperature proximity considered "close enough" to the target
#endif

/**
 * Redundant Temperature Sensor (TEMP_SENSOR_REDUNDANT)
 *
 * Use a temp sensor as a redundant sensor for another reading. Select an unused
 * temperature sensor, and another sensor you'd like it to be redundant for. If
 * the two thermistors differ by TEMP_SENSOR_REDUNDANT_MAX_DIFF (°C), the print
 * will be aborted. Whichever sensor is selected will have its normal functions
 * disabled; i.e. selecting the Bed sensor (-1) will disable bed
 * heating/monitoring.
 *
 * For selecting source/target use: COOLER, PROBE, BOARD, CHAMBER, BED, E0, E1,
 * E2, E3, E4, E5, E6, E7
 */
#if TEMP_SENSOR_REDUNDANT
#define TEMP_SENSOR_REDUNDANT_SOURCE                                           \
  E1 // The sensor that will provide the redundant reading.
#define TEMP_SENSOR_REDUNDANT_TARGET                                           \
  E0 // The sensor that we are providing a redundant reading for.
#define TEMP_SENSOR_REDUNDANT_MAX_DIFF                                         \
  10 // (°C) Temperature difference that will trigger a print abort.
#endif

// @section temperature

#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define HEATER_3_MINTEMP 5
#define HEATER_4_MINTEMP 5
#define HEATER_5_MINTEMP 5
#define HEATER_6_MINTEMP 5
#define HEATER_7_MINTEMP 5
#define BED_MINTEMP 5
#define CHAMBER_MINTEMP 5

#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define HEATER_3_MAXTEMP 275
#define HEATER_4_MAXTEMP 275
#define HEATER_5_MAXTEMP 275
#define HEATER_6_MAXTEMP 275
#define HEATER_7_MAXTEMP 275
#define BED_MAXTEMP 150
#define CHAMBER_MAXTEMP 60

/**
 * Thermal Overshoot
 * During heatup (and printing) the temperature can often "overshoot" the target
 * by many degrees (especially before PID tuning). Setting the target
 * temperature too close to MAXTEMP guarantees a MAXTEMP shutdown! Use these
 * values to forbid temperatures being set too close to MAXTEMP.
 */
#define HOTEND_OVERSHOOT 15 // (°C) Forbid temperatures over MAXTEMP - OVERSHOOT
#define BED_OVERSHOOT 10    // (°C) Forbid temperatures over MAXTEMP - OVERSHOOT
#define COOLER_OVERSHOOT 2  // (°C) Forbid temperatures closer than OVERSHOOT

//===========================================================================
//============================= PID Settings ================================
//===========================================================================

// @section hotend temp

/**
 * Temperature Control
 *
 *  (NONE) : Bang-bang heating
 * PIDTEMP : PID temperature control (~4.1K)
 * MPCTEMP : Predictive Model temperature control. (~1.8K without auto-tune)
 */
#define PIDTEMP // See the PID Tuning Guide at

#define PID_MAX                                                                \
  255 // Limit hotend current while PID is active (see PID_FUNCTIONAL_RANGE
#define PID_K1 0.95 // Smoothing factor within any PID loop

#if ENABLED(PIDTEMP)

#if ENABLED(PID_PARAMS_PER_HOTEND)
#define DEFAULT_KP_LIST {22.20, 22.20}
#define DEFAULT_KI_LIST {1.08, 1.08}
#define DEFAULT_KD_LIST {114.00, 114.00}
#else
#define DEFAULT_KP 22.20
#define DEFAULT_KI 1.08
#define DEFAULT_KD 114.00
#endif
#else
#define BANG_MAX                                                               \
  255 // Limit hotend current while in bang-bang mode; 255=full current
#endif

/**
 * Model Predictive Control for hotend
 *
 * Use a physical model of the hotend to control temperature. When configured
 * correctly this gives better responsiveness and stability than PID and removes
 * the need for PID_EXTRUSION_SCALING and PID_FAN_SCALING. Enable MPC_AUTOTUNE
 * and use M306 T to autotune the model.
 * @section mpc temp
 */
#if ENABLED(MPCTEMP)
#define MPC_AUTOTUNE // Include a method to do MPC auto-tuning (~6.3K bytes of

#define MPC_MAX 255 // (0..255) Current to nozzle while MPC is active.
#define MPC_HEATER_POWER {40.0f} // (W) Nominal heat cartridge powers.
#if ENABLED(MPC_PTC)
#define MPC_HEATER_ALPHA                                                       \
  {0.0028f} // Temperature coefficient of resistance of the heat cartridges.
#define MPC_HEATER_REFTEMP                                                     \
  {20} // (°C) Reference temperature for MPC_HEATER_POWER and MPC_HEATER_ALPHA.
#endif

#define MPC_INCLUDE_FAN // Model the fan speed?

#define MPC_BLOCK_HEAT_CAPACITY {16.7f} // (J/K) Heat block heat capacities.
#define MPC_SENSOR_RESPONSIVENESS                                              \
  {0.22f} // (K/s per ∆K) Rate of change of sensor temperature from heat block.
#define MPC_AMBIENT_XFER_COEFF                                                 \
  {0.068f} // (W/K) Heat transfer coefficients from heat block to room air with
#if ENABLED(MPC_INCLUDE_FAN)
#define MPC_AMBIENT_XFER_COEFF_FAN255                                          \
  {0.097f} // (W/K) Heat transfer coefficients from heat block to room air with
#endif

#define FILAMENT_HEAT_CAPACITY_PERMM                                           \
  {5.6e-3f} // 0.0056 J/K/mm for 1.75mm PLA (0.0149 J/K/mm for 2.85mm PLA).

#define MPC_SMOOTHING_FACTOR                                                   \
  0.5f // (0.0...1.0) Noisy temperature sensors may need a lower value for
#define MPC_MIN_AMBIENT_CHANGE                                                 \
  1.0f // (K/s) Modeled ambient temperature rate of change, when correcting
#define MPC_STEADYSTATE                                                        \
  0.5f // (K/s) Temperature change rate for steady state logic to be enforced.

#define MPC_TUNING_POS                                                         \
  {X_CENTER, Y_CENTER, 1.0f}                                                   \
  // (mm) M306 Autotuning position, ideally bed center at first layer height.
#define MPC_TUNING_END_Z 10.0f // (mm) M306 Autotuning final Z position.
#endif

//===========================================================================
//====================== PID > Bed Temperature Control ======================
//===========================================================================

// @section bed temp

/**
 * Max Bed Power
 * Applies to all forms of bed control (PID, bang-bang, and bang-bang with
 * hysteresis). When set to any value below 255, enables a form of PWM to the
 * bed that acts like a divider so don't use it unless you are OK with PWM on
 * your bed. (See the comment on enabling PIDTEMPBED)
 */
#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current

#if ENABLED(PIDTEMPBED)

#define DEFAULT_BED_KP 10.00
#define DEFAULT_BED_KI 0.023
#define DEFAULT_BED_KD 305.4

#else
#endif

#if ENABLED(PELTIER_BED)
#define PELTIER_DIR_PIN -1 // Relay control pin for Peltier
#define PELTIER_DIR_HEAT_STATE                                                 \
  LOW // The relay pin state that causes the Peltier to heat
#endif

//===========================================================================
//==================== PID > Chamber Temperature Control ====================
//===========================================================================

/**
 * Max Chamber Power
 * Applies to all forms of chamber control (PID, bang-bang, and bang-bang with
 * hysteresis). When set to any value below 255, enables a form of PWM to the
 * chamber heater that acts like a divider so don't use it unless you are OK
 * with PWM on your heater. (See the comment on enabling PIDTEMPCHAMBER)
 */
#define MAX_CHAMBER_POWER                                                      \
  255 // limits duty cycle to chamber heater; 255=full current

#if ENABLED(PIDTEMPCHAMBER)

#define DEFAULT_CHAMBER_KP 37.04
#define DEFAULT_CHAMBER_KI 1.40
#define DEFAULT_CHAMBER_KD 655.17

#endif // PIDTEMPCHAMBER

// @section pid temp

#if ANY(PIDTEMP, PIDTEMPBED, PIDTEMPCHAMBER)
#define PID_FUNCTIONAL_RANGE                                                   \
  20 // If the temperature difference between the target temperature and the

#endif

// @section safety

/**
 * Prevent extrusion if the temperature is below EXTRUDE_MINTEMP.
 * Add M302 to set the minimum extrusion temperature and/or turn
 * cold extrusion prevention on and off.
 *
 * *** IT IS HIGHLY RECOMMENDED TO LEAVE THIS OPTION ENABLED! ***
 */
#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 170

/**
 * Prevent a single extrusion longer than EXTRUDE_MAXLENGTH.
 * Note: For Bowden Extruders make this large enough to allow load/unload.
 */
#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH 200

//===========================================================================
//======================== Thermal Runaway Protection =======================
//===========================================================================

/**
 * Thermal Protection provides additional protection to your printer from damage
 * and fire. Marlin always includes safe min and max temperature ranges which
 * protect against a broken or disconnected thermistor wire.
 *
 * The issue: If a thermistor falls out, it will report the much lower
 * temperature of the air in the room, and the firmware will keep
 * the heater on.
 *
 * If you get "Thermal Runaway" or "Heating failed" errors the
 * details can be tuned in Configuration_adv.h
 */

#define THERMAL_PROTECTION_HOTENDS // Enable thermal protection for all
#define THERMAL_PROTECTION_BED // Enable thermal protection for the heated bed
#define THERMAL_PROTECTION_CHAMBER // Enable thermal protection for the heated
#define THERMAL_PROTECTION_COOLER  // Enable thermal protection for the laser

//===========================================================================
//============================= Mechanical Settings =========================
//===========================================================================

// @section kinematics

// @section polargraph

#if ENABLED(POLARGRAPH)
#define POLARGRAPH_MAX_BELT_LEN                                                \
  1035.0 // (mm) Belt length at full extension. Override with M665 H.
#define DEFAULT_SEGMENTS_PER_SECOND 5 // Move segmentation based on duration
#define PEN_UP_DOWN_MENU // Add "Pen Up" and "Pen Down" to the MarlinUI menu
#endif

// @section delta

#if ENABLED(DELTA)

#define DEFAULT_SEGMENTS_PER_SECOND 200

#if ENABLED(DELTA_AUTO_CALIBRATION)
#define DELTA_CALIBRATION_DEFAULT_POINTS 4
#endif

#if ANY(DELTA_AUTO_CALIBRATION, DELTA_CALIBRATION_MENU)
#define PROBE_MANUALLY_STEP 0.05 // (mm)
#endif

#define PRINTABLE_RADIUS 140.0 // (mm)

#define DELTA_DIAGONAL_ROD 250.0 // (mm)

#define DELTA_HEIGHT 250.00 // (mm) Get this value from G33 auto calibrate

#define DELTA_ENDSTOP_ADJ                                                      \
  {0.0, 0.0, 0.0} // (mm) Get these values from G33 auto calibrate

#define DELTA_RADIUS 124.0 // (mm) Get this value from G33 auto calibrate

#define DELTA_TOWER_ANGLE_TRIM                                                 \
  {0.0, 0.0, 0.0} // (mm) Get these values from G33 auto calibrate

#endif // DELTA

// @section scara

#if ANY(MORGAN_SCARA, MP_SCARA)
#define DEFAULT_SEGMENTS_PER_SECOND 200

#define SCARA_LINKAGE_1 150 // (mm)
#define SCARA_LINKAGE_2 150 // (mm)

#define SCARA_OFFSET_X 100 // (mm)
#define SCARA_OFFSET_Y -56 // (mm)

#if ENABLED(MORGAN_SCARA)

#define FEEDRATE_SCALING // Convert XY feedrate from mm/s to degrees/s on the

#define MIDDLE_DEAD_ZONE_R 0 // (mm)

#elif ENABLED(MP_SCARA)

#define SCARA_OFFSET_THETA1 12  // degrees
#define SCARA_OFFSET_THETA2 131 // degrees

#endif

#endif

// @section tpara

#if ENABLED(AXEL_TPARA)
#define DEBUG_TPARA_KINEMATICS
#define DEFAULT_SEGMENTS_PER_SECOND 200

#define TPARA_LINKAGE_1 120 // (mm)
#define TPARA_LINKAGE_2 120 // (mm)

#define TPARA_SHOULDER_AXIS_HEIGHT 135.0 // (mm)

#define TPARA_ARM_X_HOME_POS                                                   \
  28.75 // (mm) Measured from shoulder axis to tool holder axis in home position
#define TPARA_ARM_Y_HOME_POS 0 // (mm)
#define TPARA_ARM_Z_HOME_POS                                                   \
  250.00 // (mm) Measured from tool holder axis to the floor

#define TPARA_OFFSET_X                                                         \
  127.0 // (mm) to coincide with minimum radius MIDDLE_DEAD_ZONE_R, and W(0,0,0)
#define TPARA_OFFSET_Y 0.0 // (mm)
#define TPARA_OFFSET_Z 0.0 // (mm)

#define TPARA_TCP_OFFSET_X                                                     \
  27.0 // (mm) Tool flange: 27 (distance from pivot to bolt holes), extruder
#define TPARA_TCP_OFFSET_Y 0.0 // (mm)
#define TPARA_TCP_OFFSET_Z                                                     \
  -65.0 // (mm) Tool flange (bottom): -6 (caution as Z 0 posiion will crash

#define FEEDRATE_SCALING // Convert XY feedrate from mm/s to degrees/s on the

#define MIDDLE_DEAD_ZONE_R 100 // (mm)

#define TPARA_MAX_L1L2_ANGLE 140.0f // (degrees)
#endif                              // AXEL_TPARA

// @section polar

#if ENABLED(POLAR)
#define DEFAULT_SEGMENTS_PER_SECOND                                            \
  180                          // If movement is choppy try lowering this value
#define PRINTABLE_RADIUS 82.0f // (mm) Maximum travel of X axis

#define POLAR_FAST_RADIUS 3.0f // (mm)

#define POLAR_CENTER_OFFSET 0.0f // (mm)

#define FEEDRATE_SCALING // Convert XY feedrate from mm/s to degrees/s on the
#endif

//===========================================================================
//============================== Endstop Settings ===========================
//===========================================================================

// @section endstops

#define ENDSTOPPULLUPS

/**
 * Endstop "Hit" State
 * Set to the state (HIGH or LOW) that applies to each endstop.
 */
#define X_MIN_ENDSTOP_HIT_STATE HIGH
#define X_MAX_ENDSTOP_HIT_STATE HIGH
#define Y_MIN_ENDSTOP_HIT_STATE HIGH
#define Y_MAX_ENDSTOP_HIT_STATE HIGH
#define Z_MIN_ENDSTOP_HIT_STATE HIGH
#define Z_MAX_ENDSTOP_HIT_STATE HIGH
#define I_MIN_ENDSTOP_HIT_STATE HIGH
#define I_MAX_ENDSTOP_HIT_STATE HIGH
#define J_MIN_ENDSTOP_HIT_STATE HIGH
#define J_MAX_ENDSTOP_HIT_STATE HIGH
#define K_MIN_ENDSTOP_HIT_STATE HIGH
#define K_MAX_ENDSTOP_HIT_STATE HIGH
#define U_MIN_ENDSTOP_HIT_STATE HIGH
#define U_MAX_ENDSTOP_HIT_STATE HIGH
#define V_MIN_ENDSTOP_HIT_STATE HIGH
#define V_MAX_ENDSTOP_HIT_STATE HIGH
#define W_MIN_ENDSTOP_HIT_STATE HIGH
#define W_MAX_ENDSTOP_HIT_STATE HIGH
#define Z_MIN_PROBE_ENDSTOP_HIT_STATE HIGH

//=============================================================================
//============================== Movement Settings ============================
//=============================================================================
// @section motion

/**
 * Default Axis Steps Per Unit (linear=steps/mm, rotational=steps/°)
 * Override with M92 (when enabled below)
 *                                      X, Y, Z [, I [, J [, K...]]], E0 [, E1[,
 * E2...]]
 */
// Overit treti hodnotu - mela by byt 1600, ale nechce se ji
#define DEFAULT_AXIS_STEPS_PER_UNIT {100, 100, 100}

/**
 * Enable support for M92. Disable to save at least ~530 bytes of flash.
 */
#define EDITABLE_STEPS_PER_UNIT

/**
 * Default Max Feed Rate (linear=mm/s, rotational=°/s)
 * Override with M203
 *                                      X, Y, Z [, I [, J [, K...]]], E0 [, E1[,
 * E2...]]
 */
#define DEFAULT_MAX_FEEDRATE {300, 300, 5}

#if ENABLED(LIMITED_MAX_FR_EDITING)
#define MAX_FEEDRATE_EDIT_VALUES                                               \
  {600, 600, 10, 50} // ...or, set your own edit limits
#endif

/**
 * Default Max Acceleration (speed change with time) (linear=mm/(s^2),
 * rotational=°/(s^2)) (Maximum start speed for accelerated moves) Override with
 * M201 X, Y, Z [, I [, J [, K...]]], E0 [, E1[, E2...]]
 */
#define DEFAULT_MAX_ACCELERATION {3000, 3000, 100}

#if ENABLED(LIMITED_MAX_ACCEL_EDITING)
#define MAX_ACCEL_EDIT_VALUES                                                  \
  {6000, 6000, 200, 20000} // ...or, set your own edit limits
#endif

/**
 * Default Acceleration (speed change with time) (linear=mm/(s^2),
 * rotational=°/(s^2)) Override with M204
 *
 *   M204 P    Acceleration
 *   M204 R    Retract Acceleration
 *   M204 T    Travel Acceleration
 */
#define DEFAULT_ACCELERATION                                                   \
  3000 // X, Y, Z and E acceleration for printing moves
#define DEFAULT_RETRACT_ACCELERATION 3000 // E acceleration for retracts
#define DEFAULT_TRAVEL_ACCELERATION                                            \
  3000 // X, Y, Z acceleration for travel (non printing) moves

#if ENABLED(CLASSIC_JERK)
#define DEFAULT_XJERK 10.0
#define DEFAULT_YJERK 10.0
#define DEFAULT_ZJERK 0.3
#define DEFAULT_EJERK 5.0

#if ENABLED(LIMITED_JERK_EDITING)
#define MAX_JERK_EDIT_VALUES                                                   \
  {20, 20, 0.6, 10} // ...or, set your own edit limits
#endif
#endif

/**
 * Junction Deviation Factor
 *
 * See:
 *   https://reprap.org/forum/read.php?1,739819
 *   https://blog.kyneticcnc.com/2018/10/computing-junction-deviation-for-marlin.html
 */
#if DISABLED(CLASSIC_JERK)
#define JUNCTION_DEVIATION_MM 0.013 // (mm) Distance from real junction edge
#define JD_HANDLE_SMALL_SEGMENTS // Use curvature estimation instead of just the
#endif

//===========================================================================
//============================= Z Probe Options =============================
//===========================================================================
// @section probes

/**
 * Enable this option for a probe connected to the Z-MIN pin.
 * The probe replaces the Z-MIN endstop and is used for Z homing.
 * (Automatically enables USE_PROBE_FOR_Z_HOMING.)
 */
#define Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN

#if ENABLED(MAGLEV4)
#define MAGLEV_TRIGGER_DELAY 15 // Changing this risks overheating the coil
#endif

#if ENABLED(TOUCH_MI_PROBE)
#define TOUCH_MI_RETRACT_Z 0.5 // Height at which the probe retracts
#endif

#if ENABLED(RACK_AND_PINION_PROBE)
#define Z_PROBE_DEPLOY_X X_MIN_POS
#define Z_PROBE_RETRACT_X X_MAX_POS
#endif

#if ENABLED(MAG_MOUNTED_PROBE)
#define PROBE_DEPLOY_FEEDRATE (133 * 60) // (mm/min) Probe deploy speed
#define PROBE_STOW_FEEDRATE (133 * 60)   // (mm/min) Probe stow speed

#ifdef MAG_MOUNTED_PROBE_SERVO_NR
#define MAG_MOUNTED_PROBE_SERVO_ANGLES                                         \
  {90, 0} // Servo Angles for Deployed, Stowed
#define MAG_MOUNTED_PRE_DEPLOY                                                 \
  {PROBE_DEPLOY_FEEDRATE, {15, 160, 30}} // Safe position for servo activation
#define MAG_MOUNTED_PRE_STOW                                                   \
  {PROBE_DEPLOY_FEEDRATE, {15, 160, 30}} // Safe position for servo deactivation
#endif

#define MAG_MOUNTED_DEPLOY_1                                                   \
  {PROBE_DEPLOY_FEEDRATE, {245, 114, 30}} // Move to side Dock & Attach probe
#define MAG_MOUNTED_DEPLOY_2                                                   \
  {PROBE_DEPLOY_FEEDRATE, {210, 114, 30}} // Move probe off dock
#define MAG_MOUNTED_DEPLOY_3                                                   \
  {PROBE_DEPLOY_FEEDRATE, {0, 0, 0}} // Extra move if needed
#define MAG_MOUNTED_DEPLOY_4                                                   \
  {PROBE_DEPLOY_FEEDRATE, {0, 0, 0}} // Extra move if needed
#define MAG_MOUNTED_DEPLOY_5                                                   \
  {PROBE_DEPLOY_FEEDRATE, {0, 0, 0}} // Extra move if needed
#define MAG_MOUNTED_STOW_1 {PROBE_STOW_FEEDRATE, {245, 114, 20}} // Move to dock
#define MAG_MOUNTED_STOW_2                                                     \
  {PROBE_STOW_FEEDRATE, {245, 114, 0}} // Place probe beside remover
#define MAG_MOUNTED_STOW_3                                                     \
  {PROBE_STOW_FEEDRATE, {230, 114, 0}} // Side move to remove probe
#define MAG_MOUNTED_STOW_4                                                     \
  {PROBE_STOW_FEEDRATE, {210, 114, 20}} // Side move to remove probe
#define MAG_MOUNTED_STOW_5                                                     \
  {PROBE_STOW_FEEDRATE, {0, 0, 0}} // Extra move if needed
#endif

#if ENABLED(DUET_SMART_EFFECTOR)
#define SMART_EFFECTOR_MOD_PIN                                                 \
  -1 // Connect a GPIO pin to the Smart Effector MOD pin
#endif

#if ENABLED(Z_PROBE_ALLEN_KEY)

#define Z_PROBE_ALLEN_KEY_DEPLOY_1 {30.0, PRINTABLE_RADIUS, 100.0}
#define Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE XY_PROBE_FEEDRATE

#define Z_PROBE_ALLEN_KEY_DEPLOY_2 {0.0, PRINTABLE_RADIUS, 100.0}
#define Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE (XY_PROBE_FEEDRATE) / 10

#define Z_PROBE_ALLEN_KEY_DEPLOY_3 {0.0, (PRINTABLE_RADIUS) * 0.75, 100.0}
#define Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE XY_PROBE_FEEDRATE

#define Z_PROBE_ALLEN_KEY_STOW_1                                               \
  {-64.0, 56.0, 23.0} // Move the probe into position
#define Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE XY_PROBE_FEEDRATE

#define Z_PROBE_ALLEN_KEY_STOW_2 {-64.0, 56.0, 3.0} // Push it down
#define Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE (XY_PROBE_FEEDRATE) / 10

#define Z_PROBE_ALLEN_KEY_STOW_3 {-64.0, 56.0, 50.0} // Move it up to clear
#define Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE XY_PROBE_FEEDRATE

#define Z_PROBE_ALLEN_KEY_STOW_4 {0.0, 0.0, 50.0}
#define Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE XY_PROBE_FEEDRATE

#endif // Z_PROBE_ALLEN_KEY

/**
 * Nozzle-to-Probe offsets { X, Y, Z }
 *
 * X and Y offset
 *   Use a caliper or ruler to measure the distance (in mm) from the tip of
 *   the Nozzle to the center-point of the Probe in the X and Y axes.
 *
 * Z offset
 * - For the Z offset use your best known value and adjust at runtime.
 * - Common probes trigger below the nozzle and have negative values for Z
 * offset.
 * - Probes triggering above the nozzle height are uncommon but do exist. When
 * using probes such as this, carefully set Z_CLEARANCE_DEPLOY_PROBE and
 * Z_CLEARANCE_BETWEEN_PROBES to avoid collisions during probing.
 *
 * Tune and Adjust
 * -  Probe Offsets can be tuned at runtime with 'M851', LCD menus,
 * babystepping, etc.
 * -  PROBE_OFFSET_WIZARD (Configuration_adv.h) can be used for setting the Z
 * offset.
 *
 * Assuming the typical work area orientation:
 *  - Probe to RIGHT of the Nozzle has a Positive X offset
 *  - Probe to LEFT  of the Nozzle has a Negative X offset
 *  - Probe in BACK  of the Nozzle has a Positive Y offset
 *  - Probe in FRONT of the Nozzle has a Negative Y offset
 *
 * Some examples:
 *   #define NOZZLE_TO_PROBE_OFFSET { 10, 10, -1 }   // Example "1"
 *   #define NOZZLE_TO_PROBE_OFFSET {-10,  5, -1 }   // Example "2"
 *   #define NOZZLE_TO_PROBE_OFFSET {  5, -5, -1 }   // Example "3"
 *   #define NOZZLE_TO_PROBE_OFFSET {-15,-10, -1 }   // Example "4"
 *
 *     +-- BACK ---+
 *     |    [+]    |
 *   L |        1  | R <-- Example "1" (right+,  back+)
 *   E |  2        | I <-- Example "2" ( left-,  back+)
 *   F |[-]  N  [+]| G <-- Nozzle
 *   T |       3   | H <-- Example "3" (right+, front-)
 *     | 4         | T <-- Example "4" ( left-, front-)
 *     |    [-]    |
 *     O-- FRONT --+
 */
#define NOZZLE_TO_PROBE_OFFSET                                                 \
  {10, 10, 0} // (mm) X, Y, Z distance from Nozzle tip to Probe trigger-point

#define PROBING_TOOL 0

#define PROBING_MARGIN 10

#define XY_PROBE_FEEDRATE (133 * 60) // (mm/min)

#define Z_PROBE_FEEDRATE_FAST (4 * 60) // (mm/min)

#define Z_PROBE_FEEDRATE_SLOW (Z_PROBE_FEEDRATE_FAST / 2) // (mm/min)

#if ENABLED(PROBE_ACTIVATION_SWITCH)
#define PROBE_ACTIVATION_SWITCH_STATE LOW // State indicating probe is active
#endif

#if ENABLED(PROBE_TARE)
#define PROBE_TARE_TIME 200   // (ms) Time to hold tare pin
#define PROBE_TARE_DELAY 200  // (ms) Delay after tare before
#define PROBE_TARE_STATE HIGH // State to write pin for tare
#endif

/**
 * Z probes require clearance when deploying, stowing, and moving between
 * probe points to avoid hitting the bed and other hardware.
 * Servo-mounted probes require extra space for the arm to rotate.
 * Inductive probes need space to keep from triggering early.
 *
 * Use these settings to specify the distance (mm) to raise the probe (or
 * lower the bed). The values set here apply over and above any (negative)
 * probe Z Offset set with NOZZLE_TO_PROBE_OFFSET, M851, or the LCD.
 * Only integer values >= 1 are valid here.
 *
 * Example: 'M851 Z-5' with a CLEARANCE of 4  =>  9mm from bed to nozzle.
 *     But: 'M851 Z+1' with a CLEARANCE of 2  =>  2mm from bed to nozzle.
 */
#define Z_CLEARANCE_DEPLOY_PROBE 10  // (mm) Z Clearance for Deploy/Stow
#define Z_CLEARANCE_BETWEEN_PROBES 5 // (mm) Z Clearance between probe points
#define Z_CLEARANCE_MULTI_PROBE 5    // (mm) Z Clearance between multiple probes
#define Z_PROBE_ERROR_TOLERANCE                                                \
  3 // (mm) Tolerance for early trigger (<= -probe.offset.z + ZPET)

#define Z_PROBE_LOW_POINT                                                      \
  -2 // (mm) Farthest distance below the trigger-point to go before stopping

#if ENABLED(PREHEAT_BEFORE_PROBING)
#define PROBING_NOZZLE_TEMP 120 // (°C) Only applies to E0 at this time
#define PROBING_BED_TEMP 50
#endif

// @section stepper drivers

#define X_ENABLE_ON LOW
#define Y_ENABLE_ON LOW
#define Z_ENABLE_ON LOW
#define E_ENABLE_ON LOW // For all extruders

// @section extruder

#define DISABLE_OTHER_EXTRUDERS // Keep only the active extruder enabled

// @section stepper drivers

#define INVERT_X_DIR false
#define INVERT_Y_DIR true
#define INVERT_Z_DIR false

#define INVERT_E0_DIR false
#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false
#define INVERT_E4_DIR false
#define INVERT_E5_DIR false
#define INVERT_E6_DIR false
#define INVERT_E7_DIR false

// @section homing

#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

// @section geometry

#define X_BED_SIZE 200
#define Y_BED_SIZE 200

#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0
#define X_MAX_POS X_BED_SIZE
#define Y_MAX_POS Y_BED_SIZE
#define Z_MAX_POS 200

#define MIN_SOFTWARE_ENDSTOPS
#if ENABLED(MIN_SOFTWARE_ENDSTOPS)
#define MIN_SOFTWARE_ENDSTOP_X
#define MIN_SOFTWARE_ENDSTOP_Y
#define MIN_SOFTWARE_ENDSTOP_Z
#define MIN_SOFTWARE_ENDSTOP_I
#define MIN_SOFTWARE_ENDSTOP_J
#define MIN_SOFTWARE_ENDSTOP_K
#define MIN_SOFTWARE_ENDSTOP_U
#define MIN_SOFTWARE_ENDSTOP_V
#define MIN_SOFTWARE_ENDSTOP_W
#endif

#define MAX_SOFTWARE_ENDSTOPS
#if ENABLED(MAX_SOFTWARE_ENDSTOPS)
#define MAX_SOFTWARE_ENDSTOP_X
#define MAX_SOFTWARE_ENDSTOP_Y
#define MAX_SOFTWARE_ENDSTOP_Z
#define MAX_SOFTWARE_ENDSTOP_I
#define MAX_SOFTWARE_ENDSTOP_J
#define MAX_SOFTWARE_ENDSTOP_K
#define MAX_SOFTWARE_ENDSTOP_U
#define MAX_SOFTWARE_ENDSTOP_V
#define MAX_SOFTWARE_ENDSTOP_W
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
#define FIL_RUNOUT_ENABLED_DEFAULT                                             \
  true // Enable the sensor on startup. Override with M412 followed by M500.
#define NUM_RUNOUT_SENSORS                                                     \
  1 // Number of sensors, up to one per extruder. Define a FIL_RUNOUT#_PIN for

#define FIL_RUNOUT_STATE                                                       \
  LOW                     // Pin state indicating that filament is NOT present.
#define FIL_RUNOUT_PULLUP // Use internal pullup for filament runout pins.

#define FILAMENT_RUNOUT_SCRIPT "M600"

#ifdef FILAMENT_RUNOUT_DISTANCE_MM

#if ENABLED(FILAMENT_MOTION_SENSOR)
#if ENABLED(FILAMENT_SWITCH_AND_MOTION)

#define FILAMENT_MOTION_DISTANCE_MM                                            \
  3.0 // (mm) Missing distance required to trigger runout

#define NUM_MOTION_SENSORS                                                     \
  1 // Number of sensors, up to one per extruder. Define a FIL_MOTION#_PIN for

#endif // FILAMENT_SWITCH_AND_MOTION
#endif // FILAMENT_MOTION_SENSOR
#endif // FILAMENT_RUNOUT_DISTANCE_MM
#endif // FILAMENT_RUNOUT_SENSOR

//===========================================================================
//=============================== Bed Leveling ==============================
//===========================================================================
// @section calibrate

#if ENABLED(PREHEAT_BEFORE_LEVELING)
#define LEVELING_NOZZLE_TEMP 120 // (°C) Only applies to E0 at this time
#define LEVELING_BED_TEMP 50
#endif

#if ANY(MESH_BED_LEVELING, AUTO_BED_LEVELING_UBL, PROBE_MANUALLY)
#define MANUAL_PROBE_START_Z                                                   \
  0.2 // (mm) Comment out to use the last-measured height
#endif

#if ANY(MESH_BED_LEVELING, AUTO_BED_LEVELING_BILINEAR, AUTO_BED_LEVELING_UBL)
/**
 * Gradually reduce leveling correction until a set height is reached,
 * at which point movement will be level to the machine's XY plane.
 * The height can be set with M420 Z<height>
 */
#define ENABLE_LEVELING_FADE_HEIGHT
#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
#define DEFAULT_LEVELING_FADE_HEIGHT 10.0 // (mm) Default fade height.
#endif

/**
 * For Cartesian machines, instead of dividing moves on mesh boundaries,
 * split up moves into short segments like a Delta. This follows the
 * contours of the bed more closely than edge-to-edge straight moves.
 */
#define SEGMENT_LEVELED_MOVES
#define LEVELED_SEGMENT_LENGTH                                                 \
  5.0 // (mm) Length of all segments (except the last one)

#if ENABLED(G26_MESH_VALIDATION)
#define MESH_TEST_NOZZLE_SIZE 0.4  // (mm) Diameter of primary nozzle.
#define MESH_TEST_LAYER_HEIGHT 0.2 // (mm) Default layer height for G26.
#define MESH_TEST_HOTEND_TEMP 205  // (°C) Default nozzle temperature for G26.
#define MESH_TEST_BED_TEMP 60      // (°C) Default bed temperature for G26.
#define G26_XY_FEEDRATE 20         // (mm/s) Feedrate for G26 XY moves.
#define G26_XY_FEEDRATE_TRAVEL 100 // (mm/s) Feedrate for G26 XY travel moves.
#define G26_RETRACT_MULTIPLIER                                                 \
  1.0 // G26 Q (retraction) used by default between mesh test elements.
#endif

#endif

#if ANY(AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_BILINEAR)

#define GRID_MAX_POINTS_X 3
#define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)

#if ENABLED(ABL_BILINEAR_SUBDIVISION)
#define BILINEAR_SUBDIVISIONS 3
#endif

#endif

#elif ENABLED(AUTO_BED_LEVELING_UBL)

//===========================================================================
//========================= Unified Bed Leveling ============================
//===========================================================================

#define MESH_INSET 1 // Set Mesh bounds as an inset region of the bed
#define GRID_MAX_POINTS_X                                                      \
  10 // Don't use more than 15 points per axis, implementation limited.
#define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X

#define UBL_MESH_EDIT_MOVES_Z   // Sophisticated users prefer no movement of
#define UBL_SAVE_ACTIVE_ON_M500 // Save the currently active mesh in the current

#if ENABLED(AVOID_OBSTACLES)
#define CLIP_W                                                                 \
  23 // Bed clip width, should be padded a few mm over its physical size
#define CLIP_H                                                                 \
  14 // Bed clip height, should be padded a few mm over its physical size

#define OBSTACLE1                                                              \
  {(X_BED_SIZE) / 4 - (CLIP_W) / 2, 0, (X_BED_SIZE) / 4 + (CLIP_W) / 2, CLIP_H}
#define OBSTACLE2                                                              \
  {(X_BED_SIZE) * 3 / 4 - (CLIP_W) / 2, 0,                                     \
   (X_BED_SIZE) * 3 / 4 + (CLIP_W) / 2, CLIP_H}
#define OBSTACLE3                                                              \
  {(X_BED_SIZE) / 4 - (CLIP_W) / 2, (Y_BED_SIZE) - (CLIP_H),                   \
   (X_BED_SIZE) / 4 + (CLIP_W) / 2, Y_BED_SIZE}
#define OBSTACLE4                                                              \
  {(X_BED_SIZE) * 3 / 4 - (CLIP_W) / 2, (Y_BED_SIZE) - (CLIP_H),               \
   (X_BED_SIZE) * 3 / 4 + (CLIP_W) / 2, Y_BED_SIZE}

#define G29J_MESH_TILT_MARGIN ((CLIP_H) + 1)
#endif

#elif ENABLED(MESH_BED_LEVELING)

//===========================================================================
//=================================== Mesh ==================================
//===========================================================================

#define MESH_INSET 10 // Set Mesh bounds as an inset region of the bed
#define GRID_MAX_POINTS_X 3
#define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X

#endif // BED_LEVELING

#if ENABLED(LCD_BED_LEVELING)
#define MESH_EDIT_Z_STEP 0.025 // (mm) Step size while manually probing Z axis.
#define LCD_PROBE_Z_RANGE                                                      \
  4 // (mm) Z Range centered on Z_MIN_POS for LCD Z adjustment
#endif

#if ENABLED(LCD_BED_TRAMMING)
#define BED_TRAMMING_INSET_LFRB                                                \
  {30, 30, 30, 30}              // (mm) Left, Front, Right, Back insets
#define BED_TRAMMING_HEIGHT 0.0 // (mm) Z height of nozzle at tramming points
#define BED_TRAMMING_Z_HOP 4.0  // (mm) Z raise between tramming points
#if ENABLED(BED_TRAMMING_USE_PROBE)
#define BED_TRAMMING_PROBE_TOLERANCE 0.1 // (mm)
#define BED_TRAMMING_VERIFY_RAISED       // After adjustment triggers the probe,
#endif

/**
 * Corner Leveling Order
 *
 * Set 2 or 4 points. When 2 points are given, the 3rd is the center of the
 * opposite edge.
 *
 *  LF  Left-Front    RF  Right-Front
 *  LB  Left-Back     RB  Right-Back
 *
 * Examples:
 *
 *      Default        {LF,RB,LB,RF}         {LF,RF}           {LB,LF}
 *  LB --------- RB   LB --------- RB    LB --------- RB   LB --------- RB
 *  |  4       3  |   | 3         2 |    |     <3>     |   | 1           |
 *  |             |   |             |    |             |   |          <3>|
 *  |  1       2  |   | 1         4 |    | 1         2 |   | 2           |
 *  LF --------- RF   LF --------- RF    LF --------- RF   LF --------- RF
 */
#define BED_TRAMMING_LEVELING_ORDER {LF, RF, RB, LB}
#endif

// @section homing

#if ENABLED(Z_SAFE_HOMING)
#define Z_SAFE_HOMING_X_POINT X_CENTER // (mm) X point for Z homing
#define Z_SAFE_HOMING_Y_POINT Y_CENTER // (mm) Y point for Z homing
#endif

#define HOMING_FEEDRATE_MM_M {(50 * 60), (50 * 60), (4 * 60)}

#define VALIDATE_HOMING_ENDSTOPS

// @section calibrate

#if ENABLED(SKEW_CORRECTION)
#define XY_DIAG_AC 282.8427124746
#define XY_DIAG_BD 282.8427124746
#define XY_SIDE_AD 200

#if ENABLED(SKEW_CORRECTION_FOR_Z)
#define XZ_DIAG_AC 282.8427124746
#define XZ_DIAG_BD 282.8427124746
#define YZ_DIAG_AC 282.8427124746
#define YZ_DIAG_BD 282.8427124746
#define YZ_SIDE_AD 200

#endif

#endif

//=============================================================================
//============================= Additional Features ===========================
//=============================================================================

// @section eeprom

#define EEPROM_CHITCHAT    // Give feedback on EEPROM commands. Disable to save
#define EEPROM_BOOT_SILENT // Keep M503 quiet and only give errors during first

// @section host

#define HOST_KEEPALIVE_FEATURE // Disable this if your host doesn't like
#define DEFAULT_KEEPALIVE_INTERVAL                                             \
  2 // Number of seconds between "busy" messages. Set with M113.
#define BUSY_WHILE_HEATING // Some hosts require "busy" messages even during

// @section units

// @section temperature presets

#define PREHEAT_1_LABEL "PLA"
#define PREHEAT_1_TEMP_HOTEND 180
#define PREHEAT_1_TEMP_BED 70
#define PREHEAT_1_TEMP_CHAMBER 35
#define PREHEAT_1_FAN_SPEED 0 // Value from 0 to 255

#define PREHEAT_2_LABEL "ABS"
#define PREHEAT_2_TEMP_HOTEND 240
#define PREHEAT_2_TEMP_BED 110
#define PREHEAT_2_TEMP_CHAMBER 35
#define PREHEAT_2_FAN_SPEED 0 // Value from 0 to 255

#if ENABLED(NOZZLE_PARK_FEATURE)
#define NOZZLE_PARK_POINT {(X_MIN_POS + 10), (Y_MAX_POS - 10), 20}
#define NOZZLE_PARK_MOVE                                                       \
  0 // Park motion: 0 = XY Move, 1 = X Only, 2 = Y Only, 3 = X before Y, 4 = Y
#define NOZZLE_PARK_Z_RAISE_MIN                                                \
  2 // (mm) Always raise Z by at least this distance
#define NOZZLE_PARK_XY_FEEDRATE                                                \
  100 // (mm/s) X and Y axes feedrate (also used for delta Z axis)
#define NOZZLE_PARK_Z_FEEDRATE                                                 \
  5 // (mm/s) Z axis feedrate (not used for delta printers)
#endif

#if ENABLED(NOZZLE_CLEAN_FEATURE)
#define NOZZLE_CLEAN_PATTERN_LINE // Provide 'G12 P0' - a simple linear cleaning
#define NOZZLE_CLEAN_PATTERN_ZIGZAG // Provide 'G12 P1' - a zigzag cleaning
#define NOZZLE_CLEAN_PATTERN_CIRCLE // Provide 'G12 P2' - a circular cleaning

#define NOZZLE_CLEAN_DEFAULT_PATTERN 0

#define NOZZLE_CLEAN_STROKES 12 // Default number of pattern repetitions

#if ENABLED(NOZZLE_CLEAN_PATTERN_ZIGZAG)
#define NOZZLE_CLEAN_TRIANGLES 3 // Default number of triangles
#endif

#define NOZZLE_CLEAN_START_POINT                                               \
  {                                                                            \
    {                                                                          \
      30, 30, (Z_MIN_POS + 1)                                                  \
    }                                                                          \
  }
#define NOZZLE_CLEAN_END_POINT                                                 \
  {                                                                            \
    {                                                                          \
      100, 60, (Z_MIN_POS + 1)                                                 \
    }                                                                          \
  }

#if ENABLED(NOZZLE_CLEAN_PATTERN_CIRCLE)
#define NOZZLE_CLEAN_CIRCLE_RADIUS 6.5 // (mm) Circular pattern radius
#define NOZZLE_CLEAN_CIRCLE_FN 10 // Circular pattern circle number of segments
#define NOZZLE_CLEAN_CIRCLE_MIDDLE                                             \
  NOZZLE_CLEAN_START_POINT // Middle point of circle
#endif

#define NOZZLE_CLEAN_GOBACK

#define NOZZLE_CLEAN_MIN_TEMP 170

#endif

// @section host

/**
 * Print Job Timer
 *
 * Automatically start and stop the print job timer on
 * M104/M109/M140/M190/M141/M191. The print job timer will only be stopped if
 * the bed/chamber target temp is below BED_MINTEMP/CHAMBER_MINTEMP.
 *
 *   M104 (hotend, no wait)  - high temp = none,        low temp = stop timer
 *   M109 (hotend, wait)     - high temp = start timer, low temp = stop timer
 *   M140 (bed, no wait)     - high temp = none,        low temp = stop timer
 *   M190 (bed, wait)        - high temp = start timer, low temp = none
 *   M141 (chamber, no wait) - high temp = none,        low temp = stop timer
 *   M191 (chamber, wait)    - high temp = start timer, low temp = none
 *
 * For M104/M109, high temp is anything over EXTRUDE_MINTEMP / 2.
 * For M140/M190, high temp is anything over BED_MINTEMP.
 * For M141/M191, high temp is anything over CHAMBER_MINTEMP.
 *
 * The timer can also be controlled with the following commands:
 *
 *   M75 - Start the print job timer
 *   M76 - Pause the print job timer
 *   M77 - Stop the print job timer
 */
#define PRINTJOB_TIMER_AUTOSTART

// @section stats

#if ENABLED(PRINTCOUNTER)
#define PRINTCOUNTER_SAVE_INTERVAL                                             \
  60 // (minutes) EEPROM save interval during print. A value of 0 will save
#endif

// @section security

#if ENABLED(PASSWORD_FEATURE)
#define PASSWORD_LENGTH 4 // (#) Number of digits (1-9). 3 or 4 is recommended
#define PASSWORD_ON_STARTUP
#define PASSWORD_UNLOCK_GCODE // Unlock with the M511 P<password> command.
#define PASSWORD_CHANGE_GCODE // Change the password with M512 P<old> S<new>.
#endif

// @section media

/**
 * SD CARD: ENABLE CRC
 *
 * Use CRC checks and retries on the SD communication.
 */

// @section interface

/**
 * LCD LANGUAGE
 *
 * Select the language to display on the LCD. These languages are available:
 *
 *   en, an, bg, ca, cz, da, de, el, el_CY, es, eu, fi, fr, gl, hr, hu, it,
 *   jp_kana, ko_KR, nl, pl, pt, pt_br, ro, ru, sk, sv, tr, uk, vi, zh_CN, zh_TW
 *
 * :{ 'en':'English', 'an':'Aragonese', 'bg':'Bulgarian', 'ca':'Catalan',
 * 'cz':'Czech', 'da':'Danish', 'de':'German', 'el':'Greek (Greece)',
 * 'el_CY':'Greek (Cyprus)', 'es':'Spanish', 'eu':'Basque-Euskera',
 * 'fi':'Finnish', 'fr':'French', 'gl':'Galician', 'hr':'Croatian',
 * 'hu':'Hungarian', 'it':'Italian', 'jp_kana':'Japanese', 'ko_KR':'Korean
 * (South Korea)', 'nl':'Dutch', 'pl':'Polish', 'pt':'Portuguese',
 * 'pt_br':'Portuguese (Brazilian)', 'ro':'Romanian', 'ru':'Russian',
 * 'sk':'Slovak', 'sv':'Swedish', 'tr':'Turkish', 'uk':'Ukrainian',
 * 'vi':'Vietnamese', 'zh_CN':'Chinese (Simplified)', 'zh_TW':'Chinese
 * (Traditional)' }
 */
#define LCD_LANGUAGE en

/**
 * LCD Character Set
 *
 * Note: This option is NOT applicable to Graphical Displays.
 *
 * All character-based LCDs provide ASCII plus one of these
 * language extensions:
 *
 *  - JAPANESE ... the most common
 *  - WESTERN  ... with more accented characters
 *  - CYRILLIC ... for the Russian language
 *
 * To determine the language extension installed on your controller:
 *
 *  - Compile and upload with LCD_LANGUAGE set to 'test'
 *  - Click the controller to view the LCD menu
 *  - The LCD will display Japanese, Western, or Cyrillic text
 *
 * See https://marlinfw.org/docs/development/lcd_language.html
 *
 * :['JAPANESE', 'WESTERN', 'CYRILLIC']
 */
#define DISPLAY_CHARSET_HD44780 JAPANESE

/**
 * Info Screen Style (0:Classic, 1:Průša, 2:CNC)
 *
 * :[0:'Classic', 1:'Průša', 2:'CNC']
 */
#define LCD_INFO_SCREEN_STYLE 0

#if ENABLED(ENCODER_NOISE_FILTER)
#define ENCODER_SAMPLES 10
#endif

//=============================================================================
//======================== LCD / Controller Selection =========================
//========================   (Character-based LCDs)   =========================
//=============================================================================
// @section lcd

//=============================================================================
//======================== LCD / Controller Selection =========================
//=====================   (I2C and Shift-Register LCDs)   =====================
//=============================================================================

//=============================================================================
//=======================   LCD / Controller Selection  =======================
//=========================      (Graphical LCDs)      ========================
//=============================================================================

//=============================================================================
//==============================  OLED Displays  ==============================
//=============================================================================

#if ENABLED(SAV_3DGLCD)
#define U8GLIB_SSD1306
#endif

//=============================================================================
//========================== Extensible UI Displays ===========================
//=============================================================================

#if DGUS_UI_IS(MKS)
#define USE_MKS_GREEN_UI
#elif DGUS_UI_IS(IA_CREALITY)
#endif

//=============================================================================
//=============================== Graphical TFTs ==============================
//=============================================================================

#if ENABLED(TFT_GENERIC)
#define TFT_DRIVER AUTO

#endif

#if ENABLED(TFT_COLOR_UI)
/**
 * TFT Font for Color UI. Choose one of the following:
 *
 * NOTOSANS  - Default font with anti-aliasing. Supports Latin Extended and
 * non-Latin characters. UNIFONT   - Lightweight font, no anti-aliasing.
 * Supports Latin Extended and non-Latin characters. HELVETICA - Lightweight
 * font, no anti-aliasing. Supports Basic Latin (0x0020-0x007F) and Latin-1
 * Supplement (0x0080-0x00FF) characters only.
 * :['NOTOSANS', 'UNIFONT', 'HELVETICA']
 */
#define TFT_FONT NOTOSANS

/**
 * TFT Theme for Color UI. Choose one of the following or add a new one to
 * 'Marlin/src/lcd/tft/themes' directory
 *
 * BLUE_MARLIN  - Default theme with 'midnight blue' background
 * BLACK_MARLIN - Theme with 'black' background
 * ANET_BLACK   - Theme used for Anet ET4/5
 * :['BLUE_MARLIN', 'BLACK_MARLIN', 'ANET_BLACK']
 */
#define TFT_THEME BLACK_MARLIN

#define COMPACT_MARLIN_BOOT_LOGO // Use compressed data to save Flash space
#endif

//=============================================================================
//============================  Other Controllers  ============================
//=============================================================================

#if ENABLED(TOUCH_SCREEN)
#define BUTTON_DELAY_EDIT 50  // (ms) Button repeat delay for edit screens
#define BUTTON_DELAY_MENU 250 // (ms) Button repeat delay for menus

#define TOUCH_SCREEN_CALIBRATION

#if ALL(TOUCH_SCREEN_CALIBRATION, EEPROM_SETTINGS)
#define TOUCH_CALIBRATION_AUTO_SAVE // Auto save successful calibration values
#endif

#endif

//=============================================================================
//=============================== Extra Features ==============================
//=============================================================================

// @section fans

/**
 * Incrementing this by 1 will double the software PWM frequency, affecting
 * heaters, and the fan if FAN_SOFT_PWM is enabled. However, control resolution
 * will be halved for each increment; at zero value, there are 128 effective
 * control positions.
 * :[0,1,2,3,4,5,6,7]
 */
#define SOFT_PWM_SCALE 0

// @section extras

// @section lights

#if ANY(RGB_LED, RGBW_LED, PCA9632)
#if ENABLED(RGB_STARTUP_TEST)
#define RGB_STARTUP_TEST_INNER_MS 10 // (ms) Reduce or increase fading speed
#endif
#endif

#if ENABLED(NEOPIXEL_LED)
#define NEOPIXEL_TYPE NEO_GRBW // NEO_GRBW, NEO_RGBW, NEO_GRB, NEO_RBG, etc.
#define NEOPIXEL_PIXELS                                                        \
  30 // Number of LEDs in the strip. (Longest strip when NEOPIXEL2_SEPARATE is
#define NEOPIXEL_IS_SEQUENTIAL  // Sequential display for temperature change -
#define NEOPIXEL_BRIGHTNESS 127 // Initial brightness (0-255)

#if ENABLED(NEOPIXEL2_SEPARATE)
#define NEOPIXEL2_PIXELS 15      // Number of LEDs in the second strip
#define NEOPIXEL2_BRIGHTNESS 127 // Initial brightness (0-255)
#define NEOPIXEL2_STARTUP_TEST   // Cycle through colors at startup
#define NEOPIXEL_M150_DEFAULT                                                  \
  -1 // Default strip for M150 without 'S'. Use -1 to set all by default.
#else
#endif

#endif

/**
 * Printer Event LEDs
 *
 * During printing, the LEDs will reflect the printer status:
 *
 *  - Gradually change from blue to violet as the heated bed gets to target temp
 *  - Gradually change from violet to red as the hotend gets to temperature
 *  - Change to white to illuminate work surface
 *  - Change to green once print has finished
 *  - Turn off after the print has finished and the user has pushed a button
 */
#if ANY(BLINKM, RGB_LED, RGBW_LED, PCA9632, PCA9533, NEOPIXEL_LED)
#define PRINTER_EVENT_LEDS
#endif

// @section servos

#define SERVO_DELAY {300}
