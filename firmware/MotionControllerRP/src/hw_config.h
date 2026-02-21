#pragma once
#include "utilities/math_constants.h"

//--- MOTORS ------------------------------------------------------------------

// motor pole pair count
//  * 100 for 0.9deg stepper motors
//  * 50  for 1.8deg stepper motors
constexpr float MOTOR1_POLE_PAIRS = 50;
constexpr float MOTOR2_POLE_PAIRS = 50;
constexpr float MOTOR3_POLE_PAIRS = 50;

//--- ENCODERS ----------------------------------------------------------------

// Conversion factor from encoder angle (one 2pi period every two magnets) to rotor angle.
// Used when the system can not rely on calibration data being present (e.g. during homing)
constexpr float ENCODER_MAGNET_PITCH = 3.0f;    // [mm]
constexpr float ENCODER_MAGNET_RADIUS = 30.0f;  // [mm]
constexpr float ENCODER_ANGLE_TO_ROTOR_ANGLE = (ENCODER_MAGNET_PITCH*2.0f) / 
                                               (ENCODER_MAGNET_RADIUS * Constants::TWO_PI_F);

//--- HOMING ------------------------------------------------------------------

constexpr float HOMING_VELOCITY   = 1.0f;        // rad per s
constexpr float HOMING_CURRENT    = 0.15f;       // range 0..1
constexpr float HOMING_FINISH_POS = 0.5f;        // in rad

//--- CALIBRATION -------------------------------------------------------------

// degrees from home position
constexpr float CALIBRATION_RANGE = 95; 

// velocity of the magnetic field during calibration (lower is more accurate)
constexpr float CALIBRATION_FIELD_VELOCITY = 40.0f; 

//--- PINS --------------------------------------------------------------------

// #define SINGLE_AXIS_BOARD
#ifndef SINGLE_AXIS_BOARD
  // Pins for 3Axis Board
  #define PIN_BUILTIN_LED 23
  #define PIN_USER_BUTTON 24

  #define PIN_M1_PWM_A_POS  13
  #define PIN_M1_PWM_A_NEG  12
  #define PIN_M1_PWM_B_POS  14
  #define PIN_M1_PWM_B_NEG  15

  #define PIN_M2_PWM_A_POS  9
  #define PIN_M2_PWM_A_NEG  8
  #define PIN_M2_PWM_B_POS  10
  #define PIN_M2_PWM_B_NEG  11

  #define PIN_M3_PWM_A_POS  5
  #define PIN_M3_PWM_A_NEG  4
  #define PIN_M3_PWM_B_POS  6
  #define PIN_M3_PWM_B_NEG  7


  #define PIN_MOTOR_EN      18
  #define PIN_MOTOR_PWMAB   19

  #define PIN_ENCODER1_CS 20
  #define PIN_ENCODER2_CS 21
  #define PIN_ENCODER3_CS 22
  #define PIN_ENCODER_SCK 2
  #define PIN_ENCODER_MISO 0
  #define PIN_ENCODER_MOSI 3

#else
  // Single Axis Board
  #define PIN_BUILTIN_LED 16
  #define PIN_USER_BUTTON 24

  #define PIN_M1_PWM_A_POS  13
  #define PIN_M1_PWM_A_NEG  12
  #define PIN_M1_PWM_B_POS  14
  #define PIN_M1_PWM_B_NEG  15

  #define PIN_M2_PWM_A_POS  9
  #define PIN_M2_PWM_A_NEG  8
  #define PIN_M2_PWM_B_POS  10
  #define PIN_M2_PWM_B_NEG  11

  #define PIN_M3_PWM_A_POS  5
  #define PIN_M3_PWM_A_NEG  4
  #define PIN_M3_PWM_B_POS  6
  #define PIN_M3_PWM_B_NEG  7


  #define PIN_MOTOR_EN      18
  #define PIN_MOTOR_PWMAB   19

  #define PIN_ENCODER1_CS 20
  #define PIN_ENCODER2_CS 21
  #define PIN_ENCODER3_CS 22
  #define PIN_ENCODER_SCK 2
  #define PIN_ENCODER_MISO 0
  #define PIN_ENCODER_MOSI 3
#endif