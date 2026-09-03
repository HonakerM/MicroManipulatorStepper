// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include "hardware/MT6835_encoder.h"
#include "hardware/TB6612_motor_driver.h"
#include "utilities/lookup_table.h"
#include "pid.h"

//*** CLASS *****************************************************************************

class ServoController {
  public:
    // use defines instead of virtual functions for speed
    // TODO: check if this makes any difference and change accordingly
    typedef TB6612MotorDriver MOTOR_DRIVER_TYPE;
    typedef MT6835Encoder ENCODER_TYPE;

  public:
    ServoController(MOTOR_DRIVER_TYPE& motor_driver, ENCODER_TYPE& encoder, int32_t motor_pole_pairs);

    // initialize the servo controller hardware
    void init(float max_motor_amplitude);

    // set the encoder raw angle to motor position lookup table
    void set_enc_to_pos_lut(LookupTable& lut);
    // get the encoder raw angle to motor position lookup table
    const LookupTable& get_enc_to_pos_lut() const;

    // set the motor position to field angle lookup table
    void set_pos_to_field_lut(LookupTable& lut);
    // get the motor position to field angle lookup table
    const LookupTable& get_pos_to_field_lut() const;

    // Updates the servo loop. 
    void update(float target_motor_pos,
                float dt,
                float one_over_dt);

    // Checks if the motor is at position (uses values from previous update() call).
    bool at_position(float motor_pos_eps);

    // Reads the current motor position from the encoder.
    float read_position();

    // Returns the motor position.
    float get_position();

    // Returns the current position error from the last servo loop update.
    float get_position_error();

    // Moves to a new motor position using closed loop control (blocking).
    bool move_to(float target_motor_angle, 
                 float at_pos_motor_angle_eps, 
                 float settle_time_ms,
                 float timeout_us);

    // Moves to a new motor position using open loop controll (blocking).
    // Motor updates must be disabled if servo loop is running in background.
    void move_to_open_loop(float target_motor_angle, 
                           float angular_velocity);
    
    // Returns the encoder object.
    ENCODER_TYPE& get_encoder();

    // Returns the motor driver object.
    MOTOR_DRIVER_TYPE& get_motor_driver();

    // Returns number of motor pole pairs.
    // Can be used to approximate conversion of motor position to field angle.
    float get_pole_pair_count();

    // enable or disable motor 
    void set_motor_enabled(bool enable, bool synchronize_field_angle);

    // enable or disable motor updates
    void set_motor_update_enabled(bool enable);

    // enable or disable encoder reads
    void set_encoder_update_enabled(bool enable);

  public:
    float encoder_angle_to_motor_pos(int32_t encoder_angle_raw);
    float motor_pos_to_field_angle(float motor_pos);
    float motor_pos_to_field_angle_derivative(float motor_pos);

  public:
    LowpassFilter velocity_lowpass;
    PIDController pos_controller;
    PIDController velocity_controller;

    uint32_t field_lut_clamp_count = 0;
    uint32_t enc_lut_clamp_count = 0;
    float    last_out_of_range_pos = 0.0f;
    float max_abs_pos_error = 0.0f;
    float max_abs_output = 0.0f;
    uint32_t stall_count = 0;
    float stall_pos = 0.0f;
    bool stall_reported = false;
    uint32_t skipped_update_count = 0;   // servo cycles skipped due to rejected encoder reads
    bool encoder_fault_reported = false;
  private:
    ENCODER_TYPE& encoder;
    MOTOR_DRIVER_TYPE& motor_driver;
    LookupTable encoder_raw_to_motor_pos_lut;
    LookupTable motor_pos_to_field_angle_lut;

    float motor_pole_pair_count = 0.0f;   // number as motor pole pairs (as float to avoid repeated conversion)
    float motor_current_amplitude = 0.5f; // motor current in range [0..1]
    float motor_pos = 0;                  // current motor position
    float motor_pos_prev = 0;             // previous motor position
    float pos_error = 0;                  // current position error as computed by upate()
    float velocity = 0;                   // current velocity estimate
    float output = 0.0f;                  // servo loop output (field angle offset)
    float pending_dt = 0.0f;              // time of skipped cycles, added to the next update
    bool motor_update_enabled = false;    // enables mootor field updates
    bool encoder_update_enabled = true;   // enables encoder reads
};