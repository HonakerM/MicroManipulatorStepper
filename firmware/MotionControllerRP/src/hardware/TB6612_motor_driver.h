// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include <stdint.h>
#include <functional>
// #include <Arduino.h>

class TB6612MotorDriver {
  public:
    TB6612MotorDriver(
      uint8_t pin_en_a, uint8_t pin_pos_a, uint8_t pin_neg_a, uint8_t pin_pwm_a,
      uint8_t pin_en_b, uint8_t pin_pos_b, uint8_t pin_neg_b, uint8_t pin_pwm_b,
      uint8_t ch_pos_a=0, uint8_t ch_neg_a=1,
      uint8_t ch_pos_b=2, uint8_t ch_neg_b=3,
      uint16_t pwm_freq = 20000,    // might not be hit exactly and may be rounded to nearby freq.
      uint8_t pwm_resolution = 12
    );

    void  begin();
    void  enable();
    void  disable();
    void  set_field_angle(float angle_rad);
    float get_field_angle();
    void  set_amplitude(float amplitude, bool immediate_update);  // Input in range 0.0–1.0
    // on_step is called on every ramp step, use it to keep encoders being read.
    void  set_amplitude_smooth(float amplitude, int ramp_time_ms,
                               const std::function<void()>& on_step = {});
    float get_amplitude() const;

    // Rotate the magnetic field by the given angle delta.
    // For longer moves, use on_step callback to update encoders.
    void  rotate_field(float delta_angle, float rad_per_s, const std::function<void()>& on_step);

  private:
    void  set_pwm(uint8_t ch_pos, uint8_t ch_neg, int32_t value);

    uint8_t pin_en_a, pin_pos_a, pin_neg_a, pin_pwm_a;
    uint8_t pin_en_b, pin_pos_b, pin_neg_b, pin_pwm_b;
    uint8_t ch_pos_a, ch_neg_a;
    uint8_t ch_pos_b, ch_neg_b;

    uint16_t pwm_freq;
    uint8_t pwm_resolution;
    uint16_t max_pwm;
    float amplitude_raw=0.0f;  // scaled to 0–max_pwm
    float amplitude=0.0f;      // scaled to 0–1
    float field_angle;
};

