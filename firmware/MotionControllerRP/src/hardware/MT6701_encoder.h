#pragma once

#include "compat/Wire_compat.h"
#include "utilities/math_constants.h"

class MT6701Encoder {
  public:
      using AbsRawAngleType = int32_t;

      MT6701Encoder(TwoWire& wire, uint8_t i2c_addr=0x06);
      void init();

      int32_t read_abs_angle_raw();   
      float read_abs_angle();         

      void set_hysteresis(uint8_t hyst); // 0–7
      AbsRawAngleType update_abs_raw_angle(int32_t raw_angle);

  private:
      TwoWire& wire;
      uint8_t address;

      AbsRawAngleType abs_raw_angle = 0;
      int32_t last_raw_angle = -1;

      static constexpr int CPR = 16384;         // 14-bit resolution
      static constexpr float RAW_TO_RAD = 2 * Constants::PI_F / CPR;
};
