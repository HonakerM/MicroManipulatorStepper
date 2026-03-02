#pragma once

#include <stdint.h>
#include "utilities.h"

#define ROT_DEGREE_SCALE_FACTOR 100.0

class RotDegree {
    public:
        RotDegree(float deg) {
            value=(uint32_t)(deg*ROT_DEGREE_SCALE_FACTOR);
        }
        RotDegree(uint32_t raw) {
            value=raw;
        }
        FourByteArray get_trans_value() {
            return FourByteArray{value};
        }
        float get_value() {
            return (float)value/ROT_DEGREE_SCALE_FACTOR;
        }
    
    private:
        uint32_t value;
};