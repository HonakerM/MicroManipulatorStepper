// --------------------------------------------------------------------------------------
// Trivial replacements for the handful of Arduino free functions this project used
// (delay/delayMicroseconds), mapped directly onto their pico-sdk equivalents.
// --------------------------------------------------------------------------------------
#pragma once

#include "pico/time.h"

inline void delay(uint32_t ms) { sleep_ms(ms); }
inline void delayMicroseconds(uint32_t us) { sleep_us(us); }
