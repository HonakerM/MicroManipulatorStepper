// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include <stdint.h>
#include <limits>
#include "math_constants.h"

// #define CHECK_FP_MATH_ERRORS
#ifdef CHECK_FP_MATH_ERRORS
  #include "compat/Serial_compat.h"
#endif

/**
 * @brief A header-only utility class for fixed-point arithmetic operations.
 *
 * This class provides methods to convert between floating-point and fixed-point,
 * and perform common fixed-point arithmetic (multiplication, division, reciprocal).
 * It uses a specified Q-format (number of fractional bits) for all operations.
 */
class FPMath {
  public:
    // constants
    const int32_t FP_ONE; // Represents the value 1.0 in fixed-point (1 << q)
    const int32_t FP_PI;
    const int32_t FP_TWO_PI;

  public:
    /**
     * @brief Constructor for FPMath.
     * @param q_format The number of fractional bits for fixed-point representation.
     * A Q-format of 'q' means numbers are stored as N * 2^q.
     * For example, q=16 means 16 fractional bits.
     */
    explicit FPMath(uint8_t q_format)
        : q(q_format),
          FP_ONE(int32_t(1) << q_format),
          US_TO_MSFP(FP_ONE/1000),
          FP_TO_FLOAT(1.0f/(int32_t(1)<<q_format)),
          FP_PI(to_fixpoint(Constants::PI_F)),
          FP_TWO_PI(to_fixpoint(Constants::TWO_PI_F))
          {

          }

    inline uint8_t get_qformat() {
      return q;
    }

    /**
     * @brief Converts a floating-point value to its fixed-point representation.
     * @param v The floating-point value to convert.
     * @return The fixed-point representation of 'v'.
     */
    inline int32_t to_fixpoint(float v) const {
        return static_cast<int32_t>(v * FP_ONE);
    }

    /**
     * @brief Converts a fixed-point value back to its floating-point representation.
     * @param v The fixed-point value to convert.
     * @return The floating-point representation of 'v'.
     */
    inline float from_fixpoint(int32_t v) const {
        return static_cast<float>(v) * FP_TO_FLOAT;
    }

    /**
     * @brief Converts a duration in microseconds (uint32_t) to a fixed-point
     * representation in milliseconds.
     * @param dt_us The duration in microseconds, in normal integer format.
     * @return The duration in milliseconds, in fixed-point format.
     */
    inline int32_t duration_us_to_ms(uint32_t dt_us) const {
        return static_cast<int32_t>(dt_us * US_TO_MSFP);
    }

    /**
     * @brief Performs fixed-point multiplication.
     * @param a First operand in fixed-point (Q-format).
     * @param b Second operand in fixed-point (Q-format).
     * @return Result of a * b in fixed-point (Q-format maintained).
     * Uses 64-bit intermediate multiplication to prevent overflow.
     */
    inline int32_t mul(int32_t a, int32_t b) const {
        // Multiplying two Qx numbers results in a Q(2x) number.
        // Shifting right by 'q' converts it back to Qx.
        // Use int64_t for intermediate product to prevent overflow.
        // be aware that the compiler needs to keep the sign bit untouched
        int64_t product = static_cast<int64_t>(a) * b;
        int64_t shifted = product >> q;
        #ifdef CHECK_FP_MATH_ERRORS
          if (shifted > INT32_MAX) on_overflow_error();
          if (shifted < INT32_MIN) on_overflow_error();
        #endif
        return static_cast<int32_t>(shifted);
    }

    /**
     * @brief Performs fixed-point division.
     * @param a Numerator in fixed-point (Q-format).
     * @param b Denominator in fixed-point (Q-format).
     * @return Result of a / b in fixed-point (Q-format maintained).
     * Handles division by zero by saturating the result.
     */
    inline int32_t div(int32_t a, int32_t b) const {
        if (b == 0) {
            // Handle division by zero: return saturation value.
            return (a >= 0) ? std::numeric_limits<int32_t>::max() : std::numeric_limits<int32_t>::min();
        }
        // To maintain Qx precision after division, multiply numerator by FP_ONE (2^q) before dividing.
        // Use int64_t for intermediate product to prevent overflow.
        return static_cast<int32_t>((static_cast<int64_t>(a) * FP_ONE) / b);
    }

    /**
     * @brief Calculates the reciprocal (1 / a) in fixed-point.
     * @param a The operand in fixed-point (Q-format).
     * @return The reciprocal of 'a' in fixed-point (Q-format).
     */
    inline int32_t one_over(int32_t a) const {
        return div(FP_ONE, a);
    }

    #ifdef CHECK_FP_MATH_ERRORS
    inline void on_overflow_error() const {
      Serial.printf("fp_math overflow error:");
    }
    #endif

  private:
    const uint8_t q;      // Number of fractional bits for fixed-point
    const int32_t US_TO_MSFP; // Pre-calculated fixed-point value for 1/1000 for us to ms conversion
    const float   FP_TO_FLOAT;
};
