// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file color_lut.cpp
// @brief Color lookup table implementation with compile-time generation
//
// CIE 1931 lightness lookup tables adapted from:
// - https://github.com/mrcodetastic/ESP32-HUB75-MatrixPanel-DMA
// - https://ledshield.wordpress.com/2012/11/13/led-brightness-to-your-eye-gamma-correction-no/
// - https://gist.github.com/mathiasvr/19ce1d7b6caeab230934080ae1f1380e
//
// Formula: CIE 1931 lightness curve
//   For L ≤ 8:    Y = L / 902.3
//   For L > 8:    Y = ((L + 16) / 116)³
//   Where L = input brightness (0-100), Y = output luminance (0-1)
//
// All lookup tables are generated at compile-time using constexpr functions,
// eliminating runtime overhead and enabling compile-time validation.

#include "color_lut.h"
#include <cstddef>
#include <array>

namespace hub75 {

// ============================================================================
// Compile-Time Math Helpers
// ============================================================================

/**
 * @brief Compile-time power function (x^n for integer n)
 */
static constexpr HUB75_CONST double constexpr_pow(double base, int exp) {
  if (exp < 0) {
    return 1.0 / constexpr_pow(base, -exp);
  }
  double result = 1.0;
  for (int i = 0; i < exp; i++) {
    result *= base;
  }
  return result;
}

/**
 * @brief Compile-time power function (x^y for fractional y)
 * Uses Taylor series approximation for exp and log
 */
static constexpr HUB75_CONST double constexpr_pow_frac(double base, double exp) {
  // For gamma correction, we only need positive base and exp
  // Use approximation: base^exp = e^(exp * ln(base))

  // Newton-Raphson for ln(base)
  double ln_base = 0.0;
  const double x = base;
  if (x > 0.0) {
    // ln(x) approximation using series expansion
    // For x close to 1: ln(x) ≈ 2*((x-1)/(x+1) + ((x-1)/(x+1))³/3 + ...)
    const double t = (x - 1.0) / (x + 1.0);
    const double t2 = t * t;
    ln_base = 2.0 * t * (1.0 + t2 / 3.0 + t2 * t2 / 5.0 + t2 * t2 * t2 / 7.0);
  }

  const double power = exp * ln_base;

  // e^power using Taylor series: e^x = 1 + x + x²/2! + x³/3! + ...
  double result = 1.0;
  double term = 1.0;
  for (int i = 1; i < 20; i++) {
    term *= power / i;
    result += term;
  }

  return result;
}

/**
 * @brief Compile-time rounding
 * std::lround not constexpr until C++23
 */
static constexpr HUB75_CONST int constexpr_round(double x) {
  // NOLINTNEXTLINE(bugprone-incorrect-roundings)
  return (x >= 0.0) ? static_cast<int>(x + 0.5) : static_cast<int>(x - 0.5);
}

/**
 * @brief Compile-time clamp function
 * Clamps value between min and max
 */
static constexpr HUB75_CONST int constexpr_clamp(int value, int min_val, int max_val) {
  return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

// ============================================================================
// CIE 1931 Compile-Time Generation
// ============================================================================

/**
 * @brief CIE 1931 lightness formula (constexpr)
 * @param lightness Lightness value (0-100)
 */
static constexpr HUB75_CONST double cie1931(double lightness) {
  if (lightness <= 8.0) {
    return lightness / 902.3;
  } else {
    const double temp = (lightness + 16.0) / 116.0;
    return temp * temp * temp;  // Cube
  }
}

/**
 * @brief Generate CIE 1931 lookup table at compile time
 * @tparam BitDepth Target bit depth (6-12)
 * @return std::array of 256 values scaled to BitDepth range
 */
template<uint8_t BitDepth> static constexpr std::array<uint16_t, 256> generate_cie1931_lut() {
  static_assert(BitDepth >= 6 && BitDepth <= 12, "Bit depth must be 6-12");

  constexpr uint16_t max_val = (1 << BitDepth) - 1;
  std::array<uint16_t, 256> lut{};

  for (int i = 0; i < 256; i++) {
    // Normalize input to 0-1 range, then scale to 0-100 for CIE formula
    const double lightness = (i / 255.0) * 100.0;
    // Apply CIE 1931 lightness curve
    const double luminance = cie1931(lightness);
    // Scale to target bit depth, round, and clamp to ensure no overflow
    const int rounded = constexpr_round(luminance * max_val);
    lut[i] = static_cast<uint16_t>(constexpr_clamp(rounded, 0, max_val));
  }

  return lut;
}

/**
 * @brief Generate Gamma 2.2 lookup table at compile time
 */
template<uint8_t BitDepth> static constexpr std::array<uint16_t, 256> generate_gamma22_lut() {
  static_assert(BitDepth >= 6 && BitDepth <= 12, "Bit depth must be 6-12");

  constexpr uint16_t max_val = (1 << BitDepth) - 1;
  std::array<uint16_t, 256> lut{};

  for (int i = 0; i < 256; i++) {
    const double normalized = i / 255.0;
    const double corrected = constexpr_pow_frac(normalized, 2.2);
    // Scale to target bit depth, round, and clamp to ensure no overflow
    const int rounded = constexpr_round(corrected * max_val);
    lut[i] = static_cast<uint16_t>(constexpr_clamp(rounded, 0, max_val));
  }

  return lut;
}

/**
 * @brief Generate Linear lookup table at compile time
 */
template<uint8_t BitDepth> static constexpr std::array<uint16_t, 256> generate_linear_lut() {
  static_assert(BitDepth >= 6 && BitDepth <= 12, "Bit depth must be 6-12");

  constexpr uint16_t max_val = (1 << BitDepth) - 1;
  std::array<uint16_t, 256> lut{};

  for (int i = 0; i < 256; i++) {
    // Scale to target bit depth and clamp (for consistency, though division should be safe)
    const int value = (i * max_val) / 255;
    lut[i] = static_cast<uint16_t>(constexpr_clamp(value, 0, max_val));
  }

  return lut;
}

// ============================================================================
// Compile-Time LUT Selection (Single LUT based on HUB75_GAMMA_MODE/HUB75_BIT_DEPTH)
// ============================================================================

#if HUB75_GAMMA_MODE == 0  // LINEAR/NONE
constexpr auto LUT = generate_linear_lut<HUB75_BIT_DEPTH>();
#elif HUB75_GAMMA_MODE == 1  // CIE1931
constexpr auto LUT = generate_cie1931_lut<HUB75_BIT_DEPTH>();
#elif HUB75_GAMMA_MODE == 2  // GAMMA_2_2
constexpr auto LUT = generate_gamma22_lut<HUB75_BIT_DEPTH>();
#else
#error "Invalid HUB75_GAMMA_MODE (must be 0=LINEAR, 1=CIE1931, or 2=GAMMA_2_2)"
#endif

// ============================================================================
// Runtime LUT Access (No Selection - Single LUT)
// ============================================================================

constexpr const uint16_t *get_lut() noexcept { return LUT.data(); }

// ============================================================================
// Compile-Time Validation
// ============================================================================

namespace {

// Validate LUT monotonicity (gamma curves should be non-decreasing)
consteval bool validate_lut_monotonic() {
  for (size_t i = 1; i < 256; ++i) {
    if (LUT[i] < LUT[i - 1]) {
      return false;
    }
  }
  return true;
}

// Validate LUT bounds (values don't exceed bit depth max)
consteval bool validate_lut_bounds() {
  constexpr uint16_t max_val = (1 << HUB75_BIT_DEPTH) - 1;
  for (size_t i = 0; i < 256; ++i) {
    if (LUT[i] > max_val) {
      return false;
    }
  }
  return true;
}

// Validate endpoints (black=0, white=max)
consteval bool validate_lut_endpoints() {
  constexpr uint16_t max_val = (1 << HUB75_BIT_DEPTH) - 1;
  return (LUT[0] == 0) && (LUT[255] == max_val);
}

// Force compile-time evaluation
static_assert(validate_lut_monotonic(), "LUT not monotonically increasing");
static_assert(validate_lut_bounds(), "LUT values exceed bit depth max");
static_assert(validate_lut_endpoints(), "LUT endpoints incorrect (should be 0 and max)");

}  // namespace

}  // namespace hub75
