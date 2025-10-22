/**
 * SPDX-FileCopyrightText: 2025 Stuart Parmenter
 * SPDX-License-Identifier: MIT
 *
 * @file color_lut.cpp
 * @brief Color lookup table implementation with compile-time generation
 *
 * CIE 1931 lightness lookup tables adapted from:
 * - ESP32-HUB75-MatrixPanel-DMA (stuartparmenter/ESP32-HUB75-MatrixPanel-DMA)
 * - https://ledshield.wordpress.com/2012/11/13/led-brightness-to-your-eye-gamma-correction-no/
 * - https://gist.github.com/mathiasvr/19ce1d7b6caeab230934080ae1f1380e
 *
 * Formula: CIE 1931 lightness curve
 *   For L ≤ 8:    Y = L / 902.3
 *   For L > 8:    Y = ((L + 16) / 116)³
 *   Where L = input brightness (0-100), Y = output luminance (0-1)
 *
 * All lookup tables are generated at compile-time using constexpr functions,
 * eliminating runtime overhead and enabling compile-time validation.
 */

#include "color_lut.hpp"
#include <array>

namespace hub75 {

// ============================================================================
// Compile-Time Math Helpers
// ============================================================================

/**
 * @brief Compile-time power function (x^n for integer n)
 */
static constexpr double constexpr_pow(double base, int exp) {
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
static constexpr double constexpr_pow_frac(double base, double exp) {
  // For gamma correction, we only need positive base and exp
  // Use approximation: base^exp = e^(exp * ln(base))

  // Newton-Raphson for ln(base)
  double ln_base = 0.0;
  double x = base;
  if (x > 0.0) {
    // ln(x) approximation using series expansion
    // For x close to 1: ln(x) ≈ 2*((x-1)/(x+1) + ((x-1)/(x+1))³/3 + ...)
    double t = (x - 1.0) / (x + 1.0);
    double t2 = t * t;
    ln_base = 2.0 * t * (1.0 + t2 / 3.0 + t2 * t2 / 5.0 + t2 * t2 * t2 / 7.0);
  }

  double power = exp * ln_base;

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
// NOLINTNEXTLINE(bugprone-incorrect-roundings)
static constexpr int constexpr_round(double x) {
  return (x >= 0.0) ? static_cast<int>(x + 0.5) : static_cast<int>(x - 0.5);
}

/**
 * @brief Compile-time clamp function
 * Clamps value between min and max
 */
static constexpr int constexpr_clamp(int value, int min_val, int max_val) {
  return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

// ============================================================================
// CIE 1931 Compile-Time Generation
// ============================================================================

/**
 * @brief CIE 1931 lightness formula (constexpr)
 * @param lightness Lightness value (0-100)
 */
static constexpr double cie1931(double lightness) {
  if (lightness <= 8.0) {
    return lightness / 902.3;
  } else {
    double temp = (lightness + 16.0) / 116.0;
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
    double lightness = (i / 255.0) * 100.0;
    // Apply CIE 1931 lightness curve
    double luminance = cie1931(lightness);
    // Scale to target bit depth, round, and clamp to ensure no overflow
    int rounded = constexpr_round(luminance * max_val);
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
    double normalized = i / 255.0;
    double corrected = constexpr_pow_frac(normalized, 2.2);
    // Scale to target bit depth, round, and clamp to ensure no overflow
    int rounded = constexpr_round(corrected * max_val);
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
    int value = (i * max_val) / 255;
    lut[i] = static_cast<uint16_t>(constexpr_clamp(value, 0, max_val));
  }

  return lut;
}

// ============================================================================
// Compile-Time Generated Lookup Tables (All Bit Depths 6-12)
// ============================================================================

// CIE 1931 tables for all bit depths (6-12)
constexpr auto LUM_CONV_TAB_6BIT = generate_cie1931_lut<6>();
constexpr auto LUM_CONV_TAB_7BIT = generate_cie1931_lut<7>();
constexpr auto LUM_CONV_TAB_8BIT = generate_cie1931_lut<8>();
constexpr auto LUM_CONV_TAB_9BIT = generate_cie1931_lut<9>();
constexpr auto LUM_CONV_TAB_10BIT = generate_cie1931_lut<10>();
constexpr auto LUM_CONV_TAB_11BIT = generate_cie1931_lut<11>();
constexpr auto LUM_CONV_TAB_12BIT = generate_cie1931_lut<12>();

// Gamma 2.2 tables
constexpr auto GAMMA22_LUT_6BIT = generate_gamma22_lut<6>();
constexpr auto GAMMA22_LUT_7BIT = generate_gamma22_lut<7>();
constexpr auto GAMMA22_LUT_8BIT = generate_gamma22_lut<8>();
constexpr auto GAMMA22_LUT_9BIT = generate_gamma22_lut<9>();
constexpr auto GAMMA22_LUT_10BIT = generate_gamma22_lut<10>();
constexpr auto GAMMA22_LUT_11BIT = generate_gamma22_lut<11>();
constexpr auto GAMMA22_LUT_12BIT = generate_gamma22_lut<12>();

// Linear tables
constexpr auto LINEAR_LUT_6BIT = generate_linear_lut<6>();
constexpr auto LINEAR_LUT_7BIT = generate_linear_lut<7>();
constexpr auto LINEAR_LUT_8BIT = generate_linear_lut<8>();
constexpr auto LINEAR_LUT_9BIT = generate_linear_lut<9>();
constexpr auto LINEAR_LUT_10BIT = generate_linear_lut<10>();
constexpr auto LINEAR_LUT_11BIT = generate_linear_lut<11>();
constexpr auto LINEAR_LUT_12BIT = generate_linear_lut<12>();

// ============================================================================
// Runtime LUT Selection Functions
// ============================================================================

const uint16_t *get_lut(hub75_gamma_mode_t mode, uint8_t bit_depth) {
  switch (mode) {
    case HUB75_GAMMA_CIE1931:
      return get_cie1931_lut(bit_depth);
    case HUB75_GAMMA_2_2:
      return get_gamma22_lut(bit_depth);
    case HUB75_GAMMA_NONE:
    default:
      return get_linear_lut(bit_depth);
  }
}

const uint16_t *get_cie1931_lut(uint8_t bit_depth) {
  switch (bit_depth) {
    case 6:
      return LUM_CONV_TAB_6BIT.data();
    case 7:
      return LUM_CONV_TAB_7BIT.data();
    case 8:
      return LUM_CONV_TAB_8BIT.data();
    case 9:
      return LUM_CONV_TAB_9BIT.data();
    case 10:
      return LUM_CONV_TAB_10BIT.data();
    case 11:
      return LUM_CONV_TAB_11BIT.data();
    case 12:
      return LUM_CONV_TAB_12BIT.data();
    default:
      // Fallback to 8-bit for unsupported depths
      return LUM_CONV_TAB_8BIT.data();
  }
}

const uint16_t *get_gamma22_lut(uint8_t bit_depth) {
  switch (bit_depth) {
    case 6:
      return GAMMA22_LUT_6BIT.data();
    case 7:
      return GAMMA22_LUT_7BIT.data();
    case 8:
      return GAMMA22_LUT_8BIT.data();
    case 9:
      return GAMMA22_LUT_9BIT.data();
    case 10:
      return GAMMA22_LUT_10BIT.data();
    case 11:
      return GAMMA22_LUT_11BIT.data();
    case 12:
      return GAMMA22_LUT_12BIT.data();
    default:
      return GAMMA22_LUT_8BIT.data();
  }
}

const uint16_t *get_linear_lut(uint8_t bit_depth) {
  switch (bit_depth) {
    case 6:
      return LINEAR_LUT_6BIT.data();
    case 7:
      return LINEAR_LUT_7BIT.data();
    case 8:
      return LINEAR_LUT_8BIT.data();
    case 9:
      return LINEAR_LUT_9BIT.data();
    case 10:
      return LINEAR_LUT_10BIT.data();
    case 11:
      return LINEAR_LUT_11BIT.data();
    case 12:
      return LINEAR_LUT_12BIT.data();
    default:
      return LINEAR_LUT_8BIT.data();
  }
}

}  // namespace hub75
