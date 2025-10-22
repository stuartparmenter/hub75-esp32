// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file color_convert.hpp
// @brief RGB565 scaling utilities for color conversion
//
// Provides functions to scale RGB565 color components (5-bit and 6-bit)
// to 8-bit values for display on HUB75 panels.

#pragma once

#include "hub75_config.h"
#include <stdint.h>

namespace hub75 {

// ============================================================================
// RGB565 Scaling Utilities
// ============================================================================

/**
 * @brief Scale 5-bit color value to 8-bit (for RGB565 red/blue channels)
 *
 * Formula: (val5 << 3) | (val5 >> 2)
 * Replicates MSBs into LSBs for uniform distribution (31 -> 255)
 *
 * @param val5 5-bit value (0-31)
 * @return 8-bit value (0-255)
 */
HUB75_CONST HUB75_IRAM inline constexpr uint8_t scale_5bit_to_8bit(uint8_t val5) { return (val5 << 3) | (val5 >> 2); }

/**
 * @brief Scale 6-bit color value to 8-bit (for RGB565 green channel)
 *
 * Formula: (val6 << 2) | (val6 >> 4)
 * Replicates MSBs into LSBs for uniform distribution (63 -> 255)
 *
 * @param val6 6-bit value (0-63)
 * @return 8-bit value (0-255)
 */
HUB75_CONST HUB75_IRAM inline constexpr uint8_t scale_6bit_to_8bit(uint8_t val6) { return (val6 << 2) | (val6 >> 4); }

// ============================================================================
// Compile-Time Validation
// ============================================================================

namespace {  // Anonymous namespace for compile-time validation

// Validate RGB565 5-bit to 8-bit scaling produces exact 255
consteval bool test_rgb565_5bit_scaling() {
  constexpr uint8_t r8_max = scale_5bit_to_8bit(31);
  return r8_max == 255;
}

// Validate RGB565 6-bit to 8-bit scaling produces exact 255
consteval bool test_rgb565_6bit_scaling() {
  constexpr uint8_t g8_max = scale_6bit_to_8bit(63);
  return g8_max == 255;
}

static_assert(test_rgb565_5bit_scaling(), "RGB565 5-bit scaling does not produce exact 255 for max value (31)");
static_assert(test_rgb565_6bit_scaling(), "RGB565 6-bit scaling does not produce exact 255 for max value (63)");

}  // namespace

}  // namespace hub75
