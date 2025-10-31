// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file color_lut.h
// @brief Color lookup tables for gamma correction

#pragma once

#include "hub75_types.h"
#include "hub75_config.h"
#include <stdint.h>

namespace hub75 {

/**
 * @brief Get LUT for specified gamma mode and bit depth
 * @param mode Gamma correction mode
 * @param bit_depth Bit depth (6, 7, 8, 10, or 12)
 * @return Pointer to 256-entry LUT (uint16_t array)
 */
HUB75_PURE HUB75_WARN_UNUSED const uint16_t *get_lut(Hub75GammaMode mode, uint8_t bit_depth);

/**
 * @brief Get CIE 1931 LUT for specified bit depth
 * @param bit_depth Bit depth
 * @return Pointer to CIE 1931 LUT
 */
HUB75_PURE HUB75_WARN_UNUSED const uint16_t *get_cie1931_lut(uint8_t bit_depth);

/**
 * @brief Get Gamma 2.2 LUT for specified bit depth
 * @param bit_depth Bit depth
 * @return Pointer to Gamma 2.2 LUT
 */
HUB75_PURE HUB75_WARN_UNUSED const uint16_t *get_gamma22_lut(uint8_t bit_depth);

/**
 * @brief Get linear (no correction) LUT for specified bit depth
 * @param bit_depth Bit depth
 * @return Pointer to linear LUT
 */
HUB75_PURE HUB75_WARN_UNUSED const uint16_t *get_linear_lut(uint8_t bit_depth);

}  // namespace hub75
