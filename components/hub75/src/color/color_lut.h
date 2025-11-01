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
 * @brief Get active lookup table (compile-time selected)
 *
 * Returns the LUT for the gamma mode and bit depth configured at compile time
 * via HUB75_GAMMA_MODE and HUB75_BIT_DEPTH macros.
 *
 * @return Pointer to 256-entry LUT (uint16_t array)
 */
HUB75_WARN_UNUSED constexpr const uint16_t *get_lut() noexcept;

}  // namespace hub75
