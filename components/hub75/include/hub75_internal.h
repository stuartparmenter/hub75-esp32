// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file hub75_internal.h
// @brief Internal types used within the driver implementation
//
// These types are not part of the public API and should not be used
// by external code.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Framebuffer format (internal)
 *
 * Selected automatically based on bit_depth in config.
 * Users don't need to set this directly - it's purely internal.
 */
enum class Hub75FramebufferFormat {
  RGB888 = 0,    // 32-bit format (8-bit per channel) - for bit_depth 6-8
  RGB121212 = 1  // 64-bit format (12-bit per channel) - for bit_depth 9-12
};

#ifdef __cplusplus
}
#endif
