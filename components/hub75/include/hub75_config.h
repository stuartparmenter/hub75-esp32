/**
 * SPDX-FileCopyrightText: 2025 Stuart Parmenter
 * SPDX-License-Identifier: MIT
 *
 * @file hub75_config.h
 * @brief Compile-time configuration for HUB75 driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * IRAM optimization
 * Place hot-path code in instruction RAM to prevent flash cache stalls
 * Set to 0 to disable (useful for debugging)
 */
#ifndef HUB75_ENABLE_IRAM
#define HUB75_ENABLE_IRAM 1
#endif

#if HUB75_ENABLE_IRAM
// Only use IRAM_ATTR if it's defined (ESP-IDF provides it)
#if defined(IRAM_ATTR)
#define HUB75_IRAM IRAM_ATTR
#elif defined(__IRAM_ATTR)
#define HUB75_IRAM __IRAM_ATTR
#else
// Fallback: define as empty if not available
#define HUB75_IRAM
#endif
#else
#define HUB75_IRAM
#endif

/**
 * Maximum supported bit depth
 * Affects LUT size at compile time
 */
#ifndef HUB75_MAX_BIT_DEPTH
#define HUB75_MAX_BIT_DEPTH 12
#endif

/**
 * Default CIE 1931 LUT shift value
 * Higher values = more precision but darker output
 */
#ifndef HUB75_CIE_SHIFT
#define HUB75_CIE_SHIFT 8
#endif

/**
 * Temporal dithering configuration
 */
#ifndef HUB75_DITHER_SHIFT
#define HUB75_DITHER_SHIFT 8  ///< Accumulator precision (bits)
#endif

/**
 * Maximum chained panels
 */
#ifndef HUB75_MAX_CHAINED_PANELS
#define HUB75_MAX_CHAINED_PANELS 8
#endif

/**
 * Debug assertions
 */
#ifndef HUB75_ENABLE_ASSERT
#define HUB75_ENABLE_ASSERT 1
#endif

#if HUB75_ENABLE_ASSERT
#include <assert.h>
#define HUB75_ASSERT(x) assert(x)
#else
#define HUB75_ASSERT(x) ((void) 0)
#endif

#ifdef __cplusplus
}
#endif
