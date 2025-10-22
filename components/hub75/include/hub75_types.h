/**
 * SPDX-FileCopyrightText: 2025 Stuart Parmenter
 * SPDX-License-Identifier: MIT
 *
 * @file hub75_types.h
 * @brief Common types and enums for HUB75 driver
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Gamma correction mode
 */
typedef enum {
  HUB75_GAMMA_NONE = 0,  ///< No gamma correction (linear)
  HUB75_GAMMA_CIE1931,   ///< CIE 1931 perceptual correction
  HUB75_GAMMA_2_2,       ///< Standard gamma 2.2
} hub75_gamma_mode_t;

/**
 * @brief Pixel buffer format for bulk drawing operations
 */
typedef enum {
  HUB75_PIXEL_FORMAT_RGB888,     ///< 24-bit packed RGB (RGBRGBRGB...)
  HUB75_PIXEL_FORMAT_RGB888_32,  ///< 32-bit RGB with padding (xRGBxRGB... or BGRxBGRx...)
  HUB75_PIXEL_FORMAT_RGB565,     ///< 16-bit RGB565
} hub75_pixel_format_t;

/**
 * @brief Color component order for RGB888_32 format
 */
typedef enum {
  HUB75_COLOR_ORDER_RGB,  ///< Red-Green-Blue (xRGB or RGBx)
  HUB75_COLOR_ORDER_BGR,  ///< Blue-Green-Red (xBGR or BGRx)
} hub75_color_order_t;

/**
 * @brief Data bus width mode
 */
typedef enum {
  HUB75_BUS_WIDTH_8BIT = 8,    ///< 8-bit mode (6 data pins: R1/G1/B1/R2/G2/B2)
  HUB75_BUS_WIDTH_16BIT = 16,  ///< 16-bit mode (8 data pins: R1/G1/B1/R2/G2/B2 + 2 more)
} hub75_bus_width_t;

/**
 * @brief Panel scan pattern
 */
typedef enum {
  HUB75_SCAN_1_2 = 2,    ///< 1/2 scan (upper/lower half)
  HUB75_SCAN_1_4 = 4,    ///< 1/4 scan
  HUB75_SCAN_1_8 = 8,    ///< 1/8 scan
  HUB75_SCAN_1_16 = 16,  ///< 1/16 scan
  HUB75_SCAN_1_32 = 32,  ///< 1/32 scan
} hub75_scan_pattern_t;

/**
 * @brief Pin configuration for HUB75 interface
 */
typedef struct {
  // Data pins (upper half)
  int8_t r1;
  int8_t g1;
  int8_t b1;

  // Data pins (lower half)
  int8_t r2;
  int8_t g2;
  int8_t b2;

  // Address lines (A, B, C, D, E)
  int8_t a;
  int8_t b;
  int8_t c;
  int8_t d;
  int8_t e;  ///< -1 if not used (for panels ≤32 rows)

  // Control signals
  int8_t lat;  ///< Latch
  int8_t oe;   ///< Output Enable (active low)
  int8_t clk;  ///< Clock
} hub75_pins_t;

/**
 * @brief Driver configuration
 */
typedef struct {
  // Panel dimensions
  uint16_t width;                     ///< Panel width in pixels (single panel, not total)
  uint16_t height;                    ///< Panel height in pixels
  hub75_scan_pattern_t scan_pattern;  ///< Scan pattern (typically 1/32 for 64x64)
  uint8_t chain_length;               ///< Number of panels chained horizontally (default: 1)

  // Pin configuration
  hub75_pins_t pins;

  // Performance
  uint32_t output_clock_speed;  ///< Output clock speed in Hz (default: 20MHz)
  uint8_t bit_depth;            ///< BCM bit depth: 6, 7, 8, 10, or 12 (default: 8)
  uint16_t min_refresh_rate;    ///< Minimum refresh rate in Hz (default: 60)

  // Timing
  uint8_t latch_blanking;  ///< OE blanking cycles during LAT pulse (default: 1)

  // Features
  bool double_buffer;       ///< Enable double buffering (default: false)
  bool temporal_dither;     ///< Enable temporal dithering (default: false)
  bool clk_phase_inverted;  ///< Invert clock phase (default: false)

  // Color
  hub75_gamma_mode_t gamma_mode;  ///< Gamma correction mode (default: CIE1931)
  uint8_t brightness;             ///< Initial brightness 0-255 (default: 128)
} hub75_config_t;

/**
 * @brief Default configuration initializer
 */
#define HUB75_CONFIG_DEFAULT() \
  {.width = 64, \
   .height = 64, \
   .scan_pattern = HUB75_SCAN_1_2, \
   .chain_length = 1, \
   .pins = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}, \
   .output_clock_speed = 20000000, \
   .bit_depth = 8, \
   .min_refresh_rate = 60, \
   .latch_blanking = 1, \
   .double_buffer = false, \
   .temporal_dither = false, \
   .clk_phase_inverted = false, \
   .gamma_mode = HUB75_GAMMA_CIE1931, \
   .brightness = 128}

#ifdef __cplusplus
}
#endif
