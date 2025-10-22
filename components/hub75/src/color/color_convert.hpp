/**
 * SPDX-FileCopyrightText: 2025 Stuart Parmenter
 * SPDX-License-Identifier: MIT
 *
 * @file color_convert.hpp
 * @brief Fast color conversion functions (IRAM-optimized)
 *
 * Two framebuffer formats for memory efficiency:
 *
 * Format 1: RGB888 (for bit_depth 6-8)
 *   32-bit packed: [unused 8][R 8][G 8][B 8]
 *   Bits 23-16: Red (gamma-corrected, 0-255)
 *   Bits 15-8:  Green (gamma-corrected, 0-255)
 *   Bits 7-0:   Blue (gamma-corrected, 0-255)
 *
 * Format 2: RGB121212 (for bit_depth 9-12)
 *   64-bit packed: [unused 16][R 16][G 16][B 16]
 *   Bits 47-32: Red (gamma-corrected, 0-4095)
 *   Bits 31-16: Green (gamma-corrected, 0-4095)
 *   Bits 15-0:  Blue (gamma-corrected, 0-4095)
 *
 * This format allows fast setPixel() operations while deferring BCM bit-plane
 * extraction to the refresh loop where it's done once per frame.
 */

#pragma once

#include "hub75_config.h"
#include <stdint.h>

namespace hub75 {

/**
 * @brief Pack gamma-corrected RGB values into framebuffer format
 *
 * @param r_corrected Gamma-corrected red value
 * @param g_corrected Gamma-corrected green value
 * @param b_corrected Gamma-corrected blue value
 * @return 32-bit packed value ready for framebuffer
 */
HUB75_IRAM inline uint32_t pack_rgb(uint16_t r_corrected, uint16_t g_corrected, uint16_t b_corrected) {
  // Pack into 32-bit word: [unused 8 bits][R 8 bits][G 8 bits][B 8 bits]
  // For bit depths > 8, we store the full 16-bit value but only use lower 8 bits here
  // The refresh loop will extract the appropriate bits for BCM
  return ((r_corrected & 0xFF) << 16) | ((g_corrected & 0xFF) << 8) | (b_corrected & 0xFF);
}

/**
 * @brief Convert RGB888 to framebuffer format
 *
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @param lut Gamma correction LUT
 * @param bit_depth Bit depth (currently unused, for future expansion)
 * @return 32-bit packed value ready for framebuffer
 */
HUB75_IRAM inline uint32_t rgb888_to_native(uint8_t r, uint8_t g, uint8_t b, const uint16_t *lut, uint8_t bit_depth) {
  // Apply gamma correction via LUT
  uint16_t r_corrected = lut[r];
  uint16_t g_corrected = lut[g];
  uint16_t b_corrected = lut[b];

  (void) bit_depth;  // Currently storing all bit depths in same format

  return pack_rgb(r_corrected, g_corrected, b_corrected);
}

/**
 * @brief Convert RGB565 to framebuffer format
 *
 * @param rgb565 RGB565 color value
 * @param lut Gamma correction LUT
 * @param bit_depth Bit depth
 * @param big_endian True if RGB565 is big-endian
 * @return 32-bit packed value ready for framebuffer
 */
HUB75_IRAM inline uint32_t rgb565_to_native(uint16_t rgb565, const uint16_t *lut, uint8_t bit_depth, bool big_endian) {
  uint8_t r5, g6, b5;

  if (big_endian) {
    // Big-endian (byte-swapped): Bytes are GGGBBBBB RRRRRGGG
    // Swap bytes first to get standard bit order
    rgb565 = (rgb565 >> 8) | (rgb565 << 8);
    // Now fall through to standard extraction
  }

  // Standard RGB565 bit layout: RRRRRGGG GGGBBBBB (MSB first)
  r5 = (rgb565 >> 11) & 0x1F;  // Top 5 bits
  g6 = (rgb565 >> 5) & 0x3F;   // Middle 6 bits
  b5 = rgb565 & 0x1F;          // Bottom 5 bits

  // Scale to 8-bit (5-bit: x33, 6-bit: x4)
  uint8_t r8 = (r5 << 3) | (r5 >> 2);  // 5-bit to 8-bit
  uint8_t g8 = (g6 << 2) | (g6 >> 4);  // 6-bit to 8-bit
  uint8_t b8 = (b5 << 3) | (b5 >> 2);  // 5-bit to 8-bit

  // Use RGB888 conversion
  return rgb888_to_native(r8, g8, b8, lut, bit_depth);
}

/**
 * @brief Unpack RGB888 framebuffer value to RGB components (for BCM extraction)
 *
 * This is used by the refresh loop to extract gamma-corrected RGB values
 * before BCM bit-plane extraction.
 *
 * @param packed Packed framebuffer value (32-bit)
 * @param r Output: red component
 * @param g Output: green component
 * @param b Output: blue component
 */
HUB75_IRAM inline void unpack_rgb888(uint32_t packed, uint16_t &r, uint16_t &g, uint16_t &b) {
  r = (packed >> 16) & 0xFF;
  g = (packed >> 8) & 0xFF;
  b = packed & 0xFF;
}

/**
 * @brief Pack gamma-corrected RGB values into 12-bit framebuffer format
 *
 * Used for bit_depth 9-12.
 *
 * @param r_corrected Gamma-corrected red value (0-4095)
 * @param g_corrected Gamma-corrected green value (0-4095)
 * @param b_corrected Gamma-corrected blue value (0-4095)
 * @return 64-bit packed value ready for framebuffer
 */
HUB75_IRAM inline uint64_t pack_rgb121212(uint16_t r_corrected, uint16_t g_corrected, uint16_t b_corrected) {
  // Pack into 64-bit word: [unused 16 bits][R 16 bits][G 16 bits][B 16 bits]
  return ((uint64_t) (r_corrected & 0xFFF) << 32) | ((uint64_t) (g_corrected & 0xFFF) << 16) | (b_corrected & 0xFFF);
}

/**
 * @brief Convert RGB888 to 12-bit framebuffer format
 *
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 * @param lut Gamma correction LUT (returns 12-bit values)
 * @return 64-bit packed value ready for framebuffer
 */
HUB75_IRAM inline uint64_t rgb888_to_native_12bit(uint8_t r, uint8_t g, uint8_t b, const uint16_t *lut) {
  // Apply gamma correction via LUT (returns 12-bit values)
  uint16_t r_corrected = lut[r];
  uint16_t g_corrected = lut[g];
  uint16_t b_corrected = lut[b];

  return pack_rgb121212(r_corrected, g_corrected, b_corrected);
}

/**
 * @brief Convert RGB565 to 12-bit framebuffer format
 *
 * @param rgb565 RGB565 color value
 * @param lut Gamma correction LUT (returns 12-bit values)
 * @param big_endian True if RGB565 is big-endian
 * @return 64-bit packed value ready for framebuffer
 */
HUB75_IRAM inline uint64_t rgb565_to_native_12bit(uint16_t rgb565, const uint16_t *lut, bool big_endian) {
  uint8_t r5, g6, b5;

  if (big_endian) {
    // Big-endian (byte-swapped): Bytes are GGGBBBBB RRRRRGGG
    // Swap bytes first to get standard bit order
    rgb565 = (rgb565 >> 8) | (rgb565 << 8);
    // Now fall through to standard extraction
  }

  // Standard RGB565 bit layout: RRRRRGGG GGGBBBBB (MSB first)
  r5 = (rgb565 >> 11) & 0x1F;  // Top 5 bits
  g6 = (rgb565 >> 5) & 0x3F;   // Middle 6 bits
  b5 = rgb565 & 0x1F;          // Bottom 5 bits

  // Scale to 8-bit (5-bit: x33, 6-bit: x4)
  uint8_t r8 = (r5 << 3) | (r5 >> 2);  // 5-bit to 8-bit
  uint8_t g8 = (g6 << 2) | (g6 >> 4);  // 6-bit to 8-bit
  uint8_t b8 = (b5 << 3) | (b5 >> 2);  // 5-bit to 8-bit

  // Use RGB888 conversion
  return rgb888_to_native_12bit(r8, g8, b8, lut);
}

/**
 * @brief Unpack RGB121212 framebuffer value to RGB components (for BCM extraction)
 *
 * This is used by the refresh loop to extract gamma-corrected RGB values
 * before BCM bit-plane extraction.
 *
 * @param packed Packed framebuffer value (64-bit)
 * @param r Output: red component (0-4095)
 * @param g Output: green component (0-4095)
 * @param b Output: blue component (0-4095)
 */
HUB75_IRAM inline void unpack_rgb121212(uint64_t packed, uint16_t &r, uint16_t &g, uint16_t &b) {
  r = (packed >> 32) & 0xFFF;
  g = (packed >> 16) & 0xFFF;
  b = packed & 0xFFF;
}

// Legacy alias for backward compatibility (assumes RGB888 format)
HUB75_IRAM inline void unpack_rgb(uint32_t packed, uint16_t &r, uint16_t &g, uint16_t &b) {
  unpack_rgb888(packed, r, g, b);
}

/**
 * @brief Extract single bit from RGB values for BCM bit-plane
 *
 * Used by refresh loop to generate DMA data for specific bit depth.
 *
 * @param r Red value (gamma-corrected)
 * @param g Green value (gamma-corrected)
 * @param b Blue value (gamma-corrected)
 * @param bit_index Bit index to extract (0 = LSB)
 * @param is_upper_half True for R1/G1/B1 (upper half), false for R2/G2/B2 (lower half)
 * @return 6-bit value with RGB bits set for DMA output
 *
 * Output bit layout (for upper half, R1/G1/B1):
 *   Bit 2: R1
 *   Bit 1: G1
 *   Bit 0: B1
 *
 * Output bit layout (for lower half, R2/G2/B2):
 *   Bit 5: R2
 *   Bit 4: G2
 *   Bit 3: B2
 */
HUB75_IRAM inline uint8_t extract_bcm_bits(uint16_t r, uint16_t g, uint16_t b, uint8_t bit_index, bool is_upper_half) {
  uint16_t mask = (1 << bit_index);
  uint8_t rgb_bits = 0;

  // Extract bit from each color channel
  if (r & mask)
    rgb_bits |= 0x04;  // R bit
  if (g & mask)
    rgb_bits |= 0x02;  // G bit
  if (b & mask)
    rgb_bits |= 0x01;  // B bit

  if (!is_upper_half) {
    // Shift to R2/G2/B2 position (bits 5:3)
    rgb_bits <<= 3;
  }

  return rgb_bits;
}

}  // namespace hub75
