// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file platform_dma.hpp
// @brief Platform-agnostic DMA interface
//
// This header provides a common interface that all platform-specific
// DMA implementations must follow.

#pragma once

#include <sdkconfig.h>

#include "hub75_types.h"
#include "hub75_config.h"
#include <stdint.h>
#include <stddef.h>

namespace hub75 {

/**
 * @brief Platform-agnostic DMA interface
 *
 * Each platform (ESP32, ESP32-S3, etc.) implements this interface
 * with their specific DMA hardware (I2S DMA, GDMA, etc.)
 */
class PlatformDma {
 public:
  virtual ~PlatformDma() = default;

  /**
   * @brief Initialize the DMA engine
   */
  virtual bool init() = 0;

  /**
   * @brief Shutdown the DMA engine
   */
  virtual void shutdown() = 0;

  /**
   * @brief Start DMA transfers
   */
  virtual void start_transfer() = 0;

  /**
   * @brief Stop DMA transfers
   */
  virtual void stop_transfer() = 0;

  /**
   * @brief Set basis brightness (coarse control, affects BCM timing)
   *
   * Brightness affects the display period for each bit plane. Platform implementations
   * may use different mechanisms (VBK cycles, buffer padding, etc.) to achieve this.
   *
   * @param brightness Brightness level (1-255, where 255 is maximum)
   */
  virtual void set_basis_brightness(uint8_t brightness) = 0;

  /**
   * @brief Set intensity (fine control, runtime scaling)
   *
   * Intensity provides smooth dimming without affecting refresh rate calculations.
   * Applied as a multiplier to the basis brightness.
   *
   * @param intensity Intensity multiplier (0.0-1.0, where 1.0 is maximum)
   */
  virtual void set_intensity(float intensity) = 0;

  // ============================================================================
  // Pixel API (for platforms that support direct DMA buffer writes)
  // ============================================================================

  /**
   * @brief Draw a rectangular region of pixels (bulk operation)
   * @param x X coordinate (top-left)
   * @param y Y coordinate (top-left)
   * @param w Width in pixels
   * @param h Height in pixels
   * @param buffer Pointer to pixel data (tightly packed, w*h pixels)
   * @param format Pixel format
   * @param color_order Color component order (RGB or BGR, for RGB888_32 only)
   * @param big_endian True if buffer is big-endian
   *
   * This is the primary pixel drawing function. Single-pixel operations
   * should call this with w=h=1 for consistency.
   */
  virtual void draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer,
                           Hub75PixelFormat format, Hub75ColorOrder color_order, bool big_endian) {
    // Default: no-op (platforms using framebuffer don't need this)
  }

  /**
   * @brief Set gamma correction LUT
   * @param lut Pointer to 256-entry LUT (8-bit input -> corrected output)
   */
  virtual void set_lut(const uint16_t *lut) {
    // Default: no-op
  }

  /**
   * @brief Clear all pixels to black
   *
   * In single-buffer mode: Clears the visible display immediately.
   * In double-buffer mode: Clears the back buffer (requires flip to display).
   */
  virtual void clear() {
    // Default: no-op
  }

  /**
   * @brief Swap front and back buffers (double buffer mode only)
   *
   * In single-buffer mode: No-op
   * In double-buffer mode: Atomically swaps active and back buffers
   *
   * Platform-specific implementations:
   * - PARLIO: Queues next buffer via parlio_tx_unit_transmit()
   * - GDMA: Updates descriptor chain pointers
   * - I2S: Updates descriptor chain pointers
   */
  virtual void flip_buffer() {
    // Default: no-op (single buffer mode or not implemented)
  }
};

}  // namespace hub75
