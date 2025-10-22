/**
 * SPDX-FileCopyrightText: 2025 Stuart Parmenter
 * SPDX-License-Identifier: MIT
 *
 * @file platform_dma.hpp
 * @brief Platform-agnostic DMA interface
 *
 * This header provides a common interface that all platform-specific
 * DMA implementations must follow.
 */

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
class PlatformDMA {
 public:
  virtual ~PlatformDMA() = default;

  /**
   * @brief Initialize the DMA engine
   */
  virtual bool init() = 0;

  /**
   * @brief Shutdown the DMA engine
   */
  virtual void shutdown() = 0;

  /**
   * @brief Setup circular DMA with two buffers
   * @param buffer_a First DMA buffer
   * @param buffer_b Second DMA buffer
   * @param size Size of each buffer in bytes
   */
  virtual bool setupCircularDMA(uint8_t *buffer_a, uint8_t *buffer_b, size_t size) = 0;

  /**
   * @brief Start DMA transfers
   */
  virtual void startTransfer() = 0;

  /**
   * @brief Stop DMA transfers
   */
  virtual void stopTransfer() = 0;

  /**
   * @brief Set basis brightness (coarse control, affects BCM timing)
   *
   * Brightness affects the display period for each bit plane. Platform implementations
   * may use different mechanisms (VBK cycles, buffer padding, etc.) to achieve this.
   *
   * @param brightness Brightness level (1-255, where 255 is maximum)
   */
  virtual void setBasisBrightness(uint8_t brightness) = 0;

  /**
   * @brief Set intensity (fine control, runtime scaling)
   *
   * Intensity provides smooth dimming without affecting refresh rate calculations.
   * Applied as a multiplier to the basis brightness.
   *
   * @param intensity Intensity multiplier (0.0-1.0, where 1.0 is maximum)
   */
  virtual void setIntensity(float intensity) = 0;

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
  virtual void drawPixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer,
                          hub75_pixel_format_t format, hub75_color_order_t color_order, bool big_endian) {
    // Default: no-op (platforms using framebuffer don't need this)
  }

  /**
   * @brief Set gamma correction LUT
   * @param lut Pointer to 256-entry LUT (8-bit input -> corrected output)
   */
  virtual void setLUT(const uint16_t *lut) {
    // Default: no-op
  }

  /**
   * @brief Clear all pixels to black
   */
  virtual void clear() {
    // Default: no-op
  }
};

}  // namespace hub75
