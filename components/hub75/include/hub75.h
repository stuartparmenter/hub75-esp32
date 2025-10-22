/**
 * SPDX-FileCopyrightText: 2025 Stuart Parmenter
 * SPDX-License-Identifier: MIT
 *
 * @file hub75.h
 * @brief Main public API for ESP32 HUB75 DMA Driver
 *
 * High-performance DMA-based driver for HUB75 RGB LED matrix panels.
 * Supports ESP32, ESP32-S2, ESP32-S3, ESP32-C6, and ESP32-P4.
 *
 * Features:
 * - Continuous automatic refresh (no manual render() calls)
 * - CIE 1931 gamma correction with native bit-depth LUTs
 * - Double buffering for tear-free animation
 * - Temporal dithering for improved gradients
 * - RGB888 and RGB565 input formats
 * - Configurable brightness and bit depth
 *
 * Basic Usage:
 * @code
 * #include "hub75.h"
 *
 * hub75_config_t config = HUB75_CONFIG_DEFAULT();
 * config.pins.r1 = 25;
 * config.pins.g1 = 26;
 * // ... configure pins ...
 *
 * HUB75Driver driver(config);
 * driver.begin();  // Starts continuous refresh
 *
 * // Just draw - changes appear automatically
 * driver.setPixel(10, 10, 255, 0, 0);  // Red pixel
 * driver.setPixel(20, 20, 0, 255, 0);  // Green pixel
 * @endcode
 */

#pragma once

// ESP-IDF attribute macros (needed for IRAM_ATTR)
#if __has_include(<esp_attr.h>)
#include <esp_attr.h>
#endif

#include "hub75_types.h"
#include "hub75_config.h"
#include "hub75_internal.h"  // Internal types (framebuffer format)
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus

// Forward declarations
namespace hub75 {
class PlatformDMA;
}  // namespace hub75

/**
 * @brief Main HUB75 driver class
 */
class HUB75Driver {
 public:
  /**
   * @brief Construct a new HUB75 driver
   * @param config Driver configuration
   */
  explicit HUB75Driver(const hub75_config_t &config);

  /**
   * @brief Destructor - stops refresh and frees resources
   */
  ~HUB75Driver();

  /**
   * @brief Initialize hardware and start continuous refresh
   * @return true on success, false on error
   */
  bool begin();

  /**
   * @brief Stop refresh and release hardware resources
   */
  void end();

  // ========================================================================
  // Pixel Drawing API
  // ========================================================================

  /**
   * @brief Draw a rectangular region of pixels from a buffer (bulk operation)
   * @param x X coordinate (top-left)
   * @param y Y coordinate (top-left)
   * @param w Width in pixels
   * @param h Height in pixels
   * @param buffer Pointer to pixel data
   * @param format Pixel format (RGB888, RGB888_32, or RGB565)
   * @param color_order Color component order (RGB or BGR, for RGB888_32 only)
   * @param big_endian True if buffer is big-endian (affects RGB565 and RGB888_32)
   *
   * Buffer stride is assumed to be w pixels (tightly packed rows).
   *
   * Format details:
   * - RGB888: 24-bit packed RGB (3 bytes/pixel: R, G, B)
   * - RGB888_32: 32-bit RGB with padding (4 bytes/pixel: x, R, G, B or B, G, R, x)
   * - RGB565: 16-bit RGB565 (2 bytes/pixel)
   *
   * This is the most efficient way to draw multiple pixels.
   */
  void drawPixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer, hub75_pixel_format_t format,
                  hub75_color_order_t color_order = HUB75_COLOR_ORDER_RGB, bool big_endian = false);

  /**
   * @brief Set a single pixel (RGB888 input)
   * @param x X coordinate
   * @param y Y coordinate
   * @param r Red component (0-255)
   * @param g Green component (0-255)
   * @param b Blue component (0-255)
   *
   * Note: This is a convenience wrapper around drawPixels() for single-pixel operations.
   * For drawing multiple pixels, use drawPixels() directly for better performance.
   */
  void setPixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b);

  /**
   * @brief Clear entire display to black
   */
  void clear();

  // ========================================================================
  // Double Buffering API (if enabled in config)
  // ========================================================================

  /**
   * @brief Clear the back buffer (double buffer mode only)
   */
  void clearBuffer();

  /**
   * @brief Swap front and back buffers atomically (double buffer mode only)
   * Swap happens on next refresh cycle for tear-free animation
   */
  void flipBuffer();

  // ========================================================================
  // Color Configuration
  // ========================================================================

  /**
   * @brief Set global brightness (legacy single-mode control)
   * @param brightness Brightness level (0-255)
   * @note This sets the basis brightness. For dual-mode control, use
   *       setBasisBrightness() and setIntensity() separately.
   */
  void setBrightness(uint8_t brightness);

  /**
   * @brief Get current brightness
   * @return Current brightness (0-255)
   */
  uint8_t getBrightness() const;

  /**
   * @brief Set basis brightness (dual-mode: coarse control)
   * @param brightness Brightness level (1-255, affects BCM timing)
   * @note This adjusts the fundamental BCM timer periods. Higher values = brighter.
   *       Combines with intensity for final brightness. Default: 255
   */
  void setBasisBrightness(uint8_t brightness);

  /**
   * @brief Set intensity (dual-mode: fine control)
   * @param intensity Intensity level (0.0-1.0, smooth scaling)
   * @note This provides smooth runtime dimming without changing refresh rate.
   *       Multiplies with basis brightness for final brightness. Default: 1.0
   */
  void setIntensity(float intensity);

  /**
   * @brief Set gamma correction mode
   * @param mode Gamma mode
   */
  void setGammaMode(hub75_gamma_mode_t mode);

  /**
   * @brief Get current gamma mode
   * @return Current gamma mode
   */
  hub75_gamma_mode_t getGammaMode() const;

  // ========================================================================
  // Information
  // ========================================================================

  /**
   * @brief Get panel width in pixels
   * @return Width
   */
  uint16_t getWidth() const;

  /**
   * @brief Get panel height in pixels
   * @return Height
   */
  uint16_t getHeight() const;

  /**
   * @brief Get actual refresh rate (Hz)
   * @return Measured refresh rate
   */
  float getRefreshRate() const;

  /**
   * @brief Check if driver is running
   * @return true if refresh loop is active
   */
  bool isRunning() const;

 private:
  hub75_config_t config_;
  bool running_;

  // Color conversion LUTs
  const uint16_t *lut_;

  // Platform-specific DMA engine
  hub75::PlatformDMA *dma_;

  // Helper methods
  void initializeLUT();

  HUB75_IRAM inline size_t getPixelOffset(uint16_t x, uint16_t y) const {
    // Total width includes all chained panels
    return y * (config_.width * config_.chain_length) + x;
  }
};

#endif  // __cplusplus

// ============================================================================
// C API (for compatibility)
// ============================================================================

#ifdef __cplusplus
extern "C" {
#endif

// C API will be added later if needed for Arduino/PlatformIO compatibility

#ifdef __cplusplus
}
#endif
