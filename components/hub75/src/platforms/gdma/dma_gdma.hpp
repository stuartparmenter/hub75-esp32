/**
 * SPDX-FileCopyrightText: 2025 Stuart Parmenter
 * SPDX-License-Identifier: MIT
 *
 * @file dma_gdma.hpp
 * @brief ESP32-S3 LCD_CAM peripheral + GDMA for HUB75
 *
 * Uses ESP32-S3's LCD_CAM peripheral with direct register access
 * and manual GDMA setup for continuous data transfer.
 */

#pragma once

#include "hub75_types.h"
#include "hub75_config.h"
#include "hub75_internal.h"  // For hub75_framebuffer_format_t
#include "../platform_dma.hpp"
#include <cstddef>
#include <variant>
#include <esp_private/gdma.h>
#include <hal/dma_types.h>

namespace hub75 {

// Forward declaration
class Framebuffer;

/**
 * @brief ESP32-S3 GDMA + LCD_CAM implementation for HUB75
 */
class ESP32S3_GDMA : public PlatformDMA {
 public:
  ESP32S3_GDMA(const hub75_config_t &config);
  ~ESP32S3_GDMA();

  /**
   * @brief Initialize LCD_CAM peripheral with GDMA
   */
  bool init() override;

  /**
   * @brief Shutdown LCD_CAM and free GDMA resources
   */
  void shutdown() override;

  /**
   * @brief Setup DMA with descriptor chain for BCM
   */
  bool setupCircularDMA(uint8_t *buffer_a, uint8_t *buffer_b, size_t size) override;

  /**
   * @brief Start DMA transfer (starts once, runs continuously)
   */
  void startTransfer() override;

  /**
   * @brief Stop DMA transfer
   */
  void stopTransfer() override;

  /**
   * @brief Set basis brightness (override base class)
   */
  void setBasisBrightness(uint8_t brightness) override;

  /**
   * @brief Set intensity (override base class)
   */
  void setIntensity(float intensity) override;

  // ============================================================================
  // Pixel API (Direct DMA Buffer Writes)
  // ============================================================================

  /**
   * @brief Draw pixels from buffer (bulk operation, writes directly to DMA buffers)
   */
  void drawPixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer, hub75_pixel_format_t format,
                  hub75_color_order_t color_order, bool big_endian) override;

  /**
   * @brief Set gamma correction LUT
   */
  void setLUT(const uint16_t *lut) override;

  /**
   * @brief Clear all pixels to black
   */
  void clear() override;

  // Per-row buffer structure (holds all bit planes for one row)
  struct RowBitPlaneBuffer {
    uint8_t *data;       // Contiguous buffer: [bit0 pixels][bit1 pixels]...[bitN pixels]
    size_t buffer_size;  // Total size in bytes
  };

 private:
  void configureLCDClock();
  void configureLCDMode();
  void configureGPIO();

  // Buffer management
  bool allocateRowBuffers();
  void initializeBlankBuffers();  // Initialize DMA buffers with control bits only
  void setBrightnessOE();         // Set OE bits for BCM control
  bool buildDescriptorChain();

  // BCM timing calculation (calculates lsbMsbTransitionBit for OE control)
  void calculateBCMTimings();

  hub75_config_t config_;
  gdma_channel_handle_t dma_chan_;
  uint8_t bit_depth_;            // Bit depth from config (6, 7, 8, 10, or 12)
  uint8_t lsbMsbTransitionBit_;  // BCM optimization threshold (used by setBrightnessOE)

  // Row buffers (one per physical row, contains all bit planes)
  RowBitPlaneBuffer *row_buffers_;
  uint16_t num_rows_;

  // Descriptor chain (contiguous array with BCM repetitions)
  dma_descriptor_t *descriptors_;  // Pointer to contiguous array, not array of pointers
  size_t descriptor_count_;

  // Brightness control (implementation of base class interface)
  uint8_t basis_brightness_;  // 1-255
  float intensity_;           // 0.0-1.0

  // Gamma correction LUT (owned by HUB75Driver, just a pointer here)
  const uint16_t *lut_;
};

}  // namespace hub75
