/**
 * SPDX-FileCopyrightText: 2025 Stuart Parmenter
 * SPDX-License-Identifier: MIT
 *
 * @file dma_i2s.hpp
 * @brief ESP32/ESP32-S2 I2S DMA engine (LCD mode)
 *
 * Self-contained I2S+DMA implementation with BCM, pixel operations,
 * and brightness control (mirroring GDMA architecture).
 */

#pragma once

#include "hub75_types.h"
#include "hub75_config.h"
#include "hub75_internal.h"
#include "../platform_dma.hpp"
#include <cstddef>
#include <rom/lldesc.h>
#include <soc/i2s_struct.h>

namespace hub75 {

/**
 * @brief ESP32/ESP32-S2 I2S DMA implementation for HUB75
 *
 * Uses I2S peripheral in LCD mode (16-bit parallel output) with DMA.
 * Implements full BCM refresh, pixel operations, and brightness control.
 */
class ESP32_I2S_DMA : public PlatformDMA {
 public:
  ESP32_I2S_DMA(const hub75_config_t &config);
  ~ESP32_I2S_DMA();

  /**
   * @brief Initialize I2S peripheral in LCD mode
   */
  bool init() override;

  /**
   * @brief Shutdown I2S and free DMA resources
   */
  void shutdown() override;

  /**
   * @brief Setup circular DMA with BCM descriptor chain
   */
  bool setupCircularDMA(uint8_t *buffer_a, uint8_t *buffer_b, size_t size) override;

  /**
   * @brief Start DMA transfer (continuous loop)
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
  void configureI2STiming();
  void configureGPIO();

  // Buffer management
  bool allocateRowBuffers();
  void initializeBlankBuffers();  // Initialize DMA buffers with control bits only
  void setBrightnessOE();         // Set OE bits for BCM control
  bool buildDescriptorChain();

  // BCM timing calculation (calculates lsbMsbTransitionBit for OE control)
  void calculateBCMTimings();

  hub75_config_t config_;
  volatile i2s_dev_t *i2s_dev_;
  uint8_t bit_depth_;            // Bit depth from config (6, 7, 8, 10, or 12)
  uint8_t lsbMsbTransitionBit_;  // BCM optimization threshold (used by setBrightnessOE)

  // Row buffers (one per physical row, contains all bit planes)
  RowBitPlaneBuffer *row_buffers_;
  uint16_t num_rows_;

  // Descriptor chain (contiguous array with BCM repetitions)
  lldesc_t *descriptors_;  // Pointer to contiguous array
  size_t descriptor_count_;

  // Brightness control (implementation of base class interface)
  uint8_t basis_brightness_;  // 1-255
  float intensity_;           // 0.0-1.0

  // Gamma correction LUT (owned by HUB75Driver, just a pointer here)
  const uint16_t *lut_;
};

}  // namespace hub75
