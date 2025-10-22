/**
 * SPDX-FileCopyrightText: 2025 Stuart Parmenter
 * SPDX-License-Identifier: MIT
 *
 * @file dma_parlio.hpp
 * @brief ESP32-P4 PARLIO peripheral for HUB75 with clock gating
 *
 * Uses ESP32-P4's PARLIO TX peripheral with clock gating feature to
 * embed BCM timing directly in buffer data via MSB bit control.
 */

#pragma once

#include "hub75_types.h"
#include "hub75_config.h"
#include "hub75_internal.h"
#include "../platform_dma.hpp"
#include <cstddef>
#include <driver/parlio_tx.h>

namespace hub75 {

/**
 * @brief ESP32-P4 PARLIO TX implementation for HUB75
 *
 * Uses clock gating feature where MSB bit controls PCLK output:
 * - MSB=1: Clock enabled, data shifts to panel
 * - MSB=0: Clock disabled, panel displays latched data
 *
 * This allows BCM timing to be embedded in the buffer itself via
 * padding words with MSB=0, eliminating descriptor repetition.
 */
class ESP32P4_PARLIO_DMA : public PlatformDMA {
 public:
  ESP32P4_PARLIO_DMA(const hub75_config_t &config);
  ~ESP32P4_PARLIO_DMA();

  bool init() override;
  void shutdown() override;
  bool setupCircularDMA(uint8_t *buffer_a, uint8_t *buffer_b, size_t size) override;
  void startTransfer() override;
  void stopTransfer() override;
  void setBasisBrightness(uint8_t brightness) override;
  void setIntensity(float intensity) override;

  void drawPixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer, hub75_pixel_format_t format,
                  hub75_color_order_t color_order, bool big_endian) override;
  void setLUT(const uint16_t *lut) override;
  void clear() override;

  struct BitPlaneBuffer {
    uint16_t *data;
    size_t pixel_words;
    size_t padding_words;
    size_t total_words;
  };

  struct RowBitPlaneBuffers {
    BitPlaneBuffer *bit_planes;
  };

 private:
  void configurePARLIO();
  void configureGPIO();
  bool allocateRowBuffers();
  void initializeBlankBuffers();
  void setBrightnessOE();
  bool buildTransactionQueue();
  void calculateBCMTimings();
  size_t calculateBCMPadding(uint8_t bit_plane);

  inline void setClockEnable(uint16_t &word, bool enable) { word = enable ? (word | 0x8000) : (word & 0x7FFF); }

  hub75_config_t config_;
  parlio_tx_unit_handle_t tx_unit_;
  parlio_transmit_config_t transmit_config_;
  uint8_t bit_depth_;
  uint8_t lsbMsbTransitionBit_;
  uint16_t num_rows_;
  RowBitPlaneBuffers *row_buffers_;
  uint8_t basis_brightness_;
  float intensity_;
  const uint16_t *lut_;
  bool transfer_started_;
};

}  // namespace hub75
