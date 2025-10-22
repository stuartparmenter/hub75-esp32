// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file parlio_dma.hpp
// @brief ESP32-P4 PARLIO peripheral for HUB75 with clock gating
//
// Uses ESP32-P4's PARLIO TX peripheral with clock gating feature to
// embed BCM timing directly in buffer data via MSB bit control.

#pragma once

#include "hub75_types.h"
#include "hub75_config.h"
#include "hub75_internal.h"
#include "../platform_dma.h"
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
class ParlioDma : public PlatformDma {
 public:
  ParlioDma(const Hub75Config &config);
  ~ParlioDma();

  bool init() override;
  void shutdown() override;
  bool setup_dma(uint8_t *buffer_a, uint8_t *buffer_b, size_t size) override;
  void start_transfer() override;
  void stop_transfer() override;
  void set_basis_brightness(uint8_t brightness) override;
  void set_intensity(float intensity) override;

  void draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer, Hub75PixelFormat format,
                   Hub75ColorOrder color_order, bool big_endian) override;
  void set_lut(const uint16_t *lut) override;
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
  void configure_parlio();
  void configure_gpio();
  bool allocate_row_buffers();
  void initialize_blank_buffers();
  void set_brightness_oe();
  bool build_transaction_queue();
  void calculate_bcm_timings();
  size_t calculate_bcm_padding(uint8_t bit_plane);

  inline void set_clock_enable(uint16_t &word, bool enable) { word = enable ? (word | 0x8000) : (word & 0x7FFF); }

  const Hub75Config config_;
  parlio_tx_unit_handle_t tx_unit_;
  parlio_transmit_config_t transmit_config_;
  const uint8_t bit_depth_;
  uint8_t lsbMsbTransitionBit_;  // Calculated at init

  // Panel configuration (immutable, cached from config)
  const uint16_t panel_width_;
  const uint16_t panel_height_;
  const uint16_t layout_rows_;
  const uint16_t layout_cols_;
  const uint16_t virtual_width_;   // Computed: panel_width * layout_cols
  const uint16_t virtual_height_;  // Computed: panel_height * layout_rows

  // Coordinate transformation (immutable, cached from config)
  const ScanPattern scan_wiring_;
  const PanelLayout layout_;

  // Optimization flags (immutable, for branch prediction)
  const bool needs_scan_remap_;
  const bool needs_layout_remap_;

  const uint16_t num_rows_;  // Computed: panel_height / 2
  RowBitPlaneBuffers *row_buffers_;
  uint8_t basis_brightness_;
  float intensity_;
  const uint16_t *lut_;
  bool transfer_started_;
};

}  // namespace hub75
