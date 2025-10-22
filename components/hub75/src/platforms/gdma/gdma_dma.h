// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file gdma_dma.hpp
// @brief ESP32-S3 LCD_CAM peripheral + GDMA for HUB75
//
// Uses ESP32-S3's LCD_CAM peripheral with direct register access
// and manual GDMA setup for continuous data transfer.

#pragma once

#include "hub75_types.h"
#include "hub75_config.h"
#include "hub75_internal.h"  // For Hub75FramebufferFormat
#include "../platform_dma.h"
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
class GdmaDma : public PlatformDma {
 public:
  GdmaDma(const Hub75Config &config);
  ~GdmaDma();

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
  bool setup_dma(uint8_t *buffer_a, uint8_t *buffer_b, size_t size) override;

  /**
   * @brief Start DMA transfer (starts once, runs continuously)
   */
  void start_transfer() override;

  /**
   * @brief Stop DMA transfer
   */
  void stop_transfer() override;

  /**
   * @brief Set basis brightness (override base class)
   */
  void set_basis_brightness(uint8_t brightness) override;

  /**
   * @brief Set intensity (override base class)
   */
  void set_intensity(float intensity) override;

  // ============================================================================
  // Pixel API (Direct DMA Buffer Writes)
  // ============================================================================

  /**
   * @brief Draw pixels from buffer (bulk operation, writes directly to DMA buffers)
   */
  void draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer, Hub75PixelFormat format,
                   Hub75ColorOrder color_order, bool big_endian) override;

  /**
   * @brief Set gamma correction LUT
   */
  void set_lut(const uint16_t *lut) override;

  /**
   * @brief Clear all pixels to black
   */
  void clear() override;

  // ============================================================================
  // Static Helper Functions (Public for compile-time validation)
  // ============================================================================

  /**
   * @brief Calculate BCM transmissions per row for given bit depth and transition bit
   */
  static constexpr int calculate_bcm_transmissions(int bit_depth, int lsb_msb_transition);

  // Per-row buffer structure (holds all bit planes for one row)
  struct RowBitPlaneBuffer {
    uint8_t *data;       // Contiguous buffer: [bit0 pixels][bit1 pixels]...[bitN pixels]
    size_t buffer_size;  // Total size in bytes
  };

 private:
  void configure_lcd_clock();
  void configure_lcd_mode();
  void configure_gpio();

  // Buffer management
  bool allocate_row_buffers();
  bool validate_brightness_config();  // Validate safety margins for brightness OE configuration
  void initialize_blank_buffers();    // Initialize DMA buffers with control bits only
  void set_brightness_oe();           // Set OE bits for BCM control
  bool build_descriptor_chain();

  // BCM timing calculation (calculates lsbMsbTransitionBit for OE control)
  void calculate_bcm_timings();

  const Hub75Config config_;
  gdma_channel_handle_t dma_chan_;
  const uint8_t bit_depth_;      // Bit depth from config (6, 7, 8, 10, or 12)
  uint8_t lsbMsbTransitionBit_;  // BCM optimization threshold (calculated at init)

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

  // Row buffers (one per physical row, contains all bit planes)
  RowBitPlaneBuffer *row_buffers_;
  const uint16_t num_rows_;  // Computed: panel_height / 2

  // Descriptor chain (contiguous array with BCM repetitions)
  dma_descriptor_t *descriptors_;  // Pointer to contiguous array, not array of pointers
  size_t descriptor_count_;

  // Brightness control (implementation of base class interface)
  uint8_t basis_brightness_;  // 1-255
  float intensity_;           // 0.0-1.0

  // Gamma correction LUT (owned by Hub75Driver, just a pointer here)
  const uint16_t *lut_;
};

}  // namespace hub75
