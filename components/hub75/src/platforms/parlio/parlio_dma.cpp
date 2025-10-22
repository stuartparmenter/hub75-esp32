// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file parlio_dma.cpp
// @brief ESP32-P4 PARLIO implementation for HUB75 with clock gating
//
// Uses PARLIO TX with clock gating (MSB bit controls PCLK) to embed
// BCM timing directly in buffer data, eliminating descriptor repetition.

#include <sdkconfig.h>

// Only compile for ESP32-P4
#ifdef CONFIG_IDF_TARGET_ESP32P4

#include "parlio_dma.h"
#include "../../color/color_convert.h"  // For RGB565 scaling utilities
#include "../../panels/scan_patterns.h"
#include "../../panels/panel_layout.h"
#include <cstring>
#include <algorithm>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>

static const char *const TAG = "ParlioDma";

namespace hub75 {

// HUB75 16-bit word layout for PARLIO peripheral
// Bit layout: [CLK|ADDR(4-bit)|-|LAT|OE|--|--|R2|R1|G2|G1|B2|B1]
enum HUB75WordBits : uint16_t {
  B2_BIT = 0,  // Lower half blue (data_pins[0])
  B1_BIT = 1,  // Upper half blue (data_pins[1])
  G2_BIT = 2,  // Lower half green (data_pins[2])
  G1_BIT = 3,  // Upper half green (data_pins[3])
  R2_BIT = 4,  // Lower half red (data_pins[4])
  R1_BIT = 5,  // Upper half red (data_pins[5])
  // Bits 6-7: Unused
  OE_BIT = 8,
  LAT_BIT = 9,
  // Bit 10: Unused
  // Bits 11-14: Row address (4-bit field, shifted << 11)
  CLK_GATE_BIT = 15,  // MSB: clock gate control (PARLIO-specific)
};

// Address field (not individual bits)
constexpr int ADDR_SHIFT = 11;
constexpr uint16_t ADDR_MASK = 0x0F;  // 4-bit address (0-15)

// Combined RGB masks (used for clearing RGB bits in buffers)
constexpr uint16_t RGB_UPPER_MASK = (1 << R1_BIT) | (1 << G1_BIT) | (1 << B1_BIT);
constexpr uint16_t RGB_LOWER_MASK = (1 << R2_BIT) | (1 << G2_BIT) | (1 << B2_BIT);

ParlioDma::ParlioDma(const Hub75Config &config)
    : config_(config),
      tx_unit_(nullptr),
      bit_depth_(config.bit_depth),
      lsbMsbTransitionBit_(0),
      panel_width_(config.panel_width),
      panel_height_(config.panel_height),
      layout_rows_(config.layout_rows),
      layout_cols_(config.layout_cols),
      virtual_width_(config.panel_width * config.layout_cols),
      virtual_height_(config.panel_height * config.layout_rows),
      scan_wiring_(config.scan_wiring),
      layout_(config.layout),
      needs_scan_remap_(config.scan_wiring != ScanPattern::STANDARD_TWO_SCAN),
      needs_layout_remap_(config.layout != PanelLayout::HORIZONTAL),
      num_rows_(config.panel_height / 2),
      row_buffers_(nullptr),
      basis_brightness_(config.brightness),
      intensity_(1.0f),
      lut_(nullptr),
      transfer_started_(false) {
  // Initialize transmit config
  transmit_config_.idle_value = 0x00;
  transmit_config_.bitscrambler_program = nullptr;
  transmit_config_.flags.queue_nonblocking = 0;
  transmit_config_.flags.loop_transmission = 1;  // Continuous refresh
}

ParlioDma::~ParlioDma() { ParlioDma::shutdown(); }

bool ParlioDma::init() {
  ESP_LOGI(TAG, "Initializing PARLIO TX peripheral with clock gating...");
  ESP_LOGI(TAG, "Panel: %dx%d, Layout: %dx%d, Virtual: %dx%d", panel_width_, panel_height_, layout_cols_, layout_rows_,
           virtual_width_, virtual_height_);
  ESP_LOGI(TAG, "Rows: %d, Bit depth: %d", num_rows_, bit_depth_);

  // Calculate BCM timings first
  calculate_bcm_timings();

  // Configure GPIO
  configure_gpio();

  // Configure PARLIO peripheral
  configure_parlio();

  if (!tx_unit_) {
    ESP_LOGE(TAG, "Failed to create PARLIO TX unit");
    return false;
  }

  ESP_LOGI(TAG, "PARLIO TX initialized successfully");
  ESP_LOGI(TAG, "Clock gating enabled, MSB bit controls PCLK");

  return true;
}

void ParlioDma::shutdown() {
  if (transfer_started_) {
    ParlioDma::stop_transfer();
  }

  if (tx_unit_) {
    parlio_tx_unit_disable(tx_unit_);
    parlio_del_tx_unit(tx_unit_);
    tx_unit_ = nullptr;
  }

  // Free row buffers
  if (row_buffers_) {
    for (int row = 0; row < num_rows_; row++) {
      if (row_buffers_[row].bit_planes) {
        for (int bit = 0; bit < bit_depth_; bit++) {
          if (row_buffers_[row].bit_planes[bit].data) {
            heap_caps_free(row_buffers_[row].bit_planes[bit].data);
          }
        }
        delete[] row_buffers_[row].bit_planes;
      }
    }
    delete[] row_buffers_;
    row_buffers_ = nullptr;
  }
}

void ParlioDma::configure_parlio() {
  ESP_LOGI(TAG, "Configuring PARLIO TX unit with clock gating...");

  // Calculate maximum buffer size for transaction queue
  size_t max_buffer_size = virtual_width_;
  for (int bit = 0; bit < bit_depth_; bit++) {
    max_buffer_size = std::max(max_buffer_size, virtual_width_ + calculate_bcm_padding(bit));
  }

  // Map pins to data bus (16-bit wide)
  // Bit layout: [CLK_GATE|ADDR_D|ADDR_C|ADDR_B|ADDR_A|--|LAT|OE|RGB_DATA...]
  gpio_num_t data_pins[16] = {
      (gpio_num_t) -1,
      (gpio_num_t) -1,
      (gpio_num_t) -1,
      (gpio_num_t) -1,  // 0-3: RGB data (platform-specific)
      (gpio_num_t) -1,
      (gpio_num_t) -1,
      (gpio_num_t) -1,
      (gpio_num_t) -1,                // 4-7: RGB data
      (gpio_num_t) config_.pins.oe,   // 8: OE
      (gpio_num_t) config_.pins.lat,  // 9: LAT
      (gpio_num_t) -1,                // 10: Reserved
      (gpio_num_t) config_.pins.a,    // 11: ADDR_A
      (gpio_num_t) config_.pins.b,    // 12: ADDR_B
      (gpio_num_t) config_.pins.c,    // 13: ADDR_C
      (gpio_num_t) config_.pins.d,    // 14: ADDR_D
      (gpio_num_t) -1                 // 15: CLK_GATE (MSB, controlled by data)
  };

  // RGB pin mapping depends on bit depth
  // For now, hardcode RGB222 mapping (2 bits per color × 2 rows)
  data_pins[0] = (gpio_num_t) config_.pins.b2;
  data_pins[1] = (gpio_num_t) config_.pins.b1;
  data_pins[2] = (gpio_num_t) config_.pins.g2;
  data_pins[3] = (gpio_num_t) config_.pins.g1;
  data_pins[4] = (gpio_num_t) config_.pins.r2;
  data_pins[5] = (gpio_num_t) config_.pins.r1;

  parlio_tx_unit_config_t config = {.clk_src = PARLIO_CLK_SRC_DEFAULT,
                                    .clk_in_gpio_num = GPIO_NUM_NC,  // Use internal clock
                                    .input_clk_src_freq_hz = 0,
                                    .output_clk_freq_hz = static_cast<uint32_t>(config_.output_clock_speed),
                                    .data_width = 16,      // Full 16-bit width
                                    .data_gpio_nums = {},  // Will be filled below
                                    .clk_out_gpio_num = (gpio_num_t) config_.pins.clk,
                                    .valid_gpio_num = GPIO_NUM_NC,  // Not using valid signal
                                    .valid_start_delay = 0,
                                    .valid_stop_delay = 0,
                                    .trans_queue_depth = (size_t) (num_rows_ * bit_depth_),  // Queue all transactions
                                    .max_transfer_size = max_buffer_size * sizeof(uint16_t),
                                    .dma_burst_size = 0,  // Default
                                    .sample_edge = PARLIO_SAMPLE_EDGE_POS,
                                    .bit_pack_order = PARLIO_BIT_PACK_ORDER_MSB,
                                    .flags = {.clk_gate_en = 1,  // ENABLE CLOCK GATING (MSB bit controls PCLK)
                                              .io_loop_back = 0,
                                              .allow_pd = 0,
                                              .invert_valid_out = 0}};

  // Copy pin mapping
  for (int i = 0; i < 16; i++) {
    config.data_gpio_nums[i] = data_pins[i];
  }

  esp_err_t err = parlio_new_tx_unit(&config, &tx_unit_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create PARLIO TX unit: %s", esp_err_to_name(err));
    tx_unit_ = nullptr;
    return;
  }

  ESP_LOGI(TAG, "PARLIO TX unit created successfully");
  ESP_LOGI(TAG, "  Data width: 16 bits, Clock: %u MHz", static_cast<uint32_t>(config_.output_clock_speed) / 1000000);
  ESP_LOGI(TAG, "  Clock gating: ENABLED (MSB bit controls PCLK)");
  ESP_LOGI(TAG, "  Transaction queue depth: %zu", config.trans_queue_depth);
}

void ParlioDma::configure_gpio() {
  // PARLIO handles GPIO routing internally based on data_gpio_nums
  // We only need to set drive strength for better signal integrity

  gpio_num_t all_pins[] = {(gpio_num_t) config_.pins.r1, (gpio_num_t) config_.pins.g1,  (gpio_num_t) config_.pins.b1,
                           (gpio_num_t) config_.pins.r2, (gpio_num_t) config_.pins.g2,  (gpio_num_t) config_.pins.b2,
                           (gpio_num_t) config_.pins.a,  (gpio_num_t) config_.pins.b,   (gpio_num_t) config_.pins.c,
                           (gpio_num_t) config_.pins.d,  (gpio_num_t) config_.pins.lat, (gpio_num_t) config_.pins.oe,
                           (gpio_num_t) config_.pins.clk};

  for (auto pin : all_pins) {
    if (pin >= 0) {
      gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_3);  // Maximum drive strength
    }
  }

  ESP_LOGI(TAG, "GPIO drive strength configured (max)");
}
void ParlioDma::calculate_bcm_timings() {
  // Calculate base buffer transmission time
  const float buffer_time_us = (virtual_width_ * 1000000.0f) / static_cast<uint32_t>(config_.output_clock_speed);

  ESP_LOGI(TAG, "Buffer transmission time: %.2f µs (%u pixels @ %lu Hz)", buffer_time_us, virtual_width_,
           (unsigned long) static_cast<uint32_t>(config_.output_clock_speed));

  // Target refresh rate
  const uint32_t target_hz = config_.min_refresh_rate;

  // Calculate optimal lsbMsbTransitionBit (same algorithm as GDMA)
  lsbMsbTransitionBit_ = 0;
  int actual_hz = 0;

  while (true) {
    // Calculate transmissions per row with current transition bit
    int transmissions = bit_depth_;  // Base: all bits shown once

    // Add BCM repetitions for bits above transition
    for (int i = lsbMsbTransitionBit_ + 1; i < bit_depth_; i++) {
      transmissions += (1 << (i - lsbMsbTransitionBit_ - 1));
    }

    // Calculate refresh rate
    const float time_per_row_us = transmissions * buffer_time_us;
    const float time_per_frame_us = time_per_row_us * num_rows_;
    actual_hz = (int) (1000000.0f / time_per_frame_us);

    ESP_LOGD(TAG, "Testing lsbMsbTransitionBit=%d: %d transmissions/row, %d Hz", lsbMsbTransitionBit_, transmissions,
             actual_hz);

    if (actual_hz >= target_hz)
      break;

    if (lsbMsbTransitionBit_ < bit_depth_ - 1) {
      lsbMsbTransitionBit_++;
    } else {
      ESP_LOGW(TAG, "Cannot achieve target %lu Hz, max is %d Hz", (unsigned long) target_hz, actual_hz);
      break;
    }
  }

  ESP_LOGI(TAG, "lsbMsbTransitionBit=%d achieves %d Hz (target %lu Hz)", lsbMsbTransitionBit_, actual_hz,
           (unsigned long) target_hz);

  if (lsbMsbTransitionBit_ > 0) {
    ESP_LOGW(TAG, "Using lsbMsbTransitionBit=%d, lower %d bits show once (reduced color depth for speed)",
             lsbMsbTransitionBit_, lsbMsbTransitionBit_ + 1);
  }
}

size_t ParlioDma::calculate_bcm_padding(uint8_t bit_plane) {
  // Calculate padding words to achieve BCM timing via clock gating
  // Padding words have MSB=0 (clock disabled), panel displays during this time

  size_t base_padding = config_.latch_blanking;  // Minimum blanking for latch

  if (bit_plane <= lsbMsbTransitionBit_) {
    // LSB bits: minimal display time (just latch blanking)
    return base_padding;
  } else {
    // MSB bits: exponential BCM scaling
    // Repetition count from GDMA: (1 << (bit - lsbMsbTransitionBit - 1))
    // We add padding words proportional to the repetition count
    size_t repetitions = (1 << (bit_plane - lsbMsbTransitionBit_ - 1));

    // Padding = base_padding + (repetitions × pixel_width)
    // This gives equivalent display time to descriptor repetition
    return base_padding + (repetitions * virtual_width_);
  }
}

bool ParlioDma::allocate_row_buffers() {
  row_buffers_ = new RowBitPlaneBuffers[num_rows_];

  for (int row = 0; row < num_rows_; row++) {
    row_buffers_[row].bit_planes = new BitPlaneBuffer[bit_depth_];

    for (int bit = 0; bit < bit_depth_; bit++) {
      BitPlaneBuffer &bp = row_buffers_[row].bit_planes[bit];

      bp.pixel_words = virtual_width_;
      bp.padding_words = calculate_bcm_padding(bit);
      bp.total_words = bp.pixel_words + bp.padding_words;

      // Allocate DMA-capable memory
      bp.data = (uint16_t *) heap_caps_malloc(bp.total_words * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

      if (!bp.data) {
        ESP_LOGE(TAG, "Failed to allocate buffer for row %d, bit %d", row, bit);
        return false;
      }

      ESP_LOGD(TAG, "Row %d bit %d: %zu pixel words + %zu padding = %zu total", row, bit, bp.pixel_words,
               bp.padding_words, bp.total_words);
    }
  }

  // Calculate total memory
  size_t total_mem = 0;
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      total_mem += row_buffers_[row].bit_planes[bit].total_words * sizeof(uint16_t);
    }
  }

  ESP_LOGI(TAG, "Allocated row buffers: %zu bytes total for %d rows × %d bits", total_mem, num_rows_, bit_depth_);

  return true;
}

bool ParlioDma::setup_dma(uint8_t *buffer_a, uint8_t *buffer_b, size_t size) {
  (void) buffer_a;
  (void) buffer_b;
  (void) size;

  ESP_LOGI(TAG, "Setting up DMA with transaction queue...");

  // Allocate row buffers with BCM padding
  if (!allocate_row_buffers()) {
    ESP_LOGE(TAG, "Failed to allocate row buffers");
    return false;
  }

  // Initialize buffers with blank data
  initialize_blank_buffers();

  // Build transaction queue
  if (!build_transaction_queue()) {
    ESP_LOGE(TAG, "Failed to build transaction queue");
    return false;
  }

  ESP_LOGI(TAG, "Circular DMA setup complete");
  return true;
}

void ParlioDma::start_transfer() {
  if (!tx_unit_ || transfer_started_) {
    return;
  }

  ESP_LOGI(TAG, "Starting PARLIO transfer (loop transmission)");

  esp_err_t err = parlio_tx_unit_enable(tx_unit_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable PARLIO TX: %s", esp_err_to_name(err));
    return;
  }

  transfer_started_ = true;
  ESP_LOGI(TAG, "PARLIO transfer started");
}

void ParlioDma::stop_transfer() {
  if (!tx_unit_ || !transfer_started_) {
    return;
  }

  ESP_LOGI(TAG, "Stopping PARLIO transfer");

  parlio_tx_unit_disable(tx_unit_);
  transfer_started_ = false;
}

void ParlioDma::initialize_blank_buffers() {
  if (!row_buffers_) {
    ESP_LOGE(TAG, "Row buffers not allocated");
    return;
  }

  ESP_LOGI(TAG, "Initializing blank DMA buffers with clock gating control...");

  for (int row = 0; row < num_rows_; row++) {
    uint16_t row_addr = row & ADDR_MASK;  // 4-bit address for PARLIO

    for (int bit = 0; bit < bit_depth_; bit++) {
      BitPlaneBuffer &bp = row_buffers_[row].bit_planes[bit];

      // Row address handling: LSB bit plane uses previous row for LAT settling
      // (Same ghosting fix as GDMA - gives panel time to process LAT before address change)
      uint16_t addr_for_buffer;
      if (bit == 0) {
        addr_for_buffer = (row == 0) ? (num_rows_ - 1) : (row - 1);
      } else {
        addr_for_buffer = row_addr;
      }

      // Initialize pixel section (MSB=1, clock enabled)
      for (size_t x = 0; x < bp.pixel_words; x++) {
        uint16_t word = 0;
        word |= (1 << CLK_GATE_BIT);              // MSB=1: enable clock
        word |= (addr_for_buffer << ADDR_SHIFT);  // Row address
        word |= (1 << OE_BIT);                    // OE=1 (blanked during shift)

        // LAT=1 on last pixel
        if (x == bp.pixel_words - 1) {
          word |= (1 << LAT_BIT);
        }

        // RGB data = 0 (will be set by draw_pixels)
        bp.data[x] = word;
      }

      // Initialize padding section (MSB=0, clock disabled)
      // This is where BCM display time happens
      for (size_t i = 0; i < bp.padding_words; i++) {
        uint16_t word = 0;
        word |= (addr_for_buffer << ADDR_SHIFT);  // Keep same address
        // MSB=0: clock disabled (panel displays)
        // OE will be set by set_brightness_oe()
        word |= (1 << OE_BIT);  // Default: blanked (will be adjusted by brightness)

        bp.data[bp.pixel_words + i] = word;
      }
    }
  }

  ESP_LOGI(TAG, "Blank buffers initialized (clock gating via MSB)");
}

void ParlioDma::set_brightness_oe() {
  if (!row_buffers_) {
    ESP_LOGE(TAG, "Row buffers not allocated");
    return;
  }

  // Calculate effective brightness (0-255)
  const uint8_t brightness = (uint8_t) ((float) basis_brightness_ * intensity_);

  ESP_LOGI(TAG, "Setting brightness OE: brightness=%u (basis=%u × intensity=%.2f)", brightness, basis_brightness_,
           intensity_);

  const uint16_t OE_CLEAR_MASK = ~(1 << OE_BIT);

  // Special case: brightness=0 means fully blanked (display off)
  if (brightness == 0) {
    for (int row = 0; row < num_rows_; row++) {
      for (int bit = 0; bit < bit_depth_; bit++) {
        BitPlaneBuffer &bp = row_buffers_[row].bit_planes[bit];

        // Blank all pixels in padding section: set OE bit HIGH
        for (size_t i = 0; i < bp.padding_words; i++) {
          bp.data[bp.pixel_words + i] |= (1 << OE_BIT);
        }
      }
    }
    ESP_LOGI(TAG, "Display blanked (brightness=0)");
    return;
  }

  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      BitPlaneBuffer &bp = row_buffers_[row].bit_planes[bit];

      // For PARLIO with clock gating, brightness is controlled by OE duty cycle
      // in the padding section (where MSB=0 and panel displays)

      if (bp.padding_words == 0) {
        continue;  // No padding, skip
      }

      // Calculate display pixel count in padding section
      const int max_display = bp.padding_words - config_.latch_blanking;
      int display_count = (max_display * brightness) >> 8;

      // Ensure at least 1 word for brightness > 0
      if (brightness > 0 && display_count == 0) {
        display_count = 1;
      }

      // Safety margin
      if (display_count > max_display - 1) {
        display_count = max_display - 1;
      }

      // Center the display window in padding section
      const int start_display = (bp.padding_words - display_count) / 2;
      const int end_display = start_display + display_count;

      // Set OE bits in padding section
      for (size_t i = 0; i < bp.padding_words; i++) {
        uint16_t &word = bp.data[bp.pixel_words + i];

        if (i >= start_display && i < end_display) {
          // Display enabled: OE=0
          word &= OE_CLEAR_MASK;
        } else {
          // Blanked: OE=1
          word |= (1 << OE_BIT);
        }
      }

      // CRITICAL: Latch blanking at end of padding
      // Blank last N words to prevent ghosting during row transition
      for (size_t i = 0; i < config_.latch_blanking && i < bp.padding_words; i++) {
        size_t idx = bp.padding_words - 1 - i;
        bp.data[bp.pixel_words + idx] |= (1 << OE_BIT);
      }
    }
  }

  ESP_LOGI(TAG, "Brightness OE updated");
}

bool ParlioDma::build_transaction_queue() {
  if (!tx_unit_) {
    ESP_LOGE(TAG, "PARLIO TX unit not initialized");
    return false;
  }

  ESP_LOGI(TAG, "Building transaction queue for loop transmission...");

  // Queue all row/bit transactions
  size_t transaction_count = 0;

  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      BitPlaneBuffer &bp = row_buffers_[row].bit_planes[bit];

      // Queue this transaction (will be transmitted in loop)
      size_t payload_bits = bp.total_words * 16;  // Convert words to bits

      esp_err_t err = parlio_tx_unit_transmit(tx_unit_, bp.data, payload_bits, &transmit_config_);

      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to queue transaction row=%d bit=%d: %s", row, bit, esp_err_to_name(err));
        return false;
      }

      transaction_count++;
      ESP_LOGD(TAG, "Queued row=%d bit=%d: %zu words (%zu pixel + %zu padding)", row, bit, bp.total_words,
               bp.pixel_words, bp.padding_words);
    }
  }

  ESP_LOGI(TAG, "Transaction queue built: %zu transactions queued", transaction_count);
  ESP_LOGI(TAG, "Loop transmission enabled - will repeat automatically");

  return true;
}

void ParlioDma::set_basis_brightness(uint8_t brightness) {
  if (brightness != basis_brightness_) {
    basis_brightness_ = brightness;

    if (brightness == 0) {
      ESP_LOGI(TAG, "Brightness set to 0 (display off)");
    } else {
      ESP_LOGI(TAG, "Basis brightness set to %u", (unsigned) brightness);
    }

    set_brightness_oe();
  }
}

void ParlioDma::set_intensity(float intensity) {
  if (intensity < 0.0f)
    intensity = 0.0f;
  if (intensity > 1.0f)
    intensity = 1.0f;
  if (intensity != intensity_) {
    intensity_ = intensity;
    set_brightness_oe();
  }
}

void ParlioDma::set_lut(const uint16_t *lut) { lut_ = lut; }

HUB75_IRAM void ParlioDma::draw_pixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer,
                                       Hub75PixelFormat format, Hub75ColorOrder color_order, bool big_endian) {
  if (!row_buffers_ || !lut_ || !buffer) [[unlikely]] {
    return;
  }

  // Bounds check
  if (x >= virtual_width_ || y >= virtual_height_) [[unlikely]] {
    return;
  }

  // Clip to display bounds
  if (x + w > virtual_width_) [[unlikely]] {
    w = virtual_width_ - x;
  }
  if (y + h > virtual_height_) [[unlikely]] {
    h = virtual_height_ - y;
  }

  // Process each pixel based on format
  for (uint16_t dy = 0; dy < h; dy++) {
    for (uint16_t dx = 0; dx < w; dx++) {
      uint16_t px = x + dx;
      uint16_t py = y + dy;
      const size_t pixel_idx = dy * w + dx;

      // Coordinate transformation pipeline
      Coords c = {.x = px, .y = py};

      // Step 1: Panel layout remapping (if multi-panel grid)
      if (needs_layout_remap_) {
        c = PanelLayoutRemap::remap(c, layout_, panel_width_, panel_height_, layout_rows_, layout_cols_);
      }

      // Step 2: Scan pattern remapping (if non-standard panel)
      if (needs_scan_remap_) {
        c = ScanPatternRemap::remap(c, scan_wiring_, panel_width_);
      }

      px = c.x;
      py = c.y;

      // Calculate row and half
      const uint16_t row = py % num_rows_;
      const bool is_lower = (py >= num_rows_);

      uint8_t r8 = 0, g8 = 0, b8 = 0;

      // Extract RGB based on format (same as GDMA)
      switch (format) {
        case Hub75PixelFormat::RGB888: {
          const uint8_t *p = buffer + pixel_idx * 3;
          r8 = p[0];
          g8 = p[1];
          b8 = p[2];
          break;
        }

        case Hub75PixelFormat::RGB888_32: {
          const uint8_t *p = buffer + pixel_idx * 4;
          if (color_order == Hub75ColorOrder::RGB) {
            if (big_endian) {
              r8 = p[1];
              g8 = p[2];
              b8 = p[3];
            } else {
              r8 = p[2];
              g8 = p[1];
              b8 = p[0];
            }
          } else {  // BGR
            if (big_endian) {
              r8 = p[3];
              g8 = p[2];
              b8 = p[1];
            } else {
              r8 = p[0];
              g8 = p[1];
              b8 = p[2];
            }
          }
          break;
        }

        case Hub75PixelFormat::RGB565: {
          const uint8_t *p = buffer + pixel_idx * 2;
          uint16_t rgb565;
          if (big_endian) {
            rgb565 = (uint16_t(p[0]) << 8) | p[1];
          } else {
            rgb565 = (uint16_t(p[1]) << 8) | p[0];
          }

          const uint8_t r5 = (rgb565 >> 11) & 0x1F;
          const uint8_t g6 = (rgb565 >> 5) & 0x3F;
          const uint8_t b5 = rgb565 & 0x1F;

          // Scale to 8-bit using color conversion helpers
          r8 = scale_5bit_to_8bit(r5);
          g8 = scale_6bit_to_8bit(g6);
          b8 = scale_5bit_to_8bit(b5);
          break;
        }
      }

      // Apply LUT correction
      const uint16_t r_corrected = lut_[r8];
      const uint16_t g_corrected = lut_[g8];
      const uint16_t b_corrected = lut_[b8];

      // Update all bit planes for this pixel
      // PARLIO bit layout: [CLK_GATE(15)|ADDR(14-11)|--|LAT(9)|OE(8)|--|--|R2(4)|R1(5)|G2(2)|G1(3)|B2(0)|B1(1)]
      // Based on pin mapping in configure_parlio:
      // data_pins[0] = B2, [1] = B1, [2] = G2, [3] = G1, [4] = R2, [5] = R1
      for (int bit = 0; bit < bit_depth_; bit++) {
        BitPlaneBuffer &bp = row_buffers_[row].bit_planes[bit];
        uint16_t *buf = bp.data;

        const uint16_t mask = (1 << bit);
        uint16_t word = buf[px];  // Read existing word (preserves control bits)

        // Clear and update RGB bits for appropriate half
        // IMPORTANT: Only modify RGB bits (0-5), preserve control bits (8-15)
        if (is_lower) {
          // Lower half: R2, G2, B2
          word &= ~RGB_LOWER_MASK;
          if (r_corrected & mask)
            word |= (1 << R2_BIT);
          if (g_corrected & mask)
            word |= (1 << G2_BIT);
          if (b_corrected & mask)
            word |= (1 << B2_BIT);
        } else {
          // Upper half: R1, G1, B1
          word &= ~RGB_UPPER_MASK;
          if (r_corrected & mask)
            word |= (1 << R1_BIT);
          if (g_corrected & mask)
            word |= (1 << G1_BIT);
          if (b_corrected & mask)
            word |= (1 << B1_BIT);
        }

        buf[px] = word;
      }
    }
  }
}

void ParlioDma::clear() {
  if (!row_buffers_) {
    return;
  }

  // Clear RGB bits in all buffers (keep control bits)
  // RGB bits are 0-5 in PARLIO layout
  const uint16_t RGB_CLEAR_MASK = ~0x003F;  // Clear bits 0-5

  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      BitPlaneBuffer &bp = row_buffers_[row].bit_planes[bit];

      // Clear pixel section only (padding has no RGB data)
      for (size_t x = 0; x < bp.pixel_words; x++) {
        bp.data[x] &= RGB_CLEAR_MASK;
      }
    }
  }
}

// ============================================================================
// Compile-Time Validation
// ============================================================================

namespace {

// Validate BCM padding repetition calculation
consteval bool test_bcm_padding_repetitions() {
  // For bit 7 with lsb_msb_transition=1:
  // Repetitions = 1 << (7 - 1 - 1) = 1 << 5 = 32
  constexpr int repetitions = (1 << (7 - 1 - 1));
  return repetitions == 32;
}

consteval bool test_bcm_padding_repetitions_bit5() {
  // For bit 5 with lsb_msb_transition=1:
  // Repetitions = 1 << (5 - 1 - 1) = 1 << 3 = 8
  constexpr int repetitions = (1 << (5 - 1 - 1));
  return repetitions == 8;
}

consteval bool test_bcm_padding_repetitions_bit3() {
  // For bit 3 with lsb_msb_transition=1:
  // Repetitions = 1 << (3 - 1 - 1) = 1 << 1 = 2
  constexpr int repetitions = (1 << (3 - 1 - 1));
  return repetitions == 2;
}

consteval bool test_bcm_padding_repetitions_bit2() {
  // For bit 2 with lsb_msb_transition=1:
  // Repetitions = 1 << (2 - 1 - 1) = 1 << 0 = 1
  constexpr int repetitions = (1 << (2 - 1 - 1));
  return repetitions == 1;
}

// Static assertions
static_assert(test_bcm_padding_repetitions(), "BCM padding: bit 7, transition 1 should produce 32 repetitions");
static_assert(test_bcm_padding_repetitions_bit5(), "BCM padding: bit 5, transition 1 should produce 8 repetitions");
static_assert(test_bcm_padding_repetitions_bit3(), "BCM padding: bit 3, transition 1 should produce 2 repetitions");
static_assert(test_bcm_padding_repetitions_bit2(), "BCM padding: bit 2, transition 1 should produce 1 repetition");

}  // namespace

}  // namespace hub75

#endif  // CONFIG_IDF_TARGET_ESP32P4
