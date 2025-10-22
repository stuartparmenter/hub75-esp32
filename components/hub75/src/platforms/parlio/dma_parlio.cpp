/**
 * SPDX-FileCopyrightText: 2025 Stuart Parmenter
 * SPDX-License-Identifier: MIT
 *
 * @file dma_parlio.cpp
 * @brief ESP32-P4 PARLIO implementation for HUB75 with clock gating
 *
 * Uses PARLIO TX with clock gating (MSB bit controls PCLK) to embed
 * BCM timing directly in buffer data, eliminating descriptor repetition.
 */

#include <sdkconfig.h>

// Only compile for ESP32-P4
#ifdef CONFIG_IDF_TARGET_ESP32P4

#include "dma_parlio.hpp"
#include "../../color/color_convert.hpp"
#include <cstring>
#include <algorithm>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>

static const char *const TAG = "ESP32P4_PARLIO";

// Bit positions in 16-bit word (MSB-first)
#define BIT_CLK_GATE 15  // MSB: Clock gate control
#define BIT_ADDR_A 11    // Address bits
#define BIT_ADDR_B 12
#define BIT_ADDR_C 13
#define BIT_ADDR_D 14
#define BIT_LAT 9  // Latch
#define BIT_OE 8   // Output Enable
// Bits 0-7: RGB data (platform-specific packing)

namespace hub75 {

ESP32P4_PARLIO_DMA::ESP32P4_PARLIO_DMA(const hub75_config_t &config)
    : config_(config),
      tx_unit_(nullptr),
      bit_depth_(config.bit_depth),
      lsbMsbTransitionBit_(0),
      num_rows_(config.height / 2),
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

ESP32P4_PARLIO_DMA::~ESP32P4_PARLIO_DMA() { shutdown(); }

bool ESP32P4_PARLIO_DMA::init() {
  ESP_LOGI(TAG, "Initializing PARLIO TX peripheral with clock gating...");
  ESP_LOGI(TAG, "Display: %dx%d, %d rows, %d-bit color", config_.width, config_.height, num_rows_, bit_depth_);

  // Calculate BCM timings first
  calculateBCMTimings();

  // Configure GPIO
  configureGPIO();

  // Configure PARLIO peripheral
  configurePARLIO();

  if (!tx_unit_) {
    ESP_LOGE(TAG, "Failed to create PARLIO TX unit");
    return false;
  }

  ESP_LOGI(TAG, "PARLIO TX initialized successfully");
  ESP_LOGI(TAG, "Clock gating enabled, MSB bit controls PCLK");

  return true;
}

void ESP32P4_PARLIO_DMA::shutdown() {
  if (transfer_started_) {
    stopTransfer();
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

void ESP32P4_PARLIO_DMA::configurePARLIO() {
  ESP_LOGI(TAG, "Configuring PARLIO TX unit with clock gating...");

  // Calculate maximum buffer size for transaction queue
  size_t max_buffer_size = config_.width * config_.chain_length;
  for (int bit = 0; bit < bit_depth_; bit++) {
    max_buffer_size = std::max(max_buffer_size, config_.width * config_.chain_length + calculateBCMPadding(bit));
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
                                    .output_clk_freq_hz = config_.output_clock_speed,
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
  ESP_LOGI(TAG, "  Data width: 16 bits, Clock: %u MHz", config_.output_clock_speed / 1000000);
  ESP_LOGI(TAG, "  Clock gating: ENABLED (MSB bit controls PCLK)");
  ESP_LOGI(TAG, "  Transaction queue depth: %zu", config.trans_queue_depth);
}

void ESP32P4_PARLIO_DMA::configureGPIO() {
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
void ESP32P4_PARLIO_DMA::calculateBCMTimings() {
  // Calculate base buffer transmission time
  uint16_t total_width = config_.width * config_.chain_length;
  float buffer_time_us = (total_width * 1000000.0f) / config_.output_clock_speed;

  ESP_LOGI(TAG, "Buffer transmission time: %.2f µs (%u pixels @ %lu Hz)", buffer_time_us, (unsigned) total_width,
           (unsigned long) config_.output_clock_speed);

  // Target refresh rate
  uint32_t target_hz = config_.min_refresh_rate;
  uint32_t num_rows = config_.height / 2;

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
    float time_per_row_us = transmissions * buffer_time_us;
    float time_per_frame_us = time_per_row_us * num_rows;
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

size_t ESP32P4_PARLIO_DMA::calculateBCMPadding(uint8_t bit_plane) {
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
    size_t total_width = config_.width * config_.chain_length;
    return base_padding + (repetitions * total_width);
  }
}

bool ESP32P4_PARLIO_DMA::allocateRowBuffers() {
  size_t total_width = config_.width * config_.chain_length;

  row_buffers_ = new RowBitPlaneBuffers[num_rows_];

  for (int row = 0; row < num_rows_; row++) {
    row_buffers_[row].bit_planes = new BitPlaneBuffer[bit_depth_];

    for (int bit = 0; bit < bit_depth_; bit++) {
      BitPlaneBuffer &bp = row_buffers_[row].bit_planes[bit];

      bp.pixel_words = total_width;
      bp.padding_words = calculateBCMPadding(bit);
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

bool ESP32P4_PARLIO_DMA::setupCircularDMA(uint8_t *buffer_a, uint8_t *buffer_b, size_t size) {
  (void) buffer_a;
  (void) buffer_b;
  (void) size;

  ESP_LOGI(TAG, "Setting up circular DMA with transaction queue...");

  // Allocate row buffers with BCM padding
  if (!allocateRowBuffers()) {
    ESP_LOGE(TAG, "Failed to allocate row buffers");
    return false;
  }

  // Initialize buffers with blank data
  initializeBlankBuffers();

  // Build transaction queue
  if (!buildTransactionQueue()) {
    ESP_LOGE(TAG, "Failed to build transaction queue");
    return false;
  }

  ESP_LOGI(TAG, "Circular DMA setup complete");
  return true;
}

void ESP32P4_PARLIO_DMA::startTransfer() {
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

void ESP32P4_PARLIO_DMA::stopTransfer() {
  if (!tx_unit_ || !transfer_started_) {
    return;
  }

  ESP_LOGI(TAG, "Stopping PARLIO transfer");

  parlio_tx_unit_disable(tx_unit_);
  transfer_started_ = false;
}

void ESP32P4_PARLIO_DMA::initializeBlankBuffers() {
  if (!row_buffers_) {
    ESP_LOGE(TAG, "Row buffers not allocated");
    return;
  }

  ESP_LOGI(TAG, "Initializing blank DMA buffers with clock gating control...");

  // Bit positions in 16-bit word
  const uint16_t CLK_GATE_BIT = (1 << 15);  // MSB: clock gate control
  const uint16_t ADDR_SHIFT = 11;           // Address bits start at bit 11
  const uint16_t LAT_BIT = (1 << 9);
  const uint16_t OE_BIT = (1 << 8);

  for (int row = 0; row < num_rows_; row++) {
    uint16_t row_addr = row & 0x1F;  // 5-bit address

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
        word |= CLK_GATE_BIT;                     // MSB=1: enable clock
        word |= (addr_for_buffer << ADDR_SHIFT);  // Row address
        word |= OE_BIT;                           // OE=1 (blanked during shift)

        // LAT=1 on last pixel
        if (x == bp.pixel_words - 1) {
          word |= LAT_BIT;
        }

        // RGB data = 0 (will be set by drawPixels)
        bp.data[x] = word;
      }

      // Initialize padding section (MSB=0, clock disabled)
      // This is where BCM display time happens
      for (size_t i = 0; i < bp.padding_words; i++) {
        uint16_t word = 0;
        word |= (addr_for_buffer << ADDR_SHIFT);  // Keep same address
        // MSB=0: clock disabled (panel displays)
        // OE will be set by setBrightnessOE()
        word |= OE_BIT;  // Default: blanked (will be adjusted by brightness)

        bp.data[bp.pixel_words + i] = word;
      }
    }
  }

  ESP_LOGI(TAG, "Blank buffers initialized (clock gating via MSB)");
}

void ESP32P4_PARLIO_DMA::setBrightnessOE() {
  if (!row_buffers_) {
    ESP_LOGE(TAG, "Row buffers not allocated");
    return;
  }

  // Calculate effective brightness (0-255)
  uint8_t brightness = (uint8_t) ((float) basis_brightness_ * intensity_);

  ESP_LOGI(TAG, "Setting brightness OE: brightness=%u (basis=%u × intensity=%.2f)", brightness, basis_brightness_,
           intensity_);

  const uint16_t OE_BIT = (1 << 8);
  const uint16_t OE_CLEAR_MASK = ~OE_BIT;

  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      BitPlaneBuffer &bp = row_buffers_[row].bit_planes[bit];

      // For PARLIO with clock gating, brightness is controlled by OE duty cycle
      // in the padding section (where MSB=0 and panel displays)

      if (bp.padding_words == 0) {
        continue;  // No padding, skip
      }

      // Calculate display pixel count in padding section
      int max_display = bp.padding_words - config_.latch_blanking;
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
      int start_display = (bp.padding_words - display_count) / 2;
      int end_display = start_display + display_count;

      // Set OE bits in padding section
      for (size_t i = 0; i < bp.padding_words; i++) {
        uint16_t &word = bp.data[bp.pixel_words + i];

        if (i >= start_display && i < end_display) {
          // Display enabled: OE=0
          word &= OE_CLEAR_MASK;
        } else {
          // Blanked: OE=1
          word |= OE_BIT;
        }
      }

      // CRITICAL: Latch blanking at end of padding
      // Blank last N words to prevent ghosting during row transition
      for (size_t i = 0; i < config_.latch_blanking && i < bp.padding_words; i++) {
        size_t idx = bp.padding_words - 1 - i;
        bp.data[bp.pixel_words + idx] |= OE_BIT;
      }
    }
  }

  ESP_LOGI(TAG, "Brightness OE updated");
}

bool ESP32P4_PARLIO_DMA::buildTransactionQueue() {
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

void ESP32P4_PARLIO_DMA::setBasisBrightness(uint8_t brightness) {
  if (brightness == 0)
    brightness = 1;
  if (brightness != basis_brightness_) {
    basis_brightness_ = brightness;
    setBrightnessOE();
  }
}

void ESP32P4_PARLIO_DMA::setIntensity(float intensity) {
  if (intensity < 0.0f)
    intensity = 0.0f;
  if (intensity > 1.0f)
    intensity = 1.0f;
  if (intensity != intensity_) {
    intensity_ = intensity;
    setBrightnessOE();
  }
}

void ESP32P4_PARLIO_DMA::setLUT(const uint16_t *lut) {
  lut_ = lut;
}

HUB75_IRAM void ESP32P4_PARLIO_DMA::drawPixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer,
                                               hub75_pixel_format_t format, hub75_color_order_t color_order,
                                               bool big_endian) {
  if (!row_buffers_ || !lut_ || !buffer) {
    return;
  }

  uint16_t total_width = config_.width * config_.chain_length;

  // Bounds check
  if (x >= total_width || y >= config_.height) {
    return;
  }

  // Clip to display bounds
  if (x + w > total_width) {
    w = total_width - x;
  }
  if (y + h > config_.height) {
    h = config_.height - y;
  }

  // Process each pixel based on format
  for (uint16_t dy = 0; dy < h; dy++) {
    uint16_t py = y + dy;
    uint16_t row = py % (config_.height / 2);
    bool is_lower = (py >= config_.height / 2);

    for (uint16_t dx = 0; dx < w; dx++) {
      uint16_t px = x + dx;
      size_t pixel_idx = dy * w + dx;

      uint8_t r8, g8, b8;

      // Extract RGB based on format (same as GDMA)
      switch (format) {
        case HUB75_PIXEL_FORMAT_RGB888: {
          const uint8_t *p = buffer + pixel_idx * 3;
          r8 = p[0];
          g8 = p[1];
          b8 = p[2];
          break;
        }

        case HUB75_PIXEL_FORMAT_RGB888_32: {
          const uint8_t *p = buffer + pixel_idx * 4;
          if (color_order == HUB75_COLOR_ORDER_RGB) {
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

        case HUB75_PIXEL_FORMAT_RGB565: {
          const uint8_t *p = buffer + pixel_idx * 2;
          uint16_t rgb565;
          if (big_endian) {
            rgb565 = (uint16_t(p[0]) << 8) | p[1];
          } else {
            rgb565 = (uint16_t(p[1]) << 8) | p[0];
          }

          uint8_t r5 = (rgb565 >> 11) & 0x1F;
          uint8_t g6 = (rgb565 >> 5) & 0x3F;
          uint8_t b5 = rgb565 & 0x1F;

          r8 = (r5 * 527 + 23) >> 6;
          g8 = (g6 * 259 + 33) >> 6;
          b8 = (b5 * 527 + 23) >> 6;
          break;
        }

        default:
          continue;
      }

      // Apply LUT correction
      uint16_t r_corrected = lut_[r8];
      uint16_t g_corrected = lut_[g8];
      uint16_t b_corrected = lut_[b8];

      // Update all bit planes for this pixel
      // PARLIO bit layout: [CLK_GATE(15)|ADDR(14-11)|--|LAT(9)|OE(8)|--|--|R2(4)|R1(5)|G2(2)|G1(3)|B2(0)|B1(1)]
      // Based on pin mapping in configurePARLIO:
      // data_pins[0] = B2, [1] = B1, [2] = G2, [3] = G1, [4] = R2, [5] = R1
      for (int bit = 0; bit < bit_depth_; bit++) {
        BitPlaneBuffer &bp = row_buffers_[row].bit_planes[bit];
        uint16_t *buf = bp.data;

        uint16_t mask = (1 << bit);
        uint16_t word = buf[px];  // Read existing word (preserves control bits)

        // Clear and update RGB bits for appropriate half
        // IMPORTANT: Only modify RGB bits (0-5), preserve control bits (8-15)
        if (is_lower) {
          // Lower half: R2, G2, B2 (bits 4, 2, 0)
          word &= ~((1 << 4) | (1 << 2) | (1 << 0));
          if (r_corrected & mask)
            word |= (1 << 4);
          if (g_corrected & mask)
            word |= (1 << 2);
          if (b_corrected & mask)
            word |= (1 << 0);
        } else {
          // Upper half: R1, G1, B1 (bits 5, 3, 1)
          word &= ~((1 << 5) | (1 << 3) | (1 << 1));
          if (r_corrected & mask)
            word |= (1 << 5);
          if (g_corrected & mask)
            word |= (1 << 3);
          if (b_corrected & mask)
            word |= (1 << 1);
        }

        buf[px] = word;
      }
    }
  }
}

void ESP32P4_PARLIO_DMA::clear() {
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

}  // namespace hub75

#endif  // CONFIG_IDF_TARGET_ESP32P4
