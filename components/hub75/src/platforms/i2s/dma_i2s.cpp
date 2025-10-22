/**
 * SPDX-FileCopyrightText: 2025 Stuart Parmenter
 * SPDX-License-Identifier: MIT
 *
 * @file dma_i2s.cpp
 * @brief ESP32/ESP32-S2 I2S DMA implementation for HUB75
 *
 * Self-contained I2S+DMA with BCM, following GDMA architecture
 */

#include <sdkconfig.h>

// Only compile for ESP32 and ESP32-S2
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)

#include "dma_i2s.hpp"
#include "../../color/color_convert.hpp"
#include <cstring>
#include <algorithm>
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <esp_private/periph_ctrl.h>
#include <soc/gpio_sig_map.h>
#include <soc/i2s_periph.h>
#include <esp_heap_caps.h>

static const char *const TAG = "ESP32_I2S_DMA";

// Use I2S1 for original ESP32, I2S0 for ESP32-S2
#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define ESP32_I2S_DEVICE 0
#else
#define ESP32_I2S_DEVICE 1
#endif

namespace hub75 {

// ============================================================================
// Constructor / Destructor
// ============================================================================

ESP32_I2S_DMA::ESP32_I2S_DMA(const hub75_config_t &config)
    : config_(config),
      i2s_dev_(nullptr),
      bit_depth_(config.bit_depth),
      lsbMsbTransitionBit_(0),
      row_buffers_(nullptr),
      num_rows_(config.height / 2),
      descriptors_(nullptr),
      descriptor_count_(0),
      basis_brightness_(config.brightness),
      intensity_(1.0f),
      lut_(nullptr) {
  // Zero-copy architecture: DMA buffers ARE the display memory
}

ESP32_I2S_DMA::~ESP32_I2S_DMA() { shutdown(); }

// ============================================================================
// Initialization
// ============================================================================

bool ESP32_I2S_DMA::init() {
  ESP_LOGI(TAG, "Initializing I2S peripheral in LCD mode...");
  ESP_LOGI(TAG, "Pin config: R1=%d G1=%d B1=%d R2=%d G2=%d B2=%d", config_.pins.r1, config_.pins.g1, config_.pins.b1,
           config_.pins.r2, config_.pins.g2, config_.pins.b2);
  ESP_LOGI(TAG, "Pin config: A=%d B=%d C=%d D=%d E=%d LAT=%d OE=%d CLK=%d", config_.pins.a, config_.pins.b,
           config_.pins.c, config_.pins.d, config_.pins.e, config_.pins.lat, config_.pins.oe, config_.pins.clk);

  // Get I2S device
#if defined(CONFIG_IDF_TARGET_ESP32S2)
  i2s_dev_ = &I2S0;
#else
  i2s_dev_ = (ESP32_I2S_DEVICE == 0) ? &I2S0 : &I2S1;
#endif

  // Reset and enable I2S peripheral
  if (ESP32_I2S_DEVICE == 0) {
    periph_module_reset(PERIPH_I2S0_MODULE);
    periph_module_enable(PERIPH_I2S0_MODULE);
  } else {
#if !defined(CONFIG_IDF_TARGET_ESP32S2)
    periph_module_reset(PERIPH_I2S1_MODULE);
    periph_module_enable(PERIPH_I2S1_MODULE);
#endif
  }

  // Configure GPIO pins
  configureGPIO();

  // Configure I2S timing
  configureI2STiming();

  // Configure I2S for LCD mode
  i2s_dev_->conf2.val = 0;
  i2s_dev_->conf2.lcd_en = 1;          // Enable LCD mode
  i2s_dev_->conf2.lcd_tx_wrx2_en = 0;  // No double write
  i2s_dev_->conf2.lcd_tx_sdx2_en = 0;  // No SD double

  // I2S configuration
  i2s_dev_->conf.val = 0;

#if defined(CONFIG_IDF_TARGET_ESP32S2)
  i2s_dev_->conf.tx_dma_equal = 1;  // ESP32-S2 only
  i2s_dev_->conf.pre_req_en = 1;    // ESP32-S2 only - enable I2S to prepare data earlier
#endif

  // Configure FIFO
  i2s_dev_->fifo_conf.val = 0;
  i2s_dev_->fifo_conf.rx_data_num = 32;  // FIFO thresholds
  i2s_dev_->fifo_conf.tx_data_num = 32;
  i2s_dev_->fifo_conf.dscr_en = 1;  // Enable DMA descriptor mode

#if !defined(CONFIG_IDF_TARGET_ESP32S2)
  // FIFO mode for 16-bit parallel
  i2s_dev_->fifo_conf.tx_fifo_mod = 1;  // 16-bit mode
  i2s_dev_->fifo_conf.rx_fifo_mod_force_en = 1;
  i2s_dev_->fifo_conf.tx_fifo_mod_force_en = 1;

  // Channel configuration (16-bit single channel)
  i2s_dev_->conf_chan.val = 0;
  i2s_dev_->conf_chan.tx_chan_mod = 1;
  i2s_dev_->conf_chan.rx_chan_mod = 1;
#endif

  // Reset FIFOs
  i2s_dev_->conf.rx_fifo_reset = 1;
#if defined(CONFIG_IDF_TARGET_ESP32S2)
  while (i2s_dev_->conf.rx_fifo_reset_st)
    ;  // ESP32-S2 only
#endif
  i2s_dev_->conf.rx_fifo_reset = 0;

  i2s_dev_->conf.tx_fifo_reset = 1;
#if defined(CONFIG_IDF_TARGET_ESP32S2)
  while (i2s_dev_->conf.tx_fifo_reset_st)
    ;  // ESP32-S2 only
#endif
  i2s_dev_->conf.tx_fifo_reset = 0;

  // Reset DMA
  i2s_dev_->lc_conf.in_rst = 1;
  i2s_dev_->lc_conf.in_rst = 0;
  i2s_dev_->lc_conf.out_rst = 1;
  i2s_dev_->lc_conf.out_rst = 0;
  i2s_dev_->lc_conf.ahbm_rst = 1;
  i2s_dev_->lc_conf.ahbm_rst = 0;

  i2s_dev_->in_link.val = 0;
  i2s_dev_->out_link.val = 0;

  // Device reset
  i2s_dev_->conf.rx_reset = 1;
  i2s_dev_->conf.tx_reset = 1;
  i2s_dev_->conf.rx_reset = 0;
  i2s_dev_->conf.tx_reset = 0;

  i2s_dev_->conf1.val = 0;
  i2s_dev_->conf1.tx_stop_en = 0;
  i2s_dev_->conf1.tx_pcm_bypass = 1;

  i2s_dev_->timing.val = 0;

  ESP_LOGI(TAG, "I2S LCD mode initialized successfully");
  return true;
}

void ESP32_I2S_DMA::configureI2STiming() {
  auto dev = i2s_dev_;
  uint32_t freq = config_.output_clock_speed;

  // Sample rate configuration
  dev->sample_rate_conf.val = 0;
  dev->sample_rate_conf.rx_bits_mod = 16;  // 16-bit parallel
  dev->sample_rate_conf.tx_bits_mod = 16;

#if defined(CONFIG_IDF_TARGET_ESP32S2)
  // ESP32-S2: Use PLL_160M
  dev->clkm_conf.clk_sel = 2;  // PLL_160M_CLK
  dev->clkm_conf.clkm_div_a = 1;
  dev->clkm_conf.clkm_div_b = 0;

  // Output Frequency = (160MHz / clkm_div_num) / (tx_bck_div_num*2)
  unsigned int div_num = (freq > 8000000) ? 2 : 4;  // 20MHz or 10MHz
  dev->clkm_conf.clkm_div_num = div_num;
  dev->clkm_conf.clk_en = 1;

  // BCK divider (must be >= 2 per TRM)
  dev->sample_rate_conf.rx_bck_div_num = 2;
  dev->sample_rate_conf.tx_bck_div_num = 2;

  ESP_LOGI(TAG, "ESP32-S2 I2S clock: 160MHz / %d / 4 = %d MHz", div_num, 160 / div_num / 4);
#else
  // ESP32: Use PLL_D2 (80MHz)
  dev->clkm_conf.clka_en = 0;     // Use PLL_D2_CLK (80MHz)
  dev->clkm_conf.clkm_div_a = 1;  // Denominator
  dev->clkm_conf.clkm_div_b = 0;  // Numerator

  // Calculate divider: 80MHz / clkm_div_num / tx_bck_div_num
  unsigned int div_num = (freq > 8000000) ? 2 : 4;  // 20MHz or 10MHz
  dev->clkm_conf.clkm_div_num = div_num;

  // BCK divider (must be >= 2 per TRM)
  dev->sample_rate_conf.tx_bck_div_num = 2;
  dev->sample_rate_conf.rx_bck_div_num = 2;

  ESP_LOGI(TAG, "ESP32 I2S clock: 80MHz / %d / 4 = %d MHz", div_num, 80 / div_num / 4);
#endif
}

void ESP32_I2S_DMA::configureGPIO() {
  // GPIO matrix signals
#if defined(CONFIG_IDF_TARGET_ESP32S2)
  int iomux_signal_base = I2S0O_DATA_OUT8_IDX;
  int iomux_clock = I2S0O_WS_OUT_IDX;
#else
  int iomux_signal_base = (ESP32_I2S_DEVICE == 0) ? I2S0O_DATA_OUT8_IDX : I2S1O_DATA_OUT8_IDX;
  int iomux_clock = (ESP32_I2S_DEVICE == 0) ? I2S0O_WS_OUT_IDX : I2S1O_WS_OUT_IDX;
#endif

  // Map HUB75 pins to I2S data lines (16-bit parallel output)
  int8_t pin_map[16] = {
      config_.pins.r1,   // D0
      config_.pins.g1,   // D1
      config_.pins.b1,   // D2
      config_.pins.r2,   // D3
      config_.pins.g2,   // D4
      config_.pins.b2,   // D5
      config_.pins.a,    // D6
      config_.pins.b,    // D7
      config_.pins.c,    // D8
      config_.pins.d,    // D9
      config_.pins.e,    // D10
      config_.pins.lat,  // D11
      config_.pins.oe,   // D12
      -1,                // D13 unused
      -1,                // D14 unused
      -1                 // D15 unused
  };

  // Initialize all GPIO pins
  for (int i = 0; i < 16; i++) {
    if (pin_map[i] >= 0) {
      esp_rom_gpio_pad_select_gpio(pin_map[i]);
      gpio_set_direction((gpio_num_t) pin_map[i], GPIO_MODE_OUTPUT);
      gpio_set_drive_capability((gpio_num_t) pin_map[i], GPIO_DRIVE_CAP_3);
      esp_rom_gpio_connect_out_signal(pin_map[i], iomux_signal_base + i, false, false);
    }
  }

  // Clock pin (CLK)
  if (config_.pins.clk >= 0) {
    esp_rom_gpio_pad_select_gpio(config_.pins.clk);
    gpio_set_direction((gpio_num_t) config_.pins.clk, GPIO_MODE_OUTPUT);
    gpio_set_drive_capability((gpio_num_t) config_.pins.clk, GPIO_DRIVE_CAP_3);
    esp_rom_gpio_connect_out_signal(config_.pins.clk, iomux_clock, config_.clk_phase_inverted, false);
  }

  ESP_LOGI(TAG, "GPIO matrix configured for HUB75 pins");
}

bool ESP32_I2S_DMA::setupCircularDMA(uint8_t *buffer_a, uint8_t *buffer_b, size_t size) {
  // Calculate BCM timing (determines lsbMsbTransitionBit for OE control)
  bit_depth_ = config_.bit_depth;
  calculateBCMTimings();

  // Allocate per-row bit-plane buffers
  if (!allocateRowBuffers()) {
    return false;
  }

  // Initialize buffers with blank pixels (control bits only, RGB=0)
  initializeBlankBuffers();

  // Set OE bits for BCM control and brightness
  setBrightnessOE();

  // Build descriptor chain with BCM repetitions
  if (!buildDescriptorChain()) {
    return false;
  }

  ESP_LOGI(TAG, "Descriptor-chain DMA setup complete");
  return true;
}

bool ESP32_I2S_DMA::allocateRowBuffers() {
  uint16_t total_width = config_.width * config_.chain_length;
  size_t pixels_per_bitplane = total_width;  // Fixed size per bit plane (no padding)

  row_buffers_ = new RowBitPlaneBuffer[num_rows_];

  for (int row = 0; row < num_rows_; row++) {
    // Allocate contiguous buffer for all bit planes: [bit0][bit1]...[bitN]
    row_buffers_[row].buffer_size = pixels_per_bitplane * bit_depth_ * 2;  // uint16_t = 2 bytes

    row_buffers_[row].data = (uint8_t *) heap_caps_malloc(row_buffers_[row].buffer_size, MALLOC_CAP_DMA);

    if (!row_buffers_[row].data) {
      ESP_LOGE(TAG, "Failed to allocate row buffer %d", row);
      return false;
    }
  }

  size_t total_mem = num_rows_ * row_buffers_[0].buffer_size;
  ESP_LOGI(TAG, "Allocated %d row buffers: %zu bytes each, %zu total", num_rows_, row_buffers_[0].buffer_size,
           total_mem);
  return true;
}

void ESP32_I2S_DMA::startTransfer() {
  if (!i2s_dev_) {
    ESP_LOGE(TAG, "I2S device not initialized");
    return;
  }

  ESP_LOGI(TAG, "Starting descriptor-chain DMA:");
  ESP_LOGI(TAG, "  Descriptor count: %zu", descriptor_count_);
  ESP_LOGI(TAG, "  Rows: %d, Bits: %d", num_rows_, bit_depth_);

  // Configure DMA burst mode
  i2s_dev_->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN;

  // Set address of first DMA descriptor
  i2s_dev_->out_link.addr = (uint32_t) &descriptors_[0];

  // Start DMA operation
  i2s_dev_->out_link.stop = 0;
  i2s_dev_->out_link.start = 1;

  // Start I2S transmission (will run continuously via descriptor loop)
  i2s_dev_->conf.tx_start = 1;

  ESP_LOGI(TAG, "Descriptor-chain DMA transfer started - running continuously");
}

void ESP32_I2S_DMA::stopTransfer() {
  if (!i2s_dev_) {
    return;
  }

  // Stop I2S transmission
  i2s_dev_->conf.tx_start = 0;

  // Stop DMA
  i2s_dev_->out_link.stop = 1;
  i2s_dev_->out_link.start = 0;

  ESP_LOGI(TAG, "DMA transfer stopped");
}

void ESP32_I2S_DMA::shutdown() {
  stopTransfer();

  // Free descriptor chain
  if (descriptors_) {
    heap_caps_free(descriptors_);
    descriptors_ = nullptr;
    descriptor_count_ = 0;
  }

  // Free row buffers
  if (row_buffers_) {
    for (int i = 0; i < num_rows_; i++) {
      if (row_buffers_[i].data) {
        heap_caps_free(row_buffers_[i].data);
      }
    }
    delete[] row_buffers_;
    row_buffers_ = nullptr;
  }

  // Disable I2S peripheral
  if (i2s_dev_) {
    if (ESP32_I2S_DEVICE == 0) {
      periph_module_disable(PERIPH_I2S0_MODULE);
    } else {
#if !defined(CONFIG_IDF_TARGET_ESP32S2)
      periph_module_disable(PERIPH_I2S1_MODULE);
#endif
    }
    i2s_dev_ = nullptr;
  }

  ESP_LOGI(TAG, "Shutdown complete");
}

// ============================================================================
// Brightness Control
// ============================================================================

void ESP32_I2S_DMA::setBasisBrightness(uint8_t brightness) {
  // Clamp to valid range (1-255)
  if (brightness == 0) {
    brightness = 1;
  }

  basis_brightness_ = brightness;
  ESP_LOGI(TAG, "Basis brightness set to %u", (unsigned) brightness);

  // Apply brightness change immediately by updating OE bits in DMA buffers
  setBrightnessOE();
}

void ESP32_I2S_DMA::setIntensity(float intensity) {
  // Clamp to valid range (0.0-1.0)
  if (intensity < 0.0f) {
    intensity = 0.0f;
  } else if (intensity > 1.0f) {
    intensity = 1.0f;
  }

  intensity_ = intensity;
  ESP_LOGI(TAG, "Intensity set to %.2f", intensity);

  // Apply intensity change immediately by updating OE bits in DMA buffers
  setBrightnessOE();
}

// ============================================================================
// BCM Timing Calculation
// ============================================================================

void ESP32_I2S_DMA::calculateBCMTimings() {
  // Calculate buffer transmission time
  uint16_t total_width = config_.width * config_.chain_length;
  uint16_t buffer_pixels = total_width;
  float buffer_time_us = (buffer_pixels * 1000000.0f) / config_.output_clock_speed;

  ESP_LOGI(TAG, "Buffer transmission time: %.2f µs (%u pixels @ %lu Hz)", buffer_time_us, (unsigned) buffer_pixels,
           (unsigned long) config_.output_clock_speed);

  // Target refresh rate from config
  uint32_t target_hz = config_.min_refresh_rate;
  uint32_t num_rows = config_.height / 2;

  // Calculate optimal lsbMsbTransitionBit to achieve target refresh rate
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
    ESP_LOGW(TAG,
             "Using lsbMsbTransitionBit=%d, lower %d bits show once "
             "(reduced color depth for speed)",
             lsbMsbTransitionBit_, lsbMsbTransitionBit_ + 1);
  }

  ESP_LOGI(TAG, "BCM timing calculated (lsbMsbTransitionBit used by setBrightnessOE for OE control)");
}

// ============================================================================
// Buffer Initialization
// ============================================================================

void ESP32_I2S_DMA::initializeBlankBuffers() {
  if (!row_buffers_) {
    ESP_LOGE(TAG, "Row buffers not allocated");
    return;
  }

  uint16_t total_width = config_.width * config_.chain_length;

  // HUB75 control bit positions in 16-bit word
  const uint16_t OE_BIT = (1 << 12);
  const uint16_t LAT_BIT = (1 << 11);

  for (int row = 0; row < num_rows_; row++) {
    // Calculate row address (ABCDE bits)
    uint16_t row_addr = row & 0x1F;

    for (int bit = 0; bit < bit_depth_; bit++) {
      // Get pointer to this bit plane's buffer
      uint16_t *buf = (uint16_t *) (row_buffers_[row].data + bit * total_width * 2);

      // For bit plane 0 (LSB), use previous row's address for LAT settling
      // This prevents ghosting artifacts during row transitions
      uint16_t addr_for_buffer;
      if (bit == 0) {
        if (row == 0) {
          addr_for_buffer = (num_rows_ - 1) & 0x1F;  // Wrap to last row
        } else {
          addr_for_buffer = (row - 1) & 0x1F;
        }
      } else {
        addr_for_buffer = row_addr;
      }

      // Fill all pixels with control bits (RGB=0, row address, OE=HIGH)
      for (uint16_t x = 0; x < total_width; x++) {
        buf[x] = (addr_for_buffer << 6) | OE_BIT;
      }

      // Set LAT bit on last pixel
      buf[total_width - 1] |= LAT_BIT;
    }
  }

  ESP_LOGI(TAG, "Blank buffers initialized");
}

void ESP32_I2S_DMA::setBrightnessOE() {
  if (!row_buffers_) {
    ESP_LOGE(TAG, "Row buffers not allocated");
    return;
  }

  uint16_t total_width = config_.width * config_.chain_length;
  uint8_t latch_blanking = config_.latch_blanking;

  // Calculate brightness scaling (0-255 maps to 0-255)
  uint8_t brightness = (uint8_t) ((float) basis_brightness_ * intensity_);

  ESP_LOGI(TAG, "Setting brightness OE: brightness=%u, lsbMsbTransitionBit=%u", brightness, lsbMsbTransitionBit_);

  const uint16_t OE_BIT = (1 << 12);
  const uint16_t OE_CLEAR_MASK = ~OE_BIT;

  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      // Get pointer to this bit plane's buffer
      uint16_t *buf = (uint16_t *) (row_buffers_[row].data + bit * total_width * 2);

      // Calculate BCM weighting for this bit plane
      int bitplane = (2 * bit_depth_ - bit) % bit_depth_;
      int bitshift = (bit_depth_ - lsbMsbTransitionBit_ - 1) >> 1;
      int rightshift = std::max(bitplane - bitshift - 2, 0);

      // Calculate display pixel count for this bit plane
      int max_pixels = (total_width - latch_blanking) >> rightshift;
      int display_pixels = (max_pixels * brightness) >> 8;

      // Ensure at least 1 pixel for brightness > 0
      if (brightness > 0 && display_pixels == 0) {
        display_pixels = 1;
      }

      // Safety margin to prevent ghosting
      if (display_pixels > max_pixels - 1) {
        display_pixels = max_pixels - 1;
      }

      // Calculate center region for OE=LOW (display enabled)
      int x_min = (total_width - display_pixels) / 2;
      int x_max = (total_width + display_pixels) / 2;

      // Set OE bits: LOW in center (display), HIGH elsewhere (blanked)
      for (int x = 0; x < total_width; x++) {
        if (x >= x_min && x < x_max) {
          // Enable display: clear OE bit
          buf[x] &= OE_CLEAR_MASK;
        } else {
          // Keep blanked: set OE bit
          buf[x] |= OE_BIT;
        }
      }

      // CRITICAL: Latch blanking to prevent ghosting
      int last_pixel = total_width - 1;

      // Blank LAT pixel itself
      buf[last_pixel] |= OE_BIT;

      // Blank latch_blanking pixels BEFORE LAT
      for (int i = 1; i <= latch_blanking && (last_pixel - i) >= 0; i++) {
        buf[last_pixel - i] |= OE_BIT;
      }

      // Blank latch_blanking pixels at START of buffer
      for (int i = 0; i < latch_blanking && i < total_width; i++) {
        buf[i] |= OE_BIT;
      }
    }
  }

  ESP_LOGI(TAG, "Brightness OE configuration complete");
}

// ============================================================================
// Descriptor Chain Building (with BCM Duplication)
// ============================================================================

bool ESP32_I2S_DMA::buildDescriptorChain() {
  uint16_t total_width = config_.width * config_.chain_length;
  size_t pixels_per_bitplane = total_width;
  size_t bytes_per_bitplane = pixels_per_bitplane * 2;  // uint16_t = 2 bytes

  // Calculate total descriptors needed WITH BCM repetitions
  // I2S descriptors (lldesc_t) don't support repetition count,
  // so we create multiple descriptors pointing to the same buffer
  descriptor_count_ = 0;
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      if (bit <= lsbMsbTransitionBit_) {
        descriptor_count_ += 1;  // Base timing
      } else {
        descriptor_count_ += (1 << (bit - lsbMsbTransitionBit_ - 1));  // BCM repetitions
      }
    }
  }

  size_t descriptors_per_row = descriptor_count_ / num_rows_;
  size_t total_descriptor_bytes = sizeof(lldesc_t) * descriptor_count_;

  ESP_LOGI(TAG, "Building BCM descriptor chain: %zu descriptors (%zu per row) for %d rows × %d bits", descriptor_count_,
           descriptors_per_row, num_rows_, bit_depth_);
  ESP_LOGI(TAG, "  BCM via descriptor duplication (lsbMsbTransitionBit=%d)", lsbMsbTransitionBit_);
  ESP_LOGI(TAG, "  Allocating %zu bytes for descriptor array", total_descriptor_bytes);

  // Free existing descriptors if already allocated
  if (descriptors_) {
    heap_caps_free(descriptors_);
    descriptors_ = nullptr;
  }

  // Allocate single contiguous block for all descriptors
  descriptors_ = (lldesc_t *) heap_caps_malloc(total_descriptor_bytes, MALLOC_CAP_DMA);
  if (!descriptors_) {
    ESP_LOGE(TAG, "Failed to allocate %zu descriptors (%zu bytes) in DMA memory", descriptor_count_,
             total_descriptor_bytes);
    return false;
  }

  // Link descriptors with BCM repetitions
  size_t desc_idx = 0;
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      uint8_t *bit_buffer = row_buffers_[row].data + bit * bytes_per_bitplane;

      // Calculate number of descriptor repetitions for this bit plane
      int repetitions;
      if (bit <= lsbMsbTransitionBit_) {
        repetitions = 1;  // Base timing for LSBs
      } else {
        // BCM weighting: 2^(bit - lsbMsbTransitionBit - 1) repetitions
        repetitions = (1 << (bit - lsbMsbTransitionBit_ - 1));
      }

      // Create 'repetitions' descriptors, all pointing to the SAME buffer
      // This achieves BCM timing via temporal repetition
      for (int rep = 0; rep < repetitions; rep++) {
        lldesc_t *desc = &descriptors_[desc_idx];
        desc->size = bytes_per_bitplane;
        desc->length = bytes_per_bitplane;
        desc->buf = bit_buffer;  // Same buffer for all repetitions
        desc->eof = 0;           // EOF only on last descriptor
        desc->sosf = 0;
        desc->owner = 1;
        desc->offset = 0;

        // Link to next descriptor
        if (desc_idx < descriptor_count_ - 1) {
          desc->qe.stqe_next = &descriptors_[desc_idx + 1];
        }

        desc_idx++;
      }
    }
  }

  // Last descriptor loops back to first (continuous refresh)
  descriptors_[descriptor_count_ - 1].qe.stqe_next = &descriptors_[0];
  descriptors_[descriptor_count_ - 1].eof = 1;  // EOF once per frame

  ESP_LOGI(TAG, "BCM descriptor chain built: %zu descriptors in continuous loop", descriptor_count_);
  return true;
}

// ============================================================================
// Pixel API
// ============================================================================

void ESP32_I2S_DMA::setLUT(const uint16_t *lut) { lut_ = lut; }

HUB75_IRAM void ESP32_I2S_DMA::drawPixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer,
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

      // Extract RGB based on format
      switch (format) {
        case HUB75_PIXEL_FORMAT_RGB888: {
          // 24-bit packed RGB
          const uint8_t *p = buffer + pixel_idx * 3;
          r8 = p[0];
          g8 = p[1];
          b8 = p[2];
          break;
        }

        case HUB75_PIXEL_FORMAT_RGB888_32: {
          // 32-bit RGB with padding
          const uint8_t *p = buffer + pixel_idx * 4;
          if (color_order == HUB75_COLOR_ORDER_RGB) {
            if (big_endian) {
              // Big-endian xRGB: [x][R][G][B]
              r8 = p[1];
              g8 = p[2];
              b8 = p[3];
            } else {
              // Little-endian xRGB stored as BGRx: [B][G][R][x]
              r8 = p[2];
              g8 = p[1];
              b8 = p[0];
            }
          } else {  // BGR
            if (big_endian) {
              // Big-endian xBGR: [x][B][G][R]
              r8 = p[3];
              g8 = p[2];
              b8 = p[1];
            } else {
              // Little-endian xBGR stored as RGBx: [R][G][B][x]
              r8 = p[0];
              g8 = p[1];
              b8 = p[2];
            }
          }
          break;
        }

        case HUB75_PIXEL_FORMAT_RGB565: {
          // 16-bit RGB565
          const uint8_t *p = buffer + pixel_idx * 2;
          uint16_t rgb565;
          if (big_endian) {
            rgb565 = (uint16_t(p[0]) << 8) | p[1];
          } else {
            rgb565 = (uint16_t(p[1]) << 8) | p[0];
          }

          // Extract RGB565 components
          uint8_t r5 = (rgb565 >> 11) & 0x1F;
          uint8_t g6 = (rgb565 >> 5) & 0x3F;
          uint8_t b5 = rgb565 & 0x1F;

          // Scale to 8-bit
          r8 = (r5 * 527 + 23) >> 6;
          g8 = (g6 * 259 + 33) >> 6;
          b8 = (b5 * 527 + 23) >> 6;
          break;
        }

        default:
          continue;  // Skip unknown formats
      }

      // Apply LUT correction
      uint16_t r_corrected = lut_[r8];
      uint16_t g_corrected = lut_[g8];
      uint16_t b_corrected = lut_[b8];

      // Update all bit planes for this pixel
      for (int bit = 0; bit < bit_depth_; bit++) {
        uint16_t *buf = (uint16_t *) (row_buffers_[row].data + bit * total_width * 2);

        uint16_t mask = (1 << bit);
        uint16_t word = buf[px];  // Read existing word (preserves control bits)

        // Clear and update RGB bits for appropriate half
        // IMPORTANT: Only modify RGB bits (0-5), preserve control bits (6-12)
        if (is_lower) {
          // Lower half: R2, G2, B2 (bits 3, 4, 5)
          word &= ~((1 << 3) | (1 << 4) | (1 << 5));
          if (r_corrected & mask)
            word |= (1 << 3);
          if (g_corrected & mask)
            word |= (1 << 4);
          if (b_corrected & mask)
            word |= (1 << 5);
        } else {
          // Upper half: R1, G1, B1 (bits 0, 1, 2)
          word &= ~((1 << 0) | (1 << 1) | (1 << 2));
          if (r_corrected & mask)
            word |= (1 << 0);
          if (g_corrected & mask)
            word |= (1 << 1);
          if (b_corrected & mask)
            word |= (1 << 2);
        }

        buf[px] = word;
      }
    }
  }
}

void ESP32_I2S_DMA::clear() {
  if (!row_buffers_) {
    return;
  }

  uint16_t total_width = config_.width * config_.chain_length;

  // Clear RGB bits in all buffers (preserve control bits)
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      uint16_t *buf = (uint16_t *) (row_buffers_[row].data + bit * total_width * 2);

      for (uint16_t x = 0; x < total_width; x++) {
        // Clear RGB bits (0-5), preserve control bits (6-12)
        buf[x] &= 0xFFC0;  // Keep bits 6-15, clear bits 0-5
      }
    }
  }

  ESP_LOGI(TAG, "Display cleared");
}

}  // namespace hub75

#endif  // CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
