/**
 * SPDX-FileCopyrightText: 2025 Stuart Parmenter
 * SPDX-License-Identifier: MIT
 *
 * @file dma_gdma.cpp
 * @brief ESP32-S3 LCD_CAM + GDMA implementation for HUB75
 *
 * Uses direct LCD_CAM register access and manual GDMA setup.
 * Simplified ring buffer approach with software BCM state tracking.
 */

#include <sdkconfig.h>

// Only compile for ESP32-S3
#ifdef CONFIG_IDF_TARGET_ESP32S3

#include "dma_gdma.hpp"
#include "../../color/color_convert.hpp"  // For unpack_rgb888/unpack_rgb121212
#include <cstring>                        // for memcpy
#include <algorithm>                      // for std::max
#include <esp_log.h>
#include <esp_rom_gpio.h>
#include <esp_rom_sys.h>
#include <driver/gpio.h>
#include <esp_private/gpio.h>
#include <esp_private/gdma.h>
#include <soc/gpio_sig_map.h>
#include <soc/lcd_cam_struct.h>
#include <hal/gpio_hal.h>
#include <hal/gdma_ll.h>
#include <esp_private/periph_ctrl.h>
#include <esp_heap_caps.h>

static const char *const TAG = "ESP32S3_GDMA";

namespace hub75 {

ESP32S3_GDMA::ESP32S3_GDMA(const hub75_config_t &config)
    : config_(config),
      dma_chan_(nullptr),
      bit_depth_(config.bit_depth),
      lsbMsbTransitionBit_(0),
      row_buffers_(nullptr),
      num_rows_(config.height / 2),
      descriptors_(nullptr),
      descriptor_count_(0),
      basis_brightness_(config.brightness),  // Use config value (default: 128)
      intensity_(1.0f),
      lut_(nullptr) {
  // Zero-copy architecture: DMA buffers ARE the display memory
}

ESP32S3_GDMA::~ESP32S3_GDMA() { shutdown(); }

bool ESP32S3_GDMA::init() {
  ESP_EARLY_LOGI("ESP32S3_GDMA", "*** GDMA INIT() CALLED ***");
  ESP_LOGI(TAG, "Initializing LCD_CAM peripheral with GDMA...");
  ESP_LOGI(TAG, "Pin config: R1=%d G1=%d B1=%d R2=%d G2=%d B2=%d", config_.pins.r1, config_.pins.g1, config_.pins.b1,
           config_.pins.r2, config_.pins.g2, config_.pins.b2);
  ESP_LOGI(TAG, "Pin config: A=%d B=%d C=%d D=%d E=%d LAT=%d OE=%d CLK=%d", config_.pins.a, config_.pins.b,
           config_.pins.c, config_.pins.d, config_.pins.e, config_.pins.lat, config_.pins.oe, config_.pins.clk);

  // Enable and reset LCD_CAM peripheral
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);

  // Reset LCD bus
  LCD_CAM.lcd_user.lcd_reset = 1;
  esp_rom_delay_us(1000);

  // Configure LCD clock
  configureLCDClock();

  // Configure LCD mode (i8080 16-bit parallel)
  configureLCDMode();

  // Configure GPIO routing
  configureGPIO();

  // Allocate GDMA channel
  ESP_EARLY_LOGI("GDMA", "About to allocate GDMA channel");
  gdma_channel_alloc_config_t dma_alloc_config = {.sibling_chan = nullptr,
                                                  .direction = GDMA_CHANNEL_DIRECTION_TX,
                                                  .flags = {.reserve_sibling = 0, .isr_cache_safe = 0}};

  esp_err_t err = gdma_new_ahb_channel(&dma_alloc_config, &dma_chan_);
  if (err != ESP_OK) {
    ESP_EARLY_LOGE("GDMA", "FAILED to allocate GDMA channel: 0x%x", err);
    ESP_LOGE(TAG, "Failed to allocate GDMA channel: %s", esp_err_to_name(err));
    return false;
  }
  ESP_EARLY_LOGI("GDMA", "GDMA channel allocated successfully");

  // Connect GDMA to LCD peripheral
  gdma_connect(dma_chan_, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));

  // Configure GDMA strategy
  // owner_check = false: Static descriptors, no dynamic ownership handshaking needed
  // auto_update_desc = false: No descriptor writeback - prevents corruption with infinite ring
  // eof_till_data_popped = true: EOF fires after LCD consumes data (includes display period timing!)
  //   This is CRITICAL for proper BCM timing - EOF waits for the OE=LOW display period to complete
  gdma_strategy_config_t strategy_config = {
      .owner_check = false, .auto_update_desc = false, .eof_till_data_popped = true};
  gdma_apply_strategy(dma_chan_, &strategy_config);

  ESP_LOGI(TAG, "GDMA strategy configured: owner_check=false, auto_update_desc=false, eof_till_data_popped=true");

  // Configure GDMA transfer for SRAM (not PSRAM)
  gdma_transfer_config_t transfer_config = {
      .max_data_burst_size = 32,  // 32 bytes for SRAM
      .access_ext_mem = false     // Not accessing external memory
  };
  gdma_config_transfer(dma_chan_, &transfer_config);

  // Wait for any pending LCD operations
  while (LCD_CAM.lcd_user.lcd_start)
    ;

  // Post-init cleanup for clean state
  gdma_reset(dma_chan_);
  esp_rom_delay_us(1000);
  LCD_CAM.lcd_user.lcd_dout = 1;         // Enable data out
  LCD_CAM.lcd_user.lcd_update = 1;       // Update registers
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;  // Reset LCD TX FIFO

  // Note: No EOF callback needed with descriptor-chain approach
  // The descriptor chain encodes all timing via repetition counts

  ESP_LOGI(TAG, "GDMA EOF callback registered successfully");

  ESP_EARLY_LOGI("GDMA", "*** GDMA INIT COMPLETE ***");
  ESP_LOGI(TAG, "LCD_CAM + GDMA initialized successfully");
  ESP_LOGI(TAG, "Clock: %u MHz", (unsigned int) (config_.output_clock_speed / 1000000));

  return true;
}

void ESP32S3_GDMA::configureLCDClock() {
  // Configure LCD clock from PLL_F160M (160 MHz)
  // Calculate divider: 160MHz / desired_speed
  uint32_t div_num = 160000000 / config_.output_clock_speed;
  if (div_num < 2)
    div_num = 2;  // Minimum divider

  LCD_CAM.lcd_clock.lcd_clk_sel = 3;      // PLL_F160M_CLK (value 3, not 2!)
  LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;  // PCLK low in 1st half cycle
  LCD_CAM.lcd_clock.lcd_ck_idle_edge = config_.clk_phase_inverted ? 1 : 0;
  LCD_CAM.lcd_clock.lcd_clkcnt_n = 1;        // Should never be zero
  LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 1;  // PCLK = CLK / 1 (simple divisor)
  LCD_CAM.lcd_clock.lcd_clkm_div_num = div_num;
  LCD_CAM.lcd_clock.lcd_clkm_div_a = 1;  // Fractional divider (0/1)
  LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;

  ESP_LOGI(TAG, "LCD clock: PLL_F160M / %u = %u MHz", (unsigned int) div_num,
           (unsigned int) (160000000 / div_num / 1000000));
}

void ESP32S3_GDMA::configureLCDMode() {
  // Configure LCD in i8080 mode, 16-bit parallel, continuous output
  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;     // i8080 mode (not RGB)
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0;  // Disable RGB/YUV converter
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;   // Do NOT auto-frame
  LCD_CAM.lcd_misc.lcd_bk_en = 1;           // Enable blanking
  LCD_CAM.lcd_misc.lcd_vfk_cyclelen = 0;
  LCD_CAM.lcd_misc.lcd_vbk_cyclelen = 0;

  LCD_CAM.lcd_data_dout_mode.val = 0;      // No data delays
  LCD_CAM.lcd_user.lcd_always_out_en = 1;  // Enable 'always out' mode for arbitrary-length transfers
  LCD_CAM.lcd_user.lcd_8bits_order = 0;    // Do not swap bytes
  LCD_CAM.lcd_user.lcd_bit_order = 0;      // Do not reverse bit order
  LCD_CAM.lcd_user.lcd_2byte_en = 1;       // 16-bit mode
  LCD_CAM.lcd_user.lcd_dout = 1;           // Enable data output

  // CRITICAL: Dummy phases required for DMA to trigger reliably
  LCD_CAM.lcd_user.lcd_dummy = 1;           // Dummy phase(s) @ LCD start
  LCD_CAM.lcd_user.lcd_dummy_cyclelen = 1;  // 1+1 dummy phase
  LCD_CAM.lcd_user.lcd_cmd = 0;             // No command at LCD start

  // Disable start signal
  LCD_CAM.lcd_user.lcd_start = 0;
}

void ESP32S3_GDMA::configureGPIO() {
  // 16-bit data pins mapping
  int data_pins[16] = {
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
      -1,
      -1,
      -1  // D13-D15 unused
  };

  // Configure data pins
  for (int i = 0; i < 16; i++) {
    if (data_pins[i] >= 0) {
      esp_rom_gpio_connect_out_signal(data_pins[i], LCD_DATA_OUT0_IDX + i, false, false);
      gpio_func_sel((gpio_num_t) data_pins[i], PIN_FUNC_GPIO);                 // ESP-IDF 5.4+
      gpio_set_drive_capability((gpio_num_t) data_pins[i], GPIO_DRIVE_CAP_3);  // Max drive strength
    }
  }

  // Configure WR (clock) pin
  if (config_.pins.clk >= 0) {
    esp_rom_gpio_connect_out_signal(config_.pins.clk, LCD_PCLK_IDX, config_.clk_phase_inverted, false);
    gpio_func_sel((gpio_num_t) config_.pins.clk, PIN_FUNC_GPIO);                 // ESP-IDF 5.4+
    gpio_set_drive_capability((gpio_num_t) config_.pins.clk, GPIO_DRIVE_CAP_3);  // Max drive strength
  }

  ESP_LOGD(TAG, "GPIO routing configured");
}

bool ESP32S3_GDMA::setupCircularDMA(uint8_t *buffer_a, uint8_t *buffer_b, size_t size) {
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

  // Build descriptor chain (one descriptor per bit plane)
  if (!buildDescriptorChain()) {
    return false;
  }

  ESP_LOGI(TAG, "Descriptor-chain DMA setup complete");
  return true;
}

bool ESP32S3_GDMA::allocateRowBuffers() {
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

void ESP32S3_GDMA::startTransfer() {
  if (!dma_chan_) {
    ESP_LOGE(TAG, "DMA channel not initialized");
    return;
  }

  ESP_LOGI(TAG, "Starting descriptor-chain DMA:");
  ESP_LOGI(TAG, "  Descriptor count: %zu", descriptor_count_);
  ESP_LOGI(TAG, "  Rows: %d, Bits: %d", num_rows_, bit_depth_);

  // Prime LCD registers
  LCD_CAM.lcd_user.lcd_update = 1;
  esp_rom_delay_us(10);

  // Start GDMA transfer from first descriptor in chain
  gdma_start(dma_chan_, (intptr_t) &descriptors_[0]);

  // Delay before starting LCD
  esp_rom_delay_us(100);

  // Start LCD engine (will run continuously via descriptor loop)
  LCD_CAM.lcd_user.lcd_start = 1;

  ESP_LOGI(TAG, "Descriptor-chain DMA transfer started - running continuously");
}

void ESP32S3_GDMA::stopTransfer() {
  if (!dma_chan_) {
    return;
  }

  // Disable LCD output
  LCD_CAM.lcd_user.lcd_start = 0;
  LCD_CAM.lcd_user.lcd_update = 1;  // Apply the stop command

  gdma_stop(dma_chan_);

  ESP_LOGI(TAG, "DMA transfer stopped");
}

// No EOF callback needed - descriptor chain handles all timing

void ESP32S3_GDMA::shutdown() {
  stopTransfer();

  if (dma_chan_) {
    gdma_disconnect(dma_chan_);
    gdma_del_channel(dma_chan_);
    dma_chan_ = nullptr;
  }

  // Free descriptor chain (single contiguous block)
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

  periph_module_disable(PERIPH_LCD_CAM_MODULE);

  ESP_LOGI(TAG, "Shutdown complete");
}

// ============================================================================
// Brightness Control (Override Base Class)
// ============================================================================

void ESP32S3_GDMA::setBasisBrightness(uint8_t brightness) {
  // Clamp to valid range (1-255)
  if (brightness == 0) {
    brightness = 1;
  }

  basis_brightness_ = brightness;

  ESP_LOGI(TAG, "Basis brightness set to %u", (unsigned) brightness);

  // Apply brightness change immediately by updating OE bits in DMA buffers
  setBrightnessOE();
}

void ESP32S3_GDMA::setIntensity(float intensity) {
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
// Pixel API (Direct DMA Buffer Writes)
// ============================================================================

void ESP32S3_GDMA::setLUT(const uint16_t *lut) { lut_ = lut; }

HUB75_IRAM void ESP32S3_GDMA::drawPixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer,
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

void ESP32S3_GDMA::clear() {
  if (!row_buffers_) {
    return;
  }

  uint16_t total_width = config_.width * config_.chain_length;

  // Clear RGB bits in all buffers (keep control bits)
  for (int row = 0; row < num_rows_; row++) {
    for (int bit = 0; bit < bit_depth_; bit++) {
      uint16_t *buf = (uint16_t *) (row_buffers_[row].data + bit * total_width * 2);

      for (uint16_t x = 0; x < total_width; x++) {
        // Clear RGB bits (bits 0-5) but preserve row address, LAT, OE
        buf[x] &= ~0x003F;  // Clear bits 0-5 (R1,G1,B1,R2,G2,B2)
      }
    }
  }

  ESP_LOGI(TAG, "Display cleared to black");
}

// ============================================================================
// Buffer Initialization
// ============================================================================

void ESP32S3_GDMA::initializeBlankBuffers() {
  if (!row_buffers_) {
    ESP_LOGE(TAG, "Row buffers not allocated");
    return;
  }

  ESP_LOGI(TAG, "Initializing blank DMA buffers with control bits...");

  uint16_t total_width = config_.width * config_.chain_length;

  const uint16_t LAT_BIT = (1 << 11);
  const uint16_t OE_BIT = (1 << 12);

  for (int row = 0; row < num_rows_; row++) {
    uint16_t row_addr = row & 0x1F;

    for (int bit = 0; bit < bit_depth_; bit++) {
      uint16_t *buf = (uint16_t *) (row_buffers_[row].data + bit * total_width * 2);

      // Row address handling: LSB bit plane uses previous row for LAT settling
      //
      // HUB75 panels need time to process the LAT (latch) signal before the row
      // address changes. LAT transfers data from shift registers to the display
      // buffer. If the address changes too quickly, the panel may latch the
      // previous row's data into the current row's buffer.
      //
      // To provide settling time, bit plane 0 (LSB) is marked with the previous
      // row's address, creating a transition period:
      //
      //   Row N completes → Row N+1 bit 0 transmits (address still = N)
      //                  → Panel finishes latching Row N
      //                  → Row N+1 bit 1-7 transmit (address = N+1)
      //
      // This ensures the panel completes Row N's latch operation before it sees
      // the new address in bit planes 1-7.
      //
      // Reference: ESP32-HUB75-MatrixPanel-DMA uses this pattern
      uint16_t addr_for_buffer;
      if (bit == 0) {
        // Bit plane 0: use previous row's address for LAT settling time
        if (row == 0) {
          addr_for_buffer = (num_rows_ - 1) & 0x1F;  // Wrap to last row
        } else {
          addr_for_buffer = (row - 1) & 0x1F;
        }
        ESP_LOGD(TAG, "Row %d Bit 0: Using previous row address 0x%02X (current: 0x%02X)", row, addr_for_buffer,
                 row_addr);
      } else {
        // Bit planes 1-7: use current row's address
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

// ============================================================================
// BCM Control via OE Bit Manipulation
// ============================================================================

void ESP32S3_GDMA::setBrightnessOE() {
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
      // Formula from ESP32-HUB75-MatrixPanel-DMA
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
          // Keep blanked: set OE bit (already set, but make explicit)
          buf[x] |= OE_BIT;
        }
      }

      // CRITICAL: Latch blanking to prevent ghosting
      // Blank pixels around LAT pulse to hide row transitions
      // Pattern from ESP32-HUB75-MatrixPanel-DMA reference library
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

bool ESP32S3_GDMA::buildDescriptorChain() {
  uint16_t total_width = config_.width * config_.chain_length;
  size_t pixels_per_bitplane = total_width;             // Fixed size per bit plane
  size_t bytes_per_bitplane = pixels_per_bitplane * 2;  // uint16_t = 2 bytes

  // Calculate total descriptors needed WITH BCM repetitions
  // For bits <= lsbMsbTransitionBit: 1 descriptor each (base timing)
  // For bits > lsbMsbTransitionBit: 2^(bit - lsbMsbTransitionBit - 1) descriptors each
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
  size_t total_descriptor_bytes = sizeof(dma_descriptor_t) * descriptor_count_;

  ESP_LOGI(TAG, "Building BCM descriptor chain: %zu descriptors (%zu per row) for %d rows × %d bits", descriptor_count_,
           descriptors_per_row, num_rows_, bit_depth_);
  ESP_LOGI(TAG, "  BCM via descriptor repetition (lsbMsbTransitionBit=%d)", lsbMsbTransitionBit_);
  ESP_LOGI(TAG, "  Allocating %zu bytes for descriptor array", total_descriptor_bytes);

  // Free existing descriptors if already allocated (prevent leak on retry)
  if (descriptors_) {
    heap_caps_free(descriptors_);
    descriptors_ = nullptr;
  }

  // Allocate single contiguous block for all descriptors (matches reference library)
  descriptors_ = (dma_descriptor_t *) heap_caps_malloc(total_descriptor_bytes, MALLOC_CAP_DMA);
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
        dma_descriptor_t *desc = &descriptors_[desc_idx];
        desc->dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
        desc->dw0.suc_eof = 0;  // EOF only on last descriptor
        desc->dw0.size = bytes_per_bitplane;
        desc->dw0.length = bytes_per_bitplane;
        desc->buffer = bit_buffer;  // Same buffer for all repetitions

        // Link to next descriptor
        if (desc_idx < descriptor_count_ - 1) {
          desc->next = &descriptors_[desc_idx + 1];
        }

        desc_idx++;
      }
    }
  }

  // Last descriptor loops back to first (continuous refresh)
  descriptors_[descriptor_count_ - 1].next = &descriptors_[0];
  descriptors_[descriptor_count_ - 1].dw0.suc_eof = 1;  // Optional: EOF once per frame

  ESP_LOGI(TAG, "BCM descriptor chain built: %zu descriptors in continuous loop", descriptor_count_);
  return true;
}

// ============================================================================
// BCM Timing Calculation (Platform-Specific)
// ============================================================================

void ESP32S3_GDMA::calculateBCMTimings() {
  // Calculate buffer transmission time
  // Buffer contains total_width pixels with LAT on last pixel
  // Latch blanking is handled via OE bits, not extra pixels
  uint16_t total_width = config_.width * config_.chain_length;
  uint16_t buffer_pixels = total_width;  // LAT is on last pixel, not extra
  float buffer_time_us = (buffer_pixels * 1000000.0f) / config_.output_clock_speed;

  ESP_LOGI(TAG, "Buffer transmission time: %.2f µs (%u pixels @ %lu Hz)", buffer_time_us, (unsigned) buffer_pixels,
           (unsigned long) config_.output_clock_speed);

  // Target refresh rate from config
  uint32_t target_hz = config_.min_refresh_rate;

  // Number of rows (1/32 scan for 64-height panels)
  uint32_t num_rows = config_.height / 2;

  // Calculate optimal lsbMsbTransitionBit to achieve target refresh rate
  // Algorithm matches ESP32-HUB75-MatrixPanel-DMA library
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

}  // namespace hub75

#endif  // CONFIG_IDF_TARGET_ESP32S3
