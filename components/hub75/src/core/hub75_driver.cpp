/**
 * SPDX-FileCopyrightText: 2025 Stuart Parmenter
 * SPDX-License-Identifier: MIT
 *
 * @file hub75_driver.cpp
 * @brief Main driver implementation
 */

#include "hub75.h"
#include "../color/color_lut.hpp"
#include "../color/color_convert.hpp"
#include "../platforms/platform_dma.hpp"
#include "../platforms/platform_detect.hpp"

// Include platform-specific DMA implementation
#ifdef CONFIG_IDF_TARGET_ESP32S3
#include "../platforms/gdma/dma_gdma.hpp"
#elif defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)
#include "../platforms/i2s/dma_i2s.hpp"
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
#include "../platforms/parlio/dma_parlio.hpp"
#endif

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <utility>

static const char *const TAG = "HUB75";

using namespace hub75;

// Select platform implementation
#ifdef CONFIG_IDF_TARGET_ESP32S3
using PlatformDMAImpl = ESP32S3_GDMA;
#elif defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)
using PlatformDMAImpl = ESP32_I2S_DMA;
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
using PlatformDMAImpl = ESP32P4_PARLIO_DMA;
#endif

// ============================================================================
// Constructor / Destructor
// ============================================================================

HUB75Driver::HUB75Driver(const hub75_config_t &config)
    : config_(config), running_(false), lut_(nullptr), dma_(nullptr) {
  ESP_LOGI(TAG, "Driver created for %s (%s)", getPlatformName(), getDMAEngineName());
  ESP_LOGI(TAG, "Panel: %dx%d (%u panels × %u), %u-bit depth, scan 1/%u",
           (unsigned int) (config_.width * config_.chain_length), (unsigned int) config_.height,
           (unsigned int) config_.chain_length, (unsigned int) config_.width, (unsigned int) config_.bit_depth,
           (unsigned int) config_.scan_pattern);
}

HUB75Driver::~HUB75Driver() { end(); }

// ============================================================================
// Initialization
// ============================================================================

bool HUB75Driver::begin() {
  if (running_) {
    ESP_LOGW(TAG, "Already running");
    return true;
  }

  ESP_LOGI(TAG, "Initializing HUB75 driver...");

  // Validate configuration
  if (config_.width == 0 || config_.height == 0) {
    ESP_LOGE(TAG, "Invalid panel dimensions");
    return false;
  }

  // Initialize color LUT
  initializeLUT();

  // Create platform-specific DMA implementation
  dma_ = new PlatformDMAImpl(config_);
  if (!dma_ || !dma_->init()) {
    ESP_LOGE(TAG, "Failed to initialize DMA engine");
    return false;
  }

  // Pass LUT for gamma correction during pixel writes
  dma_->setLUT(lut_);

  // Setup circular DMA (platform-specific setup happens inside)
  if (!dma_->setupCircularDMA(nullptr, nullptr, 0)) {
    ESP_LOGE(TAG, "Failed to setup circular DMA");
    dma_->shutdown();
    delete dma_;
    dma_ = nullptr;
    return false;
  }

  // Start DMA transfer
  dma_->startTransfer();

  running_ = true;
  ESP_LOGI(TAG, "Driver started successfully");
  return true;
}

void HUB75Driver::end() {
  if (!running_) {
    return;
  }

  ESP_LOGI(TAG, "Stopping driver...");

  // Shutdown DMA
  if (dma_) {
    dma_->shutdown();
    delete dma_;
    dma_ = nullptr;
  }

  running_ = false;
  ESP_LOGI(TAG, "HUB75 driver stopped");
}

// ============================================================================
// Pixel Drawing
// ============================================================================

HUB75_IRAM void HUB75Driver::drawPixels(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer,
                                        hub75_pixel_format_t format, hub75_color_order_t color_order, bool big_endian) {
  // Forward to platform DMA layer (handles LUT and buffer writes)
  if (dma_) {
    dma_->drawPixels(x, y, w, h, buffer, format, color_order, big_endian);
  }
}

HUB75_IRAM void HUB75Driver::setPixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b) {
  // Single pixel is just a 1x1 drawPixels call with RGB888 format
  uint8_t rgb[3] = {r, g, b};
  drawPixels(x, y, 1, 1, rgb, HUB75_PIXEL_FORMAT_RGB888, HUB75_COLOR_ORDER_RGB, false);
}

void HUB75Driver::clear() {
  // Forward to platform DMA layer
  if (dma_) {
    dma_->clear();
  }
}

// ============================================================================
// Double Buffering
// ============================================================================

void HUB75Driver::clearBuffer() {
  if (!config_.double_buffer) {
    ESP_LOGW(TAG, "clearBuffer() called but double buffering not enabled");
    return;
  }
  // TODO: Implement double buffering in platform DMA layers
  clear();
}

void HUB75Driver::flipBuffer() {
  if (!config_.double_buffer) {
    ESP_LOGW(TAG, "flipBuffer() called but double buffering not enabled");
    return;
  }
  // TODO: Implement double buffering in platform DMA layers
}

// ============================================================================
// Color Configuration
// ============================================================================

void HUB75Driver::setBrightness(uint8_t brightness) {
  config_.brightness = brightness;

  // Update basis brightness in DMA layer (platform-specific implementation)
  if (dma_) {
    dma_->setBasisBrightness(brightness);
  }
}

uint8_t HUB75Driver::getBrightness() const { return config_.brightness; }

void HUB75Driver::setBasisBrightness(uint8_t brightness) {
  if (dma_) {
    dma_->setBasisBrightness(brightness);
  }
}

void HUB75Driver::setIntensity(float intensity) {
  if (dma_) {
    dma_->setIntensity(intensity);
  }
}

void HUB75Driver::setGammaMode(hub75_gamma_mode_t mode) {
  config_.gamma_mode = mode;
  initializeLUT();  // Regenerate LUT
}

hub75_gamma_mode_t HUB75Driver::getGammaMode() const { return config_.gamma_mode; }

// ============================================================================
// Information
// ============================================================================

uint16_t HUB75Driver::getWidth() const {
  // Return total width (panel width × chain length)
  return config_.width * config_.chain_length;
}

uint16_t HUB75Driver::getHeight() const { return config_.height; }

float HUB75Driver::getRefreshRate() const {
  // TODO: Implement refresh rate tracking in platform DMA layers
  return 0.0f;
}

bool HUB75Driver::isRunning() const { return running_; }

// ============================================================================
// Private Helper Methods
// ============================================================================

void HUB75Driver::initializeLUT() {
  lut_ = get_lut(config_.gamma_mode, config_.bit_depth);
  ESP_LOGI(TAG, "Initialized %s LUT for %d-bit depth",
           config_.gamma_mode == HUB75_GAMMA_CIE1931 ? "CIE1931"
           : config_.gamma_mode == HUB75_GAMMA_2_2   ? "Gamma2.2"
                                                     : "Linear",
           config_.bit_depth);
}
