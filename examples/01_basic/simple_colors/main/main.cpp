// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file main.cpp
// @brief Simple HUB75 test - colored squares and center cross
//
// This example demonstrates:
// - Basic driver initialization from menuconfig
// - Simple pixel drawing using set_pixel()
// - Clearing the display
// - Adapting to different panel sizes

#include "hub75.h"
#include "board_config.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>  // NOLINT(misc-header-include-cycle)
#include <freertos/task.h>

static const char *const TAG = "simple_colors";

extern "C" void app_main() {
  ESP_LOGI(TAG, "HUB75 Simple Colors Example Starting...");

  // Load configuration from menuconfig
  Hub75Config config = getMenuConfigSettings();

  // Print configuration summary
  ESP_LOGI(TAG, "Configuration:");
  ESP_LOGI(TAG, "  Panel: %dx%d pixels (%d-bit, %d Hz min refresh)", config.panel_width, config.panel_height,
           config.bit_depth, config.min_refresh_rate);
  ESP_LOGI(TAG, "  Layout: %dx%d panels (total %dx%d display)", config.layout_cols, config.layout_rows,
           config.panel_width * config.layout_cols, config.panel_height * config.layout_rows);

  // Print pin configuration
  printPinConfig(config.pins);

  // Create driver instance
  Hub75Driver driver(config);

  // Initialize and start continuous refresh
  if (!driver.begin()) {
    ESP_LOGE(TAG, "Failed to initialize HUB75 driver!");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");
  ESP_LOGI(TAG, "Display: %dx%d pixels", driver.get_width(), driver.get_height());

  // Clear display
  driver.clear();

  // Draw colored squares in corners (using set_pixel loops)
  const uint16_t square_size = 8;

  // Top-left: Red
  for (uint16_t y = 0; y < square_size; y++) {
    for (uint16_t x = 0; x < square_size; x++) {
      driver.set_pixel(x, y, 255, 0, 0);
    }
  }

  // Top-right: Green
  for (uint16_t y = 0; y < square_size; y++) {
    for (uint16_t x = driver.get_width() - square_size; x < driver.get_width(); x++) {
      driver.set_pixel(x, y, 0, 255, 0);
    }
  }

  // Bottom-left: Blue
  for (uint16_t y = driver.get_height() - square_size; y < driver.get_height(); y++) {
    for (uint16_t x = 0; x < square_size; x++) {
      driver.set_pixel(x, y, 0, 0, 255);
    }
  }

  // Bottom-right: White
  for (uint16_t y = driver.get_height() - square_size; y < driver.get_height(); y++) {
    for (uint16_t x = driver.get_width() - square_size; x < driver.get_width(); x++) {
      driver.set_pixel(x, y, 255, 255, 255);
    }
  }

  // Draw center cross in cyan
  uint16_t cx = driver.get_width() / 2;
  uint16_t cy = driver.get_height() / 2;

  // Horizontal line
  for (uint16_t x = cx - 10; x <= cx + 10; x++) {
    driver.set_pixel(x, cy, 0, 255, 255);
  }

  // Vertical line
  for (uint16_t y = cy - 10; y <= cy + 10; y++) {
    driver.set_pixel(cx, y, 0, 255, 255);
  }

  ESP_LOGI(TAG, "Test pattern displayed");
  ESP_LOGI(TAG, "Expected output:");
  ESP_LOGI(TAG, "  - Red square in top-left corner");
  ESP_LOGI(TAG, "  - Green square in top-right corner");
  ESP_LOGI(TAG, "  - Blue square in bottom-left corner");
  ESP_LOGI(TAG, "  - White square in bottom-right corner");
  ESP_LOGI(TAG, "  - Cyan cross in center");

  // Idle loop (display continues refresh via DMA)
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
