/**
 * SPDX-FileCopyrightText: 2025 Stuart Parmenter
 * SPDX-License-Identifier: MIT
 *
 * @file main.cpp
 * @brief Simple HUB75 test - colored squares and pixel test
 *
 * This example demonstrates:
 * - Basic driver initialization
 * - Pin configuration
 * - Simple pixel drawing
 * - Fill rectangle
 * - Color testing
 */

#include "hub75.h"
#include "../common/pins_example.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "simple_test";

extern "C" void app_main() {
  ESP_LOGI(TAG, "HUB75 Simple Test Starting...");

  // Get default pin configuration for ESP32
  // Modify pins_example.h for your specific board
  hub75_config_t config = getDefaultConfig_ESP32();

  // Print pin configuration for verification
  printPinConfig(config.pins);

  // Create driver instance
  HUB75Driver driver(config);

  // Initialize and start continuous refresh
  if (!driver.begin()) {
    ESP_LOGE(TAG, "Failed to initialize HUB75 driver!");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");
  ESP_LOGI(TAG, "Panel: %dx%d pixels", driver.getWidth(), driver.getHeight());

  // Clear display
  driver.clear();

  // Draw colored squares in corners (using setPixel loops)
  const uint16_t square_size = 8;

  // Top-left: Red
  for (uint16_t y = 0; y < square_size; y++) {
    for (uint16_t x = 0; x < square_size; x++) {
      driver.setPixel(x, y, 255, 0, 0);
    }
  }

  // Top-right: Green
  for (uint16_t y = 0; y < square_size; y++) {
    for (uint16_t x = driver.getWidth() - square_size; x < driver.getWidth(); x++) {
      driver.setPixel(x, y, 0, 255, 0);
    }
  }

  // Bottom-left: Blue
  for (uint16_t y = driver.getHeight() - square_size; y < driver.getHeight(); y++) {
    for (uint16_t x = 0; x < square_size; x++) {
      driver.setPixel(x, y, 0, 0, 255);
    }
  }

  // Bottom-right: White
  for (uint16_t y = driver.getHeight() - square_size; y < driver.getHeight(); y++) {
    for (uint16_t x = driver.getWidth() - square_size; x < driver.getWidth(); x++) {
      driver.setPixel(x, y, 255, 255, 255);
    }
  }

  // Draw center cross in cyan
  uint16_t cx = driver.getWidth() / 2;
  uint16_t cy = driver.getHeight() / 2;

  // Horizontal line
  for (uint16_t x = cx - 10; x <= cx + 10; x++) {
    driver.setPixel(x, cy, 0, 255, 255);
  }

  // Vertical line
  for (uint16_t y = cy - 10; y <= cy + 10; y++) {
    driver.setPixel(cx, y, 0, 255, 255);
  }

  ESP_LOGI(TAG, "Static test pattern displayed");
  ESP_LOGI(TAG, "You should see:");
  ESP_LOGI(TAG, "  - Red square in top-left");
  ESP_LOGI(TAG, "  - Green square in top-right");
  ESP_LOGI(TAG, "  - Blue square in bottom-left");
  ESP_LOGI(TAG, "  - White square in bottom-right");
  ESP_LOGI(TAG, "  - Cyan cross in center");

  // Animate some random pixels
  ESP_LOGI(TAG, "Starting animation...");

  uint32_t frame = 0;
  while (true) {
    // Draw a moving pixel
    uint16_t x = (frame % driver.getWidth());
    uint16_t y = (frame / driver.getWidth()) % driver.getHeight();

    // Rainbow color based on position
    uint8_t r = (x * 255) / driver.getWidth();
    uint8_t g = (y * 255) / driver.getHeight();
    uint8_t b = ((x + y) * 255) / (driver.getWidth() + driver.getHeight());

    driver.setPixel(x, y, r, g, b);

    frame++;

    // Print refresh rate every 60 frames
    if (frame % 60 == 0) {
      float refresh = driver.getRefreshRate();
      ESP_LOGI(TAG, "Frame %lu, Refresh rate: %.2f Hz", frame, refresh);
    }

    vTaskDelay(pdMS_TO_TICKS(10));  // 100 FPS draw rate
  }
}
