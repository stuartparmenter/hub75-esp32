// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file main.cpp
// @brief Brightness control demonstration
//
// This example demonstrates:
// - Runtime brightness adjustment via set_brightness()
// - Fade in/out animations
// - Continuous pulse (breathing) effect
// - Static pattern with dynamic brightness

#include "hub75.h"
#include "board_config.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>  // NOLINT(misc-header-include-cycle)
#include <freertos/task.h>

static const char *const TAG = "brightness";

// Draw colored horizontal bars across the display
static void draw_test_pattern(Hub75Driver &driver) {
  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();

  // Colors for bars (R, G, B)
  const struct {
    uint8_t r, g, b;
  } colors[] = {
      {255, 0, 0},     // Red
      {0, 255, 0},     // Green
      {0, 0, 255},     // Blue
      {255, 255, 0},   // Yellow
      {0, 255, 255},   // Cyan
      {255, 0, 255},   // Magenta
      {255, 255, 255}  // White
  };

  const uint16_t num_colors = sizeof(colors) / sizeof(colors[0]);
  const uint16_t bar_height = height / num_colors;

  for (uint16_t i = 0; i < num_colors; i++) {
    uint16_t y_start = i * bar_height;
    uint16_t y_end = (i + 1) * bar_height;
    if (i == num_colors - 1) {
      y_end = height;  // Last bar fills remaining height
    }

    for (uint16_t y = y_start; y < y_end; y++) {
      for (uint16_t x = 0; x < width; x++) {
        driver.set_pixel(x, y, colors[i].r, colors[i].g, colors[i].b);
      }
    }
  }
}

extern "C" void app_main() {
  ESP_LOGI(TAG, "HUB75 Brightness Control Example Starting...");

  // Load configuration from menuconfig
  Hub75Config config = getMenuConfigSettings();

  ESP_LOGI(TAG, "Configuration:");
  ESP_LOGI(TAG, "  Panel: %dx%d pixels", config.panel_width, config.panel_height);

  // Create and initialize driver
  Hub75Driver driver(config);
  if (!driver.begin()) {
    ESP_LOGE(TAG, "Failed to initialize HUB75 driver!");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");

  // Draw test pattern (colored bars)
  ESP_LOGI(TAG, "Drawing test pattern (colored bars)...");
  driver.clear();
  draw_test_pattern(driver);
  ESP_LOGI(TAG, "Test pattern complete");

  // Start at full brightness
  driver.set_brightness(255);
  vTaskDelay(pdMS_TO_TICKS(2000));  // Display pattern at full brightness

  // ========================================
  // Phase 1: Fade IN (0 → 255)
  // ========================================
  ESP_LOGI(TAG, "Phase 1: Fade IN (0 → 255 over 5 seconds)");

  for (uint16_t brightness = 0; brightness <= 255; brightness++) {
    driver.set_brightness(brightness);
    vTaskDelay(pdMS_TO_TICKS(20));  // 5000ms / 255 steps ≈ 20ms per step
  }

  ESP_LOGI(TAG, "Fade in complete");
  vTaskDelay(pdMS_TO_TICKS(1000));

  // ========================================
  // Phase 2: Fade OUT (255 → 0)
  // ========================================
  ESP_LOGI(TAG, "Phase 2: Fade OUT (255 → 0 over 5 seconds)");

  for (uint16_t brightness = 255; brightness > 0; brightness--) {
    driver.set_brightness(brightness);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  driver.set_brightness(0);  // Ensure fully off

  ESP_LOGI(TAG, "Fade out complete");
  vTaskDelay(pdMS_TO_TICKS(1000));

  // ========================================
  // Phase 3: Pulse Animation (Continuous)
  // ========================================
  ESP_LOGI(TAG, "Phase 3: Pulse animation (continuous breathing effect)");
  ESP_LOGI(TAG, "Pulsing indefinitely...");

  while (true) {
    // Fade in (0 → 255)
    for (uint16_t brightness = 0; brightness <= 255; brightness++) {
      driver.set_brightness(brightness);
      vTaskDelay(pdMS_TO_TICKS(10));  // Faster pulse: 2.5s in
    }

    // Fade out (255 → 0)
    for (uint16_t brightness = 255; brightness > 0; brightness--) {
      driver.set_brightness(brightness);
      vTaskDelay(pdMS_TO_TICKS(10));  // 2.5s out
    }
    driver.set_brightness(0);

    // Brief pause at minimum brightness
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}
