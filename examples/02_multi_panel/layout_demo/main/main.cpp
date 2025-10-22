// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT

// @file main.cpp
// @brief Two horizontal panels (2×1 layout) demonstration
//
// This example demonstrates:
// - Horizontal panel chaining (layout_cols = 2)
// - Different colors per panel
// - Boundary line to show panel seam
// - Bouncing ball across panel boundary

#include "hub75.h"
#include "board_config.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>  // NOLINT(misc-header-include-cycle)
#include <freertos/task.h>

static const char *const TAG = "two_horizontal";

extern "C" void app_main() {
  ESP_LOGI(TAG, "HUB75 Two Horizontal Panels Example Starting...");

  // Load configuration from menuconfig
  Hub75Config config = getMenuConfigSettings();

  // Verify layout configuration
  if (config.layout_cols != 2 || config.layout_rows != 1) {
    ESP_LOGW(TAG, "WARNING: Expected layout_cols=2, layout_rows=1");
    ESP_LOGW(TAG, "Current config: layout_cols=%d, layout_rows=%d", config.layout_cols, config.layout_rows);
    ESP_LOGW(TAG, "Please reconfigure via menuconfig!");
  }

  ESP_LOGI(TAG, "Configuration:");
  ESP_LOGI(TAG, "  Single panel: %dx%d pixels", config.panel_width, config.panel_height);
  ESP_LOGI(TAG, "  Layout: %dx%d panels", config.layout_cols, config.layout_rows);
  ESP_LOGI(TAG, "  Total display: %dx%d pixels", config.panel_width * config.layout_cols,
           config.panel_height * config.layout_rows);

  // Create and initialize driver
  Hub75Driver driver(config);
  if (!driver.begin()) {
    ESP_LOGE(TAG, "Failed to initialize HUB75 driver!");
    return;
  }

  ESP_LOGI(TAG, "Driver initialized successfully");

  const uint16_t width = driver.get_width();
  const uint16_t height = driver.get_height();
  const uint16_t panel_width = config.panel_width;

  ESP_LOGI(TAG, "Drawing test pattern...");

  // Clear display
  driver.clear();

  // Fill left panel (0 to panel_width-1) with RED
  ESP_LOGI(TAG, "  - Left panel (0-%d): RED", panel_width - 1);
  for (uint16_t y = 0; y < height; y++) {
    for (uint16_t x = 0; x < panel_width; x++) {
      driver.set_pixel(x, y, 255, 0, 0);
    }
  }

  // Fill right panel (panel_width to width-1) with GREEN
  ESP_LOGI(TAG, "  - Right panel (%d-%d): GREEN", panel_width, width - 1);
  for (uint16_t y = 0; y < height; y++) {
    for (uint16_t x = panel_width; x < width; x++) {
      driver.set_pixel(x, y, 0, 255, 0);
    }
  }

  // Draw vertical boundary line at panel seam (CYAN)
  ESP_LOGI(TAG, "  - Boundary line at x=%d: CYAN", panel_width);
  for (uint16_t y = 0; y < height; y++) {
    driver.set_pixel(panel_width, y, 0, 255, 255);
  }

  // Display test pattern for 3 seconds
  vTaskDelay(pdMS_TO_TICKS(3000));

  // ========================================
  // Bouncing Ball Animation
  // ========================================
  ESP_LOGI(TAG, "Starting bouncing ball animation...");

  // Ball state
  int16_t ball_x = width / 2;
  int16_t ball_y = height / 2;
  int16_t ball_dx = 2;  // Horizontal velocity
  int16_t ball_dy = 1;  // Vertical velocity
  const uint16_t ball_radius = 3;

  // Previous ball position (for clearing trail)
  int16_t prev_ball_x = ball_x;
  int16_t prev_ball_y = ball_y;

  while (true) {
    // Clear previous ball position (restore background color)
    for (int16_t dy = -ball_radius; dy <= ball_radius; dy++) {
      for (int16_t dx = -ball_radius; dx <= ball_radius; dx++) {
        int16_t px = prev_ball_x + dx;
        int16_t py = prev_ball_y + dy;

        if (px >= 0 && px < width && py >= 0 && py < height) {
          // Restore background color based on position
          if (px < panel_width) {
            driver.set_pixel(px, py, 255, 0, 0);  // Red (left panel)
          } else if (px == panel_width) {
            driver.set_pixel(px, py, 0, 255, 255);  // Cyan (boundary)
          } else {
            driver.set_pixel(px, py, 0, 255, 0);  // Green (right panel)
          }
        }
      }
    }

    // Update ball position
    ball_x += ball_dx;
    ball_y += ball_dy;

    // Bounce off edges
    if (ball_x - ball_radius <= 0 || ball_x + ball_radius >= width - 1) {
      ball_dx = -ball_dx;
      ball_x += ball_dx;  // Move back inside bounds
    }
    if (ball_y - ball_radius <= 0 || ball_y + ball_radius >= height - 1) {
      ball_dy = -ball_dy;
      ball_y += ball_dy;
    }

    // Draw ball (white)
    for (int16_t dy = -ball_radius; dy <= ball_radius; dy++) {
      for (int16_t dx = -ball_radius; dx <= ball_radius; dx++) {
        // Simple circle check
        if (dx * dx + dy * dy <= ball_radius * ball_radius) {
          int16_t px = ball_x + dx;
          int16_t py = ball_y + dy;

          if (px >= 0 && px < width && py >= 0 && py < height) {
            driver.set_pixel(px, py, 255, 255, 255);  // White
          }
        }
      }
    }

    // Remember current position for next frame
    prev_ball_x = ball_x;
    prev_ball_y = ball_y;

    // Delay for animation (~60 FPS)
    vTaskDelay(pdMS_TO_TICKS(16));
  }
}
