// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file driver_init.h
// @brief Shift driver chip initialization

#pragma once

#include "hub75_types.h"
#include "esp_err.h"

namespace hub75 {

// Shift driver chip initialization via GPIO bit-banging
// Called ONCE before DMA chain starts
class DriverInit {
 public:
  // Initialize shift driver chip
  // Initialize shift driver chip
  // @param config Panel configuration
  // @return ESP_OK on success
  static esp_err_t initialize(const Hub75Config &config);

 private:
  static void fm6126a_init(const Hub75Pins &pins, uint16_t pixels_per_row);
  static void dp3246_init(const Hub75Pins &pins, uint16_t pixels_per_row);
};

}  // namespace hub75
