// SPDX-FileCopyrightText: 2025 Stuart Parmenter
// SPDX-License-Identifier: MIT
//
// @file platform_dma.cpp
// @brief Platform-agnostic DMA interface implementation

#include "platform_dma.h"
#include "../color/color_lut.h"  // For get_lut()
#include <esp_log.h>

static const char *const TAG = "PlatformDma";

namespace hub75 {

PlatformDma::PlatformDma(const Hub75Config &config)
    : config_(config), lut_(get_lut(config.gamma_mode, config.bit_depth)) {
  ESP_LOGI(TAG, "Initialized %s LUT for %d-bit depth",
           config.gamma_mode == Hub75GammaMode::CIE1931     ? "CIE1931"
           : config.gamma_mode == Hub75GammaMode::GAMMA_2_2 ? "Gamma2.2"
                                                            : "Linear",
           config.bit_depth);
}

}  // namespace hub75
