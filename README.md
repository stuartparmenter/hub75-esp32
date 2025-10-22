# ESP32 HUB75 DMA Driver

High-performance DMA-based driver for HUB75 RGB LED matrix panels, supporting ESP32, ESP32-S2, ESP32-S3, ESP32-C6, and ESP32-P4.

## Features

- ✅ **Static circular DMA refresh** - No interrupts, no CPU intervention after `begin()`
- ✅ **Multi-platform support** - ESP32-S3 (GDMA), ESP32/S2 (I2S), ESP32-P4 (PARLIO)
- ✅ **BCM timing** - Descriptor duplication (GDMA/I2S) or buffer padding (PARLIO)
- ✅ **Scan pattern support** - 1/4, 1/8, 1/16, 1/32 scan panels with coordinate remapping
- ✅ **Shift driver initialization** - FM6126A/ICN2038S, FM6124, MBI5124, DP3246
- ✅ **Multi-panel layouts** - Serpentine and zigzag chaining for M×N grids
  - Serpentine: Alternate rows upside down (saves cable length)
  - Zigzag: All panels upright (longer cables)
  - Row-major traversal (matches ESP32-HUB75-MatrixPanel-DMA reference)
- ✅ **CIE 1931 gamma correction** - Native bit-depth LUTs (6-12 bit)
- ✅ **Dual-mode brightness** - Basis (1-255) + intensity (0.0-1.0) control
- ✅ **Multiple pixel formats** - RGB888, RGB888_32, RGB565 input
- ✅ **Direct buffer writes** - No separate framebuffer copy, IRAM optimized
- ✅ **Double buffering** - Tear-free animation with `flipBuffer()`
- ✅ **Ghosting prevention** - LSB bit plane previous row address technique
- ✅ **PSRAM support** - ESP32-P4 PARLIO uses PSRAM for large buffers (frees internal SRAM)
- ⏳ **Temporal dithering** - Planned

## Platform Status

- ✅ **ESP32-S3 (GDMA)** - Working
- ⏳ **ESP32/S2 (I2S)** - Implemented, untested on hardware
- ✅ **ESP32-P4 (PARLIO)** - Working (uses PSRAM via EDMA)
- ⏳ **ESP32-C6 (PARLIO)** - Implemented, untested on hardware

## Installation

This is an **ESP-IDF component**. It can be used in ESP-IDF or ESPHome projects.

**For ESP-IDF projects:** Add via Component Manager in your `idf_component.yml`:

```yaml
dependencies:
  hub75:
    git: https://github.com/stuartparmenter/esp-hub75
    path: components/hub75  # Important: point to the component subdirectory!
```

See [components/hub75/README.md](components/hub75/README.md) for detailed installation instructions and API documentation.

## Quick Start

```cpp
#include "hub75.h"

void app_main() {
    // Configure for your panel
    Hub75Config config{};
    config.panel_width = 64;
    config.panel_height = 64;
    config.scan_pattern = Hub75ScanPattern::SCAN_1_32;
    config.scan_wiring = ScanPattern::STANDARD_TWO_SCAN;  // Most panels use this
    config.shift_driver = ShiftDriver::FM6126A;            // Or GENERIC if unsure

    // Set GPIO pins (see examples/common/pins_example.h for board-specific configs)
    config.pins.r1 = 25;
    config.pins.g1 = 26;
    config.pins.b1 = 27;
    config.pins.r2 = 14;
    config.pins.g2 = 12;
    config.pins.b2 = 13;
    config.pins.a = 23;
    config.pins.b = 19;
    config.pins.c = 5;
    config.pins.d = 17;
    config.pins.e = -1;
    config.pins.lat = 4;
    config.pins.oe = 15;
    config.pins.clk = 16;

    // Create and start driver
    Hub75Driver driver(config);
    driver.begin();  // Starts continuous refresh

    // Just draw - changes appear automatically!
    driver.set_pixel(10, 10, 255, 0, 0);  // Red pixel
    driver.set_pixel(20, 20, 0, 255, 0);  // Green pixel
    driver.set_pixel(30, 30, 0, 0, 255);  // Blue pixel

    // Optional: Double buffering for tear-free animation
    if (config.double_buffer) {
        driver.clear();  // Clear back buffer
        // ... draw frame ...
        driver.flip_buffer();  // Atomic swap
    }
}
```

**See [components/hub75/README.md](components/hub75/README.md) for detailed API reference and configuration options.**

## Building & Testing Standalone

This repository includes a standalone test application and examples that can be built and run directly.

### Build and Run Test Application

The root project contains a simple test application in `main/`:

```bash
cd esp-hub75
idf.py set-target esp32s3  # or esp32, esp32s2, esp32c6, esp32p4
idf.py build
idf.py flash monitor
```

The test application is based on `examples/01_simple_test` and demonstrates basic pixel drawing.

### Build and Run Examples

Each example can be built independently:

```bash
cd examples/01_simple_test
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

**Available Examples:**

- **01_simple_test** - Basic pixel drawing and color test patterns
- **02_rgb565_test** - RGB565 pixel format usage
- **03_double_buffer** - Double buffering for tear-free animation
- **04_lvgl_integration** - LVGL graphics library integration

Each example includes a `README.md` with specific instructions and pin configuration guidance.

## ESPHome Integration

```yaml
external_components:
  - source: github://stuartparmenter/hub75-esphome
    components: [ hub75_display ]

display:
  - platform: hub75_display
    id: my_display
    panel_width: 64
    panel_height: 64
    scan_pattern: SCAN_1_32
    shift_driver: FM6126A  # Optional: for panels that need special init

    # Multi-panel layout (optional)
    layout_rows: 2
    layout_cols: 2
    layout: TOP_LEFT_DOWN  # Serpentine wiring

    # Pin configuration
    pin_r1: 25
    pin_g1: 26
    pin_b1: 27
    pin_r2: 14
    pin_g2: 12
    pin_b2: 13
    pin_a: 23
    pin_b: 19
    pin_c: 5
    pin_d: 17
    pin_e: -1
    pin_lat: 4
    pin_oe: 15
    pin_clk: 16
```

See [hub75-esphome](https://github.com/stuartparmenter/hub75-esphome) repository for the full ESPHome component documentation.

## Project Structure

This repository is structured as an **ESP-IDF component** with a standalone test application:

```
esp-hub75/                         # Repository root
├── CMakeLists.txt                   # Root project (for standalone idf.py build)
├── main/                            # Test application (for standalone build)
│   ├── CMakeLists.txt
│   └── main.cpp                     # Simple test example
├── components/
│   └── hub75/                       # ← THE COMPONENT (point here when including!)
│       ├── CMakeLists.txt           # Component build rules
│       ├── idf_component.yml        # Component manifest
│       ├── include/                 # Public API headers
│       │   ├── hub75.h              # Main API
│       │   ├── hub75_types.h        # Common types
│       │   └── hub75_config.h       # Compile-time config
│       └── src/                     # Implementation
│           ├── core/                # Core driver
│           │   └── hub75_driver.cpp # Main driver coordinator (~200 lines)
│           ├── color/               # Color conversion
│           │   ├── color_lut.cpp    # CIE 1931 gamma LUTs
│           │   └── color_convert.cpp # RGB888/RGB565 conversion
│           ├── panels/              # Panel compatibility
│           │   ├── scan_patterns.h # Non-standard scan coordinate remapping
│           │   └── panel_layout.h  # Multi-panel chaining (serpentine/zigzag)
│           └── platforms/           # Platform-specific DMA
│               ├── platform_dma.h # Abstract DMA interface
│               ├── platform_detect.cpp # Platform detection
│               ├── i2s/             # ESP32/ESP32-S2 (~871 lines)
│               │   ├── i2s_dma.cpp
│               │   └── i2s_dma.h
│               ├── gdma/            # ESP32-S3 (~857 lines)
│               │   ├── gdma_dma.cpp
│               │   └── gdma_dma.h
│               └── parlio/          # ESP32-P4 (~730 lines, untested)
│                   ├── parlio_dma.cpp
│                   └── parlio_dma.h
└── examples/                        # Additional examples
    ├── common/
    │   └── pins_example.h           # Pin configuration examples
    ├── 01_simple_test/
    ├── 02_rgb565_test/
    ├── 03_double_buffer/
    └── 04_lvgl_integration/
```

**Important**: When including this component in your project, point to `components/hub75/` subdirectory, not the repository root.

## Configuration & API

The driver is highly configurable with support for:

- **Hardware specs:** Panel dimensions, scan patterns, shift driver chips
- **Multi-panel layouts:** Serpentine and zigzag chaining for M×N grids
- **Performance tuning:** Clock speed, bit depth, refresh rate
- **Color control:** Gamma correction (CIE 1931, Gamma 2.2), brightness, intensity
- **Advanced features:** Double buffering, temporal dithering (planned)

**Pin configuration examples:** See [examples/common/pins_example.h](examples/common/pins_example.h) for board-specific GPIO layouts.

**Full documentation:** See [components/hub75/README.md](components/hub75/README.md) for complete API reference, configuration options, multi-panel layouts, and troubleshooting guidance.

## References

This driver combines best practices from:

- [mrcodetastic/ESP32-HUB75-MatrixPanel-DMA](https://github.com/mrcodetastic/ESP32-HUB75-MatrixPanel-DMA) - Native CIE LUTs, GDMA, multi-platform
- [JuPfu/hub75](https://github.com/JuPfu/hub75) - Temporal dithering, dual-mode brightness
- [liebman/esp-hub75](https://github.com/liebman/esp-hub75) - Rust patterns, IRAM optimization
- [Espressif PARLIO Example](https://github.com/espressif/esp-idf/tree/v5.4.1/examples/peripherals/parlio) - ESP32-C6/P4 PARLIO

## License

MIT License - See [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please open an issue or pull request for bugs, features, or improvements.
