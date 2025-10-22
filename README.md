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
- ⏳ **Temporal dithering** - Planned
- ⏳ **ESP32-C6 PARLIO** - Planned

## Platform Status

- ✅ **ESP32-S3 (GDMA)** - Implemented and tested
- ⏳ **ESP32/S2 (I2S)** - Implemented but untested
- ⏳ **ESP32-P4 (PARLIO)** - Implemented but untested
- ⏳ **ESP32-C6 (PARLIO)** - Not yet implemented

## Installation

This is an **ESP-IDF component**. It can be used in ESP-IDF or ESPHome projects.

### Option 1: Add via Component Manager (idf_component.yml)

In your project's `idf_component.yml`:

```yaml
dependencies:
  hub75:
    git: https://github.com/stuartparmenter/hub75-esp32
    path: components/hub75  # Important: point to the component subdirectory!
```

Or for local development:

```yaml
dependencies:
  hub75:
    path: /path/to/hub75-esp32/components/hub75  # Point to components/hub75 subdirectory
```

### Option 2: Manual Copy

Copy the `components/hub75/` directory to your project's `components/` directory:

```bash
cp -r /path/to/hub75-esp32/components/hub75 my_project/components/
```

## Quick Start

### ESP-IDF

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

    // Set GPIO pins
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
    driver.setPixel(10, 10, 255, 0, 0);  // Red pixel
    driver.setPixel(20, 20, 0, 255, 0);  // Green pixel
    driver.setPixel(30, 30, 0, 0, 255);  // Blue pixel

    // Optional: Double buffering for tear-free animation
    if (config.double_buffer) {
        driver.clearBuffer();
        // ... draw frame ...
        driver.flipBuffer();  // Atomic swap
    }
}
```

See [examples/common/pins_example.h](examples/common/pins_example.h) for pre-configured pin layouts for different boards.

### ESPHome

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

See [hub75-esphome](https://github.com/stuartparmenter/hub75-esphome) repository for the ESPHome component.

## Project Structure

This repository is structured as an **ESP-IDF component** with a standalone test application:

```
hub75-esp32/                         # Repository root
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

## Pin Configuration

See [examples/common/pins_example.h](examples/common/pins_example.h) for detailed pin configuration examples for different boards and panel sizes.

**Important:** GPIO availability varies by ESP32 variant - check the examples for platform-specific restrictions.

## Configuration Options

```cpp
Hub75Config config{};  // Start with defaults

// Single Panel Hardware Specifications
config.panel_width = 64;                                    // Single panel width in pixels
config.panel_height = 64;                                   // Single panel height in pixels
config.scan_pattern = Hub75ScanPattern::SCAN_1_32;          // Hardware scan pattern (SCAN_1_2, SCAN_1_4, SCAN_1_8, SCAN_1_16, SCAN_1_32)
config.scan_wiring = ScanPattern::STANDARD_TWO_SCAN;        // Coordinate remapping (STANDARD_TWO_SCAN, FOUR_SCAN_16PX_HIGH, etc.)
config.shift_driver = ShiftDriver::GENERIC;                 // Driver chip (GENERIC, FM6126A, ICN2038S, FM6124, MBI5124, DP3246)

// Multi-Panel Physical Layout (optional)
config.layout_rows = 1;                                     // Number of panels vertically (default: 1)
config.layout_cols = 1;                                     // Number of panels horizontally (default: 1)
config.layout = PanelLayout::HORIZONTAL;                    // Chaining pattern:
                                                            // - HORIZONTAL: Simple left-to-right (single row)
                                                            // - TOP_LEFT_DOWN: Serpentine, start top-left
                                                            // - TOP_RIGHT_DOWN: Serpentine, start top-right
                                                            // - BOTTOM_LEFT_UP: Serpentine, start bottom-left
                                                            // - BOTTOM_RIGHT_UP: Serpentine, start bottom-right
                                                            // - *_ZIGZAG variants: All panels upright (no Y-inversion)

// Pin configuration (see pins{} struct - documented in Quick Start section)

// Performance
config.output_clock_speed = Hub75ClockSpeed::HZ_20M;        // Clock: HZ_8M, HZ_10M, HZ_16M, HZ_20M
config.bit_depth = 8;                                       // BCM bit depth: 6-12
config.min_refresh_rate = 60;                               // Minimum refresh rate in Hz

// Timing
config.latch_blanking = 1;                                  // OE blanking cycles during LAT pulse
config.clk_phase_inverted = false;                          // Invert clock phase (required for MBI5124)

// Features
config.double_buffer = false;                               // Enable double buffering
config.temporal_dither = false;                             // Enable temporal dithering (not yet implemented)

// Color
config.gamma_mode = Hub75GammaMode::CIE1931;                // NONE, CIE1931, GAMMA_2_2
config.brightness = 128;                                    // Initial brightness (0-255, default: 128)
```

### Multi-Panel Layout Examples

**2×2 Serpentine Grid** (128×128, panels alternate upside down):
```cpp
config.panel_width = 64;
config.panel_height = 64;
config.layout_rows = 2;
config.layout_cols = 2;
config.layout = PanelLayout::TOP_LEFT_DOWN;  // Serpentine
```

**4×1 Horizontal Chain** (256×64, all panels upright):
```cpp
config.panel_width = 64;
config.panel_height = 64;
config.layout_rows = 1;
config.layout_cols = 4;
config.layout = PanelLayout::HORIZONTAL;
```

**3×2 Zigzag Grid** (192×128, all panels upright):
```cpp
config.panel_width = 64;
config.panel_height = 64;
config.layout_rows = 2;
config.layout_cols = 3;
config.layout = PanelLayout::TOP_LEFT_DOWN_ZIGZAG;  // All upright
```

## API Reference

### Initialization
- `Hub75Driver(config)` - Create driver
- `bool begin()` - Initialize and start refresh
- `void end()` - Stop and cleanup

### Drawing
- `void setPixel(x, y, r, g, b)` - Draw pixel (RGB888)
- `void setPixelRGB565(x, y, rgb565)` - Draw pixel (RGB565)
- `void setPixelRaw(x, y, packed)` - Draw pixel (native format)
- `void clear()` - Clear display

**Note:** Graphics primitives like rectangles, lines, and circles should be implemented using a graphics library (LVGL, Adafruit_GFX, etc.) or manually with `setPixel` loops.

### Double Buffering
- `void clearBuffer()` - Clear back buffer
- `void flipBuffer()` - Swap buffers atomically

### Configuration
- `void setBrightness(0-255)` - Set brightness
- `void setGammaMode(mode)` - Set gamma mode

### Information
- `uint16_t getWidth()` - Get panel width
- `uint16_t getHeight()` - Get panel height
- `float getRefreshRate()` - Get actual refresh rate
- `bool isRunning()` - Check if running

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
