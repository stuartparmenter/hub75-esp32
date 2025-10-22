# ESP32 HUB75 DMA Driver

High-performance DMA-based driver for HUB75 RGB LED matrix panels, supporting ESP32, ESP32-S2, ESP32-S3, ESP32-C6, and ESP32-P4.

## Features

- **Continuous Automatic Refresh** - No manual `render()` calls needed, just draw and changes appear
- **Multi-Platform Support** - ESP32 (I2S), ESP32-S2 (I2S), ESP32-S3 (LCD_CAM+GDMA), ESP32-C6/P4 (PARLIO)
- **CIE 1931 Gamma Correction** - Perceptually-linear brightness with native bit-depth LUTs
- **Double Buffering** - Tear-free animation with `flipBuffer()`
- **Temporal Dithering** - Improved gradient quality (optional)
- **RGB888 and RGB565 Input** - Direct pixel formats with endianness handling
- **IRAM Optimization** - Fast pixel writes without flash cache stalls
- **Panel Chaining** - Virtual matrix support for multiple panels

## Status

✅ **Core Implementation Complete** - ESP32, ESP32-S2, and ESP32-S3 fully implemented and ready for hardware testing.

### Platform Implementation Status

- ✅ **ESP32-S3 (GDMA)** - Complete, ~2,112 static descriptors, hardware-driven refresh, ~857 lines
- ✅ **ESP32/S2 (I2S)** - Complete, ~1,040 static descriptors, hardware-driven refresh, ~871 lines
- ⏳ **ESP32-P4 (PARLIO)** - Implemented but untested (clock gating architecture, ~730 lines)
- ⏳ **ESP32-C6 (PARLIO)** - Not yet implemented

### Features Implemented

- ✅ Static circular DMA descriptor chains (no interrupts required)
- ✅ BCM timing via descriptor duplication (GDMA/I2S) or buffer padding (PARLIO)
- ✅ Dual-mode brightness control (basis 1-255 + intensity 0.0-1.0)
- ✅ CIE 1931 gamma LUTs (6/7/8/10/12-bit native)
- ✅ Multiple pixel formats (RGB888, RGB888_32, RGB565)
- ✅ Direct DMA buffer writes (no separate framebuffer copy)
- ✅ Ghosting fix via LSB bit plane previous row address
- ⏳ Temporal dithering (planned)
- ⏳ Panel chaining (planned)

## Architecture

This driver uses a **static circular DMA descriptor chain** for continuous refresh with zero CPU overhead:

```
┌─────────────────────────────────────────────────────────┐
│ Static DMA Descriptor Chain (Allocated Once at Startup)│
│                                                          │
│ Row 0, Bit 0: 1 descriptor  → buffer[row0][bit0]       │
│ Row 0, Bit 1: 1 descriptor  → buffer[row0][bit1]       │
│ ...                                                      │
│ Row 0, Bit 7: 32 descriptors → ALL point to same buffer│  ← BCM timing!
│ Row 1, Bit 0: 1 descriptor  → buffer[row1][bit0]       │
│ ...                                                      │
│ [Last descriptor] → loops back to first                 │
└─────────────────────────────────────────────────────────┘
                        ↓
         Hardware DMA engine loops forever
      (no interrupts, no CPU intervention)
```

**Key Innovation**: BCM (Binary Code Modulation) timing is achieved by **descriptor repetition**, not buffer padding. The MSB (bit 7) is displayed 32× longer than LSB (bit 0) because 32 descriptors all point to the same bit 7 buffer. The hardware automatically repeats the transmission.

**Result**: After calling `begin()`, just call `setPixel()` or `drawPixels()` - changes appear automatically on the next refresh cycle. No manual `render()` calls needed!

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
    hub75_config_t config = HUB75_CONFIG_DEFAULT();
    config.width = 64;
    config.height = 64;
    config.scan_pattern = HUB75_SCAN_1_32;

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
    HUB75Driver driver(config);
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
  - source: github://stuartparmenter/hub75-esp32
    components: [ hub75 ]
```

_Note: ESPHome component integration is planned but not yet implemented._

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
│           ├── platforms/           # Platform-specific DMA
│           │   ├── platform_dma.hpp # Abstract DMA interface
│           │   ├── i2s/             # ESP32/ESP32-S2 (~871 lines)
│           │   ├── gdma/            # ESP32-S3 (~857 lines)
│           │   └── parlio/          # ESP32-P4 (~730 lines, untested)
│           └── util/
│               └── platform_detect.cpp
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
hub75_config_t config = {
    .width = 64,                      // Panel width
    .height = 64,                     // Panel height
    .scan_pattern = HUB75_SCAN_1_32,  // Scan pattern
    .output_clock_speed = 20000000,   // 20 MHz
    .bit_depth = 8,                   // 6, 7, 8, 10, or 12
    .min_refresh_rate = 60,           // Minimum Hz
    .double_buffer = false,           // Enable double buffering
    .temporal_dither = false,         // Enable dithering
    .gamma_mode = HUB75_GAMMA_CIE1931,// Gamma correction
    .brightness = 255                 // Global brightness (0-255)
};
```

## API Reference

### Initialization
- `HUB75Driver(config)` - Create driver
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

## Development Roadmap

See [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) for detailed phases:

- **Phase 1** (2-3 weeks) - ESP32 basic implementation, CIE LUTs, 8-bit mode
- **Phase 2** (2 weeks) - ESP32-S2/S3 support, 16-bit mode, dithering, double buffering
- **Phase 3** (1-2 weeks) - ESP32-C6 PARLIO, brightness modes, panel chaining
- **Phase 4** (1 week) - ESP32-P4 PARLIO, polish, documentation

## References

This driver combines best practices from:

- [stuartparmenter/ESP32-HUB75-MatrixPanel-DMA](https://github.com/stuartparmenter/ESP32-HUB75-MatrixPanel-DMA) - Native CIE LUTs, multi-platform
- [JuPfu/hub75](https://github.com/JuPfu/hub75) - Temporal dithering, dual-mode brightness
- [liebman/esp-hub75](https://github.com/liebman/esp-hub75) - Rust patterns, IRAM optimization
- [Espressif PARLIO Example](https://github.com/espressif/esp-idf/tree/v5.4.1/examples/peripherals/parlio) - ESP32-C6/P4 PARLIO

## License

MIT License - See [LICENSE](LICENSE) file for details.

## Contributing

This project is in early development. See [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) for areas needing implementation.
