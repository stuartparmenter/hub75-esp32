# HUB75 DMA Driver Component

ESP-IDF component for driving HUB75 RGB LED matrix panels via DMA. Supports ESP32, ESP32-S2, ESP32-S3, ESP32-C6, and ESP32-P4.

**Repository:** https://github.com/stuartparmenter/esp-hub75
**Examples & Documentation:** See main repository for working examples and detailed documentation

## Key Features

- Static circular DMA refresh (no CPU intervention after initialization)
- Multi-platform support: GDMA (S3), I2S (ESP32/S2), PARLIO (P4/C6)
- CIE 1931 gamma correction with native bit-depth LUTs (6-12 bit)
- Multi-panel layouts with serpentine and zigzag chaining
- Double buffering for tear-free animation
- Multiple pixel formats: RGB888, RGB888_32, RGB565

## Installation

### Option 1: Component Manager (Recommended)

In your project's `idf_component.yml`:

```yaml
dependencies:
  hub75:
    git: https://github.com/stuartparmenter/esp-hub75
    path: components/hub75  # Important: point to subdirectory!
```

For local development:

```yaml
dependencies:
  hub75:
    path: /path/to/esp-hub75/components/hub75
```

### Option 2: Manual Copy

Copy the component directory to your project:

```bash
cp -r /path/to/esp-hub75/components/hub75 my_project/components/
```

**Important:** Always point to `components/hub75/` subdirectory, not the repository root. The root contains a standalone test project that will conflict if included as a component.

## Quick Start

```cpp
#include "hub75.h"

void app_main() {
    // Configure for your panel
    Hub75Config config{};
    config.panel_width = 64;
    config.panel_height = 64;
    config.scan_pattern = Hub75ScanPattern::SCAN_1_32;
    config.scan_wiring = ScanPattern::STANDARD_TWO_SCAN;  // Most panels
    config.shift_driver = ShiftDriver::FM6126A;            // Or GENERIC

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

    // Draw pixels - changes appear automatically!
    driver.set_pixel(10, 10, 255, 0, 0);  // Red
    driver.set_pixel(20, 20, 0, 255, 0);  // Green
    driver.set_pixel(30, 30, 0, 0, 255);  // Blue

    // Optional: Double buffering for tear-free animation
    if (config.double_buffer) {
        driver.clear();  // Clear back buffer
        // ... draw frame ...
        driver.flip_buffer();  // Atomic swap
    }
}
```

**Pin Configuration:** See repository [examples/common/pins_example.h](https://github.com/stuartparmenter/esp-hub75/blob/main/examples/common/pins_example.h) for board-specific pre-configured pin layouts.

## API Reference

### Initialization

- `Hub75Driver(config)` - Create driver with configuration
- `bool begin()` - Initialize hardware and start continuous refresh loop
- `void end()` - Stop refresh and cleanup resources

### Drawing

- `void draw_pixels(x, y, w, h, buffer, format, color_order, big_endian)` - Bulk pixel write (most efficient)
  - `format`: `PixelFormat::RGB888` (24-bit packed), `RGB888_32` (32-bit padded), or `RGB565` (16-bit)
  - `color_order`: `ColorOrder::RGB` or `BGR` (for RGB888_32 only)
  - `big_endian`: Byte order control (affects RGB565 and RGB888_32)
- `void set_pixel(x, y, r, g, b)` - Draw single pixel with RGB888 values (0-255)
- `void clear()` - Clear entire display to black

**Note:** For graphics primitives (rectangles, lines, circles), use a graphics library like LVGL or Adafruit_GFX, or implement manually with `set_pixel()` loops.

### Double Buffering

- `void flip_buffer()` - Swap front and back buffers atomically

When double buffering is enabled, drawing operations (`clear()`, `set_pixel()`, `draw_pixels()`) operate on the back buffer. Call `flip_buffer()` to atomically swap buffers and display the new frame.

**Memory Usage:**
- **GDMA/I2S** (internal SRAM): ~57 KB single-buffer, ~114 KB double-buffer (64×64 panel, 8-bit)
- **PARLIO** (PSRAM): ~284 KB single-buffer, ~568 KB double-buffer (64×64 panel, 8-bit)

Double buffering doubles memory usage but enables tear-free animation. PARLIO uses ~5× more memory than GDMA/I2S, but allocates from PSRAM (typically 8-16 MB available) rather than scarce internal SRAM (~500 KB total). Larger panels scale linearly: 128×128 uses ~4× memory (PARLIO: ~1.1 MB, GDMA: ~228 KB).

### Brightness & Color Control

- `void set_brightness(uint8_t brightness)` - Set display brightness (0-255)
- `void set_intensity(float intensity)` - Set intensity multiplier (0.0-1.0) for smooth dimming
- `uint8_t get_brightness()` - Get current brightness value
- `void set_gamma_mode(Hub75GammaMode mode)` - Set gamma correction mode
  - `Hub75GammaMode::NONE` - No gamma correction (linear)
  - `Hub75GammaMode::CIE1931` - CIE 1931 standard (recommended)
  - `Hub75GammaMode::GAMMA_2_2` - Gamma 2.2 correction
- `Hub75GammaMode get_gamma_mode()` - Get current gamma mode

**Dual-Mode Brightness System:**
- **Basis brightness** (0-255): Adjusts hardware OE (output enable) timing in DMA buffers
- **Intensity** (0.0-1.0): Runtime scaling multiplier for smooth dimming without refresh rate changes
- Final brightness = (basis × intensity) >> 8

### Information

- `uint16_t get_width()` - Get display width in pixels (panel_width × layout_cols)
- `uint16_t get_height()` - Get display height in pixels (panel_height × layout_rows)
- `bool is_running()` - Check if refresh loop is active

## Configuration Options

### Hardware Specifications

```cpp
Hub75Config config{};  // Start with defaults

// Single Panel Hardware
config.panel_width = 64;                               // Single panel width in pixels
config.panel_height = 64;                              // Single panel height in pixels
config.scan_pattern = Hub75ScanPattern::SCAN_1_32;     // Hardware scan pattern
config.scan_wiring = ScanPattern::STANDARD_TWO_SCAN;   // Coordinate remapping
config.shift_driver = ShiftDriver::GENERIC;            // LED driver chip type
```

**Scan Pattern Options:**
- `SCAN_1_2` - 2-row pairs (4px high panels)
- `SCAN_1_4` - 4-row pairs (8px high panels)
- `SCAN_1_8` - 8-row pairs (16px high panels)
- `SCAN_1_16` - 16-row pairs (32px high panels)
- `SCAN_1_32` - 32-row pairs (64px high panels)

Must match panel hardware. Formula: `num_rows = height / scan_pattern_value`

**Scan Wiring Options:**
- `STANDARD_TWO_SCAN` - Most panels (default, no coordinate remapping)
- `FOUR_SCAN_16PX_HIGH` - Four-scan 1/4 scan, 16-pixel high panels
- `FOUR_SCAN_32PX_HIGH` - Four-scan 1/8 scan, 32-pixel high panels
- `FOUR_SCAN_64PX_HIGH` - Four-scan 1/8 scan, 64-pixel high panels

For panels with non-standard internal wiring that require coordinate remapping.

**Shift Driver Options:**
- `GENERIC` - Standard panels with no special initialization (default)
- `FM6126A` - Very common in modern panels (also works for ICN2038S)
- `ICN2038S` - Alias for FM6126A (same initialization sequence)
- `FM6124` - FM6124 family panels
- `MBI5124` - MBI5124 panels (requires `clk_phase_inverted = true`)
- `DP3246` - DP3246 panels (special timing requirements)

**Tip:** If panel shows incorrect colors or doesn't light up with `GENERIC`, try `FM6126A` first - it's the most common driver chip in modern panels.

### Multi-Panel Physical Layout

```cpp
// Multi-panel configuration (optional, defaults to single panel)
config.layout_rows = 1;                                // Number of panels vertically
config.layout_cols = 1;                                // Number of panels horizontally
config.layout = PanelLayout::HORIZONTAL;               // Chaining pattern
```

**Layout Options:**
- `HORIZONTAL` - Simple left-to-right chain (single row only)
- `TOP_LEFT_DOWN` - Serpentine, start top-left corner
- `TOP_RIGHT_DOWN` - Serpentine, start top-right corner
- `BOTTOM_LEFT_UP` - Serpentine, start bottom-left corner
- `BOTTOM_RIGHT_UP` - Serpentine, start bottom-right corner
- `TOP_LEFT_DOWN_ZIGZAG` - Zigzag, start top-left (all panels upright)
- `TOP_RIGHT_DOWN_ZIGZAG` - Zigzag, start top-right (all panels upright)
- `BOTTOM_LEFT_UP_ZIGZAG` - Zigzag, start bottom-left (all panels upright)
- `BOTTOM_RIGHT_UP_ZIGZAG` - Zigzag, start bottom-right (all panels upright)

**Serpentine vs Zigzag:**
- **Serpentine**: Alternate rows are physically mounted upside down (saves cable length)
- **Zigzag**: All panels mounted upright, cables route back between rows (longer cables)

**Row-Major Chaining:** Panels chain HORIZONTALLY across rows (not vertically down columns). This matches the ESP32-HUB75-MatrixPanel-DMA reference library.

### Performance & Timing

```cpp
// Performance
config.output_clock_speed = Hub75ClockSpeed::HZ_20M;   // Clock speed
config.bit_depth = 8;                                   // BCM bit depth: 6-12
config.min_refresh_rate = 60;                           // Minimum refresh rate in Hz

// Timing
config.latch_blanking = 1;                              // OE blanking cycles during LAT
config.clk_phase_inverted = false;                      // Invert clock phase (MBI5124)

// Features
config.double_buffer = false;                           // Enable double buffering
config.temporal_dither = false;                         // Enable temporal dithering (NYI)

// Color
config.gamma_mode = Hub75GammaMode::CIE1931;            // Gamma correction mode
config.brightness = 128;                                // Initial brightness (0-255)
```

**Clock Speed Options:**
- `HZ_8M` - 8 MHz (most compatible)
- `HZ_10M` - 10 MHz
- `HZ_16M` - 16 MHz
- `HZ_20M` - 20 MHz (default, works with most panels)

Higher speeds may cause signal integrity issues with long cables or poor-quality panels.

**Bit Depth:**
- 6-bit: Fast refresh, basic color depth
- 8-bit: Good balance (default)
- 10-bit: Better gradients, recommended for smooth animations
- 12-bit: Best quality, slower refresh

Higher bit depth = more descriptors + slower refresh rate.

### Pin Configuration

```cpp
// GPIO pin assignments (example for ESP32-S3)
config.pins.r1 = 25;   // Red data (top half)
config.pins.g1 = 26;   // Green data (top half)
config.pins.b1 = 27;   // Blue data (top half)
config.pins.r2 = 14;   // Red data (bottom half)
config.pins.g2 = 12;   // Green data (bottom half)
config.pins.b2 = 13;   // Blue data (bottom half)
config.pins.a = 23;    // Row address bit A
config.pins.b = 19;    // Row address bit B
config.pins.c = 5;     // Row address bit C
config.pins.d = 17;    // Row address bit D
config.pins.e = -1;    // Row address bit E (-1 if unused)
config.pins.lat = 4;   // Latch
config.pins.oe = 15;   // Output enable
config.pins.clk = 16;  // Clock
```

**GPIO Restrictions:**
- Avoid strapping pins (GPIO0, GPIO46, etc.)
- ESP32-S3: GPIO19/20 unavailable if USB CDC enabled
- Check ESP32 variant datasheets for input-only pins
- Some platforms have restrictions on which peripherals can use which pins

**Pre-configured Examples:** See repository [examples/common/pins_example.h](https://github.com/stuartparmenter/esp-hub75/blob/main/examples/common/pins_example.h) for tested pin configurations for different boards.

## Multi-Panel Layouts

### Layout Examples

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

### Physical Wiring

**Serpentine Example** (TOP_LEFT_DOWN):
```
Virtual Display:        Physical Chain:         Panel Orientation:
┌────┬────┐             ┌───┬───┬───┬───┐      ┌────┬────┐
│ 0  │ 1  │             │ 0 │ 1 │ 2 │ 3 │      │ 0→ │ ←1 │  (Panel 1 upside down)
├────┼────┤             └───┴───┴───┴───┘      ├────┼────┤
│ 2  │ 3  │                                     │ 2→ │ ←3 │  (Panel 3 upside down)
└────┴────┘                                     └────┴────┘
```

Panel chain is always horizontal in DMA buffer: `[Panel 0][Panel 1][Panel 2][Panel 3]`

**Coordinate Transformation Pipeline:**
Virtual coordinates → Panel layout remap → Scan pattern remap → Physical DMA buffer

## Platform-Specific Notes

### ESP32-S3 (GDMA)

- **Status:** ✅ Working
- **Memory:** Internal SRAM allocation
- **BCM Method:** Descriptor duplication (bit 7 has 32 descriptors → same buffer)
- **Architecture:** Direct LCD_CAM register access, static circular descriptor chain
- **Memory Usage:** ~57 KB single-buffer (64×64, 8-bit)

### ESP32/ESP32-S2 (I2S)

- **Status:** ⏳ Implemented, untested on hardware
- **Memory:** Internal SRAM allocation
- **BCM Method:** Descriptor duplication (same as GDMA)
- **Architecture:** I2S peripheral in LCD mode, static circular descriptor chain
- **Memory Usage:** Similar to GDMA

### ESP32-P4 (PARLIO)

- **Status:** ✅ Tested and working
- **Memory:** PSRAM allocation via EDMA (frees internal SRAM for application use)
- **BCM Method:** Buffer padding (not descriptor duplication)
- **Clock Gating:** MSB (bit 15) controls PCLK on/off for display timing
- **Architecture:** Transaction-based API, single buffer with loop transmission
- **Memory Usage:** ~284 KB single-buffer (64×64, 8-bit) in PSRAM
- **Cache Sync:** Automatic cache flushing for CPU writes to DMA-visible PSRAM

**Why PSRAM?**
- Frees scarce internal SRAM (~500 KB) for application code
- Better scalability for large displays (128×128 = ~1 MB)
- PSRAM typically 8-32 MB on ESP32-P4

### ESP32-C6 (PARLIO)

- **Status:** ⏳ Implemented, untested on hardware
- **Memory:** Same as ESP32-P4 (PSRAM via EDMA)
- **Difference:** No clock gating support (MSB unused, BCM via padding only)
- **Architecture:** Same as ESP32-P4

### Memory Comparison

| Platform | 64×64 8-bit | 128×128 8-bit | Memory Pool | Notes |
|----------|-------------|---------------|-------------|-------|
| GDMA/I2S | ~57 KB      | ~228 KB       | Internal SRAM (~500 KB) | Faster access, descriptor chains |
| PARLIO   | ~284 KB     | ~1.1 MB       | PSRAM (8-32 MB) | Frees SRAM, simpler code |

PARLIO trades memory efficiency for code simplicity and application SRAM availability.

## Troubleshooting

### Black Screen

**Symptom:** Panel doesn't light up at all

**Solutions:**
- Try `shift_driver = ShiftDriver::FM6126A` (most common in modern panels)
- Verify pin mapping matches your board layout
- Check power supply is adequate (64×64 panels can draw 3-4A at full brightness)
- Verify data cable connections are secure

### Incorrect Colors or Garbled Display

**Symptom:** Colors are wrong, swapped, or display shows random patterns

**Solutions:**
- **Wrong shift driver:** Try `FM6126A` (works for most modern panels including ICN2038S)
- **Wrong scan wiring:** Try different `ScanPattern` values if `STANDARD_TWO_SCAN` doesn't work
- **Pin mapping:** Verify R1/G1/B1/R2/G2/B2 are connected to correct GPIOs
- **Scan pattern:** Ensure `scan_pattern` matches panel height (64px = SCAN_1_32)

### Ghosting

**Symptom:** Previous frame content faintly visible, or adjacent rows bleeding into each other

**Solutions:**
- Increase `latch_blanking` parameter (default is 1, try 2-4)
- Lower `output_clock_speed` (signal integrity issue)
- Check for poor-quality data cables

### Flickering

**Symptom:** Display flickers or has visible scanlines

**Solutions:**
- Refresh rate too low - driver will automatically adjust `lsbMsbTransitionBit`
- Increase `min_refresh_rate` parameter
- Reduce `bit_depth` if refresh rate is critically low

### Scrambled/Weird Patterns

**Symptom:** Display shows correct colors but scrambled geometry

**Solutions:**
- **Wrong scan_wiring:** Try `FOUR_SCAN_*` variants for non-standard panels
- **Multi-panel layout:** Verify `layout` setting matches physical wiring
- **Scan pattern:** Double-check `scan_pattern` matches panel specifications

### MBI5124 Panels

**Symptom:** MBI5124 panels don't work correctly

**Solution:**
- Must set `clk_phase_inverted = true` in configuration
- Set `shift_driver = ShiftDriver::MBI5124`

## Advanced Topics

### BCM Timing

The driver uses Binary Code Modulation (BCM) for color depth control. Instead of PWM (which would flicker), BCM varies how long each bit plane is displayed:

- **Pure BCM:** Bit 0 shown 1×, bit 1 shown 2×, bit 2 shown 4×, ..., bit 7 shown 128×
- **Optimized:** Lower bits (≤ lsbMsbTransitionBit) shown only 1×, upper bits get BCM weighting

`lsbMsbTransitionBit` is auto-calculated to achieve target refresh rate. Higher values = faster refresh but slight color depth trade-off on lower bits (perceptually minor due to CIE correction).

**Implementation differs by platform:**
- **GDMA/I2S:** Multiple DMA descriptors pointing to same buffer (descriptor duplication)
- **PARLIO:** Buffer padding with extended display periods (clock gating on ESP32-P4)

### Double Buffering Best Practices

```cpp
// Enable double buffering in config
config.double_buffer = true;
Hub75Driver driver(config);
driver.begin();

// Rendering loop
while (true) {
    // Draw to back buffer
    driver.clear();
    draw_my_frame();

    // Atomic swap
    driver.flip_buffer();

    // Front buffer displays while we draw next frame
    vTaskDelay(pdMS_TO_TICKS(16));  // ~60 FPS
}
```

**Benefits:**
- Eliminates tearing artifacts during animation
- Allows complex frame preparation without visible rendering

**Costs:**
- Doubles memory usage
- Slightly more complex code

### Scan Pattern Details

The `Hub75ScanPattern` enum determines how many row pairs are addressed simultaneously:

```
SCAN_1_32 (64px high panel):
- 32 row pairs (rows 0+32, 1+33, 2+34, ...)
- Address lines A/B/C/D/E select which pair (0-31)

SCAN_1_16 (32px high panel):
- 16 row pairs (rows 0+16, 1+17, 2+18, ...)
- Address lines A/B/C/D select which pair (0-15)
```

Non-standard panels may have shifted registers wired unusually, requiring `scan_wiring` remapping.

## License

MIT License - See repository [LICENSE](https://github.com/stuartparmenter/esp-hub75/blob/main/LICENSE) file for details.

## Support

For issues, examples, and detailed documentation, visit the main repository:
https://github.com/stuartparmenter/esp-hub75
