# ESP32 HUB75 DMA Driver - Implementation Plan

## 📁 Repository Structure (2025-01-24)

This repository is now structured as an **ESP-IDF component** with standalone build capability:

```
hub75-esp32/                         # Repository root
├── CMakeLists.txt                   # Root project (for standalone builds via idf.py)
├── main/                            # Test application (for standalone builds)
└── components/
    └── hub75/                       # ← THE COMPONENT
        ├── CMakeLists.txt           # Component build rules
        ├── idf_component.yml        # Component manifest (https://github.com/stuartparmenter/hub75-esp32)
        ├── include/                 # Public API
        └── src/                     # Implementation
```

**When including in your project**: Point to `components/hub75/` subdirectory, not repository root.
**Standalone testing**: Run `idf.py build` at repository root.

See [CLAUDE.md](CLAUDE.md) for detailed build instructions.

## 🎯 Current Status (as of 2025-01-23)

### ✅ Phase 1-2: MOSTLY COMPLETE

**ESP32-S3 GDMA** (Fully Implemented):
- ✅ Static circular DMA descriptor chain (~2,112 descriptors for 8-bit/32-row panel)
- ✅ BCM timing via descriptor duplication (MSB bit 7 gets 32 descriptors pointing to same buffer)
- ✅ NO interrupts, NO ISRs - purely hardware-driven continuous refresh
- ✅ Direct LCD_CAM register manipulation (not high-level API)
- ✅ Previous row address for LSB bit plane (ghosting fix)
- ✅ Dual-mode brightness (basis 1-255 + intensity 0.0-1.0) via OE bit manipulation
- ✅ CIE 1931 LUTs (6/7/8/10/12-bit native)
- ✅ Direct pixel writes to DMA buffers via `drawPixels()`
- ✅ RGB888, RGB888_32, RGB565 input formats with endianness handling
- ✅ Memory: ~4-8KB descriptors + ~32KB row buffers

**ESP32/ESP32-S2 I2S** (Fully Implemented):
- ✅ Static circular DMA descriptor chain (~1,040 descriptors for 8-bit/16-row panel)
- ✅ Same architecture as GDMA, using I2S peripheral in LCD mode
- ✅ NO interrupts - hardware-driven via circular `lldesc_t` chain
- ✅ Previous row address LSB ghosting fix
- ✅ All GDMA features ported to I2S peripheral

**ESP32-P4 PARLIO** (Implemented but Untested):
- ✅ Clock gating architecture (MSB bit 15 controls PCLK on/off)
- ✅ BCM timing via buffer padding (not descriptor repetition)
- ✅ Transaction-based API (no manual descriptor management)
- ✅ Previous row address LSB ghosting fix
- ⚠️ RGB222 only (2-bit per color, hardcoded)
- ⚠️ Memory overhead: ~768KB vs ~48KB for GDMA (buffer padding approach trades memory for simplicity)
- ⏳ Untested on hardware
- ⏳ ESP32-C6 variant not implemented

**Architecture Highlights**:
- Descriptor count formula: LSB bits get 1 descriptor each, MSB bits get 2^(bit - lsbMsbTransitionBit - 1) descriptors
- Example (8-bit, lsbMsbTransitionBit=1): Bit 0-1: 1 desc, Bit 2: 1 desc, Bit 3: 2 desc, ..., Bit 7: 32 desc = 66 per row
- PARLIO memory trade-off: Larger buffers (~16x) but no descriptor management complexity

### 📋 Remaining Work

**Phase 2 Incomplete**:
- ⏳ Temporal dithering (not started)
- ⏳ PSRAM testing
- ⏳ Hardware testing on actual panels

**Phase 3** (Advanced Features):
- ⏳ Different scan patterns (1/4, 1/8, 1/16 scan panels)
- ⏳ Panel chaining support
- ⏳ Driver chip initialization (FM6126A, SM5266P, etc.)
- ⏳ Virtual matrix mapping

### 📋 Next Steps
1. **Hardware testing** on ESP32-S3, ESP32, ESP32-S2 with actual panels
2. Verify refresh rates and color accuracy
3. Test PARLIO implementation on ESP32-P4 (when hardware available)
4. Implement temporal dithering for gradient improvement

---

## 🔧 Recent Technical Work

### Static Circular Descriptor Chain Architecture

The driver uses a fundamentally different approach from traditional ISR-based refresh:

**Key Innovation**: Single static circular DMA descriptor chain that runs **continuously in hardware** with zero CPU intervention after initialization.

**How It Works**:
1. At startup, allocate a static array of DMA descriptors (~2,112 for ESP32-S3 8-bit)
2. Build a circular chain: each descriptor points to the next, last points to first
3. For BCM timing: Create multiple descriptors pointing to the **same buffer**
   - Bit 0 (LSB): 1 descriptor
   - Bit 7 (MSB): 32 descriptors (all pointing to same bit 7 buffer)
   - Hardware automatically transmits same data 32× for MSB = 32× longer display time
4. Start DMA once - hardware loops forever until explicitly stopped
5. Pixel updates write directly to row buffers; changes appear on next refresh cycle

**Ghosting Fix**: LSB bit plane uses **previous row's address** instead of current row's address. This provides settling time for the LAT (latch) signal to complete before the row address changes on subsequent bit planes.

**Memory Trade-offs**:
- **GDMA/I2S**: Descriptor duplication (more descriptors, compact buffers)
  - ~2,112 descriptors (~8KB) + ~32KB row buffers = ~40KB total
- **PARLIO**: Buffer padding (fewer descriptors, larger buffers)
  - ~512 transaction handles + ~768KB padded buffers = ~770KB total
  - MSB bit controls PCLK on/off for BCM timing

**No Interrupts Needed**: The entire refresh mechanism is hardware-driven. No EOF callbacks, no timer ISRs, no state machine complexity.

---

## Project Overview

A highly optimized ESP32 HUB75 LED matrix driver library that builds on insights from four reference implementations:
- **stuartparmenter/ESP32-HUB75-MatrixPanel-DMA** - Mature ESP32 implementation with native CIE LUTs and multi-platform support
- **JuPfu/hub75** - Raspberry Pi Pico PIO-based driver with temporal dithering and dual-mode brightness
- **liebman/esp-hub75** - Rust multi-platform driver with excellent patterns (transfer handle, IRAM optimization, platform abstraction)
- **Espressif PARLIO Example** - Official ESP32-C6/P4 example showing simplified transaction-based API

**Primary Goals:**
1. ESP-IDF first, Arduino compatible
2. Minimal API surface (pixel-setting only, no drawing primitives)
3. Clean, maintainable code without accumulated cruft
4. Support for ESP32, S2, S3, with future C6/P4 PARLIO support

---

## Research Findings Summary

### HUB75 Panel Architecture
- **2 rows rendered simultaneously** (1/16 scan for 32px height, 1/32 scan for 64px height)
- Address lines (A,B,C,D,E) select row **pairs**
- R1/G1/B1 pins → upper half row
- R2/G2/B2 pins → lower half row (offset by height/2)
- Both libraries already optimize for this with interleaved framebuffers
- **8-bit mode**: 6 data pins (R1/G1/B1/R2/G2/B2) + control signals
- **16-bit mode**: 8 data pins (R1/G1/B1/R2/G2/B2 + 2 more) + control signals (for panels with external latch chips)

### JuPfu/hub75 Library (Raspberry Pi Pico - PIO based)

**Key Strengths:**
- **Highly optimized DMA chain**: Eliminates CPU busy-waiting via chained DMA channels
  - pixel_dma → dummy_pixel_dma → oen_dma → oen_finished_dma (triggers IRQ)
- **PIO state machines**: RP2040's Programmable IO for precise timing
  - `hub75_data_rgb888.pio` - shifts pixel data with clock
  - `hub75_row.pio` - handles row address + OEn pulse
- **Temporal dithering**: Accumulator-based for 12-14 bit perceived color depth from 10-bit output
  - ~48KB RAM for 64x64 panel (3 × uint32_t accumulators per pixel)
- **Brightness control**: Dual-mode (basis brightness + intensity scaling) using Q16 fixed-point
- **Clean separation**: Driver logic vs PIO programs vs color management

**Limitations:**
- Raspberry Pi Pico specific (PIO not available on ESP32)
- No panel chaining/remapping logic
- Tied to PicoGraphics library

**Directly Portable Concepts:**
- ✅ Temporal dithering algorithm (pure software)
- ✅ Dual-mode brightness control API
- ✅ Cleaner DMA chain architecture (conceptually)
- ✅ Interrupt-driven approach

**Not Portable:**
- ❌ PIO state machines (no ESP32 hardware equivalent)
- ❌ PIO instruction set
- ❌ Hardware bit extraction/shifting

### Your ESP32-HUB75-MatrixPanel-DMA Fork

**Key Strengths:**
- **Mature ESP32 implementation**: Uses ESP32 I2S peripheral in LCD mode
- **Multi-platform**: ESP32, S2, S3, C6 with platform-specific optimizations
- **Virtual matrix panel**: Sophisticated chaining (CHAIN_TOP_LEFT_DOWN, zigzag, etc.)
- **Native bit-depth CIE LUTs**: Recent work (commits dfd085f, 77a1415) is excellent!
  - 6/7/8/10/12-bit native lookups eliminate runtime conversion
  - Python tool (`tools/generate_cie_luts.py`) to regenerate
- **Extensive panel support**: FM6126A init, SM5266P, SM5368, 4-scan patterns
- **Flexible configuration**: Runtime pin mapping, driver chip support

**Recent Improvements:**
- Native CIE LUTs for each bit depth (no more runtime conversion)
- Fixed brightness calculations
- Better gradient quality

**Complexity Issues:**
- Code has accumulated cruft over years of development
- Mix of concerns (drawing primitives, GFX libraries, panel management)
- DMA descriptor creation is complex but **necessary for ESP32**
- Control signals (LAT, OE, address) embedded in pixel data

### ESP32 I2S LCD Mode vs PIO

**Why ESP32 DMA is Complex:**
- I2S peripheral is being "abused" for parallel GPIO output
- No programmable logic like PIO
- All timing done via:
  - DMA transfer speed
  - Data duplication in framebuffer
  - Descriptor repetition for BCM
- Control signals must be embedded in pixel data
- Original ESP32 has TX FIFO byte reordering quirk

**ESP32-S3 Advantages:**
- GDMA (Generic DMA) instead of I2S DMA - more flexible
- LCD_CAM peripheral purpose-built for this
- Better clock control (up to 160MHz base)
- Cleaner register interface
- **Note:** async_memcpy probably won't help (DMA already async)

**ESP32-C6/P4 PARLIO:**
- Has PARLIO (Parallel IO) peripheral - **much simpler than I2S/LCD_CAM**
- Transaction-based API with built-in queue
- No manual DMA descriptor management
- Clean clock generation (up to 80MHz output)
- Available on: ESP32-C5, C6, H2, P4
- Worth prioritizing in Phase 4

### liebman/esp-hub75 Library (Rust - Multi-platform)

**Key Strengths:**
- **Excellent platform abstraction**: Conditional compilation for ESP32/S3/C6
  - `i2s_parallel.rs` - ESP32 I2S implementation
  - `lcd_cam.rs` - ESP32-S3 LCD_CAM implementation
  - `parl_io.rs` - ESP32-C6 PARLIO implementation
- **Transfer handle pattern**: Rust-style ownership for DMA transfers
  - `render()` consumes driver, returns `Hub75Transfer`
  - `wait()` on transfer returns driver back (prevents concurrent rendering)
- **IRAM optimization**: `#[ram]` attribute for hot-path functions
- **8-bit and 16-bit mode support**:
  - 8-bit: 6 data pins (standard HUB75)
  - 16-bit: 8 data pins (for panels with external latch chips)
- **Zero-copy framebuffer design**: Direct DMA from user buffer
- **Async and blocking APIs**: Both `wait()` and `wait_for_done().await`

**Directly Portable Concepts:**
- ✅ Transfer handle pattern (C++ move semantics)
- ✅ Platform abstraction via conditional compilation
- ✅ IRAM placement for critical code
- ✅ 8-bit mode support (standard HUB75)
- ✅ Zero-copy framebuffer design

**Not Directly Portable:**
- ❌ Rust's ownership system (but can approximate with move semantics)
- ❌ Async/await syntax (but can use FreeRTOS tasks)

### Espressif PARLIO Example

**Key Insights:**
- **Much simpler API than I2S/LCD_CAM**:
  ```c
  parlio_tx_unit_config_t config = {
      .clk_src = PARLIO_CLK_SRC_DEFAULT,
      .data_width = 8,  // or 16
      .output_clk_freq_hz = 10 * 1000 * 1000,
      .trans_queue_depth = 32,
  };
  parlio_tx_unit_transmit(tx_unit, payload, size, -1);
  parlio_tx_unit_wait_all_done(tx_unit, -1);
  ```
- **No manual DMA descriptor creation** - handled internally
- **Transaction queue** - submit multiple transfers efficiently
- **Clean clock generation** - specify output frequency directly
- **Available on**: ESP32-C5, ESP32-C6, ESP32-H2, ESP32-P4

**Implications:**
- PARLIO should be prioritized for C6/P4 support
- Could enable higher refresh rates with less CPU overhead
- Simpler code = easier to maintain
- Consider PARLIO as "reference implementation" for clarity

---

## Architecture Design

### Core Principles

1. **ESP-IDF first, Arduino compatible**
2. **Minimal API surface** - pixel setting only, no drawing primitives
3. **Platform abstraction** - clean separation between ESP32 variants
4. **Modular design** - separate DMA engine, gamma/brightness, panel remapping
5. **Zero-overhead abstractions** - use templates where beneficial

### Component Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     User Application                         │
│        (Uses Adafruit_GFX, LVGL, or direct pixel API)       │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────────┐
│                  HUB75Driver API                             │
│  • begin()                     - Start continuous refresh   │
│  • setPixel(x, y, r, g, b)     - RGB888                     │
│  • setPixelRGB565(x, y, rgb)   - RGB565 (endian aware)      │
│  • setPixelRaw(x, y, packed)   - Native format              │
│  • setBrightness() / setGammaMode()                          │
│  • flipBuffer() / clearBuffer()                              │
└──────────────────────┬──────────────────────────────────────┘
                       │
        ┌──────────────┼──────────────┬─────────────────┐
        │              │              │                 │
┌───────▼──────┐ ┌────▼─────┐ ┌──────▼──────┐ ┌────────▼────────┐
│ Color LUT    │ │Framebuffer│ │DMA Engine   │ │Panel Mapper     │
│ Management   │ │Management │ │(Platform)   │ │(OPTIONAL)       │
│              │ │           │ │             │ │                 │
│• CIE LUTs    │ │• Alloc    │ │• ESP32      │ │• Virtual matrix │
│• RGB565 conv │ │• Layout   │ │• ESP32-S3   │ │• Chain types    │
│• Temporal    │ │• Clear    │ │• ESP32-S2   │ │• Scan remapping │
│  dithering   │ │• Swap     │ │• ESP32-C6   │ │  (template)     │
│              │ │           │ │• ESP32-P4   │ │                 │
└──────────────┘ └───────────┘ └──────┬──────┘ └─────────────────┘
                                      │
                         ┌────────────▼────────────┐
                         │  Static Circular DMA    │
                         │  Descriptor Chain       │
                         │  • Hardware-driven      │
                         │  • No CPU intervention  │
                         └─────────────────────────┘
```

### Simple Usage Model - Continuous Refresh

**Important:** HUB75 panels have **no internal framebuffer memory**. They must be continuously refreshed or the display goes blank.

The driver handles this automatically with a dead-simple API:

```cpp
// 1. Setup and start
HUB75Driver driver(config);
driver.begin();  // Starts internal refresh loop (ISR or FreeRTOS task)

// 2. Just draw - changes appear automatically
driver.setPixel(10, 10, 255, 0, 0);  // Red pixel
driver.setPixel(20, 20, 0, 255, 0);  // Green pixel
driver.setPixel(30, 30, 0, 0, 255);  // Blue pixel

// 3. Optional: Double buffering for tear-free animation
driver.clearBuffer();           // Draw to back buffer
driver.setPixel(30, 30, 255, 255, 255);
// ... draw complete frame ...
driver.flipBuffer();            // Atomic swap on next refresh

// That's it! No render() calls, no waiting, no transfer handles.
```

**How it works internally:**
- Static circular DMA descriptor chain loops continuously in hardware (no CPU needed)
- Pixel updates write directly to row buffers, changes appear on next refresh cycle
- Typical refresh rate: 60-300+ Hz depending on panel size, bit depth, and lsbMsbTransitionBit
- **Compatible with LVGL, Adafruit_GFX, and other graphics libraries** - they just call setPixel/drawPixels

**Inspired by ESP32-HUB75-MatrixPanel-DMA and liebman/esp-hub75**, with architectural improvements for zero CPU overhead.

### Pixel Format Support

#### Internal Format (in framebuffer)
Store pixels in BCM-ready format:
- **8-bit mode**: R8:G8:B8 packed into 32-bit words (with padding)
- **10-bit mode**: R10:G10:B10 packed into 32-bit words
- **12-bit mode**: R12:G12:B12 packed into 32-bit words
- Matches JuPfu's approach for efficient DMA transfer

#### Input Formats
1. **RGB888** (uint8_t r, g, b)
   - Most common, easy to use
   - Apply CIE LUT during conversion

2. **RGB565** (uint16_t)
   - Common in graphics libraries
   - Handle both endianness:
     - Little-endian: `GGGBBBBB RRRRRGGG`
     - Big-endian: `RRRRRGGG GGGBBBBB`
   - Expand 5→8 bits (R,B) and 6→8 bits (G), then apply LUT

3. **Raw/Native** (uint32_t)
   - For advanced users who pre-compute
   - Bypass LUT for maximum performance

---

## File Structure

```
hub75/
├── include/
│   ├── hub75.h                    # Main public API
│   ├── hub75_config.h             # Compile-time configuration
│   └── hub75_types.h              # Common types/enums
│
├── src/
│   ├── core/
│   │   ├── hub75_driver.cpp       # Core driver implementation
│   │   └── hub75_driver.hpp
│   │
│   ├── color/
│   │   ├── color_lut.cpp          # Gamma/CIE tables (native bit-depth)
│   │   ├── color_lut.hpp
│   │   ├── color_convert.cpp      # Pixel format conversions
│   │   └── color_convert.hpp
│   │
│   ├── platforms/
│   │   ├── platform_dma.hpp       # Abstract DMA interface
│   │   ├── platform_detect.cpp    # Platform detection
│   │   ├── i2s/                   # ESP32/ESP32-S2 I2S implementation
│   │   │   ├── dma_i2s.cpp        # ~871 lines, static descriptor chains
│   │   │   └── dma_i2s.hpp
│   │   ├── gdma/                  # ESP32-S3 GDMA implementation
│   │   │   ├── dma_gdma.cpp       # ~857 lines, direct LCD_CAM register access
│   │   │   └── dma_gdma.hpp
│   │   └── parlio/                # ESP32-P4 PARLIO implementation
│   │       ├── dma_parlio.cpp     # ~730 lines, clock gating architecture
│   │       └── dma_parlio.hpp
│   │
│   └── optional/
│       ├── panel_mapper.cpp       # Virtual matrix/remapping
│       ├── panel_mapper.hpp       # (template-based)
│       ├── temporal_dither.cpp    # Accumulator-based dithering
│       ├── temporal_dither.hpp
│       └── driver_init.cpp        # FM6126A, SM5266P, etc.
│
├── tools/
│   ├── generate_luts.py           # Port from your repo
│   └── benchmark/                 # Performance testing
│
├── examples/
│   ├── common/
│   │   └── pins_example.h         # Well-commented pin template (NEW)
│   ├── 01_simple_test/            # Single pixel, basic colors
│   ├── 02_rgb565_test/            # RGB565 endianness test
│   ├── 03_double_buffer/          # Smooth animation (flipBuffer demo)
│   ├── 04_lvgl_integration/       # LVGL integration example (NEW)
│   ├── 05_chained_panels/         # Virtual matrix demo
│   ├── 06_temporal_dither/        # Dithering comparison
│   ├── 07_brightness_control/     # Brightness API demo
│   └── 08_parlio_c6/              # ESP32-C6 PARLIO demo (NEW)
│
├── CMakeLists.txt                 # ESP-IDF component
├── library.properties             # Arduino library metadata
└── README.md
```

---

## API Design

### Core Driver Class

```cpp
// hub75_driver.hpp
class HUB75Driver {
public:
    // Initialization
    HUB75Driver(const Config& config);
    bool begin();
    void end();

    // ─────────────────────────────────────────────────────────
    // Pixel Setting (Multiple formats)
    // ─────────────────────────────────────────────────────────

    // RGB888 (most common)
    void setPixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b);

    // RGB565 (graphics library compatible)
    void setPixelRGB565(uint16_t x, uint16_t y, uint16_t rgb565);

    // Native format (advanced users, bypass LUT)
    void setPixelRaw(uint16_t x, uint16_t y, uint32_t packed);

    // ─────────────────────────────────────────────────────────
    // Bulk Operations
    // ─────────────────────────────────────────────────────────

    void fillScreen(uint8_t r, uint8_t g, uint8_t b);
    void fillScreenRGB565(uint16_t rgb565);
    void clearScreen();

    // ─────────────────────────────────────────────────────────
    // Buffer Management
    // ─────────────────────────────────────────────────────────

    void* getFrameBuffer();           // Direct access (advanced)
    size_t getFrameBufferSize();
    void flipBuffer();                // Double buffering (atomic swap)
    void clearBuffer();               // Clear current buffer to black

    // ─────────────────────────────────────────────────────────
    // Brightness & Gamma
    // ─────────────────────────────────────────────────────────

    // Dual-mode brightness (inspired by JuPfu)
    void setBasisBrightness(uint8_t factor);  // 1-255, coarse
    void setIntensity(float intensity);        // 0.0-1.0, fine

    // Legacy compatibility
    void setBrightness(uint8_t brightness);    // 0-255

    // Gamma modes
    enum class GammaMode {
        CIE1931,        // Perceptually linear (default)
        GAMMA22,        // Standard gamma 2.2
        LINEAR,         // No correction
        CUSTOM          // User-provided LUT
    };
    void setGammaMode(GammaMode mode);
    void setCustomGammaLUT(const uint16_t* lut);

    // ─────────────────────────────────────────────────────────
    // Configuration
    // ─────────────────────────────────────────────────────────

    uint16_t getWidth() const;
    uint16_t getHeight() const;
    uint8_t getBitDepth() const;
    uint16_t getRefreshRate() const;

private:
    // Platform-specific DMA engine
    std::unique_ptr<DMAEngine> dma_;

    // Color management
    ColorLUT color_lut_;

    // Optional temporal dithering
    std::unique_ptr<TemporalDither> dither_;

    // Framebuffer(s)
    Framebuffer* fb_;
    Framebuffer* fb_back_;  // For double buffering

    Config config_;

    // Platform-specific implementation handles all DMA/refresh automatically
    // No internal refresh task needed - hardware-driven circular chain
};
```

### Configuration Structure

```cpp
// hub75_config.h
struct Config {
    // Panel dimensions
    uint16_t panel_width = 64;
    uint16_t panel_height = 64;

    // Data width mode (NEW)
    enum class DataWidth {
        MODE_8BIT,   // 6 data pins (standard HUB75)
        MODE_16BIT   // 8 data pins (external latch support)
    } data_width = DataWidth::MODE_8BIT;

    // Pin mapping - USER MUST SPECIFY (no defaults)
    struct Pins {
        int8_t r1, g1, b1;     // Upper half RGB (always used)
        int8_t r2, g2, b2;     // Lower half RGB (always used)

        // 16-bit mode extra pins (set to -1 if using standard 8-bit/6-pin mode)
        // Note: Actual usage depends on panel type - see Phase 2 research
        int8_t r3, g3;         // Extra data pins for 16-bit mode panels

        int8_t a, b, c, d, e;  // Row address (e = -1 for 1/16 scan)
        int8_t lat, oe, clk;   // Control signals
    } pins;

    // Performance
    uint32_t output_clock_speed = 20000000;  // 20MHz (I2S/LCD_CAM/PARLIO output clock)
    uint8_t bit_depth = 8;                    // 6, 7, 8, 10, or 12
    uint16_t min_refresh_rate = 60;           // Hz

    // Features
    bool double_buffer = false;
    bool temporal_dither = false;
    bool clk_phase_inverted = false;

    // Driver chip (for init sequence)
    enum DriverChip {
        GENERIC,
        FM6126A,
        SM5266P,
        SM5368
    } driver_chip = GENERIC;

    // RGB565 endianness
    bool rgb565_big_endian = false;  // Platform default
};
```

**Example Usage (8-bit mode):**
```cpp
// User specifies pins based on their board layout
Config config;
config.panel_width = 64;
config.panel_height = 64;
config.data_width = Config::DataWidth::MODE_8BIT;

// Pin assignments - user's responsibility
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
config.pins.e = -1;  // Not used for 1/16 scan
config.pins.lat = 4;
config.pins.oe = 15;
config.pins.clk = 16;

// 16-bit mode unused
config.pins.r3 = -1;
config.pins.g3 = -1;

HUB75Driver driver(config);
driver.begin();
```

**See `examples/common/pins_example.h` for well-commented templates.**

**Example: setPixelRGB565 Implementation**
```cpp
void HUB75Driver::setPixelRGB565(uint16_t x, uint16_t y, uint16_t rgb565) {
    // Convert RGB565 directly to native format (skip RGB888 intermediate step)
    uint32_t native = ColorConvert::rgb565_to_native(
        rgb565,
        color_lut_.getLUT(),
        config_.bit_depth,
        config_.rgb565_big_endian
    );

    // Write directly to framebuffer
    setPixelRaw(x, y, native);
}
```

**Performance Note:** Direct RGB565→native conversion is more efficient than RGB565→RGB888→native.

### IRAM Optimization (from liebman/esp-hub75)

Critical functions placed in IRAM to avoid flash cache misses during pixel writes:

```cpp
// hub75_config.h
#ifdef HUB75_ENABLE_IRAM
    #define HUB75_IRAM IRAM_ATTR
#else
    #define HUB75_IRAM
#endif

// Usage in hot-path functions
// hub75_driver.hpp and platform implementations
class HUB75Driver {
    HUB75_IRAM void setPixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b);
    HUB75_IRAM void drawPixels(...);
};

// No ISRs in this implementation - all refresh is hardware-driven
```

### Platform Abstraction (from liebman/esp-hub75)

```cpp
// dma_engine_base.hpp
class DMAEngine {
public:
    virtual ~DMAEngine() = default;

    // Platform-agnostic interface
    virtual bool init(const Config& config) = 0;
    virtual void start_transfer(const void* data, size_t len) = 0;
    virtual bool is_transfer_done() const = 0;
    virtual void wait_transfer_complete() = 0;
    virtual void stop() = 0;
};

// platform_detect.hpp - Conditional compilation
#if defined(CONFIG_IDF_TARGET_ESP32)
    #include "platforms/esp32/dma_i2s.hpp"
    using PlatformDMA = ESP32_I2S_DMA;
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    #include "platforms/esp32s2/dma_i2s.hpp"
    using PlatformDMA = ESP32S2_I2S_DMA;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    #include "platforms/esp32s3/dma_gdma.hpp"
    using PlatformDMA = ESP32S3_GDMA;
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
    #include "platforms/esp32c6/dma_parlio.hpp"
    using PlatformDMA = ESP32C6_PARLIO;
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
    #include "platforms/esp32p4/dma_parlio.hpp"
    using PlatformDMA = ESP32P4_PARLIO;
#else
    #error "Unsupported ESP32 variant"
#endif
```

### Pixel Format Conversion (Inline)

```cpp
// color_convert.hpp
namespace ColorConvert {

    // RGB888 to native format (apply LUT + pack)
    HUB75_IRAM inline uint32_t rgb888_to_native(uint8_t r, uint8_t g, uint8_t b,
                                                  const uint16_t* lut, uint8_t bit_depth) {
        uint32_t r_val = lut[r];
        uint32_t g_val = lut[g];
        uint32_t b_val = lut[b];

        // Pack based on bit depth
        if (bit_depth == 10) {
            return (r_val << 20) | (g_val << 10) | b_val;
        } else if (bit_depth == 12) {
            return (r_val << 24) | (g_val << 12) | b_val;
        }
        // ... other depths
        return 0;
    }

    // RGB565 directly to native format (more efficient - no intermediate RGB888)
    HUB75_IRAM inline uint32_t rgb565_to_native(uint16_t rgb565,
                                                  const uint16_t* lut,
                                                  uint8_t bit_depth,
                                                  bool big_endian) {
        uint8_t r5, g6, b5;

        if (big_endian) {
            // Big-endian (byte-swapped): Bytes are GGGBBBBB RRRRRGGG
            // Swap bytes first to get standard bit order
            rgb565 = (rgb565 >> 8) | (rgb565 << 8);
            // Now fall through to standard extraction
        }

        // Standard RGB565 bit layout: RRRRRGGG GGGBBBBB (MSB first)
        r5 = (rgb565 >> 11) & 0x1F;  // Top 5 bits
        g6 = (rgb565 >> 5) & 0x3F;   // Middle 6 bits
        b5 = rgb565 & 0x1F;          // Bottom 5 bits

        // Expand 5/6-bit to 8-bit for LUT lookup
        uint8_t r8 = (r5 << 3) | (r5 >> 2);  // Replicate upper bits
        uint8_t g8 = (g6 << 2) | (g6 >> 4);
        uint8_t b8 = (b5 << 3) | (b5 >> 2);

        // Apply LUT and pack directly to native format
        uint32_t r_val = lut[r8];
        uint32_t g_val = lut[g8];
        uint32_t b_val = lut[b8];

        if (bit_depth == 10) {
            return (r_val << 20) | (g_val << 10) | b_val;
        } else if (bit_depth == 12) {
            return (r_val << 24) | (g_val << 12) | b_val;
        }
        // ... other depths
        return 0;
    }
}
```

---

## Implementation Phases

### Phase 1: Core Driver (MVP) - ~2 weeks

**Goals:**
- Single panel support (64x32, 64x64)
- ESP32 (original) only
- Basic functionality working end-to-end
- Transfer handle pattern implemented

**Tasks:**
- [x] Project structure setup
  - [x] ESP-IDF component structure
  - [x] Arduino library.properties
  - [x] CMakeLists.txt
  - [x] Create `iram_utils.h` header (integrated into hub75_config.h)

- [x] Port CIE LUTs from your repo
  - [x] Copy `src/cie_luts.h` (implemented in color_lut.cpp)
  - [ ] Copy `tools/generate_cie_luts.py`
  - [x] Verify 6/7/8/10/12-bit tables

- [x] Platform abstraction foundation
  - [x] `dma_engine_base.hpp` abstract interface (platform_dma.hpp)
  - [x] `platform_detect.hpp` with conditional compilation
  - [x] Prepare for ESP32-S3/C6/P4 expansion

- [x] Implement core driver class
  - [x] Config structure (with 8-bit/16-bit mode support)
  - [x] Basic initialization
  - [x] Framebuffer allocation (DMA-capable memory)

- [x] Static circular DMA descriptor chain
  - [x] Implemented for ESP32/S2 (I2S) and ESP32-S3 (GDMA)
  - [x] Hardware-driven continuous refresh (no CPU intervention)
  - [x] BCM timing via descriptor duplication
  - [x] IRAM optimization for pixel write functions

- [x] ESP32/S2 I2S DMA implementation
  - [x] I2S LCD mode setup
  - [x] Static descriptor chain with lldesc_t
  - [x] Circular linked list (~1,040 descriptors for 8-bit/16-row)
  - [x] Previous row address ghosting fix

- [x] Pixel format support
  - [x] RGB888 input (with CIE LUT, IRAM-optimized)
  - [x] RGB565 input (both endianness, IRAM-optimized)
  - [x] Native raw format

- [x] Basic brightness control
  - [x] OE bit manipulation (LAT/OE timing in DMA buffer)
  - [x] setBrightness(0-255)
  - [x] **BONUS:** Dual-mode brightness (setBasisBrightness + setIntensity)

- [x] Pin configuration template
  - [x] Create `examples/common/pins_example.h` with detailed comments
  - [x] Document all pin assignments (R1, G1, B1, R2, G2, B2, A-E, LAT, OE, CLK)
  - [x] Show examples for 8-bit and 16-bit modes
  - [x] Include notes about GPIO restrictions per platform

- [x] Examples
  - [x] Simple test (colored squares) - includes pin config example
  - [x] RGB565 endianness test
  - [x] LVGL integration demo (showing continuous refresh compatibility)
  - [x] Double buffer demo

**Success Criteria:**
- Single 64x64 panel displays correct colors
- RGB888 and RGB565 both work
- Continuous refresh runs smoothly without DMA conflicts
- Brightness control functional
- Works seamlessly with LVGL (no manual render() calls needed)
- Compiles for both ESP-IDF and Arduino
- IRAM optimization demonstrates performance improvement

---

### Phase 2: Optimizations & Features - ~2 weeks

**Goals:**
- ESP32-S3 support
- 8-bit and 16-bit mode support
- Performance optimizations
- Advanced color features

**Tasks:**
- [x] ESP32-S3 platform
  - [x] GDMA + LCD_CAM implementation (direct register access, not high-level LCD driver API)
  - [x] Static circular descriptor chain (~2,112 descriptors for 8-bit/32-row)
  - [x] BCM timing via descriptor duplication
  - [x] Previous row address ghosting fix
  - [x] ESP-IDF 5.4+ API compatibility (gpio_func_sel, gdma_new_ahb_channel)
  - [ ] Test PSRAM DMA (if needed)
  - [ ] Benchmark vs ESP32/S2

- [x] ESP32-S2 platform
  - [x] I2S implementation (similar to original)
  - [ ] 8-bit and 16-bit mode support (stubs exist)
  - [ ] Verify pin mappings

- [x] ESPHome integration
  - [x] Component namespace renamed to hub75_display (avoids collision with library namespace)
  - [x] library.json created
  - [x] Component setup and integration code

- [ ] 16-bit mode expansion (ESP32 classic and S2)
  - [ ] Support for 8 data pins on ESP32 classic
  - [ ] Support for 8 data pins on ESP32-S2
  - [ ] Pin configuration validation
  - [ ] Research actual 16-bit panel pinouts from liebman's implementation

- [ ] Temporal dithering (from JuPfu)
  - [ ] Port accumulator algorithm
  - [ ] Per-pixel uint32_t accumulators (R, G, B)
  - [ ] Compile-time enable/disable
  - [ ] Memory budget checks
  - [ ] IRAM optimization

- [x] Dual-mode brightness ✅ **COMPLETED!**
  - [x] setBasisBrightness() - coarse control
  - [x] setIntensity() - fine control
  - [x] Q16 fixed-point math
  - [x] Runtime recalculation of scaled basis
  - [x] Thread-safe updates with spinlock
  - [x] Pre-computed scaled_periods_us[] array

- [x] Double buffering
  - [x] Allocate second framebuffer
  - [x] flipBuffer() implementation
  - [x] DMA descriptor swapping
  - [ ] Thread-safe buffer management (basic swap implemented)

- [ ] Examples
  - [ ] Double buffer animation
  - [ ] Temporal dithering comparison
  - [ ] Brightness control demo
  - [ ] 16-bit mode demo

**Success Criteria:**
- Works on ESP32, S2, S3 (both 8-bit and 16-bit modes)
- Temporal dithering shows improved gradients
- Double buffering eliminates tearing
- Brightness control is smooth
- 16-bit mode works with external latch panels

---

### Phase 3: Advanced Features - ~2 weeks

**Goals:**
- Panel chaining support
- Weird panel compatibility
- Production-ready

**Tasks:**
- [ ] Panel mapper (port VirtualMatrixPanel)
  - [ ] Template-based for zero overhead
  - [ ] Chain types (TOP_LEFT_DOWN, zigzag, etc.)
  - [ ] Scan type remapping (2-scan, 4-scan)
  - [ ] Make optional (don't link if not used)

- [ ] Driver chip initialization
  - [ ] FM6126A init sequence
  - [ ] SM5266P support
  - [ ] SM5368 support
  - [ ] Generic fallback

- [ ] 4-scan panel support
  - [ ] FOUR_SCAN_16PX_HIGH
  - [ ] FOUR_SCAN_32PX_HIGH
  - [ ] FOUR_SCAN_64PX_HIGH
  - [ ] Coordinate remapping logic

- [ ] Board presets (optional - future enhancement)
  - [ ] YAML/JSON preset file format design
  - [ ] Common boards: ESP32-Trinity, Adafruit MatrixPortal S3, etc.
  - [ ] `ConfigPresets::load("board-name")` helper class
  - [ ] Pin override support (user can modify preset)
  - [ ] Documentation for adding custom presets

- [ ] Build system improvements
  - [ ] Kconfig for ESP-IDF
  - [ ] Build-time feature selection
  - [ ] Memory usage reports

- [ ] Documentation
  - [ ] API reference
  - [ ] Wiring guide (with pin configuration examples)
  - [ ] Panel compatibility list
  - [ ] Migration guide from old library
  - [ ] Board preset documentation (if implemented)

- [ ] Examples
  - [ ] Chained panels (2x1, 2x2, 4x1)
  - [ ] Different scan types
  - [ ] Virtual matrix demo
  - [ ] Update `examples/common/pins_example.h` with more boards

**Success Criteria:**
- Panel chaining works for 2x2 grid (128x128)
- Supports FM6126A panels
- Well-documented with clear pin configuration guidance
- Easy to migrate from old library
- (Optional) Board presets simplify common configurations

---

### Phase 4: Next-Gen Platforms & Experimental - Ongoing

**Goals:**
- ESP32-C6/P4 PARLIO support (HIGH PRIORITY)
- Multi-port controllers
- Performance limits

**Tasks:**
- [ ] ESP32-C6 PARLIO support (PRIORITIZED)
  - [ ] Implement PARLIO peripheral using Espressif example as reference
  - [ ] Transaction-based API (no manual DMA descriptors!)
  - [ ] 8-bit and 16-bit mode support
  - [ ] Clean clock generation (up to 80MHz)
  - [ ] Benchmark vs ESP32-S3 GDMA

- [ ] ESP32-P4 PARLIO support
  - [ ] Port C6 implementation to P4
  - [ ] Test higher clock speeds (if supported)
  - [ ] Benchmark vs S3

- [ ] PARLIO optimizations
  - [ ] Transaction queue depth tuning
  - [ ] IRAM optimization
  - [ ] Measure CPU overhead reduction vs I2S/LCD_CAM

- [ ] Multi-port controllers (HD-WF2, etc.)
  - [ ] Research clock sharing between ports
  - [ ] Parallel rendering architecture
  - [ ] Synchronization between ports
  - [ ] PARLIO advantages for multi-port

- [ ] Memory optimizations
  - [ ] PSRAM usage strategies
  - [ ] Compression for static content
  - [ ] Framebuffer sharing techniques

- [ ] Advanced dithering
  - [ ] Error diffusion (Floyd-Steinberg)
  - [ ] Blue noise dithering
  - [ ] Comparison benchmarks

- [ ] Performance profiling
  - [ ] Refresh rate measurements per platform
  - [ ] CPU usage monitoring
  - [ ] Power consumption testing
  - [ ] I2S vs LCD_CAM vs PARLIO comparison

**Success Criteria:**
- ESP32-C6/P4 PARLIO support working
- Cleaner, simpler code for PARLIO platforms
- Performance comparison shows PARLIO advantages
- Multi-port rendering proof-of-concept
- Comprehensive performance documentation

---

## Platform Comparison Table

| Feature | ESP32 | ESP32-S2 | ESP32-S3 | ESP32-C6 | ESP32-P4 |
|---------|-------|----------|----------|----------|----------|
| **Peripheral** | I2S (abused) | I2S (abused) | LCD_CAM + GDMA | PARLIO | PARLIO |
| **DMA Type** | I2S DMA | I2S DMA | GDMA | PARLIO TX | PARLIO TX |
| **Max Clock** | ~20MHz | ~20MHz | ~40MHz | ~80MHz | ~80MHz? |
| **Descriptor Mgmt** | Manual | Manual | Manual | **Automatic** | **Automatic** |
| **Code Complexity** | High | High | Medium | **Low** | **Low** |
| **8-bit Mode** | ✅ | ✅ | ✅ | ✅ | ✅ |
| **16-bit Mode** | ✅ | ✅ | ✅ | ✅ | ✅ |
| **IRAM Required** | Yes | Yes | Yes | Yes | Yes |
| **Phase Priority** | Phase 1 | Phase 2 | Phase 2 | **Phase 4** | Phase 4 |

**Key Insight:** PARLIO (C6/P4) significantly simplifies implementation with transaction-based API and automatic descriptor management. This makes it the "reference implementation" for clarity.

---

## Architecture Pattern Comparison

### liebman/esp-hub75 (Rust) vs Our Plan (C++)

| Aspect | liebman (Rust) | Our Plan (C++) |
|--------|----------------|----------------|
| **Platform Abstraction** | Conditional compilation (`#[cfg_attr]`) | Conditional compilation (`#if defined`) |
| **Transfer Handle** | Consumed by `render()`, returned by `wait()` | Move semantics, returned by `render()` |
| **IRAM Placement** | `#[ram]` attribute | `IRAM_ATTR` / `HUB75_IRAM` macro |
| **8-bit/16-bit Mode** | Runtime enum | Compile-time or runtime config |
| **Async Support** | `async fn wait_for_done().await` | Optional FreeRTOS task integration |
| **Zero-Copy** | Yes | Yes |
| **Memory Safety** | Rust ownership | C++ RAII + move semantics |

**Portability Assessment:**
- ✅ Transfer handle pattern → Directly portable using C++ move semantics
- ✅ IRAM optimization → Use `IRAM_ATTR` instead of `#[ram]`
- ✅ Platform abstraction → Same conditional compilation approach
- ⚠️ Async/await → Can use FreeRTOS tasks or polling instead
- ⚠️ Ownership safety → Requires careful C++ design

---

## Technical Decisions & Rationale

### 1. Target Bit Depth
**Decision:** Configurable at compile-time (6, 7, 8, 10, 12-bit)

**Rationale:**
- Your fork already supports this with native LUTs
- 8-bit default for memory-constrained projects
- 10-bit default for quality (JuPfu uses this)
- 12-bit for maximum quality
- User can choose based on panel size and RAM

### 2. RGB565 Endianness
**Decision:** Runtime config option with sensible default (little-endian)

**Rationale:**
- Most ESP32 graphics libraries use little-endian RGB565
- Some libraries (e.g., certain TFT_eSPI configs) use big-endian
- Runtime config allows flexibility without recompilation
- Default to `false` (little-endian) in Config struct
- User can set `config.rgb565_big_endian = true` if needed
- Conversion function takes runtime parameter for zero-overhead branch prediction

### 3. Temporal Dithering
**Decision:** Optional compile-time feature

**Rationale:**
- Adds ~48KB RAM for 64x64 panel
- Significant quality improvement
- Let user decide: `HUB75_ENABLE_TEMPORAL_DITHER`
- Benchmark shows minimal CPU overhead

### 4. Drawing Primitives
**Decision:** Pure driver, no drawing functions

**Rationale:**
- Keep library focused
- Let users choose: Adafruit_GFX, LVGL, FastLED, etc.
- Reduces code size
- Easier to maintain

### 5. Panel Remapping
**Decision:** Separate optional module, template-based

**Rationale:**
- Not everyone needs it
- Template-based compiles away when not used
- Can be complex for weird panels
- Port your VirtualMatrixPanel logic

### 6. Double Buffering
**Decision:** Optional, enabled at init time

**Rationale:**
- Doubles memory usage
- Eliminates tearing for animations
- User choice based on application needs

### 7. Pin Configuration Approach
**Decision:** No default pin files - users must specify pins explicitly in Config

**Rationale:**
- Pins are board/user-specific, NOT platform-specific
- GPIO assignments depend on PCB layout, user preferences
- Avoids misleading "defaults" that don't fit most boards
- More explicit - users understand they need to know their layout
- Examples provide well-commented templates (see `examples/common/pins_example.h`)
- **Future Enhancement (Phase 3):** Board presets via YAML/JSON
  - Common boards: ESP32-Trinity, Adafruit MatrixPortal S3, etc.
  - `ConfigPresets::load("board-name")` helper
  - User can still override individual pins

---

## Memory Budget Analysis

### 64x64 Panel Examples

**Minimum Configuration (8-bit depth, single buffer, no dithering):**
- Framebuffer: 4096 pixels × 4 bytes = ~16 KB
  - (Stored as BCM-ready format: packed R8:G8:B8:padding in 32-bit words)
- DMA descriptors: ~2 KB
- Code: ~40 KB
- **Total: ~58 KB** ✅ Very feasible

**Recommended Configuration (10-bit, single buffer, temporal dithering):**
- Framebuffer: 4096 pixels × 4 bytes = ~16 KB
- Temporal dither accumulators: 4096 × 3 × 4 bytes = ~48 KB
- DMA descriptors: ~3 KB
- Code: ~50 KB
- **Total: ~117 KB** ✅ Feasible on ESP32

**Maximum Configuration (12-bit, double buffer, temporal dithering):**
- Framebuffer × 2: 4096 × 4 × 2 = ~32 KB
- Temporal dither accumulators: ~48 KB
- DMA descriptors × 2: ~6 KB
- Code: ~60 KB
- **Total: ~146 KB** ✅ Possible but tight

### 128x64 Panel (2×64x64 chained)

**Recommended Configuration:**
- Framebuffer: 8192 pixels × 4 bytes = ~32 KB
- Temporal dither: 8192 × 3 × 4 = ~96 KB
- DMA descriptors: ~6 KB
- Code: ~60 KB
- **Total: ~194 KB** ⚠️ Very tight on ESP32 (~200KB free SRAM)

**Without temporal dithering:**
- **Total: ~98 KB** ✅ Much more comfortable

---

## Testing Strategy

### Unit Tests
- Color conversion functions (RGB565 endianness)
- CIE LUT application
- Temporal dithering accumulator math
- Panel coordinate remapping

### Integration Tests
- Single panel display
- Panel chaining (2x1, 2x2)
- Different scan types
- Different driver chips

### Performance Tests
- Refresh rate measurement
- CPU usage profiling
- Memory leak detection
- DMA descriptor chain validation

### Hardware Compatibility Tests
- ESP32 (original)
- ESP32-S2
- ESP32-S3 (with/without PSRAM)
- Various panel types (1/16 scan, 1/32 scan, 4-scan)
- Different driver chips (FM6126A, SM5266P, SM5368)

---

## Migration Path from Old Library

### API Compatibility Layer (Optional)

```cpp
// For users migrating from old library
class MatrixPanel_I2S_DMA {
public:
    // Old API - wraps new HUB75Driver
    MatrixPanel_I2S_DMA(const HUB75_I2S_CFG& cfg);

    void drawPixel(int16_t x, int16_t y, uint16_t color);
    void fillScreen(uint16_t color);
    void setBrightness(uint8_t b);
    // ... other old API methods

private:
    HUB75Driver driver_;  // New implementation
};
```

### Migration Guide Document

- Side-by-side API comparison
- Code examples (old vs new)
- Performance improvements
- Memory usage changes
- Breaking changes list

---

## Key Learnings from Library Analysis

### From JuPfu's Library (Raspberry Pi Pico - PIO)
1. ✅ **Temporal dithering is portable** - pure software, big quality win
2. ✅ **Dual-mode brightness control is cleaner** - basis + intensity
3. ✅ **Cleaner DMA chain architecture** - separate concerns better
4. ✅ **Interrupt-driven approach** - less CPU overhead
5. ❌ **PIO is not portable** - ESP32 has no equivalent hardware

### From Your Library (ESP32 - Multi-platform)
1. ✅ **Native bit-depth CIE LUTs are brilliant** - keep this approach
2. ✅ **Multi-platform abstraction is essential** - ESP32 variants differ
3. ✅ **Virtual matrix is valuable** - no equivalent in JuPfu
4. ✅ **Driver chip init is necessary** - many panels need it
5. ⚠️ **Complexity is somewhat necessary** - ESP32 requires embedded control signals

### From liebman/esp-hub75 (Rust - Multi-platform)
1. ✅ **Transfer handle pattern prevents bugs** - enforces sequential rendering
2. ✅ **IRAM optimization is critical** - prevents flash cache stalls in ISR
3. ✅ **8-bit and 16-bit mode support** - broader panel compatibility
4. ✅ **Platform abstraction via conditional compilation** - clean separation
5. ✅ **Zero-copy framebuffer** - better performance, less memory

### From Espressif PARLIO Example (ESP32-C6/P4)
1. ✅ **PARLIO is dramatically simpler** - transaction API vs manual descriptors
2. ✅ **Automatic descriptor management** - less code, fewer bugs
3. ✅ **Clean clock generation** - specify frequency directly
4. ✅ **Transaction queue** - submit multiple transfers efficiently
5. ✅ **Should be reference implementation** - clearest code for understanding

### ESP32 DMA Constraints (I2S/LCD_CAM)
1. I2S peripheral "abuse" is the standard approach for ESP32/S2/S3
2. Control signals MUST be embedded in data
3. DMA descriptor complexity is unavoidable (until PARLIO)
4. Original ESP32 has TX FIFO quirks (byte reordering)
5. ESP32-S3 GDMA is cleaner but similar concepts
6. **PARLIO (C6/P4) eliminates most complexity** - prioritize for future

---

## Key Implementation Patterns to Adopt

### 1. Internal Refresh Loop (Priority: HIGH)
**From:** liebman/esp-hub75 (adapted from Transfer Handle pattern)
**Benefit:** Prevents DMA overlap bugs, provides continuous refresh for HUB75
**Implementation:** Internal detail only, not exposed in public API
```cpp
// Internal refresh loop (ISR or FreeRTOS task)
void refresh_loop() {
    while (running) {
        start_dma_transfer();
        wait_for_completion();  // Semaphore/flag prevents overlap
        // Immediately start next transfer
    }
}

// User just calls begin() and draws whenever
driver.begin();  // Starts refresh loop
driver.setPixel(x, y, r, g, b);  // Changes appear automatically
```

### 2. IRAM Optimization (Priority: HIGH)
**From:** liebman/esp-hub75
**Benefit:** Prevents flash cache stalls in interrupt handlers
```cpp
#define HUB75_IRAM IRAM_ATTR  // Compile-time toggle
HUB75_IRAM void setPixel(...);
HUB75_IRAM void dma_isr_handler(...);
```

### 3. Native Bit-Depth CIE LUTs (Priority: HIGH)
**From:** Your ESP32-HUB75-MatrixPanel-DMA fork
**Benefit:** Eliminates runtime conversion, faster pixel writes
```cpp
// Direct lookup, no shifting/rounding
auto red_val = lumConvTab_8bit[red];   // For 8-bit mode
auto red_val = lumConvTab_10bit[red];  // For 10-bit mode
```

### 4. Platform Abstraction (Priority: HIGH)
**From:** liebman/esp-hub75
**Benefit:** Clean separation, easier to add new platforms
```cpp
#if defined(CONFIG_IDF_TARGET_ESP32C6)
    using PlatformDMA = ESP32C6_PARLIO;
#elif ...
```

### 5. Temporal Dithering (Priority: MEDIUM)
**From:** JuPfu/hub75
**Benefit:** Significantly improved gradient quality
```cpp
acc_r[j] += lut[r];  // Accumulate high-precision
uint32_t out = acc_r[j] >> SHIFT;  // Quantize
acc_r[j] -= (out << SHIFT);  // Keep error
```

### 6. Dual-Mode Brightness (Priority: MEDIUM)
**From:** JuPfu/hub75
**Benefit:** Coarse + fine brightness control
```cpp
setBasisBrightness(128);  // Coarse
setIntensity(0.75f);      // Fine scaling
```

### 7. Zero-Copy Framebuffer (Priority: LOW)
**From:** liebman/esp-hub75
**Benefit:** Less memory, no copy overhead
```cpp
void* fb = driver.getFrameBuffer();
// User writes directly to DMA buffer
```

---

## Open Questions for User

Before proceeding, please clarify:

1. **Target bit depth default?**
   - Suggest: 8-bit default, configurable up to 12-bit
   - Your preference?

2. **Temporal dithering default?**
   - Suggest: Disabled by default (save RAM), enable via config
   - Your preference?

3. **RGB565 endianness default?**
   - Decision: Runtime config option (little-endian default)
   - User can set `config.rgb565_big_endian = true` if needed
   - Most flexible approach

4. **Priority platforms?**
   - ESP32 (original) first, then S3, then S2?
   - Or different order?

5. **Panel chaining priority?**
   - Implement in Phase 1 or Phase 3?
   - Common use case for you?

6. **Compatibility layer?**
   - Provide old API wrapper for migration?
   - Or clean break?

7. **ESP32-P4 timeline?**
   - Wait for hardware availability?
   - Research-only for now?

8. **Multi-port controllers (HD-WF2)?**
   - High priority or future exploration?
   - Do you have hardware to test?

---

## Next Steps

1. **User confirms plan and answers questions above**
2. **Create project structure** (directories, CMakeLists, etc.)
3. **Begin Phase 1 implementation**
   - Port CIE LUTs
   - Implement core driver class
   - ESP32 DMA engine
4. **Iterative development with testing**
5. **Documentation as we go**

---

## References

### Reference Libraries
- **JuPfu/hub75**: https://github.com/JuPfu/hub75
  - Raspberry Pi Pico PIO-based driver
  - Temporal dithering, dual-mode brightness control
- **Your fork**: https://github.com/stuartparmenter/ESP32-HUB75-MatrixPanel-DMA
  - Mature ESP32 implementation with native CIE LUTs
  - Multi-platform support, virtual matrix
- **Upstream**: https://github.com/mrfaptastic/ESP32-HUB75-MatrixPanel-I2S-DMA
  - Original ESP32 implementation
- **liebman/esp-hub75**: https://github.com/liebman/esp-hub75
  - Rust implementation with excellent patterns
  - Transfer handle, IRAM optimization, platform abstraction
- **Espressif PARLIO Example**: https://github.com/espressif/esp-idf/tree/v5.4.1/examples/peripherals/parlio/parlio_tx/simple_rgb_led_matrix
  - Official ESP32-C6/P4 PARLIO example
  - Transaction-based API, much simpler than I2S/LCD_CAM

### Technical Resources
- **CIE 1931**: https://jared.geek.nz/2013/02/linear-led-pwm/
- **HUB75 Protocol**: https://learn.adafruit.com/32x16-32x32-rgb-led-matrix
- **ESP32-S3 LCD**: https://blog.adafruit.com/2022/06/21/esp32uesday-more-s3-lcd-peripheral-hacking-with-code/
- **ESP32 PARLIO**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/parlio.html

---

**Last Updated:** 2025-10-21 (Simplified API - removed Transfer Handle from public API, continuous refresh model)
**Status:** Planning Phase - Awaiting User Confirmation
