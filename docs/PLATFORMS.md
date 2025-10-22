# Platform Implementation Details

Detailed comparison and implementation notes for ESP32 platform variants.

## Platform Comparison Table

| Feature | ESP32 / ESP32-S2 | ESP32-S3 | ESP32-P4 | ESP32-C6 |
|---------|------------------|----------|----------|----------|
| **Peripheral** | I2S (LCD mode) | LCD_CAM | PARLIO | PARLIO |
| **DMA Engine** | I2S DMA | GDMA (AHB) | EDMA | EDMA |
| **Memory** | Internal SRAM | Internal SRAM | **PSRAM** | Internal SRAM |
| **Buffer Size** (64×64) | ~57 KB | ~57 KB | ~284 KB | ~284 KB |
| **BCM Method** | Descriptor dup | Descriptor dup | Buffer padding | Buffer padding |
| **Clock Gating** | No | No | **Yes** (MSB) | **No** |
| **Max Clock** | 20 MHz | 40 MHz | 40 MHz+ | 40 MHz+ |
| **Status** | ✅ Tested | ✅ Tested | ✅ Tested | ⏳ Planned |

---

## ESP32 / ESP32-S2: I2S DMA

### Architecture

**Peripheral**: I2S in "LCD mode" (parallel data output)
**DMA**: Internal I2S DMA engine
**Memory**: Internal SRAM only (DMA-capable)

### Implementation

**File**: `src/platforms/i2s/i2s_dma.cpp` (~871 lines)

**Key Features**:
- Static circular `lldesc_t` descriptor chain
- Manual descriptor linking
- Direct I2S register configuration
- BCM via descriptor duplication (bit 7 = 32 descriptors)
- No interrupts or callbacks

**Descriptor Count** (64×32 panel, 8-bit, lsbMsbTransitionBit=1):
- ~1,040 descriptors total
- Bit 0: 1 descriptor
- Bit 7: 32 descriptors (all point to same buffer)

### Memory Layout

**Row Buffer Structure**:
```
[Pixel Data: width×chain_length pixels, OE=HIGH]
[LAT pulse: 1 pixel with LAT=HIGH]
[Latch blanking: latch_blanking pixels, OE=HIGH]
```

**Total Memory** (64×32 panel):
- Row buffers: ~32 KB
- Descriptors: ~25 KB
- **Total: ~57 KB internal SRAM**

### Initialization Sequence

1. Configure I2S peripheral in LCD mode (16-bit parallel)
2. Allocate row buffers in DMA-capable SRAM
3. Build static descriptor chain with BCM repetitions
4. Link last descriptor → first (circular)
5. Start I2S DMA transmission (runs forever)

### Limitations

- **I2S peripheral misuse**: Designed for audio, not parallel data
- **20 MHz max clock**: Higher speeds may be unstable
- **No PSRAM support**: All buffers must be internal SRAM

### Advantages

- **Mature**: Well-tested on ESP32/S2
- **Low memory**: ~57 KB for 64×64 panel
- **Fast internal SRAM**: No cache sync overhead

---

## ESP32-S3: GDMA + LCD_CAM

### Architecture

**Peripheral**: LCD_CAM peripheral (proper parallel data interface)
**DMA**: Generic DMA (GDMA) with AHB channels
**Memory**: Internal SRAM only

### Implementation

**File**: `src/platforms/gdma/gdma_dma.cpp` (~857 lines)

**Key Features**:
- Static circular `gdma_descriptor_t` chain
- GDMA channel allocation: `gdma_new_ahb_channel()`
- Direct LCD_CAM register manipulation (not high-level API)
- BCM via descriptor duplication
- Previous row addressing for ghosting fix
- No interrupts

**Descriptor Count** (64×64 panel, 8-bit, lsbMsbTransitionBit=1):
- ~2,112 descriptors total (32 rows × 66 descriptors/row)
- Bit 0: 1 descriptor
- Bit 7: 33 descriptors (includes +1 for previous row address)

### Memory Layout

Same as I2S (row buffers + LAT + latch blanking), but:
- More rows (32 vs 16 for typical panels)
- More descriptors due to 64-row support

**Total Memory** (64×64 panel):
- Row buffers: ~32 KB
- Descriptors: ~25 KB
- **Total: ~57 KB internal SRAM**

### Initialization Sequence

1. Allocate GDMA AHB channel
2. Configure LCD_CAM peripheral (16-bit parallel mode)
3. Allocate row buffers in DMA-capable SRAM
4. Build static descriptor chain with BCM repetitions
5. Configure GDMA channel (connect to LCD_CAM)
6. Link last descriptor → first (circular)
7. Start GDMA transmission (runs forever)

### Ghosting Fix

**Issue**: Row N+1 can show faint ghosting from Row N
**Solution**: For LSB bit planes only, use previous row's address

```cpp
// For LSB bit planes (bit 0-1):
if (bit <= 1) {
  row_address = (row == 0) ? (rows - 1) : (row - 1);  // Previous row
} else {
  row_address = row;  // Current row
}
```

This gives LAT signal extra settling time during fast LSB transitions.

### Advantages

- **Proper peripheral**: LCD_CAM designed for parallel data
- **Faster**: Up to 40 MHz clock
- **More control**: Direct register access
- **Ghosting fix**: Previous row addressing for clean LSB

### Limitations

- **No PSRAM**: All buffers in internal SRAM
- **Complex registers**: Requires deep ESP32-S3 TRM knowledge

---

## ESP32-P4: PARLIO + EDMA

### Architecture

**Peripheral**: Parallel I/O (PARLIO) - dedicated parallel data interface
**DMA**: Enhanced DMA (EDMA) with **PSRAM support**
**Memory**: **PSRAM via EDMA** (not internal SRAM)

### Implementation

**File**: `src/platforms/parlio/parlio_dma.cpp` (~887 lines)

**Status**: ✅ **Fully tested on ESP32-P4 hardware**

**Key Features**:
- **BCM via buffer padding** (not descriptor duplication)
- Transaction-based API: `parlio_tx_unit_transmit()`
- **Clock gating**: MSB (bit 15) controls PCLK on/off
- **PSRAM buffers**: Frees internal SRAM for application
- Loop transmission mode (hardware circular)
- Cache synchronization (`esp_cache_msync`)
- Single contiguous buffer allocation

### BCM Implementation Difference

**GDMA/I2S**: Duplicate descriptors pointing to same buffer
**PARLIO**: Single buffer with variable padding

**Buffer Structure** (per bit plane):
```
[Pixel Section: MSB=1, RGB data]  ← Clock enabled, data shifts in
[LAT pulse: MSB=1, LAT=HIGH]
[Latch blanking: MSB=1, OE=HIGH]
[PADDING: MSB=0, all zeros] ← Clock disabled, display time!
```

**Padding Formula**:
```cpp
padding = base_padding + (2^(bit - lsbMsbTransitionBit - 1) × width)
```

**Example** (lsbMsbTransitionBit=1):
- Bit 0: ~3 words padding
- Bit 1: ~3 words padding
- Bit 2: ~66 words padding (base + 1×64)
- Bit 7: ~3,073 words padding (base + 32×64)

### Clock Gating (ESP32-P4 Only)

**Conditional**: `#ifdef SOC_PARLIO_TX_CLK_SUPPORT_GATING`

**How it works**:
- MSB (bit 15) of 16-bit word controls PCLK
- `MSB=1`: Clock enabled, panel shifts data
- `MSB=0`: Clock disabled, panel displays latched data

**Pixel Section**: MSB=1 (clock on, data shifts)
**Padding Section**: MSB=0 (clock off, display time = BCM timing)

**ESP32-C6**: No clock gating support, MSB unused, BCM via padding length only

### Memory Layout

**Single Contiguous Buffer**:
```cpp
dma_buffer_ = heap_caps_malloc(
    total_bytes,
    MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM  // ← PSRAM!
);
```

**Metadata Array**: `BitPlaneBuffer[]` tracks offsets within buffer

**Total Memory** (64×64 panel, 8-bit):
- Row buffers: ~284 KB PSRAM
- Transaction handles: ~512 bytes (negligible)
- **Total: ~284 KB PSRAM** (not internal SRAM!)

### Cache Synchronization

PSRAM is cached on ESP32-P4. CPU writes must be flushed for DMA visibility:

```cpp
void flush_cache_to_dma() {
    if (esp_ptr_external_ram(dma_buffer_)) {
        // Flush CPU cache → PSRAM (Cache to Memory)
        esp_cache_msync(dma_buffer_, total_buffer_bytes_,
                       ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                       ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    }
}
```

Called after updating framebuffer pixels.

### Initialization Sequence

1. Allocate PARLIO TX unit: `parlio_new_tx_unit()`
2. Allocate single contiguous PSRAM buffer
3. Build buffer with pixel data + padding (all bit planes for all rows)
4. Cache metadata (offsets) for each bit plane
5. Enable PARLIO unit: `parlio_tx_unit_enable()`
6. Start loop transmission: `parlio_tx_unit_transmit()` with `loop_transmission=1`

**Critical Order**: Unit MUST be enabled BEFORE transmit call!

### Advantages

- **PSRAM usage**: Frees ~57 KB internal SRAM for application
- **Scalability**: Large displays (128×128) fit in PSRAM (8-16 MB)
- **Clock gating**: Precise BCM timing via hardware
- **Simpler code**: No manual descriptor chain management
- **Transaction API**: Higher-level abstraction

### Limitations

- **More memory**: ~5× buffer size vs GDMA (284 KB vs 57 KB)
- **Cache sync overhead**: CPU must flush cache after pixel updates
- **Different memory pool**: PSRAM (abundant) vs SRAM (scarce)

### When to Use PARLIO

**Ideal for**:
- Large displays (128×128+) where GDMA would exhaust SRAM
- Applications needing maximum internal SRAM for code/heap
- ESP32-P4 projects with abundant PSRAM

**Consider GDMA when**:
- Small/medium displays (≤64×64) where SRAM sufficient
- Need absolute minimum memory footprint
- Cache sync overhead is concern

---

## ESP32-C6: PARLIO (Planned)

### Architecture

Same as ESP32-P4 PARLIO **but NO clock gating support**.

**Differences from P4**:
- `SOC_PARLIO_TX_CLK_SUPPORT_GATING` undefined
- MSB (bit 15) unused (always 0)
- BCM timing via padding length only (no hardware clock control)

**Implementation Status**: ⏳ Planned (same code as P4, conditionally compiled)

---

## Platform Selection Guide

### Choose ESP32/S2 when:
- Budget-friendly, well-supported
- Small/medium displays (≤64×64)
- Proven stability required

### Choose ESP32-S3 when:
- Best overall choice for new projects
- Need higher clock speeds (40 MHz)
- Want ghosting fix (previous row addressing)
- Medium/large displays (64×64 to 128×64)

### Choose ESP32-P4 when:
- Need maximum internal SRAM for application
- Large displays (128×128+)
- Have abundant PSRAM (8-16 MB)
- Clock gating feature beneficial

### Choose ESP32-C6 when:
- (When implemented) Similar to P4 but lower cost
- PARLIO features without clock gating

---

## Common Implementation Details

### All Platforms Share

1. **Static descriptor/buffer allocation** - Once at init
2. **Circular refresh** - Hardware-driven infinite loop
3. **CIE1931 gamma LUT** - Same color correction
4. **Dual-mode brightness** - Same OE bit manipulation
5. **Panel layout remapping** - Same coordinate transforms
6. **Shift driver init** - FM6126A, MBI5124, etc.

### Platform-Specific Code Isolation

**Interface**: `src/platforms/platform_dma.h`
**Implementations**:
- `src/platforms/i2s/i2s_dma.cpp`
- `src/platforms/gdma/gdma_dma.cpp`
- `src/platforms/parlio/parlio_dma.cpp`

**Detection**: `src/util/platform_detect.h` (compile-time via `CONFIG_IDF_TARGET_*`)

---

## Performance Comparison

### Refresh Rate (64×64 panel, 8-bit, 60 Hz target)

| Platform | Actual Refresh | CPU Usage | Notes |
|----------|---------------|-----------|-------|
| ESP32 | 60-90 Hz | ~0% | I2S DMA |
| ESP32-S3 | 90-120 Hz | ~0% | Faster clock |
| ESP32-P4 | 60-90 Hz | ~0% + cache sync | PSRAM latency |

**CPU Usage**: Near-zero during refresh (hardware-driven). Only spikes during pixel updates.

### Memory Comparison (64×64 panel)

| Platform | Internal SRAM | PSRAM | Total |
|----------|---------------|-------|-------|
| ESP32/S2 | 57 KB | 0 | 57 KB |
| ESP32-S3 | 57 KB | 0 | 57 KB |
| ESP32-P4 | ~1 KB | 284 KB | 285 KB |

---

## Troubleshooting Platform-Specific Issues

### ESP32/S2: Flickering at High Bit Depths
- **Cause**: I2S DMA timing constraints
- **Fix**: Reduce bit depth to 8-bit or lower clock to 10 MHz

### ESP32-S3: Ghosting on LSB
- **Cause**: Fast row transitions without LAT settling
- **Fix**: Already implemented (previous row addressing for LSB)

### ESP32-P4: Black screen
- **Cause**: Cache not synchronized or unit not enabled before transmit
- **Fix**: Ensure `parlio_tx_unit_enable()` before `transmit()`, verify cache sync

### ESP32-P4: Colors incorrect
- **Cause**: Clock gating not working or MSB bit inverted
- **Fix**: Verify `SOC_PARLIO_TX_CLK_SUPPORT_GATING` defined for P4

---

## Related Documentation

- **[ARCHITECTURE.md](ARCHITECTURE.md)** - Core BCM and DMA concepts
- **[MEMORY_USAGE.md](MEMORY_USAGE.md)** - Detailed memory calculations
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Platform-specific debugging

---

**For implementation details**, see source files:
- ESP32/S2: `components/hub75/src/platforms/i2s/i2s_dma.cpp`
- ESP32-S3: `components/hub75/src/platforms/gdma/gdma_dma.cpp`
- ESP32-P4: `components/hub75/src/platforms/parlio/parlio_dma.cpp`
