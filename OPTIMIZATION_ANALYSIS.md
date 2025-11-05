# Optimization Analysis: DMA, SPI Speed, Partial Updates

## Current Status

### ✅ DMA: Enabled but Suboptimal
- **Status**: DMA is enabled (`SPI_DMA_CH_AUTO`)
- **Issue**: Not using DMA completion callbacks
- **Impact**: Using `usleep(900)` delays instead of waiting for DMA completion

### ⚠️ SPI Speed: Could Be Higher
- **Current**: 40 MHz
- **Maximum**: ST7789 supports up to 62.5 MHz (typical)
- **Recommendation**: Increase to 60 MHz for better throughput

### ❌ Partial Updates: NOT IMPLEMENTED
- **Critical Issue**: `redraw()` function **completely ignores** x/y/w/h parameters
- **Current behavior**: Always updates entire screen (240x320 = 76,800 pixels)
- **Impact**: Wasting ~75% of transfer time on unchanged pixels

## Code Issues Found

### Issue 1: Redraw Function Ignores Partial Update Parameters

**File**: `main.c:1269-1289`

```c
static void redraw(void *opaque, int x, int y, int w, int h)
{
    // ❌ Parameters x, y, w, h are COMPLETELY IGNORED!
    // Always updates full screen:
    for (int i = 0; i < NN; i++) {
        // ... full screen update ...
        esp_lcd_panel_draw_bitmap(
            thepanel,
            0, 320 / NN * i,      // Always starts at 0
            240, 320 / NN * (i + 1),  // Always full width
            s->fb1));
    }
}
```

**Fix**: Use the x/y/w/h parameters to only update changed regions.

### Issue 2: VGA Refresh Always Calls Full Screen

**File**: `vga.c:1101`

```c
// At end of vga_graphic_refresh():
redraw_func(opaque, 0, 0, fb_dev->width, fb_dev->height);
// ❌ Always passes full screen dimensions!
```

**Fix**: Track dirty rectangles and pass actual changed regions.

### Issue 3: No DMA Completion Callback

**File**: `esp_main.c:147`

```c
esp_lcd_panel_io_spi_config_t io_config = {
    // ...
    .on_color_trans_done = NULL,  // ❌ Not using DMA completion!
};
```

**Fix**: Implement callback to eliminate sleep delays.

## Recommended Fixes

### Fix 1: Increase SPI Speed

```c
.pclk_hz = 60 * 1000 * 1000, // 60MHz (was 40MHz)
```

### Fix 2: Implement Partial Updates

Track dirty rectangles in VGA state and only update changed regions.

### Fix 3: Use DMA Completion Callbacks

Replace `usleep(900)` with proper DMA completion handling.
