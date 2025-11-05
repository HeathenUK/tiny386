# T-Deck Adaptation - Key Code Locations

## Files Requiring Changes

### 1. Display Driver (`esp/main/esp_main.c`)

**Lines 54-72**: GPIO pin definitions for display
- Current: AXS15231B QSPI pins
- Change to: ST7789 SPI pins

**Lines 76-109**: LCD initialization commands
- Current: AXS15231B init sequence
- Change to: ST7789 init sequence

**Lines 172-178**: SPI bus configuration
- Current: QSPI configuration
- Change to: Standard SPI (3-wire)

**Lines 183-203**: Panel driver initialization
- Current: `esp_lcd_new_panel_axs15231b()`
- Change to: ST7789 panel driver

**Lines 61-62**: Display resolution constants
```c
#define TEST_LCD_SPI_H_RES              (320)  // → (240)
#define TEST_LCD_SPI_V_RES              (480)  // → (320)
```

### 2. Display Rendering (`main.c`)

**Lines 1257-1258**: Framebuffer allocation
```c
c->fb1 = fbmalloc(480 * 320 / NN * 2);  // → (240 * 320 / NN * 2)
c->fb = bigmalloc(480 * 320 * 2);        // → (240 * 320 * 2)
```

**Lines 1273-1286**: Display refresh function
- Update dimensions for 240x320
- Verify chunking strategy (NN = 32)

### 3. Component Dependencies (`esp/main/idf_component.yml`)

**Line 17**: Component dependency
```yaml
espressif/esp_lcd_axs15231b: ==1.0.0  # Remove, add ST7789 driver
```

### 4. Input Handling (New Files Needed)

**File**: `esp/main/tdeck_keyboard.c` (NEW)
- Implement keyboard scanning/reading
- Map to PS/2 scancodes
- Call `ps2_put_keycode(thekbd, is_down, keycode)`

**File**: `esp/main/tdeck_trackball.c` (NEW)
- Implement trackball sensor reading
- Read motion deltas and buttons
- Call `ps2_mouse_event(themouse, dx, dy, dz, buttons)`

**File**: `esp/main/esp_main.c`
- **Line 337**: In `app_main()`, add keyboard task
- **Line 337**: In `app_main()`, add trackball task
- Optionally remove/modify WiFi keyboard setup (lines 466-468)

### 5. Configuration Files (`conf/*.ini`)

**Display section**:
```ini
[display]
width = 720   # → 240
height = 480   # → 320
```

### 6. Hardware Definitions (New File)

**File**: `esp/main/tdeck_hw.h` (NEW)
- GPIO pin definitions
- I2C port definitions
- Hardware-specific constants

## Integration Points

### Keyboard Integration
- Uses existing `ps2_put_keycode()` from `i8042.h`
- Global variable `thekbd` set in `main.c:1460`
- No changes needed to `i8042.c` keyboard emulation

### Mouse Integration  
- Uses existing `ps2_mouse_event()` from `i8042.h`
- Global variable `themouse` set in `main.c:1461`
- No changes needed to `i8042.c` mouse emulation

### Display Integration
- Uses existing `SimpleFBDrawFunc` callback system
- `redraw()` function in `main.c` handles updates
- VGA emulation (`vga.c`) unchanged

## Code Flow

```
app_main() [esp_main.c]
  ├─> vga_task() [esp_main.c:158]
  │     └─> pc_vga_step() [main.c:576]
  │           └─> vga_step() [vga.c]
  │                 └─> vga_refresh() [vga.c]
  │                       └─> redraw() [main.c:1268]
  │                             └─> esp_lcd_panel_draw_bitmap()
  │
  ├─> i386_task() [esp_main.c:37]
  │     └─> main() [main.c:1427]
  │           └─> pc_step() [main.c:585]
  │
  ├─> tdeck_keyboard_task() [NEW]
  │     └─> ps2_put_keycode(thekbd, ...)
  │
  └─> tdeck_trackball_task() [NEW]
        └─> ps2_mouse_event(themouse, ...)
```

## Minimal Changes Summary

**Essential changes for basic functionality**:

1. **Display**: Replace AXS15231B with ST7789 in `esp_main.c`
2. **Resolution**: Update constants in `esp_main.c` and `main.c`
3. **Keyboard**: Create `tdeck_keyboard.c` and integrate in `app_main()`
4. **Trackball**: Create `tdeck_trackball.c` and integrate in `app_main()`
5. **Dependencies**: Update `idf_component.yml`

**Optional/Secondary changes**:
- Configuration file updates
- WiFi keyboard removal (can keep as fallback)
- Display refresh optimization
- Power management
