# T-Deck Adaptation Summary

## Overview

The Tiny386 emulator currently targets generic ESP32-S3 hardware with:
- **AXS15231B LCD** (320x480, QSPI)
- **WiFi-based keyboard/mouse** (TCP socket server)

To adapt for **Lilygo T-Deck**, three main components need changes:

## 1. Display: AXS15231B → ST7789

**Impact**: High - Core functionality  
**Complexity**: Medium - Driver replacement

**Changes**:
- Replace LCD driver from `esp_lcd_axs15231b` to ST7789 driver
- Change from QSPI (4-wire) to SPI (3-wire) interface  
- Update GPIO pins (CS, MOSI, SCLK, DC, RST, BL)
- Change resolution from 320x480 to 240x320
- Update framebuffer allocations

**Files**: `esp/main/esp_main.c`, `main.c`, `esp/main/idf_component.yml`

## 2. Keyboard: WiFi Socket → Hardware Keyboard

**Impact**: High - User input  
**Complexity**: Medium - New driver needed

**Changes**:
- Implement GPIO matrix or I2C keyboard driver
- Map physical keys to PS/2 scancodes
- Replace WiFi socket server input with hardware polling
- Create new task for keyboard scanning

**Files**: New `esp/main/tdeck_keyboard.c`, modify `esp/main/esp_main.c`

## 3. Mouse: WiFi Socket → Trackball

**Impact**: High - User input  
**Complexity**: Medium - New driver needed

**Changes**:
- Implement I2C trackball sensor driver (e.g., PMW3360)
- Read motion deltas and button states
- Replace WiFi socket mouse events with hardware reading
- Create new task for trackball polling

**Files**: New `esp/main/tdeck_trackball.c`, modify `esp/main/esp_main.c`

## Estimated Effort

| Component | Time Estimate | Risk Level |
|-----------|---------------|------------|
| Display (ST7789) | 4-8 hours | Medium |
| Keyboard Driver | 4-6 hours | Low-Medium |
| Trackball Driver | 4-6 hours | Medium |
| Integration & Testing | 4-8 hours | Low |
| **Total** | **16-28 hours** | |

## Dependencies

- ESP-IDF LCD Panel API (already used)
- ST7789 driver component (community or custom)
- T-Deck hardware schematic/documentation
- I2C driver (if keyboard/trackball use I2C)
- GPIO driver (for keyboard matrix if applicable)

## What Stays the Same

✅ **No changes needed**:
- CPU emulation (`i386.c`)
- VGA emulation (`vga.c`) 
- PS/2 emulation (`i8042.c`)
- IDE/PCI/peripheral emulation
- BIOS/VGA BIOS loading
- Main emulator loop

✅ **Optional to keep**:
- WiFi functionality (can remain for remote access)
- SD card support
- Audio (I2S) output

## Next Steps

1. **Verify T-Deck hardware specs**
   - Confirm ST7789 pin assignments
   - Identify keyboard interface (GPIO matrix vs I2C)
   - Identify trackball sensor chip and interface

2. **Implement display driver**
   - Find/adapt ST7789 ESP-IDF driver
   - Update GPIO and initialization code
   - Test basic display output

3. **Implement input drivers**
   - Create keyboard driver based on interface type
   - Create trackball driver
   - Test PS/2 integration

4. **Integration testing**
   - Verify all components work together
   - Test with actual emulator workload
   - Optimize performance

## Documentation References

- `T_DECK_ADAPTATION.md` - Detailed adaptation guide
- `T_DECK_CODE_LOCATIONS.md` - Specific code locations to modify
