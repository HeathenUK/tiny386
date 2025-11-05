# T-Deck Implementation Notes

## What Was Changed

### Display (ST7789)
- ✅ Replaced AXS15231B QSPI driver with ST7789 SPI driver
- ✅ Created custom ST7789 panel driver (`st7789_panel.c/h`)
- ✅ Updated GPIO pins to T-Deck configuration
- ✅ Changed resolution from 320x480 to 240x320
- ✅ Updated framebuffer allocations

### Keyboard
- ✅ Created GPIO matrix keyboard driver (`tdeck_keyboard.c/h`)
- ✅ Supports both GPIO matrix and I2C (configurable via `TDECK_KBD_USE_I2C`)
- ✅ Maps physical keys to PS/2 scancodes
- ✅ Integrates with existing PS/2 keyboard emulation

### Trackball
- ✅ Created I2C trackball driver (`tdeck_trackball.c/h`)
- ✅ Reads motion deltas and button states
- ✅ Integrates with existing PS/2 mouse emulation

### Configuration
- ✅ Updated all `.ini` config files with T-Deck resolution
- ✅ Removed AXS15231B component dependency
- ✅ Updated CMakeLists.txt with new source files

## Hardware Configuration

**IMPORTANT**: The GPIO pin assignments in `tdeck_hw.h` are **example values** and need to be verified against the actual T-Deck schematic.

### Display Pins (ST7789)
- MOSI: GPIO 19 (verify)
- SCLK: GPIO 18 (verify)
- CS: GPIO 5 (verify)
- DC: GPIO 23 (verify)
- RST: GPIO 26 (verify)
- BL: GPIO 4 (verify)

### Keyboard
- Default: GPIO matrix (4x4)
- Alternative: I2C (set `TDECK_KBD_USE_I2C` to 1)
- Key mapping needs to be customized in `keycode_map[][]`

### Trackball
- I2C interface
- Default address: 0x42 (verify)
- Register addresses are placeholder - adjust for actual sensor chip

## Next Steps

1. **Verify Hardware Pins**
   - Check T-Deck schematic for actual GPIO assignments
   - Update `tdeck_hw.h` with correct pin numbers

2. **Customize Keyboard Mapping**
   - Map actual T-Deck keys to PS/2 scancodes in `tdeck_keyboard.c`
   - Adjust `keycode_map[][]` array based on keyboard layout

3. **Implement Trackball Protocol**
   - Identify trackball sensor chip (PMW3360, ADNS9800, etc.)
   - Update register addresses in `tdeck_trackball.c`
   - Adjust I2C address if different

4. **Test and Debug**
   - Compile and flash to T-Deck
   - Test display initialization
   - Test keyboard input
   - Test trackball input
   - Adjust delays/timing if needed

5. **Optimize**
   - Adjust display refresh chunking (`NN` in `main.c`)
   - Optimize keyboard scan rate
   - Optimize trackball polling rate

## Known Limitations

- Keyboard keycode mapping is placeholder - needs actual T-Deck layout
- Trackball sensor protocol is placeholder - needs actual chip documentation
- GPIO pins are example values - must verify against hardware
- Display orientation/rotation may need adjustment

## Building

```bash
cd esp
idf.py build
idf.py flash
```

## WiFi Keyboard

The WiFi keyboard functionality is still available as an optional fallback. To disable it, comment out the `wifi_main()` call in `esp_main.c` if you want to use only hardware keyboard/trackball.
