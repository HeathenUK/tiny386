# Adapting Tiny386 for Lilygo T-Deck

## Overview

This document outlines the changes needed to adapt the Tiny386 emulator from its current generic ESP32-S3 implementation to the Lilygo T-Deck hardware, which features:
- **ST7789 LCD display** (240x320 pixels, SPI interface)
- **Built-in keyboard** (GPIO matrix or I2C)
- **Trackball mouse** (I2C or GPIO-based)

## Current Implementation Analysis

### Display (Current: AXS15231B)
- **Location**: `esp/main/esp_main.c` (lines 54-222)
- **Driver**: ESP-IDF `esp_lcd_axs15231b` component
- **Interface**: QSPI (4-line SPI)
- **Resolution**: 320x480 (16-bit RGB565)
- **GPIO Pins**:
  - CS: GPIO 45
  - PCLK: GPIO 47
  - DATA0-3: GPIO 21, 48, 40, 39
  - DC: GPIO 8
  - BL (Backlight): GPIO 1
  - RST: GPIO_NC (not connected)

### Input (Current: WiFi Socket Server)
- **Location**: `esp/main/wifi.c`
- **Method**: TCP socket server on port 9999
- **Protocol**: PS/2 keycodes and mouse events over TCP
- **Keyboard**: Receives keycode bytes (7-bit + up/down flag)
- **Mouse**: Receives 4-byte packets (dx, dy, dz, buttons)

### Display Rendering
- **Location**: `main.c` (lines 1266-1287)
- **Method**: Framebuffer is split into 32 chunks (`NN = 32`)
- **Format**: RGB565 (16-bit per pixel)
- **Update**: Uses `esp_lcd_panel_draw_bitmap()` to update display

## Required Changes

### 1. Display Driver Replacement

#### 1.1 Replace AXS15231B with ST7789

**File**: `esp/main/esp_main.c`

**Changes needed**:
1. Replace `#include "esp_lcd_axs15231b.h"` with ST7789 driver includes
2. Replace AXS15231B initialization commands with ST7789 init sequence
3. Update GPIO pin definitions for T-Deck hardware
4. Change from QSPI to standard SPI interface
5. Update resolution from 320x480 to 240x320

**ST7789 Typical Pinout** (verify against T-Deck schematic):
- MOSI: Typically GPIO 19
- SCLK: Typically GPIO 18
- CS: Typically GPIO 5
- DC (RS): Typically GPIO 23
- RST: Typically GPIO 26
- BL (Backlight): Typically GPIO 4

**Key Code Changes**:
```c
// Replace AXS15231B includes
#include "esp_lcd_panel_rgb.h"  // For ST7789 SPI mode
// OR use a community ST7789 driver component

// Replace panel driver initialization
// Change from axs15231b to st7789 driver
esp_lcd_panel_handle_t panel_handle = NULL;
esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = T_DECK_RST_PIN,
    .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
    .bits_per_pixel = 16,  // RGB565
};

// Update SPI bus configuration for 3-wire SPI instead of QSPI
spi_bus_config_t buscfg = {
    .mosi_io_num = T_DECK_MOSI_PIN,
    .sclk_io_num = T_DECK_SCLK_PIN,
    .quad_wp_io_num = -1,  // Not used for SPI
    .quad_hd_io_num = -1,  // Not used for SPI
    .max_transfer_sz = 240 * 320 * 2,  // RGB565 framebuffer size
};
```

#### 1.2 Update Display Resolution

**Files**: 
- `esp/main/esp_main.c` (TEST_LCD_SPI_H_RES, TEST_LCD_SPI_V_RES)
- `main.c` (console_init, framebuffer allocation)
- Configuration files in `conf/` directory

**Changes**:
```c
#define TEST_LCD_SPI_H_RES              (240)  // Changed from 320
#define TEST_LCD_SPI_V_RES              (320)  // Changed from 480
```

**Framebuffer size calculation**:
```c
// In main.c console_init()
c->fb1 = fbmalloc(240 * 320 / NN * 2);  // Changed from 480 * 320
c->fb = bigmalloc(240 * 320 * 2);        // Changed from 480 * 320
```

#### 1.3 Update Display Refresh Logic

**File**: `main.c` (redraw function, lines 1266-1287)

The current implementation splits the display into 32 chunks. For 240x320, you may want to adjust this:
```c
#define NN 16  // Or keep 32, adjust accordingly
// Update redraw function to use correct dimensions
```

### 2. Keyboard Input Integration

#### 2.1 Remove WiFi Keyboard Dependencies

**Files**: 
- `esp/main/esp_main.c` (remove wifi_main call if not needed)
- `esp/main/wifi.c` (can be kept for optional WiFi features, but keyboard disabled)

#### 2.2 Implement Hardware Keyboard Driver

**T-Deck Keyboard Interface**:
The T-Deck keyboard is typically a GPIO matrix or I2C device. You'll need to:

1. **Detect keyboard type**: Check T-Deck documentation/schematic
   - If GPIO matrix: Scan rows/columns periodically
   - If I2C: Use I2C driver to read key states

2. **Create keyboard task**:
```c
// New file: esp/main/tdeck_keyboard.c
#include "driver/gpio.h"
#include "driver/i2c.h"  // If I2C-based
#include "../../i8042.h"

extern volatile void *thekbd;

// GPIO matrix keyboard (example)
static void tdeck_keyboard_task(void *arg)
{
    // Initialize GPIO pins for keyboard matrix
    // Scan matrix periodically
    // Convert physical keys to PS/2 scancodes
    // Call ps2_put_keycode(thekbd, is_down, keycode)
}

// OR I2C keyboard (example)
static void tdeck_keyboard_task(void *arg)
{
    // Initialize I2C for keyboard
    // Read key states via I2C
    // Convert to PS/2 scancodes
    // Call ps2_put_keycode(thekbd, is_down, keycode)
}
```

3. **Keycode Mapping**:
   - Map T-Deck physical keys to PC/AT scancodes
   - Handle modifiers (Shift, Ctrl, Alt)
   - Handle special keys (Function keys, etc.)

**Integration Point**:
In `esp/main/esp_main.c`, `app_main()`:
```c
// Replace or supplement wifi keyboard
xTaskCreatePinnedToCore(tdeck_keyboard_task, "keyboard", 4096, NULL, 1, NULL, 0);
```

### 3. Trackball Mouse Integration

#### 3.1 Remove WiFi Mouse Input

Same as keyboard - remove dependency on WiFi socket server for mouse.

#### 3.2 Implement Trackball Driver

**T-Deck Trackball Interface**:
Typically I2C-based (e.g., PMW3360 or similar optical sensor)

**Implementation**:
```c
// New file: esp/main/tdeck_trackball.c
#include "driver/i2c.h"
#include "../../i8042.h"

extern volatile void *themouse;

static void tdeck_trackball_task(void *arg)
{
    // Initialize I2C for trackball sensor
    // Read motion deltas (dx, dy)
    // Read button states
    // Call ps2_mouse_event(themouse, dx, dy, dz, buttons)
    
    int8_t dx, dy;
    uint8_t buttons = 0;
    
    // Read from trackball sensor via I2C
    // dx, dy = trackball_read_motion();
    // buttons = trackball_read_buttons();
    
    ps2_mouse_event(themouse, dx, dy, 0, buttons);
}
```

**Integration Point**:
In `esp/main/esp_main.c`, `app_main()`:
```c
xTaskCreatePinnedToCore(tdeck_trackball_task, "trackball", 4096, NULL, 1, NULL, 0);
```

### 4. Component Dependencies

#### 4.1 Update ESP-IDF Component

**File**: `esp/main/idf_component.yml`

**Current**:
```yaml
espressif/esp_lcd_axs15231b: ==1.0.0
```

**Change to** (if using community ST7789 driver):
```yaml
# Remove axs15231b, add ST7789 driver
# Example (verify actual component name):
esp_lcd_st7789: "^1.0.0"
# OR use a community component like:
# lvgl/lvgl: "^8.0.0"  # If using LVGL which has ST7789 support
```

**Alternative**: Implement ST7789 driver directly using ESP-IDF LCD panel API.

### 5. Configuration Updates

#### 5.1 Update Display Configuration

**Files**: `conf/*.ini`

Update display resolution:
```ini
[display]
width = 240
height = 320
```

#### 5.2 Hardware-Specific Configuration

Consider adding a new config file or section:
```ini
[hardware]
board = tdeck
display = st7789
keyboard = gpio_matrix  # or i2c
trackball = i2c
```

### 6. GPIO Pin Definitions

Create a hardware abstraction header:

**New file**: `esp/main/tdeck_hw.h`
```c
#ifndef TDECK_HW_H
#define TDECK_HW_H

// ST7789 Display Pins (verify against T-Deck schematic)
#define TDECK_LCD_MOSI_PIN     GPIO_NUM_19
#define TDECK_LCD_SCLK_PIN     GPIO_NUM_18
#define TDECK_LCD_CS_PIN       GPIO_NUM_5
#define TDECK_LCD_DC_PIN       GPIO_NUM_23
#define TDECK_LCD_RST_PIN      GPIO_NUM_26
#define TDECK_LCD_BL_PIN       GPIO_NUM_4

// Keyboard Pins (if GPIO matrix - verify schematic)
#define TDECK_KBD_ROW_START    0  // First row GPIO
#define TDECK_KBD_ROW_END      3   // Last row GPIO
#define TDECK_KBD_COL_START    10 // First column GPIO
#define TDECK_KBD_COL_END      13 // Last column GPIO

// Trackball I2C (if I2C-based)
#define TDECK_TRACKBALL_I2C_PORT    I2C_NUM_0
#define TDECK_TRACKBALL_SDA_PIN     GPIO_NUM_21
#define TDECK_TRACKBALL_SCL_PIN     GPIO_NUM_22
#define TDECK_TRACKBALL_I2C_ADDR    0x42  // Verify address

// Keyboard I2C (if I2C-based instead of GPIO)
#define TDECK_KBD_I2C_PORT          I2C_NUM_0
#define TDECK_KBD_SDA_PIN           GPIO_NUM_21
#define TDECK_KBD_SCL_PIN           GPIO_NUM_22
#define TDECK_KBD_I2C_ADDR          0x55  // Verify address

#endif
```

## Implementation Steps

### Phase 1: Display
1. Replace AXS15231B driver with ST7789
2. Update GPIO pin definitions
3. Update resolution constants
4. Test display initialization and basic drawing

### Phase 2: Keyboard
1. Identify keyboard interface (GPIO vs I2C)
2. Implement keyboard scanning/task
3. Map keys to PS/2 scancodes
4. Integrate with existing PS/2 keyboard emulation

### Phase 3: Trackball
1. Identify trackball sensor chip
2. Implement I2C/GPIO interface
3. Read motion deltas and buttons
4. Integrate with PS/2 mouse emulation

### Phase 4: Testing & Optimization
1. Test all input/output paths
2. Optimize display refresh rate
3. Adjust task priorities
4. Verify performance

## Key Considerations

1. **Display Refresh**: The current chunked update (32 parts) may need adjustment for 240x320. Consider full-screen updates or different chunking strategy.

2. **Input Latency**: Hardware keyboard/trackball should have lower latency than WiFi. Monitor task priorities.

3. **Power Management**: T-Deck may have power management features. Consider backlight control and sleep modes.

4. **I2C Bus Sharing**: If both keyboard and trackball use I2C, ensure proper bus sharing and addressing.

5. **GPIO Conflicts**: Verify no GPIO pin conflicts between display, keyboard, trackball, and other peripherals (SD card, etc.).

## Resources

- ESP-IDF LCD Panel API: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/lcd.html
- ST7789 Datasheet: Standard ST7789 controller documentation
- T-Deck Schematic: Consult Lilygo documentation for exact pin assignments
- PS/2 Scancode Reference: Standard PC/AT keyboard scancodes

## Notes

- The current code uses RGB565 (16-bit) format which is compatible with ST7789
- The PS/2 emulation (`i8042.c/h`) should work without changes
- WiFi functionality can remain optional for remote access if desired
- Consider keeping WiFi keyboard as fallback during development
