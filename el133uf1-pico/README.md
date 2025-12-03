# EL133UF1 13.3" Spectra 6 E-Ink Display Driver

A PlatformIO/Arduino-Pico driver for the EL133UF1 13.3" Spectra 6 e-ink panel, designed for the **Pimoroni Pico Plus 2 W**.

## Features

- **Resolution**: 1600 x 1200 pixels
- **6 Colors**: Black, White, Red, Yellow, Green, Blue
- **Full frame buffer**: Easy pixel manipulation
- **Simple API**: Clear, setPixel, fillRect, drawRect, and more
- **Optimized for RP2350**: Takes advantage of the Pico Plus 2 W's 8MB PSRAM

## Hardware Requirements

- **Pimoroni Pico Plus 2 W** (RP2350 with 8MB PSRAM)
- **EL133UF1 13.3" Spectra 6 E-Ink Display Panel**
- Appropriate flex cable and breakout board for the panel

## Wiring

Default pin configuration for **Pimoroni Inky Impression 13.3"** connected to **Pico Plus 2 W**:

| Display Pin | Pico Plus 2 W Pin | Description |
|-------------|-------------------|-------------|
| MOSI        | GP11              | SPI1 TX (MOSI) |
| SCLK        | GP10              | SPI1 SCK |
| CS0         | GP26              | Chip Select 0 (left half) |
| CS1         | GP16              | Chip Select 1 (right half) |
| DC          | GP22              | Data/Command |
| RESET       | GP27              | Reset |
| BUSY        | GP17              | Busy signal |
| GND         | GND               | Ground |
| 3.3V        | 3V3               | Power (3.3V) |

> **Note**: The EL133UF1 uses two chip select lines because the panel is driven by two controllers (left and right halves). Uses **SPI1** (not SPI0).

## Installation

### PlatformIO (Recommended)

1. Clone this repository or copy it to your project
2. Open in VS Code with PlatformIO extension
3. Build and upload

```bash
# Build
pio run

# Upload
pio run --target upload

# Monitor serial output
pio device monitor
```

### Manual Installation

Copy the `lib/EL133UF1` folder to your Arduino libraries folder or PlatformIO lib folder.

## Usage

### Basic Example

```cpp
#include <Arduino.h>
#include "EL133UF1.h"

// Pin definitions for Pimoroni Pico Plus 2 W + Inky Impression 13.3"
#define PIN_SPI_SCK   10    // SPI1 SCK
#define PIN_SPI_MOSI  11    // SPI1 MOSI
#define PIN_CS0       26    // Left half
#define PIN_CS1       16    // Right half
#define PIN_DC        22
#define PIN_RESET     27
#define PIN_BUSY      17

EL133UF1 display(&SPI1);  // Use SPI1

void setup() {
    Serial.begin(115200);
    
    // Configure SPI1 pins BEFORE initializing display
    // (arduino-pico requires pin configuration before SPI.begin())
    SPI1.setSCK(PIN_SPI_SCK);
    SPI1.setTX(PIN_SPI_MOSI);
    
    // Initialize display
    if (!display.begin(PIN_CS0, PIN_CS1, PIN_DC, PIN_RESET, PIN_BUSY)) {
        Serial.println("Display init failed!");
        while (1) delay(1000);
    }
    
    // Clear to white
    display.clear(EL133UF1_WHITE);
    
    // Draw a red rectangle
    display.fillRect(100, 100, 200, 150, EL133UF1_RED);
    
    // Draw a black border
    display.drawRect(0, 0, 1600, 1200, EL133UF1_BLACK);
    
    // Update the display (takes ~30 seconds)
    display.update();
}

void loop() {
    delay(10000);
}
```

### Color Constants

```cpp
EL133UF1_BLACK   // 0
EL133UF1_WHITE   // 1
EL133UF1_YELLOW  // 2
EL133UF1_RED     // 3
EL133UF1_BLUE    // 5
EL133UF1_GREEN   // 6
```

### API Reference

#### Initialization

```cpp
// Create display instance (use SPI1 for GP10/GP11)
EL133UF1 display(&SPI1);

// Configure SPI pins before calling begin() (arduino-pico requirement)
SPI1.setSCK(10);   // GP10
SPI1.setTX(11);    // GP11

// Initialize with pin configuration
bool begin(int8_t cs0Pin, int8_t cs1Pin, int8_t dcPin, 
           int8_t resetPin, int8_t busyPin);
```

#### Drawing Functions

```cpp
// Clear display buffer to a color
void clear(uint8_t color = EL133UF1_WHITE);

// Set a single pixel
void setPixel(int16_t x, int16_t y, uint8_t color);

// Get pixel color
uint8_t getPixel(int16_t x, int16_t y);

// Draw horizontal line
void drawHLine(int16_t x, int16_t y, int16_t w, uint8_t color);

// Draw vertical line
void drawVLine(int16_t x, int16_t y, int16_t h, uint8_t color);

// Draw rectangle outline
void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);

// Draw filled rectangle
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);

// Set image from raw buffer (1 byte per pixel)
void setImage(const uint8_t* data, size_t len);
```

#### Display Control

```cpp
// Update display with buffer contents (blocking, ~30 seconds)
void update();

// Check if display is busy
bool isBusy();

// Get display dimensions
uint16_t width();   // Returns 1600
uint16_t height();  // Returns 1200

// Set flip modes
void setHFlip(bool flip);
void setVFlip(bool flip);

// Get raw buffer pointer for direct manipulation
uint8_t* getBuffer();
```

## Memory Usage

The frame buffer requires approximately **1.92 MB** of RAM (1600 × 1200 bytes). The Pico Plus 2 W has 8 MB of PSRAM, making this feasible.

## Refresh Time

E-ink displays have slow refresh times. The EL133UF1 takes approximately **30 seconds** for a full refresh. The `update()` function blocks until the refresh is complete.

## Technical Details

### Display Architecture

The EL133UF1 panel is controlled by two separate driver ICs:
- **CS0**: Controls the left half of the display (columns 0-599 after rotation)
- **CS1**: Controls the right half of the display (columns 600-1199 after rotation)

Data is sent to each controller separately, with pixels packed as nibbles (4 bits per pixel, 2 pixels per byte).

### SPI Communication

- **Speed**: 10 MHz (configurable via `EL133UF1_SPI_SPEED`)
- **Mode**: SPI Mode 0 (CPOL=0, CPHA=0)
- **Bit Order**: MSB First

### Command Sequence

1. Hardware reset
2. Initialization sequence (panel settings, power, timing)
3. Data transmission to both controllers
4. Power on
5. Display refresh
6. Power off

## Troubleshooting

### Display Not Responding
- Check all wiring connections
- Verify 3.3V power supply is adequate (e-ink displays can draw significant current during refresh)
- Check that the busy pin is correctly connected

### Garbled Display
- Verify SPI connections (MOSI, SCK)
- Check CS0 and CS1 are not swapped
- Ensure DC pin is correctly wired

### Slow Updates
- E-ink refresh is inherently slow (~30 seconds for this panel)
- This is normal behavior for Spectra 6 technology

## License

MIT License - See LICENSE file for details.

## Acknowledgments

Based on the [Pimoroni Inky Python library](https://github.com/pimoroni/inky) - specifically the `inky_el133uf1.py` driver.

## Contributing

Contributions are welcome! Please open an issue or pull request on GitHub.
