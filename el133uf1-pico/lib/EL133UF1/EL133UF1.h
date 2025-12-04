/**
 * @file EL133UF1.h
 * @brief Driver for EL133UF1 13.3" Spectra 6 E-Ink Display
 * 
 * This driver is designed for the Pimoroni Pico Plus 2 W and supports
 * the 6-color (Black, White, Yellow, Red, Blue, Green) Spectra e-ink panel.
 * 
 * Resolution: 1600x1200 pixels
 * 
 * Based on the Pimoroni Inky Python driver.
 */

#ifndef EL133UF1_H
#define EL133UF1_H

#include <Arduino.h>
#include <SPI.h>

// Display resolution
#define EL133UF1_WIDTH  1600
#define EL133UF1_HEIGHT 1200

// Color definitions (3-bit values)
#define EL133UF1_BLACK   0
#define EL133UF1_WHITE   1
#define EL133UF1_YELLOW  2
#define EL133UF1_RED     3
#define EL133UF1_BLUE    5
#define EL133UF1_GREEN   6

// Chip select bit masks
#define CS0_SEL      0x01
#define CS1_SEL      0x02
#define CS_BOTH_SEL  (CS0_SEL | CS1_SEL)

// Command definitions
#define CMD_PSR              0x00  // Panel Setting
#define CMD_PWR              0x01  // Power Setting
#define CMD_POF              0x02  // Power Off
#define CMD_PON              0x04  // Power On
#define CMD_BTST_N           0x05  // Booster Soft Start (Negative)
#define CMD_BTST_P           0x06  // Booster Soft Start (Positive)
#define CMD_DTM              0x10  // Data Transmission
#define CMD_DRF              0x12  // Display Refresh
#define CMD_PLL              0x30  // PLL Control
#define CMD_TSC              0x40  // Temperature Sensor Calibration
#define CMD_TSE              0x41  // Temperature Sensor Enable
#define CMD_TSW              0x42  // Temperature Sensor Write
#define CMD_TSR              0x43  // Temperature Sensor Read
#define CMD_CDI              0x50  // VCOM and Data Interval
#define CMD_LPD              0x51  // Low Power Detection
#define CMD_TCON             0x60  // TCON Setting
#define CMD_TRES             0x61  // Resolution Setting
#define CMD_DAM              0x65  // Data Access Mode
#define CMD_REV              0x70  // Revision
#define CMD_FLG              0x71  // Status Flag
#define CMD_AMV              0x80  // Auto Measure Vcom
#define CMD_VV               0x81  // VV
#define CMD_VDCS             0x82  // VDC Setting
#define CMD_PTLW             0x83  // Partial Window
#define CMD_ANTM             0x74  // Anti-crosstalk Magic
#define CMD_AGID             0x86  // Auto Gate ID
#define CMD_PWS              0xE3  // Power Saving
#define CMD_TSSET            0xE5  // Temperature Sensor Setting
#define CMD_CMD66            0xF0  // Command 0xF0
#define CMD_CCSET            0xE0  // Cascade Setting
#define CMD_BOOST_VDDP_EN    0xB7  // Boost VDDP Enable
#define CMD_EN_BUF           0xB6  // Enable Buffer
#define CMD_TFT_VCOM_POWER   0xB1  // TFT VCOM Power
#define CMD_BUCK_BOOST_VDDN  0xB0  // Buck Boost VDDN

// Default SPI speed (can be overridden)
// CircuitPython reference uses 40 MHz successfully
#ifndef EL133UF1_SPI_SPEED
#define EL133UF1_SPI_SPEED 40000000  // 40 MHz
#endif

// Debug output (disable for faster operation)
#ifndef EL133UF1_DEBUG
#define EL133UF1_DEBUG 1  // Set to 0 to disable debug output
#endif

#if EL133UF1_DEBUG
#define EL133UF1_DBG(x) Serial.println(x)
#define EL133UF1_DBGF(...) Serial.printf(__VA_ARGS__)
#else
#define EL133UF1_DBG(x)
#define EL133UF1_DBGF(...)
#endif

// Buffer sizes
// Unpacked: 1 byte per pixel (1600 * 1200 = 1,920,000 bytes) - requires PSRAM
// Packed: 2 pixels per byte, split into two halves for the two controllers
// Each half: 1600 rows * 600 cols / 2 = 480,000 bytes
#define PACKED_HALF_SIZE ((1600 * 600) / 2)  // 480,000 bytes per half

/**
 * @class EL133UF1
 * @brief Driver class for the EL133UF1 13.3" Spectra 6 E-Ink Display
 */
class EL133UF1 {
public:
    /**
     * @brief Construct a new EL133UF1 driver
     * 
     * @param spi Pointer to SPI instance (default: &SPI)
     */
    EL133UF1(SPIClass* spi = &SPI);

    /**
     * @brief Initialize the display with specified pins
     * 
     * Note: For custom SPI pins, configure them BEFORE calling begin():
     *   SPI.setSCK(sckPin);
     *   SPI.setTX(mosiPin);
     * 
     * @param cs0Pin Chip select 0 pin
     * @param cs1Pin Chip select 1 pin
     * @param dcPin Data/Command pin
     * @param resetPin Reset pin
     * @param busyPin Busy pin
     * @return true if initialization successful
     */
    bool begin(int8_t cs0Pin, int8_t cs1Pin, int8_t dcPin, 
               int8_t resetPin, int8_t busyPin);

    /**
     * @brief Clear the display buffer
     * 
     * @param color Color to fill (default: WHITE)
     */
    void clear(uint8_t color = EL133UF1_WHITE);

    /**
     * @brief Set a single pixel
     * 
     * @param x X coordinate (0 to WIDTH-1)
     * @param y Y coordinate (0 to HEIGHT-1)
     * @param color Color value (0-6, see color definitions)
     */
    void setPixel(int16_t x, int16_t y, uint8_t color);

    /**
     * @brief Get the color of a pixel
     * 
     * @param x X coordinate
     * @param y Y coordinate
     * @return uint8_t Color value
     */
    uint8_t getPixel(int16_t x, int16_t y);

    /**
     * @brief Draw a horizontal line
     * 
     * @param x Start X coordinate
     * @param y Y coordinate
     * @param w Width
     * @param color Color value
     */
    void drawHLine(int16_t x, int16_t y, int16_t w, uint8_t color);

    /**
     * @brief Draw a vertical line
     * 
     * @param x X coordinate
     * @param y Start Y coordinate
     * @param h Height
     * @param color Color value
     */
    void drawVLine(int16_t x, int16_t y, int16_t h, uint8_t color);

    /**
     * @brief Draw a rectangle outline
     * 
     * @param x Top-left X coordinate
     * @param y Top-left Y coordinate
     * @param w Width
     * @param h Height
     * @param color Color value
     */
    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);

    /**
     * @brief Draw a filled rectangle
     * 
     * @param x Top-left X coordinate
     * @param y Top-left Y coordinate
     * @param w Width
     * @param h Height
     * @param color Color value
     */
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);

    /**
     * @brief Update the display with buffer contents (blocking)
     * 
     * @param skipInit If true, skip init sequence on subsequent updates (~1.5s faster)
     *                 Only use after first successful update in same session
     */
    void update(bool skipInit = false);

    /**
     * @brief Start async display update (non-blocking)
     * 
     * Returns immediately after sending data. Panel refreshes in background.
     * Use isUpdateComplete() to check status or waitForUpdate() to block.
     * 
     * @param skipInit If true, skip init sequence
     */
    void updateAsync(bool skipInit = false);

    /**
     * @brief Check if async update is complete
     * @return true if panel refresh is finished
     */
    bool isUpdateComplete();

    /**
     * @brief Wait for async update to complete (blocking)
     */
    void waitForUpdate();

    /**
     * @brief Check if display is busy with a refresh
     * @return true if display is busy
     */
    bool isBusy();

    /**
     * @brief Enable/disable pre-rotated buffer mode
     * 
     * When enabled, pixels are stored in panel-native format:
     * - Drawing is ~3x slower (more complex coordinate math)
     * - Update is ~300ms faster (no rotation needed)
     * 
     * Call before begin() or after clear() when changing modes.
     * 
     * @param enable True to enable pre-rotated mode
     */
    void setPreRotatedMode(bool enable);

    /**
     * @brief Check if using pre-rotated buffer mode
     */
    bool isPreRotatedMode() const { return _preRotatedMode; }

    /**
     * @brief Get display width
     * @return uint16_t Width in pixels
     */
    uint16_t width() const { return EL133UF1_WIDTH; }

    /**
     * @brief Get display height
     * @return uint16_t Height in pixels
     */
    uint16_t height() const { return EL133UF1_HEIGHT; }

    /**
     * @brief Set horizontal flip
     * @param flip Enable/disable horizontal flip
     */
    void setHFlip(bool flip) { _hFlip = flip; }

    /**
     * @brief Set vertical flip
     * @param flip Enable/disable vertical flip
     */
    void setVFlip(bool flip) { _vFlip = flip; }

    /**
     * @brief Get pointer to frame buffer
     * 
     * If using unpacked mode: 1600x1200 pixels, 1 byte per pixel
     * If using packed mode: Two half buffers, packed nibbles
     * 
     * @return uint8_t* Pointer to frame buffer (or left half if packed)
     */
    uint8_t* getBuffer() { return _buffer; }
    
    /**
     * @brief Check if using packed buffer mode
     * @return true if using packed buffers (no PSRAM)
     */
    bool isPackedMode() { return _packedMode; }

    /**
     * @brief Set an image from a raw buffer
     * 
     * @param data Pointer to image data (1 byte per pixel, values 0-6)
     * @param len Length of data (should be WIDTH * HEIGHT)
     */
    void setImage(const uint8_t* data, size_t len);

    /**
     * @brief Draw a character at specified position
     * 
     * @param x X coordinate (top-left of character)
     * @param y Y coordinate (top-left of character)
     * @param c Character to draw
     * @param color Foreground color
     * @param bg Background color (use same as color for transparent)
     * @param size Scale factor (1 = 8x8 pixels, 2 = 16x16, etc.)
     */
    void drawChar(int16_t x, int16_t y, char c, uint8_t color, uint8_t bg, uint8_t size = 1);

    /**
     * @brief Draw a string at specified position
     * 
     * @param x X coordinate
     * @param y Y coordinate  
     * @param text String to draw
     * @param color Foreground color
     * @param bg Background color
     * @param size Scale factor
     */
    void drawText(int16_t x, int16_t y, const char* text, uint8_t color, uint8_t bg, uint8_t size = 1);

private:
    SPIClass* _spi;
    SPISettings _spiSettings;
    
    int8_t _cs0Pin;
    int8_t _cs1Pin;
    int8_t _dcPin;
    int8_t _resetPin;
    int8_t _busyPin;
    
    bool _hFlip;
    bool _vFlip;
    bool _initialized;
    bool _packedMode;       // True if using packed buffers (no PSRAM)
    bool _preRotatedMode;   // True if buffer stores pre-rotated data
    bool _initDone;         // True if init sequence has run at least once
    bool _asyncInProgress;  // True if async update is running
    
    // Frame buffer(s)
    // Unpacked mode: single 1.92MB buffer (requires PSRAM)
    // Packed mode: two 480KB buffers (works without PSRAM)
    uint8_t* _buffer;       // Main buffer (or left half in packed mode)
    uint8_t* _bufferRight;  // Right half (only used in packed mode)
    
    /**
     * @brief Perform hardware reset
     */
    void _reset();

    /**
     * @brief Wait for busy pin to go low
     * 
     * @param timeoutMs Timeout in milliseconds
     * @return true if ready, false if timeout
     */
    bool _busyWait(uint32_t timeoutMs = 40000);

    /**
     * @brief Send a command to the display
     * 
     * @param cmd Command byte
     * @param csSel Chip select mask (CS0_SEL, CS1_SEL, or CS_BOTH_SEL)
     * @param data Optional data bytes
     * @param len Length of data
     */
    void _sendCommand(uint8_t cmd, uint8_t csSel, const uint8_t* data = nullptr, size_t len = 0);

    /**
     * @brief Send initialization sequence
     */
    void _initSequence();

    /**
     * @brief Prepare and send buffer data to display
     */
    void _sendBuffer();
};

#endif // EL133UF1_H
