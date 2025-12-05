/**
 * @file EL133UF1.cpp
 * @brief Implementation of EL133UF1 13.3" Spectra 6 E-Ink Display Driver
 */

#include "EL133UF1.h"
#include "hardware/dma.h"
#include "hardware/sync.h"

// Simple 8x8 bitmap font (ASCII 32-127)
// Each character is 8 bytes, each byte is one row (MSB = leftmost pixel)
static const uint8_t font8x8[][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 32 (space)
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // 33 !
    {0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00}, // 34 "
    {0x36,0x36,0x7F,0x36,0x7F,0x36,0x36,0x00}, // 35 #
    {0x0C,0x3E,0x03,0x1E,0x30,0x1F,0x0C,0x00}, // 36 $
    {0x00,0x63,0x33,0x18,0x0C,0x66,0x63,0x00}, // 37 %
    {0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00}, // 38 &
    {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00}, // 39 '
    {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00}, // 40 (
    {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00}, // 41 )
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // 42 *
    {0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00}, // 43 +
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x06}, // 44 ,
    {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00}, // 45 -
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00}, // 46 .
    {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00}, // 47 /
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, // 48 0
    {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, // 49 1
    {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, // 50 2
    {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, // 51 3
    {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, // 52 4
    {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, // 53 5
    {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, // 54 6
    {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, // 55 7
    {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, // 56 8
    {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, // 57 9
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00}, // 58 :
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x06}, // 59 ;
    {0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x00}, // 60 <
    {0x00,0x00,0x3F,0x00,0x00,0x3F,0x00,0x00}, // 61 =
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, // 62 >
    {0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00}, // 63 ?
    {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00}, // 64 @
    {0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00}, // 65 A
    {0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00}, // 66 B
    {0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00}, // 67 C
    {0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00}, // 68 D
    {0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00}, // 69 E
    {0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00}, // 70 F
    {0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00}, // 71 G
    {0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00}, // 72 H
    {0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // 73 I
    {0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00}, // 74 J
    {0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00}, // 75 K
    {0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00}, // 76 L
    {0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00}, // 77 M
    {0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00}, // 78 N
    {0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00}, // 79 O
    {0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00}, // 80 P
    {0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00}, // 81 Q
    {0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00}, // 82 R
    {0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00}, // 83 S
    {0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // 84 T
    {0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00}, // 85 U
    {0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00}, // 86 V
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, // 87 W
    {0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00}, // 88 X
    {0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00}, // 89 Y
    {0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00}, // 90 Z
    {0x1E,0x06,0x06,0x06,0x06,0x06,0x1E,0x00}, // 91 [
    {0x03,0x06,0x0C,0x18,0x30,0x60,0x40,0x00}, // 92 backslash
    {0x1E,0x18,0x18,0x18,0x18,0x18,0x1E,0x00}, // 93 ]
    {0x08,0x1C,0x36,0x63,0x00,0x00,0x00,0x00}, // 94 ^
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, // 95 _
    {0x0C,0x0C,0x18,0x00,0x00,0x00,0x00,0x00}, // 96 `
    {0x00,0x00,0x1E,0x30,0x3E,0x33,0x6E,0x00}, // 97 a
    {0x07,0x06,0x06,0x3E,0x66,0x66,0x3B,0x00}, // 98 b
    {0x00,0x00,0x1E,0x33,0x03,0x33,0x1E,0x00}, // 99 c
    {0x38,0x30,0x30,0x3E,0x33,0x33,0x6E,0x00}, // 100 d
    {0x00,0x00,0x1E,0x33,0x3F,0x03,0x1E,0x00}, // 101 e
    {0x1C,0x36,0x06,0x0F,0x06,0x06,0x0F,0x00}, // 102 f
    {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x1F}, // 103 g
    {0x07,0x06,0x36,0x6E,0x66,0x66,0x67,0x00}, // 104 h
    {0x0C,0x00,0x0E,0x0C,0x0C,0x0C,0x1E,0x00}, // 105 i
    {0x30,0x00,0x30,0x30,0x30,0x33,0x33,0x1E}, // 106 j
    {0x07,0x06,0x66,0x36,0x1E,0x36,0x67,0x00}, // 107 k
    {0x0E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // 108 l
    {0x00,0x00,0x33,0x7F,0x7F,0x6B,0x63,0x00}, // 109 m
    {0x00,0x00,0x1F,0x33,0x33,0x33,0x33,0x00}, // 110 n
    {0x00,0x00,0x1E,0x33,0x33,0x33,0x1E,0x00}, // 111 o
    {0x00,0x00,0x3B,0x66,0x66,0x3E,0x06,0x0F}, // 112 p
    {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x78}, // 113 q
    {0x00,0x00,0x3B,0x6E,0x66,0x06,0x0F,0x00}, // 114 r
    {0x00,0x00,0x3E,0x03,0x1E,0x30,0x1F,0x00}, // 115 s
    {0x08,0x0C,0x3E,0x0C,0x0C,0x2C,0x18,0x00}, // 116 t
    {0x00,0x00,0x33,0x33,0x33,0x33,0x6E,0x00}, // 117 u
    {0x00,0x00,0x33,0x33,0x33,0x1E,0x0C,0x00}, // 118 v
    {0x00,0x00,0x63,0x6B,0x7F,0x7F,0x36,0x00}, // 119 w
    {0x00,0x00,0x63,0x36,0x1C,0x36,0x63,0x00}, // 120 x
    {0x00,0x00,0x33,0x33,0x33,0x3E,0x30,0x1F}, // 121 y
    {0x00,0x00,0x3F,0x19,0x0C,0x26,0x3F,0x00}, // 122 z
    {0x38,0x0C,0x0C,0x07,0x0C,0x0C,0x38,0x00}, // 123 {
    {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, // 124 |
    {0x07,0x0C,0x0C,0x38,0x0C,0x0C,0x07,0x00}, // 125 }
    {0x6E,0x3B,0x00,0x00,0x00,0x00,0x00,0x00}, // 126 ~
};

EL133UF1::EL133UF1(SPIClass* spi) :
    _spi(spi),
    _spiSettings(EL133UF1_SPI_SPEED, MSBFIRST, SPI_MODE0),
    _cs0Pin(-1),
    _cs1Pin(-1),
    _dcPin(-1),
    _resetPin(-1),
    _busyPin(-1),
    _hFlip(false),
    _vFlip(false),
    _initialized(false),
    _packedMode(false),
    _preRotatedMode(false),
    _initDone(false),
    _asyncInProgress(false),
    _buffer(nullptr),
    _bufferRight(nullptr)
{
}

bool EL133UF1::begin(int8_t cs0Pin, int8_t cs1Pin, int8_t dcPin, 
                     int8_t resetPin, int8_t busyPin) {
    _cs0Pin = cs0Pin;
    _cs1Pin = cs1Pin;
    _dcPin = dcPin;
    _resetPin = resetPin;
    _busyPin = busyPin;

    // Allocate frame buffer in PSRAM (1.92MB)
    _buffer = (uint8_t*)pmalloc(EL133UF1_WIDTH * EL133UF1_HEIGHT);
    _packedMode = false;
    _bufferRight = nullptr;
    
    if (_buffer == nullptr) {
        Serial.println("EL133UF1: PSRAM allocation failed!");
        return false;
    }
    
    // Clear to white
    memset(_buffer, EL133UF1_WHITE, EL133UF1_WIDTH * EL133UF1_HEIGHT);

    // Configure GPIO pins
    pinMode(_cs0Pin, OUTPUT);
    pinMode(_cs1Pin, OUTPUT);
    pinMode(_dcPin, OUTPUT);
    pinMode(_resetPin, OUTPUT);
    pinMode(_busyPin, INPUT_PULLUP);

    digitalWrite(_cs0Pin, HIGH);
    digitalWrite(_cs1Pin, HIGH);
    digitalWrite(_dcPin, LOW);
    digitalWrite(_resetPin, HIGH);

    // Initialize SPI
    _spi->begin();

    // Hardware reset and init sequence
    _reset();
    _busyWait(1000);
    _initSequence();

    _initialized = true;
    return true;
}

void EL133UF1::_reset() {
    // Reset sequence from Python reference (verified working)
    // Pull reset LOW, wait 30ms, then HIGH, wait 30ms
    digitalWrite(_resetPin, LOW);
    delay(30);
    digitalWrite(_resetPin, HIGH);
    delay(30);
}

bool EL133UF1::_busyWait(uint32_t timeoutMs) {
    // Two-phase wait: wait for busy to assert (LOW) then deassert (HIGH)
    uint32_t startTime = millis();
    
    // Phase 1: Wait for busy to go LOW
    while (digitalRead(_busyPin) == HIGH) {
        if (millis() - startTime > timeoutMs) return true;
        delay(1);
    }
    
    // Phase 2: Wait for busy to go HIGH
    while (digitalRead(_busyPin) == LOW) {
        if (millis() - startTime > timeoutMs) return false;
        delay(10);
    }
    
    return true;
}

void EL133UF1::_sendCommand(uint8_t cmd, uint8_t csSel, const uint8_t* data, size_t len) {
    // Match CircuitPython exactly
    
    // Assert chip select(s) - active LOW
    if (csSel & CS0_SEL) {
        digitalWrite(_cs0Pin, LOW);
    }
    if (csSel & CS1_SEL) {
        digitalWrite(_cs1Pin, LOW);
    }

    // Send command (DC low = command mode)
    // 90ms delay - tested working (CircuitPython uses 100ms)
    digitalWrite(_dcPin, LOW);
    delay(90);
    
    _spi->beginTransaction(_spiSettings);
    _spi->transfer(cmd);
    _spi->endTransaction();

    // Send data if present (DC high = data mode)
    // CircuitPython: if data: dc.value = True; spi.write(bytes(data))
    if (data != nullptr && len > 0) {
        digitalWrite(_dcPin, HIGH);
        
        _spi->beginTransaction(_spiSettings);
        // For large transfers, send in chunks
        const size_t chunkSize = 4096;
        for (size_t offset = 0; offset < len; offset += chunkSize) {
            size_t remaining = len - offset;
            size_t toSend = (remaining < chunkSize) ? remaining : chunkSize;
            // Use transfer with nullptr for write-only
            _spi->transfer(data + offset, nullptr, toSend);
        }
        _spi->endTransaction();
    }

    // CircuitPython finally block: dc.value = False; cs0.value = True; cs1.value = True
    digitalWrite(_dcPin, LOW);
    digitalWrite(_cs0Pin, HIGH);
    digitalWrite(_cs1Pin, HIGH);
}

void EL133UF1::_initSequence() {
    // Initialization sequence from working CircuitPython reference
    // 17 commands total, each with 90ms DC setup delay = ~1.5s
    
    // ANTM - Anti-crosstalk magic (CS0 only, must be first)
    const uint8_t antm[] = {0xC0, 0x1C, 0x1C, 0xCC, 0xCC, 0xCC, 0x15, 0x15, 0x55};
    _sendCommand(CMD_ANTM, CS0_SEL, antm, sizeof(antm));

    const uint8_t cmd66[] = {0x49, 0x55, 0x13, 0x5D, 0x05, 0x10};
    _sendCommand(CMD_CMD66, CS_BOTH_SEL, cmd66, sizeof(cmd66));

    const uint8_t psr[] = {0xDF, 0x69};
    _sendCommand(CMD_PSR, CS_BOTH_SEL, psr, sizeof(psr));

    const uint8_t pll[] = {0x08};
    _sendCommand(CMD_PLL, CS_BOTH_SEL, pll, sizeof(pll));

    const uint8_t cdi[] = {0xF7};
    _sendCommand(CMD_CDI, CS_BOTH_SEL, cdi, sizeof(cdi));

    const uint8_t tcon[] = {0x03, 0x03};
    _sendCommand(CMD_TCON, CS_BOTH_SEL, tcon, sizeof(tcon));

    const uint8_t agid[] = {0x10};
    _sendCommand(CMD_AGID, CS_BOTH_SEL, agid, sizeof(agid));

    const uint8_t pws[] = {0x22};
    _sendCommand(CMD_PWS, CS_BOTH_SEL, pws, sizeof(pws));

    const uint8_t ccset[] = {0x01};
    _sendCommand(CMD_CCSET, CS_BOTH_SEL, ccset, sizeof(ccset));

    // Resolution: 1200 x 800 per half (0x04B0 x 0x0320)
    const uint8_t tres[] = {0x04, 0xB0, 0x03, 0x20};
    _sendCommand(CMD_TRES, CS_BOTH_SEL, tres, sizeof(tres));

    // Power configuration
    const uint8_t pwr[] = {0x0F, 0x00, 0x28, 0x2C, 0x28, 0x38};
    _sendCommand(CMD_PWR, CS_BOTH_SEL, pwr, sizeof(pwr));

    const uint8_t en_buf[] = {0x07};
    _sendCommand(CMD_EN_BUF, CS_BOTH_SEL, en_buf, sizeof(en_buf));

    const uint8_t btst_p[] = {0xD8, 0x18};
    _sendCommand(CMD_BTST_P, CS_BOTH_SEL, btst_p, sizeof(btst_p));

    const uint8_t boost_vddp[] = {0x01};
    _sendCommand(CMD_BOOST_VDDP_EN, CS_BOTH_SEL, boost_vddp, sizeof(boost_vddp));

    const uint8_t btst_n[] = {0xD8, 0x18};
    _sendCommand(CMD_BTST_N, CS_BOTH_SEL, btst_n, sizeof(btst_n));

    const uint8_t buck_boost[] = {0x01};
    _sendCommand(CMD_BUCK_BOOST_VDDN, CS_BOTH_SEL, buck_boost, sizeof(buck_boost));

    const uint8_t vcom_power[] = {0x02};
    _sendCommand(CMD_TFT_VCOM_POWER, CS_BOTH_SEL, vcom_power, sizeof(vcom_power));
}

void EL133UF1::clear(uint8_t color) {
    if (_buffer == nullptr) return;
    
    uint8_t packedColor = ((color & 0x07) << 4) | (color & 0x07);
    
    if (_packedMode || _preRotatedMode) {
        // Packed/pre-rotated: each byte has two pixels
        memset(_buffer, packedColor, PACKED_HALF_SIZE);
        if (_bufferRight) {
            memset(_bufferRight, packedColor, PACKED_HALF_SIZE);
        }
    } else {
        // Unpacked mode: 1 byte per pixel
        memset(_buffer, color & 0x07, EL133UF1_WIDTH * EL133UF1_HEIGHT);
    }
}

void EL133UF1::setPreRotatedMode(bool enable) {
    if (enable == _preRotatedMode) return;
    
    _preRotatedMode = enable;
    
    if (enable && !_packedMode) {
        // Switching to pre-rotated mode: reallocate buffers
        // Free old unpacked buffer
        if (_buffer) {
            free(_buffer);
            _buffer = nullptr;
        }
        
        // Allocate two packed buffers (480KB each)
        _buffer = (uint8_t*)pmalloc(PACKED_HALF_SIZE);
        _bufferRight = (uint8_t*)pmalloc(PACKED_HALF_SIZE);
        
        if (_buffer && _bufferRight) {
            // Clear to white
            memset(_buffer, 0x11, PACKED_HALF_SIZE);
            memset(_bufferRight, 0x11, PACKED_HALF_SIZE);
        }
    } else if (!enable && !_packedMode) {
        // Switching back to unpacked mode
        if (_buffer) free(_buffer);
        if (_bufferRight) free(_bufferRight);
        _bufferRight = nullptr;
        
        _buffer = (uint8_t*)pmalloc(EL133UF1_WIDTH * EL133UF1_HEIGHT);
        if (_buffer) {
            memset(_buffer, EL133UF1_WHITE, EL133UF1_WIDTH * EL133UF1_HEIGHT);
        }
    }
}

void EL133UF1::setPixel(int16_t x, int16_t y, uint8_t color) {
    if (_buffer == nullptr) return;
    if (x < 0 || x >= EL133UF1_WIDTH || y < 0 || y >= EL133UF1_HEIGHT) return;
    
    if (_packedMode || _preRotatedMode) {
        // Pre-rotated/packed mode: write directly to panel format
        // Transform: (x,y) -> panel row = 1599-x, panel col = y
        // Then split at col 600
        int16_t panelRow = 1599 - x;
        int16_t panelCol = y;
        
        uint8_t* buf;
        int16_t bufCol;
        if (panelCol < 600) {
            buf = _buffer;
            bufCol = panelCol;
        } else {
            buf = _bufferRight;
            bufCol = panelCol - 600;
        }
        
        if (buf == nullptr) return;
        
        size_t byteIdx = panelRow * 300 + (bufCol / 2);
        uint8_t c = color & 0x07;
        
        if (bufCol % 2 == 0) {
            buf[byteIdx] = (buf[byteIdx] & 0x0F) | (c << 4);
        } else {
            buf[byteIdx] = (buf[byteIdx] & 0xF0) | c;
        }
    } else {
        // Unpacked mode: direct indexing
        _buffer[y * EL133UF1_WIDTH + x] = color & 0x07;
    }
}

uint8_t EL133UF1::getPixel(int16_t x, int16_t y) {
    if (_buffer == nullptr) return 0;
    if (x < 0 || x >= EL133UF1_WIDTH || y < 0 || y >= EL133UF1_HEIGHT) return 0;
    
    return _buffer[y * EL133UF1_WIDTH + x];
}

void EL133UF1::drawHLine(int16_t x, int16_t y, int16_t w, uint8_t color) {
    if (y < 0 || y >= EL133UF1_HEIGHT) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > EL133UF1_WIDTH) { w = EL133UF1_WIDTH - x; }
    if (w <= 0) return;
    
    if (!_packedMode && !_preRotatedMode && _buffer) {
        // Fast path: direct memset for unpacked mode
        memset(_buffer + y * EL133UF1_WIDTH + x, color & 0x07, w);
    } else {
        // Use setPixel for packed/pre-rotated modes
        for (int16_t i = 0; i < w; i++) {
            setPixel(x + i, y, color);
        }
    }
}

void EL133UF1::drawVLine(int16_t x, int16_t y, int16_t h, uint8_t color) {
    if (x < 0 || x >= EL133UF1_WIDTH) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > EL133UF1_HEIGHT) { h = EL133UF1_HEIGHT - y; }
    if (h <= 0) return;
    
    uint8_t c = color & 0x07;
    if (!_packedMode && !_preRotatedMode && _buffer) {
        // Fast path: direct buffer access for unpacked mode
        uint8_t* ptr = _buffer + y * EL133UF1_WIDTH + x;
        for (int16_t i = 0; i < h; i++) {
            *ptr = c;
            ptr += EL133UF1_WIDTH;
        }
    } else {
        // Use setPixel for packed/pre-rotated modes
        for (int16_t i = 0; i < h; i++) {
            setPixel(x, y + i, color);
        }
    }
}

void EL133UF1::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color) {
    drawHLine(x, y, w, color);
    drawHLine(x, y + h - 1, w, color);
    drawVLine(x, y, h, color);
    drawVLine(x + w - 1, y, h, color);
}

void EL133UF1::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color) {
    // Clip to display bounds
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > EL133UF1_WIDTH) { w = EL133UF1_WIDTH - x; }
    if (y + h > EL133UF1_HEIGHT) { h = EL133UF1_HEIGHT - y; }
    if (w <= 0 || h <= 0) return;
    
    if (!_packedMode && !_preRotatedMode && _buffer) {
        // Fast path: direct memset for each row in unpacked mode
        uint8_t c = color & 0x07;
        uint8_t* ptr = _buffer + y * EL133UF1_WIDTH + x;
        for (int16_t i = 0; i < h; i++) {
            memset(ptr, c, w);
            ptr += EL133UF1_WIDTH;
        }
    } else {
        // Slow path for packed mode
        for (int16_t i = 0; i < h; i++) {
            drawHLine(x, y + i, w, color);
        }
    }
}

void EL133UF1::setImage(const uint8_t* data, size_t len) {
    if (_buffer == nullptr || data == nullptr) return;
    
    size_t copyLen = len;
    if (copyLen > EL133UF1_WIDTH * EL133UF1_HEIGHT) {
        copyLen = EL133UF1_WIDTH * EL133UF1_HEIGHT;
    }
    
    memcpy(_buffer, data, copyLen);
}

void EL133UF1::drawChar(int16_t x, int16_t y, char c, uint8_t color, uint8_t bg, uint8_t size) {
    // Only support printable ASCII (32-126)
    if (c < 32 || c > 126) c = '?';
    
    const uint8_t* charData = font8x8[c - 32];
    uint8_t fgc = color & 0x07;
    uint8_t bgc = bg & 0x07;
    
    // Fast path for size=1 in unpacked mode with no clipping needed
    if (size == 1 && !_packedMode && _buffer &&
        x >= 0 && x + 8 <= EL133UF1_WIDTH && 
        y >= 0 && y + 8 <= EL133UF1_HEIGHT) {
        
        uint8_t* ptr = _buffer + y * EL133UF1_WIDTH + x;
        for (int8_t row = 0; row < 8; row++) {
            uint8_t rowData = charData[row];
            // Flipped bit order to compensate for rotation transform
            ptr[0] = (rowData & 0x01) ? fgc : bgc;
            ptr[1] = (rowData & 0x02) ? fgc : bgc;
            ptr[2] = (rowData & 0x04) ? fgc : bgc;
            ptr[3] = (rowData & 0x08) ? fgc : bgc;
            ptr[4] = (rowData & 0x10) ? fgc : bgc;
            ptr[5] = (rowData & 0x20) ? fgc : bgc;
            ptr[6] = (rowData & 0x40) ? fgc : bgc;
            ptr[7] = (rowData & 0x80) ? fgc : bgc;
            ptr += EL133UF1_WIDTH;
        }
        return;
    }
    
    // General path for scaled text or edge cases
    for (int8_t row = 0; row < 8; row++) {
        uint8_t rowData = charData[row];
        for (int8_t col = 0; col < 8; col++) {
            // Flipped bit order: use col instead of (7 - col)
            bool pixel = (rowData >> col) & 0x01;
            uint8_t pixelColor = pixel ? fgc : bgc;
            
            if (size == 1) {
                setPixel(x + col, y + row, pixelColor);
            } else {
                fillRect(x + col * size, y + row * size, size, size, pixelColor);
            }
        }
    }
}

void EL133UF1::drawText(int16_t x, int16_t y, const char* text, uint8_t color, uint8_t bg, uint8_t size) {
    int16_t cursorX = x;
    int16_t charWidth = 8 * size;
    
    while (*text) {
        if (*text == '\n') {
            cursorX = x;
            y += 8 * size;
        } else {
            drawChar(cursorX, y, *text, color, bg, size);
            cursorX += charWidth;
        }
        text++;
    }
}

bool EL133UF1::isBusy() {
    return digitalRead(_busyPin) == LOW;
}

// ============================================================================
// Optimized buffer rotation with SIMD-style packing and DMA
// ============================================================================

// DMA channel for async memory transfers (-1 = not initialized)
static int _dmaChannel = -1;

// Initialize DMA channel for memory transfers
static void initDMA() {
    if (_dmaChannel < 0) {
        _dmaChannel = dma_claim_unused_channel(false);
        if (_dmaChannel >= 0) {
            Serial.printf("    DMA channel:    %d claimed\n", _dmaChannel);
        }
    }
}

// Start async DMA memcpy (returns immediately)
static void dmaMemcpyStart(void* dst, const void* src, size_t size) {
    if (_dmaChannel < 0) {
        // Fallback to regular memcpy
        memcpy(dst, src, size);
        return;
    }
    
    dma_channel_config c = dma_channel_get_default_config(_dmaChannel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);  // 32-bit transfers
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    
    dma_channel_configure(
        _dmaChannel,
        &c,
        dst,           // Write address
        src,           // Read address
        size / 4,      // Transfer count (32-bit words)
        true           // Start immediately
    );
}

// Wait for DMA to complete
static void dmaMemcpyWait() {
    if (_dmaChannel >= 0) {
        dma_channel_wait_for_finish_blocking(_dmaChannel);
    }
}

// Synchronous DMA memcpy
static void dmaMemcpy(void* dst, const void* src, size_t size) {
    dmaMemcpyStart(dst, src, size);
    dmaMemcpyWait();
}

// Process 8 pixels into 4 bytes using 32-bit operations
static inline void packPixels8_SIMD(const uint8_t* src, size_t stride, uint8_t* dst) {
    // Pack first 4 pixels
    uint32_t p0 = src[0] & 0x07;
    uint32_t p1 = src[stride] & 0x07;
    uint32_t p2 = src[stride * 2] & 0x07;
    uint32_t p3 = src[stride * 3] & 0x07;
    
    // Pack second 4 pixels
    uint32_t p4 = src[stride * 4] & 0x07;
    uint32_t p5 = src[stride * 5] & 0x07;
    uint32_t p6 = src[stride * 6] & 0x07;
    uint32_t p7 = src[stride * 7] & 0x07;
    
    // Combine into single 32-bit word for one write
    uint32_t packed = ((p0 << 4) | p1) |
                     (((p2 << 4) | p3) << 8) |
                     (((p4 << 4) | p5) << 16) |
                     (((p6 << 4) | p7) << 24);
    *((uint32_t*)dst) = packed;
}

// Process a strip from SRAM to output buffer (PSRAM)
static void processStrip(const uint8_t* sramStrip, uint8_t* outBuf, 
                         int stripStart, int stripH, int outRowOffset) {
    for (int srcCol = 1599; srcCol >= 0; srcCol--) {
        int outRow = 1599 - srcCol;
        uint8_t* outPtr = outBuf + outRow * 300 + (stripStart / 2) - outRowOffset;
        const uint8_t* sramCol = sramStrip + srcCol;
        
        // Process 8 rows at a time with SIMD
        int r = 0;
        for (; r + 8 <= stripH; r += 8) {
            packPixels8_SIMD(sramCol + r * EL133UF1_WIDTH, EL133UF1_WIDTH, outPtr);
            outPtr += 4;
        }
        // Handle remaining rows
        for (; r + 2 <= stripH; r += 2) {
            uint8_t p0 = sramCol[r * EL133UF1_WIDTH] & 0x07;
            uint8_t p1 = sramCol[(r + 1) * EL133UF1_WIDTH] & 0x07;
            *outPtr++ = (p0 << 4) | p1;
        }
    }
}

void EL133UF1::_sendBuffer() {
    if (_buffer == nullptr) return;

    uint32_t stepStart;

    // Pre-rotated mode or packed mode: buffers are already in panel format
    if (_preRotatedMode || _packedMode) {
        stepStart = millis();
        _sendCommand(CMD_DTM, CS0_SEL, _buffer, PACKED_HALF_SIZE);
        _sendCommand(CMD_DTM, CS1_SEL, _bufferRight, PACKED_HALF_SIZE);
        Serial.printf("    SPI transmit:   %4lu ms (pre-rotated)\n", millis() - stepStart);
        return;
    }

    const size_t SEND_HALF_SIZE = PACKED_HALF_SIZE;
    
    stepStart = millis();
    uint8_t* bufA = (uint8_t*)pmalloc(SEND_HALF_SIZE);
    uint8_t* bufB = (uint8_t*)pmalloc(SEND_HALF_SIZE);
    
    if (bufA == nullptr || bufB == nullptr) {
        Serial.println("EL133UF1: Failed to allocate send buffers in PSRAM");
        if (bufA) free(bufA);
        if (bufB) free(bufB);
        return;
    }
    Serial.printf("    Buffer alloc:   %4lu ms\n", millis() - stepStart);

    // Initialize DMA for async memory transfers
    initDMA();
    
    // Debug: show available heap
    Serial.printf("    Free heap:      %lu KB\n", rp2040.getFreeHeap() / 1024);
    
    // SRAM-accelerated rotation with SIMD packing and DMA
    // Strategy: Double-buffered strips with DMA prefetch
    //   - DMA fetches next strip while CPU processes current strip
    //   - Overlaps memory latency with computation
    
    int STRIP_ROWS = 0;
    uint8_t* sramStrip[2] = {nullptr, nullptr};  // Double buffer
    
    // Try to allocate two strip buffers for double-buffering
    static const int stripSizes[] = {56, 24, 8};  // ~90KB, ~38KB, ~13KB each
    for (int i = 0; i < 3 && sramStrip[0] == nullptr; i++) {
        STRIP_ROWS = stripSizes[i];
        size_t trySize = EL133UF1_WIDTH * STRIP_ROWS;
        sramStrip[0] = (uint8_t*)malloc(trySize);
        sramStrip[1] = (uint8_t*)malloc(trySize);
        if (sramStrip[0] && sramStrip[1]) {
            Serial.printf("    SRAM strips:    2x %d rows (%lu KB) - DMA double-buffer\n", 
                         STRIP_ROWS, trySize / 1024);
        } else {
            // Fall back to single buffer
            if (sramStrip[0]) { free(sramStrip[0]); sramStrip[0] = nullptr; }
            if (sramStrip[1]) { free(sramStrip[1]); sramStrip[1] = nullptr; }
        }
    }
    
    // If double-buffer failed, try single buffer
    if (sramStrip[0] == nullptr) {
        static const int singleSizes[] = {104, 56, 24, 8};
        for (int i = 0; i < 4 && sramStrip[0] == nullptr; i++) {
            STRIP_ROWS = singleSizes[i];
            size_t trySize = EL133UF1_WIDTH * STRIP_ROWS;
            sramStrip[0] = (uint8_t*)malloc(trySize);
            if (sramStrip[0]) {
                Serial.printf("    SRAM strip:     %d rows (%lu KB) - single buffer\n", 
                             STRIP_ROWS, trySize / 1024);
            }
        }
    }
    
    if (sramStrip[0] == nullptr) {
        Serial.println("EL133UF1: Failed to allocate SRAM strip buffer, falling back");
        // Fallback: Direct PSRAM access with basic SIMD
        stepStart = millis();
        
        for (int srcCol = 1599; srcCol >= 0; srcCol--) {
            int outRow = 1599 - srcCol;
            uint8_t* outPtrA = bufA + outRow * 300;
            uint8_t* outPtrB = bufB + outRow * 300;
            const uint8_t* srcPtr = _buffer + srcCol;
            
            // Process bufA region (rows 0-599) - 8 pixels at a time
            for (int i = 0; i < 300; i += 4) {
                packPixels8_SIMD(srcPtr, EL133UF1_WIDTH, outPtrA);
                srcPtr += EL133UF1_WIDTH * 8;
                outPtrA += 4;
            }
            
            // Process bufB region (rows 600-1199)
            for (int i = 0; i < 300; i += 4) {
                packPixels8_SIMD(srcPtr, EL133UF1_WIDTH, outPtrB);
                srcPtr += EL133UF1_WIDTH * 8;
                outPtrB += 4;
            }
        }
        Serial.printf("    Rotate/pack:    %4lu ms (SIMD fallback)\n", millis() - stepStart);
    } else if (sramStrip[1] != nullptr) {
        // Double-buffered DMA mode: prefetch next strip while processing current
        stepStart = millis();
        int currentBuf = 0;
        
        // ===== Process bufA (source rows 0-599) =====
        // Prime the pipeline: start first DMA transfer
        dmaMemcpyStart(sramStrip[0], _buffer, STRIP_ROWS * EL133UF1_WIDTH);
        
        for (int stripStart = 0; stripStart < 600; stripStart += STRIP_ROWS) {
            int stripH = min(STRIP_ROWS, 600 - stripStart);
            int nextStrip = stripStart + STRIP_ROWS;
            
            // Wait for current strip's DMA to complete
            dmaMemcpyWait();
            
            // Start DMA for next strip (overlaps with processing)
            if (nextStrip < 600) {
                int nextBuf = 1 - currentBuf;
                dmaMemcpyStart(sramStrip[nextBuf], 
                              _buffer + nextStrip * EL133UF1_WIDTH, 
                              min(STRIP_ROWS, 600 - nextStrip) * EL133UF1_WIDTH);
            }
            
            // Process current strip (CPU works while DMA runs)
            processStrip(sramStrip[currentBuf], bufA, stripStart, stripH, 0);
            
            currentBuf = 1 - currentBuf;
        }
        
        // ===== Process bufB (source rows 600-1199) =====
        currentBuf = 0;
        dmaMemcpyStart(sramStrip[0], _buffer + 600 * EL133UF1_WIDTH, STRIP_ROWS * EL133UF1_WIDTH);
        
        for (int stripStart = 600; stripStart < 1200; stripStart += STRIP_ROWS) {
            int stripH = min(STRIP_ROWS, 1200 - stripStart);
            int nextStrip = stripStart + STRIP_ROWS;
            
            dmaMemcpyWait();
            
            if (nextStrip < 1200) {
                int nextBuf = 1 - currentBuf;
                dmaMemcpyStart(sramStrip[nextBuf],
                              _buffer + nextStrip * EL133UF1_WIDTH,
                              min(STRIP_ROWS, 1200 - nextStrip) * EL133UF1_WIDTH);
            }
            
            // Process with offset for bufB (strips start at row 600)
            processStrip(sramStrip[currentBuf], bufB, stripStart, stripH, 300);
            
            currentBuf = 1 - currentBuf;
        }
        
        free(sramStrip[0]);
        free(sramStrip[1]);
        Serial.printf("    Rotate/pack:    %4lu ms (DMA+SIMD)\n", millis() - stepStart);
    } else {
        // Single buffer mode with DMA
        stepStart = millis();
        
        // Process bufA (source rows 0-599) in strips
        for (int stripStart = 0; stripStart < 600; stripStart += STRIP_ROWS) {
            int stripH = min(STRIP_ROWS, 600 - stripStart);
            
            // DMA copy (synchronous since single buffer)
            dmaMemcpy(sramStrip[0], _buffer + stripStart * EL133UF1_WIDTH, 
                     stripH * EL133UF1_WIDTH);
            
            processStrip(sramStrip[0], bufA, stripStart, stripH, 0);
        }
        
        // Process bufB (source rows 600-1199) in strips
        for (int stripStart = 600; stripStart < 1200; stripStart += STRIP_ROWS) {
            int stripH = min(STRIP_ROWS, 1200 - stripStart);
            
            dmaMemcpy(sramStrip[0], _buffer + stripStart * EL133UF1_WIDTH,
                     stripH * EL133UF1_WIDTH);
            
            processStrip(sramStrip[0], bufB, stripStart, stripH, 300);
        }
        
        free(sramStrip[0]);
        Serial.printf("    Rotate/pack:    %4lu ms (DMA+SIMD single)\n", millis() - stepStart);
    }

    // SPI transmission - already uses DMA internally via Arduino-Pico
    stepStart = millis();
    _sendCommand(CMD_DTM, CS0_SEL, bufA, SEND_HALF_SIZE);
    uint32_t spiA = millis() - stepStart;
    
    stepStart = millis();
    _sendCommand(CMD_DTM, CS1_SEL, bufB, SEND_HALF_SIZE);
    uint32_t spiB = millis() - stepStart;
    
    Serial.printf("    SPI transmit:   %4lu ms (CS0: %lu, CS1: %lu)\n", 
                  spiA + spiB, spiA, spiB);

    free(bufA);
    free(bufB);
}

void EL133UF1::update(bool skipInit) {
    if (!_initialized) {
        Serial.println("EL133UF1: Not initialized!");
        return;
    }

    Serial.println("\n=== EL133UF1: Display Update ===");
    uint32_t totalStart = millis();
    uint32_t stepStart;
    
    // Run init sequence (can skip on subsequent updates)
    if (!skipInit || !_initDone) {
        stepStart = millis();
        _initSequence();
        _initDone = true;
        Serial.printf("  Init sequence:    %4lu ms\n", millis() - stepStart);
    } else {
        Serial.println("  Init sequence:    skipped");
    }
    
    // Send buffer data
    stepStart = millis();
    _sendBuffer();
    Serial.printf("  Send buffer:      %4lu ms\n", millis() - stepStart);

    // Power on
    stepStart = millis();
    _sendCommand(CMD_PON, CS_BOTH_SEL);
    _busyWait(200);
    Serial.printf("  Power on:         %4lu ms\n", millis() - stepStart);

    // Display refresh
    stepStart = millis();
    const uint8_t drf[] = {0x00};
    _sendCommand(CMD_DRF, CS_BOTH_SEL, drf, sizeof(drf));
    _busyWait(32000);
    Serial.printf("  Panel refresh:    %4lu ms\n", millis() - stepStart);

    // Power off
    stepStart = millis();
    const uint8_t pof[] = {0x00};
    _sendCommand(CMD_POF, CS_BOTH_SEL, pof, sizeof(pof));
    _busyWait(200);
    Serial.printf("  Power off:        %4lu ms\n", millis() - stepStart);

    Serial.printf("  TOTAL:            %4lu ms (%.1f sec)\n", 
                  millis() - totalStart, (millis() - totalStart) / 1000.0);
}

void EL133UF1::updateAsync(bool skipInit) {
    if (!_initialized || _asyncInProgress) return;

    // Run init sequence if needed
    if (!skipInit || !_initDone) {
        _initSequence();
        _initDone = true;
    }
    
    // Send buffer data
    _sendBuffer();

    // Power on
    _sendCommand(CMD_PON, CS_BOTH_SEL);
    _busyWait(200);

    // Start display refresh (non-blocking after this)
    const uint8_t drf[] = {0x00};
    _sendCommand(CMD_DRF, CS_BOTH_SEL, drf, sizeof(drf));
    
    _asyncInProgress = true;
}

bool EL133UF1::isUpdateComplete() {
    if (!_asyncInProgress) return true;
    
    // Check if busy pin is HIGH (refresh complete)
    if (digitalRead(_busyPin) == HIGH) {
        // Power off
        const uint8_t pof[] = {0x00};
        _sendCommand(CMD_POF, CS_BOTH_SEL, pof, sizeof(pof));
        _busyWait(200);
        _asyncInProgress = false;
        return true;
    }
    return false;
}

void EL133UF1::waitForUpdate() {
    if (!_asyncInProgress) return;
    
    // Wait for refresh to complete
    _busyWait(32000);
    
    // Power off
    const uint8_t pof[] = {0x00};
    _sendCommand(CMD_POF, CS_BOTH_SEL, pof, sizeof(pof));
    _busyWait(200);
    
    _asyncInProgress = false;
}
