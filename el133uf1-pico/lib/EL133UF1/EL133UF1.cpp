/**
 * @file EL133UF1.cpp
 * @brief Implementation of EL133UF1 13.3" Spectra 6 E-Ink Display Driver
 */

#include "EL133UF1.h"

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
    _buffer(nullptr)
{
}

bool EL133UF1::begin(int8_t cs0Pin, int8_t cs1Pin, int8_t dcPin, 
                     int8_t resetPin, int8_t busyPin) {
    _cs0Pin = cs0Pin;
    _cs1Pin = cs1Pin;
    _dcPin = dcPin;
    _resetPin = resetPin;
    _busyPin = busyPin;

    // Allocate frame buffer (1.92 MB for 1600x1200)
    _buffer = (uint8_t*)malloc(EL133UF1_WIDTH * EL133UF1_HEIGHT);
    if (_buffer == nullptr) {
        Serial.println("EL133UF1: Failed to allocate frame buffer!");
        return false;
    }
    
    // Initialize buffer to white
    memset(_buffer, EL133UF1_WHITE, EL133UF1_WIDTH * EL133UF1_HEIGHT);

    // Configure GPIO pins
    pinMode(_cs0Pin, OUTPUT);
    pinMode(_cs1Pin, OUTPUT);
    pinMode(_dcPin, OUTPUT);
    pinMode(_resetPin, OUTPUT);
    pinMode(_busyPin, INPUT_PULLUP);

    // Set initial pin states (CS pins are active low, so set high)
    digitalWrite(_cs0Pin, HIGH);
    digitalWrite(_cs1Pin, HIGH);
    digitalWrite(_dcPin, LOW);
    digitalWrite(_resetPin, HIGH);

    // Initialize SPI
    // Note: For arduino-pico with custom pin mapping, configure SPI pins 
    // in your setup() BEFORE calling display.begin():
    //   SPI.setSCK(sckPin);
    //   SPI.setTX(mosiPin);
    // Then call display.begin() which will call _spi->begin()
    _spi->begin();

    // Perform hardware reset
    _reset();

    // Wait for display to be ready
    if (!_busyWait(1000)) {
        Serial.println("EL133UF1: Display not responding after reset");
        return false;
    }

    // Send initialization sequence
    _initSequence();

    _initialized = true;
    Serial.println("EL133UF1: Initialization complete");
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
    // From working CircuitPython reference - TWO PHASE wait:
    // 1. First wait for busy to ASSERT (go LOW) - display starts working
    // 2. Then wait for busy to DEASSERT (go HIGH) - display finished
    //
    // This ensures we catch the full busy cycle, not just part of it.
    
    uint32_t startTime = millis();
    
    // Phase 1: Wait for busy to go LOW (display acknowledges command)
    while (digitalRead(_busyPin) == HIGH) {
        if (millis() - startTime > timeoutMs) {
            // If it never went low, the display might not be responding
            // or the command completed very fast
            Serial.println("EL133UF1: Busy never asserted (stayed HIGH)");
            return true;  // Continue anyway - might be OK
        }
        delay(10);
    }
    
    // Phase 2: Wait for busy to go HIGH (display finished)
    while (digitalRead(_busyPin) == LOW) {
        if (millis() - startTime > timeoutMs) {
            Serial.println("EL133UF1: Busy timeout (stuck LOW)");
            return false;
        }
        delay(10);
    }
    
    return true;
}

void EL133UF1::_sendCommand(uint8_t cmd, uint8_t csSel, const uint8_t* data, size_t len) {
    _spi->beginTransaction(_spiSettings);
    
    // Assert chip select(s) - active LOW
    if (csSel & CS0_SEL) {
        digitalWrite(_cs0Pin, LOW);
    }
    if (csSel & CS1_SEL) {
        digitalWrite(_cs1Pin, LOW);
    }

    // Send command (DC low = command mode)
    // CircuitPython reference uses time.sleep(0.1) = 100ms here
    digitalWrite(_dcPin, LOW);
    delay(100);  // 100ms delay for DC setup (matches working CircuitPython)
    _spi->transfer(cmd);

    // Send data if present (DC high = data mode)
    if (data != nullptr && len > 0) {
        digitalWrite(_dcPin, HIGH);
        
        // Send data in chunks to avoid issues with large transfers
        const size_t chunkSize = 4096;
        for (size_t offset = 0; offset < len; offset += chunkSize) {
            size_t remaining = len - offset;
            size_t toSend = (remaining < chunkSize) ? remaining : chunkSize;
            _spi->transfer(data + offset, nullptr, toSend);
        }
    }

    // Deassert chip selects - return to HIGH (inactive)
    digitalWrite(_cs0Pin, HIGH);
    digitalWrite(_cs1Pin, HIGH);
    digitalWrite(_dcPin, LOW);

    _spi->endTransaction();
}

void EL133UF1::_initSequence() {
    // Initialization sequence from working CircuitPython reference
    // Note: CircuitPython sends power commands to CS_BOTH (differs from Pimoroni Python)
    // but both approaches appear to work
    
    // ANTM - Anti-crosstalk magic (CS0 only) - must be sent first after reset
    const uint8_t antm[] = {0xC0, 0x1C, 0x1C, 0xCC, 0xCC, 0xCC, 0x15, 0x15, 0x55};
    _sendCommand(CMD_ANTM, CS0_SEL, antm, sizeof(antm));

    // CMD66 / 0xF0 (both CS)
    const uint8_t cmd66[] = {0x49, 0x55, 0x13, 0x5D, 0x05, 0x10};
    _sendCommand(CMD_CMD66, CS_BOTH_SEL, cmd66, sizeof(cmd66));

    // PSR - Panel Setting (both CS)
    const uint8_t psr[] = {0xDF, 0x69};
    _sendCommand(CMD_PSR, CS_BOTH_SEL, psr, sizeof(psr));

    // PLL Control (both CS)
    const uint8_t pll[] = {0x08};
    _sendCommand(CMD_PLL, CS_BOTH_SEL, pll, sizeof(pll));

    // CDI - VCOM and Data Interval (both CS)
    const uint8_t cdi[] = {0xF7};
    _sendCommand(CMD_CDI, CS_BOTH_SEL, cdi, sizeof(cdi));

    // TCON Setting (both CS)
    const uint8_t tcon[] = {0x03, 0x03};
    _sendCommand(CMD_TCON, CS_BOTH_SEL, tcon, sizeof(tcon));

    // AGID - Auto Gate ID (both CS)
    const uint8_t agid[] = {0x10};
    _sendCommand(CMD_AGID, CS_BOTH_SEL, agid, sizeof(agid));

    // PWS - Power Saving (both CS)
    const uint8_t pws[] = {0x22};
    _sendCommand(CMD_PWS, CS_BOTH_SEL, pws, sizeof(pws));

    // CCSET - Cascade Setting (both CS)
    const uint8_t ccset[] = {0x01};
    _sendCommand(CMD_CCSET, CS_BOTH_SEL, ccset, sizeof(ccset));

    // TRES - Resolution Setting (both CS)
    // 0x04B0 = 1200, 0x0320 = 800
    const uint8_t tres[] = {0x04, 0xB0, 0x03, 0x20};
    _sendCommand(CMD_TRES, CS_BOTH_SEL, tres, sizeof(tres));

    // === Power settings ===
    // CircuitPython sends these to CS_BOTH and it works
    // Pimoroni Python sends to CS0_SEL only
    // Using CS_BOTH_SEL to match working CircuitPython reference
    
    // PWR - Power Setting
    const uint8_t pwr[] = {0x0F, 0x00, 0x28, 0x2C, 0x28, 0x38};
    _sendCommand(CMD_PWR, CS_BOTH_SEL, pwr, sizeof(pwr));

    // EN_BUF - Enable Buffer
    const uint8_t en_buf[] = {0x07};
    _sendCommand(CMD_EN_BUF, CS_BOTH_SEL, en_buf, sizeof(en_buf));

    // BTST_P - Booster Soft Start Positive
    const uint8_t btst_p[] = {0xD8, 0x18};
    _sendCommand(CMD_BTST_P, CS_BOTH_SEL, btst_p, sizeof(btst_p));

    // BOOST_VDDP_EN
    const uint8_t boost_vddp[] = {0x01};
    _sendCommand(CMD_BOOST_VDDP_EN, CS_BOTH_SEL, boost_vddp, sizeof(boost_vddp));

    // BTST_N - Booster Soft Start Negative
    const uint8_t btst_n[] = {0xD8, 0x18};
    _sendCommand(CMD_BTST_N, CS_BOTH_SEL, btst_n, sizeof(btst_n));

    // BUCK_BOOST_VDDN
    const uint8_t buck_boost[] = {0x01};
    _sendCommand(CMD_BUCK_BOOST_VDDN, CS_BOTH_SEL, buck_boost, sizeof(buck_boost));

    // TFT_VCOM_POWER
    const uint8_t vcom_power[] = {0x02};
    _sendCommand(CMD_TFT_VCOM_POWER, CS_BOTH_SEL, vcom_power, sizeof(vcom_power));

    Serial.println("EL133UF1: Init sequence complete");
}

void EL133UF1::clear(uint8_t color) {
    if (_buffer == nullptr) return;
    memset(_buffer, color & 0x07, EL133UF1_WIDTH * EL133UF1_HEIGHT);
}

void EL133UF1::setPixel(int16_t x, int16_t y, uint8_t color) {
    if (_buffer == nullptr) return;
    if (x < 0 || x >= EL133UF1_WIDTH || y < 0 || y >= EL133UF1_HEIGHT) return;
    
    _buffer[y * EL133UF1_WIDTH + x] = color & 0x07;
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
    
    uint8_t* ptr = _buffer + y * EL133UF1_WIDTH + x;
    memset(ptr, color & 0x07, w);
}

void EL133UF1::drawVLine(int16_t x, int16_t y, int16_t h, uint8_t color) {
    if (x < 0 || x >= EL133UF1_WIDTH) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > EL133UF1_HEIGHT) { h = EL133UF1_HEIGHT - y; }
    if (h <= 0) return;
    
    uint8_t c = color & 0x07;
    for (int16_t i = 0; i < h; i++) {
        _buffer[(y + i) * EL133UF1_WIDTH + x] = c;
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
    
    for (int16_t i = 0; i < h; i++) {
        drawHLine(x, y + i, w, color);
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

bool EL133UF1::isBusy() {
    return digitalRead(_busyPin) == LOW;
}

void EL133UF1::_sendBuffer() {
    if (_buffer == nullptr) return;

    // Allocate temporary buffers for packed data
    // Each half of the display: 1200 rows * 600 cols = 720000 pixels
    // Packed as nibbles: 720000 / 2 = 360000 bytes
    uint8_t* bufA = (uint8_t*)malloc(HALF_BUFFER_SIZE);
    uint8_t* bufB = (uint8_t*)malloc(HALF_BUFFER_SIZE);
    
    if (bufA == nullptr || bufB == nullptr) {
        Serial.println("EL133UF1: Failed to allocate send buffers");
        if (bufA) free(bufA);
        if (bufB) free(bufB);
        return;
    }

    Serial.println("EL133UF1: Preparing buffer data...");

    // The Python code does:
    // 1. Optional flips
    // 2. Rotate 90 degrees counter-clockwise
    // 3. Split into left (600 cols) and right (600 cols)
    // 4. Pack nibbles
    
    // After -90 degree rotation:
    // - Original (1600 x 1200) becomes (1200 x 1600)
    // - For each pixel at (x, y) in rotated image:
    //   - It comes from (y, height - 1 - x) in the original
    //   - Or equivalently, original (row, col) goes to rotated (width - 1 - col, row)
    
    // The display expects:
    // - buf_a: first 600 columns of the rotated image (all 1200 rows)
    // - buf_b: remaining 1000 columns... wait, let me re-check
    
    // Looking at Python: region = numpy.rot90(region, -1) rotates 90 deg clockwise
    // Then buf_a = region[:, :600] and buf_b = region[:, 600:]
    // Original is (1200 rows, 1600 cols), after rot90(-1) it's (1600 rows, 1200 cols)
    // So buf_a is (1600 x 600) = 960000 pixels, buf_b is (1600 x 600) = 960000 pixels
    // Packed: 480000 bytes each
    
    // Wait, that doesn't match HALF_BUFFER_SIZE. Let me recalculate.
    // The Python code: self.buf = numpy.zeros((self.rows, self.cols), dtype=numpy.uint8)
    // where rows = 1200, cols = 1600
    // After rot90(-1, clockwise): shape becomes (1600, 1200)
    // buf_a = region[:, :600] -> (1600, 600) -> 960000 pixels -> 480000 bytes packed
    // buf_b = region[:, 600:] -> (1600, 600) -> 960000 pixels -> 480000 bytes packed
    
    // My HALF_BUFFER_SIZE was wrong. Let me fix it.
    // Actually, I need to reallocate with correct size.
    
    free(bufA);
    free(bufB);
    
    const size_t CORRECT_HALF_SIZE = (1600 * 600) / 2;  // 480000 bytes
    bufA = (uint8_t*)malloc(CORRECT_HALF_SIZE);
    bufB = (uint8_t*)malloc(CORRECT_HALF_SIZE);
    
    if (bufA == nullptr || bufB == nullptr) {
        Serial.println("EL133UF1: Failed to allocate send buffers (second attempt)");
        if (bufA) free(bufA);
        if (bufB) free(bufB);
        return;
    }

    // Process the buffer with rotation
    // After 90° clockwise rotation (rot90 with k=-1):
    // new[new_row][new_col] = old[cols - 1 - new_col][new_row]
    // where old is (1200 rows x 1600 cols), new is (1600 rows x 1200 cols)
    
    size_t idxA = 0;
    size_t idxB = 0;
    
    for (int newRow = 0; newRow < 1600; newRow++) {
        // First half (columns 0-599 of rotated image go to bufA)
        for (int newCol = 0; newCol < 600; newCol += 2) {
            int oldRow0, oldCol0, oldRow1, oldCol1;
            
            if (!_vFlip && !_hFlip) {
                // No flips
                oldCol0 = newRow;
                oldRow0 = 1199 - newCol;
                oldCol1 = newRow;
                oldRow1 = 1199 - (newCol + 1);
            } else if (_vFlip && !_hFlip) {
                // Vertical flip (in original coordinates)
                oldCol0 = 1599 - newRow;
                oldRow0 = 1199 - newCol;
                oldCol1 = 1599 - newRow;
                oldRow1 = 1199 - (newCol + 1);
            } else if (!_vFlip && _hFlip) {
                // Horizontal flip (in original coordinates)
                oldCol0 = newRow;
                oldRow0 = newCol;
                oldCol1 = newRow;
                oldRow1 = newCol + 1;
            } else {
                // Both flips
                oldCol0 = 1599 - newRow;
                oldRow0 = newCol;
                oldCol1 = 1599 - newRow;
                oldRow1 = newCol + 1;
            }
            
            uint8_t p0 = _buffer[oldRow0 * EL133UF1_WIDTH + oldCol0] & 0x07;
            uint8_t p1 = _buffer[oldRow1 * EL133UF1_WIDTH + oldCol1] & 0x07;
            bufA[idxA++] = (p0 << 4) | p1;
        }
        
        // Second half (columns 600-1199 of rotated image go to bufB)
        for (int newCol = 600; newCol < 1200; newCol += 2) {
            int oldRow0, oldCol0, oldRow1, oldCol1;
            
            if (!_vFlip && !_hFlip) {
                oldCol0 = newRow;
                oldRow0 = 1199 - newCol;
                oldCol1 = newRow;
                oldRow1 = 1199 - (newCol + 1);
            } else if (_vFlip && !_hFlip) {
                oldCol0 = 1599 - newRow;
                oldRow0 = 1199 - newCol;
                oldCol1 = 1599 - newRow;
                oldRow1 = 1199 - (newCol + 1);
            } else if (!_vFlip && _hFlip) {
                oldCol0 = newRow;
                oldRow0 = newCol;
                oldCol1 = newRow;
                oldRow1 = newCol + 1;
            } else {
                oldCol0 = 1599 - newRow;
                oldRow0 = newCol;
                oldCol1 = 1599 - newRow;
                oldRow1 = newCol + 1;
            }
            
            uint8_t p0 = _buffer[oldRow0 * EL133UF1_WIDTH + oldCol0] & 0x07;
            uint8_t p1 = _buffer[oldRow1 * EL133UF1_WIDTH + oldCol1] & 0x07;
            bufB[idxB++] = (p0 << 4) | p1;
        }
    }

    Serial.println("EL133UF1: Sending image data to display...");
    
    // Send data to display
    _sendCommand(CMD_DTM, CS0_SEL, bufA, CORRECT_HALF_SIZE);
    _sendCommand(CMD_DTM, CS1_SEL, bufB, CORRECT_HALF_SIZE);

    free(bufA);
    free(bufB);
    
    Serial.println("EL133UF1: Image data sent");
}

void EL133UF1::update() {
    if (!_initialized) {
        Serial.println("EL133UF1: Not initialized!");
        return;
    }

    Serial.println("EL133UF1: Starting display update...");
    
    // Run setup/init sequence (Python calls self.setup() in _update)
    _initSequence();
    
    // Send buffer data to both controllers
    _sendBuffer();

    // Power on (Python: self._send_command(EL133UF1_PON, CS_BOTH_SEL))
    Serial.println("EL133UF1: Power on...");
    _sendCommand(CMD_PON, CS_BOTH_SEL);
    _busyWait(200);  // Python: self._busy_wait(0.2)

    // Display refresh (Python: self._send_command(EL133UF1_DRF, CS_BOTH_SEL, [0x00]))
    Serial.println("EL133UF1: Triggering refresh (this takes ~30 seconds)...");
    const uint8_t drf[] = {0x00};
    _sendCommand(CMD_DRF, CS_BOTH_SEL, drf, sizeof(drf));
    _busyWait(32000);  // Python: self._busy_wait(32.0)

    // Power off (Python: self._send_command(EL133UF1_POF, CS_BOTH_SEL, [0x00]))
    Serial.println("EL133UF1: Power off...");
    const uint8_t pof[] = {0x00};
    _sendCommand(CMD_POF, CS_BOTH_SEL, pof, sizeof(pof));
    _busyWait(200);  // Python: self._busy_wait(0.2)

    Serial.println("EL133UF1: Display update complete");
}
