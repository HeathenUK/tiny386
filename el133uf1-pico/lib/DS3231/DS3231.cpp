/**
 * @file DS3231.cpp
 * @brief DS3231 RTC driver implementation
 */

#include "DS3231.h"

// Global instance
DS3231 rtc;

DS3231::DS3231() : _wire(nullptr), _address(0), _present(false) {
}

bool DS3231::begin(TwoWire* wire, int sda, int scl) {
    _wire = wire;
    _present = false;
    _address = 0;
    
    Serial.printf("DS3231: Initializing I2C (SDA=%d, SCL=%d)...\n", sda, scl);
    Serial.flush();
    
    // Initialize I2C with custom pins if specified
    if (sda >= 0 && scl >= 0) {
        _wire->setSDA(sda);
        _wire->setSCL(scl);
    }
    
    _wire->begin();
    _wire->setClock(100000);  // 100kHz for reliability
    _wire->setTimeout(1000);  // 1 second timeout
    
    Serial.println("DS3231: I2C initialized, scanning...");
    Serial.flush();
    
    // Only check the primary DS3231 address (0x68)
    uint8_t addr = DS3231_ADDR_PRIMARY;
    Serial.printf("DS3231: Checking 0x%02X... ", addr);
    Serial.flush();
    
    _wire->beginTransmission(addr);
    uint8_t error = _wire->endTransmission();
    
    Serial.printf("result=%d\n", error);
    Serial.flush();
    
    if (error == 0) {
        Serial.printf("DS3231: Found device at 0x%02X\n", addr);
        
        // Verify it's a DS3231 by reading control/status registers
        _address = addr;
        uint8_t ctrl = readRegister(DS3231_REG_CONTROL);
        uint8_t status = readRegister(DS3231_REG_STATUS);
        
        Serial.printf("DS3231: Control=0x%02X, Status=0x%02X\n", ctrl, status);
        
        // DS3231 should have reasonable values in these registers
        // The OSF bit might be set on first power-up
        if (status != 0xFF && ctrl != 0xFF) {
            _present = true;
            Serial.printf("DS3231: Confirmed at 0x%02X\n", addr);
            
            // Clear OSF flag if set (indicates oscillator was stopped)
            if (status & DS3231_STAT_OSF) {
                Serial.println("DS3231: Oscillator was stopped, clearing flag");
                writeRegister(DS3231_REG_STATUS, status & ~DS3231_STAT_OSF);
            }
            
            // Configure for alarm interrupt (not square wave)
            // INTCN=1, disable both alarms initially
            writeRegister(DS3231_REG_CONTROL, DS3231_CTRL_INTCN);
            
            // Enable 32kHz output
            enable32kHz(true);
        }
    }
    
    if (!_present) {
        Serial.println("DS3231: Not found!");
    }
    
    return _present;
}

uint8_t DS3231::readRegister(uint8_t reg) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->endTransmission();
    
    _wire->requestFrom(_address, (uint8_t)1);
    if (_wire->available()) {
        return _wire->read();
    }
    return 0xFF;
}

void DS3231::writeRegister(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(value);
    _wire->endTransmission();
}

void DS3231::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->endTransmission();
    
    _wire->requestFrom(_address, len);
    for (uint8_t i = 0; i < len && _wire->available(); i++) {
        buffer[i] = _wire->read();
    }
}

void DS3231::writeRegisters(uint8_t reg, uint8_t* buffer, uint8_t len) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    for (uint8_t i = 0; i < len; i++) {
        _wire->write(buffer[i]);
    }
    _wire->endTransmission();
}

void DS3231::setTime(time_t unixTime) {
    struct tm* t = gmtime(&unixTime);
    
    uint8_t data[7];
    data[0] = decToBcd(t->tm_sec);
    data[1] = decToBcd(t->tm_min);
    data[2] = decToBcd(t->tm_hour);  // 24-hour format
    data[3] = decToBcd(t->tm_wday + 1);  // DS3231 uses 1-7
    data[4] = decToBcd(t->tm_mday);
    data[5] = decToBcd(t->tm_mon + 1);  // DS3231 uses 1-12
    
    // Year: DS3231 stores 0-99, tm_year is years since 1900
    int year = t->tm_year + 1900;
    if (year >= 2000) {
        data[5] |= 0x80;  // Century bit
        data[6] = decToBcd(year - 2000);
    } else {
        data[6] = decToBcd(year - 1900);
    }
    
    writeRegisters(DS3231_REG_SECONDS, data, 7);
    
    Serial.printf("DS3231: Set time to %04d-%02d-%02d %02d:%02d:%02d\n",
                  year, t->tm_mon + 1, t->tm_mday,
                  t->tm_hour, t->tm_min, t->tm_sec);
}

void DS3231::setTimeMs(uint64_t timeMs) {
    setTime((time_t)(timeMs / 1000));
}

time_t DS3231::getTime() {
    uint8_t data[7];
    readRegisters(DS3231_REG_SECONDS, data, 7);
    
    struct tm t;
    t.tm_sec = bcdToDec(data[0] & 0x7F);
    t.tm_min = bcdToDec(data[1] & 0x7F);
    t.tm_hour = bcdToDec(data[2] & 0x3F);  // Assume 24-hour format
    t.tm_wday = bcdToDec(data[3] & 0x07) - 1;  // Convert to 0-6
    t.tm_mday = bcdToDec(data[4] & 0x3F);
    t.tm_mon = bcdToDec(data[5] & 0x1F) - 1;  // Convert to 0-11
    
    int year = bcdToDec(data[6]);
    if (data[5] & 0x80) {
        year += 2000;
    } else {
        year += 1900;
    }
    t.tm_year = year - 1900;
    t.tm_isdst = 0;
    
    return mktime(&t);
}

uint64_t DS3231::getTimeMs() {
    time_t t = getTime();
    // If time is before 2000 (invalid/unset RTC), return 0
    if (t < 946684800) {  // 2000-01-01 00:00:00
        return 0;
    }
    return (uint64_t)t * 1000;
}

void DS3231::setAlarm1(uint32_t delayMs) {
    // Get current time and add delay
    time_t alarmTime = getTime() + (delayMs / 1000) + 1;  // Round up
    setAlarm1At(alarmTime);
}

void DS3231::setAlarm1At(time_t alarmTime) {
    struct tm* t = gmtime(&alarmTime);
    
    // Alarm 1 has seconds, minutes, hours, day/date
    // We'll use "match hours, minutes, seconds" mode (A1M4=1, A1M3=0, A1M2=0, A1M1=0)
    // Actually, let's use "match day, hours, minutes, seconds" for full matching
    uint8_t data[4];
    data[0] = decToBcd(t->tm_sec) & 0x7F;       // A1M1=0 (match seconds)
    data[1] = decToBcd(t->tm_min) & 0x7F;       // A1M2=0 (match minutes)
    data[2] = decToBcd(t->tm_hour) & 0x3F;      // A1M3=0 (match hours), 24hr format
    data[3] = decToBcd(t->tm_mday) & 0x3F;      // A1M4=0, DY/DT=0 (match date)
    
    writeRegisters(DS3231_REG_ALARM1_SEC, data, 4);
    
    // Clear any existing alarm flag
    clearAlarm1();
    
    // Enable alarm 1 interrupt
    enableAlarm1Interrupt(true);
    
    Serial.printf("DS3231: Alarm 1 set for %02d:%02d:%02d on day %d\n",
                  t->tm_hour, t->tm_min, t->tm_sec, t->tm_mday);
}

void DS3231::clearAlarm1() {
    uint8_t status = readRegister(DS3231_REG_STATUS);
    writeRegister(DS3231_REG_STATUS, status & ~DS3231_STAT_A1F);
}

bool DS3231::alarm1Triggered() {
    return (readRegister(DS3231_REG_STATUS) & DS3231_STAT_A1F) != 0;
}

void DS3231::enableAlarm1Interrupt(bool enable) {
    uint8_t ctrl = readRegister(DS3231_REG_CONTROL);
    
    // Always set INTCN for interrupt mode (not square wave)
    ctrl |= DS3231_CTRL_INTCN;
    
    if (enable) {
        ctrl |= DS3231_CTRL_A1IE;
    } else {
        ctrl &= ~DS3231_CTRL_A1IE;
    }
    
    writeRegister(DS3231_REG_CONTROL, ctrl);
}

void DS3231::enable32kHz(bool enable) {
    uint8_t status = readRegister(DS3231_REG_STATUS);
    
    if (enable) {
        status |= DS3231_STAT_EN32KHZ;
    } else {
        status &= ~DS3231_STAT_EN32KHZ;
    }
    
    writeRegister(DS3231_REG_STATUS, status);
    Serial.printf("DS3231: 32kHz output %s\n", enable ? "enabled" : "disabled");
}

float DS3231::getTemperature() {
    uint8_t msb = readRegister(DS3231_REG_TEMP_MSB);
    uint8_t lsb = readRegister(DS3231_REG_TEMP_LSB);
    
    // Temperature is in 0.25°C increments
    // MSB is integer part (signed), LSB upper 2 bits are fractional
    int16_t temp = ((int16_t)msb << 2) | (lsb >> 6);
    if (temp & 0x200) {  // Sign extend
        temp |= 0xFC00;
    }
    
    return temp * 0.25f;
}

void DS3231::printStatus() {
    if (!_present) {
        Serial.println("DS3231: Not present");
        return;
    }
    
    uint8_t ctrl = readRegister(DS3231_REG_CONTROL);
    uint8_t status = readRegister(DS3231_REG_STATUS);
    time_t now = getTime();
    struct tm* t = gmtime(&now);
    
    Serial.println("=== DS3231 Status ===");
    Serial.printf("  Address: 0x%02X\n", _address);
    Serial.printf("  Time: %04d-%02d-%02d %02d:%02d:%02d\n",
                  t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                  t->tm_hour, t->tm_min, t->tm_sec);
    Serial.printf("  Temperature: %.2f°C\n", getTemperature());
    Serial.printf("  Control: 0x%02X (INTCN=%d, A1IE=%d, A2IE=%d)\n",
                  ctrl,
                  (ctrl & DS3231_CTRL_INTCN) ? 1 : 0,
                  (ctrl & DS3231_CTRL_A1IE) ? 1 : 0,
                  (ctrl & DS3231_CTRL_A2IE) ? 1 : 0);
    Serial.printf("  Status: 0x%02X (OSF=%d, A1F=%d, A2F=%d, EN32K=%d)\n",
                  status,
                  (status & DS3231_STAT_OSF) ? 1 : 0,
                  (status & DS3231_STAT_A1F) ? 1 : 0,
                  (status & DS3231_STAT_A2F) ? 1 : 0,
                  (status & DS3231_STAT_EN32KHZ) ? 1 : 0);
    Serial.println("=====================");
}
