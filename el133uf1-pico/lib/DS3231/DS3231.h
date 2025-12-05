/**
 * @file DS3231.h
 * @brief DS3231 RTC driver for RP2350 with deep sleep wake support
 * 
 * This driver provides:
 * - I2C scanning for DS3231 at common addresses
 * - Time read/write with the RTC
 * - Alarm-based wake from deep sleep using the INT/SQW pin
 */

#ifndef DS3231_H
#define DS3231_H

#include <Arduino.h>
#include <Wire.h>

// DS3231 I2C addresses to scan
#define DS3231_ADDR_PRIMARY   0x68  // Main DS3231 address
#define DS3231_ADDR_ALT1      0x57  // Sometimes EEPROM, but check anyway

// DS3231 Register addresses
#define DS3231_REG_SECONDS    0x00
#define DS3231_REG_MINUTES    0x01
#define DS3231_REG_HOURS      0x02
#define DS3231_REG_DAY        0x03  // Day of week (1-7)
#define DS3231_REG_DATE       0x04  // Day of month (1-31)
#define DS3231_REG_MONTH      0x05  // Month (1-12, bit 7 = century)
#define DS3231_REG_YEAR       0x06  // Year (0-99)
#define DS3231_REG_ALARM1_SEC 0x07
#define DS3231_REG_ALARM1_MIN 0x08
#define DS3231_REG_ALARM1_HR  0x09
#define DS3231_REG_ALARM1_DAY 0x0A
#define DS3231_REG_ALARM2_MIN 0x0B
#define DS3231_REG_ALARM2_HR  0x0C
#define DS3231_REG_ALARM2_DAY 0x0D
#define DS3231_REG_CONTROL    0x0E
#define DS3231_REG_STATUS     0x0F
#define DS3231_REG_AGING      0x10
#define DS3231_REG_TEMP_MSB   0x11
#define DS3231_REG_TEMP_LSB   0x12

// Control register bits
#define DS3231_CTRL_A1IE      0x01  // Alarm 1 interrupt enable
#define DS3231_CTRL_A2IE      0x02  // Alarm 2 interrupt enable
#define DS3231_CTRL_INTCN     0x04  // Interrupt control (1=alarm, 0=square wave)
#define DS3231_CTRL_RS1       0x08  // Rate select 1
#define DS3231_CTRL_RS2       0x10  // Rate select 2
#define DS3231_CTRL_CONV      0x20  // Convert temperature
#define DS3231_CTRL_BBSQW     0x40  // Battery-backed square wave enable
#define DS3231_CTRL_EOSC      0x80  // Enable oscillator (0=run, 1=stop when on battery)

// Status register bits
#define DS3231_STAT_A1F       0x01  // Alarm 1 flag
#define DS3231_STAT_A2F       0x02  // Alarm 2 flag
#define DS3231_STAT_BSY       0x04  // Busy
#define DS3231_STAT_EN32KHZ   0x08  // Enable 32kHz output
#define DS3231_STAT_OSF       0x80  // Oscillator stop flag

class DS3231 {
public:
    DS3231();
    
    /**
     * @brief Initialize and scan for DS3231 on I2C bus
     * @param wire Pointer to TwoWire instance (default: &Wire)
     * @param sda SDA pin (default: -1 for default pins)
     * @param scl SCL pin (default: -1 for default pins)
     * @return true if DS3231 found, false otherwise
     */
    bool begin(TwoWire* wire = &Wire, int sda = -1, int scl = -1);
    
    /**
     * @brief Check if DS3231 was detected
     */
    bool isPresent() const { return _present; }
    
    /**
     * @brief Get the I2C address where DS3231 was found
     */
    uint8_t getAddress() const { return _address; }
    
    /**
     * @brief Set the RTC time from Unix timestamp (seconds since 1970)
     * @param unixTime Unix timestamp in seconds
     */
    void setTime(time_t unixTime);
    
    /**
     * @brief Set the RTC time from milliseconds since epoch
     * @param timeMs Milliseconds since epoch
     */
    void setTimeMs(uint64_t timeMs);
    
    /**
     * @brief Get the current time as Unix timestamp
     * @return Unix timestamp in seconds
     */
    time_t getTime();
    
    /**
     * @brief Get the current time in milliseconds since epoch
     * @return Milliseconds since epoch
     */
    uint64_t getTimeMs();
    
    /**
     * @brief Set alarm 1 to trigger after specified milliseconds
     * @param delayMs Delay in milliseconds (will be rounded to nearest second)
     */
    void setAlarm1(uint32_t delayMs);
    
    /**
     * @brief Set alarm 1 to trigger at specific Unix time
     * @param alarmTime Unix timestamp for alarm
     */
    void setAlarm1At(time_t alarmTime);
    
    /**
     * @brief Clear alarm 1 flag (must be called after wake to re-arm)
     */
    void clearAlarm1();
    
    /**
     * @brief Check if alarm 1 has triggered
     */
    bool alarm1Triggered();
    
    /**
     * @brief Enable alarm 1 interrupt output on INT/SQW pin
     * @param enable true to enable, false to disable
     */
    void enableAlarm1Interrupt(bool enable);
    
    /**
     * @brief Enable 32kHz output on 32K pin
     * @param enable true to enable, false to disable
     */
    void enable32kHz(bool enable);
    
    /**
     * @brief Get temperature from DS3231 (for diagnostics)
     * @return Temperature in degrees Celsius
     */
    float getTemperature();
    
    /**
     * @brief Print DS3231 status to Serial (for debugging)
     */
    void printStatus();

private:
    TwoWire* _wire;
    uint8_t _address;
    bool _present;
    
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len);
    void writeRegisters(uint8_t reg, uint8_t* buffer, uint8_t len);
    
    // BCD conversion helpers
    static uint8_t bcdToDec(uint8_t bcd) { return ((bcd >> 4) * 10) + (bcd & 0x0F); }
    static uint8_t decToBcd(uint8_t dec) { return ((dec / 10) << 4) | (dec % 10); }
};

// Global instance
extern DS3231 rtc;

#endif // DS3231_H
