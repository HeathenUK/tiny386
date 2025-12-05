/**
 * @file AT24C32.h
 * @brief AT24C32 I2C EEPROM driver (32Kbit / 4KB)
 * 
 * Common on DS3231 RTC modules at address 0x57
 */

#ifndef AT24C32_H
#define AT24C32_H

#include <Arduino.h>
#include <Wire.h>

#define AT24C32_DEFAULT_ADDR  0x57
#define AT24C32_SIZE          4096    // 4KB
#define AT24C32_PAGE_SIZE     32      // 32 bytes per page

// Storage layout - define what goes where
#define EEPROM_MAGIC_ADDR     0x0000  // 4 bytes: magic number
#define EEPROM_VERSION_ADDR   0x0004  // 1 byte: config version
#define EEPROM_BOOT_COUNT     0x0010  // 4 bytes: total boot count
#define EEPROM_TOTAL_UPTIME   0x0014  // 4 bytes: total uptime in seconds
#define EEPROM_LAST_NTP_SYNC  0x0020  // 4 bytes: last NTP sync (unix time)
#define EEPROM_WIFI_SSID      0x0100  // 33 bytes: WiFi SSID (32 + null)
#define EEPROM_WIFI_PSK       0x0140  // 65 bytes: WiFi password (64 + null)
#define EEPROM_SLEEP_SEC      0x0200  // 2 bytes: sleep duration in seconds
#define EEPROM_TEMP_LOG_START 0x0800  // Temperature log (to end of EEPROM)
#define EEPROM_TEMP_LOG_SIZE  (AT24C32_SIZE - EEPROM_TEMP_LOG_START)

#define EEPROM_MAGIC_VALUE    0x52544345  // "ECTR" - EEPROM Config for RTC

class AT24C32 {
public:
    AT24C32();
    
    /**
     * @brief Initialize EEPROM
     * @param wire I2C bus
     * @param addr I2C address (default 0x57)
     * @return true if EEPROM found
     */
    bool begin(TwoWire* wire, uint8_t addr = AT24C32_DEFAULT_ADDR);
    
    bool isPresent() const { return _present; }
    
    // Low-level read/write
    uint8_t readByte(uint16_t addr);
    void writeByte(uint16_t addr, uint8_t value);
    void readBytes(uint16_t addr, uint8_t* buffer, uint16_t len);
    void writeBytes(uint16_t addr, const uint8_t* buffer, uint16_t len);
    
    // Typed helpers
    uint32_t readUInt32(uint16_t addr);
    void writeUInt32(uint16_t addr, uint32_t value);
    uint16_t readUInt16(uint16_t addr);
    void writeUInt16(uint16_t addr, uint16_t value);
    void readString(uint16_t addr, char* buffer, uint16_t maxLen);
    void writeString(uint16_t addr, const char* str, uint16_t maxLen);
    
    // High-level storage API
    bool isFormatted();
    void format();  // Initialize with defaults
    
    // Boot counter
    uint32_t getBootCount();
    void incrementBootCount();
    
    // Uptime tracking
    uint32_t getTotalUptime();
    void addUptime(uint32_t seconds);
    
    // Last NTP sync
    uint32_t getLastNtpSync();
    void setLastNtpSync(uint32_t unixTime);
    
    // WiFi credentials
    bool getWifiCredentials(char* ssid, size_t ssidLen, char* psk, size_t pskLen);
    void setWifiCredentials(const char* ssid, const char* psk);
    bool hasWifiCredentials();
    
    // Sleep duration
    uint16_t getSleepSeconds();
    void setSleepSeconds(uint16_t seconds);
    
    // Temperature logging
    void logTemperature(float temp);
    uint16_t getTemperatureLogCount();
    float getLoggedTemperature(uint16_t index);
    
    void printStatus();

private:
    TwoWire* _wire;
    uint8_t _addr;
    bool _present;
    
    void waitForWrite();  // Wait for write cycle to complete
};

extern AT24C32 eeprom;

#endif // AT24C32_H
