/**
 * @file main.cpp
 * @brief Example application for EL133UF1 13.3" Spectra 6 E-Ink Display
 * 
 * This example demonstrates driving the EL133UF1 e-ink panel with a
 * Pimoroni Pico Plus 2 W using the Arduino-Pico framework.
 * 
 * Wiring:
 *   Display      Pico Plus 2 W
 *   -------      -------------
 *   MOSI    ->   GP19 (SPI0 TX)
 *   SCLK    ->   GP18 (SPI0 SCK)
 *   CS0     ->   GP17
 *   CS1     ->   GP16
 *   DC      ->   GP20
 *   RESET   ->   GP21
 *   BUSY    ->   GP22
 *   GND     ->   GND
 *   3.3V    ->   3V3
 */

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include "EL133UF1.h"
#include "EL133UF1_TTF.h"
#include "fonts/opensans.h"
#include "pico_sleep.h"
#include "DS3231.h"
#include "AT24C32.h"
#include "hardware/structs/powman.h"
#include "hardware/powman.h"

// WiFi credentials
const char* WIFI_SSID = "JELLING";
const char* WIFI_PSK = "Crusty jugglers";

// NTP servers - use pool.ntp.org which is more reliable than NIST
// NIST servers often rate-limit and can be slow
const char* NTP_SERVER1_NAME = "pool.ntp.org";
const char* NTP_SERVER2_NAME = "time.google.com";

// Pin definitions for Pimoroni Pico Plus 2 W with Inky Impression 13.3"
// These match the working CircuitPython reference
#define PIN_SPI_SCK   10    // SPI1 SCK (GP10)
#define PIN_SPI_MOSI  11    // SPI1 TX/MOSI (GP11)
#define PIN_CS0       26    // Chip Select 0 - left half (GP26)
#define PIN_CS1       16    // Chip Select 1 - right half (GP16)
#define PIN_DC        22    // Data/Command (GP22)
#define PIN_RESET     27    // Reset (GP27)
#define PIN_BUSY      17    // Busy (GP17)

// DS3231 RTC pins (I2C1)
#define PIN_RTC_SDA    2    // I2C1 SDA (GP2)
#define PIN_RTC_SCL    3    // I2C1 SCL (GP3)
#define PIN_RTC_INT   18    // DS3231 INT/SQW pin for wake (GP18)

// Create display instance using SPI1
// (SPI1 is the correct bus for GP10/GP11 on Pico)
EL133UF1 display(&SPI1);

// TTF font renderer
EL133UF1_TTF ttf;

// Forward declarations
void drawDemoPattern();
bool connectWiFiAndGetNTP();
void formatTime(uint64_t time_ms, char* buf, size_t len);

// ================================================================
// Connect to WiFi and sync NTP time (arduino-pico native)
// ================================================================
bool connectWiFiAndGetNTP() {
    Serial.println("\n=== Connecting to WiFi ===");
    Serial.printf("SSID: %s\n", WIFI_SSID);
    
    WiFi.begin(WIFI_SSID, WIFI_PSK);
    
    Serial.print("Connecting");
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start < 30000)) {
        Serial.print(".");
        delay(500);
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi connect failed!");
        return false;
    }
    
    Serial.println("\nWiFi connected!");
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    
    // Override DNS with Cloudflare and Google
    WiFi.setDNS(IPAddress(1, 1, 1, 1), IPAddress(8, 8, 8, 8));
    delay(500);
    
    Serial.println("\n=== Getting NTP time ===");
    
    // Use Google NTP (very reliable) with pool.ntp.org as backup
    IPAddress ntpServer1(216, 239, 35, 0);   // time.google.com
    IPAddress ntpServer2(216, 239, 35, 4);   // time2.google.com
    
    // Try DNS for pool.ntp.org as alternative
    IPAddress poolServer;
    if (WiFi.hostByName("pool.ntp.org", poolServer)) {
        ntpServer2 = poolServer;
        Serial.printf("Using: time.google.com + pool.ntp.org (%s)\n", poolServer.toString().c_str());
    } else {
        Serial.println("Using: time.google.com (primary + backup)");
    }
    
    NTP.begin(ntpServer1, ntpServer2);
    Serial.println("NTP initialized, waiting for sync...");
    delay(1000);
    
    // Wait for valid time - up to 90 seconds total with periodic status
    time_t now = time(nullptr);
    int totalWait = 0;
    const int maxWait = 90;  // 90 seconds max
    
    Serial.print("Syncing: ");
    while (now < 1700000000 && totalWait < maxWait) {
        // Process network for 1 second
        for (int i = 0; i < 10; i++) {
            delay(100);
            yield();
        }
        totalWait++;
        now = time(nullptr);
        
        // Show progress every 5 seconds
        if (totalWait % 5 == 0) {
            Serial.printf("[%ds", totalWait);
            if (now > 0) Serial.printf(":%lld", (long long)now);
            Serial.print("] ");
        } else {
            Serial.print(".");
        }
        
        // Success! Exit early
        if (now >= 1700000000) {
            Serial.println(" OK!");
            break;
        }
    }
    
    if (now < 1700000000) {
        Serial.println("\nNTP sync FAILED!");
        WiFi.disconnect(true);
        return false;
    }
    
    Serial.printf("NTP sync successful after %d seconds\n", totalWait);
    Serial.flush();
    
    // Got NTP time - calibrate drift and set time
    Serial.println("Calling sleep_calibrate_drift...");
    Serial.flush();
    
    uint64_t now_ms = (uint64_t)now * 1000;
    sleep_calibrate_drift(now_ms);
    
    Serial.printf("Drift correction: %d ppm\n", sleep_get_drift_ppm());
    Serial.flush();
    
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    Serial.print("Current time: ");
    Serial.println(asctime(&timeinfo));
    Serial.printf("Epoch: %lld\n", (long long)now);
    Serial.flush();
    
    // Save NTP sync time to EEPROM
    if (eeprom.isPresent()) {
        eeprom.setLastNtpSync((uint32_t)now);
        Serial.println("NTP sync time saved to EEPROM");
    }
    
    // Disconnect WiFi to save power
    WiFi.disconnect(true);
    Serial.println("WiFi disconnected (saving power)");
    
    return true;
}

// ================================================================
// Format time from powman timer (ms since epoch) to readable string
// ================================================================
void formatTime(uint64_t time_ms, char* buf, size_t len) {
    time_t time_sec = (time_t)(time_ms / 1000);
    struct tm* timeinfo = gmtime(&time_sec);
    strftime(buf, len, "%Y-%m-%d %H:%M:%S UTC", timeinfo);
}

void doDisplayUpdate(int updateNumber);  // Forward declaration

// Track how many updates we've done (stored in scratch register 1)
#define UPDATE_COUNT_REG 1

// NTP resync interval - LPOSC drifts ~1-5%, so resync periodically
// Every 10 updates at 10s sleep = ~100s between syncs
#define NTP_RESYNC_INTERVAL  10  // Resync every N updates

int getUpdateCount() {
    return (int)powman_hw->scratch[UPDATE_COUNT_REG];
}

void setUpdateCount(int count) {
    powman_hw->scratch[UPDATE_COUNT_REG] = (uint32_t)count;
}

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    
    // Wait for serial connection (longer timeout for reliability)
    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait < 3000)) {
        delay(100);
    }
    delay(500);  // Extra delay for serial to stabilize
    
    // Immediate sign of life
    Serial.println("\n\n>>> BOOT <<<");
    Serial.flush();
    delay(100);
    
    // ================================================================
    // EARLY BOOT TIMER DIAGNOSTICS
    // Capture timer state before anything else touches it
    // ================================================================
    uint64_t boot_timer_value = powman_timer_get_ms();
    bool boot_timer_running = powman_timer_is_running();
    bool boot_using_lposc = (powman_hw->timer & POWMAN_TIMER_USING_LPOSC_BITS) != 0;
    bool boot_using_xosc = (powman_hw->timer & POWMAN_TIMER_USING_XOSC_BITS) != 0;
    uint32_t boot_millis = millis();
    
    Serial.println("=== EARLY BOOT TIMER STATE ===");
    Serial.printf("  powman timer: %llu ms\n", boot_timer_value);
    Serial.printf("  timer running: %d\n", boot_timer_running);
    Serial.printf("  using LPOSC: %d, using XOSC: %d\n", boot_using_lposc, boot_using_xosc);
    Serial.printf("  Arduino millis(): %lu\n", boot_millis);
    Serial.printf("  powman_hw->timer raw: 0x%08lx\n", (unsigned long)powman_hw->timer);
    Serial.println("==============================");
    Serial.flush();
    delay(100);
    
    // ================================================================
    // Initialize DS3231 RTC if present
    // ================================================================
    Serial.println("\n=== Checking for DS3231 RTC ===");
    Serial.printf("  I2C pins: SDA=%d, SCL=%d, INT=%d\n", PIN_RTC_SDA, PIN_RTC_SCL, PIN_RTC_INT);
    Serial.flush();
    delay(100);
    
    Serial.println("  Calling sleep_init_rtc...");
    Serial.flush();
    
    bool hasRTC = sleep_init_rtc(PIN_RTC_SDA, PIN_RTC_SCL, PIN_RTC_INT);
    if (hasRTC) {
        Serial.println("DS3231 RTC found - using for timekeeping");
        // Read current RTC time
        uint64_t rtcTime = sleep_get_time_ms();
        char timeBuf[32];
        formatTime(rtcTime, timeBuf, sizeof(timeBuf));
        Serial.printf("  RTC time: %s\n", timeBuf);
        Serial.printf("  Temperature: %.1f°C\n", rtc.getTemperature());
        
        // Initialize EEPROM (on same I2C bus)
        if (eeprom.begin(&Wire1, 0x57)) {
            eeprom.printStatus();
            
            // Log temperature
            eeprom.logTemperature(rtc.getTemperature());
        }
    } else {
        Serial.println("No DS3231 found - using LPOSC (less accurate)");
    }
    Serial.println("===============================\n");
    Serial.flush();
    
    // Get current update count (from powman scratch for this session)
    int updateCount = sleep_woke_from_deep_sleep() ? getUpdateCount() : 0;
    uint32_t uptime = sleep_get_uptime_seconds();
    
    // Increment persistent boot count in EEPROM (survives power loss)
    uint32_t totalBoots = 0;
    if (eeprom.isPresent()) {
        eeprom.incrementBootCount();
        totalBoots = eeprom.getBootCount();
    }
    
    // ================================================================
    // Check if we woke from deep sleep
    // ================================================================
    bool needsNtpSync = false;
    
    if (sleep_woke_from_deep_sleep()) {
        Serial.println("\n\n========================================");
        Serial.printf("*** WOKE FROM DEEP SLEEP! (update #%d) ***\n", updateCount + 1);
        if (eeprom.isPresent()) {
            Serial.printf("*** Total boots (EEPROM): %lu ***\n", totalBoots);
        }
        Serial.printf("*** RTC uptime: %lu seconds ***\n", uptime);
        if (hasRTC) {
            Serial.println("*** Wake source: DS3231 RTC alarm ***");
            // Clear the DS3231 alarm flag
            rtc.clearAlarm1();
        } else {
            Serial.println("*** Wake source: LPOSC timer ***");
        }
        Serial.println("========================================\n");
        
        // Clear the wake flag
        sleep_clear_wake_flag();
        
        // With DS3231: NTP resync much less often (crystal accurate ~2ppm)
        // Without: Resync every NTP_RESYNC_INTERVAL updates (LPOSC drifts ~1-5%)
        int resyncInterval = hasRTC ? 100 : NTP_RESYNC_INTERVAL;  // 100 = ~1000 seconds
        
        if ((updateCount + 1) % resyncInterval == 0) {
            Serial.println(">>> Periodic NTP resync <<<");
            needsNtpSync = true;
        }
    } else {
        // First boot
        Serial.println("\n\n===========================================");
        Serial.println("EL133UF1 13.3\" Spectra 6 E-Ink Display Demo");
        Serial.println("===========================================\n");
        
        // Check if DS3231 already has valid time (battery-backed)
        // Note: RTC time before 1970 returns garbage when cast to uint64_t
        // So we check the raw time_t value from the RTC
        if (hasRTC) {
            time_t rtcTimeSec = (time_t)(sleep_get_time_ms() / 1000);
            // Valid if between 2020 and 2100
            if (rtcTimeSec > 1577836800 && rtcTimeSec < 4102444800) {
                Serial.println("DS3231 already has valid time from battery backup");
                needsNtpSync = false;
            } else {
                Serial.println("DS3231 time is invalid, need NTP sync");
                needsNtpSync = true;
            }
        } else {
            needsNtpSync = true;
        }
        setUpdateCount(0);
    }
    
    // Sync NTP if needed (cold boot or periodic resync)
    if (needsNtpSync) {
        uint64_t oldTime = sleep_get_time_ms();
        if (connectWiFiAndGetNTP()) {
            uint64_t newTime = sleep_get_time_ms();
            int64_t drift = (int64_t)(newTime - oldTime);
            if (oldTime > 1700000000000ULL) {
                Serial.printf(">>> Time correction: %+lld ms <<<\n", drift);
            }
        } else {
            Serial.println("WARNING: NTP sync failed, using existing time");
            if (sleep_get_time_ms() < 1700000000000ULL) {
                Serial.println("ERROR: No valid time available!");
            }
        }
    }
    
    // ================================================================
    // Common setup
    // ================================================================

    // Check memory availability
    Serial.println("Memory check:");
    Serial.printf("  Total heap: %d bytes\n", rp2040.getTotalHeap());
    Serial.printf("  Free heap:  %d bytes\n", rp2040.getFreeHeap());
    
    // Check PSRAM availability (critical for this display!)
    size_t psramSize = rp2040.getPSRAMSize();
    Serial.printf("  PSRAM size: %d bytes", psramSize);
    if (psramSize > 0) {
        Serial.printf(" (%d MB)\n", psramSize / (1024*1024));
        // Show PSRAM clock speed
        uint32_t sysClk = rp2040.f_cpu();
        Serial.printf("  System clock: %lu MHz\n", sysClk / 1000000);
        #ifdef RP2350_PSRAM_MAX_SCK_HZ
        Serial.printf("  PSRAM max: %d MHz (divisor ~%lu)\n", 
                      RP2350_PSRAM_MAX_SCK_HZ / 1000000,
                      (sysClk + RP2350_PSRAM_MAX_SCK_HZ - 1) / RP2350_PSRAM_MAX_SCK_HZ);
        #endif
    } else {
        Serial.println(" (NOT DETECTED!)");
        Serial.println("\n  WARNING: No PSRAM detected!");
        Serial.println("  This display requires ~2MB PSRAM for the frame buffer.");
    }
    
    // Quick test of pmalloc
    void* testPsram = pmalloc(1024);
    if (testPsram) {
        Serial.printf("  pmalloc test: OK at %p\n", testPsram);
        free(testPsram);
    } else {
        Serial.println("  pmalloc test: FAILED - PSRAM not working!");
    }
    
    Serial.println("\nPico Plus 2 W Pin Configuration:");
    Serial.printf("  SPI SCK:  GP%d\n", PIN_SPI_SCK);
    Serial.printf("  SPI MOSI: GP%d\n", PIN_SPI_MOSI);
    Serial.printf("  CS0:      GP%d\n", PIN_CS0);
    Serial.printf("  CS1:      GP%d\n", PIN_CS1);
    Serial.printf("  DC:       GP%d\n", PIN_DC);
    Serial.printf("  RESET:    GP%d\n", PIN_RESET);
    Serial.printf("  BUSY:     GP%d\n", PIN_BUSY);
    Serial.println();

    // Test: Read BUSY pin state before anything
    pinMode(PIN_BUSY, INPUT_PULLUP);
    Serial.printf("BUSY pin initial state: %s\n", digitalRead(PIN_BUSY) ? "HIGH" : "LOW");

    // Configure SPI1 pins BEFORE initializing display
    // arduino-pico requires pin configuration before SPI.begin()
    Serial.println("Configuring SPI1 pins...");
    SPI1.setSCK(PIN_SPI_SCK);
    SPI1.setTX(PIN_SPI_MOSI);
    Serial.println("SPI1 pins configured");

    // Initialize the display
    Serial.println("Initializing display...");
    if (!display.begin(PIN_CS0, PIN_CS1, PIN_DC, PIN_RESET, PIN_BUSY)) {
        Serial.println("ERROR: Display initialization failed!");
        Serial.println("Check wiring and connections.");
        while (1) {
            delay(1000);
        }
    }
    
    Serial.printf("Display initialized: %dx%d pixels\n", 
                  display.width(), display.height());
    
    // Initialize TTF font renderer
    ttf.begin(&display);
    if (ttf.loadFont(opensans_ttf, opensans_ttf_len)) {
        Serial.println("TTF font loaded successfully");
    } else {
        Serial.println("WARNING: TTF font failed to load");
    }
    Serial.println();

    // Do display update
    updateCount++;
    setUpdateCount(updateCount);
    doDisplayUpdate(updateCount);
    
    // Enter deep sleep for 10 seconds
    Serial.println("\n=== Entering deep sleep for 10 seconds ===");
    Serial.printf("RTC time: %lu seconds\n", sleep_get_uptime_seconds());
    Serial.println("Using RP2350 powman - TRUE deep sleep (core powers down)");
    
    Serial.flush();
    delay(100);
    
    // Prepare for deep sleep
    if (sleep_has_rtc()) {
        // DS3231 handles wake via alarm - no LPOSC setup needed
        Serial.println("Using DS3231 RTC for wake timing");
    } else {
        // LPOSC fallback - need to configure powman timer
        Serial.println("Using LPOSC for wake timing (preparing timer...)");
        sleep_run_from_lposc();
    }
    
    // Go to deep sleep for 10 seconds
    sleep_goto_dormant_for_ms(10000);
    
    // We should never reach here
    Serial.println("ERROR: Should not reach here after deep sleep!");
    while(1) delay(1000);
}

// ================================================================
// Perform a display update (called on each wake cycle)
// ================================================================
// Expected time for FULL display update cycle (from reading time to display complete)
// This includes: drawing (~0.5s) + rotate/pack (~0.7s) + SPI (~0.5s) + panel refresh (~20-32s)
// Cold boot (with init):  ~35 seconds (init 1.5s + panel refresh is slower first time)
// Warm update (skipInit): ~26 seconds (no init, faster refresh)
#define DISPLAY_REFRESH_COLD_MS  35000  // First update after power-on
#define DISPLAY_REFRESH_WARM_MS  26000  // Subsequent updates (skipInit=true)

void doDisplayUpdate(int updateNumber) {
    Serial.printf("\n=== Display Update #%d ===\n", updateNumber);
    
    // Get current time with drift correction applied
    uint64_t now_ms = sleep_get_corrected_time_ms();
    char timeStr[32];
    formatTime(now_ms, timeStr, sizeof(timeStr));
    Serial.printf("Drift correction: %d ppm\n", sleep_get_drift_ppm());
    Serial.printf("Current time: %s\n", timeStr);
    
    // Predict what time it will be when the display finishes refreshing
    // First update needs init sequence (~1.7s extra), subsequent updates can skip it
    bool isColdBoot = (updateNumber == 1);
    uint32_t expectedRefreshMs = isColdBoot ? DISPLAY_REFRESH_COLD_MS : DISPLAY_REFRESH_WARM_MS;
    
    uint64_t display_time_ms = now_ms + expectedRefreshMs;
    char displayTimeStr[32];
    formatTime(display_time_ms, displayTimeStr, sizeof(displayTimeStr));
    Serial.printf("Display will show: %s (compensating +%lu ms, %s)\n", 
                  displayTimeStr, expectedRefreshMs, 
                  isColdBoot ? "cold boot" : "warm update");
    
    // Reinitialize SPI
    SPI1.setSCK(PIN_SPI_SCK);
    SPI1.setTX(PIN_SPI_MOSI);
    SPI1.begin();
    
    // Initialize display
    if (!display.begin(PIN_CS0, PIN_CS1, PIN_DC, PIN_RESET, PIN_BUSY)) {
        Serial.println("ERROR: Display initialization failed!");
        return;
    }
    
    // Reinitialize TTF (display was reinitialized)
    ttf.begin(&display);
    ttf.loadFont(opensans_ttf, opensans_ttf_len);
    
    // Enable glyph cache for time display (160px digits)
    // This pre-renders 0-9, colon, space - used repeatedly
    ttf.enableGlyphCache(160.0, "0123456789: ");
    
    // Draw update info with performance profiling
    uint32_t drawStart = millis();
    uint32_t t0, t1;
    uint32_t ttfTotal = 0, bitmapTotal = 0;
    
    t0 = millis();
    display.clear(EL133UF1_WHITE);
    Serial.printf("  clear:          %lu ms\n", millis() - t0);
    
    // ================================================================
    // MAIN TIME DISPLAY - centered with outline for readability
    // ================================================================
    
    // Display the PREDICTED time (what it will be when refresh completes)
    time_t time_sec = (time_t)(display_time_ms / 1000);
    struct tm* tm = gmtime(&time_sec);
    char timeBuf[16];
    strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", tm);
    
    // Cycle through background colors based on update number
    uint8_t colors[] = {EL133UF1_RED, EL133UF1_GREEN, EL133UF1_BLUE, EL133UF1_YELLOW};
    uint8_t bgColor = colors[(updateNumber - 1) % 4];
    
    // Large colored banner for time
    t0 = millis();
    display.fillRect(0, 80, display.width(), 220, bgColor);
    Serial.printf("  fillRect:       %lu ms\n", millis() - t0);
    
    // Time - large outlined text, centered on anchor point
    // Using anchor at center of banner (800, 190)
    t0 = millis();
    ttf.drawTextAlignedOutlined(display.width() / 2, 190, timeBuf, 160.0,
                                 EL133UF1_WHITE, EL133UF1_BLACK,
                                 ALIGN_CENTER, ALIGN_MIDDLE, 2);
    t1 = millis() - t0;
    ttfTotal += t1;
    Serial.printf("  TTF time 160px: %lu ms\n", t1);
    
    // Date - centered below banner
    char dateBuf[32];
    strftime(dateBuf, sizeof(dateBuf), "%A, %d %B %Y", tm);
    t0 = millis();
    ttf.drawTextAligned(display.width() / 2, 340, dateBuf, 48.0, EL133UF1_BLACK,
                        ALIGN_CENTER, ALIGN_TOP);
    t1 = millis() - t0;
    ttfTotal += t1;
    Serial.printf("  TTF date 48px:  %lu ms\n", t1);
    
    // Update count
    char buf[64];
    snprintf(buf, sizeof(buf), "Update #%d", updateNumber);
    t0 = millis();
    ttf.drawTextAligned(display.width() / 2, 410, buf, 36.0, EL133UF1_BLACK,
                        ALIGN_CENTER, ALIGN_TOP);
    t1 = millis() - t0;
    ttfTotal += t1;
    Serial.printf("  TTF count 36px: %lu ms\n", t1);
    
    // ================================================================
    // ALIGNMENT DEMO - show anchor points and alignment modes
    // ================================================================
    
    int16_t demoY = 500;
    int16_t anchorX = display.width() / 2;  // Center of screen
    
    // Draw vertical anchor line
    display.drawVLine(anchorX, demoY, 180, EL133UF1_BLACK);
    
    // Draw horizontal baseline indicators
    int16_t baselineY = demoY + 90;
    display.drawHLine(anchorX - 300, baselineY, 600, EL133UF1_BLACK);
    
    // Small marker at anchor point
    display.fillRect(anchorX - 3, baselineY - 3, 6, 6, EL133UF1_RED);
    
    // Left-aligned text (anchor on left edge, baseline)
    t0 = millis();
    ttf.drawTextAligned(anchorX - 280, baselineY, "Left", 32.0, EL133UF1_BLACK,
                        ALIGN_LEFT, ALIGN_BASELINE);
    t1 = millis() - t0;
    ttfTotal += t1;
    
    // Center-aligned text (anchor at center, baseline)
    t0 = millis();
    ttf.drawTextAligned(anchorX, baselineY, "Center", 32.0, EL133UF1_RED,
                        ALIGN_CENTER, ALIGN_BASELINE);
    t1 = millis() - t0;
    ttfTotal += t1;
    
    // Right-aligned text (anchor on right edge, baseline)
    t0 = millis();
    ttf.drawTextAligned(anchorX + 280, baselineY, "Right", 32.0, EL133UF1_BLACK,
                        ALIGN_RIGHT, ALIGN_BASELINE);
    t1 = millis() - t0;
    ttfTotal += t1;
    Serial.printf("  Alignment demo: %lu ms\n", t1 * 3);
    
    // ================================================================
    // VERTICAL ALIGNMENT DEMO - showing descenders
    // ================================================================
    
    int16_t vdemoX = 200;
    int16_t vdemoY = 750;
    
    // Draw anchor line
    display.drawHLine(vdemoX, vdemoY, 500, EL133UF1_RED);
    display.fillRect(vdemoX - 3, vdemoY - 3, 6, 6, EL133UF1_RED);
    
    // Demonstrate baseline alignment with descenders (g, y, p)
    t0 = millis();
    ttf.drawTextAligned(vdemoX, vdemoY, "gyp Baseline", 36.0, EL133UF1_BLACK,
                        ALIGN_LEFT, ALIGN_BASELINE);
    t1 = millis() - t0;
    ttfTotal += t1;
    
    // Show different vertical alignments side by side
    int16_t vX2 = 900;
    display.drawHLine(vX2, vdemoY, 400, EL133UF1_BLUE);
    
    ttf.drawTextAligned(vX2, vdemoY, "Top", 28.0, EL133UF1_BLUE,
                        ALIGN_LEFT, ALIGN_TOP);
    ttf.drawTextAligned(vX2 + 100, vdemoY, "Mid", 28.0, EL133UF1_GREEN,
                        ALIGN_LEFT, ALIGN_MIDDLE);
    ttf.drawTextAligned(vX2 + 200, vdemoY, "Base", 28.0, EL133UF1_RED,
                        ALIGN_LEFT, ALIGN_BASELINE);
    ttf.drawTextAligned(vX2 + 320, vdemoY, "Bot", 28.0, EL133UF1_BLACK,
                        ALIGN_LEFT, ALIGN_BOTTOM);
    Serial.printf("  V-align demo:   %lu ms\n", millis() - t0);
    
    // ================================================================
    // OUTLINED TEXT DEMO
    // ================================================================
    
    // Gradient background for outline demo
    for (int i = 0; i < 6; i++) {
        display.fillRect(0, 850 + i * 25, display.width(), 25, i);
    }
    
    t0 = millis();
    ttf.drawTextAlignedOutlined(display.width() / 2, 925, "Outlined Text on Any Background", 40.0,
                                 EL133UF1_WHITE, EL133UF1_BLACK,
                                 ALIGN_CENTER, ALIGN_MIDDLE, 1);
    t1 = millis() - t0;
    ttfTotal += t1;
    Serial.printf("  Outlined demo:  %lu ms\n", t1);
    
    // ================================================================
    // INFO FOOTER
    // ================================================================
    
    t0 = millis();
    
    // Line 1: Time source info
    if (sleep_has_rtc()) {
        // DS3231 RTC present
        snprintf(buf, sizeof(buf), "DS3231 RTC: crystal accurate (~2ppm), battery-backed");
    } else {
        // LPOSC fallback
        uint32_t lposcFreq = sleep_get_lposc_freq_hz();
        int32_t lposcDev = sleep_get_lposc_deviation_centipercent();
        if (lposcFreq > 0) {
            snprintf(buf, sizeof(buf), "LPOSC: %lu Hz (%+ld.%02ld%% from 32768)", 
                     lposcFreq, lposcDev / 100, abs(lposcDev) % 100);
        } else {
            snprintf(buf, sizeof(buf), "LPOSC: not calibrated");
        }
    }
    ttf.drawTextAligned(display.width() / 2, 1020, buf, 22.0, EL133UF1_BLACK,
                        ALIGN_CENTER, ALIGN_TOP);
    
    // Line 2: Status
    if (sleep_has_rtc()) {
        snprintf(buf, sizeof(buf), "Wake: DS3231 alarm | Sleep: 10s | Update #%d", updateNumber);
    } else {
        int32_t driftPpm = sleep_get_drift_ppm();
        snprintf(buf, sizeof(buf), "Drift: %+ld ppm | Sleep: 10s | Update #%d", 
                 (long)driftPpm, updateNumber);
    }
    ttf.drawTextAligned(display.width() / 2, 1055, buf, 22.0, EL133UF1_BLACK,
                        ALIGN_CENTER, ALIGN_TOP);
    ttfTotal += millis() - t0;

    // Line 3: Status
    const char* statusMsg = sleep_has_rtc() 
        ? "DS3231 RTC maintains time during deep sleep"
        : "NTP synced on boot, time maintained during deep sleep";
    ttf.drawTextAligned(display.width() / 2, 1090, statusMsg, 
                        20.0, EL133UF1_BLACK, ALIGN_CENTER, ALIGN_TOP);
    
    // Version/tech info - right aligned at bottom
    ttf.drawTextAligned(display.width() - 20, 1150, "RP2350 + EL133UF1 + Open Sans TTF", 
                        18.0, EL133UF1_BLACK, ALIGN_RIGHT, ALIGN_BOTTOM);
    
    Serial.printf("--- Drawing summary ---\n");
    Serial.printf("  TTF total:      %lu ms\n", ttfTotal);
    Serial.printf("  Bitmap total:   %lu ms\n", bitmapTotal);
    Serial.printf("  All drawing:    %lu ms\n", millis() - drawStart);
    
    // Update display and measure actual refresh time
    // Skip init sequence on warm updates (saves ~1.7 seconds)
    uint32_t refreshStart = millis();
    display.update(!isColdBoot);  // skipInit=true for warm updates
    uint32_t actualRefreshMs = millis() - refreshStart;
    
    // Get actual time now for comparison (with drift correction)
    uint64_t actual_now_ms = sleep_get_corrected_time_ms();
    char actualTimeStr[32];
    formatTime(actual_now_ms, actualTimeStr, sizeof(actualTimeStr));
    
    Serial.printf("Update #%d complete.\n", updateNumber);
    Serial.printf("  Displayed time: %s\n", displayTimeStr);
    Serial.printf("  Actual time:    %s\n", actualTimeStr);
    Serial.printf("  Refresh took:   %lu ms (predicted %lu ms)\n", 
                  actualRefreshMs, expectedRefreshMs);
    
    int32_t errorMs = (int32_t)(actual_now_ms - display_time_ms);
    const char* accuracy = (abs(errorMs) < 2000) ? "excellent" : 
                           (abs(errorMs) < 5000) ? "good" : "acceptable";
    Serial.printf("  Display vs actual: %+ld ms (%s)\n", errorMs, accuracy);
}

void loop() {
    // Nothing to do in loop for this demo
    delay(10000);
}

/**
 * @brief Draw a demonstration pattern showing all 6 colors
 */
void drawDemoPattern() {
    Serial.println("Drawing orientation test pattern...");
    
    const uint16_t w = display.width();   // 1600
    const uint16_t h = display.height();  // 1200
    
    // Clear to white
    display.clear(EL133UF1_WHITE);
    
    // Draw a black border around the whole display
    for (int i = 0; i < 5; i++) {
        display.drawRect(i, i, w - 2*i, h - 2*i, EL133UF1_BLACK);
    }
    
    // Text size for corner labels (size 6 = 48x48 pixels per char)
    const uint8_t textSize = 6;
    const uint16_t charW = 8 * textSize;  // 48 pixels per character
    const uint16_t charH = 8 * textSize;  // 48 pixels tall
    const uint16_t margin = 30;
    
    // Top-Left corner label
    display.fillRect(margin, margin, charW * 8 + 20, charH + 20, EL133UF1_WHITE);
    display.drawRect(margin, margin, charW * 8 + 20, charH + 20, EL133UF1_BLACK);
    display.drawText(margin + 10, margin + 10, "TOP-LEFT", EL133UF1_BLACK, EL133UF1_WHITE, textSize);
    
    // Top-Right corner label  
    uint16_t trX = w - margin - (charW * 9 + 20);
    display.fillRect(trX, margin, charW * 9 + 20, charH + 20, EL133UF1_WHITE);
    display.drawRect(trX, margin, charW * 9 + 20, charH + 20, EL133UF1_BLACK);
    display.drawText(trX + 10, margin + 10, "TOP-RIGHT", EL133UF1_BLACK, EL133UF1_WHITE, textSize);
    
    // Bottom-Left corner label
    uint16_t blY = h - margin - (charH + 20);
    display.fillRect(margin, blY, charW * 11 + 20, charH + 20, EL133UF1_WHITE);
    display.drawRect(margin, blY, charW * 11 + 20, charH + 20, EL133UF1_BLACK);
    display.drawText(margin + 10, blY + 10, "BOTTOM-LEFT", EL133UF1_BLACK, EL133UF1_WHITE, textSize);
    
    // Bottom-Right corner label
    uint16_t brX = w - margin - (charW * 12 + 20);
    display.fillRect(brX, blY, charW * 12 + 20, charH + 20, EL133UF1_WHITE);
    display.drawRect(brX, blY, charW * 12 + 20, charH + 20, EL133UF1_BLACK);
    display.drawText(brX + 10, blY + 10, "BOTTOM-RIGHT", EL133UF1_BLACK, EL133UF1_WHITE, textSize);
    
    // Draw colored corners to make orientation obvious
    // Top-left: RED square
    display.fillRect(margin, margin + charH + 40, 100, 100, EL133UF1_RED);
    display.drawText(margin, margin + charH + 150, "RED", EL133UF1_RED, EL133UF1_WHITE, 3);
    
    // Top-right: BLUE square
    display.fillRect(w - margin - 100, margin + charH + 40, 100, 100, EL133UF1_BLUE);
    display.drawText(w - margin - 100, margin + charH + 150, "BLUE", EL133UF1_BLUE, EL133UF1_WHITE, 3);
    
    // Bottom-left: GREEN square
    display.fillRect(margin, blY - 150, 100, 100, EL133UF1_GREEN);
    display.drawText(margin, blY - 170, "GREEN", EL133UF1_GREEN, EL133UF1_WHITE, 3);
    
    // Bottom-right: YELLOW square
    display.fillRect(w - margin - 100, blY - 150, 100, 100, EL133UF1_YELLOW);
    display.drawText(w - margin - 140, blY - 170, "YELLOW", EL133UF1_YELLOW, EL133UF1_WHITE, 3);
    
    // Center info
    const char* centerText1 = "EL133UF1 Display";
    const char* centerText2 = "1600 x 1200 pixels";
    uint16_t cx = w / 2;
    uint16_t cy = h / 2;
    
    display.drawText(cx - (16 * 8 * 4) / 2, cy - 50, centerText1, EL133UF1_BLACK, EL133UF1_WHITE, 4);
    display.drawText(cx - (18 * 8 * 3) / 2, cy + 30, centerText2, EL133UF1_BLACK, EL133UF1_WHITE, 3);
    
    // Draw arrows pointing to edges
    // Arrow pointing UP at top center
    int16_t arrowX = cx;
    int16_t arrowY = 150;
    for (int i = 0; i < 30; i++) {
        display.drawHLine(arrowX - i, arrowY + i, i * 2 + 1, EL133UF1_BLACK);
    }
    display.drawText(arrowX - 24, arrowY + 40, "UP", EL133UF1_BLACK, EL133UF1_WHITE, 3);
    
    Serial.println("Orientation test pattern drawn to buffer");
}
