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
#include "hardware/structs/powman.h"

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
    Serial.printf("Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("DNS (DHCP): %s\n", WiFi.dnsIP().toString().c_str());
    
    // Override DNS with Cloudflare (1.1.1.1) and Google (8.8.8.8)
    IPAddress cloudflare(1, 1, 1, 1);
    IPAddress google(8, 8, 8, 8);
    WiFi.setDNS(cloudflare, google);
    Serial.printf("DNS (override): %s, %s\n", cloudflare.toString().c_str(), google.toString().c_str());
    
    // Small delay to let network stack stabilize
    delay(500);
    
    // Use arduino-pico's NTP class
    Serial.println("\n=== Getting NTP time ===");
    
    // Try DNS resolution first
    IPAddress ntpServer1, ntpServer2;
    bool dns1 = WiFi.hostByName(NTP_SERVER1_NAME, ntpServer1);
    bool dns2 = WiFi.hostByName(NTP_SERVER2_NAME, ntpServer2);
    
    Serial.printf("DNS %s: %s -> %s\n", NTP_SERVER1_NAME, dns1 ? "OK" : "FAIL", 
                  dns1 ? ntpServer1.toString().c_str() : "N/A");
    Serial.printf("DNS %s: %s -> %s\n", NTP_SERVER2_NAME, dns2 ? "OK" : "FAIL",
                  dns2 ? ntpServer2.toString().c_str() : "N/A");
    
    // If DNS failed, use hardcoded IPs (Google's NTP servers are very reliable)
    if (!dns1) {
        ntpServer1 = IPAddress(216, 239, 35, 0);  // time.google.com
        Serial.printf("Using fallback IP: %s\n", ntpServer1.toString().c_str());
    }
    if (!dns2) {
        ntpServer2 = IPAddress(216, 239, 35, 4);  // time2.google.com
    }
    
    // Initialize NTP - use IPAddress overload to ensure sntp_init() is called
    NTP.begin(ntpServer1, ntpServer2);
    
    // Give SNTP time to initialize and send first request
    Serial.println("NTP initialized, waiting for response...");
    delay(1000);
    
    // Wait for valid time with better feedback
    Serial.print("Syncing: ");
    time_t now = time(nullptr);
    start = millis();
    int attempts = 0;
    const int maxAttempts = 60;  // 60 seconds max
    
    while (now < 1700000000 && attempts < maxAttempts) {  // Valid if > Sept 2023
        // Let network stack process - yield() is important for lwIP
        for (int i = 0; i < 10; i++) {
            delay(100);
            yield();  // Allow background network processing
        }
        attempts++;
        now = time(nullptr);
        
        if (attempts % 5 == 0) {
            Serial.printf("[%ds: %lld] ", attempts, (long long)now);
        } else {
            Serial.print(".");
        }
    }
    Serial.println();
    
    Serial.printf("Final time: %lld (valid threshold: 1700000000)\n", (long long)now);
    Serial.printf("NTP attempts: %d seconds\n", attempts);
    
    if (now < 1700000000) {  // Sept 2023
        Serial.println("NTP sync failed - time not set!");
        Serial.println("Possible causes:");
        Serial.println("  - Firewall blocking UDP port 123");
        Serial.println("  - NTP servers unreachable");
        Serial.println("  - Network issues");
        WiFi.disconnect(true);
        return false;
    }
    
    // Got NTP time!
    uint64_t now_ms = (uint64_t)now * 1000;
    sleep_set_time_ms(now_ms);
    
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    Serial.print("Current time: ");
    Serial.println(asctime(&timeinfo));
    Serial.printf("Epoch: %lld\n", (long long)now);
    
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

// Drift compensation (stored in scratch register 2)
// Stored as: (drift_ppm + 500000) to handle negative values
// e.g., -5% drift = -50000 ppm, stored as 450000
#define DRIFT_PPM_REG 2
#define DRIFT_PPM_OFFSET 500000  // Offset to handle negative values

// NTP resync interval - LPOSC can drift >5%, resync frequently
#define NTP_RESYNC_INTERVAL  5  // Resync every N updates (reduced from 10)

// Get/set drift compensation in PPM (parts per million)
// Negative = LPOSC running slow, Positive = LPOSC running fast
int32_t getDriftPPM() {
    uint32_t stored = powman_hw->scratch[DRIFT_PPM_REG];
    if (stored == 0) return 0;  // Not calibrated
    return (int32_t)(stored - DRIFT_PPM_OFFSET);
}

void setDriftPPM(int32_t ppm) {
    powman_hw->scratch[DRIFT_PPM_REG] = (uint32_t)(ppm + DRIFT_PPM_OFFSET);
}

// Apply drift compensation to a time value
uint64_t compensateForDrift(uint64_t raw_time_ms, uint64_t reference_time_ms) {
    int32_t ppm = getDriftPPM();
    if (ppm == 0) return raw_time_ms;  // No compensation
    
    // Calculate elapsed time since reference
    int64_t elapsed = (int64_t)(raw_time_ms - reference_time_ms);
    
    // Apply correction: corrected = raw + (elapsed * ppm / 1000000)
    int64_t correction = (elapsed * ppm) / 1000000;
    
    return (uint64_t)((int64_t)raw_time_ms + correction);
}

int getUpdateCount() {
    return (int)powman_hw->scratch[UPDATE_COUNT_REG];
}

void setUpdateCount(int count) {
    powman_hw->scratch[UPDATE_COUNT_REG] = (uint32_t)count;
}

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    
    // Wait for serial connection
    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait < 2000)) {
        delay(100);
    }
    
    // Get current update count and RTC time
    int updateCount = sleep_woke_from_deep_sleep() ? getUpdateCount() : 0;
    uint32_t uptime = sleep_get_uptime_seconds();
    
    // ================================================================
    // Check if we woke from deep sleep
    // ================================================================
    bool needsNtpSync = false;
    
    if (sleep_woke_from_deep_sleep()) {
        Serial.println("\n\n========================================");
        Serial.printf("*** WOKE FROM DEEP SLEEP! (update #%d) ***\n", updateCount + 1);
        Serial.printf("*** RTC uptime: %lu seconds ***\n", uptime);
        Serial.println("========================================\n");
        
        // Clear the wake flag
        sleep_clear_wake_flag();
        
        // Check if we need to resync NTP (LPOSC drifts over time)
        if ((updateCount + 1) % NTP_RESYNC_INTERVAL == 0) {
            Serial.println(">>> Periodic NTP resync to correct LPOSC drift <<<");
            needsNtpSync = true;
        }
    } else {
        // First boot
        Serial.println("\n\n===========================================");
        Serial.println("EL133UF1 13.3\" Spectra 6 E-Ink Display Demo");
        Serial.println("===========================================\n");
        
        needsNtpSync = true;
        setUpdateCount(0);
    }
    
    // Sync NTP if needed (cold boot or periodic resync)
    if (needsNtpSync) {
        uint64_t oldTime = sleep_get_time_ms();
        uint32_t syncStartMs = millis();
        
        if (connectWiFiAndGetNTP()) {
            uint64_t newTime = sleep_get_time_ms();
            uint32_t syncDurationMs = millis() - syncStartMs;
            
            // The sync itself takes time, so account for that
            // oldTime was captured before sync, newTime is "now" after sync
            // The drift is: (newTime - oldTime) - syncDuration
            // because oldTime should have advanced by syncDuration if LPOSC was perfect
            
            if (oldTime > 1700000000000ULL) {  // Valid old time (after Sept 2023)
                int64_t expectedAdvance = syncDurationMs;  // How much time should have passed
                int64_t actualAdvance = (int64_t)(newTime - oldTime);
                int64_t driftMs = actualAdvance - expectedAdvance;
                
                // But we also need to know how long since LAST NTP sync to calculate rate
                // For now, just report the correction
                Serial.printf(">>> Time correction: %+lld ms <<<\n", -driftMs);
                Serial.printf(">>> (LPOSC was %s by %lld ms during ~%lu ms awake) <<<\n",
                              driftMs > 0 ? "fast" : "slow", 
                              driftMs > 0 ? driftMs : -driftMs,
                              syncDurationMs);
                
                // Calculate drift in PPM based on time since last sync
                // We track total elapsed LPOSC time vs real time
                // For simplicity, assume updates happen every ~10s + refresh time (~30s)
                // So between syncs at interval 5: ~5 * 40s = 200s of real time
                // The drift observed is relative to that period
                
                // Store approximate drift rate if significant
                if (abs(driftMs) > 100) {  // More than 100ms drift is significant
                    // Rough estimate: assume 3 minutes between syncs
                    int64_t estimatedIntervalMs = 180000;  // 3 minutes
                    int32_t driftPPM = (int32_t)((driftMs * 1000000LL) / estimatedIntervalMs);
                    setDriftPPM(-driftPPM);  // Negative because we correct in opposite direction
                    Serial.printf(">>> Estimated drift rate: %+ld ppm (%+.2f%%) <<<\n", 
                                  (long)driftPPM, driftPPM / 10000.0);
                }
            }
        } else {
            Serial.println("WARNING: NTP sync failed, using existing time");
            if (sleep_get_time_ms() == 0) {
                sleep_set_time_ms(0);  // No time available
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
    
    // Prepare powman timer for deep sleep
    sleep_run_from_lposc();
    
    // Go to deep sleep for 10 seconds
    sleep_goto_dormant_for_ms(10000);
    
    // We should never reach here
    Serial.println("ERROR: Should not reach here after deep sleep!");
    while(1) delay(1000);
}

// ================================================================
// Perform a display update (called on each wake cycle)
// ================================================================
// Expected time for display refresh (measured empirically)
// Cold boot (with init):  ~23 seconds (init 1.7s + rotate 1s + SPI 0.5s + refresh 19.5s + overhead 0.3s)
// Warm update (skipInit): ~21 seconds (no init sequence)
#define DISPLAY_REFRESH_COLD_MS  23000  // First update after power-on
#define DISPLAY_REFRESH_WARM_MS  21300  // Subsequent updates (skipInit=true)

void doDisplayUpdate(int updateNumber) {
    Serial.printf("\n=== Display Update #%d ===\n", updateNumber);
    
    // Get current time from powman timer (persists across sleep!)
    uint64_t now_ms = sleep_get_time_ms();
    char timeStr[32];
    formatTime(now_ms, timeStr, sizeof(timeStr));
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
    
    // Draw update info with performance profiling
    uint32_t drawStart = millis();
    uint32_t t0, t1;
    uint32_t ttfTotal = 0, bitmapTotal = 0;
    
    t0 = millis();
    display.clear(EL133UF1_WHITE);
    Serial.printf("  clear:          %lu ms\n", millis() - t0);
    
    // Title - using TTF font (64px, 23 chars)
    t0 = millis();
    ttf.drawTextCentered(0, 60, display.width(), "RP2350 Deep Sleep Clock", 64.0, EL133UF1_BLACK);
    t1 = millis() - t0;
    ttfTotal += t1;
    Serial.printf("  TTF title 64px: %lu ms\n", t1);
    
    // Display the PREDICTED time (what it will be when refresh completes)
    time_t time_sec = (time_t)(display_time_ms / 1000);
    struct tm* tm = gmtime(&time_sec);
    char timeBuf[16];
    strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", tm);
    
    // Cycle through colors based on update number
    uint8_t colors[] = {EL133UF1_RED, EL133UF1_GREEN, EL133UF1_BLUE, EL133UF1_YELLOW};
    uint8_t color = colors[(updateNumber - 1) % 4];
    
    t0 = millis();
    display.fillRect(150, 200, 900, 200, color);
    Serial.printf("  fillRect:       %lu ms\n", millis() - t0);
    
    // Time display - large TTF (140px, 8 chars)
    t0 = millis();
    ttf.drawTextCentered(150, 230, 900, timeBuf, 140.0, EL133UF1_WHITE);
    t1 = millis() - t0;
    ttfTotal += t1;
    Serial.printf("  TTF time 140px: %lu ms\n", t1);
    
    // Date - TTF (42px, ~20 chars)
    char dateBuf[32];
    strftime(dateBuf, sizeof(dateBuf), "%A, %d %B %Y", tm);
    t0 = millis();
    ttf.drawTextCentered(0, 450, display.width(), dateBuf, 42.0, EL133UF1_BLACK);
    t1 = millis() - t0;
    ttfTotal += t1;
    Serial.printf("  TTF date 42px:  %lu ms\n", t1);
    
    // Update count - TTF (56px, ~10 chars)
    char buf[32];
    snprintf(buf, sizeof(buf), "Update #%d", updateNumber);
    t0 = millis();
    ttf.drawTextCentered(0, 530, display.width(), buf, 56.0, EL133UF1_BLACK);
    t1 = millis() - t0;
    ttfTotal += t1;
    Serial.printf("  TTF count 56px: %lu ms\n", t1);
    
    // Info line 1 - TTF (28px, 48 chars)
    t0 = millis();
    ttf.drawTextCentered(0, 650, display.width(), "NTP synced on boot, time maintained during sleep", 28.0, EL133UF1_BLACK);
    t1 = millis() - t0;
    ttfTotal += t1;
    Serial.printf("  TTF info1 28px: %lu ms\n", t1);
    
    // Info line 2 - TTF (28px, 38 chars)
    t0 = millis();
    ttf.drawTextCentered(0, 700, display.width(), "Next update in 10 seconds (deep sleep)", 28.0, EL133UF1_BLACK);
    t1 = millis() - t0;
    ttfTotal += t1;
    Serial.printf("  TTF info2 28px: %lu ms\n", t1);
    
    // Bitmap font comparison (size 2 = 16px, 24 chars)
    t0 = millis();
    display.drawText(50, 800, "Bitmap font (8x8 scaled)", EL133UF1_BLACK, EL133UF1_WHITE, 2);
    t1 = millis() - t0;
    bitmapTotal += t1;
    Serial.printf("  Bitmap 16px:    %lu ms\n", t1);
    
    // TTF comparison (24px, 25 chars)
    t0 = millis();
    ttf.drawText(50, 850, "TrueType font (Open Sans)", 24.0, EL133UF1_BLACK);
    t1 = millis() - t0;
    ttfTotal += t1;
    Serial.printf("  TTF 24px:       %lu ms\n", t1);
    
    Serial.printf("--- Drawing summary ---\n");
    Serial.printf("  TTF total:      %lu ms\n", ttfTotal);
    Serial.printf("  Bitmap total:   %lu ms\n", bitmapTotal);
    Serial.printf("  All drawing:    %lu ms\n", millis() - drawStart);
    
    // Update display and measure actual refresh time
    // Skip init sequence on warm updates (saves ~1.7 seconds)
    uint32_t refreshStart = millis();
    display.update(!isColdBoot);  // skipInit=true for warm updates
    uint32_t actualRefreshMs = millis() - refreshStart;
    
    // Get actual time now for comparison
    uint64_t actual_now_ms = sleep_get_time_ms();
    char actualTimeStr[32];
    formatTime(actual_now_ms, actualTimeStr, sizeof(actualTimeStr));
    
    Serial.printf("Update #%d complete.\n", updateNumber);
    Serial.printf("  Displayed time: %s\n", displayTimeStr);
    Serial.printf("  Actual time:    %s\n", actualTimeStr);
    Serial.printf("  Refresh took:   %lu ms (predicted %lu ms)\n", 
                  actualRefreshMs, expectedRefreshMs);
    
    int32_t errorMs = (int32_t)(actual_now_ms - display_time_ms);
    Serial.printf("  Time error:     %+ld ms (%s)\n", errorMs,
                  abs(errorMs) < 2000 ? "good" : "adjust DISPLAY_REFRESH_MS");
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
