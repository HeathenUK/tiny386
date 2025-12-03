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
#include <time.h>
#include "EL133UF1.h"
#include "pico_sleep.h"
#include "hardware/structs/powman.h"

// Pico SDK WiFi/network includes
#include "pico/cyw43_arch.h"
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/timeouts.h"

// WiFi credentials
#define WIFI_SSID "JELLING"
#define WIFI_PSK "Crusty jugglers"

// NTP settings
#define NTP_SERVER "pool.ntp.org"
#define NTP_PORT 123
#define NTP_DELTA 2208988800ULL  // Seconds between 1900 and 1970

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

// Forward declarations
void drawDemoPattern();
bool connectWiFiAndGetNTP();
void formatTime(uint64_t time_ms, char* buf, size_t len);

// ================================================================
// NTP packet structure
// ================================================================
typedef struct {
    uint8_t li_vn_mode;
    uint8_t stratum;
    uint8_t poll;
    uint8_t precision;
    uint32_t root_delay;
    uint32_t root_dispersion;
    uint32_t ref_id;
    uint32_t ref_ts_sec;
    uint32_t ref_ts_frac;
    uint32_t orig_ts_sec;
    uint32_t orig_ts_frac;
    uint32_t rx_ts_sec;
    uint32_t rx_ts_frac;
    uint32_t tx_ts_sec;
    uint32_t tx_ts_frac;
} ntp_packet_t;

static volatile bool ntp_done = false;
static volatile time_t ntp_time = 0;

// NTP response callback
static void ntp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                               const ip_addr_t *addr, u16_t port) {
    if (p->tot_len >= sizeof(ntp_packet_t)) {
        ntp_packet_t packet;
        pbuf_copy_partial(p, &packet, sizeof(packet), 0);
        
        // Extract transmit timestamp (seconds since 1900)
        uint32_t tx_sec = lwip_ntohl(packet.tx_ts_sec);
        
        // Convert to Unix epoch (seconds since 1970)
        ntp_time = (time_t)(tx_sec - NTP_DELTA);
        ntp_done = true;
    }
    pbuf_free(p);
}

// DNS callback
static ip_addr_t ntp_server_addr;
static volatile bool dns_done = false;

static void dns_callback(const char *name, const ip_addr_t *addr, void *arg) {
    if (addr) {
        ntp_server_addr = *addr;
    }
    dns_done = true;
}

// ================================================================
// Connect to WiFi and sync NTP time (pico-sdk approach)
// ================================================================
bool connectWiFiAndGetNTP() {
    Serial.println("\n=== Connecting to WiFi (pico-sdk) ===");
    Serial.printf("SSID: %s\n", WIFI_SSID);
    
    // Initialize CYW43
    if (cyw43_arch_init()) {
        Serial.println("CYW43 init failed!");
        return false;
    }
    
    cyw43_arch_enable_sta_mode();
    
    // Connect to WiFi
    Serial.println("Connecting...");
    int result = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PSK, 
                                                      CYW43_AUTH_WPA2_AES_PSK, 15000);
    if (result != 0) {
        Serial.printf("WiFi connect failed: %d\n", result);
        cyw43_arch_deinit();
        return false;
    }
    
    Serial.println("WiFi connected!");
    Serial.printf("IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    
    // Resolve NTP server
    Serial.println("\n=== Getting NTP time ===");
    Serial.printf("Resolving %s...\n", NTP_SERVER);
    
    dns_done = false;
    ip_addr_set_zero(&ntp_server_addr);
    
    err_t err = dns_gethostbyname(NTP_SERVER, &ntp_server_addr, dns_callback, NULL);
    if (err == ERR_INPROGRESS) {
        // Wait for DNS
        uint32_t start = millis();
        while (!dns_done && (millis() - start < 5000)) {
            cyw43_arch_poll();
            delay(10);
        }
    } else if (err != ERR_OK) {
        Serial.println("DNS lookup failed!");
        cyw43_arch_deinit();
        return false;
    }
    
    if (ip_addr_isany(&ntp_server_addr)) {
        Serial.println("Could not resolve NTP server!");
        cyw43_arch_deinit();
        return false;
    }
    
    Serial.printf("NTP server: %s\n", ipaddr_ntoa(&ntp_server_addr));
    
    // Create UDP socket for NTP
    struct udp_pcb *pcb = udp_new();
    if (!pcb) {
        Serial.println("Failed to create UDP PCB!");
        cyw43_arch_deinit();
        return false;
    }
    
    udp_recv(pcb, ntp_recv_callback, NULL);
    
    // Create NTP request packet
    ntp_packet_t packet;
    memset(&packet, 0, sizeof(packet));
    packet.li_vn_mode = (0 << 6) | (4 << 3) | 3;  // LI=0, VN=4, Mode=3 (client)
    
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(packet), PBUF_RAM);
    if (!p) {
        Serial.println("Failed to allocate pbuf!");
        udp_remove(pcb);
        cyw43_arch_deinit();
        return false;
    }
    
    memcpy(p->payload, &packet, sizeof(packet));
    
    // Send NTP request
    ntp_done = false;
    err = udp_sendto(pcb, p, &ntp_server_addr, NTP_PORT);
    pbuf_free(p);
    
    if (err != ERR_OK) {
        Serial.printf("UDP send failed: %d\n", err);
        udp_remove(pcb);
        cyw43_arch_deinit();
        return false;
    }
    
    // Wait for response
    Serial.print("Waiting for NTP response");
    uint32_t start = millis();
    while (!ntp_done && (millis() - start < 5000)) {
        cyw43_arch_poll();
        Serial.print(".");
        delay(100);
    }
    Serial.println();
    
    udp_remove(pcb);
    
    if (!ntp_done) {
        Serial.println("NTP timeout!");
        cyw43_arch_deinit();
        return false;
    }
    
    // Got NTP time!
    time_t received_time = ntp_time;  // Copy from volatile
    uint64_t now_ms = (uint64_t)received_time * 1000;
    sleep_set_time_ms(now_ms);
    
    struct tm* timeinfo = gmtime(&received_time);
    char buf[64];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S UTC", timeinfo);
    Serial.printf("NTP time: %s\n", buf);
    Serial.printf("Epoch: %lld\n", (long long)ntp_time);
    
    // Deinit WiFi to save power
    cyw43_arch_deinit();
    Serial.println("WiFi deinitialized (saving power)");
    
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
    if (sleep_woke_from_deep_sleep()) {
        Serial.println("\n\n========================================");
        Serial.printf("*** WOKE FROM DEEP SLEEP! (update #%d) ***\n", updateCount + 1);
        Serial.printf("*** RTC uptime: %lu seconds ***\n", uptime);
        Serial.println("========================================\n");
        
        // Clear the wake flag
        sleep_clear_wake_flag();
    } else {
        // First boot
        Serial.println("\n\n===========================================");
        Serial.println("EL133UF1 13.3\" Spectra 6 E-Ink Display Demo");
        Serial.println("===========================================\n");
        
        // Connect to WiFi and get NTP time
        if (!connectWiFiAndGetNTP()) {
            Serial.println("WARNING: Using local time (starting from 0)");
            sleep_set_time_ms(0);
        }
        
        setUpdateCount(0);
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
void doDisplayUpdate(int updateNumber) {
    Serial.printf("\n=== Display Update #%d ===\n", updateNumber);
    
    // Get current time from powman timer (persists across sleep!)
    uint64_t now_ms = sleep_get_time_ms();
    char timeStr[32];
    formatTime(now_ms, timeStr, sizeof(timeStr));
    
    Serial.printf("Current time: %s\n", timeStr);
    
    // Reinitialize SPI
    SPI1.setSCK(PIN_SPI_SCK);
    SPI1.setTX(PIN_SPI_MOSI);
    SPI1.begin();
    
    // Initialize display
    if (!display.begin(PIN_CS0, PIN_CS1, PIN_DC, PIN_RESET, PIN_BUSY)) {
        Serial.println("ERROR: Display initialization failed!");
        return;
    }
    
    // Draw update info
    uint32_t drawStart = millis();
    
    display.clear(EL133UF1_WHITE);
    
    // Title
    display.drawText(250, 80, "RP2350 Deep Sleep Clock", EL133UF1_BLACK, EL133UF1_WHITE, 4);
    
    // Current time - big display
    // Extract just the time portion
    time_t time_sec = (time_t)(now_ms / 1000);
    struct tm* tm = gmtime(&time_sec);
    char timeBuf[16];
    strftime(timeBuf, sizeof(timeBuf), "%H:%M:%S", tm);
    
    // Cycle through colors based on update number
    uint8_t colors[] = {EL133UF1_RED, EL133UF1_GREEN, EL133UF1_BLUE, EL133UF1_YELLOW};
    uint8_t color = colors[(updateNumber - 1) % 4];
    
    display.fillRect(150, 200, 900, 200, color);
    display.drawText(200, 250, timeBuf, EL133UF1_WHITE, color, 10);
    
    // Date
    char dateBuf[32];
    strftime(dateBuf, sizeof(dateBuf), "%A, %d %B %Y", tm);
    display.drawText(300, 450, dateBuf, EL133UF1_BLACK, EL133UF1_WHITE, 3);
    
    // Update count
    char buf[32];
    snprintf(buf, sizeof(buf), "Update #%d", updateNumber);
    display.drawText(550, 550, buf, EL133UF1_BLACK, EL133UF1_WHITE, 4);
    
    // Info
    display.drawText(200, 700, "NTP synced on boot, time maintained during sleep", EL133UF1_BLACK, EL133UF1_WHITE, 2);
    display.drawText(250, 780, "Next update in 10 seconds (deep sleep)", EL133UF1_BLACK, EL133UF1_WHITE, 2);
    
    Serial.printf("Drawing took: %lu ms\n", millis() - drawStart);
    
    // Update display
    display.update(false);
    
    Serial.printf("Update #%d complete. Time: %s\n", updateNumber, timeStr);
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
