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
#include "EL133UF1.h"

// Pin definitions for Pico Plus 2 W
// Using SPI0 with custom pin mapping
#define PIN_SPI_SCK   18    // SPI0 SCK
#define PIN_SPI_MOSI  19    // SPI0 TX (MOSI)
#define PIN_CS0       17    // Chip Select 0
#define PIN_CS1       16    // Chip Select 1
#define PIN_DC        20    // Data/Command
#define PIN_RESET     21    // Reset
#define PIN_BUSY      22    // Busy

// Create display instance using default SPI (SPI0)
EL133UF1 display(&SPI);

// Forward declaration
void drawDemoPattern();

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    
    // Wait for serial connection (useful for debugging)
    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait < 3000)) {
        delay(100);
    }
    
    Serial.println("\n\n===========================================");
    Serial.println("EL133UF1 13.3\" Spectra 6 E-Ink Display Demo");
    Serial.println("===========================================\n");
    
    Serial.println("Pico Plus 2 W Pin Configuration:");
    Serial.printf("  SPI SCK:  GP%d\n", PIN_SPI_SCK);
    Serial.printf("  SPI MOSI: GP%d\n", PIN_SPI_MOSI);
    Serial.printf("  CS0:      GP%d\n", PIN_CS0);
    Serial.printf("  CS1:      GP%d\n", PIN_CS1);
    Serial.printf("  DC:       GP%d\n", PIN_DC);
    Serial.printf("  RESET:    GP%d\n", PIN_RESET);
    Serial.printf("  BUSY:     GP%d\n", PIN_BUSY);
    Serial.println();

    // Configure SPI pins BEFORE initializing display
    // arduino-pico requires pin configuration before SPI.begin()
    SPI.setSCK(PIN_SPI_SCK);
    SPI.setTX(PIN_SPI_MOSI);

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

    // Draw demo pattern
    drawDemoPattern();
    
    // Update the display
    Serial.println("Updating display (this will take ~30 seconds)...");
    display.update();
    
    Serial.println("\nDemo complete!");
}

void loop() {
    // Nothing to do in loop for this demo
    delay(10000);
}

/**
 * @brief Draw a demonstration pattern showing all 6 colors
 */
void drawDemoPattern() {
    Serial.println("Drawing demo pattern...");
    
    const uint16_t w = display.width();
    const uint16_t h = display.height();
    
    // Clear to white
    display.clear(EL133UF1_WHITE);
    
    // Draw color bars across the top third
    const uint16_t barHeight = h / 4;
    const uint16_t barWidth = w / 6;
    
    display.fillRect(0 * barWidth, 0, barWidth, barHeight, EL133UF1_BLACK);
    display.fillRect(1 * barWidth, 0, barWidth, barHeight, EL133UF1_WHITE);
    display.fillRect(2 * barWidth, 0, barWidth, barHeight, EL133UF1_RED);
    display.fillRect(3 * barWidth, 0, barWidth, barHeight, EL133UF1_YELLOW);
    display.fillRect(4 * barWidth, 0, barWidth, barHeight, EL133UF1_GREEN);
    display.fillRect(5 * barWidth, 0, barWidth, barHeight, EL133UF1_BLUE);
    
    // Draw a black border around the whole display
    display.drawRect(0, 0, w, h, EL133UF1_BLACK);
    display.drawRect(1, 1, w - 2, h - 2, EL133UF1_BLACK);
    
    // Draw some rectangles in the middle section
    const uint16_t midY = barHeight + 50;
    const uint16_t rectSize = 150;
    const uint16_t spacing = 50;
    uint16_t xPos = (w - (6 * rectSize + 5 * spacing)) / 2;
    
    // Filled rectangles
    display.fillRect(xPos, midY, rectSize, rectSize, EL133UF1_BLACK);
    xPos += rectSize + spacing;
    display.fillRect(xPos, midY, rectSize, rectSize, EL133UF1_RED);
    xPos += rectSize + spacing;
    display.fillRect(xPos, midY, rectSize, rectSize, EL133UF1_YELLOW);
    xPos += rectSize + spacing;
    display.fillRect(xPos, midY, rectSize, rectSize, EL133UF1_GREEN);
    xPos += rectSize + spacing;
    display.fillRect(xPos, midY, rectSize, rectSize, EL133UF1_BLUE);
    xPos += rectSize + spacing;
    display.fillRect(xPos, midY, rectSize, rectSize, EL133UF1_WHITE);
    display.drawRect(xPos, midY, rectSize, rectSize, EL133UF1_BLACK);
    
    // Draw outlined rectangles below
    xPos = (w - (6 * rectSize + 5 * spacing)) / 2;
    const uint16_t outlineY = midY + rectSize + spacing;
    
    for (int i = 0; i < 6; i++) {
        uint8_t colors[] = {EL133UF1_BLACK, EL133UF1_RED, EL133UF1_YELLOW, 
                           EL133UF1_GREEN, EL133UF1_BLUE, EL133UF1_BLACK};
        for (int j = 0; j < 5; j++) {
            display.drawRect(xPos + j, outlineY + j, 
                           rectSize - 2*j, rectSize - 2*j, colors[i]);
        }
        xPos += rectSize + spacing;
    }
    
    // Draw diagonal lines pattern in bottom section
    const uint16_t patternY = outlineY + rectSize + spacing;
    const uint16_t patternHeight = h - patternY - 20;
    
    // Draw a checkerboard pattern
    const uint16_t checkSize = 40;
    uint8_t colors[] = {EL133UF1_BLACK, EL133UF1_RED, EL133UF1_YELLOW, 
                       EL133UF1_GREEN, EL133UF1_BLUE, EL133UF1_WHITE};
    
    for (uint16_t y = patternY; y < patternY + patternHeight; y += checkSize) {
        for (uint16_t x = 20; x < w - 20; x += checkSize) {
            int colorIdx = ((x / checkSize) + (y / checkSize)) % 6;
            display.fillRect(x, y, checkSize, checkSize, colors[colorIdx]);
        }
    }
    
    Serial.println("Demo pattern drawn to buffer");
}
