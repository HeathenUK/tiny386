/**
 * @file EL133UF1_TTF.h
 * @brief TrueType font rendering for EL133UF1 display
 * 
 * Uses stb_truetype for TTF rendering. Fonts can be stored in flash
 * or loaded from filesystem.
 * 
 * Usage:
 *   #include "EL133UF1_TTF.h"
 *   
 *   // Include your font as a byte array (use xxd -i font.ttf > font.h)
 *   #include "my_font.h"
 *   
 *   EL133UF1_TTF ttf;
 *   ttf.begin(&display);
 *   ttf.loadFont(my_font_ttf, my_font_ttf_len);
 *   ttf.drawText(100, 100, "Hello World", 48, EL133UF1_BLACK);
 */

#ifndef EL133UF1_TTF_H
#define EL133UF1_TTF_H

#include <Arduino.h>
#include "EL133UF1.h"

// Forward declaration
class EL133UF1;

class EL133UF1_TTF {
public:
    EL133UF1_TTF();
    ~EL133UF1_TTF();
    
    /**
     * @brief Initialize the TTF renderer
     * @param display Pointer to EL133UF1 display instance
     * @return true if successful
     */
    bool begin(EL133UF1* display);
    
    /**
     * @brief Load a TTF font from memory
     * @param fontData Pointer to TTF font data (can be in flash)
     * @param fontDataSize Size of font data in bytes
     * @return true if font loaded successfully
     */
    bool loadFont(const uint8_t* fontData, size_t fontDataSize);
    
    /**
     * @brief Draw text at specified position
     * @param x X coordinate (top-left of text)
     * @param y Y coordinate (baseline of text)
     * @param text Text string to render
     * @param fontSize Font size in pixels
     * @param color Text color (EL133UF1_BLACK, etc.)
     * @param bgColor Background color (use same as text color for transparent)
     */
    void drawText(int16_t x, int16_t y, const char* text, float fontSize, 
                  uint8_t color, uint8_t bgColor = 0xFF);
    
    /**
     * @brief Get width of text string at given size
     * @param text Text string
     * @param fontSize Font size in pixels
     * @return Width in pixels
     */
    int16_t getTextWidth(const char* text, float fontSize);
    
    /**
     * @brief Get font metrics
     * @param fontSize Font size in pixels
     * @param ascent Output: pixels above baseline
     * @param descent Output: pixels below baseline (negative)
     * @param lineGap Output: recommended line gap
     */
    void getFontMetrics(float fontSize, int16_t* ascent, int16_t* descent, int16_t* lineGap);
    
    /**
     * @brief Check if a font is loaded
     */
    bool fontLoaded() { return _fontLoaded; }
    
    /**
     * @brief Draw text centered within a given width
     * @param x X coordinate of left edge
     * @param y Y coordinate (top of text)
     * @param width Width to center within
     * @param text Text string
     * @param fontSize Font size in pixels
     * @param color Text color
     */
    void drawTextCentered(int16_t x, int16_t y, int16_t width, 
                          const char* text, float fontSize, uint8_t color);
    
    /**
     * @brief Draw text right-aligned within a given width
     * @param x X coordinate of left edge
     * @param y Y coordinate (top of text)
     * @param width Width to align within
     * @param text Text string
     * @param fontSize Font size in pixels
     * @param color Text color
     */
    void drawTextRight(int16_t x, int16_t y, int16_t width,
                       const char* text, float fontSize, uint8_t color);

private:
    EL133UF1* _display;
    const uint8_t* _fontData;
    size_t _fontDataSize;
    bool _fontLoaded;
    void* _fontInfo;  // Opaque pointer to stbtt_fontinfo
    
    // Render a single glyph and draw to display
    void renderGlyph(int codepoint, int16_t x, int16_t y, float scale, 
                     uint8_t color, uint8_t bgColor);
};

#endif // EL133UF1_TTF_H
