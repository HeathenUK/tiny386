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

/**
 * @brief Horizontal text alignment (anchor point)
 */
enum TextAlignH : uint8_t {
    ALIGN_LEFT   = 0,  // x is left edge of text
    ALIGN_CENTER = 1,  // x is horizontal center of text
    ALIGN_RIGHT  = 2   // x is right edge of text
};

/**
 * @brief Vertical text alignment (anchor point)
 */
enum TextAlignV : uint8_t {
    ALIGN_TOP      = 0,  // y is top of text (ascender line)
    ALIGN_BASELINE = 1,  // y is baseline (where most letters sit)
    ALIGN_BOTTOM   = 2,  // y is bottom of text (descender line)
    ALIGN_MIDDLE   = 3   // y is vertical center of text
};

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

    /**
     * @brief Draw text with precise anchor-based alignment
     * 
     * The anchor point (x, y) can be positioned at any combination of
     * horizontal (left/center/right) and vertical (top/baseline/bottom/middle)
     * alignment points on the text.
     * 
     * Example: To center text at screen position (400, 300):
     *   drawTextAligned(400, 300, "Hello", 48, color, ALIGN_CENTER, ALIGN_MIDDLE);
     * 
     * Example: To place text with baseline at y=500, right-aligned to x=800:
     *   drawTextAligned(800, 500, "Hello", 48, color, ALIGN_RIGHT, ALIGN_BASELINE);
     * 
     * @param x X coordinate of anchor point
     * @param y Y coordinate of anchor point
     * @param text Text string
     * @param fontSize Font size in pixels
     * @param color Text color
     * @param alignH Horizontal alignment (ALIGN_LEFT, ALIGN_CENTER, ALIGN_RIGHT)
     * @param alignV Vertical alignment (ALIGN_TOP, ALIGN_BASELINE, ALIGN_BOTTOM, ALIGN_MIDDLE)
     * @param bgColor Background color (0xFF for transparent, default)
     */
    void drawTextAligned(int16_t x, int16_t y, const char* text, float fontSize,
                         uint8_t color, TextAlignH alignH, TextAlignV alignV,
                         uint8_t bgColor = 0xFF);

    /**
     * @brief Draw outlined text with precise anchor-based alignment
     * 
     * Same as drawTextAligned but with outline.
     */
    void drawTextAlignedOutlined(int16_t x, int16_t y, const char* text, float fontSize,
                                  uint8_t color, uint8_t outlineColor,
                                  TextAlignH alignH, TextAlignV alignV,
                                  int outlineWidth = 1, bool exactOutline = false);

    /**
     * @brief Get text height including ascenders and descenders
     * @param fontSize Font size in pixels
     * @return Total height in pixels (ascent + |descent|)
     */
    int16_t getTextHeight(float fontSize);

    /**
     * @brief Draw text with an outline (stroke) around it
     * 
     * Useful for making text readable on any background.
     * 
     * @param x X coordinate
     * @param y Y coordinate (top of text)
     * @param text Text string
     * @param fontSize Font size in pixels
     * @param color Text fill color
     * @param outlineColor Outline/stroke color
     * @param outlineWidth Outline thickness (1-3 recommended)
     * @param exactOutline If true, use slower but pixel-perfect outline (9× render)
     */
    void drawTextOutlined(int16_t x, int16_t y, const char* text, float fontSize,
                          uint8_t color, uint8_t outlineColor, int outlineWidth = 1,
                          bool exactOutline = false);

    /**
     * @brief Draw centered text with outline
     */
    void drawTextOutlinedCentered(int16_t x, int16_t y, int16_t width,
                                   const char* text, float fontSize,
                                   uint8_t color, uint8_t outlineColor, 
                                   int outlineWidth = 1, bool exactOutline = false);

    /**
     * @brief Enable glyph caching for faster repeated rendering
     * 
     * Caches rendered glyphs for characters that are used frequently
     * (digits 0-9, punctuation, etc). Significantly speeds up clock displays.
     * 
     * @param fontSize Font size to cache (only one size cached at a time)
     * @param characters String of characters to cache (e.g., "0123456789:")
     * @return true if cache was created successfully
     */
    bool enableGlyphCache(float fontSize, const char* characters = "0123456789:.-/ ");

    /**
     * @brief Clear the glyph cache
     */
    void clearGlyphCache();

    /**
     * @brief Check if glyph cache is active
     */
    bool isCacheEnabled() { return _cacheEnabled; }

private:
    EL133UF1* _display;
    const uint8_t* _fontData;
    size_t _fontDataSize;
    bool _fontLoaded;
    void* _fontInfo;  // Opaque pointer to stbtt_fontinfo
    
    // Glyph cache for fast rendering
    struct CachedGlyph {
        int codepoint;
        int16_t width, height;
        int16_t xOffset, yOffset;  // Offset from baseline
        int16_t advance;
        uint8_t* bitmap;  // Pre-rendered bitmap (1-bit or 8-bit alpha)
    };
    static const int MAX_CACHED_GLYPHS = 32;
    CachedGlyph _glyphCache[MAX_CACHED_GLYPHS];
    int _cachedGlyphCount;
    float _cacheScale;
    float _cacheFontSize;
    bool _cacheEnabled;
    
    // Find cached glyph or return nullptr
    CachedGlyph* findCachedGlyph(int codepoint);
    
    // Render glyph from cache
    void renderCachedGlyph(CachedGlyph* glyph, int16_t x, int16_t baseline, uint8_t color);
    
    // Render a single glyph with outline (optimized)
    void renderGlyphOutlined(int codepoint, int16_t x, int16_t baseline,
                             float scale, uint8_t color, uint8_t outlineColor,
                             int outlineWidth);
    
    // Render a single glyph and draw to display
    void renderGlyph(int codepoint, int16_t x, int16_t y, float scale, 
                     uint8_t color, uint8_t bgColor);
};

#endif // EL133UF1_TTF_H
