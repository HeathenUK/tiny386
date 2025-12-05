/**
 * @file EL133UF1_TTF.cpp
 * @brief TrueType font rendering implementation for EL133UF1 display
 * 
 * Uses stb_truetype for TTF parsing and glyph rasterization.
 * Optimized for RP2350 with PSRAM support.
 */

#include "EL133UF1_TTF.h"

// Define implementation before including stb_truetype
#define STB_TRUETYPE_IMPLEMENTATION

// Use PSRAM-aware allocation on RP2350 (pmalloc is declared in Arduino.h)
#ifdef ARDUINO_ARCH_RP2040
#define STBTT_malloc(x,u)  ((void)(u), pmalloc(x))
#define STBTT_free(x,u)    ((void)(u), free(x))
#endif

#include "stb_truetype.h"

// ============================================================================
// Constructor / Destructor
// ============================================================================

EL133UF1_TTF::EL133UF1_TTF() : 
    _display(nullptr),
    _fontData(nullptr),
    _fontDataSize(0),
    _fontLoaded(false), 
    _fontInfo(nullptr),
    _cachedGlyphCount(0),
    _cacheScale(0),
    _cacheFontSize(0),
    _cacheEnabled(false)
{
    memset(_glyphCache, 0, sizeof(_glyphCache));
}

EL133UF1_TTF::~EL133UF1_TTF() {
    clearGlyphCache();
    if (_fontInfo) {
        free(_fontInfo);
        _fontInfo = nullptr;
    }
}

// ============================================================================
// Glyph Caching for Fast Rendering
// ============================================================================

void EL133UF1_TTF::clearGlyphCache() {
    for (int i = 0; i < _cachedGlyphCount; i++) {
        if (_glyphCache[i].bitmap) {
            free(_glyphCache[i].bitmap);
            _glyphCache[i].bitmap = nullptr;
        }
    }
    _cachedGlyphCount = 0;
    _cacheEnabled = false;
}

bool EL133UF1_TTF::enableGlyphCache(float fontSize, const char* characters) {
    if (!_fontLoaded || _fontInfo == nullptr) return false;
    
    // Clear any existing cache
    clearGlyphCache();
    
    stbtt_fontinfo* info = (stbtt_fontinfo*)_fontInfo;
    float scale = stbtt_ScaleForPixelHeight(info, fontSize);
    _cacheScale = scale;
    _cacheFontSize = fontSize;
    
    Serial.printf("TTF: Caching glyphs for size %.0f: ", fontSize);
    
    const char* p = characters;
    while (*p && _cachedGlyphCount < MAX_CACHED_GLYPHS) {
        // Decode UTF-8
        int codepoint = (uint8_t)*p++;
        if (codepoint >= 0xC0 && codepoint < 0xE0 && *p) {
            codepoint = ((codepoint & 0x1F) << 6) | (*p++ & 0x3F);
        }
        
        // Check if already cached
        bool found = false;
        for (int i = 0; i < _cachedGlyphCount; i++) {
            if (_glyphCache[i].codepoint == codepoint) {
                found = true;
                break;
            }
        }
        if (found) continue;
        
        // Get glyph metrics
        int x0, y0, x1, y1;
        stbtt_GetCodepointBitmapBox(info, codepoint, scale, scale, &x0, &y0, &x1, &y1);
        
        int width = x1 - x0;
        int height = y1 - y0;
        
        if (width <= 0 || height <= 0) {
            // Space or non-printable - still cache metrics
            int advance, lsb;
            stbtt_GetCodepointHMetrics(info, codepoint, &advance, &lsb);
            
            CachedGlyph* g = &_glyphCache[_cachedGlyphCount++];
            g->codepoint = codepoint;
            g->width = 0;
            g->height = 0;
            g->xOffset = 0;
            g->yOffset = 0;
            g->advance = (int16_t)(advance * scale);
            g->bitmap = nullptr;
            continue;
        }
        
        // Allocate bitmap
        size_t bitmapSize = width * height;
        uint8_t* bitmap = (uint8_t*)malloc(bitmapSize);
        if (!bitmap) {
            Serial.println("\nTTF: Cache allocation failed");
            break;
        }
        
        // Render glyph
        stbtt_MakeCodepointBitmap(info, bitmap, width, height, width, scale, scale, codepoint);
        
        // Get advance
        int advance, lsb;
        stbtt_GetCodepointHMetrics(info, codepoint, &advance, &lsb);
        
        // Store in cache
        CachedGlyph* g = &_glyphCache[_cachedGlyphCount++];
        g->codepoint = codepoint;
        g->width = width;
        g->height = height;
        g->xOffset = x0;
        g->yOffset = y0;
        g->advance = (int16_t)(advance * scale);
        g->bitmap = bitmap;
        
        Serial.printf("%c", codepoint < 128 ? (char)codepoint : '?');
    }
    
    _cacheEnabled = true;
    Serial.printf(" (%d glyphs cached)\n", _cachedGlyphCount);
    return true;
}

EL133UF1_TTF::CachedGlyph* EL133UF1_TTF::findCachedGlyph(int codepoint) {
    if (!_cacheEnabled) return nullptr;
    for (int i = 0; i < _cachedGlyphCount; i++) {
        if (_glyphCache[i].codepoint == codepoint) {
            return &_glyphCache[i];
        }
    }
    return nullptr;
}

void EL133UF1_TTF::renderCachedGlyph(CachedGlyph* glyph, int16_t x, int16_t baseline, uint8_t color) {
    if (!glyph || !glyph->bitmap) return;
    
    int16_t screenX = x + glyph->xOffset;
    int16_t screenY = baseline + glyph->yOffset;
    
    // Fast path: direct pixel writes with threshold
    for (int py = 0; py < glyph->height; py++) {
        int16_t drawY = screenY + py;
        if (drawY < 0 || drawY >= _display->height()) continue;
        
        const uint8_t* row = glyph->bitmap + py * glyph->width;
        for (int px = 0; px < glyph->width; px++) {
            int16_t drawX = screenX + px;
            if (drawX < 0 || drawX >= _display->width()) continue;
            
            if (row[px] > 127) {
                _display->setPixel(drawX, drawY, color);
            }
        }
    }
}

// ============================================================================
// Initialization
// ============================================================================

bool EL133UF1_TTF::begin(EL133UF1* display) {
    if (display == nullptr) {
        return false;
    }
    _display = display;
    return true;
}

// ============================================================================
// Font Loading
// ============================================================================

bool EL133UF1_TTF::loadFont(const uint8_t* fontData, size_t fontDataSize) {
    if (fontData == nullptr || fontDataSize == 0) {
        Serial.println("TTF: Invalid font data");
        return false;
    }
    
    _fontData = fontData;
    _fontDataSize = fontDataSize;
    
    // Free existing font info if any
    if (_fontInfo) {
        free(_fontInfo);
        _fontInfo = nullptr;
        _fontLoaded = false;
    }
    
    // Allocate font info structure
    _fontInfo = malloc(sizeof(stbtt_fontinfo));
    if (_fontInfo == nullptr) {
        Serial.println("TTF: Failed to allocate font info");
        return false;
    }
    
    // Initialize the font
    stbtt_fontinfo* info = (stbtt_fontinfo*)_fontInfo;
    int offset = stbtt_GetFontOffsetForIndex(fontData, 0);
    if (offset < 0) {
        Serial.println("TTF: Invalid font offset");
        free(_fontInfo);
        _fontInfo = nullptr;
        return false;
    }
    
    if (!stbtt_InitFont(info, fontData, offset)) {
        Serial.println("TTF: Failed to initialize font");
        free(_fontInfo);
        _fontInfo = nullptr;
        return false;
    }
    
    _fontLoaded = true;
    Serial.println("TTF: Font loaded successfully");
    return true;
}

// ============================================================================
// Font Metrics
// ============================================================================

void EL133UF1_TTF::getFontMetrics(float fontSize, int16_t* ascent, int16_t* descent, int16_t* lineGap) {
    if (!_fontLoaded || _fontInfo == nullptr) {
        if (ascent) *ascent = 0;
        if (descent) *descent = 0;
        if (lineGap) *lineGap = 0;
        return;
    }
    
    stbtt_fontinfo* info = (stbtt_fontinfo*)_fontInfo;
    float scale = stbtt_ScaleForPixelHeight(info, fontSize);
    
    int asc, desc, gap;
    stbtt_GetFontVMetrics(info, &asc, &desc, &gap);
    
    if (ascent) *ascent = (int16_t)(asc * scale);
    if (descent) *descent = (int16_t)(desc * scale);
    if (lineGap) *lineGap = (int16_t)(gap * scale);
}

int16_t EL133UF1_TTF::getTextWidth(const char* text, float fontSize) {
    if (!_fontLoaded || _fontInfo == nullptr || text == nullptr) {
        return 0;
    }
    
    stbtt_fontinfo* info = (stbtt_fontinfo*)_fontInfo;
    float scale = stbtt_ScaleForPixelHeight(info, fontSize);
    
    int width = 0;
    const char* p = text;
    
    while (*p) {
        int codepoint = (uint8_t)*p++;
        
        // Handle UTF-8 (basic 2-byte sequences)
        if (codepoint >= 0xC0 && codepoint < 0xE0 && *p) {
            codepoint = ((codepoint & 0x1F) << 6) | (*p++ & 0x3F);
        }
        
        int advance, lsb;
        stbtt_GetCodepointHMetrics(info, codepoint, &advance, &lsb);
        width += (int)(advance * scale);
        
        // Add kerning if there's a next character
        if (*p) {
            int nextCodepoint = (uint8_t)*p;
            if (nextCodepoint >= 0xC0 && nextCodepoint < 0xE0 && *(p+1)) {
                nextCodepoint = ((nextCodepoint & 0x1F) << 6) | (*(p+1) & 0x3F);
            }
            int kern = stbtt_GetCodepointKernAdvance(info, codepoint, nextCodepoint);
            width += (int)(kern * scale);
        }
    }
    
    return (int16_t)width;
}

// ============================================================================
// Glyph Rendering
// ============================================================================

void EL133UF1_TTF::renderGlyph(int codepoint, int16_t x, int16_t y, float scale, 
                                uint8_t color, uint8_t bgColor) {
    if (!_fontLoaded || _fontInfo == nullptr || _display == nullptr) {
        return;
    }
    
    stbtt_fontinfo* info = (stbtt_fontinfo*)_fontInfo;
    
    // Get glyph bounding box
    int x0, y0, x1, y1;
    stbtt_GetCodepointBitmapBox(info, codepoint, scale, scale, &x0, &y0, &x1, &y1);
    
    int glyphWidth = x1 - x0;
    int glyphHeight = y1 - y0;
    
    if (glyphWidth <= 0 || glyphHeight <= 0) {
        return;  // No visible glyph (e.g., space)
    }
    
    // Allocate temporary bitmap for glyph
    // Use regular malloc - this is a small temporary buffer
    size_t bitmapSize = glyphWidth * glyphHeight;
    uint8_t* bitmap = (uint8_t*)malloc(bitmapSize);
    if (bitmap == nullptr) {
        Serial.printf("TTF: Failed to allocate glyph bitmap (%d bytes)\n", bitmapSize);
        return;
    }
    
    // Render glyph to bitmap (8-bit grayscale)
    stbtt_MakeCodepointBitmap(info, bitmap, glyphWidth, glyphHeight, 
                               glyphWidth, scale, scale, codepoint);
    
    // Calculate actual screen position
    int16_t screenX = x + x0;
    int16_t screenY = y + y0;
    
    // Draw glyph pixels to display
    // For e-ink with limited colors, we use threshold-based rendering
    // Alpha > 127 = foreground color, else background (or skip if transparent)
    bool transparentBg = (bgColor == 0xFF);
    
    for (int py = 0; py < glyphHeight; py++) {
        int16_t drawY = screenY + py;
        if (drawY < 0 || drawY >= _display->height()) continue;
        
        for (int px = 0; px < glyphWidth; px++) {
            int16_t drawX = screenX + px;
            if (drawX < 0 || drawX >= _display->width()) continue;
            
            uint8_t alpha = bitmap[py * glyphWidth + px];
            
            // Threshold-based rendering for e-ink
            // Could be enhanced with dithering for smoother edges
            if (alpha > 127) {
                _display->setPixel(drawX, drawY, color);
            } else if (!transparentBg && alpha > 32) {
                // Optional: intermediate threshold for anti-aliasing effect
                // with limited e-ink colors, might want to skip this
                _display->setPixel(drawX, drawY, bgColor);
            } else if (!transparentBg) {
                _display->setPixel(drawX, drawY, bgColor);
            }
            // If transparent background, just skip pixels with low alpha
        }
    }
    
    free(bitmap);
}

// ============================================================================
// Text Drawing
// ============================================================================

void EL133UF1_TTF::drawText(int16_t x, int16_t y, const char* text, float fontSize, 
                            uint8_t color, uint8_t bgColor) {
    if (!_fontLoaded || _fontInfo == nullptr || _display == nullptr || text == nullptr) {
        return;
    }
    
    stbtt_fontinfo* info = (stbtt_fontinfo*)_fontInfo;
    float scale = stbtt_ScaleForPixelHeight(info, fontSize);
    
    // Check if we can use the cache (size must match)
    bool useCache = _cacheEnabled && (fontSize == _cacheFontSize);
    
    // Get font metrics for baseline positioning
    int ascent, descent, lineGap;
    stbtt_GetFontVMetrics(info, &ascent, &descent, &lineGap);
    int baselineOffset = (int)(ascent * scale);
    
    // Adjust y to be top of text (caller provides top-left, we need baseline)
    int16_t baseline = y + baselineOffset;
    
    int xPos = x;
    const char* p = text;
    int prevCodepoint = 0;
    
    while (*p) {
        // Decode character (handle basic UTF-8)
        int codepoint = (uint8_t)*p++;
        
        // Handle UTF-8 multi-byte sequences (2-byte only for now)
        if (codepoint >= 0xC0 && codepoint < 0xE0 && *p) {
            codepoint = ((codepoint & 0x1F) << 6) | (*p++ & 0x3F);
        } else if (codepoint >= 0xE0 && codepoint < 0xF0 && *p && *(p+1)) {
            // 3-byte UTF-8
            codepoint = ((codepoint & 0x0F) << 12) | 
                       ((*p++ & 0x3F) << 6) | 
                       (*p++ & 0x3F);
        }
        
        // Handle newline
        if (codepoint == '\n') {
            xPos = x;
            baseline += (int)((ascent - descent + lineGap) * scale);
            prevCodepoint = 0;
            continue;
        }
        
        // Apply kerning from previous character
        if (prevCodepoint) {
            int kern = stbtt_GetCodepointKernAdvance(info, prevCodepoint, codepoint);
            xPos += (int)(kern * scale);
        }
        
        // Try to use cached glyph first
        CachedGlyph* cached = useCache ? findCachedGlyph(codepoint) : nullptr;
        if (cached) {
            // Fast path: use pre-rendered glyph
            renderCachedGlyph(cached, xPos, baseline, color);
            xPos += cached->advance;
        } else {
            // Slow path: render glyph on-the-fly
            renderGlyph(codepoint, xPos, baseline, scale, color, bgColor);
            
            // Advance cursor
            int advance, lsb;
            stbtt_GetCodepointHMetrics(info, codepoint, &advance, &lsb);
            xPos += (int)(advance * scale);
        }
        
        prevCodepoint = codepoint;
    }
}

// ============================================================================
// Advanced Text Drawing with Alignment
// ============================================================================

void EL133UF1_TTF::drawTextCentered(int16_t x, int16_t y, int16_t width, 
                                     const char* text, float fontSize, uint8_t color) {
    int16_t textWidth = getTextWidth(text, fontSize);
    int16_t offsetX = (width - textWidth) / 2;
    drawText(x + offsetX, y, text, fontSize, color);
}

void EL133UF1_TTF::drawTextRight(int16_t x, int16_t y, int16_t width,
                                  const char* text, float fontSize, uint8_t color) {
    int16_t textWidth = getTextWidth(text, fontSize);
    int16_t offsetX = width - textWidth;
    drawText(x + offsetX, y, text, fontSize, color);
}

// ============================================================================
// Anchor-Based Alignment
// ============================================================================

int16_t EL133UF1_TTF::getTextHeight(float fontSize) {
    if (!_fontLoaded || _fontInfo == nullptr) return 0;
    
    stbtt_fontinfo* info = (stbtt_fontinfo*)_fontInfo;
    float scale = stbtt_ScaleForPixelHeight(info, fontSize);
    
    int ascent, descent, lineGap;
    stbtt_GetFontVMetrics(info, &ascent, &descent, &lineGap);
    
    // descent is negative, so we subtract it (adding its absolute value)
    return (int16_t)((ascent - descent) * scale);
}

void EL133UF1_TTF::drawTextAligned(int16_t x, int16_t y, const char* text, float fontSize,
                                    uint8_t color, TextAlignH alignH, TextAlignV alignV,
                                    uint8_t bgColor) {
    if (!_fontLoaded || _fontInfo == nullptr || _display == nullptr || text == nullptr) return;
    
    stbtt_fontinfo* info = (stbtt_fontinfo*)_fontInfo;
    float scale = stbtt_ScaleForPixelHeight(info, fontSize);
    
    // Get font metrics
    int ascent, descent, lineGap;
    stbtt_GetFontVMetrics(info, &ascent, &descent, &lineGap);
    int16_t ascentPx = (int16_t)(ascent * scale);
    int16_t descentPx = (int16_t)(descent * scale);  // negative value
    int16_t totalHeight = ascentPx - descentPx;
    
    // Calculate horizontal offset based on alignment
    int16_t drawX = x;
    if (alignH != ALIGN_LEFT) {
        int16_t textWidth = getTextWidth(text, fontSize);
        if (alignH == ALIGN_CENTER) {
            drawX = x - textWidth / 2;
        } else if (alignH == ALIGN_RIGHT) {
            drawX = x - textWidth;
        }
    }
    
    // Calculate vertical offset based on alignment
    // Note: drawText expects y to be TOP of text, so we adjust accordingly
    int16_t drawY = y;
    switch (alignV) {
        case ALIGN_TOP:
            // y is already top, no adjustment needed
            break;
        case ALIGN_BASELINE:
            // y is baseline, top is baseline minus ascent
            drawY = y - ascentPx;
            break;
        case ALIGN_BOTTOM:
            // y is bottom (descender line), top is y minus total height
            drawY = y - totalHeight;
            break;
        case ALIGN_MIDDLE:
            // y is vertical center, top is y minus half the height
            drawY = y - totalHeight / 2;
            break;
    }
    
    drawText(drawX, drawY, text, fontSize, color, bgColor);
}

void EL133UF1_TTF::drawTextAlignedOutlined(int16_t x, int16_t y, const char* text, float fontSize,
                                            uint8_t color, uint8_t outlineColor,
                                            TextAlignH alignH, TextAlignV alignV,
                                            int outlineWidth, bool exactOutline) {
    if (!_fontLoaded || _fontInfo == nullptr || _display == nullptr || text == nullptr) return;
    
    stbtt_fontinfo* info = (stbtt_fontinfo*)_fontInfo;
    float scale = stbtt_ScaleForPixelHeight(info, fontSize);
    
    // Get font metrics
    int ascent, descent, lineGap;
    stbtt_GetFontVMetrics(info, &ascent, &descent, &lineGap);
    int16_t ascentPx = (int16_t)(ascent * scale);
    int16_t descentPx = (int16_t)(descent * scale);
    int16_t totalHeight = ascentPx - descentPx;
    
    // Calculate horizontal offset
    int16_t drawX = x;
    if (alignH != ALIGN_LEFT) {
        int16_t textWidth = getTextWidth(text, fontSize);
        if (alignH == ALIGN_CENTER) {
            drawX = x - textWidth / 2;
        } else if (alignH == ALIGN_RIGHT) {
            drawX = x - textWidth;
        }
    }
    
    // Calculate vertical offset
    int16_t drawY = y;
    switch (alignV) {
        case ALIGN_TOP:
            break;
        case ALIGN_BASELINE:
            drawY = y - ascentPx;
            break;
        case ALIGN_BOTTOM:
            drawY = y - totalHeight;
            break;
        case ALIGN_MIDDLE:
            drawY = y - totalHeight / 2;
            break;
    }
    
    drawTextOutlined(drawX, drawY, text, fontSize, color, outlineColor, outlineWidth, exactOutline);
}

// ============================================================================
// Outlined Text
// ============================================================================

// Render a single glyph with outline (optimized single-pass)
void EL133UF1_TTF::renderGlyphOutlined(int codepoint, int16_t x, int16_t baseline,
                                        float scale, uint8_t color, uint8_t outlineColor,
                                        int outlineWidth) {
    stbtt_fontinfo* info = (stbtt_fontinfo*)_fontInfo;
    
    // Get glyph bounding box
    int x0, y0, x1, y1;
    stbtt_GetCodepointBitmapBox(info, codepoint, scale, scale, &x0, &y0, &x1, &y1);
    
    int glyphWidth = x1 - x0;
    int glyphHeight = y1 - y0;
    
    if (glyphWidth <= 0 || glyphHeight <= 0) return;
    
    // Expanded buffer size to accommodate outline
    int pad = outlineWidth;
    int bufWidth = glyphWidth + pad * 2;
    int bufHeight = glyphHeight + pad * 2;
    size_t bufSize = bufWidth * bufHeight;
    
    uint8_t* glyphBuf = (uint8_t*)malloc(bufSize);
    if (!glyphBuf) return;
    
    memset(glyphBuf, 0, bufSize);
    
    // Render glyph centered in padded buffer
    stbtt_MakeCodepointBitmap(info, glyphBuf + pad * bufWidth + pad,
                               glyphWidth, glyphHeight, bufWidth, scale, scale, codepoint);
    
    // Screen position (adjusted for padding)
    int16_t screenX = x + x0 - pad;
    int16_t screenY = baseline + y0 - pad;
    
    // Single pass: for each pixel, check if it's glyph, outline, or skip
    for (int py = 0; py < bufHeight; py++) {
        int16_t drawY = screenY + py;
        if (drawY < 0 || drawY >= _display->height()) continue;
        
        for (int px = 0; px < bufWidth; px++) {
            int16_t drawX = screenX + px;
            if (drawX < 0 || drawX >= _display->width()) continue;
            
            uint8_t alpha = glyphBuf[py * bufWidth + px];
            
            // Is this a glyph pixel?
            if (alpha > 127) {
                _display->setPixel(drawX, drawY, color);
                continue;
            }
            
            // Check if any neighbor within outlineWidth is a glyph pixel (dilation)
            bool isOutline = false;
            for (int oy = -outlineWidth; oy <= outlineWidth && !isOutline; oy++) {
                for (int ox = -outlineWidth; ox <= outlineWidth && !isOutline; ox++) {
                    if (ox == 0 && oy == 0) continue;
                    int nx = px + ox;
                    int ny = py + oy;
                    if (nx >= 0 && nx < bufWidth && ny >= 0 && ny < bufHeight) {
                        if (glyphBuf[ny * bufWidth + nx] > 127) {
                            isOutline = true;
                        }
                    }
                }
            }
            
            if (isOutline) {
                _display->setPixel(drawX, drawY, outlineColor);
            }
        }
    }
    
    free(glyphBuf);
}

void EL133UF1_TTF::drawTextOutlined(int16_t x, int16_t y, const char* text, 
                                     float fontSize, uint8_t color, 
                                     uint8_t outlineColor, int outlineWidth,
                                     bool exactOutline) {
    if (!_fontLoaded || _fontInfo == nullptr || _display == nullptr || text == nullptr) return;
    
    // Exact mode: render text multiple times at offsets (slower but pixel-perfect)
    if (exactOutline) {
        for (int w = outlineWidth; w >= 1; w--) {
            for (int dy = -w; dy <= w; dy++) {
                for (int dx = -w; dx <= w; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    if (abs(dx) < w && abs(dy) < w) continue;
                    drawText(x + dx, y + dy, text, fontSize, outlineColor);
                }
            }
        }
        drawText(x, y, text, fontSize, color);
        return;
    }
    
    // Fast mode: single render with dilation
    stbtt_fontinfo* info = (stbtt_fontinfo*)_fontInfo;
    float scale = stbtt_ScaleForPixelHeight(info, fontSize);
    
    // Get font metrics for baseline
    int ascent, descent, lineGap;
    stbtt_GetFontVMetrics(info, &ascent, &descent, &lineGap);
    int baseline = y + (int)(ascent * scale);
    
    int xPos = x;
    int prevCodepoint = 0;
    const char* p = text;
    
    while (*p) {
        // Decode UTF-8 (same as drawText)
        int codepoint = (uint8_t)*p++;
        if (codepoint >= 0xC0 && codepoint < 0xE0 && *p) {
            codepoint = ((codepoint & 0x1F) << 6) | (*p++ & 0x3F);
        } else if (codepoint >= 0xE0 && codepoint < 0xF0 && *p && *(p+1)) {
            codepoint = ((codepoint & 0x0F) << 12) | 
                       ((*p++ & 0x3F) << 6) | 
                       (*p++ & 0x3F);
        }
        
        // Handle newlines
        if (codepoint == '\n') {
            xPos = x;
            baseline += (int)((ascent - descent + lineGap) * scale);
            prevCodepoint = 0;
            continue;
        }
        
        // Apply kerning
        if (prevCodepoint) {
            int kern = stbtt_GetCodepointKernAdvance(info, prevCodepoint, codepoint);
            xPos += (int)(kern * scale);
        }
        
        // Render glyph with outline
        renderGlyphOutlined(codepoint, xPos, baseline, scale, color, outlineColor, outlineWidth);
        
        // Advance cursor
        int advance, lsb;
        stbtt_GetCodepointHMetrics(info, codepoint, &advance, &lsb);
        xPos += (int)(advance * scale);
        
        prevCodepoint = codepoint;
    }
}

void EL133UF1_TTF::drawTextOutlinedCentered(int16_t x, int16_t y, int16_t width,
                                             const char* text, float fontSize,
                                             uint8_t color, uint8_t outlineColor, 
                                             int outlineWidth, bool exactOutline) {
    int16_t textWidth = getTextWidth(text, fontSize);
    int16_t offsetX = (width - textWidth) / 2;
    drawTextOutlined(x + offsetX, y, text, fontSize, color, outlineColor, outlineWidth, exactOutline);
}
