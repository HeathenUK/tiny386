/**
 * @file EL133UF1_BMP.h
 * @brief BMP image loader for EL133UF1 display
 * 
 * Loads standard BMP files and renders to the display.
 * Supports 1/4/8/24-bit BMPs with automatic color mapping to Spectra 6 palette.
 * 
 * Usage:
 *   #include "EL133UF1_BMP.h"
 *   
 *   // Include your BMP as a byte array (use xxd -i image.bmp > image.h)
 *   #include "my_image.h"
 *   
 *   EL133UF1_BMP bmp;
 *   bmp.begin(&display);
 *   bmp.draw(0, 0, my_image_bmp, my_image_bmp_len);
 */

#ifndef EL133UF1_BMP_H
#define EL133UF1_BMP_H

#include <Arduino.h>
#include "EL133UF1.h"

// BMP file header structures (packed)
#pragma pack(push, 1)
struct BMPFileHeader {
    uint16_t signature;      // 'BM' = 0x4D42
    uint32_t fileSize;
    uint16_t reserved1;
    uint16_t reserved2;
    uint32_t dataOffset;     // Offset to pixel data
};

struct BMPInfoHeader {
    uint32_t headerSize;     // 40 for BITMAPINFOHEADER
    int32_t  width;
    int32_t  height;         // Negative = top-down
    uint16_t planes;         // Must be 1
    uint16_t bitsPerPixel;   // 1, 4, 8, 16, 24, or 32
    uint32_t compression;    // 0 = uncompressed
    uint32_t imageSize;
    int32_t  xPixelsPerMeter;
    int32_t  yPixelsPerMeter;
    uint32_t colorsUsed;
    uint32_t colorsImportant;
};
#pragma pack(pop)

// Result codes
enum BMPResult {
    BMP_OK = 0,
    BMP_ERR_NULL_DATA,
    BMP_ERR_INVALID_SIGNATURE,
    BMP_ERR_UNSUPPORTED_FORMAT,
    BMP_ERR_COMPRESSED,
    BMP_ERR_NO_DISPLAY
};

class EL133UF1_BMP {
public:
    EL133UF1_BMP();
    
    /**
     * @brief Initialize the BMP loader
     * @param display Pointer to EL133UF1 display instance
     * @return true if successful
     */
    bool begin(EL133UF1* display);
    
    /**
     * @brief Draw a BMP image at specified position
     * @param x X coordinate (top-left)
     * @param y Y coordinate (top-left)
     * @param data Pointer to BMP file data
     * @param len Length of BMP data
     * @return BMPResult code
     */
    BMPResult draw(int16_t x, int16_t y, const uint8_t* data, size_t len);
    
    /**
     * @brief Draw a BMP scaled to fit display (centered)
     * @param data Pointer to BMP file data
     * @param len Length of BMP data
     * @return BMPResult code
     */
    BMPResult drawFullscreen(const uint8_t* data, size_t len);
    
    /**
     * @brief Get info about a BMP without drawing
     * @param data Pointer to BMP file data
     * @param len Length of BMP data
     * @param width Output: image width
     * @param height Output: image height
     * @param bpp Output: bits per pixel
     * @return BMPResult code
     */
    BMPResult getInfo(const uint8_t* data, size_t len, 
                      int32_t* width, int32_t* height, uint16_t* bpp);
    
    /**
     * @brief Get last error message
     */
    const char* getErrorString(BMPResult result);

private:
    EL133UF1* _display;
    
    // Map RGB color to nearest Spectra 6 color
    uint8_t mapToSpectra6(uint8_t r, uint8_t g, uint8_t b);
    
    // Parse BMP headers
    BMPResult parseHeaders(const uint8_t* data, size_t len,
                           const BMPFileHeader** fileHeader,
                           const BMPInfoHeader** infoHeader);
    
    // Draw functions for different bit depths
    void draw24bit(int16_t x, int16_t y, const uint8_t* data,
                   const BMPInfoHeader* info, uint32_t dataOffset);
    void draw8bit(int16_t x, int16_t y, const uint8_t* data,
                  const BMPInfoHeader* info, uint32_t dataOffset,
                  const uint8_t* palette);
    void draw4bit(int16_t x, int16_t y, const uint8_t* data,
                  const BMPInfoHeader* info, uint32_t dataOffset,
                  const uint8_t* palette);
    void draw1bit(int16_t x, int16_t y, const uint8_t* data,
                  const BMPInfoHeader* info, uint32_t dataOffset,
                  const uint8_t* palette);
};

#endif // EL133UF1_BMP_H
