# TTF Fonts for EL133UF1

This directory is for TrueType font files embedded as C headers.

## How to Add a Font

### Step 1: Get a Font File

Use any TTF font file. Some recommended open-source options:
- **Liberation Mono** - Good for code/fixed width
- **Open Sans** - Clean sans-serif
- **Roboto** - Modern, readable
- **Source Sans Pro** - Adobe's open source font
- **Noto Sans** - Wide Unicode coverage

### Step 2: Convert to C Header

Use `xxd` to convert the TTF file to a C header:

```bash
xxd -i myfont.ttf > myfont.h
```

This creates a header like:

```c
unsigned char myfont_ttf[] = {
  0x00, 0x01, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x80, ...
};
unsigned int myfont_ttf_len = 123456;
```

### Step 3: Optimize for Flash Storage (Optional)

Add `PROGMEM` and `const` for better flash storage:

Edit the generated header to change:
```c
unsigned char myfont_ttf[] = {
```
to:
```c
const unsigned char myfont_ttf[] PROGMEM = {
```

### Step 4: Use in Your Code

```cpp
#include "EL133UF1.h"
#include "EL133UF1_TTF.h"
#include "fonts/myfont.h"

EL133UF1 display(&SPI1);
EL133UF1_TTF ttf;

void setup() {
    // ... display init ...
    
    ttf.begin(&display);
    if (!ttf.loadFont(myfont_ttf, myfont_ttf_len)) {
        Serial.println("Failed to load font!");
        return;
    }
    
    // Draw text at 48 pixel size
    ttf.drawText(100, 200, "Hello World!", 48.0, EL133UF1_BLACK);
    
    // Centered text
    ttf.drawTextCentered(0, 300, display.width(), "Centered!", 36.0, EL133UF1_RED);
    
    // Get metrics
    int16_t width = ttf.getTextWidth("Test", 24.0);
    int16_t ascent, descent, lineGap;
    ttf.getFontMetrics(24.0, &ascent, &descent, &lineGap);
}
```

## Font Size Recommendations

For the EL133UF1 (1600x1200 @ ~150 DPI):

| Use Case | Font Size (pixels) |
|----------|-------------------|
| Small caption | 18-24 |
| Body text | 28-36 |
| Subheading | 42-48 |
| Title | 64-96 |
| Large display | 120+ |

## Memory Considerations

- Font files typically range from 50KB to 500KB
- Fonts are stored in flash (PROGMEM)
- Rendering uses ~4KB temporary RAM per glyph
- Consider using a subset font for minimal size

## Creating Font Subsets

To reduce font size, you can create a subset containing only needed characters:

```bash
# Using fonttools (pip install fonttools)
pyftsubset myfont.ttf --unicodes="U+0020-007E" --output-file=myfont_subset.ttf
```

This keeps only ASCII printable characters (space through ~).
