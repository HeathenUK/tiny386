// requires esp-idf v5.5.x and badge-bsp
#define BUILD_ESP32
#define USE_BADGE_BSP

// PSRAM pool for emulator - leave room for BSP framebuffers (~1.5MB)
// User's SD card ini may specify up to 24MB mem_size
#define PSRAM_ALLOC_LEN (28 * 1024 * 1024)

#define IRAM_ATTR_CPU_EXEC1

#define BPP 16
#define FULL_UPDATE
#define USE_LCD_BSP
// Display logical dimensions (VGA emulator output size)
// lcd_bsp.c uses smaller internal dimensions (640x480) for PPA-friendly scaling
#define LCD_WIDTH 800
#define LCD_HEIGHT 480

// SD card pins (same as JC4880P433)
#define SD_CLK 43
#define SD_CMD 44
#define SD_D0 39
#define SD_D1 40
#define SD_D2 41
#define SD_D3 42
