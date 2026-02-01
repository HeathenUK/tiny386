// requires esp-idf v5.5.x and badge-bsp
#define BUILD_ESP32
#define USE_BADGE_BSP
#define TANMATSU_BUILD  // Guards for Tanmatsu-specific features in core code

// PSRAM pool for emulator - leave room for BSP framebuffers (~1.5MB)
// User's SD card ini may specify up to 24MB mem_size
#define PSRAM_ALLOC_LEN (28 * 1024 * 1024)

// Note: cpu_exec1() is too large (~1600 lines) for IRAM on ESP32-P4
// Keep empty to avoid IRAM overflow. Dispatch table in IRAM provides partial benefit.
#define IRAM_ATTR_CPU_EXEC1

#define BPP 16
#define FULL_UPDATE
#define USE_LCD_BSP
// Physical LCD dimensions (landscape, before 270-degree rotation to portrait)
// VGA framebuffer matches these dimensions; content is centered within
#define LCD_WIDTH 800
#define LCD_HEIGHT 480

// SD card pins (same as JC4880P433)
#define SD_CLK 43
#define SD_CMD 44
#define SD_D0 39
#define SD_D1 40
#define SD_D2 41
#define SD_D3 42
