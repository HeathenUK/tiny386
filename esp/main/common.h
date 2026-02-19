#ifndef COMMON_H
#define COMMON_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>

#define KEYCODE_MAX 0x80

#define BRIGHTNESS_MIN 0
#define BRIGHTNESS_MAX 110
#define BRIGHTNESS_BOOT_DEFAULT 50

static inline int clamp_brightness(int value)
{
	if (value < BRIGHTNESS_MIN) return BRIGHTNESS_MIN;
	if (value > BRIGHTNESS_MAX) return BRIGHTNESS_MAX;
	return value;
}

static inline uint8_t brightness_to_bsp_percent(int value)
{
	int clamped = clamp_brightness(value);
	return (uint8_t)((clamped * 100 + (BRIGHTNESS_MAX / 2)) / BRIGHTNESS_MAX);
}

struct Globals {
	void *pc;
	void *kbd;
	void *mouse;
	void *panel;
	void * _Atomic fb;  // Main framebuffer - atomic for cross-core visibility
	QueueHandle_t input_queue;
	void *fb_rotated;  // Rotated framebuffer (portrait 480x800)
	void *osd;         // OSD context
	bool osd_enabled;  // OSD visible
	bool reset_pending; // Soft reset requested from OSD
	bool emu_restart_pending; // Full emulator restart (reload config, reinit PC)
	char emu_new_hda_path[256]; // New HDA path for restart (set by OSD)
	bool vga_force_redraw; // Force full VGA redraw (e.g., after OSD closes)
	bool batch_reset_pending; // Reset dynamic batch size (on VGA mode change)
	uint8_t key_pressed[KEYCODE_MAX];  // Key state tracking
	int vga_width;     // VGA framebuffer dimensions for OSD rendering
	int vga_height;
	int vga_mode_width;   // Native VGA mode dimensions (before pixel doubling)
	int vga_mode_height;
	int vga_pixel_double; // Pixel doubling factor (1=none, 2=double width for mode 13h)
	uint32_t vga_frame_gen; // Frame generation counter - incremented on each VGA render
	int brightness;   // Current brightness (0-110 normalized)
	int volume;       // Current volume (0-100)
	// Overlay bar for META+arrow feedback
	int overlay_type;     // 0=none, 1=brightness, 2=volume
	int overlay_value;    // Current value (brightness:0-110, volume:0-100)
	uint32_t overlay_hide_time;  // Timestamp when overlay should hide
	// Toast notification system
	char toast_message[48];      // Current toast message
	uint32_t toast_hide_time;    // Timestamp when toast should hide (0 = not active)
	// Emulator stats for OSD display (lazy - only collected when viewing)
	bool stats_collecting;        // Set true when Status panel is open
	bool stats_bar_visible;       // Persistent stats bar overlay on VGA output
	uint8_t emu_cpu_percent;      // CPU time percentage (0-100)
	uint8_t emu_periph_percent;   // Peripheral time percentage (0-100)
	uint16_t emu_batch_size;      // Current instruction batch size
	uint32_t emu_calls_per_sec;   // pc_step calls per second
	uint32_t emu_cycles_per_sec;  // Emulated CPU cycles per second (IPS)
	uint8_t emu_seq_pct;          // Sequential fast path hit rate (0-100%)
	uint8_t emu_vga_fps;          // VGA frames per second
	uint8_t emu_hlt_pct;          // HLT idle percentage (0-100%)
	uint32_t emu_tlb_miss_per_sec;  // TLB misses per second
	uint32_t emu_irq_per_sec;      // Hardware IRQ deliveries per second
	uint32_t emu_disk_kb_per_sec;   // Disk I/O KB/s (read+write combined)
	uint8_t emu_hle_pct;           // HLE hit rate (0-100%)
	uint8_t emu_fusion_pct;        // CMP/TEST+Jcc fusion rate (0-100%)
	uint32_t emu_free_sram;        // Free internal SRAM bytes
	uint32_t emu_free_psram;       // Free PSRAM (SPI RAM) bytes
	// USB storage state
	bool usb_storage_connected;   // True when USB mass storage is attached
	bool usb_vfs_mounted;         // True when USB VFS is mounted at /usb
	// Mouse emulation settings
	int mouse_speed;              // Mouse speed setting (1-10, default 5)
	// USB passthrough setting
	int usb_passthru;             // USB passthrough to emulator (1=enabled, 0=disabled, default 1)
	// System config (set from INI, read by OSD)
	int cpu_gen;                  // CPU generation (3=386, 4=486, 5=586)
	int fpu;                      // FPU enabled (0/1)
	int mem_size_mb;              // Guest RAM in MB (actual, after PSRAM cap)
	bool cpu_debug_enabled;       // CPU debug instrumentation enabled (OSD toggle)

	// INI selector state
	bool ini_selector_active;     // INI selector screen is showing
	bool ini_selector_done;       // Selection complete, path is ready
	char ini_selected_path[256];  // Selected INI path (result from selector)

	// INI switch (runtime)
	bool screenshot_pending;      // Screenshot requested (Meta+S)

	bool ini_switch_pending;      // Switch to different INI requested
	char ini_switch_path[256];    // Path to new INI file
	char current_ini_path[256];   // Currently loaded INI path
};

extern EventGroupHandle_t global_event_group;
extern struct Globals globals;

// Set VGA dimensions for PPA scaling calculation
void lcd_set_vga_dimensions(int width, int height);

#endif /* COMMON_H */
