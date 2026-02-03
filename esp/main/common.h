#ifndef COMMON_H
#define COMMON_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#ifdef USE_BADGE_BSP
#include "freertos/queue.h"
#include <stdbool.h>
#endif
#include <stdatomic.h>

#define KEYCODE_MAX 0x80

struct Globals {
	void *pc;
	void *kbd;
	void *mouse;
	void *panel;
	void * _Atomic fb;  // Main framebuffer - atomic for cross-core visibility
#ifdef USE_BADGE_BSP
	QueueHandle_t input_queue;
	void *fb_rotated;  // Rotated framebuffer (portrait 480x800)
	void *osd;         // OSD context
	bool osd_enabled;  // OSD visible
	bool reset_pending; // Soft reset requested from OSD
	bool vga_force_redraw; // Force full VGA redraw (e.g., after OSD closes)
	bool batch_reset_pending; // Reset dynamic batch size (on VGA mode change)
	uint8_t key_pressed[KEYCODE_MAX];  // Key state tracking
	int vga_width;     // VGA framebuffer dimensions for OSD rendering
	int vga_height;
	int vga_mode_width;   // Native VGA mode dimensions (before pixel doubling)
	int vga_mode_height;
	int vga_pixel_double; // Pixel doubling factor (1=none, 2=double width for mode 13h)
	uint32_t vga_frame_gen; // Frame generation counter - incremented on each VGA render
	int brightness;   // Current brightness (0-100)
	int volume;       // Current volume (0-100)
	// Overlay bar for META+arrow feedback
	int overlay_type;     // 0=none, 1=brightness, 2=volume
	int overlay_value;    // Current value (0-100)
	uint32_t overlay_hide_time;  // Timestamp when overlay should hide
	// Toast notification system
	char toast_message[48];      // Current toast message
	uint32_t toast_hide_time;    // Timestamp when toast should hide (0 = not active)
	// Emulator stats for OSD display
	uint8_t emu_cpu_percent;      // CPU time percentage (0-100)
	uint8_t emu_periph_percent;   // Peripheral time percentage (0-100)
	uint16_t emu_batch_size;      // Current instruction batch size
	uint32_t emu_calls_per_sec;   // pc_step calls per second
#endif
};

extern EventGroupHandle_t global_event_group;
extern struct Globals globals;

#ifdef USE_LCD_BSP
// Set VGA dimensions for PPA scaling calculation
void lcd_set_vga_dimensions(int width, int height);
#endif

#endif /* COMMON_H */
