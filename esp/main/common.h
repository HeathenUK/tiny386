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
	uint8_t key_pressed[KEYCODE_MAX];  // Key state tracking
	int vga_width;     // VGA dimensions for OSD rendering
	int vga_height;
#endif
};

extern EventGroupHandle_t global_event_group;
extern struct Globals globals;

#ifdef USE_LCD_BSP
// Set VGA dimensions for PPA scaling calculation
void lcd_set_vga_dimensions(int width, int height);
#endif

#endif /* COMMON_H */
