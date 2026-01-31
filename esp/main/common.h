#ifndef COMMON_H
#define COMMON_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#ifdef USE_BADGE_BSP
#include "freertos/queue.h"
#endif

struct Globals {
	void *pc;
	void *kbd;
	void *mouse;
	void *panel;
	void *fb;          // Main framebuffer (landscape 800x480)
#ifdef USE_BADGE_BSP
	QueueHandle_t input_queue;
	void *fb_rotated;  // Rotated framebuffer (portrait 480x800)
#endif
};

extern EventGroupHandle_t global_event_group;
extern struct Globals globals;

#ifdef USE_LCD_BSP
// Set VGA dimensions for PPA scaling calculation
void lcd_set_vga_dimensions(int width, int height);
#endif

#endif /* COMMON_H */
