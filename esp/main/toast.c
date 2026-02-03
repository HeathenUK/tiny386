#ifdef USE_BADGE_BSP
/*
 * Toast notification system for Tanmatsu
 * Displays brief text messages at the bottom of the screen
 */

#include "toast.h"
#include "common.h"
#include "vga_font_8x16.h"
#include "esp_log.h"
#include <string.h>

// Display dimensions (portrait)
#define DISPLAY_WIDTH  480
#define DISPLAY_HEIGHT 800

// Font dimensions
#define FONT_WIDTH 8
#define FONT_HEIGHT 16

// Toast styling
#define TOAST_BG_COLOR    0x18E3  // Dark blue-gray (same as OSD panel)
#define TOAST_BORDER_COLOR 0x4A69  // Medium gray border
#define TOAST_TEXT_COLOR  0xFFFF  // White text
#define TOAST_PADDING     8
#define TOAST_MARGIN_BOTTOM 60

// Get font bitmap for a character
static const uint8_t *toast_get_font_char(int c)
{
	if (c >= 32 && c <= 126) {
		return vga_font_8x16_data[c - 32];
	}
	return vga_font_8x16_data['?' - 32];
}

// Draw filled rectangle to portrait buffer
static void toast_fill_rect(uint16_t *fb, int x, int y, int w, int h, uint16_t color)
{
	for (int dy = 0; dy < h; dy++) {
		int py = y + dy;
		if (py < 0 || py >= DISPLAY_HEIGHT) continue;
		uint16_t *row = fb + py * DISPLAY_WIDTH;
		for (int dx = 0; dx < w; dx++) {
			int px = x + dx;
			if (px < 0 || px >= DISPLAY_WIDTH) continue;
			row[px] = color;
		}
	}
}

// Draw text to portrait buffer
static void toast_draw_text(uint16_t *fb, int x, int y, const char *text, uint16_t color)
{
	while (*text) {
		unsigned char c = *text++;
		if (c < 32 || c > 126) c = '?';

		const uint8_t *glyph = toast_get_font_char(c);

		for (int dy = 0; dy < FONT_HEIGHT; dy++) {
			int py = y + dy;
			if (py < 0 || py >= DISPLAY_HEIGHT) continue;
			uint16_t *row = fb + py * DISPLAY_WIDTH;
			uint8_t bits = glyph[dy];
			for (int dx = 0; dx < FONT_WIDTH; dx++) {
				int px = x + dx;
				if (px < 0 || px >= DISPLAY_WIDTH) continue;
				if (bits & (0x80 >> dx)) {
					row[px] = color;
				}
			}
		}
		x += FONT_WIDTH;
	}
}

void toast_show(const char *message)
{
	if (!message) return;
	strncpy(globals.toast_message, message, sizeof(globals.toast_message) - 1);
	globals.toast_message[sizeof(globals.toast_message) - 1] = '\0';
	globals.toast_hide_time = esp_log_timestamp() + TOAST_DURATION_MS;
}

void toast_clear(void)
{
	globals.toast_hide_time = 0;
	globals.toast_message[0] = '\0';
}

bool toast_is_visible(void)
{
	if (globals.toast_hide_time == 0) return false;
	return esp_log_timestamp() < globals.toast_hide_time;
}

void toast_render(uint16_t *fb)
{
	if (!fb) return;
	if (!toast_is_visible()) return;

	const char *msg = globals.toast_message;
	if (!msg[0]) return;

	// Calculate text width
	int text_len = strlen(msg);
	int text_w = text_len * FONT_WIDTH;

	// Toast box dimensions
	int box_w = text_w + TOAST_PADDING * 2;
	int box_h = FONT_HEIGHT + TOAST_PADDING * 2;
	int box_x = (DISPLAY_WIDTH - box_w) / 2;
	int box_y = DISPLAY_HEIGHT - TOAST_MARGIN_BOTTOM - box_h;

	// Draw background
	toast_fill_rect(fb, box_x, box_y, box_w, box_h, TOAST_BG_COLOR);

	// Draw border (1 pixel)
	toast_fill_rect(fb, box_x, box_y, box_w, 1, TOAST_BORDER_COLOR);
	toast_fill_rect(fb, box_x, box_y + box_h - 1, box_w, 1, TOAST_BORDER_COLOR);
	toast_fill_rect(fb, box_x, box_y, 1, box_h, TOAST_BORDER_COLOR);
	toast_fill_rect(fb, box_x + box_w - 1, box_y, 1, box_h, TOAST_BORDER_COLOR);

	// Draw text (centered in box)
	int text_x = box_x + TOAST_PADDING;
	int text_y = box_y + TOAST_PADDING;
	toast_draw_text(fb, text_x, text_y, msg, TOAST_TEXT_COLOR);
}

#endif /* USE_BADGE_BSP */
