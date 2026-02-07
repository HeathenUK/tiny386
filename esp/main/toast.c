/*
 * Toast notification system for Tanmatsu
 * Displays brief text messages at the bottom of the screen
 * Optimized: renders only when visible, caches when unchanged
 */

#include "toast.h"
#include "common.h"
#include "vga_font_8x16.h"
#include "esp_log.h"
#include <string.h>

// Physical display dimensions (portrait buffer)
#define PHYS_W 480
#define PHYS_H 800

// Logical dimensions (landscape, same as OSD)
#define LOGICAL_WIDTH  800
#define LOGICAL_HEIGHT 480

// Font dimensions
#define FONT_WIDTH 8
#define FONT_HEIGHT 16

// Toast styling
#define TOAST_BG_COLOR    0x18E3  // Dark blue-gray
#define TOAST_BORDER_COLOR 0x4A69  // Medium gray border
#define TOAST_TEXT_COLOR  0xFFFF  // White text
#define TOAST_PADDING     8
#define TOAST_MARGIN_BOTTOM 40
#define TOAST_FADE_MS     300     // Fade out duration

// Rendering state
static uint16_t *toast_fb = NULL;
static int toast_alpha = 255;

// Cache to avoid re-rendering unchanged toasts
static char cached_msg[64] = {0};
static int cached_alpha = -1;

// Convert logical (x,y) to physical buffer index
// Rotate 90 CCW: logical (x,y) -> buffer (phys_w-1-y, x)
static inline int toast_idx(int x, int y)
{
	int bx = PHYS_W - 1 - y;
	int by = x;
	return by * PHYS_W + bx;
}

// Blend RGB565 color with background using alpha (0-255)
static inline uint16_t blend_rgb565(uint16_t fg, uint16_t bg, int alpha)
{
	if (alpha >= 255) return fg;
	if (alpha <= 0) return bg;

	int fg_r = (fg >> 11) & 0x1f;
	int fg_g = (fg >> 5) & 0x3f;
	int fg_b = fg & 0x1f;

	int bg_r = (bg >> 11) & 0x1f;
	int bg_g = (bg >> 5) & 0x3f;
	int bg_b = bg & 0x1f;

	int r = (fg_r * alpha + bg_r * (255 - alpha)) / 255;
	int g = (fg_g * alpha + bg_g * (255 - alpha)) / 255;
	int b = (fg_b * alpha + bg_b * (255 - alpha)) / 255;

	return (r << 11) | (g << 5) | b;
}

// Draw horizontal line (optimized - writes sequential memory when possible)
static void toast_hline(int x, int y, int w, uint16_t color)
{
	if (y < 0 || y >= LOGICAL_HEIGHT) return;
	if (x < 0) { w += x; x = 0; }
	if (x + w > LOGICAL_WIDTH) w = LOGICAL_WIDTH - x;
	if (w <= 0) return;

	// For a horizontal line in logical coords, we write to a vertical
	// strip in physical coords (non-contiguous), so just loop
	if (toast_alpha >= 255) {
		for (int i = 0; i < w; i++) {
			toast_fb[toast_idx(x + i, y)] = color;
		}
	} else {
		for (int i = 0; i < w; i++) {
			int idx = toast_idx(x + i, y);
			toast_fb[idx] = blend_rgb565(color, toast_fb[idx], toast_alpha);
		}
	}
}

// Draw vertical line (optimized - contiguous memory access)
static void toast_vline(int x, int y, int h, uint16_t color)
{
	if (x < 0 || x >= LOGICAL_WIDTH) return;
	if (y < 0) { h += y; y = 0; }
	if (y + h > LOGICAL_HEIGHT) h = LOGICAL_HEIGHT - y;
	if (h <= 0) return;

	// Vertical line in logical = horizontal in physical = contiguous!
	int bx_start = PHYS_W - 1 - y;
	int by = x;
	uint16_t *row = &toast_fb[by * PHYS_W + bx_start];

	if (toast_alpha >= 255) {
		for (int i = 0; i < h; i++) {
			row[-i] = color;
		}
	} else {
		for (int i = 0; i < h; i++) {
			row[-i] = blend_rgb565(color, row[-i], toast_alpha);
		}
	}
}

// Draw filled rectangle - optimized using hlines
static void toast_fill_rect(int x, int y, int w, int h, uint16_t color)
{
	for (int dy = 0; dy < h; dy++) {
		toast_hline(x, y + dy, w, color);
	}
}

// Draw text - optimized to skip empty pixels
static void toast_draw_text(int x, int y, const char *text, uint16_t color)
{
	while (*text) {
		unsigned char c = *text++;
		if (c < 32 || c > 126) c = '?';

		const uint8_t *glyph = vga_font_8x16_data[c - 32];

		for (int dy = 0; dy < FONT_HEIGHT; dy++) {
			uint8_t bits = glyph[dy];
			if (!bits) continue;  // Skip empty rows

			int ly = y + dy;
			if (ly < 0 || ly >= LOGICAL_HEIGHT) continue;

			// Process each set bit
			for (int dx = 0; dx < FONT_WIDTH; dx++) {
				if (bits & (0x80 >> dx)) {
					int lx = x + dx;
					if (lx >= 0 && lx < LOGICAL_WIDTH) {
						int idx = toast_idx(lx, ly);
						if (toast_alpha >= 255) {
							toast_fb[idx] = color;
						} else {
							toast_fb[idx] = blend_rgb565(color, toast_fb[idx], toast_alpha);
						}
					}
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
	// Invalidate cache
	cached_msg[0] = '\0';
	cached_alpha = -1;
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

	// Calculate alpha for fade-out effect
	uint32_t now = esp_log_timestamp();
	uint32_t remaining = globals.toast_hide_time - now;
	if (remaining <= TOAST_FADE_MS) {
		toast_alpha = (remaining * 255) / TOAST_FADE_MS;
	} else {
		toast_alpha = 255;
	}

	// Set up rendering state
	toast_fb = fb;

	// Calculate text width
	int text_len = strlen(msg);
	int text_w = text_len * FONT_WIDTH;

	// Toast box dimensions (in logical 800x480 coords)
	int box_w = text_w + TOAST_PADDING * 2;
	int box_h = FONT_HEIGHT + TOAST_PADDING * 2;
	int box_x = (LOGICAL_WIDTH - box_w) / 2;
	int box_y = LOGICAL_HEIGHT - TOAST_MARGIN_BOTTOM - box_h;

	// Draw background
	toast_fill_rect(box_x, box_y, box_w, box_h, TOAST_BG_COLOR);

	// Draw border (4 lines instead of 4 fill_rects)
	toast_hline(box_x, box_y, box_w, TOAST_BORDER_COLOR);
	toast_hline(box_x, box_y + box_h - 1, box_w, TOAST_BORDER_COLOR);
	toast_vline(box_x, box_y, box_h, TOAST_BORDER_COLOR);
	toast_vline(box_x + box_w - 1, box_y, box_h, TOAST_BORDER_COLOR);

	// Draw text
	int text_x = box_x + TOAST_PADDING;
	int text_y = box_y + TOAST_PADDING;
	toast_draw_text(text_x, text_y, msg, TOAST_TEXT_COLOR);
}
