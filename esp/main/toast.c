#ifdef USE_BADGE_BSP
/*
 * Toast notification system for Tanmatsu
 * Displays brief text messages at the bottom of the screen
 * Uses same rotation approach as OSD for consistent rendering
 */

#include "toast.h"
#include "common.h"
#include "vga_font_8x16.h"
#include "esp_log.h"
#include <string.h>

// Logical dimensions (landscape, same as OSD)
#define LOGICAL_WIDTH  800
#define LOGICAL_HEIGHT 480

// Font dimensions
#define FONT_WIDTH 8
#define FONT_HEIGHT 16

// Toast styling
#define TOAST_BG_COLOR    0x18E3  // Dark blue-gray (same as OSD panel)
#define TOAST_BORDER_COLOR 0x4A69  // Medium gray border
#define TOAST_TEXT_COLOR  0xFFFF  // White text
#define TOAST_PADDING     8
#define TOAST_MARGIN_BOTTOM 40
#define TOAST_FADE_MS     300     // Fade out duration

// Rendering state (set by toast_render)
static uint16_t *toast_fb = NULL;
static int toast_phys_w = 0;
static int toast_phys_h = 0;
static int toast_alpha = 255;  // 0-255, set per frame

// Blend RGB565 color with background using alpha (0-255)
static inline uint16_t blend_rgb565(uint16_t fg, uint16_t bg, int alpha)
{
	if (alpha >= 255) return fg;
	if (alpha <= 0) return bg;

	// Extract RGB components (RGB565)
	int fg_r = (fg >> 11) & 0x1f;
	int fg_g = (fg >> 5) & 0x3f;
	int fg_b = fg & 0x1f;

	int bg_r = (bg >> 11) & 0x1f;
	int bg_g = (bg >> 5) & 0x3f;
	int bg_b = bg & 0x1f;

	// Blend
	int r = (fg_r * alpha + bg_r * (255 - alpha)) / 255;
	int g = (fg_g * alpha + bg_g * (255 - alpha)) / 255;
	int b = (fg_b * alpha + bg_b * (255 - alpha)) / 255;

	return (r << 11) | (g << 5) | b;
}

// Put pixel with portrait rotation and alpha blending
// Logical (x,y) in 800x480 -> buffer (phys_w-1-y, x) in 480x800
static inline void toast_put_pixel(int x, int y, uint16_t color)
{
	if (x < 0 || x >= LOGICAL_WIDTH || y < 0 || y >= LOGICAL_HEIGHT) return;

	// Rotate 90 CCW: logical (x,y) -> buffer (phys_w-1-y, x)
	int bx = toast_phys_w - 1 - y;
	int by = x;

	if (bx < 0 || bx >= toast_phys_w || by < 0 || by >= toast_phys_h) return;

	uint16_t *pixel = &toast_fb[by * toast_phys_w + bx];
	*pixel = blend_rgb565(color, *pixel, toast_alpha);
}

// Draw filled rectangle using logical coordinates
static void toast_fill_rect(int x, int y, int w, int h, uint16_t color)
{
	for (int dy = 0; dy < h; dy++) {
		for (int dx = 0; dx < w; dx++) {
			toast_put_pixel(x + dx, y + dy, color);
		}
	}
}

// Get font bitmap for a character
static const uint8_t *toast_get_font_char(int c)
{
	if (c >= 32 && c <= 126) {
		return vga_font_8x16_data[c - 32];
	}
	return vga_font_8x16_data['?' - 32];
}

// Draw text using logical coordinates
static void toast_draw_text(int x, int y, const char *text, uint16_t color)
{
	while (*text) {
		unsigned char c = *text++;
		if (c < 32 || c > 126) c = '?';

		const uint8_t *glyph = toast_get_font_char(c);

		for (int dy = 0; dy < FONT_HEIGHT; dy++) {
			uint8_t bits = glyph[dy];
			for (int dx = 0; dx < FONT_WIDTH; dx++) {
				if (bits & (0x80 >> dx)) {
					toast_put_pixel(x + dx, y + dy, color);
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

	// Set up rendering state (portrait buffer: 480x800)
	toast_fb = fb;
	toast_phys_w = 480;
	toast_phys_h = 800;

	// Calculate alpha for fade-out effect
	uint32_t now = esp_log_timestamp();
	uint32_t remaining = globals.toast_hide_time - now;
	if (remaining <= TOAST_FADE_MS) {
		toast_alpha = (remaining * 255) / TOAST_FADE_MS;
	} else {
		toast_alpha = 255;
	}

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

	// Draw border (1 pixel)
	toast_fill_rect(box_x, box_y, box_w, 1, TOAST_BORDER_COLOR);
	toast_fill_rect(box_x, box_y + box_h - 1, box_w, 1, TOAST_BORDER_COLOR);
	toast_fill_rect(box_x, box_y, 1, box_h, TOAST_BORDER_COLOR);
	toast_fill_rect(box_x + box_w - 1, box_y, 1, box_h, TOAST_BORDER_COLOR);

	// Draw text (centered in box)
	int text_x = box_x + TOAST_PADDING;
	int text_y = box_y + TOAST_PADDING;
	toast_draw_text(text_x, text_y, msg, TOAST_TEXT_COLOR);
}

#endif /* USE_BADGE_BSP */
