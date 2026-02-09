/*
 * Display backend using badge-bsp for Tanmatsu device
 * Uses MIPI DSI display via BSP abstraction with PPA rotation
 *
 * The VGA emulator renders to an 800x480 landscape framebuffer.
 * PPA rotates it 270 degrees to 480x800 portrait for the physical display.
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "esp_lcd_panel_ops.h"
#include "driver/ppa.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include <stdatomic.h>
#include <string.h>

#include "bsp/device.h"
#include "bsp/display.h"
#include "esp_psram.h"

#include "common.h"
#include "tanmatsu_osd.h"
#include "toast.h"
#include "led_activity.h"
#include "ini_selector.h"
#include "esp_cache.h"
#include "vga_font_8x16.h"

static const char *TAG = "lcd_bsp";

// PPA client handle for rotation
static ppa_client_handle_t ppa_srm_handle = NULL;

// Physical display dimensions (portrait orientation)
#define DISPLAY_WIDTH  480
#define DISPLAY_HEIGHT 800

// Maximum VGA framebuffer - matches physical LCD dimensions (landscape)
// VBE modes or other content may need the full display resolution
#define VGA_MAX_WIDTH  800
#define VGA_MAX_HEIGHT 480

// Actual VGA dimensions - set from globals after PC init
// Defaults match physical LCD dimensions (landscape orientation)
static int vga_width = VGA_MAX_WIDTH;    // 800
static int vga_height = VGA_MAX_HEIGHT;  // 480

// Panel framebuffer for direct rendering (zero-copy blit)
static uint16_t *fb_rotated;

void pc_vga_step(void *o);

static esp_err_t ppa_init(void)
{
	ppa_client_config_t ppa_config = {
		.oper_type = PPA_OPERATION_SRM,
		.max_pending_trans_num = 1,
	};
	esp_err_t ret = ppa_register_client(&ppa_config, &ppa_srm_handle);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to register PPA SRM client: %s", esp_err_to_name(ret));
		return ret;
	}
	ESP_LOGI(TAG, "PPA SRM client registered for rotation");
	return ESP_OK;
}

static int blit_debug_count = 0;
static bool use_software_rotation = false;

// Software rotation with stride support using block-based approach for cache efficiency
// Source: 720x480 VGA in 800-stride buffer -> Dest: 480x720 portrait
// Block-based rotation improves cache hit rate significantly
#define ROTATE_BLOCK_SIZE 32

static void IRAM_ATTR software_rotate_90cw_stride(uint16_t *src, uint16_t *dst,
                                                   int src_w, int src_h, int src_stride)
{
	// 90 CW: src(x,y) -> dst(src_h-1-y, x)
	// dst_w = src_h, dst_h = src_w
	int dst_w = src_h;

	// Process in blocks for better cache utilization
	for (int by = 0; by < src_h; by += ROTATE_BLOCK_SIZE) {
		int y_end = by + ROTATE_BLOCK_SIZE;
		if (y_end > src_h) y_end = src_h;

		for (int bx = 0; bx < src_w; bx += ROTATE_BLOCK_SIZE) {
			int x_end = bx + ROTATE_BLOCK_SIZE;
			if (x_end > src_w) x_end = src_w;

			// Process one block
			for (int y = by; y < y_end; y++) {
				uint16_t *src_row = src + y * src_stride;
				int dst_x = src_h - 1 - y;
				for (int x = bx; x < x_end; x++) {
					dst[x * dst_w + dst_x] = src_row[x];
				}
			}
		}
	}
}

// Cached scale parameters (recalculated when VGA mode dimensions change)
static float cached_scale_x = 0, cached_scale_y = 0;
static int cached_offset_x = 0, cached_offset_y = 0;
static int cached_out_w = 0, cached_out_h = 0;
static int cached_mode_w = 0, cached_mode_h = 0, cached_pixel_double = 0;

static void update_scale_params(void)
{
	/* Get native VGA dimensions and pixel doubling factor from vga.c */
	int native_w = globals.vga_mode_width;
	int native_h = globals.vga_mode_height;
	int pixel_double = globals.vga_pixel_double;

	/* Sanity check - fall back to framebuffer dimensions if not set */
	if (native_w <= 0 || native_h <= 0) {
		native_w = vga_width;
		native_h = vga_height;
	}
	if (pixel_double <= 0) pixel_double = 1;

	if (cached_mode_w == native_w && cached_mode_h == native_h &&
	    cached_pixel_double == pixel_double)
		return;

	cached_mode_w = native_w;
	cached_mode_h = native_h;
	cached_pixel_double = pixel_double;

	/* Mode changed - clear all rotated framebuffers to black to avoid
	 * leaving behind artifacts in the outer areas not covered by the
	 * new scaled/centered content */
	size_t fb_rot_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
	if (fb_rotated) {
		memset(fb_rotated, 0, fb_rot_size);
		/* Flush cache to ensure DMA sees zeros */
		esp_cache_msync(fb_rotated, fb_rot_size,
		                ESP_CACHE_MSYNC_FLAG_DIR_C2M);
	}
	ESP_LOGI(TAG, "Mode change: cleared rotated buffer");

	/* Calculate intended VGA output dimensions (native * pixel_double).
	 * For mode 13h: native 320x200, intended 640x400 (2x both dimensions).
	 * For text mode: native 720x400, intended 720x400 (pixel_double=1). */
	int intended_w = native_w * pixel_double;
	int intended_h = native_h * pixel_double;

	/* Calculate uniform base scale to fit INTENDED dimensions in display.
	 * After 270° rotation: intended_w -> display Y, intended_h -> display X */
	float base_scale_x = (float)DISPLAY_HEIGHT / intended_w;
	float base_scale_y = (float)DISPLAY_WIDTH / intended_h;
	float base_scale = (base_scale_x < base_scale_y) ? base_scale_x : base_scale_y;

	/* Apply uniform scale to NATIVE dimensions:
	 * Both scale_x and scale_y include pixel_double factor since vga.c
	 * now outputs native resolution (320x200 for mode 13h) without any
	 * software pixel doubling or scanline repetition.
	 * After rotation: native_w (with scale_x) -> display height
	 *                 native_h (with scale_y) -> display width */
	float ideal_scale_x = base_scale * pixel_double;
	float ideal_scale_y = base_scale * pixel_double;

	/* PPA supports 1/16 scale precision - quantize both scales */
	int scale_x_16 = (int)(ideal_scale_x * 16.0f);
	int scale_y_16 = (int)(ideal_scale_y * 16.0f);
	if (scale_x_16 < 1) scale_x_16 = 1;
	if (scale_y_16 < 1) scale_y_16 = 1;
	cached_scale_x = scale_x_16 / 16.0f;
	cached_scale_y = scale_y_16 / 16.0f;

	/* Output dimensions after non-uniform scaling and rotation */
	cached_out_w = (int)(cached_scale_y * native_h);  /* rotated: height -> width */
	cached_out_h = (int)(cached_scale_x * native_w);  /* rotated: width -> height */
	cached_offset_x = (DISPLAY_WIDTH - cached_out_w) / 2;
	cached_offset_y = (DISPLAY_HEIGHT - cached_out_h) / 2;
	if (cached_offset_x < 0) cached_offset_x = 0;
	if (cached_offset_y < 0) cached_offset_y = 0;

	ESP_LOGI(TAG, "Scale: native %dx%d, double=%d, scale_x=%.3f scale_y=%.3f, "
	         "output %dx%d at (%d,%d)",
	         native_w, native_h, pixel_double, cached_scale_x, cached_scale_y,
	         cached_out_w, cached_out_h, cached_offset_x, cached_offset_y);
}

// Overlay bar rendering - using same colors as OSD (tanmatsu_osd.c)
#define OVERLAY_NONE       0
#define OVERLAY_BRIGHTNESS 1
#define OVERLAY_VOLUME     2

// OSD color scheme (RGB565)
#define OSD_COLOR_PANEL   0x18E3  // Dark blue-gray panel background
#define OSD_COLOR_BORDER  0x4A69  // Medium gray border
#define OSD_COLOR_TEXT    0xFFFF  // White
#define OSD_COLOR_HILITE  0x04FF  // Bright blue highlight
#define OSD_COLOR_TITLE   0xFFE0  // Yellow for titles
#define OSD_COLOR_VALUE   0x87FF  // Light cyan for values

// Draw filled rectangle (portrait buffer coordinates)
static void overlay_fill_rect(uint16_t *fb, int x, int y, int w, int h, uint16_t color)
{
	for (int dy = 0; dy < h; dy++) {
		if (y + dy < 0 || y + dy >= DISPLAY_HEIGHT) continue;
		uint16_t *row = fb + (y + dy) * DISPLAY_WIDTH;
		for (int dx = 0; dx < w; dx++) {
			if (x + dx < 0 || x + dx >= DISPLAY_WIDTH) continue;
			row[x + dx] = color;
		}
	}
}

// Logical landscape dimensions (same as OSD/toast coordinate system)
#define LOGICAL_WIDTH  800
#define LOGICAL_HEIGHT 480

// Convert logical landscape (x,y) to portrait buffer index
// Rotate 90 CCW: logical (x,y) -> buffer (phys_w-1-y, x)
static inline void stats_put_pixel(uint16_t *fb, int x, int y, uint16_t color)
{
	int bx = DISPLAY_WIDTH - 1 - y;
	int by = x;
	if (bx < 0 || bx >= DISPLAY_WIDTH || by < 0 || by >= DISPLAY_HEIGHT) return;
	fb[by * DISPLAY_WIDTH + bx] = color;
}

// Fill rect in logical landscape coords with rotation
static void stats_fill_rect(uint16_t *fb, int x, int y, int w, int h, uint16_t color)
{
	for (int dy = 0; dy < h; dy++) {
		int ly = y + dy;
		if (ly < 0 || ly >= LOGICAL_HEIGHT) continue;
		for (int dx = 0; dx < w; dx++) {
			int lx = x + dx;
			if (lx < 0 || lx >= LOGICAL_WIDTH) continue;
			stats_put_pixel(fb, lx, ly, color);
		}
	}
}

// Draw text in logical landscape coords with rotation
static void stats_draw_text(uint16_t *fb, int x, int y, const char *text, uint16_t color)
{
	while (*text) {
		unsigned char c = *text++;
		if (c < 32 || c > 126) c = '?';
		const uint8_t *glyph = vga_font_8x16_data[c - 32];
		for (int dy = 0; dy < 16; dy++) {
			uint8_t bits = glyph[dy];
			if (!bits) continue;
			int ly = y + dy;
			if (ly < 0 || ly >= LOGICAL_HEIGHT) continue;
			for (int dx = 0; dx < 8; dx++) {
				if (bits & (0x80 >> dx)) {
					int lx = x + dx;
					if (lx >= 0 && lx < LOGICAL_WIDTH)
						stats_put_pixel(fb, lx, ly, color);
				}
			}
		}
		x += 8;
	}
}

// Render persistent stats bar at top of screen (logical landscape coords)
static bool stats_bar_was_visible = false;
static void render_stats_bar(uint16_t *fb)
{
	if (!globals.stats_bar_visible) {
		if (stats_bar_was_visible) {
			// Clear the bar area — PPA doesn't overwrite this region
			stats_fill_rect(fb, 0, 0, LOGICAL_WIDTH, 20, 0x0000);
			stats_bar_was_visible = false;
		}
		return;
	}
	stats_bar_was_visible = true;

	// Bar at top of logical landscape: full width, 20px tall
	int bar_h = 20;
	stats_fill_rect(fb, 0, 0, LOGICAL_WIDTH, bar_h, 0x18E3);

	// Format stats: IPS ~MHz | CPU/Periph% | VGA WxH fps | Seq% | Bat N | Calls/s
	char line[100];
	uint32_t cps = globals.emu_cycles_per_sec;
	int pos = 0;
	if (cps >= 1000000) {
		pos += snprintf(line + pos, sizeof(line) - pos, " %.1fM IPS ~%luMHz",
		                cps / 1000000.0f, (unsigned long)(cps / 1000000));
	} else {
		pos += snprintf(line + pos, sizeof(line) - pos, " %luK IPS",
		                (unsigned long)(cps / 1000));
	}
	pos += snprintf(line + pos, sizeof(line) - pos,
	                " CPU:%d%% IO:%d%% %dx%d %dfps Seq:%d%% Bat:%d",
	                globals.emu_cpu_percent, globals.emu_periph_percent,
	                globals.vga_mode_width, globals.vga_mode_height,
	                globals.emu_vga_fps, globals.emu_seq_pct,
	                globals.emu_batch_size);

	// Draw text centered vertically in bar (bar_h=20, font=16, offset=2)
	stats_draw_text(fb, 4, 2, line, 0xFFFF);
}

// Render brightness/volume overlay bar
static void render_overlay_bar(uint16_t *fb)
{
	uint32_t now = esp_log_timestamp();

	// Check if overlay should still be visible
	if (globals.overlay_type == OVERLAY_NONE) return;
	if (now >= globals.overlay_hide_time) {
		globals.overlay_type = OVERLAY_NONE;
		return;
	}

	// Bar dimensions - centered horizontally, near bottom
	int bar_w = 300;
	int bar_h = 32;
	int bar_x = (DISPLAY_WIDTH - bar_w) / 2;
	int bar_y = DISPLAY_HEIGHT - 80;  // Near bottom of portrait display
	int padding = 4;
	int icon_area = 24;  // Space for icon on left

	// Colors matching OSD
	uint16_t bg_color = OSD_COLOR_PANEL;
	uint16_t border_color = OSD_COLOR_BORDER;
	uint16_t fill_color = (globals.overlay_type == OVERLAY_BRIGHTNESS)
	                      ? OSD_COLOR_TITLE    // Yellow for brightness
	                      : OSD_COLOR_VALUE;   // Light cyan for volume
	uint16_t icon_color = OSD_COLOR_TEXT;

	// Draw background
	overlay_fill_rect(fb, bar_x, bar_y, bar_w, bar_h, bg_color);

	// Draw border (1 pixel)
	overlay_fill_rect(fb, bar_x, bar_y, bar_w, 1, border_color);
	overlay_fill_rect(fb, bar_x, bar_y + bar_h - 1, bar_w, 1, border_color);
	overlay_fill_rect(fb, bar_x, bar_y, 1, bar_h, border_color);
	overlay_fill_rect(fb, bar_x + bar_w - 1, bar_y, 1, bar_h, border_color);

	// Draw fill bar background (darker area)
	int fill_area_x = bar_x + icon_area + padding;
	int fill_area_w = bar_w - icon_area - padding * 2;
	overlay_fill_rect(fb, fill_area_x, bar_y + padding,
	                  fill_area_w, bar_h - padding * 2, border_color);

	// Draw fill bar (active portion)
	int fill_w = fill_area_w * globals.overlay_value / 100;
	overlay_fill_rect(fb, fill_area_x, bar_y + padding,
	                  fill_w, bar_h - padding * 2, fill_color);

	// Draw icon (centered in icon area)
	int icon_cx = bar_x + icon_area / 2;
	int icon_cy = bar_y + bar_h / 2;
	if (globals.overlay_type == OVERLAY_BRIGHTNESS) {
		// Sun icon - simple cross/star pattern
		overlay_fill_rect(fb, icon_cx - 1, icon_cy - 5, 3, 10, icon_color);
		overlay_fill_rect(fb, icon_cx - 5, icon_cy - 1, 10, 3, icon_color);
		overlay_fill_rect(fb, icon_cx - 3, icon_cy - 3, 7, 7, icon_color);
	} else {
		// Speaker icon - simple wedge shape
		overlay_fill_rect(fb, icon_cx - 4, icon_cy - 2, 3, 5, icon_color);
		overlay_fill_rect(fb, icon_cx - 1, icon_cy - 4, 2, 9, icon_color);
		overlay_fill_rect(fb, icon_cx + 1, icon_cy - 5, 2, 11, icon_color);
	}
}

// Rotate VGA buffer to display buffer (does not blit to display)
static void IRAM_ATTR rotate_vga_to_display(uint16_t *fb, uint16_t *fb_rotated)
{
	if (!fb || !fb_rotated) {
		if (blit_debug_count < 5) {
			ESP_LOGE(TAG, "rotate: invalid params fb=%p fb_rot=%p",
			         fb, fb_rotated);
			blit_debug_count++;
		}
		return;
	}

	update_scale_params();

	// Debug: print buffer info once
	if (blit_debug_count == 0) {
		ESP_LOGI(TAG, "PPA blit: fb=%p (ext=%d) fb_rot=%p (ext=%d)",
		         fb, esp_ptr_external_ram(fb),
		         fb_rotated, esp_ptr_external_ram(fb_rotated));
		ESP_LOGI(TAG, "VGA: %dx%d (x%d), scale %.3fx%.3f, out %dx%d at (%d,%d)",
		         cached_mode_w, cached_mode_h, cached_pixel_double,
		         cached_scale_x, cached_scale_y,
		         cached_out_w, cached_out_h, cached_offset_x, cached_offset_y);
		ESP_LOGI(TAG, "Display: %dx%d", DISPLAY_WIDTH, DISPLAY_HEIGHT);
		blit_debug_count++;
	}

	// Try PPA first if not already failed
	if (!use_software_rotation && ppa_srm_handle) {
		/* PPA reads native VGA content and scales with pixel doubling.
		 * scale_x includes pixel_double factor (e.g., 2x for mode 13h).
		 * This offloads pixel doubling from CPU to PPA hardware. */
		ppa_srm_oper_config_t srm_config = {
			.in.buffer = fb,
			.in.pic_w = vga_width,           /* Full framebuffer stride */
			.in.pic_h = vga_height,
			.in.block_w = cached_mode_w,     /* Native VGA content width */
			.in.block_h = cached_mode_h,
			.in.block_offset_x = 0,          /* Content at top-left */
			.in.block_offset_y = 0,
			.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
			.out.buffer = fb_rotated,
			.out.buffer_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t),
			.out.pic_w = DISPLAY_WIDTH,
			.out.pic_h = DISPLAY_HEIGHT,
			.out.block_offset_x = cached_offset_x,
			.out.block_offset_y = cached_offset_y,
			.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
			.rotation_angle = PPA_SRM_ROTATION_ANGLE_270,
			.scale_x = cached_scale_x,       /* Includes pixel_double factor */
			.scale_y = cached_scale_y,
			.rgb_swap = 0,
			.byte_swap = 0,
			.mode = PPA_TRANS_MODE_BLOCKING,
		};

		esp_err_t ret = ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config);
		if (ret == ESP_OK) {
			return;
		}

		// PPA failed, switch to software rotation permanently
		ESP_LOGW(TAG, "PPA failed (%s), using software rotation", esp_err_to_name(ret));
		use_software_rotation = true;
	}

	// Software rotation fallback (no scaling, just rotation)
	software_rotate_90cw_stride(fb, fb_rotated, vga_width, vga_height, vga_width);
}

// Set VGA framebuffer dimensions - called after PC config is loaded
void lcd_set_vga_dimensions(int width, int height)
{
	// Clamp to allocated framebuffer size
	if (width > VGA_MAX_WIDTH) width = VGA_MAX_WIDTH;
	if (height > VGA_MAX_HEIGHT) height = VGA_MAX_HEIGHT;
	vga_width = width;
	vga_height = height;
	globals.vga_width = width;
	globals.vga_height = height;
	// Reset cached params to force recalculation on next frame
	cached_mode_w = 0;
	cached_mode_h = 0;
	ESP_LOGI(TAG, "VGA framebuffer dimensions set to %dx%d", width, height);
}

// This function is called by the VGA emulator but we do full-frame updates instead
void lcd_draw(int x_start, int y_start, int x_end, int y_end, void *src)
{
	// No-op: we do full-frame rotation in the main loop instead of strip updates
	(void)x_start;
	(void)y_start;
	(void)x_end;
	(void)y_end;
	(void)src;
}

void vga_task(void *arg)
{
	// BSP already initialized in app_main - just get panel handle
	esp_lcd_panel_handle_t panel = NULL;
	ESP_ERROR_CHECK(bsp_display_get_panel(&panel));
	ESP_ERROR_CHECK(bsp_display_set_backlight_brightness(30));
	ESP_ERROR_CHECK(ppa_init());
	globals.panel = panel;

	// Enable tearing effect sync to eliminate screen tearing
	SemaphoreHandle_t te_sem = NULL;
	ESP_ERROR_CHECK(bsp_display_set_tearing_effect_mode(BSP_DISPLAY_TE_V_BLANKING));
	ESP_ERROR_CHECK(bsp_display_get_tearing_effect_semaphore(&te_sem));

	// Initialize drive activity LEDs
	led_activity_init();

	// Allocate framebuffers (after BSP init, like trackmatsu)
	size_t fb_size = VGA_MAX_WIDTH * VGA_MAX_HEIGHT * sizeof(uint16_t);
	uint16_t *fb = heap_caps_aligned_alloc(64, fb_size,
	                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
	if (!fb) {
		fprintf(stderr, "Failed to allocate VGA framebuffer\n");
		vTaskDelete(NULL);
		return;
	}
	memset(fb, 0, fb_size);
	atomic_store(&globals.fb, fb);

	// Allocate separate rotation buffer — we render here while DPI
	// hardware scans the panel's own FB.  After TE blanking signal,
	// bsp_display_blit DMA-copies this buffer into the panel FB.
	size_t fb_rot_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
	fb_rotated = heap_caps_aligned_alloc(64, fb_rot_size,
	                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
	if (!fb_rotated) {
		fprintf(stderr, "Failed to allocate rotation framebuffer\n");
		vTaskDelete(NULL);
		return;
	}
	memset(fb_rotated, 0, fb_rot_size);
	globals.fb_rotated = fb_rotated;

	xEventGroupSetBits(global_event_group, BIT1);  // Signal display ready

	ESP_LOGI(TAG, "Starting display loop");

	while (1) {
		/* ── INI selector screen (no PC running yet) ──────────── */
		if (globals.ini_selector_active) {
			ini_selector_render(fb_rotated,
			                    DISPLAY_WIDTH, DISPLAY_HEIGHT,
			                    DISPLAY_WIDTH * sizeof(uint16_t));
			toast_render(fb_rotated);

			if (xSemaphoreTake(te_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
				ESP_LOGW(TAG, "TE signal timeout — blitting without vsync");
			}
			bsp_display_blit(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, fb_rotated);
			vTaskDelay(pdMS_TO_TICKS(33));  /* ~30 fps for selector */
			continue;
		}

		/* ── PC not ready — idle until BIT0 ──────────────────── */
		if (!globals.pc) {
			vTaskDelay(pdMS_TO_TICKS(50));
			continue;
		}

		/* ── Normal VGA rendering ────────────────────────────── */
		// Step the VGA emulator (vga_step + vga_refresh)
		pc_vga_step(globals.pc);

		// Rotate + overlay into our buffer, then DMA to panel FB at TE
		if (globals.fb) {
			/* Clear ghost pixels when OSD or overlay bar is dismissed.
			 * PPA rotation only overwrites the scaled VGA region, so
			 * pixels drawn in border areas by overlays persist. */
			static bool had_overlay = false;
			bool has_overlay = (globals.osd_enabled && globals.osd) ||
			                   (globals.overlay_type != OVERLAY_NONE);
			if (had_overlay && !has_overlay) {
				size_t sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
				memset(fb_rotated, 0, sz);
				esp_cache_msync(fb_rotated, sz,
				                ESP_CACHE_MSYNC_FLAG_DIR_C2M);
			}

			rotate_vga_to_display((uint16_t *)globals.fb, fb_rotated);

			// Render OSD overlay
			if (globals.osd_enabled && globals.osd) {
				osd_render(globals.osd, (uint8_t *)fb_rotated,
					   DISPLAY_WIDTH, DISPLAY_HEIGHT,
					   DISPLAY_WIDTH * sizeof(uint16_t));
			}

			// Render brightness/volume overlay (if not showing full OSD)
			if (!globals.osd_enabled) {
				render_overlay_bar(fb_rotated);
			}

			/* Update overlay tracking (after render_overlay_bar may
			 * have expired the timer and set overlay_type to NONE) */
			had_overlay = (globals.osd_enabled && globals.osd) ||
			              (globals.overlay_type != OVERLAY_NONE);

			// Persistent stats bar (renders on top of everything including OSD)
			render_stats_bar(fb_rotated);

			// Toast renders on top of everything (including OSD)
			toast_render(fb_rotated);

			// Wait for vertical blanking then DMA blit to panel
			if (xSemaphoreTake(te_sem, pdMS_TO_TICKS(100)) != pdTRUE) {
				ESP_LOGW(TAG, "TE signal timeout — blitting without vsync");
			}
			bsp_display_blit(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, fb_rotated);
		}

		// Update drive activity LEDs (turn off after timeout)
		led_activity_tick();
	}
}
