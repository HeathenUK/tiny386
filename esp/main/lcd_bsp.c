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
#include "driver/jpeg_encode.h"
#include "../../pc.h"
#include "../../vga.h"

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
static int cached_mode_w = 0, cached_mode_h = 0;
static int cached_pixel_double_x = 0, cached_pixel_double_y = 0;

static void update_scale_params(void)
{
	/* Get native VGA dimensions and pixel doubling factors from vga.c */
	int native_w = globals.vga_mode_width;
	int native_h = globals.vga_mode_height;
	int pdx = globals.vga_pixel_double;    /* horizontal (pixel clock halved) */
	int pdy = globals.vga_pixel_double_y;  /* vertical (double-scan) */

	/* Sanity check - fall back to framebuffer dimensions if not set */
	if (native_w <= 0 || native_h <= 0) {
		native_w = vga_width;
		native_h = vga_height;
	}
	if (pdx <= 0) pdx = 1;
	if (pdy <= 0) pdy = 1;

	if (cached_mode_w == native_w && cached_mode_h == native_h &&
	    cached_pixel_double_x == pdx && cached_pixel_double_y == pdy)
		return;

	cached_mode_w = native_w;
	cached_mode_h = native_h;
	cached_pixel_double_x = pdx;
	cached_pixel_double_y = pdy;

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

	/* Calculate intended VGA output dimensions.
	 * Horizontal (pdx) and vertical (pdy) doubling are separate because
	 * 256-color modes always halve the pixel clock (pdx=2) but double-scan
	 * may be disabled (pdy=1) — e.g., Win95 splash uses 320x400 where
	 * only horizontal doubling applies, giving intended 640x400. */
	int intended_w = native_w * pdx;
	int intended_h = native_h * pdy;

	/* Calculate uniform base scale to fit INTENDED dimensions in display.
	 * After 270° rotation: intended_w -> display Y, intended_h -> display X */
	float base_scale_x = (float)DISPLAY_HEIGHT / intended_w;
	float base_scale_y = (float)DISPLAY_WIDTH / intended_h;
	float base_scale = (base_scale_x < base_scale_y) ? base_scale_x : base_scale_y;

	/* Apply base scale to NATIVE dimensions, incorporating the respective
	 * pixel doubling factor for each axis.
	 * After rotation: native_w (with scale_x) -> display height
	 *                 native_h (with scale_y) -> display width */
	float ideal_scale_x = base_scale * pdx;
	float ideal_scale_y = base_scale * pdy;

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

	ESP_LOGI(TAG, "Scale: native %dx%d, pdx=%d pdy=%d, scale_x=%.3f scale_y=%.3f, "
	         "output %dx%d at (%d,%d)",
	         native_w, native_h, pdx, pdy, cached_scale_x, cached_scale_y,
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
#define STATS_BAR_H 36
static void render_stats_bar(uint16_t *fb)
{
	if (!globals.stats_bar_visible) {
		stats_bar_was_visible = false;
		return;
	}
	stats_bar_was_visible = true;

	// Bar at top of logical landscape: full width, 2 lines
	stats_fill_rect(fb, 0, 0, LOGICAL_WIDTH, STATS_BAR_H, 0x18E3);

	// Line 1: IPS ~MHz | CPU/IO% | VGA WxH fps | Seq% | Bat | HLT%
	char line1[120];
	uint32_t cps = globals.emu_cycles_per_sec;
	int pos = 0;
	if (cps >= 1000000) {
		pos += snprintf(line1 + pos, sizeof(line1) - pos, " %.1fM ~%luMHz",
		                cps / 1000000.0f, (unsigned long)(cps / 1000000));
	} else {
		pos += snprintf(line1 + pos, sizeof(line1) - pos, " %luK",
		                (unsigned long)(cps / 1000));
	}
	pos += snprintf(line1 + pos, sizeof(line1) - pos,
	                " CPU:%d%% IO:%d%% %dx%d %dfps Seq:%d%% Bat:%d HLT:%d%%",
	                globals.emu_cpu_percent, globals.emu_periph_percent,
	                globals.vga_mode_width, globals.vga_mode_height,
	                globals.emu_vga_fps, globals.emu_seq_pct,
	                globals.emu_batch_size, globals.emu_hlt_pct);

	// Line 2: TLB/s | IRQ/s | Fus% | HLE% | Disk KB/s | SRAM | PSRAM
	char line2[120];
	uint32_t sram_kb = globals.emu_free_sram / 1024;
	uint32_t psram_kb = globals.emu_free_psram / 1024;
	snprintf(line2, sizeof(line2),
	         " TLB:%luk/s IRQ:%lu/s Fus:%d%% HLE:%d%% Dsk:%lukB/s SRAM:%lukB PSRAM:%lukB",
	         (unsigned long)(globals.emu_tlb_miss_per_sec / 1000),
	         (unsigned long)globals.emu_irq_per_sec,
	         globals.emu_fusion_pct, globals.emu_hle_pct,
	         (unsigned long)globals.emu_disk_kb_per_sec,
	         (unsigned long)sram_kb, (unsigned long)psram_kb);

	// Draw two lines of text (font=16px, line spacing=18)
	stats_draw_text(fb, 4, 1, line1, 0xFFFF);
	stats_draw_text(fb, 4, 19, line2, 0xBDF7);  // Slightly dimmer for line 2
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
	int overlay_max = (globals.overlay_type == OVERLAY_BRIGHTNESS) ? BRIGHTNESS_MAX : 100;
	int overlay_value = globals.overlay_value;
	if (overlay_value < 0) overlay_value = 0;
	if (overlay_value > overlay_max) overlay_value = overlay_max;
	int fill_w = fill_area_w * overlay_value / overlay_max;
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
		ESP_LOGI(TAG, "VGA: %dx%d (x%d/%d), scale %.3fx%.3f, out %dx%d at (%d,%d)",
		         cached_mode_w, cached_mode_h, cached_pixel_double_x, cached_pixel_double_y,
		         cached_scale_x, cached_scale_y,
		         cached_out_w, cached_out_h, cached_offset_x, cached_offset_y);
		ESP_LOGI(TAG, "Display: %dx%d", DISPLAY_WIDTH, DISPLAY_HEIGHT);
		blit_debug_count++;
	}

	// Try PPA first if not already failed
	if (!use_software_rotation && ppa_srm_handle) {
		/* Flush VGA framebuffer from CPU cache to PSRAM before PPA DMA reads it.
		 * Without this, PPA may read stale cache lines → scanline misalignment. */
		size_t fb_size = (size_t)vga_width * vga_height * sizeof(uint16_t);
		esp_cache_msync(fb, fb_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

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

/* ── JPEG screenshot capture ─────────────────────────────────── */
static jpeg_encoder_handle_t jpeg_enc = NULL;

static void screenshot_capture(uint16_t *fb, int width, int height)
{
	if (!fb || width <= 0 || height <= 0) {
		toast_show("Screenshot: no framebuffer");
		return;
	}

	if (!jpeg_enc) {
		toast_show("Screenshot: no encoder");
		return;
	}

	size_t in_size = (size_t)width * height * 2;

	/* Flush fb cache before JPEG DMA reads it */
	esp_cache_msync(fb, in_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

	/* Allocate output buffer with proper cache-line alignment for DMA */
	size_t out_alloc = in_size / 5 + 4096;
	jpeg_encode_memory_alloc_cfg_t out_mem_cfg = {
		.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
	};
	size_t out_actual = 0;
	uint8_t *out_buf = (uint8_t *)jpeg_alloc_encoder_mem(
		out_alloc, &out_mem_cfg, &out_actual);
	if (!out_buf) {
		ESP_LOGE(TAG, "JPEG output buffer alloc failed (%u bytes)", (unsigned)out_alloc);
		toast_show("Screenshot: no memory");
		return;
	}

	/* Encode RGB565 → JPEG */
	jpeg_encode_cfg_t enc_cfg = {
		.width = width,
		.height = height,
		.src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
		.sub_sample = JPEG_DOWN_SAMPLING_YUV420,
		.image_quality = 80,
	};

	uint32_t jpg_size = 0;
	esp_err_t ret = jpeg_encoder_process(jpeg_enc, &enc_cfg,
		(const uint8_t *)fb, in_size,
		out_buf, out_actual, &jpg_size);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "JPEG encode failed: %s", esp_err_to_name(ret));
		free(out_buf);
		toast_show("Screenshot: encode failed");
		return;
	}

	/* Generate filename from uptime timestamp (avoids collision) */
	char fname[48];
	snprintf(fname, sizeof(fname), "/sdcard/scr_%08lu.jpg",
	         (unsigned long)esp_log_timestamp());

	FILE *f = fopen(fname, "wb");
	if (!f) {
		ESP_LOGE(TAG, "Failed to create %s", fname);
		free(out_buf);
		toast_show("Screenshot: SD error");
		return;
	}
	size_t written = fwrite(out_buf, 1, jpg_size, f);
	fclose(f);
	free(out_buf);

	if (written != jpg_size) {
		ESP_LOGE(TAG, "Screenshot write incomplete: %u/%lu",
		         (unsigned)written, (unsigned long)jpg_size);
		toast_show("Screenshot: write error");
		return;
	}

	ESP_LOGI(TAG, "Screenshot: %s (%lu bytes)", fname, (unsigned long)jpg_size);

	/* Dump VRAM plane data to a .txt file alongside the screenshot */
	if (globals.pc) {
		PC *pc = (PC *)globals.pc;
		if (pc->vga) {
			char txtname[48];
			snprintf(txtname, sizeof(txtname), "/sdcard/scr_%08lu.txt",
			         (unsigned long)esp_log_timestamp());
			vga_dump_vram_to_file(pc->vga, txtname, height);
		}
	}

	char msg[48];
	snprintf(msg, sizeof(msg), "Screenshot %luKB saved",
	         (unsigned long)(jpg_size / 1024));
	toast_show(msg);
}

void vga_task(void *arg)
{
	// BSP already initialized in app_main - just get panel handle
	esp_lcd_panel_handle_t panel = NULL;
	ESP_ERROR_CHECK(bsp_display_get_panel(&panel));
	ESP_ERROR_CHECK(bsp_display_set_backlight_brightness(brightness_to_bsp_percent(globals.brightness)));
	ESP_ERROR_CHECK(ppa_init());
	globals.panel = panel;

	// Enable tearing effect sync to eliminate screen tearing
	SemaphoreHandle_t te_sem = NULL;
	ESP_ERROR_CHECK(bsp_display_set_tearing_effect_mode(BSP_DISPLAY_TE_V_BLANKING));
	ESP_ERROR_CHECK(bsp_display_get_tearing_effect_semaphore(&te_sem));

	// Initialize drive activity LEDs
	led_activity_init();

	// Pre-init JPEG encoder while internal DMA SRAM is still available
	{
		jpeg_encode_engine_cfg_t cfg = {
			.intr_priority = 0,
			.timeout_ms = 3000,
		};
		esp_err_t jerr = jpeg_new_encoder_engine(&cfg, &jpeg_enc);
		if (jerr != ESP_OK) {
			ESP_LOGW(TAG, "JPEG encoder init failed: %s (screenshots disabled)",
			         esp_err_to_name(jerr));
		}
	}

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

	bool pc_gone_cleared = false;
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
			/* Clear display once when PC goes away (INI switch).
			 * Without this, the old session's last frame lingers. */
			if (!pc_gone_cleared) {
				memset(fb_rotated, 0, fb_rot_size);
				if (xSemaphoreTake(te_sem, pdMS_TO_TICKS(100)) == pdTRUE)
					bsp_display_blit(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, fb_rotated);
				pc_gone_cleared = true;
			}
			vTaskDelay(pdMS_TO_TICKS(50));
			continue;
		}
		pc_gone_cleared = false;

		/* ── Normal VGA rendering ────────────────────────────── */
		// Step the VGA emulator (vga_step + vga_refresh)
		pc_vga_step(globals.pc);

		/* Screenshot capture (before rotation/overlays) */
		if (globals.screenshot_pending && globals.fb) {
			globals.screenshot_pending = false;
			screenshot_capture((uint16_t *)globals.fb, vga_width, vga_height);
		}

		/* Burst capture: save consecutive frames as JPEGs for motion analysis */
		if (globals.burst_capture_remaining > 0 && globals.fb) {
			static int burst_seq = 0;
			int remaining = globals.burst_capture_remaining;
			if (remaining == 20) {
				burst_seq = 0;  /* Reset sequence on new burst */
				toast_show("Burst capture...");
			}
			/* Encode and save this frame */
			if (jpeg_enc) {
				/* Flush fb cache before JPEG DMA reads it */
				size_t in_size = (size_t)vga_width * vga_height * 2;
				esp_cache_msync(globals.fb, in_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
				size_t out_alloc = in_size / 5 + 4096;
				jpeg_encode_memory_alloc_cfg_t out_mem_cfg = {
					.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
				};
				size_t out_actual = 0;
				uint8_t *out_buf = (uint8_t *)jpeg_alloc_encoder_mem(
					out_alloc, &out_mem_cfg, &out_actual);
				if (out_buf) {
					jpeg_encode_cfg_t enc_cfg = {
						.width = vga_width,
						.height = vga_height,
						.src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
						.sub_sample = JPEG_DOWN_SAMPLING_YUV420,
						.image_quality = 90,
					};
					uint32_t jpg_size = 0;
					esp_err_t ret = jpeg_encoder_process(jpeg_enc, &enc_cfg,
						(const uint8_t *)globals.fb, in_size,
						out_buf, out_actual, &jpg_size);
					if (ret == ESP_OK) {
						char fname[48];
						snprintf(fname, sizeof(fname),
						         "/sdcard/burst_%03d.jpg", burst_seq);
						FILE *f = fopen(fname, "wb");
						if (f) {
							fwrite(out_buf, 1, jpg_size, f);
							fclose(f);
						}
					}
					free(out_buf);
				}
			}
			burst_seq++;
			globals.burst_capture_remaining = remaining - 1;
			if (remaining - 1 == 0) {
				char msg[48];
				snprintf(msg, sizeof(msg), "Burst: %d frames saved", burst_seq);
				toast_show(msg);
			}
		}

		// Rotate + overlay into our buffer, then DMA to panel FB at TE
		if (globals.fb) {
			/* Clear ghost pixels when OSD or overlay bar is dismissed.
			 * PPA rotation only overwrites the scaled VGA region, so
			 * pixels drawn in border areas by overlays persist. */
			static bool had_overlay = false;
			bool has_overlay = (globals.osd_enabled && globals.osd) ||
			                   (globals.overlay_type != OVERLAY_NONE) ||
			                   globals.stats_bar_visible;
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
			// Persistent stats bar (renders on top of everything including OSD)
			render_stats_bar(fb_rotated);

			/* Update overlay tracking (after render_overlay_bar and
			 * render_stats_bar may have changed visibility state) */
			had_overlay = (globals.osd_enabled && globals.osd) ||
			              (globals.overlay_type != OVERLAY_NONE) ||
			              globals.stats_bar_visible;

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
