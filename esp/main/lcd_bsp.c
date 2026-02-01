#ifdef USE_LCD_BSP
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
#include "esp_cache.h"

static const char *TAG = "lcd_bsp";

// Network stub - ne2000.c needs this symbol
void (*_Atomic esp32_send_packet)(uint8_t *buf, int size) = NULL;

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

// Double buffering for rotation output
#define NUM_ROTATE_BUFFERS 2
static uint16_t *fb_rotated_buffers[NUM_ROTATE_BUFFERS];
static _Atomic int rotate_write_idx = 0;  // Buffer being written by PPA

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

	/* Check if VGA mode changed - clear ALL rotated buffers */
	if (globals.vga_mode_changed) {
		globals.vga_mode_changed = false;
		size_t fb_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
		for (int i = 0; i < NUM_ROTATE_BUFFERS; i++) {
			if (fb_rotated_buffers[i]) {
				memset(fb_rotated_buffers[i], 0, fb_size);
				/* Flush cache to ensure DMA sees zeros */
				esp_cache_msync(fb_rotated_buffers[i], fb_size,
				                ESP_CACHE_MSYNC_FLAG_DIR_C2M);
			}
		}
		ESP_LOGI(TAG, "Mode change: cleared all rotated buffers");
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
	int core_id = esp_cpu_get_core_id();
	fprintf(stderr, "vga runs on core %d\n", core_id);

	ESP_LOGI(TAG, "Init display via badge-bsp with PPA rotation");

	// Debug: Check heap SPIRAM info
	multi_heap_info_t heap_info;
	heap_caps_get_info(&heap_info, MALLOC_CAP_SPIRAM);
	ESP_LOGI(TAG, "SPIRAM heap: total=%d free=%d largest=%d",
	         (int)heap_info.total_allocated_bytes + (int)heap_info.total_free_bytes,
	         (int)heap_info.total_free_bytes,
	         (int)heap_info.largest_free_block);

	// 1. Allocate framebuffers BEFORE BSP init (while PSRAM region tracking is correct)
	// Main VGA framebuffer - allocate max size to handle any VGA mode
	size_t fb_size = VGA_MAX_WIDTH * VGA_MAX_HEIGHT * sizeof(uint16_t);
	ESP_LOGI(TAG, "Allocating VGA framebuffer: %d bytes (max %dx%d)", (int)fb_size, VGA_MAX_WIDTH, VGA_MAX_HEIGHT);

	uint16_t *fb = heap_caps_aligned_alloc(64, fb_size,
	                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
	if (!fb) {
		ESP_LOGE(TAG, "Failed to allocate VGA framebuffer");
		vTaskDelete(NULL);
		return;
	}
	memset(fb, 0, fb_size);
	atomic_store(&globals.fb, fb);  // Atomic store for cross-core visibility

	ESP_LOGI(TAG, "VGA framebuffer at %p (ext=%d)", fb, esp_ptr_external_ram(fb));

	// Rotated framebuffers (480x800 RGB565) - double buffer for lock-free updates
	size_t fb_rot_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
	ESP_LOGI(TAG, "Allocating %d rotated framebuffers: %d bytes each", NUM_ROTATE_BUFFERS, (int)fb_rot_size);

	for (int i = 0; i < NUM_ROTATE_BUFFERS; i++) {
		fb_rotated_buffers[i] = heap_caps_aligned_alloc(64, fb_rot_size,
		                                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
		if (!fb_rotated_buffers[i]) {
			ESP_LOGE(TAG, "Failed to allocate rotated framebuffer %d", i);
			vTaskDelete(NULL);
			return;
		}
		memset(fb_rotated_buffers[i], 0, fb_rot_size);
		ESP_LOGI(TAG, "Rotated framebuffer[%d] at %p (ext=%d)", i,
		         fb_rotated_buffers[i], esp_ptr_external_ram(fb_rotated_buffers[i]));
	}
	globals.fb_rotated = fb_rotated_buffers[0];

	// 2. Reset display before BSP init to ensure clean state
	// (Launcher may have left MIPI DSI in unknown state)
	#define LCD_RESET_PIN 14
	gpio_config_t reset_conf = {
		.pin_bit_mask = (1ULL << LCD_RESET_PIN),
		.mode = GPIO_MODE_OUTPUT,
	};
	gpio_config(&reset_conf);
	gpio_set_level(LCD_RESET_PIN, 0);  // Assert reset
	vTaskDelay(pdMS_TO_TICKS(10));
	gpio_set_level(LCD_RESET_PIN, 1);  // Release reset
	vTaskDelay(pdMS_TO_TICKS(50));     // Wait for display to stabilize
	ESP_LOGI(TAG, "Display reset complete");

	// 3. Initialize BSP (display, input, audio, etc.)
	bsp_configuration_t bsp_config = {
		.display = {
			.requested_color_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
			.num_fbs = 1,  // Single FB to reduce init time
		},
	};
	ESP_LOGI(TAG, "Calling bsp_device_initialize...");
	uint32_t t0 = esp_log_timestamp();
	ESP_ERROR_CHECK(bsp_device_initialize(&bsp_config));
	ESP_LOGI(TAG, "bsp_device_initialize took %lu ms", esp_log_timestamp() - t0);

	// Get the display panel handle
	esp_lcd_panel_handle_t panel = NULL;
	t0 = esp_log_timestamp();
	ESP_ERROR_CHECK(bsp_display_get_panel(&panel));
	ESP_LOGI(TAG, "bsp_display_get_panel took %lu ms", esp_log_timestamp() - t0);

	// Set backlight brightness
	t0 = esp_log_timestamp();
	ESP_ERROR_CHECK(bsp_display_set_backlight_brightness(30));
	ESP_LOGI(TAG, "bsp_display_set_backlight_brightness took %lu ms", esp_log_timestamp() - t0);

	// Get display parameters for debugging
	size_t h_res, v_res;
	lcd_color_rgb_pixel_format_t color_fmt;
	lcd_rgb_data_endian_t data_endian;
	if (bsp_display_get_parameters(&h_res, &v_res, &color_fmt, &data_endian) == ESP_OK) {
		ESP_LOGI(TAG, "Display: %dx%d, format=%d", (int)h_res, (int)v_res, color_fmt);
	}

	// 3. Initialize PPA for rotation
	ESP_ERROR_CHECK(ppa_init());

	globals.panel = panel;
	xEventGroupSetBits(global_event_group, BIT1);  // Signal display ready

	// Wait for PC emulator to be ready
	xEventGroupWaitBits(global_event_group,
			    BIT0,
			    pdFALSE,
			    pdFALSE,
			    portMAX_DELAY);

	ESP_LOGI(TAG, "Starting display loop");

	// Profiling stats
	uint32_t prof_vga_total = 0, prof_rotate_total = 0;
	uint32_t prof_frame_count = 0;
	uint32_t prof_last_report = esp_log_timestamp();

	while (1) {
		uint32_t t0, t1, t2, t3;

		// Step the VGA emulator (this calls the redraw callback internally)
		t0 = esp_log_timestamp();
		pc_vga_step(globals.pc);
		t1 = esp_log_timestamp();

		// Rotate, render OSD, and blit using alternating buffers
		if (globals.fb) {
			int write_idx = atomic_load(&rotate_write_idx);
			uint16_t *fb_rot = fb_rotated_buffers[write_idx];
			t2 = esp_log_timestamp();

			// Rotate VGA content to display buffer
			rotate_vga_to_display((uint16_t *)globals.fb, fb_rot);

			// Render OSD overlay to rotated buffer (consistent size/position)
			if (globals.osd_enabled && globals.osd) {
				osd_render(globals.osd, (uint8_t *)fb_rot,
					   DISPLAY_WIDTH, DISPLAY_HEIGHT,
					   DISPLAY_WIDTH * sizeof(uint16_t));
			}

			// Render brightness/volume overlay (if not showing full OSD)
			if (!globals.osd_enabled) {
				render_overlay_bar(fb_rot);
			}

			// Blit to display
			bsp_display_blit(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, fb_rot);

			t3 = esp_log_timestamp();
			// Swap to next buffer for next frame
			atomic_store(&rotate_write_idx, (write_idx + 1) % NUM_ROTATE_BUFFERS);

			prof_vga_total += (t1 - t0);
			prof_rotate_total += (t3 - t2);
			prof_frame_count++;
		}

		// Report profiling stats every 5 seconds
		uint32_t now = esp_log_timestamp();
		if (now - prof_last_report >= 5000 && prof_frame_count > 0) {
			ESP_LOGI(TAG, "PERF: %lu frames, vga=%lums/f, rotate=%lums/f, fps=%.1f",
			         prof_frame_count,
			         prof_vga_total / prof_frame_count,
			         prof_rotate_total / prof_frame_count,
			         (float)prof_frame_count * 1000.0f / (now - prof_last_report));
			prof_vga_total = 0;
			prof_rotate_total = 0;
			prof_frame_count = 0;
			prof_last_report = now;
		}

		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}
#endif
