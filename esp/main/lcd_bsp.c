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

static const char *TAG = "lcd_bsp";

// Network stub - ne2000.c needs this symbol
void (*_Atomic esp32_send_packet)(uint8_t *buf, int size) = NULL;

// PPA client handle for rotation
static ppa_client_handle_t ppa_srm_handle = NULL;

// Physical display dimensions (portrait orientation)
#define DISPLAY_WIDTH  480
#define DISPLAY_HEIGHT 800

// Maximum VGA framebuffer - allocate large enough for any reasonable mode
// Actual VGA resolution comes from ini config and may be smaller
#define VGA_MAX_WIDTH  800
#define VGA_MAX_HEIGHT 600

// Actual VGA dimensions - set from globals after PC init
static int vga_width = 720;   // Default, updated at runtime
static int vga_height = 480;

// Triple buffering for rotation output (lock-free)
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

// Software rotation with stride support
// Source: 720x480 VGA in 800-stride buffer -> Dest: 480x720 portrait
static void IRAM_ATTR software_rotate_90cw_stride(uint16_t *src, uint16_t *dst,
                                                   int src_w, int src_h, int src_stride)
{
	// 90 CW: src(x,y) -> dst(src_h-1-y, x)
	// dst_w = src_h, dst_h = src_w
	int dst_w = src_h;
	for (int y = 0; y < src_h; y++) {
		for (int x = 0; x < src_w; x++) {
			int dst_x = src_h - 1 - y;
			int dst_y = x;
			dst[dst_y * dst_w + dst_x] = src[y * src_stride + x];
		}
	}
}

static void IRAM_ATTR rotate_and_blit(uint16_t *fb, uint16_t *fb_rotated)
{
	if (!fb || !fb_rotated) {
		if (blit_debug_count < 5) {
			ESP_LOGE(TAG, "rotate_and_blit: invalid params fb=%p fb_rot=%p",
			         fb, fb_rotated);
			blit_debug_count++;
		}
		return;
	}

	// Calculate scale factors to fit display while respecting PPA's 1/16 precision
	// After 270° rotation: out_w = scale_y * in_h, out_h = scale_x * in_w
	// We need: out_w <= DISPLAY_WIDTH (480), out_h <= DISPLAY_HEIGHT (800)
	float ideal_scale_x = (float)DISPLAY_HEIGHT / vga_width;   // For height after rotation
	float ideal_scale_y = (float)DISPLAY_WIDTH / vga_height;   // For width after rotation

	// Use the smaller scale to maintain aspect ratio
	float ideal_scale = (ideal_scale_x < ideal_scale_y) ? ideal_scale_x : ideal_scale_y;

	// Round DOWN to 1/16 precision (PPA truncates, not rounds)
	int scale_16 = (int)(ideal_scale * 16.0f);
	if (scale_16 < 1) scale_16 = 1;  // Minimum scale
	float scale = scale_16 / 16.0f;

	// Calculate actual output dimensions after PPA's truncation
	int out_w = (int)(scale * vga_height);  // Width after rotation
	int out_h = (int)(scale * vga_width);   // Height after rotation

	// Center on display
	int offset_x = (DISPLAY_WIDTH - out_w) / 2;
	int offset_y = (DISPLAY_HEIGHT - out_h) / 2;
	if (offset_x < 0) offset_x = 0;
	if (offset_y < 0) offset_y = 0;

	// Debug: print buffer info once
	if (blit_debug_count == 0) {
		ESP_LOGI(TAG, "PPA blit: fb=%p (ext=%d) fb_rot=%p (ext=%d)",
		         fb, esp_ptr_external_ram(fb),
		         fb_rotated, esp_ptr_external_ram(fb_rotated));
		ESP_LOGI(TAG, "VGA: %dx%d, ideal_scale=%.4f, actual_scale=%.4f (%d/16)",
		         vga_width, vga_height, ideal_scale, scale, scale_16);
		ESP_LOGI(TAG, "Output: %dx%d at offset (%d,%d) on %dx%d display",
		         out_w, out_h, offset_x, offset_y, DISPLAY_WIDTH, DISPLAY_HEIGHT);
		blit_debug_count++;
	}

	// Try PPA first if not already failed
	if (!use_software_rotation && ppa_srm_handle) {
		ppa_srm_oper_config_t srm_config = {
			.in.buffer = fb,
			.in.pic_w = vga_width,
			.in.pic_h = vga_height,
			.in.block_w = vga_width,
			.in.block_h = vga_height,
			.in.block_offset_x = 0,
			.in.block_offset_y = 0,
			.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
			.out.buffer = fb_rotated,
			.out.buffer_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t),
			.out.pic_w = DISPLAY_WIDTH,
			.out.pic_h = DISPLAY_HEIGHT,
			.out.block_offset_x = offset_x,
			.out.block_offset_y = offset_y,
			.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
			.rotation_angle = PPA_SRM_ROTATION_ANGLE_270,
			.scale_x = scale,
			.scale_y = scale,
			.rgb_swap = 0,
			.byte_swap = 0,
			.mode = PPA_TRANS_MODE_BLOCKING,
		};

		esp_err_t ret = ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config);
		if (ret == ESP_OK) {
			bsp_display_blit(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, fb_rotated);
			return;
		}

		// PPA failed, switch to software rotation permanently
		ESP_LOGW(TAG, "PPA failed (%s), using software rotation", esp_err_to_name(ret));
		use_software_rotation = true;
	}

	// Software rotation fallback (no scaling, just rotation)
	software_rotate_90cw_stride(fb, fb_rotated, vga_width, vga_height, vga_width);
	// After rotation: width=vga_height, height=vga_width
	bsp_display_blit(0, 0, vga_height, vga_width, fb_rotated);
}

// Set VGA dimensions - called after PC config is loaded
void lcd_set_vga_dimensions(int width, int height)
{
	vga_width = width;
	vga_height = height;
	globals.vga_width = width;
	globals.vga_height = height;
	ESP_LOGI(TAG, "VGA dimensions set to %dx%d", width, height);
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

	while (1) {
		// Step the VGA emulator (this calls the redraw callback internally)
		pc_vga_step(globals.pc);

		// Render OSD overlay if enabled
		if (globals.osd_enabled && globals.osd && globals.fb) {
			osd_render(globals.osd, (uint8_t *)globals.fb,
				   vga_width, vga_height,
				   vga_width * sizeof(uint16_t));
		}

		// Rotate and blit using alternating buffers (lock-free double buffering)
		if (globals.fb) {
			int write_idx = atomic_load(&rotate_write_idx);
			rotate_and_blit((uint16_t *)globals.fb, fb_rotated_buffers[write_idx]);
			// Swap to next buffer for next frame
			atomic_store(&rotate_write_idx, (write_idx + 1) % NUM_ROTATE_BUFFERS);
		}

		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}
#endif
