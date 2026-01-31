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
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include <stdatomic.h>
#include <string.h>

#include "bsp/device.h"
#include "bsp/display.h"
#include "esp_psram.h"

#include "common.h"

static const char *TAG = "lcd_bsp";

// Network stub - ne2000.c needs this symbol
void (*_Atomic esp32_send_packet)(uint8_t *buf, int size) = NULL;

// PPA client handle for rotation
static ppa_client_handle_t ppa_srm_handle = NULL;

// Framebuffer dimensions
// VGA emulator outputs 720x480 with stride matching width
#define VGA_WIDTH  720   // Actual VGA output width
#define VGA_HEIGHT 480   // VGA output height
#define FB_STRIDE  720   // Buffer stride = width (VGA sets stride to width)
#define DISPLAY_WIDTH  480  // Portrait width (physical display)
#define DISPLAY_HEIGHT 800  // Portrait height

// Scaling: 720x480 rotated to 480x720, scaled to 480x800
// Scale factor: 800/720 = 1.111 (vertical stretch after rotation)
#define SCALE_ENABLE 1

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

	// Debug: print buffer info once
	if (blit_debug_count == 0) {
		float scale_x = 800.0f / 720.0f;
		ESP_LOGI(TAG, "PPA blit: fb=%p (ext=%d) fb_rot=%p (ext=%d) scale_x=%.6f",
		         fb, esp_ptr_external_ram(fb),
		         fb_rotated, esp_ptr_external_ram(fb_rotated),
		         scale_x);
		blit_debug_count++;
	}

	// Try PPA first if not already failed
	if (!use_software_rotation && ppa_srm_handle) {
		ppa_srm_oper_config_t srm_config = {
			.in.buffer = fb,
			.in.pic_w = VGA_WIDTH,
			.in.pic_h = VGA_HEIGHT,
			.in.block_w = VGA_WIDTH,
			.in.block_h = VGA_HEIGHT,
			.in.block_offset_x = 0,
			.in.block_offset_y = 0,
			.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
			.out.buffer = fb_rotated,
			.out.buffer_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t),
			.out.pic_w = DISPLAY_WIDTH,
			.out.pic_h = DISPLAY_HEIGHT,
			.out.block_offset_x = 0,
			.out.block_offset_y = 0,
			.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
			.rotation_angle = PPA_SRM_ROTATION_ANGLE_270,
			// Scale 720x480 -> 800x480 before rotation, then rotate to 480x800
			// For 270° rotation: out_w = scale_y * in_h, out_h = scale_x * in_w
			// We want 480x800 output from 720x480 input:
			//   out_w = scale_y * 480 = 480, so scale_y = 1.0
			//   out_h = scale_x * 720 = 800, so scale_x = 800/720 = 1.111
			.scale_x = 800.0f / 720.0f,
			.scale_y = 1.0f,
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

	// Software rotation fallback
	// VGA outputs 720x480 with stride 800, rotate to 480x720 portrait
	// Display is 480x800, so image will be centered with black bars
	software_rotate_90cw_stride(fb, fb_rotated, VGA_WIDTH, VGA_HEIGHT, FB_STRIDE);
	bsp_display_blit(0, 0, DISPLAY_WIDTH, VGA_WIDTH, fb_rotated);
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
	// Main VGA framebuffer (720x480 RGB565)
	size_t fb_size = VGA_WIDTH * VGA_HEIGHT * sizeof(uint16_t);
	ESP_LOGI(TAG, "Allocating VGA framebuffer: %d bytes", (int)fb_size);

	uint16_t *fb = heap_caps_aligned_alloc(64, fb_size,
	                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
	if (!fb) {
		ESP_LOGE(TAG, "Failed to allocate VGA framebuffer");
		vTaskDelete(NULL);
		return;
	}
	memset(fb, 0, fb_size);
	globals.fb = fb;

	ESP_LOGI(TAG, "VGA framebuffer at %p (ext=%d)", fb, esp_ptr_external_ram(fb));

	// Rotated framebuffer (480x800 RGB565)
	size_t fb_rot_size = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
	ESP_LOGI(TAG, "Allocating rotated framebuffer: %d bytes", (int)fb_rot_size);

	uint16_t *fb_rotated = heap_caps_aligned_alloc(64, fb_rot_size,
	                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
	if (!fb_rotated) {
		ESP_LOGE(TAG, "Failed to allocate rotated framebuffer");
		vTaskDelete(NULL);
		return;
	}
	memset(fb_rotated, 0, fb_rot_size);
	globals.fb_rotated = fb_rotated;

	ESP_LOGI(TAG, "Rotated framebuffer at %p (ext=%d)", fb_rotated, esp_ptr_external_ram(fb_rotated));

	// 2. Initialize BSP (display, input, audio, etc.)
	bsp_configuration_t bsp_config = {
		.display = {
			.requested_color_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
			.num_fbs = 2,
		},
	};
	ESP_ERROR_CHECK(bsp_device_initialize(&bsp_config));

	// Get the display panel handle
	esp_lcd_panel_handle_t panel = NULL;
	ESP_ERROR_CHECK(bsp_display_get_panel(&panel));

	// Set backlight brightness
	ESP_ERROR_CHECK(bsp_display_set_backlight_brightness(30));

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

		// Rotate and blit the full framebuffer to display
		if (globals.fb) {
			rotate_and_blit((uint16_t *)globals.fb, fb_rotated);
		}

		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}
#endif
