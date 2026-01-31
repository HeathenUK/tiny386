#ifdef USE_BADGE_BSP
/*
 * Keyboard input backend using badge-bsp for Tanmatsu device
 * The BSP provides PC AT keyboard scancodes directly.
 * Handles OSD toggle (Meta/Windows key) and routes input appropriately.
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "bsp/input.h"

#include "common.h"
#include "../../i8042.h"
#include "tanmatsu_osd.h"
#include "../../pc.h"

static const char *TAG = "input_bsp";

// OSD toggle scancode (Meta/Windows key)
#define SC_OSD_TOGGLE BSP_INPUT_SCANCODE_ESCAPED_LEFTMETA  // 0xe05b

static void handle_scancode(uint8_t code, int is_down)
{
	// Track key state
	if (code < KEYCODE_MAX) {
		globals.key_pressed[code] = is_down;
	}

	// Route input based on OSD state
	if (globals.osd_enabled) {
		if (osd_handle_key(globals.osd, code, is_down)) {
			// OSD requested close
			globals.osd_enabled = false;
			ESP_LOGI(TAG, "OSD disabled");
		}
	} else {
		ps2_put_keycode(globals.kbd, is_down, code);
	}
}

static void toggle_osd(void)
{
	globals.osd_enabled = !globals.osd_enabled;
	if (globals.osd_enabled) {
		// Attach PC components to OSD and refresh buffers
		PC *pc = (PC *)globals.pc;
		osd_attach_emulink(globals.osd, pc->emulink);
		osd_attach_ide(globals.osd, pc->ide, pc->ide2);
		osd_attach_pc(globals.osd, pc);
		osd_refresh(globals.osd);
	}
	ESP_LOGI(TAG, "OSD %s", globals.osd_enabled ? "enabled" : "disabled");
}

static void input_task(void *arg)
{
	int core_id = esp_cpu_get_core_id();
	fprintf(stderr, "input runs on core %d\n", core_id);

	// Wait for BSP initialization (done by vga_task)
	xEventGroupWaitBits(global_event_group,
			    BIT1,
			    pdFALSE,
			    pdFALSE,
			    portMAX_DELAY);

	// Get input event queue from BSP
	QueueHandle_t input_queue = NULL;
	if (bsp_input_get_queue(&input_queue) != ESP_OK || !input_queue) {
		ESP_LOGE(TAG, "Failed to get input queue");
		vTaskDelete(NULL);
		return;
	}

	globals.input_queue = input_queue;

	// Initialize OSD
	globals.osd = osd_init();
	globals.osd_enabled = false;
	memset(globals.key_pressed, 0, sizeof(globals.key_pressed));

	// Wait for PC to be initialized
	xEventGroupWaitBits(global_event_group,
			    BIT0,
			    pdFALSE,
			    pdFALSE,
			    portMAX_DELAY);

	// Attach console to OSD for button callbacks
	osd_attach_console(globals.osd, NULL);

	ESP_LOGI(TAG, "Input task started, Meta key to toggle OSD");

	bsp_input_event_t event;
	while (1) {
		if (xQueueReceive(input_queue, &event, portMAX_DELAY) == pdTRUE) {
			// The BSP provides scancodes in AT keyboard format
			if (event.type == INPUT_EVENT_TYPE_SCANCODE) {
				uint16_t scancode = event.args_scancode.scancode;

				// Check for OSD toggle (Meta key) - check raw scancode before processing
				uint16_t base_scancode = scancode & ~BSP_INPUT_SCANCODE_RELEASE_MODIFIER;
				if (base_scancode == SC_OSD_TOGGLE) {
					int is_down = !(scancode & BSP_INPUT_SCANCODE_RELEASE_MODIFIER);
					if (is_down) {
						toggle_osd();
					}
					vTaskDelay(5 / portTICK_PERIOD_MS);
					continue;  // Don't pass meta key to emulator
				}

				// Check if this is an extended scancode (0xE0xx)
				if (scancode >= 0xE000) {
					// Extended scancode (E0 prefix)
					// Send E0 prefix to emulator if not in OSD mode
					if (!globals.osd_enabled) {
						ps2_put_keycode(globals.kbd, 1, 0xE0);
						vTaskDelay(1 / portTICK_PERIOD_MS);
					}

					// Extract the actual scancode (low byte)
					uint8_t code = scancode & 0xFF;
					int is_down = !(code & BSP_INPUT_SCANCODE_RELEASE_MODIFIER);
					code &= ~BSP_INPUT_SCANCODE_RELEASE_MODIFIER;

					handle_scancode(code, is_down);
				} else {
					// Regular scancode
					uint8_t code = scancode & 0xFF;
					int is_down = !(code & BSP_INPUT_SCANCODE_RELEASE_MODIFIER);
					code &= ~BSP_INPUT_SCANCODE_RELEASE_MODIFIER;

					handle_scancode(code, is_down);
				}

				vTaskDelay(5 / portTICK_PERIOD_MS);
			}
		}
	}
}

void input_bsp_init(void)
{
	// Create input processing task
	// The task itself will wait for BSP initialization (BIT1)
	xTaskCreatePinnedToCore(input_task, "input_task", 4096, NULL, 1, NULL, 0);
}

// Called by OSD to send keypresses to emulator
void console_send_kbd(void *opaque, int keypress, int keycode)
{
	ps2_put_keycode(globals.kbd, keypress, keycode);
}
#endif
