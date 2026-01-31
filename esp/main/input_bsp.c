#ifdef USE_BADGE_BSP
/*
 * Keyboard input backend using badge-bsp for Tanmatsu device
 * The BSP provides PC AT keyboard scancodes directly - no mapping needed!
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "bsp/input.h"

#include "common.h"
#include "../../i8042.h"

static const char *TAG = "input_bsp";

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

	// Wait for PC to be initialized
	xEventGroupWaitBits(global_event_group,
			    BIT0,
			    pdFALSE,
			    pdFALSE,
			    portMAX_DELAY);

	ESP_LOGI(TAG, "Input task started");

	bsp_input_event_t event;
	while (1) {
		if (xQueueReceive(input_queue, &event, portMAX_DELAY) == pdTRUE) {
			// The BSP provides scancodes in AT keyboard format
			if (event.type == INPUT_EVENT_TYPE_SCANCODE) {
				uint16_t scancode = event.args_scancode.scancode;

				// Check if this is a release (high bit set in low byte)
				// or an escaped scancode (0xE0xx)
				if (scancode >= 0xE000) {
					// Extended scancode (E0 prefix)
					// Send E0 prefix first
					ps2_put_keycode(globals.kbd, 1, 0xE0);
					vTaskDelay(1 / portTICK_PERIOD_MS);

					// Extract the actual scancode (low byte)
					uint8_t code = scancode & 0xFF;
					// The scancode includes release bit if applicable
					int is_down = !(code & BSP_INPUT_SCANCODE_RELEASE_MODIFIER);
					code &= ~BSP_INPUT_SCANCODE_RELEASE_MODIFIER;

					ps2_put_keycode(globals.kbd, is_down, code);
				} else {
					// Regular scancode
					uint8_t code = scancode & 0xFF;
					int is_down = !(code & BSP_INPUT_SCANCODE_RELEASE_MODIFIER);
					code &= ~BSP_INPUT_SCANCODE_RELEASE_MODIFIER;

					ps2_put_keycode(globals.kbd, is_down, code);
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
#endif
