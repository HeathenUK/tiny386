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
#include "mouse_emu.h"
#include "../../pc.h"
#include "bsp/display.h"
#include "bsp/audio.h"
#include "esp_log.h"  // For esp_log_timestamp()

static const char *TAG = "input_bsp";

// Overlay types
#define OVERLAY_NONE       0
#define OVERLAY_BRIGHTNESS 1
#define OVERLAY_VOLUME     2
#define OVERLAY_DURATION_MS 1500  // How long to show overlay

// OSD toggle scancode (Meta/Windows key)
#define SC_OSD_TOGGLE BSP_INPUT_SCANCODE_ESCAPED_LEFTMETA  // 0xe05b

// Arrow scancodes (after E0 prefix removed)
#define SC_ARROW_UP    0x48
#define SC_ARROW_DOWN  0x50
#define SC_ARROW_LEFT  0x4B
#define SC_ARROW_RIGHT 0x4D

// Function key scancodes (for META+F1-F6 -> F7-F12 mapping)
#define SC_F1  0x3B
#define SC_F2  0x3C
#define SC_F3  0x3D
#define SC_F4  0x3E
#define SC_F5  0x3F
#define SC_F6  0x40
#define SC_F7  0x41
#define SC_F8  0x42
#define SC_F9  0x43
#define SC_F10 0x44
#define SC_F11 0x57
#define SC_F12 0x58

// META key state for META+arrow shortcuts
static bool meta_held = false;
static bool meta_consumed = false;  // Set if META+combo was used

// Ctrl key scancode
#define SC_LEFT_CTRL  0x1D

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
			globals.stats_collecting = false;  // Stop any lazy stats
			globals.vga_force_redraw = true;  // Force VGA to redraw over OSD
			ESP_LOGI(TAG, "OSD disabled");
		}
	} else {
		ps2_put_keycode(globals.kbd, is_down, code);
	}
}

static void adjust_brightness(int delta)
{
	globals.brightness += delta;
	if (globals.brightness < 0) globals.brightness = 0;
	if (globals.brightness > 100) globals.brightness = 100;
	bsp_display_set_backlight_brightness((uint8_t)globals.brightness);
	// Show overlay bar
	globals.overlay_type = OVERLAY_BRIGHTNESS;
	globals.overlay_value = globals.brightness;
	globals.overlay_hide_time = esp_log_timestamp() + OVERLAY_DURATION_MS;
}

static void adjust_volume(int delta)
{
	globals.volume += delta;
	if (globals.volume < 0) globals.volume = 0;
	if (globals.volume > 100) globals.volume = 100;
	bsp_audio_set_volume((float)globals.volume);
	// Show overlay bar
	globals.overlay_type = OVERLAY_VOLUME;
	globals.overlay_value = globals.volume;
	globals.overlay_hide_time = esp_log_timestamp() + OVERLAY_DURATION_MS;
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

	// Initialize OSD and mouse emulation
	globals.osd = osd_init();
	globals.osd_enabled = false;
	memset(globals.key_pressed, 0, sizeof(globals.key_pressed));
	mouse_emu_init();

	// Wait for PC to be initialized
	xEventGroupWaitBits(global_event_group,
			    BIT0,
			    pdFALSE,
			    pdFALSE,
			    portMAX_DELAY);

	// Attach console to OSD for button callbacks
	osd_attach_console(globals.osd, NULL);

	// Apply initial brightness/volume from config (globals set by esp_main.c)
	bsp_display_set_backlight_brightness((uint8_t)globals.brightness);
	bsp_audio_set_volume((float)globals.volume);

	ESP_LOGI(TAG, "Input task started, brightness=%d%%, volume=%d%%",
	         globals.brightness, globals.volume);

	bsp_input_event_t event;
	while (1) {
		if (xQueueReceive(input_queue, &event, portMAX_DELAY) == pdTRUE) {
			// The BSP provides scancodes in AT keyboard format
			if (event.type == INPUT_EVENT_TYPE_SCANCODE) {
				uint16_t scancode = event.args_scancode.scancode;

				// Check for Meta key - track state for META+arrow shortcuts
				uint16_t base_scancode = scancode & ~BSP_INPUT_SCANCODE_RELEASE_MODIFIER;
				if (base_scancode == SC_OSD_TOGGLE) {
					int is_down = !(scancode & BSP_INPUT_SCANCODE_RELEASE_MODIFIER);
					if (is_down) {
						meta_held = true;
						meta_consumed = false;
					} else {
						// META released - toggle OSD only if no arrow was pressed
						if (!meta_consumed) {
							toggle_osd();
						}
						meta_held = false;
					}
					vTaskDelay(5 / portTICK_PERIOD_MS);
					continue;  // Don't pass meta key to emulator
				}

				// Check if this is an extended scancode (0xE0xx)
				if (scancode >= 0xE000) {
					// Extended scancode (E0 prefix)
					uint8_t code = scancode & 0xFF;
					int is_down = !(code & BSP_INPUT_SCANCODE_RELEASE_MODIFIER);
					code &= ~BSP_INPUT_SCANCODE_RELEASE_MODIFIER;

					// Block arrow keys when mouse mode is active
					// (mouse_emu handles them via navigation events)
					if (mouse_emu_is_active() && !globals.osd_enabled) {
						if (code == SC_ARROW_UP || code == SC_ARROW_DOWN ||
						    code == SC_ARROW_LEFT || code == SC_ARROW_RIGHT) {
							vTaskDelay(5 / portTICK_PERIOD_MS);
							continue;
						}
					}

					// Check for META+arrow shortcuts (only on key down)
					if (meta_held && is_down) {
						bool handled = false;
						switch (code) {
						case SC_ARROW_LEFT:
							adjust_brightness(-5);
							handled = true;
							break;
						case SC_ARROW_RIGHT:
							adjust_brightness(5);
							handled = true;
							break;
						case SC_ARROW_UP:
							adjust_volume(5);
							handled = true;
							break;
						case SC_ARROW_DOWN:
							adjust_volume(-5);
							handled = true;
							break;
						}
						if (handled) {
							meta_consumed = true;
							vTaskDelay(5 / portTICK_PERIOD_MS);
							continue;
						}
					}

					// Send E0 prefix to emulator if not in OSD mode
					if (!globals.osd_enabled) {
						ps2_put_keycode(globals.kbd, 1, 0xE0);
						vTaskDelay(1 / portTICK_PERIOD_MS);
					}

					handle_scancode(code, is_down);
				} else {
					// Regular scancode
					uint8_t code = scancode & 0xFF;
					int is_down = !(code & BSP_INPUT_SCANCODE_RELEASE_MODIFIER);
					code &= ~BSP_INPUT_SCANCODE_RELEASE_MODIFIER;

					// Block space key when mouse mode is active
					// (mouse_emu handles L/M/R sections via navigation events)
					if (mouse_emu_is_active() && !globals.osd_enabled) {
						if (code == BSP_INPUT_SCANCODE_SPACE) {
							vTaskDelay(5 / portTICK_PERIOD_MS);
							continue;
						}
					}

					// META+F1-F6 -> F7-F12 (Tanmatsu only has F1-F6 keys)
					if (meta_held && code >= SC_F1 && code <= SC_F6) {
						code = (code - SC_F1) + SC_F7;  // Map F1-F6 to F7-F12
						meta_consumed = true;
					}

					// META+Ctrl -> Toggle mouse emulation mode
					if (meta_held && is_down && code == SC_LEFT_CTRL) {
						mouse_emu_toggle();
						meta_consumed = true;
						vTaskDelay(5 / portTICK_PERIOD_MS);
						continue;  // Don't pass Ctrl to emulator
					}

					handle_scancode(code, is_down);
				}

				vTaskDelay(5 / portTICK_PERIOD_MS);
			}
			else if (event.type == INPUT_EVENT_TYPE_NAVIGATION) {
				// Handle navigation events for mouse emulation
				// (gives us separate space L/M/R sections)
				uint8_t nav_key = event.args_navigation.key;
				bool pressed = event.args_navigation.state;

				// Don't process mouse input when OSD is open
				if (!globals.osd_enabled) {
					// Let mouse_emu handle navigation keys
					// Returns true if consumed (mouse mode active or F6 toggle)
					if (mouse_emu_handle_nav_key(nav_key, pressed)) {
						vTaskDelay(5 / portTICK_PERIOD_MS);
						continue;
					}
				}
				// If not consumed by mouse_emu, navigation events are
				// redundant with scancodes, so we can ignore them
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
