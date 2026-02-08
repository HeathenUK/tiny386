/*
 * Drive Activity LED indicator for Tanmatsu
 * LED 4: HDD activity (red)
 * LED 5: Floppy activity (green)
 */

#include "led_activity.h"
#include "bsp/led.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "led_activity";

// LED indices on Tanmatsu
// LEDs 0-3 are reserved for system (Power, Radio, Messages, Power Button)
#define LED_HDD_INDEX    4
#define LED_FLOPPY_INDEX 5

// LED timeout in microseconds (100ms)
#define LED_TIMEOUT_US   100000

// Activity timestamps
static int64_t hdd_last_activity = 0;
static int64_t floppy_last_activity = 0;

// LED state tracking
static bool hdd_led_on = false;
static bool floppy_led_on = false;
static bool led_initialized = false;

void led_activity_init(void)
{
	// Take manual control of LEDs 4-5
	// This allows us to control them independently of the system LEDs
	esp_err_t err = bsp_led_set_mode(false);
	if (err != ESP_OK) {
		ESP_LOGW(TAG, "Failed to set LED manual mode: %s", esp_err_to_name(err));
	}

	// Ensure activity LEDs are off initially
	bsp_led_set_pixel_rgb(LED_HDD_INDEX, 0, 0, 0);
	bsp_led_set_pixel_rgb(LED_FLOPPY_INDEX, 0, 0, 0);
	bsp_led_send();

	led_initialized = true;
	ESP_LOGI(TAG, "Drive activity LEDs initialized (HDD=LED%d, Floppy=LED%d)",
	         LED_HDD_INDEX, LED_FLOPPY_INDEX);
}

void led_activity_hdd(void)
{
	if (!led_initialized) return;

	hdd_last_activity = esp_timer_get_time();
	if (!hdd_led_on) {
		bsp_led_set_pixel_rgb(LED_HDD_INDEX, 255, 0, 0);  // Red for HDD
		bsp_led_send();
		hdd_led_on = true;
	}
}

void led_activity_floppy(void)
{
	if (!led_initialized) return;

	floppy_last_activity = esp_timer_get_time();
	if (!floppy_led_on) {
		bsp_led_set_pixel_rgb(LED_FLOPPY_INDEX, 0, 255, 0);  // Green for floppy
		bsp_led_send();
		floppy_led_on = true;
	}
}

void led_activity_off(void)
{
	if (!led_initialized) return;

	bool need_update = false;
	if (hdd_led_on) {
		bsp_led_set_pixel_rgb(LED_HDD_INDEX, 0, 0, 0);
		hdd_led_on = false;
		need_update = true;
	}
	if (floppy_led_on) {
		bsp_led_set_pixel_rgb(LED_FLOPPY_INDEX, 0, 0, 0);
		floppy_led_on = false;
		need_update = true;
	}
	if (need_update)
		bsp_led_send();
}

void led_activity_tick(void)
{
	if (!led_initialized) return;

	int64_t now = esp_timer_get_time();
	bool need_update = false;

	// Turn off HDD LED if timeout expired
	if (hdd_led_on && (now - hdd_last_activity > LED_TIMEOUT_US)) {
		bsp_led_set_pixel_rgb(LED_HDD_INDEX, 0, 0, 0);
		hdd_led_on = false;
		need_update = true;
	}

	// Turn off floppy LED if timeout expired
	if (floppy_led_on && (now - floppy_last_activity > LED_TIMEOUT_US)) {
		bsp_led_set_pixel_rgb(LED_FLOPPY_INDEX, 0, 0, 0);
		floppy_led_on = false;
		need_update = true;
	}

	if (need_update) {
		bsp_led_send();
	}
}
