#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "common.h"
#include "bsp/audio.h"

static i2s_chan_handle_t                tx_chan;        // I2S tx channel handler
void mixer_callback (void *opaque, uint8_t *stream, int free);

static int last_volume = -1;

static void i2s_bsp_task(void *arg)
{
	int core_id = esp_cpu_get_core_id();
	fprintf(stderr, "i2s runs on core %d\n", core_id);

	// Wait for BSP initialization (done by vga_task)
	xEventGroupWaitBits(global_event_group,
			    BIT1,
			    pdFALSE,
			    pdFALSE,
			    portMAX_DELAY);

	// Get I2S handle from BSP (audio already configured by bsp_device_initialize)
	if (bsp_audio_get_i2s_handle(&tx_chan) != ESP_OK || !tx_chan) {
		fprintf(stderr, "Failed to get I2S handle\n");
		vTaskDelete(NULL);
		return;
	}

	// Enable amplifier (rate already set by BSP init)
	bsp_audio_set_amplifier(true);

	// Wait for PC to be initialized
	xEventGroupWaitBits(global_event_group,
			    BIT0,
			    pdFALSE,
			    pdFALSE,
			    portMAX_DELAY);

	// Apply initial volume from globals
	int vol = globals.volume;
	if (vol < 0) vol = 0;
	if (vol > 100) vol = 100;
	bsp_audio_set_volume((float)vol);
	last_volume = vol;

	int16_t buf[128];
	i2s_channel_enable(tx_chan);
	for (;;) {
		size_t bwritten;
		memset(buf, 0, 128 * 2);
		mixer_callback(globals.pc, (uint8_t *) buf, 128 * 2);
		for (int i = 0; i < 128; i++) {
			buf[i] = buf[i] / 2;
		}
		i2s_channel_write(tx_chan, buf, 128 * 2, &bwritten, portMAX_DELAY);

		/* Check if volume changed (from OSD or META+arrow) */
		if (globals.volume != last_volume) {
			vol = globals.volume;
			if (vol < 0) vol = 0;
			if (vol > 100) vol = 100;
			bsp_audio_set_volume((float)vol);
			last_volume = vol;
		}
	}
	i2s_channel_disable(tx_chan);
}

void i2s_main()
{
	// Audio is initialized by bsp_device_initialize() in vga_task
	// Just create the task here - it will wait for BSP init
	xTaskCreatePinnedToCore(i2s_bsp_task, "i2s_task", 4096, NULL, 0, NULL, 0);
}

// Silence and disable audio before restart to avoid crackling
void i2s_shutdown(void)
{
	if (!tx_chan) return;

	// Disable amplifier first to cut output
	bsp_audio_set_amplifier(false);

	// Write silence to flush any pending audio data
	int16_t silence[256] = {0};
	size_t bwritten;
	for (int i = 0; i < 4; i++) {
		i2s_channel_write(tx_chan, silence, sizeof(silence), &bwritten, pdMS_TO_TICKS(50));
	}

	// Small delay to let silence play out
	vTaskDelay(pdMS_TO_TICKS(50));

	// Disable the I2S channel
	i2s_channel_disable(tx_chan);
}
