/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_psram.h"
#include "esp_partition.h"
#include "driver/uart.h"
#include "driver/sdmmc_host.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"

#include "sdmmc_cmd.h"
#include "../../ini.h"

static const char *TAG = "esp_main";

int main(int argc, char *argv[]);
void *esp_psram_get(size_t *size);

struct esp_ini_config {
	const char *filename;
	char ssid[16];
	char pass[32];
};

static void i386_task(void *arg)
{
	struct esp_ini_config *config = arg;
	int core_id = esp_cpu_get_core_id();
	fprintf(stderr, "main runs on core %d\n", core_id);
	char *argv[2];
	argv[0] = "tiny386";
	argv[1] = (char *) config->filename;
	main(2, argv);
	vTaskDelete(NULL);
}

#include "esp_log.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "tdeck_hw.h"
#include "st7789_panel.h"
#include "tdeck_keyboard.h"
#include "tdeck_trackball.h"

#define TEST_LCD_BIT_PER_PIXEL          (16)

/* ST7789 uses standard SPI (not QSPI) */
#define TEST_LCD_SPI_H_RES              TDECK_LCD_WIDTH
#define TEST_LCD_SPI_V_RES              TDECK_LCD_HEIGHT
#define TEST_LCD_SPI_HOST               TDECK_LCD_SPI_HOST

static SemaphoreHandle_t refresh_finish = NULL;

#define LCD_LEDC_CH            1
static esp_err_t bsp_display_brightness_init(void)
{
	// Setup LEDC peripheral for PWM backlight control
	const ledc_channel_config_t LCD_backlight_channel = {
		.gpio_num = TDECK_LCD_BL_PIN,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LCD_LEDC_CH,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = 1,
		.duty = 0,
		.hpoint = 0
	};
	const ledc_timer_config_t LCD_backlight_timer = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = LEDC_TIMER_10_BIT,
		.timer_num = 1,
		.freq_hz = 5000,
		.clk_cfg = LEDC_AUTO_CLK
	};

	ESP_ERROR_CHECK(ledc_timer_config(&LCD_backlight_timer));
	ESP_ERROR_CHECK(ledc_channel_config(&LCD_backlight_channel));

	return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
	if (brightness_percent > 100) {
		brightness_percent = 100;
	}
	if (brightness_percent < 0) {
		brightness_percent = 0;
	}

	ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
	uint32_t duty_cycle = (1023 * brightness_percent) / 100; // LEDC resolution set to 10bits, thus: 100% = 1023
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty_cycle));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));

	return ESP_OK;
}

void *thepc;
void *thepanel;
void pc_vga_step(void *o);
static void vga_task(void *arg)
{
	int core_id = esp_cpu_get_core_id();
	fprintf(stderr, "vga runs on core %d\n", core_id);

	ESP_LOGI(TAG, "Initialize BL");
	gpio_config_t init_gpio_config = {
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = (1ULL << TDECK_LCD_BL_PIN),
	};
	ESP_ERROR_CHECK(gpio_config(&init_gpio_config));
	gpio_set_level(TDECK_LCD_BL_PIN, 1);

	ESP_LOGI(TAG, "Initialize SPI bus for ST7789");
	const spi_bus_config_t buscfg = {
		.mosi_io_num = TDECK_LCD_MOSI_PIN,
		.sclk_io_num = TDECK_LCD_SCLK_PIN,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = TEST_LCD_SPI_H_RES * TEST_LCD_SPI_V_RES * TEST_LCD_BIT_PER_PIXEL / 8,
	};
	ESP_ERROR_CHECK(spi_bus_initialize(TEST_LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

	ESP_LOGI(TAG, "Install panel IO");
	esp_lcd_panel_io_handle_t io_handle = NULL;
	esp_lcd_panel_io_spi_config_t io_config = {
		.cs_gpio_num = TDECK_LCD_CS_PIN,
		.dc_gpio_num = TDECK_LCD_DC_PIN,
		.spi_mode = 0,
		.pclk_hz = 40 * 1000 * 1000, // 40MHz
		.trans_queue_depth = 10,
		.on_color_trans_done = NULL,
	};
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)TEST_LCD_SPI_HOST, &io_config, &io_handle));

	ESP_LOGI(TAG, "Install ST7789 panel driver");
	esp_lcd_panel_handle_t panel_handle = NULL;
	esp_lcd_panel_dev_config_t panel_config = {
		.reset_gpio_num = TDECK_LCD_RST_PIN,
		.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
		.bits_per_pixel = TEST_LCD_BIT_PER_PIXEL,
		.flags = {
			.reset_active_high = 0,
		},
	};
	ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
	esp_lcd_panel_reset(panel_handle);
	vTaskDelay(pdMS_TO_TICKS(10));
	esp_lcd_panel_init(panel_handle);
	esp_lcd_panel_invert_color(panel_handle, false);
	esp_lcd_panel_disp_on_off(panel_handle, true);

	bsp_display_brightness_init();
	bsp_display_brightness_set(50);

	thepanel = panel_handle;
	while (1) {
		if (thepc)
			pc_vga_step(thepc);
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}

	ESP_ERROR_CHECK(esp_lcd_panel_del(panel_handle));
	ESP_ERROR_CHECK(esp_lcd_panel_io_del(io_handle));
	ESP_ERROR_CHECK(spi_bus_free(TEST_LCD_SPI_HOST));
}

static char *psram;
static long psram_off;
static long psram_len;
void *psmalloc(long size)
{
	void *ret = psram + psram_off;

	size = (size + 4095) / 4096 * 4096;
	if (psram_off + size > psram_len) {
		fprintf(stderr, "psram error %ld %ld %ld\n", size, psram_off, psram_len);
		abort();
	}
	psram_off += size;
	return ret;
}

void *fbmalloc(long size)
{
	void *fb = (uint8_t *) heap_caps_calloc(1, size, MALLOC_CAP_DMA);
	if (!fb) {
		fprintf(stderr, "fbmalloc error %ld\n", size);
		abort();
	}
	return fb;
}

static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

static i2s_chan_handle_t                tx_chan;        // I2S tx channel handler
void mixer_callback (void *opaque, uint8_t *stream, int free);

static void i2s_task(void *arg)
{
	int16_t buf[128];
	int core_id = esp_cpu_get_core_id();
	fprintf(stderr, "i2s runs on core %d\n", core_id);

	while (!thepc)
		usleep(200000);

	i2s_channel_enable(tx_chan);
	for (;;) {
		size_t bwritten;
		memset(buf, 0, 128 * 2);
		mixer_callback(thepc, buf, 128 * 2);
		for (int i = 0; i < 128; i++) {
			buf[i] = buf[i] / 16;
		}
		i2s_channel_write(tx_chan, buf, 128 * 2, &bwritten, portMAX_DELAY);
	}
	i2s_channel_disable(tx_chan);
}

void i2s_main()
{
	/* Setp 1: Determine the I2S channel configuration and allocate two channels one by one
	 * The default configuration can be generated by the helper macro,
	 * it only requires the I2S controller id and I2S role
	 * The tx and rx channels here are registered on different I2S controller,
	 * Except ESP32 and ESP32-S2, others allow to register two separate tx & rx channels on a same controller */
	i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
	ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));

	/* Step 2: Setting the configurations of standard mode and initialize each channels one by one
	 * The slot configuration and clock configuration can be generated by the macros
	 * These two helper macros is defined in 'i2s_std.h' which can only be used in STD mode.
	 * They can help to specify the slot and clock configurations for initialization or re-configuring */
	i2s_std_config_t tx_std_cfg = {
		.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(44100),
		.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
		.gpio_cfg = {
			.mclk = I2S_GPIO_UNUSED,    // some codecs may require mclk signal, this example doesn't need it
			.bclk = 42,
			.ws   = 2,
			.dout = 41,
			.din  = -1,
			.invert_flags = {
				.mclk_inv = false,
				.bclk_inv = false,
				.ws_inv   = false,
			},
		},
	};
	ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));
	xTaskCreatePinnedToCore(i2s_task, "i2s_task", 4096, NULL, 0, NULL, 0);
}

void *rawsd;

void wifi_main(const char *, const char *);

static int parse_ini(void* user, const char* section,
		     const char* name, const char* value)
{
	struct esp_ini_config *conf = user;
#define SEC(a) (strcmp(section, a) == 0)
#define NAME(a) (strcmp(name, a) == 0)
	if (SEC("esp")) {
		if (NAME("ssid")) {
			if (strlen(value) < 32)
				strcpy(conf->ssid, value);
		} else if (NAME("pass")) {
			if (strlen(value) < 64)
				strcpy(conf->pass, value);
		}
	}
#undef SEC
#undef NAME
	return 1;
}

void app_main(void)
{
	i2s_main();

#ifdef ESPDEBUG
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity	= UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	uart_param_config(UART_NUM_0, &uart_config);
	if (uart_driver_install(UART_NUM_0, 2 * 1024, 0, 0, NULL, 0) != ESP_OK) {
		assert(false);
	}
#endif

#ifndef USE_RAWSD
	// Options for mounting the filesystem.
	esp_vfs_fat_sdmmc_mount_config_t sdmount_config = {
		.format_if_mount_failed = false,
		.max_files = 3,
		.allocation_unit_size = 16 * 1024
	};
	sdmmc_card_t *card;
	ESP_LOGI(TAG, "Initializing SD card");

	// Use settings defined above to initialize SD card and mount FAT filesystem.
	// Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
	// Please check its source code and implement error recovery when developing
	// production applications.
	ESP_LOGI(TAG, "Using SDMMC peripheral");

	// By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
	// For setting a specific frequency, use host.max_freq_khz (range 400kHz - 40MHz for SDMMC)
	// Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
	sdmmc_host_t host = SDMMC_HOST_DEFAULT();
	host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

	// This initializes the slot without card detect (CD) and write protect (WP) signals.
	// Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
	sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

	// Set bus width to use:
	slot_config.width = 1;

	// On chips where the GPIOs used for SD card can be configured, set them in
	// the slot_config structure:
	slot_config.clk = 12;
	slot_config.cmd = 11;
	slot_config.d0 = 13;

	// Enable internal pullups on enabled pins. The internal pullups
	// are insufficient however, please make sure 10k external pullups are
	// connected on the bus. This is for debug / example purpose only.
	slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

	ESP_LOGI(TAG, "Mounting filesystem");
	esp_err_t ret;
	ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &sdmount_config, &card);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount filesystem. "
				 "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
		} else {
			ESP_LOGE(TAG, "Failed to initialize the card (%s). "
				 "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
		}
	} else {
		ESP_LOGI(TAG, "Filesystem mounted");
	}
#else
	sdmmc_card_t *card = malloc(sizeof(sdmmc_card_t));
	memset(card, 0, sizeof(sdmmc_card_t));
	ESP_LOGI(TAG, "Initializing SD card");
	ESP_LOGI(TAG, "Using SDMMC peripheral");
	sdmmc_host_t host = SDMMC_HOST_DEFAULT();
	host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

	sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
	slot_config.width = 1;
	slot_config.clk = 12;
	slot_config.cmd = 11;
	slot_config.d0 = 13;
	slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

	esp_err_t ret;
	ret = host.init();
	assert(ret == 0);
	ret = sdmmc_host_init_slot(host.slot, &slot_config);
	assert(ret == 0);
	ret = sdmmc_card_init(&host, card);
	assert(ret == 0);
	sdmmc_card_print_info(stderr, card);
#endif
	rawsd = card;

#ifdef ESPDEBUG
	const esp_vfs_fat_mount_config_t mount_config = {
		.max_files = 4,
		.format_if_mount_failed = true,
		.allocation_unit_size = CONFIG_WL_SECTOR_SIZE
	};

	if(esp_vfs_fat_spiflash_mount_rw_wl("/spiflash", "storage",
					    &mount_config, &s_wl_handle) != ESP_OK) {
		assert(false);
	}
#endif

	size_t len;
	esp_psram_init();
	psram = esp_psram_get(&len);
	psram_len = len;

	const static char *files[] = {
		"/sdcard/tiny386.ini",
		"/spiflash/tiny386.ini",
		NULL,
	};
	static struct esp_ini_config config;
	for (int i = 0; files[i]; i++) {
		if (ini_parse(files[i], parse_ini, &config) == 0) {
			config.filename = files[i];
			break;
		}
	}
	if (config.ssid[0]) {
		wifi_main(config.ssid, config.pass);
	}

	if (psram) {
		xTaskCreatePinnedToCore(i386_task, "i386_main", 4096, &config, 3, NULL, 1);
		xTaskCreatePinnedToCore(vga_task, "vga_task", 4096, NULL, 0, NULL, 0);
		// Initialize T-Deck keyboard and trackball
		tdeck_keyboard_init();
		tdeck_trackball_init();
	}
}
