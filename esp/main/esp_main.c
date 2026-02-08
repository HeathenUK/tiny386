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
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "../../ini.h"
#include "../../pc.h"
#include "../../vga.h"
#include "common.h"
#include <errno.h>
#include <sys/stat.h>

#include "bsp/device.h"
#include "bsp/display.h"
#include "mouse_emu.h"

/* From storage.c */
extern bool storage_sd_mounted(void);

//
#include "esp_private/system_internal.h"
uint32_t get_uticks()
{
	return esp_system_get_time();
}

void *psmalloc(long size);
void *fbmalloc(long size);
void *bigmalloc(size_t size)
{
	return psmalloc(size);
}

char *pcram;
long pcram_off;
long pcram_len;
void *pcmalloc(long size)
{
	void *ret = pcram + pcram_off;

	size = (size + 31) / 32 * 32;
	if (pcram_off + size > pcram_len) {
		printf("pcram error %ld %ld %ld\n", size, pcram_off, pcram_len);
		abort();
	}
	pcram_off += size;
	return ret;
}

int load_rom(void *phys_mem, const char *file, uword addr, int backward)
{
	if (file && file[0] == '/') {
		FILE *fp = fopen(file, "rb");
		assert(fp);
		fseek(fp, 0, SEEK_END);
		int len = ftell(fp);
		rewind(fp);
		if (backward)
			fread(phys_mem + addr - len, 1, len, fp);
		else
			fread(phys_mem + addr, 1, len, fp);
		fclose(fp);
		return len;
	}
	const esp_partition_t *part =
		esp_partition_find_first(ESP_PARTITION_TYPE_ANY,
					 ESP_PARTITION_SUBTYPE_ANY,
					 file);
	assert(part);
	int len = part->size;
	if (backward)
		esp_partition_read(part, 0, phys_mem + addr - len, len);
	else
		esp_partition_read(part, 0, phys_mem + addr, len);
	return len;
}

//
EventGroupHandle_t global_event_group;
struct Globals globals;

typedef struct {
	PC *pc;
	u8 *fb1;
	u8 *fb;
} Console;

#define NN 32
Console *console_init(int width, int height)
{
	Console *c = malloc(sizeof(Console));
	c->fb1 = NULL;  /* Not used for BSP backend */
	// For BSP backend, fb is allocated in vga_task after BSP init
	// Use atomic load for cross-core visibility
	c->fb = atomic_load(&globals.fb);
	if (!c->fb) {
		abort();
	}
	return c;
}

void lcd_draw(int x_start, int y_start, int x_end, int y_end, void *src);
static void redraw(void *opaque,
		   int x, int y, int w, int h)
{
	/* For BSP backend, lcd_bsp.c handles full-frame rotation via PPA.
	 * The callback is a no-op - rendering is already done to fb_data. */
	(void)opaque;
	(void)x;
	(void)y;
	(void)w;
	(void)h;
}

static void stub(void *opaque)
{
}

/* Default INI file content */
static const char *default_ini_content =
	"[pc]\n"
	"bios = /sdcard/bios.bin\n"
	"vga_bios = /sdcard/vgabios.bin\n"
	"mem_size = 16M\n"
	"vga_mem_size = 2M\n"
	"fill_cmos = 1\n"
	"; Uncomment and set paths to your disk images:\n"
	"; hda = /sdcard/hdd.img\n"
	"; cda = /sdcard/cdrom.iso\n"
	"; fda = /sdcard/floppy.img\n"
	"\n"
	"[cpu]\n"
	"gen = 5\n"
	"fpu = 1\n"
	"\n"
	"[display]\n"
	"width = 800\n"
	"height = 480\n";

/* Check if a file exists */
static bool file_exists(const char *path)
{
	struct stat st;
	return (stat(path, &st) == 0);
}

/* Create default INI file */
static bool create_default_ini(const char *path)
{
	fprintf(stderr, "Creating default INI file: %s\n", path);
	FILE *f = fopen(path, "w");
	if (!f) {
		fprintf(stderr, "ERROR: Failed to create %s\n", path);
		return false;
	}
	size_t len = strlen(default_ini_content);
	size_t written = fwrite(default_ini_content, 1, len, f);
	fclose(f);
	if (written != len) {
		fprintf(stderr, "ERROR: Failed to write INI content\n");
		return false;
	}
	fprintf(stderr, "Default INI file created successfully\n");
	return true;
}

static int pc_main(const char *file)
{
	PCConfig conf;
	memset(&conf, 0, sizeof(conf));
	conf.linuxstart = "linuxstart.bin";
	conf.bios = "bios.bin";
	conf.vga_bios = "vgabios.bin";
	conf.mem_size = 8 * 1024 * 1024;
	conf.vga_mem_size = 256 * 1024;
	conf.width = LCD_WIDTH;
	conf.height = LCD_HEIGHT;
	conf.cpu_gen = 4;
	conf.fpu = 0;
	conf.brightness = 30;  // Default brightness
	conf.volume = 80;      // Default volume
	conf.mouse_speed = 5;  // Default mouse speed (1-10)
	conf.usb_passthru = 1; // Default USB passthrough enabled

	if (!file) {
		fprintf(stderr, "ERROR: No INI file path provided!\n");
		return -1;
	}
	int err = ini_parse(file, parse_conf_ini, &conf);
	if (err != 0) {
		if (err == -1) {
			fprintf(stderr, "ERROR: Cannot open INI file: %s\n", file);
		} else if (err == -2) {
			fprintf(stderr, "ERROR: Memory allocation failed parsing INI\n");
		} else {
			fprintf(stderr, "ERROR: INI parse error on line %d in %s\n", err, file);
		}
		return err;
	}
	conf.ini_path = file;  // Store ini path for saving settings

	/* Validate mem_size - must be at least 1M */
	if (conf.mem_size < 1024 * 1024) {
		fprintf(stderr, "Warning: mem_size too small (%ld), using default 8M\n",
		        conf.mem_size);
		conf.mem_size = 8 * 1024 * 1024;
	}
#ifdef MAX_MEM_SIZE
	// Cap mem_size to available PSRAM pool
	if (conf.mem_size > MAX_MEM_SIZE) {
		fprintf(stderr, "Warning: mem_size %ldM exceeds max %dM, capping\n",
		        conf.mem_size / (1024 * 1024), MAX_MEM_SIZE / (1024 * 1024));
		conf.mem_size = MAX_MEM_SIZE;
	}
#endif
	// Tell LCD backend the actual VGA dimensions for PPA scaling
	lcd_set_vga_dimensions(conf.width, conf.height);

	Console *console = console_init(conf.width, conf.height);
	PC *pc = pc_new(redraw, stub, console, console->fb, &conf);
	console->pc = pc;
	globals.pc = pc;
	globals.kbd = pc->kbd;
	globals.mouse = pc->mouse;

	/* Apply settings from config */
	vga_frame_skip_max = conf.frame_skip;
	pc_batch_size_setting = conf.batch_size;

	/* Store brightness/volume/mouse_speed in globals for input_bsp and OSD to use */
	globals.brightness = conf.brightness;
	globals.volume = conf.volume;
	globals.mouse_speed = conf.mouse_speed;
	globals.usb_passthru = conf.usb_passthru;
	mouse_emu_set_speed(conf.mouse_speed);
	xEventGroupSetBits(global_event_group, BIT0);

	load_bios_and_reset(pc);

	pc->boot_start_time = get_uticks();
	uint32_t last_delay_time = get_uticks();
	uint32_t last_sync_time = get_uticks();
	uint32_t last_mouse_tick = get_uticks();
	for (; pc->shutdown_state != 8;) {
		// Mouse emulation tick at ~60Hz
		uint32_t now_mouse = get_uticks();
		if (now_mouse - last_mouse_tick >= 16667) {  // ~60Hz
			last_mouse_tick = now_mouse;
			mouse_emu_tick();
		}

		// Check for soft reset request from OSD
		if (globals.reset_pending) {
			globals.reset_pending = false;
			// Clear VGA framebuffer to avoid artifacts from old mode
			if (globals.fb && globals.vga_width > 0 && globals.vga_height > 0) {
				memset(globals.fb, 0, globals.vga_width * globals.vga_height * sizeof(uint16_t));
			}
			// Clear rotated framebuffer (480x800 portrait)
			if (globals.fb_rotated) {
				memset(globals.fb_rotated, 0, 480 * 800 * sizeof(uint16_t));
			}
			pc_reset(pc);
			pc->boot_start_time = get_uticks();
		}

		// Check for emulator restart request (e.g., HDD change)
		if (globals.emu_restart_pending) {
			globals.emu_restart_pending = false;
			// Flush all disk data before restart
			ide_sync(pc->ide, pc->ide2);
			// Change HDD if new path was provided
			if (globals.emu_new_hda_path[0]) {
				fprintf(stderr, "Restart: changing HDD to %s\n", globals.emu_new_hda_path);
				ide_change_hdd(pc->ide, 0, globals.emu_new_hda_path);
				// Update PC's stored hda_path
				pc->hda_path = strdup(globals.emu_new_hda_path);
			}
			// Clear VGA framebuffers
			if (globals.fb && globals.vga_width > 0 && globals.vga_height > 0) {
				memset(globals.fb, 0, globals.vga_width * globals.vga_height * sizeof(uint16_t));
			}
			if (globals.fb_rotated) {
				memset(globals.fb_rotated, 0, 480 * 800 * sizeof(uint16_t));
			}
			// Full reset
			pc_reset(pc);
			pc->boot_start_time = get_uticks();
			fprintf(stderr, "Restart: done\n");
		}
		pc_step(pc);
		// CRITICAL: Use vTaskDelay() not taskYIELD() to let IDLE task run.
		// taskYIELD only yields to equal/higher priority tasks, but IDLE
		// is priority 0. Without IDLE running, the watchdog triggers.
		// Delay every 100ms for ~1 tick to feed the watchdog.
		uint32_t now = get_uticks();
		if (now - last_delay_time >= 100000) {  // 100ms
			last_delay_time = now;
			vTaskDelay(1);
		}
		/* Sync disk files to SD card every 5 seconds to limit data loss
		 * if the device is powered off unexpectedly. */
		if (now - last_sync_time >= 30000000) {  // 30s
			last_sync_time = now;
			ide_sync(pc->ide, pc->ide2);
		}
	}
	/* Final sync on emulator exit */
	ide_sync(pc->ide, pc->ide2);
	return 0;
}

void *esp_psram_get(size_t *size);
void vga_task(void *arg);
void i2s_main();
void input_bsp_init(void);
void wifi_init(void);
void wifi_connect(void);

// Override the weak stub implementation from badge-bsp that uses
// incompatible bootloader_common_get_rtc_retain_mem() API on ESP32-P4
void bsp_device_restart_to_launcher(void) {
    esp_restart();
}
void storage_init(void);

struct esp_ini_config {
	const char *filename;
};

static void i386_task(void *arg)
{
	struct esp_ini_config *config = arg;
	int core_id = esp_cpu_get_core_id();
	fprintf(stderr, "main runs on core %d\n", core_id);

	// Wait for display to be ready (vga_task allocates fb after BSP init)
	xEventGroupWaitBits(global_event_group,
			    BIT1,
			    pdFALSE,
			    pdFALSE,
			    portMAX_DELAY);
	void *fb_check = atomic_load(&globals.fb);
	if (!fb_check) {
		fprintf(stderr, "ERROR: globals.fb is NULL!\n");
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}

	// Initialize USB host subsystem (needs BSP ready for power control)
	extern esp_err_t usb_host_init(void);
	extern bool usb_host_msc_connected(void);
	esp_err_t usb_err = usb_host_init();
	if (usb_err != ESP_OK) {
		fprintf(stderr, "USB host init failed: %d\n", usb_err);
	} else {
		// Wait up to 2 seconds for USB device to enumerate
		// This allows USB drives to be detected before BIOS runs
		fprintf(stderr, "USB: Waiting for device enumeration...\n");
		for (int i = 0; i < 20; i++) {
			vTaskDelay(pdMS_TO_TICKS(100));
			if (usb_host_msc_connected()) {
				fprintf(stderr, "USB: Device ready\n");
				// Debug: read and dump sector 0 (MBR)
				extern int usb_host_msc_read(uint64_t sector, uint8_t *buf, int count);
				uint8_t mbr[512];
				int ret = usb_host_msc_read(0, mbr, 1);
				fprintf(stderr, "USB: Read sector 0 returned %d\n", ret);
				if (ret == 0) {
					fprintf(stderr, "USB: MBR signature: %02X %02X (should be 55 AA)\n",
					        mbr[510], mbr[511]);
					fprintf(stderr, "USB: Partition 1 type: %02X\n", mbr[0x1BE + 4]);
					fprintf(stderr, "USB: First bytes: %02X %02X %02X %02X\n",
					        mbr[0], mbr[1], mbr[2], mbr[3]);
				}
				break;
			}
		}
		if (!usb_host_msc_connected()) {
			fprintf(stderr, "USB: No device found (continuing)\n");
		}
	}

	pc_main(config->filename);
	vTaskDelete(NULL);
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

static int parse_ini(void* user, const char* section,
		     const char* name, const char* value)
{
	(void)user;
	(void)section;
	(void)name;
	(void)value;
	return 1;
}

static void wifi_task(void *arg)
{
	wifi_connect(); /* never returns — monitors and auto-reconnects */
}

void app_main(void)
{
	global_event_group = xEventGroupCreate();
	globals.usb_passthru = -1;  // Sentinel: not yet loaded from INI

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

	// Initialize NVS — required for WiFi credentials stored by launcher
	esp_err_t nvs_ret = nvs_flash_init();
	if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		nvs_flash_erase();
		nvs_ret = nvs_flash_init();
	}

	// Initialize BSP early in app_main (before storage/PSRAM) for fast display startup
	bsp_configuration_t bsp_config = {
		.display = {
			.requested_color_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
			.num_fbs = 1,
		},
	};
	ESP_ERROR_CHECK(bsp_device_initialize(&bsp_config));

	wifi_init();     // ESP-HOSTED SDIO slot 1 — must init before SD (shared SDMMC host)
	storage_init();  // SDMMC native slot 0
	i2s_main();      // Audio task on core 0 — after SDMMC init completes

	/* Check if SD card mounted - required for operation */
	if (!storage_sd_mounted()) {
		fprintf(stderr, "\n");
		fprintf(stderr, "==========================================\n");
		fprintf(stderr, "ERROR: SD card not mounted!\n");
		fprintf(stderr, "Please insert an SD card and restart.\n");
		fprintf(stderr, "Resetting in 5 seconds...\n");
		fprintf(stderr, "==========================================\n");
		vTaskDelay(pdMS_TO_TICKS(5000));
		esp_restart();
	}

	esp_psram_init();
#ifndef PSRAM_ALLOC_LEN
	// use the whole psram
	size_t len;
	psram = esp_psram_get(&len);
	psram_len = len;
#else
	psram_len = PSRAM_ALLOC_LEN;
	psram = heap_caps_calloc(1, psram_len, MALLOC_CAP_SPIRAM);
#endif

	/* INI file handling with fallback creation */
	const char *ini_path = "/sdcard/tiny386.ini";
	static struct esp_ini_config config;

	if (!file_exists(ini_path)) {
		fprintf(stderr, "INI file not found: %s\n", ini_path);
		if (!create_default_ini(ini_path)) {
			fprintf(stderr, "\n");
			fprintf(stderr, "==========================================\n");
			fprintf(stderr, "ERROR: Could not create default INI file!\n");
			fprintf(stderr, "Check SD card is writable.\n");
			fprintf(stderr, "Resetting in 5 seconds...\n");
			fprintf(stderr, "==========================================\n");
			vTaskDelay(pdMS_TO_TICKS(5000));
			esp_restart();
		}
	}

	int ini_err = ini_parse(ini_path, parse_ini, &config);
	if (ini_err != 0) {
		fprintf(stderr, "\n");
		fprintf(stderr, "==========================================\n");
		if (ini_err == -1) {
			fprintf(stderr, "ERROR: Cannot open INI file: %s\n", ini_path);
		} else if (ini_err == -2) {
			fprintf(stderr, "ERROR: Memory allocation failed parsing INI\n");
		} else {
			fprintf(stderr, "ERROR: INI parse error on line %d\n", ini_err);
			fprintf(stderr, "Please check %s for syntax errors.\n", ini_path);
		}
		fprintf(stderr, "Resetting in 5 seconds...\n");
		fprintf(stderr, "==========================================\n");
		vTaskDelay(pdMS_TO_TICKS(5000));
		esp_restart();
	}
	config.filename = ini_path;

	if (psram) {
		xTaskCreatePinnedToCore(i386_task, "i386_main", 8192, &config, 3, NULL, 1);
		xTaskCreatePinnedToCore(vga_task, "vga_task", 12288, NULL, 0, NULL, 0);
		// Start input task after vga_task (which initializes BSP)
		input_bsp_init();
		// WiFi: non-blocking auto-connect using launcher-saved credentials
		xTaskCreatePinnedToCore(wifi_task, "wifi", 8192, NULL, 2, NULL, 0);
	}
}
