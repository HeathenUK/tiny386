#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"
#include "sdmmc_cmd.h"
#include "esp_attr.h"
#include "sd_pwr_ctrl.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "common.h"

static const char *TAG = "storage";
void *rawsd;
static sd_pwr_ctrl_handle_t sd_pwr_handle = NULL;
static bool sd_mounted = false;

bool storage_sd_mounted(void)
{
	return sd_mounted;
}

#ifdef SD_CLK
/* Power cycle SD card — mirrors launcher's reset_sd_card() exactly */
static void reset_sd_card(void)
{
	if (!sd_pwr_handle)
		return;

	ESP_LOGI(TAG, "Power cycling SD card...");

	gpio_config_t cfg = {
		.pin_bit_mask = BIT64(SD_D0) | BIT64(SD_D1) | BIT64(SD_D2) |
				BIT64(SD_D3) | BIT64(SD_CLK) | BIT64(SD_CMD),
		.mode = GPIO_MODE_OUTPUT_OD,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&cfg);

	gpio_set_level(SD_D0, 0);
	gpio_set_level(SD_D1, 0);
	gpio_set_level(SD_D2, 0);
	gpio_set_level(SD_D3, 0);
	gpio_set_level(SD_CLK, 0);
	gpio_set_level(SD_CMD, 0);

	sd_pwr_ctrl_set_io_voltage(sd_pwr_handle, 0);
	vTaskDelay(pdMS_TO_TICKS(150));

	cfg.mode = GPIO_MODE_INPUT;
	gpio_config(&cfg);
	sd_pwr_ctrl_set_io_voltage(sd_pwr_handle, 3300);
	vTaskDelay(pdMS_TO_TICKS(150));
}
#endif

void storage_init(void)
{
#ifdef SD_CLK
	/* Init LDO4 power control — same as launcher's initialize_sd_ldo() */
	sd_pwr_ctrl_ldo_config_t ldo_config = {
		.ldo_chan_id = 4,
	};
	sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &sd_pwr_handle);

	reset_sd_card();

	esp_vfs_fat_sdmmc_mount_config_t mount_config = {
		.format_if_mount_failed = false,
		.max_files = 3,
		.allocation_unit_size = 16 * 1024,
	};

	sdmmc_card_t *card = NULL;

	ESP_LOGI(TAG, "Initializing SD card");

	/* SDMMC native slot 0 — mirrors launcher's sd_mount() */
	sdmmc_host_t host = SDMMC_HOST_DEFAULT();
	host.slot = SDMMC_HOST_SLOT_0;
	host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
	host.pwr_ctrl_handle = sd_pwr_handle;

	/* DMA buffer in internal RAM — same as launcher */
	static DRAM_DMA_ALIGNED_ATTR uint8_t dma_buf[512 * 4];
	host.dma_aligned_buffer = dma_buf;

	sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
	slot_config.clk   = SD_CLK;  /* GPIO 43 */
	slot_config.cmd   = SD_CMD;  /* GPIO 44 */
	slot_config.d0    = SD_D0;   /* GPIO 39 */
	slot_config.d1    = SD_D1;   /* GPIO 40 */
	slot_config.d2    = SD_D2;   /* GPIO 41 */
	slot_config.d3    = SD_D3;   /* GPIO 42 */
	slot_config.width = 4;

	ESP_LOGI(TAG, "Mounting filesystem");
	esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config,
						&mount_config, &card);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to mount SD card (%s)", esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG, "Filesystem mounted");
		sd_mounted = true;
		rawsd = card;
	}
#endif
}
