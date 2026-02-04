/*
 * USB Host subsystem for Tanmatsu
 * Provides USB mass storage support via ESP-IDF USB Host Library
 */

#pragma once

#ifdef USE_BADGE_BSP

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initialize the USB host subsystem
 *
 * Enables 5V boost for USB-A port and starts USB host library.
 * Must be called after BSP initialization.
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t usb_host_init(void);

/**
 * @brief Check if USB mass storage device is connected
 *
 * @return true if a USB storage device is currently connected
 */
bool usb_host_msc_connected(void);

/**
 * @brief Get USB storage sector count
 *
 * @return Number of sectors (0 if no device connected)
 */
uint64_t usb_host_msc_get_sector_count(void);

/**
 * @brief Get USB storage sector size
 *
 * @return Sector size in bytes (typically 512, 0 if no device)
 */
uint32_t usb_host_msc_get_sector_size(void);

/**
 * @brief Read sectors from USB storage
 *
 * @param sector Starting sector number
 * @param buf Buffer to read into
 * @param count Number of sectors to read
 * @return 0 on success, -1 on error
 */
int usb_host_msc_read(uint64_t sector, uint8_t *buf, int count);

/**
 * @brief Write sectors to USB storage
 *
 * @param sector Starting sector number
 * @param buf Buffer to write from
 * @param count Number of sectors to write
 * @return 0 on success, -1 on error
 */
int usb_host_msc_write(uint64_t sector, const uint8_t *buf, int count);

#endif /* USE_BADGE_BSP */
