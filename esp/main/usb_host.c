/*
 * USB Host subsystem for Tanmatsu
 * Handles USB mass storage devices for use as IDE drives
 */

#ifdef USE_BADGE_BSP

#include "usb_host.h"
#include "common.h"
#include "toast.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "bsp/power.h"
#include "usb/usb_host.h"
#include "usb/msc_host.h"
#include "usb/msc_host_vfs.h"
#include "esp_vfs_fat.h"

static const char *TAG = "usb_host";

// VFS mount point
#define USB_VFS_PATH "/usb"

// MSC device state
static msc_host_device_handle_t msc_device = NULL;
static msc_host_device_info_t msc_info = {0};
static SemaphoreHandle_t msc_mutex = NULL;
static volatile bool msc_connected = false;
static volatile bool vfs_mounted = false;
static msc_host_vfs_handle_t vfs_handle = NULL;

// USB host task handles
static TaskHandle_t usb_host_task_handle = NULL;
static TaskHandle_t msc_task_handle = NULL;

// Event queue for device notifications
static QueueHandle_t usb_event_queue = NULL;

typedef enum {
    USB_EVENT_DEVICE_CONNECTED,
    USB_EVENT_DEVICE_DISCONNECTED,
} usb_event_type_t;

typedef struct {
    usb_event_type_t type;
    uint8_t dev_addr;
} usb_event_t;

/**
 * @brief USB Host Library daemon task
 * Handles USB host library events
 */
static void usb_host_lib_task(void *arg)
{
    ESP_LOGI(TAG, "USB host library task started");

    while (1) {
        uint32_t event_flags;
        esp_err_t err = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "usb_host_lib_handle_events error: %s", esp_err_to_name(err));
            continue;
        }

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGD(TAG, "No more clients registered");
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGD(TAG, "All devices freed");
        }
    }
}

/**
 * @brief USB Host client event callback
 */
static void usb_host_client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    usb_event_t evt;

    switch (event_msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        fprintf(stderr, "USB: New device connected, addr=%d\n", event_msg->new_dev.address);
        evt.type = USB_EVENT_DEVICE_CONNECTED;
        evt.dev_addr = event_msg->new_dev.address;
        xQueueSend(usb_event_queue, &evt, 0);
        break;

    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        fprintf(stderr, "USB: Device disconnected\n");
        evt.type = USB_EVENT_DEVICE_DISCONNECTED;
        evt.dev_addr = 0;
        xQueueSend(usb_event_queue, &evt, 0);
        break;

    default:
        break;
    }
}

/**
 * @brief MSC event callback
 */
static void msc_event_cb(const msc_host_event_t *event, void *arg)
{
    switch (event->event) {
    case MSC_DEVICE_CONNECTED:
        ESP_LOGI(TAG, "MSC device connected");
        break;
    case MSC_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "MSC device disconnected");
        break;
    default:
        break;
    }
}

/**
 * @brief Handle USB device connection
 */
static void handle_device_connected(uint8_t dev_addr)
{
    fprintf(stderr, "USB: Handling device connect, addr=%d\n", dev_addr);

    if (msc_connected) {
        fprintf(stderr, "USB: MSC already connected, ignoring\n");
        return;
    }

    fprintf(stderr, "USB: Installing MSC device...\n");
    esp_err_t err = msc_host_install_device(dev_addr, &msc_device);
    if (err != ESP_OK) {
        fprintf(stderr, "USB: Not MSC or install failed: %s\n", esp_err_to_name(err));
        return;
    }

    fprintf(stderr, "USB: Getting device info...\n");
    err = msc_host_get_device_info(msc_device, &msc_info);
    if (err != ESP_OK) {
        fprintf(stderr, "USB: Failed to get info: %s\n", esp_err_to_name(err));
        msc_host_uninstall_device(msc_device);
        msc_device = NULL;
        return;
    }

    fprintf(stderr, "USB: Storage %lu sectors x %lu bytes = %lu MB\n",
             (unsigned long)msc_info.sector_count,
             (unsigned long)msc_info.sector_size,
             (unsigned long)(msc_info.sector_count * msc_info.sector_size / (1024 * 1024)));

    xSemaphoreTake(msc_mutex, portMAX_DELAY);
    msc_connected = true;
    globals.usb_storage_connected = true;
    globals.usb_mode = USB_MODE_IDE;  // Default to IDE mode
    xSemaphoreGive(msc_mutex);

    toast_show("USB storage connected");
}

/**
 * @brief Handle USB device disconnection
 */
static void handle_device_disconnected(void)
{
    if (!msc_connected) {
        return;
    }

    // Unmount VFS first if it was mounted
    if (vfs_mounted && vfs_handle) {
        msc_host_vfs_unregister(vfs_handle);
        vfs_handle = NULL;
        vfs_mounted = false;
    }

    xSemaphoreTake(msc_mutex, portMAX_DELAY);
    msc_connected = false;
    globals.usb_storage_connected = false;
    globals.usb_mode = USB_MODE_NONE;
    xSemaphoreGive(msc_mutex);

    if (msc_device) {
        msc_host_uninstall_device(msc_device);
        msc_device = NULL;
    }
    memset(&msc_info, 0, sizeof(msc_info));

    toast_show("USB storage removed");
}

/**
 * @brief MSC class driver task
 * Handles MSC events and device connections
 */
static void msc_task(void *arg)
{
    fprintf(stderr, "USB: MSC task starting...\n");

    // Register USB host client
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = usb_host_client_event_cb,
            .callback_arg = NULL,
        },
    };

    usb_host_client_handle_t client_handle;
    esp_err_t err = usb_host_client_register(&client_config, &client_handle);
    if (err != ESP_OK) {
        fprintf(stderr, "USB: Failed to register client: %s\n", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    fprintf(stderr, "USB: Client registered\n");

    // Install MSC class driver
    const msc_host_driver_config_t msc_config = {
        .create_backround_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = msc_event_cb,
    };

    err = msc_host_install(&msc_config);
    if (err != ESP_OK) {
        fprintf(stderr, "USB: Failed to install MSC driver: %s\n", esp_err_to_name(err));
        usb_host_client_deregister(client_handle);
        vTaskDelete(NULL);
        return;
    }

    fprintf(stderr, "USB: MSC driver installed, waiting for devices...\n");

    // Main event loop
    usb_event_t evt;
    while (1) {
        // Handle USB host client events (non-blocking to allow queue checks)
        usb_host_client_handle_events(client_handle, pdMS_TO_TICKS(10));

        // Check for device events
        if (xQueueReceive(usb_event_queue, &evt, pdMS_TO_TICKS(10)) == pdTRUE) {
            switch (evt.type) {
            case USB_EVENT_DEVICE_CONNECTED:
                handle_device_connected(evt.dev_addr);
                break;
            case USB_EVENT_DEVICE_DISCONNECTED:
                handle_device_disconnected();
                break;
            }
        }
    }
}

esp_err_t usb_host_init(void)
{
    esp_err_t err;

    fprintf(stderr, "USB: Initializing USB host subsystem\n");

    // Create mutex for MSC access
    msc_mutex = xSemaphoreCreateMutex();
    if (!msc_mutex) {
        ESP_LOGE(TAG, "Failed to create MSC mutex");
        return ESP_ERR_NO_MEM;
    }

    // Create event queue
    usb_event_queue = xQueueCreate(10, sizeof(usb_event_t));
    if (!usb_event_queue) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return ESP_ERR_NO_MEM;
    }

    // Enable 5V boost for USB-A port
    fprintf(stderr, "USB: Enabling 5V boost...\n");
    err = bsp_power_set_usb_host_boost_enabled(true);
    if (err != ESP_OK) {
        fprintf(stderr, "USB: Failed to enable boost: %s\n", esp_err_to_name(err));
        return err;
    }
    fprintf(stderr, "USB: 5V boost enabled\n");

    // Wait for power to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));

    // Install USB host library
    fprintf(stderr, "USB: Installing USB host library...\n");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };

    err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        fprintf(stderr, "USB: Failed to install host: %s\n", esp_err_to_name(err));
        bsp_power_set_usb_host_boost_enabled(false);
        return err;
    }
    fprintf(stderr, "USB: Host library installed\n");

    // Create USB host library task
    xTaskCreatePinnedToCore(usb_host_lib_task, "usb_host_lib", 4096,
                            NULL, 10, &usb_host_task_handle, 0);
    fprintf(stderr, "USB: Host lib task created\n");

    // Create MSC task
    xTaskCreatePinnedToCore(msc_task, "msc_task", 4096,
                            NULL, 5, &msc_task_handle, 0);
    fprintf(stderr, "USB: MSC task created\n");

    fprintf(stderr, "USB: Initialization complete\n");
    return ESP_OK;
}

bool usb_host_msc_connected(void)
{
    return msc_connected;
}

uint64_t usb_host_msc_get_sector_count(void)
{
    if (!msc_connected) {
        return 0;
    }
    return msc_info.sector_count;
}

uint32_t usb_host_msc_get_sector_size(void)
{
    if (!msc_connected) {
        return 0;
    }
    return msc_info.sector_size;
}

int usb_host_msc_read(uint64_t sector, uint8_t *buf, int count)
{
    if (!msc_connected || !msc_device || !buf) {
        return -1;
    }

    xSemaphoreTake(msc_mutex, portMAX_DELAY);

    if (!msc_connected) {
        xSemaphoreGive(msc_mutex);
        return -1;
    }

    esp_err_t err = msc_host_read_sector(msc_device, sector, buf,
                                          count * msc_info.sector_size);
    xSemaphoreGive(msc_mutex);

    return (err == ESP_OK) ? 0 : -1;
}

int usb_host_msc_write(uint64_t sector, const uint8_t *buf, int count)
{
    if (!msc_connected || !msc_device || !buf) {
        return -1;
    }

    xSemaphoreTake(msc_mutex, portMAX_DELAY);

    if (!msc_connected) {
        xSemaphoreGive(msc_mutex);
        return -1;
    }

    esp_err_t err = msc_host_write_sector(msc_device, sector, buf,
                                           count * msc_info.sector_size);
    xSemaphoreGive(msc_mutex);

    return (err == ESP_OK) ? 0 : -1;
}

esp_err_t usb_host_vfs_mount(void)
{
    if (!msc_connected || !msc_device) {
        ESP_LOGW(TAG, "No USB device to mount");
        return ESP_ERR_INVALID_STATE;
    }

    if (vfs_mounted) {
        ESP_LOGW(TAG, "USB VFS already mounted");
        return ESP_OK;
    }

    xSemaphoreTake(msc_mutex, portMAX_DELAY);

    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 0,  // Use default
    };

    esp_err_t err = msc_host_vfs_register(msc_device, USB_VFS_PATH, &mount_config, &vfs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount USB VFS: %s", esp_err_to_name(err));
        xSemaphoreGive(msc_mutex);
        return err;
    }

    vfs_mounted = true;
    xSemaphoreGive(msc_mutex);

    ESP_LOGI(TAG, "USB mounted at %s", USB_VFS_PATH);
    return ESP_OK;
}

esp_err_t usb_host_vfs_unmount(void)
{
    if (!vfs_mounted || !vfs_handle) {
        return ESP_OK;  // Already unmounted
    }

    xSemaphoreTake(msc_mutex, portMAX_DELAY);

    esp_err_t err = msc_host_vfs_unregister(vfs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unmount USB VFS: %s", esp_err_to_name(err));
        xSemaphoreGive(msc_mutex);
        return err;
    }

    vfs_handle = NULL;
    vfs_mounted = false;
    xSemaphoreGive(msc_mutex);

    ESP_LOGI(TAG, "USB unmounted from %s", USB_VFS_PATH);
    return ESP_OK;
}

bool usb_host_vfs_mounted(void)
{
    return vfs_mounted;
}

#endif /* USE_BADGE_BSP */
