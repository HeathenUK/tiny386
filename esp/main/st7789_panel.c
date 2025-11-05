#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io_interface.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stddef.h>
#include <stdlib.h>

static const char *TAG = "st7789_panel";

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    unsigned int bits_per_pixel;
} st7789_panel_t;

static esp_err_t panel_st7789_del(esp_lcd_panel_t *panel);
static esp_err_t panel_st7789_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_st7789_init(esp_lcd_panel_t *panel);
static esp_err_t panel_st7789_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_st7789_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_st7789_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_st7789_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_st7789_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_st7789_disp_on_off(esp_lcd_panel_t *panel, bool off);

esp_err_t esp_lcd_new_panel_st7789(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    st7789_panel_t *st7789 = NULL;
    esp_err_t ret = ESP_OK;

    if (io == NULL || panel_dev_config == NULL || ret_panel == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    st7789 = calloc(1, sizeof(st7789_panel_t));
    if (st7789 == NULL) {
        return ESP_ERR_NO_MEM;
    }

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        gpio_config(&io_conf);
    }

    st7789->io = io;
    st7789->bits_per_pixel = panel_dev_config->bits_per_pixel;
    st7789->reset_gpio_num = panel_dev_config->reset_gpio_num;
    st7789->reset_level = panel_dev_config->flags.reset_active_high;
    st7789->base.del = panel_st7789_del;
    st7789->base.reset = panel_st7789_reset;
    st7789->base.init = panel_st7789_init;
    st7789->base.draw_bitmap = panel_st7789_draw_bitmap;
    st7789->base.invert_color = panel_st7789_invert_color;
    st7789->base.set_gap = panel_st7789_set_gap;
    st7789->base.mirror = panel_st7789_mirror;
    st7789->base.swap_xy = panel_st7789_swap_xy;
    st7789->base.disp_on_off = panel_st7789_disp_on_off;

    *ret_panel = &(st7789->base);
    ESP_LOGI(TAG, "New ST7789 panel @%p", st7789);

    return ESP_OK;
}

#define CONTAINER_OF(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))

static esp_err_t panel_st7789_del(esp_lcd_panel_t *panel)
{
    st7789_panel_t *st7789 = CONTAINER_OF(panel, st7789_panel_t, base);
    if (st7789->reset_gpio_num >= 0) {
        gpio_reset_pin(st7789->reset_gpio_num);
    }
    ESP_LOGI(TAG, "Del ST7789 panel @%p", st7789);
    free(st7789);
    return ESP_OK;
}

static esp_err_t panel_st7789_reset(esp_lcd_panel_t *panel)
{
    st7789_panel_t *st7789 = CONTAINER_OF(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;

    if (st7789->reset_gpio_num >= 0) {
        gpio_set_level(st7789->reset_gpio_num, st7789->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(st7789->reset_gpio_num, !st7789->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    } else {
        // Perform software reset
        esp_lcd_panel_io_tx_param(io, 0x01, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return ESP_OK;
}

static esp_err_t panel_st7789_init(esp_lcd_panel_t *panel)
{
    st7789_panel_t *st7789 = CONTAINER_OF(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;

    // Send initialization commands
    // Memory access control
    esp_lcd_panel_io_tx_param(io, 0x36, (uint8_t[]){0x00}, 1);
    // Interface pixel format: 16-bit/pixel
    esp_lcd_panel_io_tx_param(io, 0x3A, (uint8_t[]){0x55}, 1);
    // Column address set
    esp_lcd_panel_io_tx_param(io, 0x2A, (uint8_t[]){0x00, 0x00, 0x00, 0xEF}, 4);
    // Row address set
    esp_lcd_panel_io_tx_param(io, 0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0x3F}, 4);
    // Display on
    esp_lcd_panel_io_tx_param(io, 0x29, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    return ESP_OK;
}

static esp_err_t panel_st7789_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    st7789_panel_t *st7789 = CONTAINER_OF(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;
    int x1 = x_start + st7789->x_gap;
    int x2 = x_end + st7789->x_gap;
    int y1 = y_start + st7789->y_gap;
    int y2 = y_end + st7789->y_gap;

    // Column address set
    esp_lcd_panel_io_tx_param(io, 0x2A, (uint8_t[]) {
        (x1 >> 8) & 0xFF, x1 & 0xFF,
        ((x2 - 1) >> 8) & 0xFF, (x2 - 1) & 0xFF
    }, 4);
    // Row address set
    esp_lcd_panel_io_tx_param(io, 0x2B, (uint8_t[]) {
        (y1 >> 8) & 0xFF, y1 & 0xFF,
        ((y2 - 1) >> 8) & 0xFF, (y2 - 1) & 0xFF
    }, 4);
    // Memory write
    size_t len = (x2 - x1) * (y2 - y1) * st7789->bits_per_pixel / 8;
    esp_lcd_panel_io_tx_color(io, 0x2C, color_data, len);
    return ESP_OK;
}

static esp_err_t panel_st7789_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    st7789_panel_t *st7789 = CONTAINER_OF(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;
    esp_lcd_panel_io_tx_param(io, 0x21, NULL, 0); // Display inversion on
    return ESP_OK;
}

static esp_err_t panel_st7789_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    st7789_panel_t *st7789 = CONTAINER_OF(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;
    uint8_t param = 0x00;
    if (mirror_x) {
        param |= 0x40; // MX
    }
    if (mirror_y) {
        param |= 0x80; // MY
    }
    esp_lcd_panel_io_tx_param(io, 0x36, &param, 1);
    return ESP_OK;
}

static esp_err_t panel_st7789_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    st7789_panel_t *st7789 = CONTAINER_OF(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;
    uint8_t param = 0x00;
    if (swap_axes) {
        param |= 0x20; // MV
    }
    esp_lcd_panel_io_tx_param(io, 0x36, &param, 1);
    return ESP_OK;
}

static esp_err_t panel_st7789_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    st7789_panel_t *st7789 = CONTAINER_OF(panel, st7789_panel_t, base);
    st7789->x_gap = x_gap;
    st7789->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_st7789_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    st7789_panel_t *st7789 = CONTAINER_OF(panel, st7789_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7789->io;
    uint8_t reg = on_off ? 0x29 : 0x28; // Display ON : Display OFF
    esp_lcd_panel_io_tx_param(io, reg, NULL, 0);
    return ESP_OK;
}
