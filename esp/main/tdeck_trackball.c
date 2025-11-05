#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "tdeck_hw.h"
#include "../../i8042.h"

static const char *TAG = "tdeck_tball";

extern volatile void *themouse;

/* Trackball sensor reading (typically PMW3360 or similar)
 * Adjust register addresses and protocol based on actual sensor chip
 */
static void tdeck_trackball_task(void *arg)
{
    ESP_LOGI(TAG, "Starting trackball task");
    
    // Initialize I2C for trackball
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TDECK_TRACKBALL_SDA_PIN,
        .scl_io_num = TDECK_TRACKBALL_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = TDECK_TRACKBALL_I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_param_config(TDECK_TRACKBALL_I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(TDECK_TRACKBALL_I2C_PORT, conf.mode, 0, 0, 0));
    
    // Wait for mouse emulation to be ready
    while (!themouse) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "Mouse emulation ready, starting trackball reading");
    
    int8_t dx = 0, dy = 0;
    uint8_t buttons = 0;
    
    while (1) {
        // Read trackball motion and buttons via I2C
        // This is a placeholder - adjust based on actual sensor protocol
        // Example for PMW3360-style sensor:
        
        uint8_t motion_reg = 0x02; // Motion register (example)
        uint8_t motion_data[3];
        
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (TDECK_TRACKBALL_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, motion_reg, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (TDECK_TRACKBALL_I2C_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, motion_data, sizeof(motion_data), I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(TDECK_TRACKBALL_I2C_PORT, cmd, pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            // Parse motion data (adjust based on sensor format)
            // Example: motion_data[0] = motion status, [1] = dx, [2] = dy
            uint8_t motion_status = motion_data[0];
            if (motion_status & 0x80) { // Motion detected
                dx = (int8_t)motion_data[1];
                dy = (int8_t)motion_data[2];
                
                // Read button states (adjust register address)
                uint8_t button_reg = 0x03; // Button register (example)
                uint8_t button_data;
                
                cmd = i2c_cmd_link_create();
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, (TDECK_TRACKBALL_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
                i2c_master_write_byte(cmd, button_reg, true);
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, (TDECK_TRACKBALL_I2C_ADDR << 1) | I2C_MASTER_READ, true);
                i2c_master_read_byte(cmd, &button_data, I2C_MASTER_NACK);
                i2c_master_stop(cmd);
                ret = i2c_master_cmd_begin(TDECK_TRACKBALL_I2C_PORT, cmd, pdMS_TO_TICKS(10));
                i2c_cmd_link_delete(cmd);
                
                if (ret == ESP_OK) {
                    buttons = 0;
                    if (button_data & 0x01) buttons |= (1 << 0); // Left button
                    if (button_data & 0x02) buttons |= (1 << 1); // Right button
                    if (button_data & 0x04) buttons |= (1 << 2); // Middle button
                    
                    // Send mouse event if there's motion or button change
                    if (dx != 0 || dy != 0 || buttons != 0) {
                        ps2_mouse_event(themouse, dx, dy, 0, buttons);
                        ESP_LOGD(TAG, "Trackball: dx=%d, dy=%d, buttons=0x%02x", dx, dy, buttons);
                    }
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Poll every 10ms
    }
}

void tdeck_trackball_init(void)
{
    xTaskCreatePinnedToCore(tdeck_trackball_task, "tdeck_tball", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "T-Deck trackball task created");
}
