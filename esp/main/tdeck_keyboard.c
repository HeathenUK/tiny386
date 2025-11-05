#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "tdeck_hw.h"
#include "../../i8042.h"

static const char *TAG = "tdeck_kbd";

extern volatile void *thekbd;

/* PC/AT Keyboard Scancode Set 2 mapping
 * This is a basic mapping - adjust based on actual T-Deck keyboard layout
 */
static const uint8_t keycode_map[TDECK_KBD_NUM_ROWS][TDECK_KBD_NUM_COLS] = {
    // Row 0
    {0x1E, 0x30, 0x2E, 0x20}, // 'a', 'b', 'c', 'd' (example)
    // Row 1  
    {0x12, 0x21, 0x22, 0x23}, // 'e', 'f', 'g', 'h' (example)
    // Row 2
    {0x17, 0x24, 0x25, 0x26}, // 'i', 'j', 'k', 'l' (example)
    // Row 3
    {0x1C, 0x32, 0x31, 0x18}, // Enter, Space, 'o', 'p' (example)
};

static uint8_t key_state[TDECK_KBD_NUM_ROWS][TDECK_KBD_NUM_COLS] = {0};

#if TDECK_KBD_USE_I2C
/* I2C-based keyboard reading */
static void tdeck_keyboard_task(void *arg)
{
    ESP_LOGI(TAG, "Starting I2C keyboard task");
    
    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TDECK_KBD_SDA_PIN,
        .scl_io_num = TDECK_KBD_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = TDECK_KBD_I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_param_config(TDECK_KBD_I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(TDECK_KBD_I2C_PORT, conf.mode, 0, 0, 0));
    
    while (!thekbd) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    uint8_t key_data[16];
    while (1) {
        // Read keyboard state via I2C
        // Adjust read length and address based on actual keyboard hardware
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (TDECK_KBD_I2C_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, key_data, sizeof(key_data), I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(TDECK_KBD_I2C_PORT, cmd, pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            // Process key_data and map to PS/2 scancodes
            // This is a placeholder - adjust based on actual keyboard protocol
            for (int i = 0; i < sizeof(key_data); i++) {
                if (key_data[i] != 0) {
                    // Process key press/release
                    // ps2_put_keycode(thekbd, 1, keycode_map[...]);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Poll every 10ms
    }
}
#else
/* GPIO matrix keyboard scanning */
static const gpio_num_t row_pins[TDECK_KBD_NUM_ROWS] = {
    TDECK_KBD_ROW0_PIN,
    TDECK_KBD_ROW1_PIN,
    TDECK_KBD_ROW2_PIN,
    TDECK_KBD_ROW3_PIN,
};

static const gpio_num_t col_pins[TDECK_KBD_NUM_COLS] = {
    TDECK_KBD_COL0_PIN,
    TDECK_KBD_COL1_PIN,
    TDECK_KBD_COL2_PIN,
    TDECK_KBD_COL3_PIN,
};

static void tdeck_keyboard_task(void *arg)
{
    ESP_LOGI(TAG, "Starting GPIO matrix keyboard task");
    
    // Configure row pins as outputs (set high)
    for (int i = 0; i < TDECK_KBD_NUM_ROWS; i++) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << row_pins[i],
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(row_pins[i], 1);
    }
    
    // Configure column pins as inputs with pull-up
    for (int i = 0; i < TDECK_KBD_NUM_COLS; i++) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = 1ULL << col_pins[i],
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
    }
    
    // Wait for keyboard emulation to be ready
    while (!thekbd) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "Keyboard emulation ready, starting scan");
    
    while (1) {
        // Scan keyboard matrix
        for (int row = 0; row < TDECK_KBD_NUM_ROWS; row++) {
            // Set current row low
            gpio_set_level(row_pins[row], 0);
            vTaskDelay(pdMS_TO_TICKS(1)); // Small delay for settling
            
            // Read columns
            for (int col = 0; col < TDECK_KBD_NUM_COLS; col++) {
                int level = gpio_get_level(col_pins[col]);
                uint8_t pressed = (level == 0) ? 1 : 0; // Low = pressed (with pull-up)
                
                // Detect key state change
                if (pressed != key_state[row][col]) {
                    key_state[row][col] = pressed;
                    uint8_t keycode = keycode_map[row][col];
                    
                    if (keycode != 0 && thekbd) {
                        ps2_put_keycode(thekbd, pressed, keycode);
                        ESP_LOGD(TAG, "Key %d %s (row=%d, col=%d)", 
                                keycode, pressed ? "pressed" : "released", row, col);
                    }
                }
            }
            
            // Set row back high
            gpio_set_level(row_pins[row], 1);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Scan every 10ms
    }
}
#endif

void tdeck_keyboard_init(void)
{
    xTaskCreatePinnedToCore(tdeck_keyboard_task, "tdeck_kbd", 4096, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "T-Deck keyboard task created");
}
