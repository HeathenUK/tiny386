#ifndef TDECK_HW_H
#define TDECK_HW_H

#include "driver/gpio.h"
#include "driver/i2c.h"

/* ST7789 Display Pins
 * NOTE: Verify these against actual T-Deck schematic/datasheet
 * These are typical assignments for T-Deck ST7789 display
 */
#define TDECK_LCD_MOSI_PIN     GPIO_NUM_19
#define TDECK_LCD_SCLK_PIN     GPIO_NUM_18
#define TDECK_LCD_CS_PIN       GPIO_NUM_5
#define TDECK_LCD_DC_PIN       GPIO_NUM_23
#define TDECK_LCD_RST_PIN      GPIO_NUM_26
#define TDECK_LCD_BL_PIN       GPIO_NUM_4

/* Display Resolution */
#define TDECK_LCD_WIDTH        240
#define TDECK_LCD_HEIGHT       320

/* SPI Host for Display */
#define TDECK_LCD_SPI_HOST     SPI2_HOST

/* Trackball I2C Configuration
 * T-Deck typically uses I2C for trackball (e.g., PMW3360 or similar)
 * NOTE: Verify I2C address and chip model from T-Deck documentation
 */
#define TDECK_TRACKBALL_I2C_PORT    I2C_NUM_0
#define TDECK_TRACKBALL_SDA_PIN     GPIO_NUM_21
#define TDECK_TRACKBALL_SCL_PIN     GPIO_NUM_22
#define TDECK_TRACKBALL_I2C_FREQ    400000  // 400kHz
#define TDECK_TRACKBALL_I2C_ADDR    0x42    // Verify actual address

/* Keyboard Configuration
 * T-Deck keyboard may be GPIO matrix or I2C
 * GPIO matrix example pins (verify against schematic):
 */
#define TDECK_KBD_USE_I2C           0       // Set to 1 if I2C, 0 if GPIO matrix

#if TDECK_KBD_USE_I2C
#define TDECK_KBD_I2C_PORT          I2C_NUM_0
#define TDECK_KBD_SDA_PIN            GPIO_NUM_21
#define TDECK_KBD_SCL_PIN            GPIO_NUM_22
#define TDECK_KBD_I2C_FREQ          400000
#define TDECK_KBD_I2C_ADDR          0x55    // Verify actual address
#else
/* GPIO Matrix Keyboard Pins
 * These are example values - verify against T-Deck schematic
 * Typical: 4 rows x 4-6 columns
 */
#define TDECK_KBD_ROW0_PIN          GPIO_NUM_32
#define TDECK_KBD_ROW1_PIN          GPIO_NUM_33
#define TDECK_KBD_ROW2_PIN          GPIO_NUM_25
#define TDECK_KBD_ROW3_PIN          GPIO_NUM_26
#define TDECK_KBD_COL0_PIN          GPIO_NUM_27
#define TDECK_KBD_COL1_PIN          GPIO_NUM_14
#define TDECK_KBD_COL2_PIN          GPIO_NUM_12
#define TDECK_KBD_COL3_PIN          GPIO_NUM_13
#define TDECK_KBD_NUM_ROWS          4
#define TDECK_KBD_NUM_COLS          4
#endif

#endif /* TDECK_HW_H */
