#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C Configuration
#define I2C_MASTER_SCL_IO           22      // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO           21      // GPIO number for I2C master data
#define I2C_MASTER_NUM              I2C_NUM_0   // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ          100000      // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0           // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0           // I2C master doesn't need buffer

// LCD Configuration
#define LCD_I2C_ADDR                0x27    // Default I2C address for PCF8574
#define LCD_COLS                    20      // Number of columns
#define LCD_ROWS                    4       // Number of rows

// PCF8574 bit mapping for LCD connections
#define LCD_BACKLIGHT               0x08
#define LCD_ENABLE                  0x04
#define LCD_READ_WRITE              0x02
#define LCD_REGISTER_SELECT         0x01

// LCD Commands
#define LCD_CLEARDISPLAY            0x01
#define LCD_RETURNHOME              0x02
#define LCD_ENTRYMODESET            0x04
#define LCD_DISPLAYCONTROL          0x08
#define LCD_CURSORSHIFT             0x10
#define LCD_FUNCTIONSET             0x20
#define LCD_SETCGRAMADDR            0x40
#define LCD_SETDDRAMADDR            0x80

// Entry mode flags
#define LCD_ENTRYRIGHT              0x00
#define LCD_ENTRYLEFT               0x02
#define LCD_ENTRYSHIFTINCREMENT     0x01
#define LCD_ENTRYSHIFTDECREMENT     0x00

// Display control flags
#define LCD_DISPLAYON               0x04
#define LCD_DISPLAYOFF              0x00
#define LCD_CURSORON                0x02
#define LCD_CURSOROFF               0x00
#define LCD_BLINKON                 0x01
#define LCD_BLINKOFF                0x00

// Function set flags
#define LCD_8BITMODE                0x10
#define LCD_4BITMODE                0x00
#define LCD_2LINE                   0x08
#define LCD_1LINE                   0x00
#define LCD_5x10DOTS                0x04
#define LCD_5x8DOTS                 0x00

// Function declarations
esp_err_t i2c_master_init(void);
esp_err_t lcd_i2c_init(void);
void lcd_write_byte(uint8_t data);
void lcd_write_nibble(uint8_t data);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_clear(void);
void lcd_home(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char* str);
void lcd_print_char(char c);
void lcd_backlight_on(void);
void lcd_backlight_off(void);
void lcd_create_char(uint8_t location, uint8_t charmap[]);
void lcd_write_custom_char(uint8_t location);

#endif // LCD_I2C_H