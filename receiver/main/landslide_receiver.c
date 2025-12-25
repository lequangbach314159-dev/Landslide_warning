#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "LCD_I2C.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include <math.h>

static const char *TAG = "LANDSLIDE_RECEIVER";
static const char *LCD_TAG = "LCD_I2C";
static const char *TELEGRAM_TAG = "TELEGRAM";

static uint8_t lcd_backlight_state = LCD_BACKLIGHT;
char LCD_rx_buffer[256];
char LCD_line1_buffer[21] = " M\001a   \002m    Nghi\003ng";  // Buffer cho dòng 1 LCD (header) - sử dụng ký tự đặc biệt
char LCD_line2_buffer[21] = {0};  // Buffer cho dòng 2 LCD (sensor data)
char LCD_line3_buffer[21] = {0};  // Buffer cho dòng 3 LCD (level data)
char LCD_line4_buffer[21] = {0};  // Buffer cho dòng 4 LCD

// Buffer riêng cho từng cảm biến với số thứ tự
char rain_data[10] = {0};     // Dữ liệu mưa
char soil_data[10] = {0};     // Dữ liệu độ ẩm đất
char tilt_data[10] = {0};     // Dữ liệu góc nghiêng
char gpio26_data[10] = {0};   // Dữ liệu GPIO26

// Trạng thái nhận dữ liệu theo số thứ tự
static bool packet_received[5] = {false, false, false, false, false}; // [0] không dùng, [1][2][3][4] cho packet 1,2,3,4
static char temp_rain_data[10] = {0};   // Buffer tạm cho mưa
static char temp_soil_data[10] = {0};   // Buffer tạm cho độ ẩm
static char temp_tilt_data[10] = {0};   // Buffer tạm cho nghiêng
static char temp_gpio26_data[10] = {0}; // Buffer tạm cho GPIO26
static uint32_t packet_timeout = 0;     // Timeout cho việc nhận packet

// Biến cho thuật toán đánh giá rủi ro sạt lở
static float RL_3h = 0.0;     // Rainfall Load 3 giờ
static float RL_6h = 0.0;     // Rainfall Load 6 giờ  
static float RL_24h = 0.0;    // Rainfall Load 24 giờ
static float prev_rain_norm = 0.0;  // Giá trị mưa chuẩn hóa trước đó
static uint32_t rain_start_time = 0; // Thời điểm bắt đầu mưa
static uint32_t soil_high_time = 0;  // Thời điểm đất ẩm cao
static int current_level = 0;        // Cấp độ cảnh báo hiện tại
static int previous_level = -1;       // Cấp độ trước đó để so sánh
static uint32_t level_change_time = 0; // Thời gian thay đổi cấp độ

// Hằng số thuật toán
#define RAIN_PERSISTENCE_TIME   30000   // 30 phút
#define SOIL_HIGH_TIME_REQ      15000   // 15 phút
#define LEVEL_CHANGE_DELAY      10000   // 10 phút trước khi thay đổi level
#define HYSTERESIS_FACTOR       0.9     // Hệ số hysteresis 10%

// Cấu hình cảnh báo Relay và Button
#define RELAY_PIN               25      // Chân điều khiển relay (GPIO25)
#define BUTTON_PIN              2       // Nút bấm tắt cảnh báo (GPIO2)

// Biến trạng thái cảnh báo
static bool alarm_active = false;      // Trạng thái cảnh báo đang hoạt động
static bool alarm_acknowledged = false; // Đã xác nhận (tắt) cảnh báo
static uint32_t button_last_press = 0; // Thời gian bấm nút cuối cùng



// === TELEGRAM & WiFi CONFIGURATION ===
// Wi-Fi cấu hình
#define WIFI_SSID      "nhà đầm trấu"
#define WIFI_PASS      "123@456@"

// Telegram Bot cấu hình (điền token/chat_id của bạn)
#define TELEGRAM_TOKEN "8568830965:AAEgdKAJBqWaciHG-anyWbDVjv0OoplfVsA"
#define TELEGRAM_CHAT_ID "7352036697"

// Biến WiFi và Telegram
static volatile bool wifi_ready = false;

// Cấu hình LoRa UART
#define LORA_UART_NUM               UART_NUM_2
#define LORA_TXD_PIN                17      // ESP32 TX2 → LoRa RXD
#define LORA_RXD_PIN                16      // ESP32 RX2 ← LoRa TXD
#define LORA_RTS_PIN                UART_PIN_NO_CHANGE
#define LORA_CTS_PIN                UART_PIN_NO_CHANGE

// Cấu hình chân điều khiển LoRa
#define LORA_M0_PIN                 4       // Chân M0
#define LORA_M1_PIN                 5       // Chân M1  
#define LORA_AUX_PIN                15      // Chân AUX (trạng thái)

// Cấu hình UART LoRa
#define LORA_BAUD_RATE              9600
#define LORA_BUF_SIZE               1024
#define PACKET_PREAMBLE             0xAA
#define MAX_PACKET_SIZE             256

// === WiFi & TELEGRAM FUNCTIONS ===

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TELEGRAM_TAG, "Wi-Fi disconnected, retrying...");
        esp_wifi_connect();
        wifi_ready = false;
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        char ip_str[16];
        esp_ip4_addr_t ip_addr = event->ip_info.ip;
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_addr));
        //ESP_LOGI(TELEGRAM_TAG, "Wi-Fi connected, got IP: %s", ip_str);
        wifi_ready = true;
    }
}

// Telegram alert struct
typedef struct {
    char message[512];
} telegram_alert_param_t;

// Telegram alert task
void telegram_alert_task(void *param) {
    telegram_alert_param_t *alert = (telegram_alert_param_t *)param;
    char safe_msg[512];
    
    // Encode message for URL (chỉ encode ký tự đặc biệt cơ bản)
    int j = 0;
    for (int i = 0; alert->message[i] && j < sizeof(safe_msg) - 4; ++i) {
        char c = alert->message[i];
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || 
            c == '-' || c == '_' || c == '.' || c == '~' || c == ' ' || c == '\n' || c == ':') {
            if (c == ' ') safe_msg[j++] = '+';
            else if (c == '\n') { safe_msg[j++] = '%'; safe_msg[j++] = '0'; safe_msg[j++] = 'A'; }
            else safe_msg[j++] = c;
        } else {
            snprintf(&safe_msg[j], 4, "%%%02X", (unsigned char)c);
            j += 3;
        }
    }
    safe_msg[j] = 0;

    char url[1024];
    snprintf(url, sizeof(url),
        "https://api.telegram.org/bot%s/sendMessage?chat_id=%s&text=%s",
        TELEGRAM_TOKEN, TELEGRAM_CHAT_ID, safe_msg);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 10000,  // Tăng timeout lên 10s
        .disable_auto_redirect = true,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 2048,  // Tăng buffer size
        .buffer_size_tx = 2048,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    //ESP_LOGI(TELEGRAM_TAG, "Sending Telegram alert: %s", alert->message);
    
    esp_err_t err = esp_http_client_perform(client);
    int status = -1;
    if (err == ESP_OK) {
        status = esp_http_client_get_status_code(client);
        //ESP_LOGI(TELEGRAM_TAG, "Telegram alert sent successfully, HTTP status: %d", status);
    } else {
        ESP_LOGE(TELEGRAM_TAG, "Telegram alert failed: %s", esp_err_to_name(err));
        status = esp_http_client_get_status_code(client);
        ESP_LOGW(TELEGRAM_TAG, "HTTP status (if any): %d", status);
    }
    
    esp_http_client_cleanup(client);
    free(alert);
    vTaskDelete(NULL);
}

// Send Telegram alert function
void send_telegram_alert(const char* message) {
    if (!wifi_ready) {
        ESP_LOGW(TELEGRAM_TAG, "WiFi not ready, cannot send Telegram alert");
        return;
    }
    
    telegram_alert_param_t *param = malloc(sizeof(telegram_alert_param_t));
    if (!param) {
        ESP_LOGE(TELEGRAM_TAG, "Failed to allocate memory for Telegram alert");
        return;
    }
    
    snprintf(param->message, sizeof(param->message), "%s", message);
    xTaskCreate(telegram_alert_task, "telegram_alert_task", 8192, param, 5, NULL);
}

// === ALARM RELAY & BUTTON FUNCTIONS ===

// Khởi tạo GPIO cho relay và button
static void alarm_gpio_init(void)
{
    // Cấu hình relay pin (output)
    gpio_config_t relay_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << RELAY_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&relay_conf);
    gpio_set_level(RELAY_PIN, 0); // Tắt relay ban đầu
    
    // Cấu hình button pin (input với pull-up)
    gpio_config_t button_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&button_conf);
    
    //ESP_LOGI(TAG, "Alarm GPIO initialized - Relay: %d, Button: %d", RELAY_PIN, BUTTON_PIN);
}

// Bật cảnh báo relay
static void alarm_activate(void)
{
    if (!alarm_active) {
        alarm_active = true;
        alarm_acknowledged = false;
        gpio_set_level(RELAY_PIN, 1); // Bật relay
        //ESP_LOGI(TAG, "🚨 ALARM ACTIVATED! Press button to acknowledge.");
    }
}

// Tắt cảnh báo relay
static void alarm_deactivate(void)
{
    if (alarm_active) {
        alarm_active = false;
        alarm_acknowledged = true;
        gpio_set_level(RELAY_PIN, 0); // Tắt relay
        //ESP_LOGI(TAG, "✅ ALARM DEACTIVATED");
    }
}

// Kiểm tra nút bấm (debounce 500ms)
static void check_button_press(void)
{
    static bool last_button_state = true; // Pull-up, nên mặc định HIGH
    bool current_button_state = gpio_get_level(BUTTON_PIN);
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Debug log trạng thái nút
    static uint32_t last_debug_time = 0;
    if (current_time - last_debug_time > 2000) { // Log mỗi 2s
        //ESP_LOGI(TAG, "🔘 BUTTON DEBUG: GPIO%d = %s, Alarm: %s", 
        //        BUTTON_PIN, current_button_state ? "HIGH" : "LOW", 
        //        alarm_active ? "ACTIVE" : "OFF");
        last_debug_time = current_time;
    }
    
    // Phát hiện nhấn nút (từ HIGH xuống LOW)
    if (last_button_state == true && current_button_state == false) {
        // Debounce 500ms
        if (current_time - button_last_press > 500) {
            button_last_press = current_time;
            
            if (alarm_active) {
                //ESP_LOGI(TAG, "🔘 BUTTON PRESSED - Acknowledging alarm");
                alarm_deactivate();
                
                // Gửi thông báo Telegram
                if (wifi_ready) {
                    send_telegram_alert("🔘 CẢNH BÁO ĐÃ ĐƯỢC XÁC NHẬN\n"
                                       "📡 Người dùng đã tắt cảnh báo\n"
                                       "⏰ Hệ thống tiếp tục giám sát");
                }
            }
        }
    }
    
    last_button_state = current_button_state;
}

// WiFi initialization
static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    //ESP_LOGI(TELEGRAM_TAG, "Wi-Fi STA connecting to SSID:%s", WIFI_SSID);
}

// === LORA FUNCTIONS ===

// Hàm khởi tạo GPIO cho LoRa
static void lora_gpio_init(void)
{
    // Cấu hình M0, M1 (output)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LORA_M0_PIN) | (1ULL << LORA_M1_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    
    // Cấu hình AUX (input)
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LORA_AUX_PIN);
    io_conf.pull_up_en = 1;  // Pull-up cho AUX
    gpio_config(&io_conf);
}

// Hàm đặt chế độ Normal (M1=0, M0=0)
static void lora_set_normal_mode(void)
{
    gpio_set_level(LORA_M0_PIN, 0);
    gpio_set_level(LORA_M1_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // Đợi AUX lên HIGH (module rảnh)
    while (gpio_get_level(LORA_AUX_PIN) == 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Hàm khởi tạo UART cho LoRa
static void lora_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = LORA_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Cài đặt UART parameters
    ESP_ERROR_CHECK(uart_param_config(LORA_UART_NUM, &uart_config));
    
    // Cài đặt UART pins
    ESP_ERROR_CHECK(uart_set_pin(LORA_UART_NUM, LORA_TXD_PIN, LORA_RXD_PIN, 
                                  LORA_RTS_PIN, LORA_CTS_PIN));
    
    // Cài đặt UART driver
    ESP_ERROR_CHECK(uart_driver_install(LORA_UART_NUM, LORA_BUF_SIZE * 2, 0, 0, NULL, 0));
}

// Hàm đọc gói tin qua LoRa với checksum validation
static bool lora_read_packet(char *buffer, size_t max_len)
{
    uint8_t data;
    int len;
    
    // Đọc preamble
    len = uart_read_bytes(LORA_UART_NUM, &data, 1, pdMS_TO_TICKS(100));
    if (len <= 0 || data != PACKET_PREAMBLE) {
        return false;
    }
    
    // Đọc độ dài
    len = uart_read_bytes(LORA_UART_NUM, &data, 1, pdMS_TO_TICKS(100));
    if (len <= 0 || data == 0) {
        return false;
    }
    
    uint8_t packet_len = data;
    if (packet_len >= max_len) {
        return false;
    }
    
    // Đọc dữ liệu
    len = uart_read_bytes(LORA_UART_NUM, (uint8_t*)buffer, packet_len, pdMS_TO_TICKS(500));
    if (len != packet_len) {
        return false;
    }
    
    // Đọc checksum
    uint8_t received_checksum;
    len = uart_read_bytes(LORA_UART_NUM, &received_checksum, 1, pdMS_TO_TICKS(100));
    if (len <= 0) {
        return false;
    }
    
    // Tính checksum
    uint8_t calculated_checksum = 0;
    for (int i = 0; i < packet_len; i++) {
        calculated_checksum += buffer[i];
    }
    
    // Kiểm tra checksum
    if (calculated_checksum != received_checksum) {
        ESP_LOGW(TAG, "Checksum mismatch: calc=0x%02X, recv=0x%02X", calculated_checksum, received_checksum);
        return false;
    }
    
    buffer[packet_len] = '\0';  // Null terminate
    return true;
}

/**
 * @brief Initialize I2C master
 */
esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(LCD_TAG, "I2C config failed");
        return err;
    }
    
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
                            I2C_MASTER_RX_BUF_DISABLE, 
                            I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(LCD_TAG, "I2C driver install failed");
        return err;
    }

    //ESP_LOGI(LCD_TAG, "I2C initialized successfully");
    return ESP_OK;
}

/**
 * @brief Write a byte to PCF8574 (LCD via I2C)
 */
void lcd_write_byte(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, LCD_I2C_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data | lcd_backlight_state, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

/**
 * @brief Write a nibble (4 bits) to LCD
 */
void lcd_write_nibble(uint8_t data)
{
    lcd_write_byte(data | LCD_ENABLE);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    lcd_write_byte(data & ~LCD_ENABLE);
    vTaskDelay(1 / portTICK_PERIOD_MS);
}

/**
 * @brief Send command to LCD
 */
void lcd_send_cmd(uint8_t cmd)
{
    uint8_t upper_nibble = cmd & 0xF0;
    uint8_t lower_nibble = (cmd << 4) & 0xF0;
    
    // Send upper nibble
    lcd_write_nibble(upper_nibble);
    // Send lower nibble
    lcd_write_nibble(lower_nibble);
    
    vTaskDelay(2 / portTICK_PERIOD_MS);
}

/**
 * @brief Send data to LCD
 */
void lcd_send_data(uint8_t data)
{
    uint8_t upper_nibble = data & 0xF0;
    uint8_t lower_nibble = (data << 4) & 0xF0;
    
    // Send upper nibble with RS high
    lcd_write_nibble(upper_nibble | LCD_REGISTER_SELECT);
    // Send lower nibble with RS high
    lcd_write_nibble(lower_nibble | LCD_REGISTER_SELECT);
    
    vTaskDelay(2 / portTICK_PERIOD_MS);
}

/**
 * @brief Initialize LCD
 */
esp_err_t lcd_i2c_init(void)
{
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        return err;
    }
    
    // Wait for LCD to power up
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    // Initialize LCD in 4-bit mode
    lcd_write_nibble(0x30);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_write_nibble(0x30);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    lcd_write_nibble(0x30);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    lcd_write_nibble(0x20);  // Set to 4-bit mode
    vTaskDelay(1 / portTICK_PERIOD_MS);
    
    // Function set: 4-bit mode, 2 lines, 5x8 dots
    lcd_send_cmd(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
    
    // Display control: display on, cursor off, blink off
    lcd_send_cmd(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    
    // Clear display
    lcd_clear();
    
    // Entry mode set: increment cursor, no shift
    lcd_send_cmd(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
    
    // Create custom degree symbol (°) at location 0
    uint8_t degree_char[8] = {
        0b00110,  // ..**.
        0b01001,  // .*..* 
        0b01001,  // .*..* 
        0b00110,  // ..**.
        0b00000,  // .....
        0b00000,  // .....
        0b00000,  // .....
        0b00000   // .....
    };
    lcd_create_char(0, degree_char);
    
    // Create custom character ư at location 1
    uint8_t u_horn_char[8] = {
        0b00010,  // *...*
        0b00001,  // *...*
        0b10010,  // *...*
        0b10010,  // *...*
        0b10010,  // *..**
        0b10110,  // .***.
        0b01010,  // ..*.. (horn)
        0b00000   // .....
    };
    lcd_create_char(1, u_horn_char);
    
    // Create custom character ẩ at location 2  
    uint8_t a_circumflex_char[8] = {
        0b01001,  // ..*..
        0b10101,  // .*.*.
        0b00000,  // .....
        0b01110,  // .***.
        0b10001,  // ....*
        0b11111,  // *...*
        0b10001,  // *...*
        0b10001   // *...*
    };
    lcd_create_char(2, a_circumflex_char);
    
    // Create custom character ê at location 3
    uint8_t e_circumflex_char[8] = {
        0b00100,  // ..*..
        0b01010,  // .*.*.
        0b00000,  // .....
        0b01110,  // .***.
        0b10001,  // *...*
        0b11111,  // *****
        0b10000,  // *....
        0b01110   // .***.
    };
    lcd_create_char(3, e_circumflex_char);

    //ESP_LOGI(LCD_TAG, "LCD initialized successfully");
    return ESP_OK;
}

/**
 * @brief Clear LCD display
 */
void lcd_clear(void)
{
    lcd_send_cmd(LCD_CLEARDISPLAY);
    vTaskDelay(2 / portTICK_PERIOD_MS);
}

/**
 * @brief Return cursor to home position
 */
void lcd_home(void)
{
    lcd_send_cmd(LCD_RETURNHOME);
    vTaskDelay(2 / portTICK_PERIOD_MS);
}

/**
 * @brief Set cursor position
 */
void lcd_set_cursor(uint8_t col, uint8_t row)
{
    // LCD 20x4 row offsets (DDRAM addresses)
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row >= LCD_ROWS) {
        row = LCD_ROWS - 1;
    }
    if (col >= LCD_COLS) {
        col = LCD_COLS - 1;
    }
    
    lcd_send_cmd(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}



/**
 * @brief Print a single character
 */
void lcd_print_char(char c)
{
    lcd_send_data(c);
}

/**
 * @brief Print a string to LCD
 */
void lcd_print(const char* str)
{
    while (*str) {
        lcd_print_char(*str++);
    }
}

/**
 * @brief Turn backlight on
 */
void lcd_backlight_on(void)
{
    lcd_backlight_state = LCD_BACKLIGHT;
    lcd_write_byte(0x00);
}

/**
 * @brief Turn backlight off
 */
void lcd_backlight_off(void)
{
    lcd_backlight_state = 0x00;
    lcd_write_byte(0x00);
}

/**
 * @brief Create custom character in CGRAM
 * @param location Location in CGRAM (0-7)
 * @param charmap Array of 8 bytes defining the character pattern
 */
void lcd_create_char(uint8_t location, uint8_t charmap[])
{
    location &= 0x7; // We only have 8 locations 0-7
    lcd_send_cmd(LCD_SETCGRAMADDR | (location << 3));
    
    for (int i = 0; i < 8; i++) {
        lcd_send_data(charmap[i]);
    }
}

/**
 * @brief Write custom character to LCD
 * @param location Location of custom character (0-7)
 */
void lcd_write_custom_char(uint8_t location)
{
    location &= 0x7; // We only have 8 locations 0-7
    lcd_send_data(location);
}

// Cập nhật Rainfall Load Index với EMA
static void update_rainfall_loads(float w_norm)
{
    // Hệ số alpha cho EMA (tương ứng 3h, 6h, 24h)
    float alpha_3h = 0.99;
    float alpha_6h = 0.995; 
    float alpha_24h = 0.999;
    
    RL_3h = alpha_3h * RL_3h + (1.0 - alpha_3h) * w_norm;
    RL_6h = alpha_6h * RL_6h + (1.0 - alpha_6h) * w_norm;
    RL_24h = alpha_24h * RL_24h + (1.0 - alpha_24h) * w_norm;
}

// Thuật toán đánh giá rủi ro sạt lở 3 cấp độ
static int assess_landslide_risk(float w_norm, float s_norm, float theta_deg, uint32_t current_time)
{
    int potential_level = 0;
    
    // Tính thời gian mưa liên tục
    float delta_w = w_norm - prev_rain_norm;
    if (w_norm >= 0.25 && delta_w >= 0.05) { // Mưa thật
        if (rain_start_time == 0) {
            rain_start_time = current_time;
        }
    } else if (w_norm < 0.25) {
        rain_start_time = 0;
    }
    uint32_t rain_duration = (rain_start_time > 0) ? (current_time - rain_start_time) : 0;
    
    // Theo dõi thời gian đất ẩm cao
    if (s_norm >= 0.8) {
        if (soil_high_time == 0) {
            soil_high_time = current_time;
        }
    } else if (s_norm < 0.8) {
        soil_high_time = 0;
    }
    uint32_t soil_high_duration = (soil_high_time > 0) ? (current_time - soil_high_time) : 0;
    
    // Logic 3 cấp độ
    // Level 3 - Emergency: S≥0.9 ∩ |θ|≥8° ∪ RL_24h≥0.6
    if ((s_norm >= 0.9 && fabs(theta_deg) >= 25.0) && RL_24h >= 0.6) {
        potential_level = 3;
    }
    // Level 2 - Alert: (RL_6h≥0.5 ∩ S≥0.8) ∪ |θ|≥3°
    else if ((RL_6h >= 0.5 && s_norm >= 0.8 && soil_high_duration >= SOIL_HIGH_TIME_REQ) && 
             fabs(theta_deg) >= 15.0) {
        potential_level = 2;
    }
    // Level 1 - Monitoring: RL_3h≥0.6 ∪ S≥0.6 ∩ |θ|<3°
    else if ((RL_3h >= 0.6 || s_norm >= 0.6 || rain_duration >= RAIN_PERSISTENCE_TIME) && 
             fabs(theta_deg) < 8.0) {
        potential_level = 1;
    }
    
    // Hysteresis và persistence filtering
    if (potential_level != current_level) {
        if (level_change_time == 0) {
            level_change_time = current_time;
        }
        
        // Đợi 10 phút trước khi thay đổi level
        if (current_time - level_change_time >= LEVEL_CHANGE_DELAY) {
            current_level = potential_level;
            level_change_time = 0;
        }
    } else {
        level_change_time = 0;
    }
    
    prev_rain_norm = w_norm;
    return current_level;
}

// Lấy mô tả cấp cảnh báo
static const char* get_level_description(int level)
{
    switch (level) {
        case 0: return "AN TOAN";
        case 1: return "THEO DOI";
        case 2: return "CANH GIAC"; 
        case 3: return "KHAN CAP";
        default: return "KHONG XAC DINH";
    }
}

// Task nhận dữ liệu qua LoRa
void lora_receive_task(void *pvParameters)
{
    char rx_buffer[256];
    
    //ESP_LOGI(TAG, "LoRa Receiver started");
    
    while (1) {
        if (lora_read_packet(rx_buffer, sizeof(rx_buffer))) {
            //ESP_LOGI(TAG, "RECEIVED: %s", rx_buffer);
            
            // Copy vào buffer chung
            for(int i=0; i<sizeof(rx_buffer); i++){
                LCD_rx_buffer[i] = rx_buffer[i];
            }
            
            // Kiểm tra số thứ tự packet (format: "N:data")
            if (strlen(rx_buffer) >= 3 && rx_buffer[1] == ':') {
                int packet_id = rx_buffer[0] - '0';  // Chuyển '1','2','3' thành 1,2,3
                char *data_part = &rx_buffer[2];     // Bỏ qua "N:"
                
                //ESP_LOGI(TAG, "Packet ID: %d, Data: %s", packet_id, data_part);
                
                // Kiểm tra packet hợp lệ
                if (packet_id >= 1 && packet_id <= 4) {
                    // Nếu đây là packet đầu tiên của chu kỳ mới
                    if (packet_id == 1) {
                        // Reset trạng thái
                        memset(packet_received, false, sizeof(packet_received));
                        memset(temp_rain_data, 0, sizeof(temp_rain_data));
                        memset(temp_soil_data, 0, sizeof(temp_soil_data));
                        memset(temp_tilt_data, 0, sizeof(temp_tilt_data));
                        packet_timeout = xTaskGetTickCount() * portTICK_PERIOD_MS;
                        //ESP_LOGI(TAG, "Starting new packet sequence");
                    }
                    
                    // Lưu dữ liệu vào buffer tạm theo packet_id
                    switch (packet_id) {
                        case 1: // Mưa
                            strncpy(temp_rain_data, data_part, sizeof(temp_rain_data)-1);
                            temp_rain_data[sizeof(temp_rain_data)-1] = '\0';
                            packet_received[1] = true;
                            ESP_LOGI(TAG, "Rain packet received: %s", temp_rain_data);
                            break;
                        case 2: // Độ ẩm đất
                            strncpy(temp_soil_data, data_part, sizeof(temp_soil_data)-1);
                            temp_soil_data[sizeof(temp_soil_data)-1] = '\0';
                            packet_received[2] = true;
                            ESP_LOGI(TAG, "Soil packet received: %s", temp_soil_data);
                            break;
                        case 3: // Góc nghiêng
                            strncpy(temp_tilt_data, data_part, sizeof(temp_tilt_data)-1);
                            temp_tilt_data[sizeof(temp_tilt_data)-1] = '\0';
                            packet_received[3] = true;
                            ESP_LOGI(TAG, "Tilt packet received: %s", temp_tilt_data);
                            break;
                        case 4: // GPIO26
                            strncpy(temp_gpio26_data, data_part, sizeof(temp_gpio26_data)-1);
                            temp_gpio26_data[sizeof(temp_gpio26_data)-1] = '\0';
                            packet_received[4] = true;
                            ESP_LOGI(TAG, "GPIO26 packet received: %s", temp_gpio26_data);
                            break;
                    }
                    
                    // Kiểm tra đã nhận đủ 4 packet chưa
                    if (packet_received[1] && packet_received[2] && packet_received[3] && packet_received[4]) {
                        //ESP_LOGI(TAG, "=== ALL 4 PACKETS RECEIVED - UPDATING ===");
                        
                        // Cập nhật dữ liệu chính thức
                        strcpy(rain_data, temp_rain_data);
                        strcpy(soil_data, temp_soil_data);
                        strcpy(tilt_data, temp_tilt_data);
                        strcpy(gpio26_data, temp_gpio26_data);
                        
                        // Cập nhật dòng 1 LCD khi có dữ liệu lần đầu
                        if (strlen(LCD_line1_buffer) == 0) {
                            strcpy(LCD_line1_buffer, "Rain  Soil  Tilt");
                        }
                        
                        // Xử lý thuật toán khi có đủ 3 cảm biến
                        float rain_norm = atof(rain_data) / 100.0;  // Chuyển % thành 0-1
                        float soil_norm = atof(soil_data) / 100.0;  // Chuyển % thành 0-1
                        float tilt_deg = atof(tilt_data);           // Góc nghiêng (độ)
                        
                        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                        
                        // Cập nhật Rainfall Load
                        update_rainfall_loads(rain_norm);
                        
                        // Đánh giá rủi ro sạt lở
                        int risk_level = assess_landslide_risk(rain_norm, soil_norm, tilt_deg, current_time);
                        
                        // Cập nhật buffer dòng 3 LCD với level
                        snprintf(LCD_line3_buffer, sizeof(LCD_line3_buffer), 
                                "Level %d - %s", risk_level, get_level_description(risk_level));
                        
                        ESP_LOGI(TAG, "ALGORITHM: Rain=%.1f%%, Soil=%.1f%%, Tilt=%.1f°, Level=%d-%s", 
                                 rain_norm*100, soil_norm*100, tilt_deg, risk_level, get_level_description(risk_level));
                        
                        // Xử lý cảnh báo âm thanh/relay theo level
                        //ESP_LOGI(TAG, "🔍 ALARM DEBUG: Level=%d, Active=%s, Acknowledged=%s", 
                        //        risk_level, alarm_active ? "YES" : "NO", alarm_acknowledged ? "YES" : "NO");
                        
                        if (risk_level >= 1 && !alarm_acknowledged) {
                            // Bật cảnh báo từ level 1 trở lên (nếu chưa được xác nhận)
                            //ESP_LOGI(TAG, "📢 SHOULD ACTIVATE ALARM - Level=%d", risk_level);
                            alarm_activate();
                        } else if (risk_level == 0 || alarm_acknowledged) {
                            // Tắt cảnh báo khi về level 0 hoặc đã được xác nhận
                            if (alarm_acknowledged) {
                                //ESP_LOGI(TAG, "🔇 ALARM ACKNOWLEDGED - Keeping OFF");
                            }
                            alarm_deactivate();
                            if (risk_level == 0) {
                                alarm_acknowledged = false; // Reset để sẵn sàng cho lần cảnh báo tiếp theo
                                //ESP_LOGI(TAG, "🔄 RESET ALARM STATE - Ready for next alert");
                            }
                        }
                        
                        // Gửi cảnh báo Telegram khi thay đổi level
                        if (risk_level != previous_level && wifi_ready) {
                            char telegram_msg[500];
                            
                            // Icon theo level
                            const char* level_icon;
                            switch(risk_level) {
                                case 0: level_icon = "✅"; break;  // An toàn
                                case 1: level_icon = "⚠️"; break;  // Theo dõi
                                case 2: level_icon = "🚨"; break;  // Cảnh giác
                                case 3: level_icon = "🔴"; break;  // Khẩn cấp
                                default: level_icon = "❓"; break;
                            }
                            
                            // Thêm thông tin cảnh báo vào tin nhắn
                            const char* alarm_status = alarm_active ? "\n🔊 CẢNH BÁO ÂM THANH: ĐANG BẬT" : 
                                                      (alarm_acknowledged ? "\n🔇 CẢNH BÁO: ĐÃ XÁC NHẬN" : "");
                            
                            snprintf(telegram_msg, sizeof(telegram_msg),
                                    "🌧️ CẢNH BÁO SẠT LỞ ĐẤT\n"
                                    "📊 Cảm biến: 🌧%.0f%% 💧%.0f%% 📐%.1f°\n"
                                    "🚨 Mức độ: Level %d - %s %s\n"
                                    "📡 Trạm thu: ONLINE ✅%s",
                                    rain_norm*100, soil_norm*100, tilt_deg,
                                    risk_level, get_level_description(risk_level), level_icon, alarm_status);
                            
                            send_telegram_alert(telegram_msg);
                            previous_level = risk_level;
                            
                            //ESP_LOGI(TELEGRAM_TAG, "Alert sent: Level %d-%s, Alarm: %s", 
                            //        risk_level, get_level_description(risk_level), 
                            //        alarm_active ? "ACTIVE" : "OFF");
                        }
                        
                        // Reset trạng thái cho chu kỳ tiếp theo
                        memset(packet_received, false, sizeof(packet_received));
                    }
                } else {
                    ESP_LOGW(TAG, "Invalid packet ID: %d", packet_id);
                }
            } else {
                ESP_LOGW(TAG, "Invalid packet format: %s", rx_buffer);
            }
        }
        
        // Kiểm tra timeout cho packet sequence (10 giây)
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (packet_timeout > 0 && (current_time - packet_timeout > 10000)) {
            if (packet_received[1] || packet_received[2] || packet_received[3] || packet_received[4]) {
                ESP_LOGW(TAG, "PACKET TIMEOUT - Missing packets: %s%s%s%s", 
                        packet_received[1] ? "" : "1 ",
                        packet_received[2] ? "" : "2 ",
                        packet_received[3] ? "" : "3 ",
                        packet_received[4] ? "" : "4 ");
                // Reset trạng thái
                memset(packet_received, false, sizeof(packet_received));
                packet_timeout = 0;
            }
        }
        
        // Kiểm tra nút bấm
        check_button_press();
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Đọc mỗi 100ms
    }
}

// Task hiển thị dữ liệu lên LCD 20x4
void lcd_display_task(void *pvParameters)
{
    char prev_line1[21] = {0};  // Lưu dòng 1 trước đó
    char prev_line3[21] = {0};  // Lưu dòng 3 trước đó
    
    while (1) {
        bool need_update = false;
        
        // Kiểm tra dòng 1 có thay đổi không (header - hiển thị 1 lần)
        if (strlen(LCD_line1_buffer) > 0 && strcmp(prev_line1, LCD_line1_buffer) != 0) {
            lcd_set_cursor(0, 0);
            lcd_print("                   "); // Xóa dòng 1 (19 ký tự)
            lcd_set_cursor(0, 0);
            lcd_print(LCD_line1_buffer);
            strcpy(prev_line1, LCD_line1_buffer);
            need_update = true;
        }
        
        // Hiển thị 3 sensor riêng biệt tại các tọa độ khác nhau
        static char prev_rain[10] = {0};
        static char prev_soil[10] = {0}; 
        static char prev_tilt[10] = {0};
        
        // Hiển thị dữ liệu mưa tại tọa độ (1,1)
        if (strlen(rain_data) > 0 && strcmp(prev_rain, rain_data) != 0) {
            lcd_set_cursor(1, 1);
            lcd_print("     "); // Xóa dữ liệu cũ (5 ký tự)
            lcd_set_cursor(1, 1);
            lcd_print(rain_data);
            strcpy(prev_rain, rain_data);
            need_update = true;
        }
        
        // Hiển thị dữ liệu độ ẩm đất tại tọa độ (6,1) 
        if (strlen(soil_data) > 0 && strcmp(prev_soil, soil_data) != 0) {
            lcd_set_cursor(7, 1);
            lcd_print("     "); // Xóa dữ liệu cũ (5 ký tự)
            lcd_set_cursor(7, 1);
            lcd_print(soil_data);
            strcpy(prev_soil, soil_data);
            need_update = true;
        }
        
        // Hiển thị dữ liệu góc nghiêng tại tọa độ (12,1)
        if (strlen(tilt_data) > 0 && strcmp(prev_tilt, tilt_data) != 0) {
            lcd_set_cursor(13, 1);
            lcd_print("      "); // Xóa dữ liệu cũ (6 ký tự)
            lcd_set_cursor(13, 1);
            lcd_print(tilt_data);
            lcd_write_custom_char(0); // Thêm ký tự độ (°)
            strcpy(prev_tilt, tilt_data);
            need_update = true;
        }
        
        // Kiểm tra dòng 3 có thay đổi không (level data)
        if (strlen(LCD_line3_buffer) > 0 && strcmp(prev_line3, LCD_line3_buffer) != 0) {
            lcd_set_cursor(1, 2);
            lcd_print("                   "); // Xóa dòng 3 (19 ký tự)
            lcd_set_cursor(1, 2);
            lcd_print(LCD_line3_buffer);
            strcpy(prev_line3, LCD_line3_buffer);
            need_update = true;
        }
        
        // Hiển thị thông tin pin dòng 4 (nếu có dữ liệu GPIO26)
        if (strlen(gpio26_data) > 0) {
            static char prev_gpio26[10] = {0};
            if (strcmp(prev_gpio26, gpio26_data) != 0) {
                // Cập nhật buffer dòng 4 với thông tin pin
                snprintf(LCD_line4_buffer, sizeof(LCD_line4_buffer), "Pin: %s", gpio26_data);
                
                lcd_set_cursor(1, 3);
                lcd_print("                   "); // Xóa dòng 4 (19 ký tự)
                lcd_set_cursor(1, 3);
                lcd_print(LCD_line4_buffer);
                strcpy(prev_gpio26, gpio26_data);
                need_update = true;
            }
        }
        
        if (need_update) {
            //ESP_LOGI(TAG, "LCD Updated - Line1: %s, Line2: %s, Line3: %s", LCD_line1_buffer, LCD_line2_buffer, LCD_line3_buffer);
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));  // Cập nhật nhanh hơn
    }
}

// Task gửi tin nhắn khởi động (chỉ 1 lần)
void startup_message_task(void *pvParameters)
{
    // Chờ WiFi kết nối
    while (!wifi_ready) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Gửi tin nhắn khởi động
    send_telegram_alert("🚀 HỆ THỐNG KHỞI ĐỘNG!\n"
                       "📡 Trạm thu sẵn sàng ✅\n"
                       "🌧️ Giám sát sạt lở đang hoạt động 📊");
    //ESP_LOGI(TELEGRAM_TAG, "Startup message sent!");
    
    // Xóa task này sau khi hoàn thành
    vTaskDelete(NULL);
}

void app_main(void)
{
    // === KHỞI TẠO NVS VÀ WiFi ===
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Khởi tạo WiFi
    wifi_init_sta();
    
    // === KHỞI TẠO LORA ===
    lora_gpio_init();
    lora_uart_init();
    lora_set_normal_mode();
     
    // === KHỞI TẠO ALARM SYSTEM ===
    alarm_gpio_init();
    
    // Turn on backlight
    lcd_backlight_on();
    
    // Initialize LCD
    esp_err_t err = lcd_i2c_init();
    if (err != ESP_OK) {
        ESP_LOGE(LCD_TAG, "LCD initialization failed: %s", esp_err_to_name(err));
        return;
    }

    // Hiển thị thông báo khởi tạo trên LCD 20x4

    lcd_set_cursor(0, 0);
    lcd_print("-Landslide Warning-");  // Dòng 1: Tiêu đề hệ thống
    vTaskDelay(pdMS_TO_TICKS(100));
    
    lcd_set_cursor(3, 2);
    lcd_print("Waiting data...");  // Dòng 3: Chờ dữ liệu
    vTaskDelay(pdMS_TO_TICKS(100));
    

    
    // === TẠO CÁC TASK ===
    // Task nhận dữ liệu qua LoRa  
    xTaskCreate(lora_receive_task, "lora_receive_task", 4096, NULL, 5, NULL);
    
    // Task hiển thị LCD
    xTaskCreate(lcd_display_task, "lcd_display_task", 2048, NULL, 4, NULL);
    
    // Task gửi tin nhắn khởi động (chạy 1 lần rồi tự xóa)
    xTaskCreate(startup_message_task, "startup_message_task", 4096, NULL, 3, NULL);

}