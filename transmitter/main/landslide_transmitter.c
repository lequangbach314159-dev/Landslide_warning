#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"

// Tag cho logging
static const char *LORA_TAG = "LORA_SENDER";

// Cấu hình ADC
#define ADC_CHANNEL     ADC_CHANNEL_0  // GPIO36 (ADC1_CH0) - Độ ẩm đất
#define RAIN_ADC_CHANNEL ADC_CHANNEL_3 // GPIO39 (ADC1_CH3) - Cảm biến mưa
#define EXTRA_ADC_CHANNEL ADC_CHANNEL_9 // GPIO26 (ADC2_CH9) - Cảm biến phụ
#define ADC_ATTEN       ADC_ATTEN_DB_11

// Cấu hình MPU6500
#define I2C_MASTER_SCL_IO           22    // GPIO22 cho SCL
#define I2C_MASTER_SDA_IO           21    // GPIO21 cho SDA
#define I2C_MASTER_NUM              0     // I2C port number
#define I2C_MASTER_FREQ_HZ          400000 // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0     // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0     // I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS       1000

// MPU6500 I2C address và registers
#define MPU6500_ADDR                0x68
#define MPU6500_WHO_AM_I_REG        0x75
#define MPU6500_PWR_MGMT_1_REG      0x6B
#define MPU6500_ACCEL_CONFIG_REG    0x1C
#define MPU6500_ACCEL_XOUT_H_REG    0x3B

// Thuật toán chẩn đoán sạt lở 3 cấp độ
#define TILT_MODERATE_ANGLE         3.0   // Góc nghiêng vừa (độ)
#define TILT_STRONG_ANGLE           10.0  // Góc nghiêng mạnh (độ)
#define TILT_MODERATE_RATE          0.2   // Tốc độ nghiêng vừa (độ/phút)
#define TILT_STRONG_RATE            0.5   // Tốc độ nghiêng mạnh (độ/phút)

// Cấu hình cảm biến độ ẩm đất (chuẩn hóa 0-1)
#define MOISTURE_ADC_WET            1800  // ADC khi ẩm 100%
#define MOISTURE_ADC_DRY            4096  // ADC khi khô (0%)
#define SOIL_LEVEL1_THRESHOLD       0.60  // Cấp 1: S ≥ 0.60
#define SOIL_LEVEL2_THRESHOLD       0.80  // Cấp 2: S ≥ 0.80
#define SOIL_LEVEL3_THRESHOLD       0.90  // Cấp 3: S ≥ 0.90

// Cấu hình cảm biến lượng mưa (chuẩn hóa 0-1)
#define RAIN_ADC_WET                1800   // ADC khi có mưa nhiều
#define RAIN_ADC_DRY                4095  // ADC khi không mưa

#define RL3H_THRESHOLD              0.6   // Rain Load 3h ≥ 0.6
#define RL6H_THRESHOLD              0.5   // Rain Load 6h ≥ 0.5
#define RL24H_THRESHOLD             0.6   // Rain Load 24h ≥ 0.6

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

// Cấu hình deep sleep
#define SLEEP_TIME_MINUTES          0.1        // Ngủ 0.1 phút
#define WAKE_TIME_SECONDS           5       // Thức 5 giây
#define SLEEP_TIME_US               (SLEEP_TIME_MINUTES * 60 * 1000000ULL)  // 0.1 phút = 6000000us

// Biến toàn cục
static adc_oneshot_unit_handle_t adc1_handle;
static adc_oneshot_unit_handle_t adc2_handle;







// === I2C & MPU6500 FUNCTIONS ===

// Hàm khởi tạo I2C
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Hàm ghi register MPU6500
static esp_err_t mpu6500_write_reg(unsigned char reg_addr, unsigned char data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6500_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Hàm đọc register MPU6500
static esp_err_t mpu6500_read_reg(unsigned char reg_addr, unsigned char *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6500_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6500_ADDR << 1) | I2C_MASTER_READ, true);
    
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Hàm khởi tạo MPU6500
static esp_err_t mpu6500_init(void)
{
    esp_err_t ret;
    unsigned char who_am_i;
    
    // Kiểm tra WHO_AM_I
    ret = mpu6500_read_reg(MPU6500_WHO_AM_I_REG, &who_am_i, 1);
    if (ret != ESP_OK) return ret;
    
    if (who_am_i != 0x70) {
        return ESP_ERR_NOT_FOUND;
    }
    
    // Thoát khỏi chế độ sleep
    ret = mpu6500_write_reg(MPU6500_PWR_MGMT_1_REG, 0x00);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Cấu hình Accelerometer (±2g)
    ret = mpu6500_write_reg(MPU6500_ACCEL_CONFIG_REG, 0x00);
    if (ret != ESP_OK) return ret;
    
    return ESP_OK;
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

// Hàm đặt chế độ Sleep cho LoRa (M1=1, M0=1) - tiết kiệm pin
static void lora_set_sleep_mode(void)
{
    gpio_set_level(LORA_M0_PIN, 1);
    gpio_set_level(LORA_M1_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(20));
}

// Hàm đánh thức LoRa từ sleep mode
static void lora_wake_up(void)
{
    ESP_LOGI(LORA_TAG, "Waking up LoRa");
    lora_set_normal_mode();
    vTaskDelay(pdMS_TO_TICKS(100)); // Đợi module ổn định
}

// Hàm đợi AUX rảnh
static void lora_wait_aux(void)
{
    while (gpio_get_level(LORA_AUX_PIN) == 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelay(pdMS_TO_TICKS(2));
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

// Hàm gửi gói tin qua LoRa với checksum
static bool lora_send_packet(const char *data)
{
    if (!data) return false;
    
    unsigned char len = strlen(data);
    if (len == 0 || len > MAX_PACKET_SIZE) return false;
    
    // Tính checksum
    unsigned char checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum += data[i];
    }
    
    // Đợi module rảnh
    lora_wait_aux();
    
    // Gửi gói tin: Preamble + Length + Data + Checksum
    unsigned char preamble = PACKET_PREAMBLE;
    uart_write_bytes(LORA_UART_NUM, &preamble, 1);
    uart_write_bytes(LORA_UART_NUM, &len, 1);
    uart_write_bytes(LORA_UART_NUM, data, len);
    uart_write_bytes(LORA_UART_NUM, &checksum, 1);
    
    // Flush và đợi gửi xong
    uart_wait_tx_done(LORA_UART_NUM, pdMS_TO_TICKS(1000));
    lora_wait_aux();
    
    return true;
}

// Hàm khởi tạo ADC
static void adc_init(void)
{
    // Cấu hình ADC1 unit
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // Cấu hình ADC2 unit cho GPIO26
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    // Cấu hình channel cho cảm biến độ ẩm đất
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config));
    
    // Cấu hình channel cho cảm biến lượng mưa
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, RAIN_ADC_CHANNEL, &config));
    
    // Cấu hình channel cho cảm biến GPIO26
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXTRA_ADC_CHANNEL, &config));
}

// Hàm đọc giá trị ADC
static int read_adc_value(void)
{
    int adc_raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw));
    return adc_raw;
}

// Hàm tính độ ẩm chuẩn hóa (0-1)
static float calculate_soil_moisture_normalized(int adc_value)
{
    // Chuẩn hóa S = (4096 - ADC_soil) / (4096 - 2400)
    if (adc_value >= MOISTURE_ADC_DRY) {
        return 0.0;  // Khô hoàn toàn
    }
    if (adc_value <= MOISTURE_ADC_WET) {
        return 1.0;  // Ẩm hoàn toàn
    }
    
    float soil_moisture = (float)(MOISTURE_ADC_DRY - adc_value) / (float)(MOISTURE_ADC_DRY - MOISTURE_ADC_WET);
    return soil_moisture;  // Trả về 0-1
}

// === RAIN SENSOR FUNCTIONS ===
static int read_rain_adc_value(void)
{
    int adc_raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, RAIN_ADC_CHANNEL, &adc_raw));
    return adc_raw;
}

// Hàm tính lượng mưa chuẩn hóa (0-1)
static float calculate_rain_intensity_normalized(int adc_value)
{
    // Chuẩn hóa w_rain = (4095 - ADC_rain) / (4095 - 800)
    if (adc_value >= RAIN_ADC_DRY) {
        return 0.0;  // Không mưa
    }
    if (adc_value <= RAIN_ADC_WET) {
        return 1.0;  // Mưa to
    }
    float rain_intensity = (float)(RAIN_ADC_DRY - adc_value) / (float)(RAIN_ADC_DRY - RAIN_ADC_WET);
    return rain_intensity;  // Trả về 0-1
}

// === HÀM THUẬT TOÁN CHẨN ĐOÁN SẠT LỞ ===

// Tính Rain Intensity Proxy (RIP) với cơ chế chống báo giả






// === 3 HÀM ĐỌC CẢM BIẾN (CẬP NHẬT) ===

// Hàm đọc cảm biến độ ẩm (trả về giá trị chuẩn hóa 0-1)
static float read_moisture_sensor(void)
{
    int moisture_adc = read_adc_value();
    float soil_moisture = calculate_soil_moisture_normalized(moisture_adc);
    return soil_moisture;
}

// Hàm đọc cảm biến mưa (trả về giá trị chuẩn hóa 0-1)
static float read_rain_sensor(void)
{
    int rain_adc = read_rain_adc_value();
    float rain_intensity = calculate_rain_intensity_normalized(rain_adc);
    return rain_intensity;
}

// Hàm đọc ADC GPIO26
static int read_gpio26_adc_value(void)
{
    int adc_raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXTRA_ADC_CHANNEL, &adc_raw));
    return adc_raw;
}

// Hàm đọc cảm biến GPIO26 (chuyển thành phần trăm)
static float read_gpio26_sensor(void)
{
    int adc_value = read_gpio26_adc_value();
    
    // Chuyển đổi ADC sang điện áp (3.3V reference, 12-bit ADC)
    float voltage = (adc_value / 4095.0) * 3.3;
    
    // Ánh xạ tuyến tính: 2.15V = 0%, 3.0V = 100%
    const float MIN_VOLTAGE = 2.15;
    const float MAX_VOLTAGE = 3.0;
    
    // Giới hạn điện áp trong khoảng cho phép
    if (voltage < MIN_VOLTAGE) voltage = MIN_VOLTAGE;
    if (voltage > MAX_VOLTAGE) voltage = MAX_VOLTAGE;
    
    // Chuyển đổi tuyến tính thành phần trăm
    float percentage = ((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100.0;
    
    return percentage;
}

// Hàm đọc cảm biến MPU6500 (góc nghiêng)
static float read_mpu_sensor(void)
{
    unsigned char data[6];
    esp_err_t ret = mpu6500_read_reg(MPU6500_ACCEL_XOUT_H_REG, data, 6);
    
    if (ret != ESP_OK) {
        return 0.0; // Trả về 0 nếu lỗi đọc
    }
    
    // Chuyển đổi dữ liệu từ big-endian
    short accel_x = (data[0] << 8) | data[1];
    short accel_y = (data[2] << 8) | data[3];
    short accel_z = (data[4] << 8) | data[5];
    
    // Chuyển đổi sang giá trị vật lý (g)
    float ax = accel_x / 16384.0;
    float ay = accel_y / 16384.0;
    float az = accel_z / 16384.0;
    
    // Tính góc nghiêng (pitch)
    float pitch = atan(ax / sqrt(ay * ay + az * az)) * 180.0 / M_PI;
    
    return fabs(pitch); // Trả về giá trị tuyệt đối của góc nghiêng
}

// Task gửi dữ liệu qua LoRa với chế độ tiết kiệm pin
void lora_send_task(void *pvParameters)
{    
    while (1) {
        // Đánh thức LoRa
        lora_wake_up();
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Đọc 4 cảm biến
        float soil_moisture = read_moisture_sensor();       // Độ ẩm đất (0-1)
        float rain_intensity = read_rain_sensor();          // Cường độ mưa (0-1)  
        float tilt_angle = read_mpu_sensor();               // Góc nghiêng (độ)
        float gpio26_value = read_gpio26_sensor();          // Cảm biến GPIO26 (%)
        
        // Gửi 3 lần riêng biệt với số thứ tự
        // Lần 1: Gửi dữ liệu mưa
        char rain_packet[50];
        snprintf(rain_packet, sizeof(rain_packet), "1:%.0f%%", rain_intensity * 100.0);
        
        if (lora_send_packet(rain_packet)) {
            ESP_LOGI(LORA_TAG, "SENT RAIN: %s", rain_packet);
        } else {
            ESP_LOGE(LORA_TAG, "RAIN SEND FAILED!");
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Chờ 0.5s giữa các lần gửi
        
        // Lần 2: Gửi dữ liệu độ ẩm đất
        char soil_packet[50];
        snprintf(soil_packet, sizeof(soil_packet), "2:%.0f%%", soil_moisture * 100.0);
        
        if (lora_send_packet(soil_packet)) {
            ESP_LOGI(LORA_TAG, "SENT SOIL: %s", soil_packet);
        } else {
            ESP_LOGE(LORA_TAG, "SOIL SEND FAILED!");
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Chờ 0.5s giữa các lần gửi
        
        // Lần 3: Gửi dữ liệu góc nghiêng
        char tilt_packet[50];
        snprintf(tilt_packet, sizeof(tilt_packet), "3:%.1f", tilt_angle);
        
        if (lora_send_packet(tilt_packet)) {
            ESP_LOGI(LORA_TAG, "SENT TILT: %s", tilt_packet);
        } else {
            ESP_LOGE(LORA_TAG, "TILT SEND FAILED!");
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Chờ 0.5s giữa các lần gửi
        
        // Lần 4: Gửi dữ liệu GPIO26
        char gpio26_packet[50];
        snprintf(gpio26_packet, sizeof(gpio26_packet), "4:%.1f%%", gpio26_value);
        
        if (lora_send_packet(gpio26_packet)) {
            ESP_LOGI(LORA_TAG, "SENT GPIO26: %s", gpio26_packet);
        } else {
            ESP_LOGE(LORA_TAG, "GPIO26 SEND FAILED!");
        }

        // Đặt LoRa vào chế độ sleep
        lora_set_sleep_mode();
        
        // Đặt ESP32 vào deep sleep mode
        ESP_LOGI(LORA_TAG, "Going to deep sleep for %d minutes...", SLEEP_TIME_MINUTES);
        esp_sleep_enable_timer_wakeup(SLEEP_TIME_US);
        esp_deep_sleep_start();
        
        // Code sẽ không bao giờ đến đây vì ESP32 đã vào deep sleep
    }
}

void app_main(void)
{
    // Kiểm tra nguyên nhân wake-up
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(LORA_TAG, "=== WAKE UP FROM TIMER ===");
            break;
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            ESP_LOGI(LORA_TAG, "=== FIRST BOOT / RESET ===");
            break;
    }
    adc_init();
    lora_gpio_init();
    lora_uart_init();
    lora_set_normal_mode();
    i2c_master_init();
    mpu6500_init();

    // Tạo task gửi dữ liệu qua LoRa (chạy 1 lần rồi sleep)
    ESP_LOGI(LORA_TAG, "Starting sensor and transmission task...");
    xTaskCreate(lora_send_task, "lora_send_task", 4096, NULL, 5, NULL);
}