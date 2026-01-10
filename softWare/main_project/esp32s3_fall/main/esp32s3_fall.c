#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

// -------------------------------------------------------------------------
// 1. CẤU HÌNH
// -------------------------------------------------------------------------
#define I2C_MASTER_SCL_IO           9    
#define I2C_MASTER_SDA_IO           8    
#define I2C_MASTER_NUM              0    
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_TIMEOUT_MS              1000
#define MAX30102_ADDR               0x57 

// Thanh ghi MAX30102
#define REG_FIFO_DATA               0x07
#define REG_FIFO_CONFIG             0x08
#define REG_MODE_CONFIG             0x09
#define REG_SPO2_CONFIG             0x0A
#define REG_LED1_PA                 0x0C
#define REG_LED2_PA                 0x0D
#define REG_FIFO_WR_PTR             0x04
#define REG_OVF_COUNTER             0x05
#define REG_FIFO_RD_PTR             0x06

static const char *TAG = "HEART_RATE";

// -------------------------------------------------------------------------
// 2. BIẾN TOÀN CỤC XỬ LÝ NHỊP TIM
// -------------------------------------------------------------------------
#define RATE_SIZE 10 // Tăng lên 10 để trung bình mượt hơn
uint8_t rates[RATE_SIZE]; 
uint8_t rateSpot = 0;
long lastBeat = 0; 

float beatsPerMinute = 0;
int beatAvg = 0; // Giá trị này sẽ được gửi đi/in ra

// -------------------------------------------------------------------------
// 3. GIAO TIẾP I2C
// -------------------------------------------------------------------------
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t max30102_write_reg(uint8_t reg, uint8_t data) {
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MAX30102_ADDR, write_buf, sizeof(write_buf), I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t max30102_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MAX30102_ADDR, &reg, 1, data, len, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void max30102_init_sensor() {
    max30102_write_reg(REG_MODE_CONFIG, 0x40); // Reset
    vTaskDelay(pdMS_TO_TICKS(100));
    max30102_write_reg(REG_FIFO_CONFIG, 0x4F); // Avg 4
    max30102_write_reg(REG_MODE_CONFIG, 0x03); // SpO2 Mode
    max30102_write_reg(REG_SPO2_CONFIG, 0x27); // 100Hz, 411us
    max30102_write_reg(REG_LED1_PA, 0x24);     // RED ~7mA
    max30102_write_reg(REG_LED2_PA, 0x24);     // IR ~7mA
    max30102_write_reg(REG_FIFO_WR_PTR, 0x00);
    max30102_write_reg(REG_OVF_COUNTER, 0x00);
    max30102_write_reg(REG_FIFO_RD_PTR, 0x00);
    ESP_LOGI(TAG, "MAX30102 Configured");
}

// -------------------------------------------------------------------------
// 4. TASK CHÍNH
// -------------------------------------------------------------------------
void max30102_task(void *arg) {
    uint8_t data_buf[6]; 
    uint32_t ir_val = 0;
    long ir_avg_reg = 0;
    
    bool beatDetected = false;
    long delta = 0;
    
    // Biến quản lý thời gian báo cáo 5s
    int64_t last_report_time = esp_timer_get_time();
    int64_t current_time = 0;

    lastBeat = esp_timer_get_time() / 1000;

    ESP_LOGI(TAG, "Task started. Waiting for finger...");

    while (1) {
        // --- PHẦN 1: ĐO LIÊN TỤC (Tốc độ cao 10ms) ---
        esp_err_t ret = max30102_read_reg(REG_FIFO_DATA, data_buf, 6);
        
        if (ret == ESP_OK) {
            ir_val = ((data_buf[3] << 16) | (data_buf[4] << 8) | data_buf[5]) & 0x03FFFF;

            // Kiểm tra xem có đặt tay không (Ngưỡng 50,000)
            if (ir_val < 50000) {
                beatsPerMinute = 0;
                beatAvg = 0;
                ir_avg_reg = 0;
                // Reset mảng average
                for(int i=0; i<RATE_SIZE; i++) rates[i] = 0;
                rateSpot = 0;
            } else {
                // Thuật toán: Lọc DC
                if (ir_avg_reg == 0) ir_avg_reg = ir_val; 
                ir_avg_reg = (long)(0.95 * ir_avg_reg + 0.05 * ir_val);
                long ac_signal = ir_val - ir_avg_reg;

                // Thuật toán: Phát hiện biên xuống (Falling edge)
                // Ngưỡng phát hiện: -100 (có thể cần chỉnh nếu quá nhạy hoặc quá lì)
                if (ac_signal < -100 && !beatDetected) { 
                    beatDetected = true;
                    long currentMillis = esp_timer_get_time() / 1000;
                    delta = currentMillis - lastBeat;
                    lastBeat = currentMillis;

                    beatsPerMinute = 60000.0 / delta;

                    // Lọc giá trị rác: chỉ chấp nhận 40 < BPM < 180
                    if (beatsPerMinute > 40 && beatsPerMinute < 180) {
                        rates[rateSpot++] = (uint8_t)beatsPerMinute; 
                        rateSpot %= RATE_SIZE; 
                        
                        // Tính trung bình
                        int tempAvg = 0;
                        int validItems = 0;
                        for (uint8_t x = 0 ; x < RATE_SIZE ; x++) {
                            if (rates[x] != 0) {
                                tempAvg += rates[x];
                                validItems++;
                            }
                        }
                        if (validItems > 0) beatAvg = tempAvg / validItems;
                    }
                }

                // Reset cờ phát hiện nhịp
                if (ac_signal > 100 && beatDetected) {
                    beatDetected = false;
                }
            }
        }
        
        // --- PHẦN 2: BÁO CÁO MỖI 5 GIÂY ---
        current_time = esp_timer_get_time();
        if ((current_time - last_report_time) > 5000000) { // 5,000,000 us = 5s
            
            if (ir_val < 50000) {
                ESP_LOGW(TAG, "[5s Tick] Chưa đặt ngón tay (IR: %lu)", ir_val);
            } else {
                if (beatAvg > 40) {
                     ESP_LOGI(TAG, "[5s Tick] KẾT QUẢ ĐO: %d BPM", beatAvg);
                     // TODO: Gửi beatAvg qua Bluetooth ở đây
                } else {
                     ESP_LOGI(TAG, "[5s Tick] Đang tính toán... Giữ yên tay.");
                }
            }
            last_report_time = current_time;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Delay 10ms
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    max30102_init_sensor();
    xTaskCreate(max30102_task, "max30102_task", 4096, NULL, 5, NULL);
}