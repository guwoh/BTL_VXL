#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"

#define TAG "MPU6050_MONITOR"

// --- CẤU HÌNH I2C ---
#define I2C_MASTER_SCL_IO           9     // Chân SCL
#define I2C_MASTER_SDA_IO           8     // Chân SDA
#define I2C_MASTER_NUM              0     // Cổng I2C 0
#define I2C_MASTER_FREQ_HZ          400000 // 400kHz
#define MPU6050_ADDR                0x68  // Địa chỉ mặc định

// --- CÁC THANH GHI MPU6050 ---
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_ACCEL_XOUT_H        0x3B

// --- HÀM KHỞI TẠO I2C ---
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// --- ĐÁNH THỨC MPU6050 ---
static void mpu6050_init() {
    // Ghi 0 vào thanh ghi 0x6B để đánh thức cảm biến (mặc định nó ngủ)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 da duoc khoi tao thanh cong!");
    } else {
        ESP_LOGE(TAG, "Khong tim thay MPU6050! Hay kiem tra day noi.");
    }
}

// --- ĐỌC DỮ LIỆU ---
void app_main(void) {
    // 1. Khởi tạo
    ESP_ERROR_CHECK(i2c_master_init());
    mpu6050_init();

    uint8_t data[6]; // Buffer chứa dữ liệu Ax, Ay, Az
    float ax, ay, az;
    float svm, roll, pitch;

    printf("\n--- BAT DAU GIAM SAT CHUYEN DONG ---\n");

    while (1) {
        // Tạo lệnh đọc 6 byte từ thanh ghi 0x3B
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, 5, I2C_MASTER_ACK);      // 5 byte đầu ACK
        i2c_master_read_byte(cmd, data + 5, I2C_MASTER_NACK); // Byte cuối NACK
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            // Ghép byte cao và thấp -> int16
            int16_t raw_ax = (data[0] << 8) | data[1];
            int16_t raw_ay = (data[2] << 8) | data[3];
            int16_t raw_az = (data[4] << 8) | data[5];

            // Chuyển sang đơn vị G (mặc định chia 16384.0)
            ax = raw_ax / 16384.0;
            ay = raw_ay / 16384.0;
            az = raw_az / 16384.0;

            // 1. Tính Vector tổng hợp (SVM)
            svm = sqrt(ax*ax + ay*ay + az*az);

            // 2. Tính góc nghiêng (Đơn giản hóa)
            // Roll: Nghiêng trái/phải
            // Pitch: Chúi đầu/Ngửa cổ
            roll  = atan2(ay, az) * 57.2958;
            pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 57.2958;

            // 3. Hiển thị ra màn hình
            // Dùng \r để in đè lên dòng cũ -> Trông như đồng hồ số
            printf("Ax: %5.2f | Ay: %5.2f | Az: %5.2f | SVM: %5.2f G | Roll: %5.1f | Pitch: %5.1f   \r", 
                    ax, ay, az, svm, roll, pitch);
            
            // Logic phát hiện ngã đơn giản
            if (svm > 2.5) {
                printf("\n\n>>> CANH BAO: PHAT HIEN NGA (SVM=%.2f) <<<\n\n", svm);
                vTaskDelay(pdMS_TO_TICKS(1000)); // Dừng 1s để nhìn thấy cảnh báo
            }

        } else {
            ESP_LOGE(TAG, "Loi doc cam bien!");
        }

        // Tốc độ đọc: 10Hz (100ms)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}