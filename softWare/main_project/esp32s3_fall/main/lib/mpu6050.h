#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "esp_err.h"

// --- CẤU HÌNH I2C ---
#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000

// --- ĐỊA CHỈ THANH GHI MPU6050 ---
#define MPU6050_SENSOR_ADDR         0x68
#define MPU6050_PWR_MGMT_1_REG      0x6B
#define MPU6050_ACCEL_XOUT_H        0x3B

// Struct chứa dữ liệu đã xử lý để dễ quản lý
typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_data_t;

// --- KHAI BÁO HÀM (PROTOTYPES) ---
/**
 * @brief Khởi tạo I2C và đánh thức MPU6050
 */
esp_err_t mpu6050_init(void);

/**
 * @brief Đọc toàn bộ dữ liệu Accel và Gyro
 * @param data Con trỏ tới struct mpu6050_data_t để lưu kết quả
 */
esp_err_t mpu6050_read_data(mpu6050_data_t *data);

#endif // MPU6050_H