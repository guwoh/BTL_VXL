#include "mpu6050.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Hàm nội bộ: Ghi vào thanh ghi
static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}

// Hàm nội bộ: Đọc từ thanh ghi
static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

// --- TRIỂN KHAI CÁC HÀM ĐÃ KHAI BÁO Ở .H ---

esp_err_t mpu6050_init(void)
{
    // 1. Cấu hình I2C
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

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) return err;

    // 2. Đánh thức MPU6050 (Ghi 0 vào PWR_MGMT_1)
    return mpu6050_register_write_byte(MPU6050_PWR_MGMT_1_REG, 0);
}

esp_err_t mpu6050_read_data(mpu6050_data_t *data)
{
    uint8_t raw_buf[14];
    esp_err_t err = mpu6050_register_read(MPU6050_ACCEL_XOUT_H, raw_buf, 14);

    if (err == ESP_OK) {
        // Dịch bit để ghép High byte và Low byte
        data->acc_x  = (raw_buf[0] << 8)  | raw_buf[1];
        data->acc_y  = (raw_buf[2] << 8)  | raw_buf[3];
        data->acc_z  = (raw_buf[4] << 8)  | raw_buf[5];
        // Bỏ qua nhiệt độ (byte 6, 7)
        data->gyro_x = (raw_buf[8] << 8)  | raw_buf[9];
        data->gyro_y = (raw_buf[10] << 8) | raw_buf[11];
        data->gyro_z = (raw_buf[12] << 8) | raw_buf[13];
    }
    return err;
}