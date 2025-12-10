#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TAG "KALMAN_MONITOR"

// --- CẤU HÌNH I2C ---
#define I2C_SCL_IO 9
#define I2C_SDA_IO 8
#define MPU6050_ADDR 0x68

// ==========================================
// 1. KALMAN FILTER CHO GÓC (SENSOR FUSION)
// Kết hợp Accel (Góc) + Gyro (Tốc độ)
// ==========================================
typedef struct {
    double Q_angle;   // Nhiễu quá trình (Góc)
    double Q_bias;    // Nhiễu quá trình (Bias)
    double R_measure; // Nhiễu đo lường (Accel)
    double angle;     // OUTPUT: Góc sau lọc
    double bias;      // Bias ước lượng
    double P[2][2];   // Ma trận lỗi
} Kalman_t;

void Kalman_Init(Kalman_t *kalman) {
    kalman->Q_angle = 0.001;
    kalman->Q_bias = 0.003;
    kalman->R_measure = 0.03; // Tăng lên nếu muốn mượt hơn
    kalman->angle = 0.0;
    kalman->bias = 0.0;
    kalman->P[0][0] = 0.0; kalman->P[0][1] = 0.0;
    kalman->P[1][0] = 0.0; kalman->P[1][1] = 0.0;
}

double Kalman_GetAngle(Kalman_t *kalman, double newAngle, double newRate, double dt) {
    // 1. Dự đoán
    double rate = newRate - kalman->bias;
    kalman->angle += rate * dt;

    kalman->P[0][0] += dt * (dt*kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    // 2. Cập nhật
    double S = kalman->P[0][0] + kalman->R_measure;
    double K[2];
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    double y = newAngle - kalman->angle;
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;

    double P00_temp = kalman->P[0][0];
    double P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    return kalman->angle;
}

// ==========================================
// 2. KALMAN FILTER 1 CHIỀU (CHO SVM)
// Chỉ làm mượt tín hiệu, loại bỏ nhiễu gai nhỏ
// ==========================================
typedef struct {
    double err_measure;
    double err_estimate;
    double q;
    double current_estimate;
    double last_estimate;
    double kalman_gain;
} SimpleKalman_t;

void SimpleKalman_Init(SimpleKalman_t *k, double mea_e, double est_e, double q) {
    k->err_measure = mea_e;
    k->err_estimate = est_e;
    k->q = q;
    k->current_estimate = 0;
    k->last_estimate = 0;
}

double SimpleKalman_Update(SimpleKalman_t *k, double mea) {
    k->kalman_gain = k->err_estimate / (k->err_estimate + k->err_measure);
    k->current_estimate = k->last_estimate + k->kalman_gain * (mea - k->last_estimate);
    k->err_estimate = (1.0 - k->kalman_gain) * k->err_estimate + k->q * fabs(k->last_estimate - k->current_estimate);
    k->last_estimate = k->current_estimate;
    return k->current_estimate;
}

// --- HÀM I2C ---
static void i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER, .sda_io_num = I2C_SDA_IO, .scl_io_num = I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000, .clk_flags = 0
    };
    i2c_param_config(0, &conf);
    i2c_driver_install(0, conf.mode, 0, 0, 0);
}

static void mpu6050_wake() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x6B, true); i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

// --- MAIN ---
void app_main(void) {
    i2c_init();
    mpu6050_wake();

    // A. Khởi tạo Kalman cho Góc
    Kalman_t kRoll, kPitch;
    Kalman_Init(&kRoll);
    Kalman_Init(&kPitch);

    // B. Khởi tạo Kalman cho SVM
    SimpleKalman_t kSVM;
    SimpleKalman_Init(&kSVM, 0.5, 0.5, 0.01); // Q=0.01 để mượt, tăng lên 0.1 nếu muốn nhạy hơn

    uint8_t data[14];
    int64_t last_time = esp_timer_get_time();

    printf("pitch_raw,pitch_kalman,roll_raw,roll_kalman,svm_raw,svm_kalman\n");

    while(1) {
        // Đọc 14 bytes (Accel + Temp + Gyro)
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x3B, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, 13, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, data + 13, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        
        if (i2c_master_cmd_begin(0, cmd, pdMS_TO_TICKS(100)) == ESP_OK) {
            // 1. Lấy dữ liệu thô
            float ax = (int16_t)((data[0] << 8) | data[1]) / 16384.0;
            float ay = (int16_t)((data[2] << 8) | data[3]) / 16384.0;
            float az = (int16_t)((data[4] << 8) | data[5]) / 16384.0;
            float gx = (int16_t)((data[8] << 8) | data[9]) / 131.0;
            float gy = (int16_t)((data[10] << 8) | data[11]) / 131.0;

            // 2. Tính dt (giây)
            int64_t now = esp_timer_get_time();
            double dt = (now - last_time) / 1000000.0;
            last_time = now;

            // ==========================================
            // PITCH
            // ==========================================
            // Pitch Raw: Tính từ Accel (atan2)
            // Pitch = atan2(-Ax, sqrt(Ay^2 + Az^2)) * 57.29
            double pitch_raw = atan2(-ax, sqrt(ay*ay + az*az)) * 57.2958;
            
            // Pitch Kalman: Kết hợp Pitch Raw + Gyro Y
            double pitch_kalman = Kalman_GetAngle(&kPitch, pitch_raw, gy, dt);

            // ==========================================
            // ROLL
            // ==========================================
            // Roll Raw: Tính từ Accel
            // Roll = atan2(Ay, Az) * 57.29
            double roll_raw = atan2(ay, az) * 57.2958;

            // Roll Kalman: Kết hợp Roll Raw + Gyro X
            double roll_kalman = Kalman_GetAngle(&kRoll, roll_raw, gx, dt);

            // ==========================================
            // SVM (Signal Vector Magnitude)
            // ==========================================
            // SVM Raw: Tổng hợp lực
            double svm_raw = sqrt(ax*ax + ay*ay + az*az);

            // SVM Kalman: Lọc nhiễu 1 chiều
            double svm_kalman = SimpleKalman_Update(&kSVM, svm_raw);

            // 3. IN RA MONITOR (Đúng thứ tự bạn yêu cầu)
            printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
                   pitch_raw, pitch_kalman, 
                   roll_raw, roll_kalman, 
                   svm_raw, svm_kalman);
        }
        i2c_cmd_link_delete(cmd);
        vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hz
    }
}