#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"

// --- CẤU HÌNH PHẦN CỨNG ---
#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000 // Tăng lên 400kHz để đọc nhanh hơn
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// --- ĐỊA CHỈ MAX30102 ---
#define MAX30102_ADDR               0x57

// --- CÁC THANH GHI ---
#define REG_FIFO_WR_PTR             0x04
#define REG_FIFO_OVERFLOW           0x05
#define REG_FIFO_RD_PTR             0x06
#define REG_FIFO_DATA               0x07
#define REG_FIFO_CONFIG             0x08
#define REG_MODE_CONFIG             0x09
#define REG_SPO2_CONFIG             0x0A
#define REG_LED1_PA                 0x0C
#define REG_LED2_PA                 0x0D

static const char *TAG = "HEART_RATE";

// --- BIẾN TOÀN CỤC CHO THUẬT TOÁN BPM ---
#define BPM_BUFFER_SIZE 10 // Số lượng mẫu BPM để lấy trung bình
float bpm_buffer[BPM_BUFFER_SIZE];
int bpm_buffer_index = 0;

// Biến bộ lọc DC
float current_dc_val = 0;
const float alpha = 0.95; // Hệ số lọc (0.95 -> giữ lại 95% DC)

// Biến phát hiện nhịp
int64_t last_beat_time = 0;
float threshold = 200; // Ngưỡng phát hiện đỉnh sóng (tùy chỉnh nếu nhiễu)
bool beat_detected = false;

// --- HÀM I2C CƠ BẢN ---
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
    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t max30102_write_reg(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MAX30102_ADDR, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}

static esp_err_t max30102_read_reg(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MAX30102_ADDR, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

// --- KHỞI TẠO CẢM BIẾN ---
void max30102_init() {
    max30102_write_reg(0x09, 0x40); // Reset
    vTaskDelay(100 / portTICK_PERIOD_MS);
    max30102_write_reg(0x08, 0x40 | 0x0F); // Average 4 samples
    max30102_write_reg(0x09, 0x03); // SpO2 Mode
    max30102_write_reg(0x0A, 0x27); // 100 samples/s, 18-bit range
    max30102_write_reg(0x0C, 0x24); // LED Red ~7mA
    max30102_write_reg(0x0D, 0x24); // LED IR ~7mA
    max30102_write_reg(0x04, 0x00); // Clear FIFO pointers
    max30102_write_reg(0x05, 0x00);
    max30102_write_reg(0x06, 0x00);
}

// --- BIẾN TOÀN CỤC CẬP NHẬT ---
// Thêm biến cho bộ lọc Low-Pass
float filtered_ac = 0; 
const float low_pass_alpha = 0.2; // Hệ số làm mượt tín hiệu (Giảm nhiễu gai)

// Tinh chỉnh tham số thời gian
#define MIN_TIME_BETWEEN_BEATS 350 // 350ms ~ Max 170 BPM. Bỏ qua các nhịp nhanh hơn mức này (loại bỏ đỉnh phụ)

void process_heart_rate(uint32_t ir_value) {
    // 1. Loại bỏ DC (Nhiễu nền)
    if (current_dc_val == 0) current_dc_val = ir_value; 
    current_dc_val = (alpha * current_dc_val) + ((1.0 - alpha) * ir_value);
    
    float raw_ac = ir_value - current_dc_val;

    // 2. Bộ lọc thông thấp (Low Pass Filter) - QUAN TRỌNG
    // Giúp làm mượt tín hiệu AC, loại bỏ các gai nhiễu nhỏ
    filtered_ac = (low_pass_alpha * raw_ac) + ((1.0 - low_pass_alpha) * filtered_ac);

    // 3. Phát hiện đỉnh (Peak Detection)
    // Thay đổi logic: Đợi tín hiệu xuống thấp hẳn rồi mới cho phép bắt đỉnh mới
    
    int64_t current_time = esp_timer_get_time() / 1000;
    long time_diff = current_time - last_beat_time;

    // Logic chống nhiễu:
    // - Chỉ nhận nhịp nếu giá trị AC vượt ngưỡng (threshold)
    // - VÀ khoảng cách giữa 2 nhịp phải > 350ms (tránh đếm đỉnh phụ Dicrotic Notch)
    if (filtered_ac > threshold && filtered_ac > 0 && !beat_detected) {
        
        if (time_diff > MIN_TIME_BETWEEN_BEATS) {
            beat_detected = true;
            last_beat_time = current_time;

            // Nếu thời gian giữa 2 nhịp quá dài (>2s) coi như mới bắt đầu đo lại
            if (time_diff < 2000) {
                float instant_bpm = 60000.0 / time_diff;
                
                // Lọc thêm: Chỉ chấp nhận BPM trong khoảng người thường (40-160)
                if (instant_bpm > 40 && instant_bpm < 160) {
                    bpm_buffer[bpm_buffer_index] = instant_bpm;
                    bpm_buffer_index++;
                    if (bpm_buffer_index >= BPM_BUFFER_SIZE) bpm_buffer_index = 0;
                    
                    // (Tùy chọn) In debug để xem từng nhịp
                    // ESP_LOGI(TAG, "Beat detected! Time: %ld ms, BPM: %.1f", time_diff, instant_bpm);
                }
            }
        }
    } 
    
    // Reset cờ: Chỉ reset khi tín hiệu đã thực sự đi xuống âm (zero crossing)
    // Điều này giúp tránh việc tín hiệu dao động quanh đỉnh bị tính là nhiều nhịp
    else if (filtered_ac < 0) {
        beat_detected = false;
    }
}

// --- MAIN TASK ---
void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "Khoi tao I2C thanh cong: SDA=8, SCL=9");
    max30102_init();

    uint8_t fifo_data[6];
    uint32_t ir_val = 0;
    
    int64_t last_log_time = esp_timer_get_time() / 1000;
    int bpm_sum = 0;
    int valid_bpm_count = 0;

    while (1) {
        // Đọc dữ liệu từ FIFO
        esp_err_t err = max30102_read_reg(REG_FIFO_DATA, fifo_data, 6);
        
        if (err == ESP_OK) {
            // Lấy giá trị IR (Cảm biến nhịp tim chủ yếu dùng IR)
            ir_val = ((fifo_data[3] << 16) | (fifo_data[4] << 8) | fifo_data[5]) & 0x03FFFF;

            // Kiểm tra xem có đặt tay vào không (Ngưỡng 50,000)
            if (ir_val > 50000) {
                process_heart_rate(ir_val);
            } else {
                // Reset thuật toán nếu bỏ tay ra
                current_dc_val = 0;
                // Xóa buffer bpm
                for(int i=0; i<BPM_BUFFER_SIZE; i++) bpm_buffer[i] = 0;
            }

            // --- LOGGING MỖI 5 GIÂY ---
            int64_t now = esp_timer_get_time() / 1000;
            if (now - last_log_time >= 5000) {
                
                // Tính trung bình các giá trị trong buffer BPM
                float avg_bpm = 0;
                int count = 0;
                for (int i = 0; i < BPM_BUFFER_SIZE; i++) {
                    if (bpm_buffer[i] > 30) { // Chỉ lấy giá trị hợp lệ
                        avg_bpm += bpm_buffer[i];
                        count++;
                    }
                }
                if (count > 0) avg_bpm /= count;

                ESP_LOGI(TAG, "---------------------------------");
                if (ir_val < 50000) {
                    ESP_LOGW(TAG, "Trang thai: CHUA DAT TAY (IR=%lu)", ir_val);
                } else {
                    ESP_LOGI(TAG, "Trang thai: DANG DO...");
                    ESP_LOGI(TAG, "Raw IR    : %lu", ir_val);
                    
                    if (avg_bpm > 40 && avg_bpm < 200) {
                        ESP_LOGI(TAG, "NHIP TIM  : %.1f BPM", avg_bpm);
                    } else {
                        ESP_LOGI(TAG, "NHIP TIM  : Dang tinh toan...");
                    }
                }
                last_log_time = now;
            }

        }
        
        // Delay nhỏ phù hợp với tốc độ mẫu (100Hz -> 10ms)
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}