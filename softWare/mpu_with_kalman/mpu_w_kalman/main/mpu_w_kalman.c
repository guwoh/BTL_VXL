#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

// ============================================================
// 1. CẤU HÌNH PHẦN CỨNG (CHÂN KẾT NỐI)
// ============================================================
#define I2C_SDA_IO      5       // Chân dữ liệu I2C
#define I2C_SCL_IO      6       // Chân xung nhịp I2C
#define LED_PIN         8       // Chân đèn LED báo nhịp tim (có thể nối LED ngoài)
#define MAX30102_ADDR   0x57    // Địa chỉ I2C của cảm biến

// Thời gian báo cáo kết quả (5 giây = 5000 ms)
#define REPORT_INTERVAL_MS 5000 

// ============================================================
// 2. CÁC THANH GHI CỦA MAX30102 (KHÔNG CẦN SỬA)
// ============================================================
#define REG_FIFO_WR_PTR   0x04
#define REG_FIFO_OVF_CNT  0x05
#define REG_FIFO_RD_PTR   0x06
#define REG_FIFO_DATA     0x07
#define REG_FIFO_CONFIG   0x08
#define REG_MODE_CONFIG   0x09
#define REG_SPO2_CONFIG   0x0A
#define REG_LED1_PA       0x0C
#define REG_LED2_PA       0x0D

// ============================================================
// 3. BIẾN TOÀN CỤC (LƯU TRỮ KẾT QUẢ)
// ============================================================
double last_beat_time = 0; // Thời điểm nhịp tim trước đó
float current_bpm = 0;     // Nhịp tim hiện tại
float current_spo2 = 0;    // Nồng độ oxy hiện tại
int finger_detected = 0;   // Cờ báo có ngón tay hay không (1=Có, 0=Không)

// ============================================================
// 4. HÀM GIAO TIẾP I2C (DRIVER)
// ============================================================
static void i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000, // Tốc độ 400kHz
        .clk_flags = 0,
    };
    i2c_param_config(0, &conf);
    i2c_driver_install(0, conf.mode, 0, 0, 0);
}

// Hàm ghi vào thanh ghi MAX30102
static void i2c_write_reg(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

// ============================================================
// 5. KHỞI TẠO CẢM BIẾN MAX30102
// ============================================================
void max30102_init() {
    // Reset cảm biến
    i2c_write_reg(REG_MODE_CONFIG, 0x40);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Xóa các con trỏ bộ đệm (FIFO)
    i2c_write_reg(REG_FIFO_WR_PTR, 0x00);
    i2c_write_reg(REG_FIFO_OVF_CNT, 0x00);
    i2c_write_reg(REG_FIFO_RD_PTR, 0x00);

    // Cấu hình bộ đệm: Lấy trung bình 4 mẫu để giảm nhiễu
    i2c_write_reg(REG_FIFO_CONFIG, 0x20); 

    // Chế độ SpO2 (Dùng cả LED Đỏ và LED Hồng ngoại)
    i2c_write_reg(REG_MODE_CONFIG, 0x03);

    // Cấu hình SpO2: Dải đo 4096nA, Tần số 100Hz, Độ rộng xung 411us
    i2c_write_reg(REG_SPO2_CONFIG, 0x27);

    // Cấu hình độ sáng đèn LED (Dòng điện cấp cho LED)
    // 0x24 (khoảng 7mA) là mức trung bình, tiết kiệm pin và ít nóng
    i2c_write_reg(REG_LED1_PA, 0x24); // LED Đỏ
    i2c_write_reg(REG_LED2_PA, 0x24); // LED Hồng ngoại (IR)
}

// Đọc dữ liệu thô từ cảm biến
void read_max30102(uint32_t *red, uint32_t *ir) {
    uint8_t data[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Chọn thanh ghi dữ liệu FIFO
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, REG_FIFO_DATA, true);
    
    // Đọc 6 byte dữ liệu (3 byte RED, 3 byte IR)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        // Ghép các byte lại thành số nguyên 32-bit (chỉ lấy 18-bit dữ liệu thực)
        *red = ((uint32_t)(data[0] & 0x03) << 16) | ((uint32_t)data[1] << 8) | data[2];
        *ir  = ((uint32_t)(data[3] & 0x03) << 16) | ((uint32_t)data[4] << 8) | data[5];
    } else {
        *red = 0; *ir = 0;
    }
}

// ============================================================
// 6. THUẬT TOÁN XỬ LÝ TÍN HIỆU (NHỊP TIM & SpO2)
// ============================================================
void process_heart_rate(uint32_t red_raw, uint32_t ir_raw) {
    // --- BƯỚC 1: Kiểm tra có ngón tay không ---
    // Nếu giá trị hồng ngoại (IR) < 50,000 nghĩa là không có vật thể phản xạ (không có tay)
    if (ir_raw < 50000) {
        current_bpm = 0;
        current_spo2 = 0;
        finger_detected = 0; // Đánh dấu là không có tay
        gpio_set_level(LED_PIN, 1); // Tắt đèn (nếu điều khiển được)
        return;
    }
    finger_detected = 1; // Đã phát hiện ngón tay

    // --- BƯỚC 2: Bộ lọc DC (Loại bỏ thành phần một chiều) ---
    // Dùng bộ lọc trung bình động (Moving Average) để tìm mức nền (DC)
    static double avg_ir = 0;
    static double avg_red = 0;

    if (avg_ir == 0) avg_ir = ir_raw;
    if (avg_red == 0) avg_red = red_raw;

    // Lọc thông thấp (Low Pass Filter)
    avg_ir = 0.95 * avg_ir + 0.05 * ir_raw;
    avg_red = 0.95 * avg_red + 0.05 * red_raw;

    // Tách lấy tín hiệu xoay chiều (AC) - đây chính là tín hiệu nhịp tim
    double ac_ir = ir_raw - avg_ir;
    double ac_red = red_raw - avg_red;

    // --- BƯỚC 3: Phát hiện đỉnh nhịp tim (Peak Detection) ---
    // Ngưỡng để xác định một nhịp đập (cần tinh chỉnh tùy vào phần cứng)
    double threshold = -100.0; 
    static int was_below = 0;
    double now = (double)esp_timer_get_time() / 1000000.0; // Lấy thời gian hiện tại (giây)

    // Thuật toán: Khi tín hiệu đi xuống qua ngưỡng âm (do máu bơm vào làm hấp thụ ánh sáng tăng -> tín hiệu raw giảm)
    if (ac_ir < threshold && !was_below) {
        was_below = 1; // Đánh dấu đã vượt ngưỡng
        
        // Tính BPM nếu đây không phải là lần đầu tiên
        if (last_beat_time > 0) {
            double delta = now - last_beat_time; // Khoảng thời gian giữa 2 nhịp
            double instant_bpm = 60.0 / delta;   // Tính ra nhịp/phút
            
            // Lọc kết quả rác (Chỉ nhận BPM từ 40 đến 180)
            if (instant_bpm > 40 && instant_bpm < 180) {
                // Làm mượt kết quả bằng công thức trung bình trọng số
                if (current_bpm == 0) current_bpm = instant_bpm;
                else current_bpm = 0.7 * current_bpm + 0.3 * instant_bpm;
                
                // --- BƯỚC 4: Tính SpO2 ---
                // Tỷ lệ R = (AC_Red / DC_Red) / (AC_IR / DC_IR)
                double r_ratio = (fabs(ac_red) / avg_red) / (fabs(ac_ir) / avg_ir);
                
                // Công thức thực nghiệm chuẩn: SpO2 = 104 - 17 * R
                double instant_spo2 = 104.0 - 17.0 * r_ratio;
                
                // Giới hạn kết quả trong khoảng 0-100%
                if (instant_spo2 > 100) instant_spo2 = 100;
                if (instant_spo2 < 80) instant_spo2 = 80; // Giới hạn dưới để tránh hiện số ảo
                
                if (current_spo2 == 0) current_spo2 = instant_spo2;
                else current_spo2 = 0.8 * current_spo2 + 0.2 * instant_spo2;

                // Nháy đèn LED theo nhịp tim (nếu LED dùng được)
                gpio_set_level(LED_PIN, 0); 
            }
        }
        last_beat_time = now; // Cập nhật thời gian cho lần sau
    } else if (ac_ir > threshold) {
        was_below = 0; // Reset trạng thái khi tín hiệu quay về mức bình thường
        gpio_set_level(LED_PIN, 1); // Tắt đèn
    }
}

// ============================================================
// 7. CHƯƠNG TRÌNH CHÍNH (MAIN)
// ============================================================
void app_main(void) {
    // A. Khởi tạo LED
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 1); // Tắt mặc định

    // B. Khởi tạo I2C và Cảm biến
    i2c_init();
    max30102_init();

    printf("\n=== MAY DO NHIP TIM & SpO2 ===\n");
    printf("Khoi dong thanh cong!\n");
    printf("Vui long dat ngon tay len cam bien va giu yen...\n");
    printf("Ket qua se hien thi moi %d giay.\n\n", REPORT_INTERVAL_MS/1000);

    // Biến đếm thời gian báo cáo
    int64_t last_report_time = esp_timer_get_time();

    uint32_t red, ir;
    
    // C. Vòng lặp vô tận
    while (1) {
        // 1. ĐỌC & XỬ LÝ (Phần này phải chạy RẤT NHANH - 100 lần/giây)
        read_max30102(&red, &ir);
        process_heart_rate(red, ir);

        // 2. HIỂN THỊ KẾT QUẢ (Chỉ chạy mỗi 5 giây 1 lần)
        int64_t now = esp_timer_get_time();
        if ((now - last_report_time) > (REPORT_INTERVAL_MS * 1000)) {
            
            if (finger_detected) {
                // Nếu có ngón tay -> In kết quả đo được
                printf("--------------------------------\n");
                printf("[KET QUA DO]\n");
                // %.0f để làm tròn số nguyên cho đẹp
                printf(">> Nhip tim (BPM)   : %.0f nhip/phut\n", current_bpm); 
                printf(">> Oxy trong mau    : %.1f %%\n", current_spo2);
                printf("--------------------------------\n");
            } else {
                // Nếu không có ngón tay -> Nhắc nhở
                printf("[TRANG THAI] Khong thay ngon tay. Vui long dat tay vao!\n");
            }
            
            // Cập nhật lại thời gian báo cáo
            last_report_time = now;
        }

        // Delay ngắn để duy trì tần số lấy mẫu ~100Hz
        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}