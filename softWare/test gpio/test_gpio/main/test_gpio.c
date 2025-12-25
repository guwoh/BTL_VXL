#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

// Cấu hình chân ADC: GPIO 1 (Tương ứng ADC1 Channel 0 trên S3)
#define ADC_UNIT           ADC_UNIT_1
#define ADC_CHANNEL        ADC_CHANNEL_0 
#define ADC_ATTEN          ADC_ATTEN_DB_12 // Đọc dải điện áp lên tới ~3.1V

static const char *TAG = "ADC_RAW_TEST";

void app_main(void)
{
    // 1. Khởi tạo ADC Unit
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    // 2. Cấu hình Channel (Độ phân giải mặc định 12-bit: 0-4095)
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config);

    ESP_LOGI(TAG, "Bắt đầu đọc giá trị ADC Raw từ GPIO 1...");

    while (1) {
        int adc_raw;
        
        // 3. Đọc giá trị thô
        esp_err_t ret = adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw);
        
        if (ret == ESP_OK) {
            // Tính toán điện áp Pin xấp xỉ (Với 2 trở 1.2k chia đôi điện áp)
            // Công thức: (Raw / 4095) * 3.1V_max * 2_hệ_số_chia
            float pin_voltage = (adc_raw / 4095.0) * 3.1 * 2.0;

            printf("ADC Raw Value: %d  |  Estimated Pin: %.2f V\n", adc_raw, pin_voltage);
        } else {
            ESP_LOGE(TAG, "Lỗi đọc ADC!");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Đọc mỗi giây 1 lần
    }
}