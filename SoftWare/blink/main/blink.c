#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_PIN 48   // Đổi sang 38 nếu board bạn khác

void app_main(void)
{
    // Cấu hình GPIO làm output
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << LED_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    while (1) {
        gpio_set_level(LED_PIN, 1);  // Bật LED
        vTaskDelay(500 / portTICK_PERIOD_MS);

        gpio_set_level(LED_PIN, 0);  // Tắt LED
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
