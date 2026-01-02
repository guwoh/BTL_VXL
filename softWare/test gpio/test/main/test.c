#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Định nghĩa chân LED
#define LED_PIN GPIO_NUM_48

static const char *TAG = "TEST_LED";

// ĐÃ SỬA: Xóa extern "C"
void app_main(void)
{
    ESP_LOGI(TAG, "Khoi tao chan GPIO 48...");

    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "Bat dau chop tat LED...");

    while (1) {
        ESP_LOGI(TAG, "LED ON");
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        ESP_LOGI(TAG, "LED OFF");
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}