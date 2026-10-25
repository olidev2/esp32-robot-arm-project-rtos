#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "hello_world";

void app_main(void)
{
    int counter = 0;

    while (1) {
        ESP_LOGI(TAG, "Hello world from ESP32! count=%d", counter++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
