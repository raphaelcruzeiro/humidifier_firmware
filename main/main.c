#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "humidifier_control.h"
#include "esp_log.h"
#include "driver/gpio.h"

void app_main(void)
{
    ESP_LOGI("main", "Initializing humidifier control...");
    humidifier_control_init();
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI("main", "Turning humidifier ON");
    humidifier_control_set_power(true);
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI("main", "Turning humidifier OFF");
    humidifier_control_set_power(false);

    // int i = 0;
    // while (1) {
    //     printf("[%d] Hello world!\n", i);
    //     i++;
    //     vTaskDelay(5000 / portTICK_PERIOD_MS);
    // }
}
