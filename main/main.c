#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "humidifier_control.h"
#include "esp_log.h"
#include "driver/gpio.h"

static char *TAG = "main";

static void humidifier_status_callback(const humidifier_status_t *s_status);

void app_main(void)
{
    ESP_LOGI("main", "Initializing humidifier control...");
    humidifier_control_init();

    humidifier_control_register_callback(&humidifier_status_callback);

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI("main", "Turning humidifier ON");
    humidifier_control_set_power(true);
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI("main", "Turning humidifier OFF");
    humidifier_control_set_power(false);
}

static void humidifier_status_callback(const humidifier_status_t *s_status) {
    ESP_LOGI(TAG, "🧾 Summary → Power: %s | 🤖 Auto mode: %s | Mist Level: %s | Warm Mist: %s | Target Humidity: %s%% | Timer: %dh | Temp: %d°C | RH: %u%%",
        s_status->power ? "ON" : "OFF",
        s_status->auto_mode ? "ON" : "OFF",
        mist_level_to_string(s_status->mist_level),
        s_status->warm_mist ? "ON" : "OFF",
        target_humidity_to_string(s_status->target_humidity),
        s_status->timer_hours,
        (int)s_status->temperature_celsius,
        s_status->current_humidity
    );
}