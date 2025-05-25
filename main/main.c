#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "cJSON.h"

#include "humidifier_control.h"
#include "wifi.h"
#include "humidifier_mqtt.h"

static char *TAG = "main";

static bool mqtt_ready = false;

static void humidifier_status_callback(const humidifier_status_t *s_status);
static char *humidifier_status_to_json(const humidifier_status_t *s_status);
static void wifi_ready_callback(void);
static void mqtt_ready_callback(void);

void app_main(void)
{
    ESP_LOGI("main", "Initializing humidifier Wifi...");

    wifi_register_connected_callback(&wifi_ready_callback);
    wifi_init_sta();

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

    if (!mqtt_ready) return;

    const char *payload = humidifier_status_to_json(s_status);
    humidifier_mqtt_publish_status(payload);
}

static char *humidifier_status_to_json(const humidifier_status_t *s_status) {
    cJSON *root = cJSON_CreateObject();

    cJSON_AddBoolToObject(root, "power", s_status->power);
    cJSON_AddBoolToObject(root, "auto_mode", s_status->auto_mode);
    cJSON_AddStringToObject(root, "mist_level", mist_level_to_string(s_status->mist_level));
    cJSON_AddBoolToObject(root, "warm_mist", s_status->warm_mist);
    cJSON_AddStringToObject(root, "target_humidity", target_humidity_to_string(s_status->target_humidity));
    cJSON_AddNumberToObject(root, "timer_hours", s_status->timer_hours);
    cJSON_AddNumberToObject(root, "temperature_celsius", s_status->temperature_celsius);
    cJSON_AddNumberToObject(root, "current_humidity", s_status->current_humidity);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_str;
}

static void wifi_ready_callback(void) {
    humidifier_mqtt_register_ready_callback(&mqtt_ready_callback);
    humidifier_mqtt_init();
}

static void mqtt_ready_callback(void) {
    mqtt_ready = true;
}