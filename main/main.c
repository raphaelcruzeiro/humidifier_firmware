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

static void mqtt_command_callback(const char *topic, const char *payload) {
    ESP_LOGI(TAG, "📥 MQTT Command Received → Topic: %s | Payload: %s", topic, payload);

    cJSON *root = cJSON_Parse(payload);
    if (!root) {
        ESP_LOGE(TAG, "❌ Failed to parse JSON");
        return;
    }

    const cJSON *cmd = cJSON_GetObjectItem(root, "command");
    const cJSON *arg = cJSON_GetObjectItem(root, "argument");

    if (!cJSON_IsString(cmd)) {
        ESP_LOGE(TAG, "❌ Invalid command format");
        cJSON_Delete(root);
        return;
    }

    if (strcmp(cmd->valuestring, "power") == 0 && cJSON_IsBool(arg)) {
        humidifier_control_set_power(cJSON_IsTrue(arg));
    } else if (strcmp(cmd->valuestring, "auto_mode") == 0 && cJSON_IsBool(arg)) {
        humidifier_control_set_auto_mode(cJSON_IsTrue(arg));
    } else if (strcmp(cmd->valuestring, "warm_mist") == 0 && cJSON_IsBool(arg)) {
        humidifier_control_set_warm_mist(cJSON_IsTrue(arg));
    } else if (strcmp(cmd->valuestring, "mist_level") == 0 && cJSON_IsString(arg)) {
        mist_level_t level;
        if (strcmp(arg->valuestring, "low") == 0) level = MIST_LEVEL_LOW;
        else if (strcmp(arg->valuestring, "medium") == 0) level = MIST_LEVEL_MEDIUM;
        else if (strcmp(arg->valuestring, "high") == 0) level = MIST_LEVEL_HIGH;
        else level = MIST_LEVEL_OFF;
        humidifier_control_set_mist_level(level);
    } else if (strcmp(cmd->valuestring, "target_humidity") == 0 && cJSON_IsNumber(arg)) {
        target_humidity_level_t level = (target_humidity_level_t)((arg->valuedouble - 35) / 5);
        humidifier_control_set_target_humidity(level);
    } else if (strcmp(cmd->valuestring, "timer") == 0 && cJSON_IsNumber(arg)) {
        humidifier_control_set_timer((uint8_t)arg->valuedouble);
    } else {
        ESP_LOGW(TAG, "⚠️ Unknown or invalid command: %s", cmd->valuestring);
    }

    cJSON_Delete(root);
}

static void wifi_ready_callback(void) {
    humidifier_mqtt_register_ready_callback(&mqtt_ready_callback);
    humidifier_mqtt_register_command_callback(&mqtt_command_callback);
    humidifier_mqtt_init();
}

static void mqtt_ready_callback(void) {
    mqtt_ready = true;
}

