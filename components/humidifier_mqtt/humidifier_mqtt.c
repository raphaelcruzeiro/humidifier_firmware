#include "humidifier_mqtt.h"
#include <esp_log.h>
#include <mqtt_client.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <nvs.h>
#include <nvs_flash.h>
#include "esp_system.h"  // for esp_random()
#include "esp_random.h"

#include "mqtt_credentials.h"

static const char *TAG = "humidifier_mqtt";
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
#define CLIENT_ID_MAX_LEN 64
static char s_client_id[CLIENT_ID_MAX_LEN] = {0};
static void (*s_command_callback)(const char *topic, const char *payload) = NULL;
static void (*s_ready_callback)(void) = NULL;

static void generate_or_load_client_id(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    nvs_handle_t nvs_handle;
    err = nvs_open("humidifier", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS");
        snprintf(s_client_id, CLIENT_ID_MAX_LEN, "humidifier_%08x", (unsigned int)esp_random());
        return;
    }
    size_t required_size = CLIENT_ID_MAX_LEN;
    err = nvs_get_str(nvs_handle, "client_id", s_client_id, &required_size);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Loaded client_id from NVS: %s", s_client_id);
    } else {
        // Not found, generate and save
        snprintf(s_client_id, CLIENT_ID_MAX_LEN, "humidifier_%08x", (unsigned int)esp_random());
        err = nvs_set_str(nvs_handle, "client_id", s_client_id);
        if (err == ESP_OK) {
            nvs_commit(nvs_handle);
            ESP_LOGI(TAG, "Generated and saved new client_id: %s", s_client_id);
        } else {
            ESP_LOGE(TAG, "Failed to save client_id to NVS");
        }
    }
    nvs_close(nvs_handle);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED: {
            char topic[128];
            snprintf(topic, sizeof(topic), "humidifier/%s/commands", s_client_id);
            int msg_id = esp_mqtt_client_subscribe(s_mqtt_client, topic, 0);
            ESP_LOGI(TAG, "MQTT connected, subscribed to %s, msg_id=%d", topic, msg_id);
            if (s_ready_callback) {
                s_ready_callback();
            }
            break;
        }
        case MQTT_EVENT_DATA: {
            char topic[128];
            snprintf(topic, sizeof(topic), "humidifier/%s/commands", s_client_id);
            // Compare topics
            if (event->topic_len == strlen(topic) &&
                strncmp(event->topic, topic, event->topic_len) == 0) {
                // Null-terminate payload
                char *payload = (char *)malloc(event->data_len + 1);
                if (payload) {
                    memcpy(payload, event->data, event->data_len);
                    payload[event->data_len] = 0;
                    if (s_command_callback) {
                        s_command_callback(topic, payload);
                    }
                    free(payload);
                }
            }
            break;
        }
        default:
            break;
    }
}

void humidifier_mqtt_init(void)
{
    generate_or_load_client_id();
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URI,
        .credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASSWORD,
        .credentials.client_id = s_client_id,
    };
    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
}

void humidifier_mqtt_register_command_callback(void (*callback)(const char *topic, const char *payload))
{
    s_command_callback = callback;
}

void humidifier_mqtt_publish_status(const char *json_payload)
{
    if (!s_mqtt_client) {
        ESP_LOGW(TAG, "MQTT client not initialized");
        return;
    }

    char topic[128];
    snprintf(topic, sizeof(topic), "humidifier/%s/status", s_client_id);
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, json_payload, 0, 1, 0);
    ESP_LOGI(TAG, "Published status to %s, msg_id=%d", topic, msg_id);
}

void humidifier_mqtt_register_ready_callback(void (*callback)(void))
{
    s_ready_callback = callback;
}