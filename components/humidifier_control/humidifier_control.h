#ifndef HUMIDIFIER_CONTROL
#define HUMIDIFIER_CONTROL

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdbool.h>
typedef enum {
    MIST_LEVEL_LOW = 0,
    MIST_LEVEL_MEDIUM = 1,
    MIST_LEVEL_HIGH = 2
} mist_level_t;

typedef enum {
    HUMIDITY_35 = 0,
    HUMIDITY_40,
    HUMIDITY_45,
    HUMIDITY_50,
    HUMIDITY_55,
    HUMIDITY_60,
    HUMIDITY_65,
    HUMIDITY_70
} target_humidity_level_t;

typedef struct {
    bool power;
    bool auto_mode;
    uint8_t timer_hours;
    uint8_t mist_level;
    bool warm_mist;
    uint8_t target_humidity;
    bool sleep_mode;
    float temperature_celsius;
    uint8_t current_humidity;
} humidifier_status_t;

void uart_read_task(void *arg);
void humidifier_control_init(void);
void humidifier_control_handle_uart_data(const uint8_t *data, size_t len);
const humidifier_status_t *humidifier_control_get_status(void);
esp_err_t humidifier_control_set_power(bool on);
esp_err_t humidifier_control_set_timer(uint8_t hours);
esp_err_t humidifier_control_set_mist_level(mist_level_t level);
esp_err_t humidifier_control_set_warm_mist(bool enable);
esp_err_t humidifier_control_set_target_humidity(target_humidity_level_t humidity);
const char *mist_level_to_string(mist_level_t level);
const char *target_humidity_to_string(target_humidity_level_t humidity);

#endif // HUMIDIFIER_CONTROL
