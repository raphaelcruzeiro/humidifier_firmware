#ifndef HUMIDIFIER_CONTROL
#define HUMIDIFIER_CONTROL

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdbool.h>

typedef struct {
    bool power;
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
esp_err_t humidifier_control_set_mist_level(uint8_t level);
esp_err_t humidifier_control_set_warm_mist(bool enable);
esp_err_t humidifier_control_set_target_humidity(uint8_t humidity);




#endif // HUMIDIFIER_CONTROL