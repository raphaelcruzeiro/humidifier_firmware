// This module handles MQTT connectivity, including initialization,
// subscribing to command topics, and publishing status updates.

#ifndef HUMIDIFIER_MQTT_H
#define HUMIDIFIER_MQTT_H

#include <stddef.h>

typedef void (*mqtt_command_callback_t)(const char *payload);

/**
 * @brief Initialize the MQTT client and register the message handler.
 */
void humidifier_mqtt_init(void);

/**
 * @brief Publish the current humidifier status to the MQTT topic.
 */
void humidifier_mqtt_publish_status(const char *json_payload);

/**
 * @brief Register a callback to handle incoming MQTT command messages.
 *
 * @param callback Function that takes a topic and payload as input.
 */
void humidifier_mqtt_register_command_callback(void (*callback)(const char *topic, const char *payload));

void humidifier_mqtt_register_ready_callback(void (*callback)(void));

#endif // HUMIDIFIER_MQTT_H