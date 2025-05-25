idf_component_register(SRCS "humidifier_mqtt.c"
                       INCLUDE_DIRS "."
                       REQUIRES mqtt nvs_flash
                       )