#ifndef WIFI_H
#define WIFI_H

#include "esp_err.h"

// Initializes the Wi-Fi subsystem with hardcoded credentials.
esp_err_t wifi_init_sta(void);

// Enables or disables raw logging of Wi-Fi debug information
void wifi_set_debug(bool enable);

#endif // WIFI_H