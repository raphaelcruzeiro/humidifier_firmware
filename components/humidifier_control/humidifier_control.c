#include "humidifier_control.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define UART_NUM_1 UART_NUM_1
#define TXD_PIN GPIO_NUM_18
#define RXD_PIN GPIO_NUM_19

static const char *TAG = "humidifier_control";

static humidifier_status_t s_status = {
    .power = false,
    .auto_mode = false,
    .timer_hours = 0,
    .mist_level = MIST_LEVEL_LOW,
    .warm_mist = false,
    .target_humidity = HUMIDITY_35,
    .sleep_mode = false,
    .temperature_celsius = 0.0f,
    .current_humidity = 0,
};

static bool s_ready = false;

// Helper to update s_status from a DP field
static void update_status_from_dp(uint8_t dp_id, uint8_t dp_type, uint8_t dp_len, const uint8_t *value)
{
    switch (dp_id) {
        case 0x02: // Auto mode
            if (dp_type == 0x01 && dp_len == 1) {
                s_status.auto_mode = value[0] ? true : false;
                ESP_LOGI(TAG, "Auto mode updated to %s", s_status.auto_mode ?  "ON" : "OFF");
            }
            break;
        case 0x13: // Timer
            if (dp_type == 0x04 && dp_len == 1) {
                s_status.timer_hours = value[0];
                ESP_LOGI(TAG, "Timer updated to %d hours", s_status.timer_hours);
            }
            break;
        case 0x17: // Mist Level
            if (dp_type == 0x04 && dp_len == 1) {
                s_status.mist_level = value[0] != 0xFF ? (mist_level_t)value[0] : MIST_LEVEL_LOW;
                ESP_LOGI(TAG, "Mist level updated to %s", mist_level_to_string(s_status.mist_level));
            }
            break;
        case 0x1A: // Warm Mist
            if (dp_type == 0x01 && dp_len == 1) {
                s_status.warm_mist = value[0] ? true : false;
                ESP_LOGI(TAG, "Warm mist updated to %s", s_status.warm_mist ? "ON" : "OFF");
            }
            break;
        case 0x04: // Target Humidity
            if (dp_type == 0x04 && dp_len == 1) {
                s_status.target_humidity = (target_humidity_level_t)value[0];
                ESP_LOGI(TAG, "Target humidity updated to %s%%", target_humidity_to_string(s_status.target_humidity));
            }
            break;
        case 0x16: // Sleep Mode
            if (dp_type == 0x01 && dp_len == 1) {
                s_status.sleep_mode = value[0] ? true : false;
                ESP_LOGI(TAG, "Sleep mode updated to %s", s_status.sleep_mode ? "ON" : "OFF");
            }
            break;
        case 0x0A: // Temperature
            if (dp_type == 0x02 && dp_len == 4) {
                int32_t temp_raw = (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | value[3];
                s_status.temperature_celsius = temp_raw;
                ESP_LOGI(TAG, "Temperature updated to %d °C", temp_raw);
            }
            break;
        case 0x0E: // Current Humidity
            if (dp_type == 0x02 && dp_len == 4) {
                int32_t humidity_raw = (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | value[3];
                s_status.current_humidity = humidity_raw;
                ESP_LOGI(TAG, "Humidity updated to %d %%", s_status.current_humidity);
            }
            break;
        case 0x01: // Power
            s_status.power = value[0] ? true : false;
            break;
        default:
            break;
    }
}

static void heartbeat_task(void *arg)
{
    const uint8_t heartbeat[] = {
        0x55, 0xAA, 0x03, 0x00, 0x00, 0x01, 0x01, 0x04
    };

    while (1) {
        uart_write_bytes(UART_NUM_1, (const char *)heartbeat, sizeof(heartbeat));
        ESP_LOGI(TAG, "Sent heartbeat frame");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Send every 5 seconds
    }
}

static void humidifier_control_extract_tuya_frames(const uint8_t *buf, int len)
{
    // Check for all-zero UART data and skip if so
    bool all_zero = true;
    for (int i = 0; i < len; ++i) {
        if (buf[i] != 0x00) {
            all_zero = false;
            break;
        }
    }
    if (all_zero) {
        ESP_LOGW(TAG, "All-zero UART data skipped");
        return;
    }
    int i = 0;
    while (i < len) {
        // Look for frame header
        if (buf[i] == 0x55 && buf[i + 1] == 0xAA) {
            // Tuya protocol: length is at offset 4 (big-endian: buf[i+4] << 8 | buf[i+5])
            if (i + 6 > len) {
                break;
            }
            uint16_t length = (buf[i + 4] << 8) | buf[i + 5];
            size_t frame_len = 6 + length + 1; // header (6 bytes) + payload + checksum
            if (i + frame_len <= len) {
                // Found complete frame
                humidifier_control_handle_uart_data(&buf[i], frame_len);
                i += frame_len;
                continue;
            } else {
                // Incomplete frame; skip rest
                ESP_LOGW(TAG, "Incomplete frame detected");
                break;
            }
        } else {
            ++i; // Skip until we find a 0x55 0xAA header
        }
    }
}

void uart_read_task(void *arg)
{
    uint8_t buf[128];
    while (1) {
        int len = uart_read_bytes(UART_NUM_1, buf, sizeof(buf), 100 / portTICK_PERIOD_MS);
        if (len < 0) {
            ESP_LOGE(TAG, "UART read error");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (len > 0) {
            ESP_LOG_BUFFER_HEX(TAG, buf, len);
            humidifier_control_extract_tuya_frames(buf, len);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void humidifier_control_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);

    xTaskCreate(uart_read_task, "uart_read_task", 4096, NULL, 10, NULL);

    // Send early sync frame
    uint8_t sync_frame[] = {0x00, 0x00, 0x00, 0x00, 0xFF};
    uart_write_bytes(UART_NUM_1, (const char *)sync_frame, sizeof(sync_frame));
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds

    // Simulate key DP initialization frames
    static const uint8_t startup_frames[][12] = {
        {0x55, 0xAA, 0x03, 0x07, 0x00, 0x05, 0x01, 0x01, 0x00, 0x01, 0x00, 0x11},
        {0x55, 0xAA, 0x03, 0x07, 0x00, 0x05, 0x13, 0x04, 0x00, 0x01, 0x00, 0x26},
        {0x55, 0xAA, 0x03, 0x07, 0x00, 0x05, 0x16, 0x05, 0x00, 0x01, 0x00, 0x2A},
        {0x55, 0xAA, 0x03, 0x07, 0x00, 0x05, 0x1A, 0x01, 0x00, 0x01, 0x00, 0x2A},
    };
    for (int i = 0; i < sizeof(startup_frames) / sizeof(startup_frames[0]); i++) {
        uart_write_bytes(UART_NUM_1, (const char *)startup_frames[i], 12);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Send status query frame to trigger DP report from MCU
    const uint8_t status_query[] = {0x55, 0xAA, 0x00, 0x08, 0x00, 0x00, 0x07};
    uart_write_bytes(UART_NUM_1, (const char *)status_query, sizeof(status_query));
    ESP_LOGI(TAG, "Sent status query frame");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for MCU to respond

    s_ready = true; // Assume MCU is now ready to accept commands

    for (int i = 0; i < 10; ++i) {
        const uint8_t heartbeat[] = {0x55, 0xAA, 0x03, 0x00, 0x00, 0x01, 0x01, 0x04};
        uart_write_bytes(UART_NUM_1, (const char *)heartbeat, sizeof(heartbeat));
        ESP_LOGI(TAG, "Sent heartbeat frame (init loop)");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    xTaskCreate(heartbeat_task, "heartbeat_task", 2048, NULL, 5, NULL);
}

void humidifier_control_handle_uart_data(const uint8_t *data, size_t len)
{
    ESP_LOG_BUFFER_HEX(TAG, data, len);
    if (len < 4) {
        ESP_LOGW(TAG, "Data too short");
        return;
    }
    // Look for frame starting with 0x55 0xAA
    if (data[0] != 0x55 || data[1] != 0xAA) {
        ESP_LOGW(TAG, "Invalid frame header");
        return;
    }

    uint8_t cmd = data[2];
    if (cmd == 0x03) {
        ESP_LOGI(TAG, "Raw status update frame:");
        ESP_LOG_BUFFER_HEX(TAG, data, len);
        ESP_LOGI(TAG, "📨 Handling incoming status report (MCU → Wi-Fi)");
        // Parse and log all DP fields in a friendly format using correct Tuya DP parsing
        size_t pos = 6; // Start at byte 6: first DP_ID
        while (pos + 3 < len) {
            uint8_t dp_id = data[pos];
            uint8_t dp_type = data[pos + 1];
            uint16_t dp_len = (data[pos + 2] << 8) | data[pos + 3];
            const uint8_t *value = &data[pos + 4];
            if (pos + 4 + dp_len > len) {
                ESP_LOGW(TAG, "⚠️ DP length exceeds buffer");
                break;
            }

            // Log and update based on DP ID
            update_status_from_dp(dp_id, dp_type, dp_len, value);

            pos += 4 + dp_len;
        }
        ESP_LOGI(TAG, "🧾 Summary → Power: %s | 🤖 Auto mode: %s | Mist Level: %s | Warm Mist: %s | Target Humidity: %s%% | Timer: %dh | Temp: %d°C | RH: %u%%",
                 s_status.power ? "ON" : "OFF",
                 s_status.auto_mode ? "ON" : "OFF",
                 mist_level_to_string(s_status.mist_level),
                 s_status.warm_mist ? "ON" : "OFF",
                 target_humidity_to_string(s_status.target_humidity),
                 s_status.timer_hours,
                 (int)s_status.temperature_celsius,
                 s_status.current_humidity);
        ESP_LOGI(TAG, "📡 End of MCU status report");
    }
    if (cmd == 0x07) { // Received DP command
        ESP_LOGI(TAG, "✅ Entered DP frame handler");
        ESP_LOGI(TAG, "📡 Received status report from MCU");
        s_ready = true;
        ESP_LOGI(TAG, "🔛 Marking MCU as ready to receive commands");
        ESP_LOGI(TAG, "MCU marked as ready to receive commands");
        // Parse DP fields (start at offset 4)
        size_t pos = 0;
        while (pos + 7 <= len) {
            if (pos + 7 > len) break;
            uint8_t dp_id = data[pos + 4];
            uint8_t dp_type = data[pos + 5];
            uint8_t dp_len = data[pos + 6];
            const uint8_t *value = &data[pos + 7];
            if (pos + 7 + dp_len > len) {
                ESP_LOGW(TAG, "DP length exceeds buffer");
                break;
            }
            update_status_from_dp(dp_id, dp_type, dp_len, value);
            pos += 7 + dp_len;
        }
        ESP_LOGI(TAG, "📝 Current Status → Power: %s | Timer: %dh | Mist Level: %s | Warm Mist: %s | Target Humidity: %s%% | Temp: %d°C | Sleep Mode: %s",
                 s_status.power ? "ON" : "OFF",
                 s_status.timer_hours,
                 mist_level_to_string(s_status.mist_level),
                 s_status.warm_mist ? "ON" : "OFF",
                 target_humidity_to_string(s_status.target_humidity),
                 (int)s_status.temperature_celsius,
                 s_status.sleep_mode ? "ON" : "OFF");
        ESP_LOGI(TAG, "📦 Status report handled");
        ESP_LOGI(TAG, "✅ End of DP frame processing");
    } else {
        ESP_LOGI(TAG, "ℹ️ Ignored frame with cmd=0x%02X", cmd);
    }
}

const humidifier_status_t *humidifier_control_get_status(void)
{
    return &s_status;
}

esp_err_t humidifier_control_set_power(bool on)
{
    if (!s_ready) {
        ESP_LOGW(TAG, "Command ignored: MCU not ready yet");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[12];
    // Construct Tuya frame matching app behavior
    buf[0] = 0x55;
    buf[1] = 0xAA;
    buf[2] = 0x00; // Message type: app → MCU
    buf[3] = 0x06; // Actual DP command
    buf[4] = 0x00;
    buf[5] = 0x05;
    buf[6] = 0x01; // DP_ID power
    buf[7] = 0x01; // DP_TYPE bool
    buf[8] = 0x00; // Reserved
    buf[9] = 0x01; // DP_LEN
    buf[10] = on ? 0x01 : 0x00;

    uint8_t checksum = 0;
    for (int i = 0; i < 11; ++i) {
        checksum += buf[i];
    }
    buf[11] = checksum;

    ESP_LOGI(TAG, "Sending power command: %s", on ? "ON" : "OFF");
    ESP_LOG_BUFFER_HEX(TAG, buf, 12);

    s_status.power = on;

    uart_write_bytes(UART_NUM_1, (const char *)buf, 12);

    return ESP_OK;
}

esp_err_t humidifier_control_set_timer(uint8_t hours)
{
    if (!s_ready) {
        ESP_LOGW(TAG, "Command ignored: MCU not ready yet");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[9];
    // Construct Tuya frame:
    // CMD = 0x06
    // DP_ID = 0x13 (timer), DP_TYPE = 0x04, DP_LEN = 0x01, DP_VALUE = hours
    buf[0] = 0x55;
    buf[1] = 0xAA;
    buf[2] = 0x06;
    buf[3] = 0x05; // LEN: 4 bytes of DP data
    buf[4] = 0x13; // DP_ID timer
    buf[5] = 0x04; // DP_TYPE uint8
    buf[6] = 0x01; // DP_LEN
    buf[7] = hours;

    uint8_t checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum += buf[i];
    }
    buf[8] = checksum;

    ESP_LOGI(TAG, "Sending timer command: %d hours", hours);
    ESP_LOG_BUFFER_HEX(TAG, buf, 9);

    s_status.timer_hours = hours;

    uart_write_bytes(UART_NUM_1, (const char *)buf, 9);

    return ESP_OK;
}

esp_err_t humidifier_control_set_mist_level(mist_level_t level)
{
    if (!s_ready) {
        ESP_LOGW(TAG, "Command ignored: MCU not ready yet");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[9];
    // Construct Tuya frame for mist level:
    // CMD = 0x06
    // DP_ID = 0x10, DP_TYPE = 0x04, DP_LEN = 0x01, DP_VALUE = level
    buf[0] = 0x55;
    buf[1] = 0xAA;
    buf[2] = 0x06;
    buf[3] = 0x05;
    buf[4] = 0x10; // DP_ID mist_level
    buf[5] = 0x04; // DP_TYPE uint8
    buf[6] = 0x01; // DP_LEN
    buf[7] = level;

    uint8_t checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum += buf[i];
    }
    buf[8] = checksum;

    ESP_LOGI(TAG, "Sending mist level command: %u", level);
    ESP_LOG_BUFFER_HEX(TAG, buf, 9);

    s_status.mist_level = level;
    uart_write_bytes(UART_NUM_1, (const char *)buf, 9);
    return ESP_OK;
}

esp_err_t humidifier_control_set_warm_mist(bool enable)
{
    if (!s_ready) {
        ESP_LOGW(TAG, "Command ignored: MCU not ready yet");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[9];
    // CMD = 0x06
    // DP_ID = 0x14, DP_TYPE = 0x01, DP_LEN = 0x01, DP_VALUE = enable
    buf[0] = 0x55;
    buf[1] = 0xAA;
    buf[2] = 0x06;
    buf[3] = 0x05;
    buf[4] = 0x14; // DP_ID warm_mist
    buf[5] = 0x01; // DP_TYPE bool
    buf[6] = 0x01; // DP_LEN
    buf[7] = enable ? 1 : 0;

    uint8_t checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum += buf[i];
    }
    buf[8] = checksum;

    ESP_LOGI(TAG, "Sending warm mist command: %s", enable ? "ON" : "OFF");
    ESP_LOG_BUFFER_HEX(TAG, buf, 9);

    s_status.warm_mist = enable;
    uart_write_bytes(UART_NUM_1, (const char *)buf, 9);
    return ESP_OK;
}

esp_err_t humidifier_control_set_target_humidity(target_humidity_level_t humidity)
{
    if (!s_ready) {
        ESP_LOGW(TAG, "Command ignored: MCU not ready yet");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[9];
    // CMD = 0x06
    // DP_ID = 0x15, DP_TYPE = 0x04, DP_LEN = 0x01, DP_VALUE = humidity
    buf[0] = 0x55;
    buf[1] = 0xAA;
    buf[2] = 0x06;
    buf[3] = 0x05;
    buf[4] = 0x15; // DP_ID target_humidity
    buf[5] = 0x04; // DP_TYPE uint8
    buf[6] = 0x01; // DP_LEN
    buf[7] = humidity;

    uint8_t checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum += buf[i];
    }
    buf[8] = checksum;

    ESP_LOGI(TAG, "Sending target humidity command: %u", humidity);
    ESP_LOG_BUFFER_HEX(TAG, buf, 9);

    s_status.target_humidity = humidity;
    uart_write_bytes(UART_NUM_1, (const char *)buf, 9);
    return ESP_OK;
}


esp_err_t humidifier_control_set_sleep_mode(bool enable)
{
    if (!s_ready) {
        ESP_LOGW(TAG, "Command ignored: MCU not ready yet");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[9];
    // CMD = 0x06
    // DP_ID = 0x16, DP_TYPE = 0x01, DP_LEN = 0x01, DP_VALUE = enable
    buf[0] = 0x55;
    buf[1] = 0xAA;
    buf[2] = 0x06;
    buf[3] = 0x05;
    buf[4] = 0x16; // DP_ID sleep_mode
    buf[5] = 0x01; // DP_TYPE bool
    buf[6] = 0x01; // DP_LEN
    buf[7] = enable ? 1 : 0;

    uint8_t checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum += buf[i];
    }
    buf[8] = checksum;

    ESP_LOGI(TAG, "Sending sleep mode command: %s", enable ? "ON" : "OFF");
    ESP_LOG_BUFFER_HEX(TAG, buf, 9);

    s_status.sleep_mode = enable;
    uart_write_bytes(UART_NUM_1, (const char *)buf, 9);
    return ESP_OK;
}

const char *mist_level_to_string(mist_level_t level) {
    switch (level) {
        case MIST_LEVEL_LOW: return "low";
        case MIST_LEVEL_MEDIUM: return "medium";
        case MIST_LEVEL_HIGH: return "high";
        default: return "unknown";
    }
}

const char *target_humidity_to_string(target_humidity_level_t humidity) {
    switch (humidity) {
        case HUMIDITY_35: return "35";
        case HUMIDITY_40: return "40";
        case HUMIDITY_45: return "45";
        case HUMIDITY_50: return "50";
        case HUMIDITY_55: return "55";
        case HUMIDITY_60: return "60";
        case HUMIDITY_65: return "65";
        case HUMIDITY_70: return "70";
        default: return "unknown";
    }
}