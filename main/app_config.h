#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#define APP_CONFIG_VERSION 1

#define WIFI_SSID_MAX_LEN      32
#define WIFI_PASS_MAX_LEN      64
#define MQTT_URI_MAX_LEN       128
#define MQTT_USER_MAX_LEN      32
#define MQTT_PASS_MAX_LEN      32
#define MQTT_TOPIC_MAX_LEN     128

typedef struct {
    uint32_t version;
    char wifi_ssid[WIFI_SSID_MAX_LEN + 1];
    char wifi_password[WIFI_PASS_MAX_LEN + 1];
    char mqtt_uri[MQTT_URI_MAX_LEN + 1];
    char mqtt_username[MQTT_USER_MAX_LEN + 1];
    char mqtt_password[MQTT_PASS_MAX_LEN + 1];
    char mqtt_topic[MQTT_TOPIC_MAX_LEN + 1];
} app_config_t;

void app_config_set_defaults(app_config_t *cfg);
esp_err_t app_config_storage_init(void);
esp_err_t app_config_load(app_config_t *cfg);
esp_err_t app_config_save(const app_config_t *cfg);
bool app_config_is_ready(const app_config_t *cfg);
