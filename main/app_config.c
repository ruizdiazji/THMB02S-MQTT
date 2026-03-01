#include "app_config.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "app_cfg";
static const char *NVS_NAMESPACE = "thmb02s";
static const char *NVS_KEY = "cfg";

void app_config_set_defaults(app_config_t *cfg) {
    memset(cfg, 0, sizeof(*cfg));
    cfg->version = APP_CONFIG_VERSION;
    strncpy(cfg->mqtt_topic, "thmb02s/sensors", MQTT_TOPIC_MAX_LEN);
    strncpy(cfg->mqtt_uri, "mqtt://test.mosquitto.org", MQTT_URI_MAX_LEN);
}

esp_err_t app_config_storage_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t app_config_load(app_config_t *cfg) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        app_config_set_defaults(cfg);
        return err;
    }

    size_t len = sizeof(*cfg);
    err = nvs_get_blob(nvs, NVS_KEY, cfg, &len);
    nvs_close(nvs);

    if (err == ESP_OK && cfg->version == APP_CONFIG_VERSION) {
        return ESP_OK;
    }

    app_config_set_defaults(cfg);
    return ESP_ERR_INVALID_VERSION;
}

esp_err_t app_config_save(const app_config_t *cfg) {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(nvs, NVS_KEY, cfg, sizeof(*cfg));
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }

    nvs_close(nvs);
    return err;
}

bool app_config_is_ready(const app_config_t *cfg) {
    return cfg->wifi_ssid[0] != '\0' && cfg->wifi_password[0] != '\0' &&
           cfg->mqtt_uri[0] != '\0' && cfg->mqtt_topic[0] != '\0';
}
