#include "app_config.h"
#include "ble_config.h"
#include "modbus_service.h"
#include "network_manager.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "app_main";
static app_config_t s_app_cfg;
static QueueHandle_t s_sample_queue = NULL;

static void sensor_sample_cb(const thmb02s_sample_t *sample, void *ctx) {
    QueueHandle_t q = (QueueHandle_t)ctx;
    if (!sample || !q) {
        return;
    }
    if (xQueueOverwrite(q, sample) != pdPASS) {
        ESP_LOGW(TAG, "Cola de muestras llena, descartando valor");
    }
}

static void config_saved_cb(const app_config_t *cfg, void *ctx) {
    (void)cfg;
    (void)ctx;
    ESP_LOGI(TAG, "Configuración BLE actualizada, notificando network manager");
    network_manager_notify_config_changed();
}

void app_main(void) {
    ESP_ERROR_CHECK(app_config_storage_init());
    app_config_set_defaults(&s_app_cfg);
    esp_err_t cfg_err = app_config_load(&s_app_cfg);
    if (cfg_err != ESP_OK) {
        ESP_LOGW(TAG, "Usando configuración por defecto (%s)", esp_err_to_name(cfg_err));
    }

    s_sample_queue = xQueueCreate(1, sizeof(thmb02s_sample_t));
    configASSERT(s_sample_queue);

    ESP_ERROR_CHECK(thmb02s_service_start(sensor_sample_cb, s_sample_queue));
    ESP_ERROR_CHECK(ble_config_start(&s_app_cfg, config_saved_cb, NULL));
    ESP_ERROR_CHECK(network_manager_start(&s_app_cfg, s_sample_queue));

    if (!app_config_is_ready(&s_app_cfg)) {
        ESP_LOGW(TAG, "Esperando credenciales via BLE antes de conectar WiFi/MQTT");
    }
}
