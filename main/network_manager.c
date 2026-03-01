#include "network_manager.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "modbus_service.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <sys/time.h>
#include <time.h>

static const char *TAG = "net_mgr";

#define WIFI_CONNECTED_BIT BIT0
#define MQTT_CONNECTED_BIT BIT1

static app_config_t *s_cfg = NULL;
static QueueHandle_t s_sample_queue = NULL;
static EventGroupHandle_t s_net_events = NULL;
static bool s_netif_ready = false;
static bool s_wifi_started = false;
static volatile bool s_config_dirty = true;
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static bool s_mqtt_connected = false;
static bool s_sntp_initialized = false;
static bool s_time_synced = false;
static bool s_warned_unsynced = false;

static void ensure_sntp_initialized(void);
static void trigger_time_resync(void);
static bool acquire_timestamp_ms(int64_t *ts_out);

static void time_sync_notification_cb(struct timeval *tv) {
    (void)tv;
    s_time_synced = true;
    s_warned_unsynced = false;
    ESP_LOGI(TAG, "Hora sincronizada con servidor SNTP");
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            xEventGroupClearBits(s_net_events, WIFI_CONNECTED_BIT);
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_net_events, WIFI_CONNECTED_BIT);
        trigger_time_resync();
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            s_mqtt_connected = true;
            xEventGroupSetBits(s_net_events, MQTT_CONNECTED_BIT);
            ESP_LOGI(TAG, "MQTT connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            s_mqtt_connected = false;
            xEventGroupClearBits(s_net_events, MQTT_CONNECTED_BIT);
            ESP_LOGW(TAG, "MQTT disconnected");
            break;
        default:
            break;
    }
}

static void stop_mqtt(void) {
    if (s_mqtt_client) {
        esp_mqtt_client_stop(s_mqtt_client);
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
        s_mqtt_connected = false;
        xEventGroupClearBits(s_net_events, MQTT_CONNECTED_BIT);
    }
}

static void init_network_stack(void) {
    if (s_netif_ready) {
        return;
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    s_netif_ready = true;
}

static void restart_mqtt_client(void) {
    stop_mqtt();

    if (s_cfg->mqtt_uri[0] == '\0') {
        ESP_LOGW(TAG, "MQTT URI vacío, omitiendo conexión");
        return;
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = s_cfg->mqtt_uri,
        .credentials.username = s_cfg->mqtt_username[0] ? s_cfg->mqtt_username : NULL,
        .credentials.authentication.password = s_cfg->mqtt_password[0] ? s_cfg->mqtt_password : NULL,
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_mqtt_client) {
        ESP_LOGE(TAG, "No se pudo crear el cliente MQTT");
        return;
    }
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
}

static void stop_wifi(void) {
    if (s_wifi_started) {
        esp_wifi_disconnect();
        esp_wifi_stop();
        s_wifi_started = false;
        xEventGroupClearBits(s_net_events, WIFI_CONNECTED_BIT);
    }
}

static void apply_network_config(void) {
    if (!app_config_is_ready(s_cfg)) {
        ESP_LOGW(TAG, "Configuración incompleta. Esperando ajustes vía BLE.");
        stop_mqtt();
        stop_wifi();
        return;
    }

    init_network_stack();
    ensure_sntp_initialized();

    wifi_config_t wifi_config = {0};
    strlcpy((char *)wifi_config.sta.ssid, s_cfg->wifi_ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, s_cfg->wifi_password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    if (!s_wifi_started) {
        ESP_ERROR_CHECK(esp_wifi_start());
        s_wifi_started = true;
    } else {
        esp_wifi_disconnect();
    }
    ESP_ERROR_CHECK(esp_wifi_connect());

    restart_mqtt_client();
}

static void publish_sample(const thmb02s_sample_t *sample) {
    if (!s_mqtt_client || !s_mqtt_connected || s_cfg->mqtt_topic[0] == '\0') {
        return;
    }

    int64_t timestamp_ms = 0;
    if (!acquire_timestamp_ms(&timestamp_ms) && !s_warned_unsynced) {
        s_warned_unsynced = true;
        ESP_LOGW(TAG, "Timestamp todavía no sincronizado; enviando valor 0");
    }

    char payload[128];
    int len = snprintf(payload, sizeof(payload),
                       "{\"temperature\":%.1f,\"humidity\":%.1f,\"addr\":%u,\"timestamp\":%" PRId64 "}",
                       sample->temperature, sample->humidity, sample->address, timestamp_ms);
    if (len <= 0) {
        return;
    }

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, s_cfg->mqtt_topic, payload, len, 1, 0);
    if (msg_id < 0) {
        ESP_LOGW(TAG, "Falló publish MQTT");
    }
}

static void network_manager_task(void *arg) {
    for (;;) {
        if (s_config_dirty) {
            apply_network_config();
            s_config_dirty = false;
        }

        thmb02s_sample_t sample;
        if (s_sample_queue &&
            xQueueReceive(s_sample_queue, &sample, pdMS_TO_TICKS(1000)) == pdTRUE) {
            publish_sample(&sample);
        }
    }
}

esp_err_t network_manager_start(app_config_t *cfg, QueueHandle_t sample_queue) {
    if (!cfg || !sample_queue) {
        return ESP_ERR_INVALID_ARG;
    }

    s_cfg = cfg;
    s_sample_queue = sample_queue;
    s_net_events = xEventGroupCreate();
    if (!s_net_events) {
        return ESP_ERR_NO_MEM;
    }

    s_config_dirty = true;

    BaseType_t ok = xTaskCreate(network_manager_task, "net_mgr", 4096, NULL, 8, NULL);
    if (ok != pdPASS) {
        vEventGroupDelete(s_net_events);
        s_net_events = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void network_manager_notify_config_changed(void) {
    s_config_dirty = true;
}

static void ensure_sntp_initialized(void) {
    if (s_sntp_initialized) {
        return;
    }

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    config.start = false;  // arranca manualmente al obtener IP
    config.sync_cb = time_sync_notification_cb;

    esp_err_t err = esp_netif_sntp_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar SNTP (%s)", esp_err_to_name(err));
        return;
    }

    s_sntp_initialized = true;
    ESP_LOGI(TAG, "SNTP inicializado (pool.ntp.org)");
}

static void trigger_time_resync(void) {
    if (!s_sntp_initialized) {
        return;
    }

    s_time_synced = false;
    esp_err_t err = esp_netif_sntp_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo iniciar/resincronizar SNTP (%s)", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Solicitando resincronización SNTP");
    }
}

static bool acquire_timestamp_ms(int64_t *ts_out) {
    if (!ts_out) {
        return false;
    }

    struct timeval tv = {0};
    if (gettimeofday(&tv, NULL) != 0) {
        return false;
    }

    // Considera válida la hora solo si SNTP reportó sincronización o el epoch supera 2021-01-01.
    if (!s_time_synced && tv.tv_sec < 1609459200) {
        *ts_out = 0;
        return false;
    }

    *ts_out = ((int64_t)tv.tv_sec * 1000LL) + (tv.tv_usec / 1000);
    return true;
}
