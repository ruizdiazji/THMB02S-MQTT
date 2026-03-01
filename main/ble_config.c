#include "ble_config.h"

#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/util/util.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CFG_FIELD_WIFI_SSID = 1,
    CFG_FIELD_WIFI_PASS,
    CFG_FIELD_MQTT_URI,
    CFG_FIELD_MQTT_USER,
    CFG_FIELD_MQTT_PASS,
    CFG_FIELD_MQTT_TOPIC,
} cfg_field_t;

static const char *TAG = "ble_cfg";

static app_config_t *s_cfg = NULL;
static ble_config_update_cb_t s_update_cb = NULL;
static void *s_update_ctx = NULL;
static bool s_ble_started = false;

typedef struct {
    char *ptr;
    size_t max_len;
} field_binding_t;

static const ble_uuid128_t service_uuid =
    BLE_UUID128_INIT(0xf0, 0xb1, 0x90, 0x2a, 0x7c, 0x47, 0x4c, 0x34, 0x84, 0xb3, 0x75, 0x5c, 0x10, 0x0f, 0x40, 0x01);
static const ble_uuid128_t wifi_ssid_uuid =
    BLE_UUID128_INIT(0xf1, 0xb1, 0x90, 0x2a, 0x7c, 0x47, 0x4c, 0x34, 0x84, 0xb3, 0x75, 0x5c, 0x10, 0x0f, 0x40, 0x01);
static const ble_uuid128_t wifi_pass_uuid =
    BLE_UUID128_INIT(0xf2, 0xb1, 0x90, 0x2a, 0x7c, 0x47, 0x4c, 0x34, 0x84, 0xb3, 0x75, 0x5c, 0x10, 0x0f, 0x40, 0x01);
static const ble_uuid128_t mqtt_uri_uuid =
    BLE_UUID128_INIT(0xf3, 0xb1, 0x90, 0x2a, 0x7c, 0x47, 0x4c, 0x34, 0x84, 0xb3, 0x75, 0x5c, 0x10, 0x0f, 0x40, 0x01);
static const ble_uuid128_t mqtt_user_uuid =
    BLE_UUID128_INIT(0xf4, 0xb1, 0x90, 0x2a, 0x7c, 0x47, 0x4c, 0x34, 0x84, 0xb3, 0x75, 0x5c, 0x10, 0x0f, 0x40, 0x01);
static const ble_uuid128_t mqtt_pass_uuid =
    BLE_UUID128_INIT(0xf5, 0xb1, 0x90, 0x2a, 0x7c, 0x47, 0x4c, 0x34, 0x84, 0xb3, 0x75, 0x5c, 0x10, 0x0f, 0x40, 0x01);
static const ble_uuid128_t mqtt_topic_uuid =
    BLE_UUID128_INIT(0xf6, 0xb1, 0x90, 0x2a, 0x7c, 0x47, 0x4c, 0x34, 0x84, 0xb3, 0x75, 0x5c, 0x10, 0x0f, 0x40, 0x01);

static void ble_config_advertise(void);

static int gatt_access_handler(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

static struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &wifi_ssid_uuid.u,
                .access_cb = gatt_access_handler,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .arg = (void *)(intptr_t)CFG_FIELD_WIFI_SSID,
            },
            {
                .uuid = &wifi_pass_uuid.u,
                .access_cb = gatt_access_handler,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .arg = (void *)(intptr_t)CFG_FIELD_WIFI_PASS,
            },
            {
                .uuid = &mqtt_uri_uuid.u,
                .access_cb = gatt_access_handler,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .arg = (void *)(intptr_t)CFG_FIELD_MQTT_URI,
            },
            {
                .uuid = &mqtt_user_uuid.u,
                .access_cb = gatt_access_handler,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .arg = (void *)(intptr_t)CFG_FIELD_MQTT_USER,
            },
            {
                .uuid = &mqtt_pass_uuid.u,
                .access_cb = gatt_access_handler,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .arg = (void *)(intptr_t)CFG_FIELD_MQTT_PASS,
            },
            {
                .uuid = &mqtt_topic_uuid.u,
                .access_cb = gatt_access_handler,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .arg = (void *)(intptr_t)CFG_FIELD_MQTT_TOPIC,
            },
            {0},
        },
    },
    {0},
};

static field_binding_t get_field_binding(cfg_field_t field) {
    switch (field) {
        case CFG_FIELD_WIFI_SSID: return (field_binding_t){ s_cfg->wifi_ssid, WIFI_SSID_MAX_LEN };
        case CFG_FIELD_WIFI_PASS: return (field_binding_t){ s_cfg->wifi_password, WIFI_PASS_MAX_LEN };
        case CFG_FIELD_MQTT_URI: return (field_binding_t){ s_cfg->mqtt_uri, MQTT_URI_MAX_LEN };
        case CFG_FIELD_MQTT_USER: return (field_binding_t){ s_cfg->mqtt_username, MQTT_USER_MAX_LEN };
        case CFG_FIELD_MQTT_PASS: return (field_binding_t){ s_cfg->mqtt_password, MQTT_PASS_MAX_LEN };
        case CFG_FIELD_MQTT_TOPIC: return (field_binding_t){ s_cfg->mqtt_topic, MQTT_TOPIC_MAX_LEN };
        default: return (field_binding_t){ NULL, 0 };
    }
}

static int gatt_access_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    cfg_field_t field = (cfg_field_t)(intptr_t)arg;
    if (!s_cfg) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    const field_binding_t ref = get_field_binding(field);
    if (!ref.ptr) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        size_t len = strnlen(ref.ptr, ref.max_len);
        return os_mbuf_append(ctxt->om, ref.ptr, len);
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t total = OS_MBUF_PKTLEN(ctxt->om);
        size_t len = total;
        if (len > ref.max_len) {
            len = ref.max_len;
        }
        char buffer[MQTT_TOPIC_MAX_LEN + 1] = {0};
        os_mbuf_copydata(ctxt->om, 0, len, buffer);
        buffer[len] = '\0';

        memset(ref.ptr, 0, ref.max_len + 1);
        memcpy(ref.ptr, buffer, len);

        esp_err_t err = app_config_save(s_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error guardando configuración: %s", esp_err_to_name(err));
        }
        if (s_update_cb) {
            s_update_cb(s_cfg, s_update_ctx);
        }
        ESP_LOGI(TAG, "Campo %d actualizado via BLE", field);
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static int ble_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status != 0) {
                ble_config_advertise();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ble_config_advertise();
            break;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ble_config_advertise();
            break;
        default:
            break;
    }
    return 0;
}

static void ble_config_advertise(void) {
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };

    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"THMB02S Config";
    fields.name_len = strlen("THMB02S Config");
    fields.name_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error al configurar advertising: %d", rc);
        return;
    }

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, ble_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error al iniciar advertising: %d", rc);
    }
}

static void ble_on_sync(void) {
    ble_addr_t addr;
    ble_hs_id_infer_auto(0, &addr.type);
    ble_hs_id_copy_addr(addr.type, addr.val, NULL);
    ble_config_advertise();
}

static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_config_start(app_config_t *cfg, ble_config_update_cb_t cb, void *ctx) {
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ble_started) {
        s_cfg = cfg;
        s_update_cb = cb;
        s_update_ctx = ctx;
        return ESP_OK;
    }

    s_cfg = cfg;
    s_update_cb = cb;
    s_update_ctx = ctx;

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_svc_gap_device_name_set("THMB02S Config");

    nimble_port_freertos_init(ble_host_task);
    s_ble_started = true;
    return ESP_OK;
}
