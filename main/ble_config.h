#pragma once

#include "app_config.h"
#include "esp_err.h"

typedef void (*ble_config_update_cb_t)(const app_config_t *cfg, void *ctx);

esp_err_t ble_config_start(app_config_t *cfg, ble_config_update_cb_t cb, void *ctx);
