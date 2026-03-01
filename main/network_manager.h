#pragma once

#include "app_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

esp_err_t network_manager_start(app_config_t *cfg, QueueHandle_t sample_queue);
void network_manager_notify_config_changed(void);
