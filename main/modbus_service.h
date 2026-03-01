#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float humidity;
    float temperature;
    int16_t cal_hum;
    int16_t cal_temp;
    uint8_t address;
    int baud_rate;
} thmb02s_sample_t;

typedef void (*thmb02s_sample_cb_t)(const thmb02s_sample_t *sample, void *ctx);

esp_err_t thmb02s_service_start(thmb02s_sample_cb_t cb, void *ctx);

#ifdef __cplusplus
}
#endif
