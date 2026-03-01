// master.c - TH-MB-02S Modbus RTU (estilo ESPHome) con Command Queue Bus
// ESP-IDF v5.x
//
// - Un solo task (bus_task) accede UART + DE/RE => cero colisiones
// - Otros tasks envían comandos por Queue y reciben respuesta por reply_q
// - Auto-baud detect (4800/9600) al iniciar
// - Read Holding (0x03), Write Single Holding (0x06)
// - Poll task publica última lectura (aquí: log, pero podés meter MQTT)

#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "modbus_service.h"

// ===================== CONFIG =====================
static const char *TAG = "thmb02s";

// UART
#define UART_PORT       UART_NUM_2
#define UART_TX_PIN     GPIO_NUM_17
#define UART_RX_PIN     GPIO_NUM_16

// MAX485 DE + /RE unidos
#define RS485_DE_RE_PIN GPIO_NUM_4

// Modbus slave ID del sensor
#define SLAVE_ID        1

// Auto-baud candidates
static const int BAUD_CANDIDATES[] = {4800, 9600};
#define BAUD_CANDIDATES_N (sizeof(BAUD_CANDIDATES)/sizeof(BAUD_CANDIDATES[0]))

// Timings (similar a ESPHome)
#define PRE_TX_DELAY_US       1000
#define POST_TX_DELAY_US      1000
#define SEND_WAIT_TIME_MS     250     // "send_wait_time"
#define RESPONSE_TIMEOUT_MS   2000
#define MAX_RETRIES           4

// UART RX buffer
#define RX_BUF_SIZE     256

// Registros TH-MB-02S (como tu ejemplo)
#define REG_HUM_X10     0
#define REG_TEMP_X10    1
#define REG_CAL_HUM     80
#define REG_CAL_TEMP    81
#define REG_ADDR        2000
#define REG_BAUD        2001   // 0=2400, 1=4800, 2=9600

// ===================== CRC16 Modbus =====================
static uint16_t modbus_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

static inline int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

// ===================== DE/RE control =====================
// Si tu módulo requiere polaridad invertida, invertí 1/0 acá.
static inline void tx_enable(void) { gpio_set_level(RS485_DE_RE_PIN, 1); }
static inline void rx_enable(void) { gpio_set_level(RS485_DE_RE_PIN, 0); }

// ===================== Bus state =====================
typedef struct {
    int baud;   // baud actual detectado
} modbus_bus_state_t;

static modbus_bus_state_t g_bus = {.baud = 0};

static thmb02s_sample_cb_t s_sample_cb = NULL;
static void *s_sample_ctx = NULL;
static bool s_service_started = false;

// ===================== UART init helpers =====================
static esp_err_t rs485_uart_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << RS485_DE_RE_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    rx_enable();

    uart_config_t cfg = {
        .baud_rate = 4800, // se setea luego
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, RX_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_flush_input(UART_PORT));

    ESP_LOGI(TAG, "UART%d TX=%d RX=%d DE/RE GPIO=%d",
             UART_PORT, UART_TX_PIN, UART_RX_PIN, RS485_DE_RE_PIN);
    return ESP_OK;
}

static esp_err_t rs485_uart_set_baud(int baud) {
    esp_err_t err = uart_set_baudrate(UART_PORT, baud);
    if (err == ESP_OK) ESP_LOGI(TAG, "UART%d baud set to %d", UART_PORT, baud);
    return err;
}

static void rs485_flush_rx(void) {
    uart_flush_input(UART_PORT);
}

// ===================== Low-level TX/RX frame =====================
static esp_err_t rs485_send_bytes(const uint8_t *data, size_t len) {
    rs485_flush_rx();

    tx_enable();
    esp_rom_delay_us(PRE_TX_DELAY_US);

    int w = uart_write_bytes(UART_PORT, (const char*)data, len);
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(200));

    esp_rom_delay_us(POST_TX_DELAY_US);
    rx_enable();

    // estilo ESPHome
    vTaskDelay(pdMS_TO_TICKS(SEND_WAIT_TIME_MS));

    return (w == (int)len) ? ESP_OK : ESP_FAIL;
}

static esp_err_t rs485_read_frame(uint8_t *buf, size_t buf_sz, size_t *out_len, uint32_t timeout_ms) {
    *out_len = 0;
    int64_t deadline = now_ms() + timeout_ms;

    while (now_ms() < deadline && *out_len < buf_sz) {
        int r = uart_read_bytes(UART_PORT, buf + *out_len, (int)(buf_sz - *out_len), pdMS_TO_TICKS(50));
        if (r > 0) {
            *out_len += (size_t)r;

            if (*out_len >= 3) {
                uint8_t func = buf[1];
                if (func == 0x03) {
                    uint8_t bc = buf[2];
                    size_t frame_len = 3 + (size_t)bc + 2;
                    if (*out_len >= frame_len) return ESP_OK;
                } else if (func == 0x06) {
                    if (*out_len >= 8) return ESP_OK;
                } else if (func & 0x80) {
                    if (*out_len >= 5) return ESP_OK;
                }
            }
        }
    }
    return ESP_ERR_TIMEOUT;
}

// ===================== Modbus operations (sync) =====================
static esp_err_t modbus_read_holding_sync(uint8_t slave_id, uint16_t start_addr, uint16_t quantity,
                                         uint16_t *out_regs, size_t out_regs_len) {
    if (quantity == 0 || quantity > 125) return ESP_ERR_INVALID_ARG;
    if (out_regs_len < quantity) return ESP_ERR_INVALID_SIZE;

    uint8_t req[8];
    req[0] = slave_id;
    req[1] = 0x03;
    req[2] = (uint8_t)(start_addr >> 8);
    req[3] = (uint8_t)(start_addr & 0xFF);
    req[4] = (uint8_t)(quantity >> 8);
    req[5] = (uint8_t)(quantity & 0xFF);
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    uint8_t resp[256];

    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        esp_err_t err = rs485_send_bytes(req, sizeof(req));
        if (err != ESP_OK) continue;

        size_t got = 0;
        err = rs485_read_frame(resp, sizeof(resp), &got, RESPONSE_TIMEOUT_MS);
        if (err != ESP_OK) continue;

        if (got < 5 || resp[0] != slave_id) continue;

        if (resp[1] & 0x80) {
            ESP_LOGW(TAG, "0x03 exception: code=0x%02X", resp[2]);
            continue;
        }

        uint8_t bc = resp[2];
        if (bc != quantity * 2) continue;

        size_t frame_len = 3 + (size_t)bc + 2;
        if (got < frame_len) continue;

        uint16_t crc_rx = (uint16_t)resp[frame_len - 2] | ((uint16_t)resp[frame_len - 1] << 8);
        uint16_t crc_calc = modbus_crc16(resp, frame_len - 2);
        if (crc_rx != crc_calc) continue;

        for (uint16_t i = 0; i < quantity; i++) {
            size_t off = 3 + (size_t)i * 2;
            out_regs[i] = (uint16_t)resp[off] << 8 | (uint16_t)resp[off + 1];
        }
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t modbus_write_single_holding_sync(uint8_t slave_id, uint16_t reg_addr, uint16_t value) {
    uint8_t req[8];
    req[0] = slave_id;
    req[1] = 0x06;
    req[2] = (uint8_t)(reg_addr >> 8);
    req[3] = (uint8_t)(reg_addr & 0xFF);
    req[4] = (uint8_t)(value >> 8);
    req[5] = (uint8_t)(value & 0xFF);
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    uint8_t resp[32];

    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        esp_err_t err = rs485_send_bytes(req, sizeof(req));
        if (err != ESP_OK) continue;

        size_t got = 0;
        err = rs485_read_frame(resp, sizeof(resp), &got, RESPONSE_TIMEOUT_MS);
        if (err != ESP_OK) continue;

        if (got < 5 || resp[0] != slave_id) continue;

        if (resp[1] & 0x80) {
            ESP_LOGW(TAG, "0x06 exception: code=0x%02X", resp[2]);
            continue;
        }

        if (got < 8 || resp[1] != 0x06) continue;

        uint16_t crc_rx = (uint16_t)resp[6] | ((uint16_t)resp[7] << 8);
        uint16_t crc_calc = modbus_crc16(resp, 6);
        if (crc_rx != crc_calc) continue;

        if (memcmp(req, resp, 6) != 0) continue;

        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

// ===================== Command queue model =====================
typedef enum {
    MB_CMD_READ_HOLDING = 1,
    MB_CMD_WRITE_SINGLE_HOLDING = 2,
} mb_cmd_type_t;

typedef struct {
    mb_cmd_type_t type;
    uint8_t slave_id;

    // READ params
    uint16_t start_addr;
    uint16_t quantity;

    // WRITE params
    uint16_t reg_addr;
    uint16_t value;

    // Reply channel (caller provides)
    QueueHandle_t reply_q;
} mb_cmd_t;

typedef struct {
    esp_err_t err;
    mb_cmd_type_t type;

    // For READ
    uint16_t regs[16];   // suficiente para nuestro caso; ampliable
    uint16_t regs_len;   // cantidad real

    // For debug
    int baud_used;
} mb_reply_t;

static QueueHandle_t g_cmd_q;

// ===================== Bus auto-baud =====================
static esp_err_t bus_autobaud_detect(int *out_baud) {
    uint16_t regs[2];

    for (size_t i = 0; i < BAUD_CANDIDATES_N; i++) {
        int baud = BAUD_CANDIDATES[i];
        if (rs485_uart_set_baud(baud) != ESP_OK) continue;

        esp_err_t err = modbus_read_holding_sync(SLAVE_ID, REG_HUM_X10, 2, regs, 2);
        if (err == ESP_OK) {
            *out_baud = baud;
            ESP_LOGI(TAG, "Auto-baud OK: %d (hum_raw=%u temp_raw=%u)", baud, (unsigned)regs[0], (unsigned)regs[1]);
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

// ===================== bus_task (único dueño del UART) =====================
static void modbus_bus_task(void *arg) {
    // Detect baud
    int baud = 0;
    if (bus_autobaud_detect(&baud) != ESP_OK) {
        ESP_LOGE(TAG, "Auto-baud failed. Check A/B and DE/RE polarity.");
        vTaskDelete(NULL);
        return;
    }
    g_bus.baud = baud;

    mb_cmd_t cmd;
    while (1) {
        if (xQueueReceive(g_cmd_q, &cmd, portMAX_DELAY) != pdTRUE) continue;

        mb_reply_t rep = {
            .err = ESP_FAIL,
            .type = cmd.type,
            .regs_len = 0,
            .baud_used = g_bus.baud,
        };

        if (cmd.type == MB_CMD_READ_HOLDING) {
            if (cmd.quantity > (sizeof(rep.regs)/sizeof(rep.regs[0]))) {
                rep.err = ESP_ERR_INVALID_SIZE;
            } else {
                rep.err = modbus_read_holding_sync(cmd.slave_id, cmd.start_addr, cmd.quantity,
                                                  rep.regs, cmd.quantity);
                if (rep.err == ESP_OK) rep.regs_len = cmd.quantity;
            }
        } else if (cmd.type == MB_CMD_WRITE_SINGLE_HOLDING) {
            rep.err = modbus_write_single_holding_sync(cmd.slave_id, cmd.reg_addr, cmd.value);
        } else {
            rep.err = ESP_ERR_INVALID_ARG;
        }

        // responder al caller
        if (cmd.reply_q) {
            // cola de 1 elemento: overwrite
            xQueueOverwrite(cmd.reply_q, &rep);
        }
    }
}

// ===================== Public API (sync wrappers) =====================
// Estos wrappers hacen: crear reply_q (1), mandar cmd, esperar respuesta.

static esp_err_t mb_read_holding(uint16_t start_addr, uint16_t quantity, uint16_t *out_regs, size_t out_len) {
    if (out_len < quantity) return ESP_ERR_INVALID_SIZE;

    QueueHandle_t rq = xQueueCreate(1, sizeof(mb_reply_t));
    if (!rq) return ESP_ERR_NO_MEM;

    mb_cmd_t cmd = {
        .type = MB_CMD_READ_HOLDING,
        .slave_id = SLAVE_ID,
        .start_addr = start_addr,
        .quantity = quantity,
        .reply_q = rq,
    };

    if (xQueueSend(g_cmd_q, &cmd, pdMS_TO_TICKS(2000)) != pdTRUE) {
        vQueueDelete(rq);
        return ESP_ERR_TIMEOUT;
    }

    mb_reply_t rep;
    if (xQueueReceive(rq, &rep, pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS + 500)) != pdTRUE) {
        vQueueDelete(rq);
        return ESP_ERR_TIMEOUT;
    }

    vQueueDelete(rq);

    if (rep.err == ESP_OK) {
        for (uint16_t i = 0; i < rep.regs_len; i++) out_regs[i] = rep.regs[i];
    }
    return rep.err;
}

static esp_err_t mb_write_single(uint16_t reg_addr, uint16_t value) {
    QueueHandle_t rq = xQueueCreate(1, sizeof(mb_reply_t));
    if (!rq) return ESP_ERR_NO_MEM;

    mb_cmd_t cmd = {
        .type = MB_CMD_WRITE_SINGLE_HOLDING,
        .slave_id = SLAVE_ID,
        .reg_addr = reg_addr,
        .value = value,
        .reply_q = rq,
    };

    if (xQueueSend(g_cmd_q, &cmd, pdMS_TO_TICKS(2000)) != pdTRUE) {
        vQueueDelete(rq);
        return ESP_ERR_TIMEOUT;
    }

    mb_reply_t rep;
    if (xQueueReceive(rq, &rep, pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS + 500)) != pdTRUE) {
        vQueueDelete(rq);
        return ESP_ERR_TIMEOUT;
    }

    vQueueDelete(rq);
    return rep.err;
}

// ===================== Poll task (consume bus) =====================
static void thmb02s_poll_task(void *arg) {
    const TickType_t delay_ticks = pdMS_TO_TICKS(2000);
    while (1) {
        uint16_t ht[2] = {0};
        uint16_t cal[2] = {0};
        uint16_t cfg[2] = {0};

        esp_err_t e1 = mb_read_holding(REG_HUM_X10, 2, ht, 2);
        esp_err_t e2 = mb_read_holding(REG_CAL_HUM, 2, cal, 2);
        esp_err_t e3 = mb_read_holding(REG_ADDR, 2, cfg, 2);

        if (e1 == ESP_OK && e2 == ESP_OK && e3 == ESP_OK) {
            float hum = ht[0] / 10.0f;
            float tmp = ht[1] / 10.0f;
            int16_t cal_h = (int16_t)cal[0];
            int16_t cal_t = (int16_t)cal[1];
            uint8_t addr = (uint8_t)(cfg[0] & 0xFF);
            uint8_t baud_code = (uint8_t)(cfg[1] & 0xFF);

            int baud = (baud_code == 0) ? 2400 : (baud_code == 1) ? 4800 : (baud_code == 2) ? 9600 : -1;

            ESP_LOGI(TAG, "TH-MB-02S: Hum=%.1f%% Temp=%.1fC | CAL hum=%d temp=%d | addr=%u | baud=%d",
                     hum, tmp, (int)cal_h, (int)cal_t, (unsigned)addr, baud);

            if (s_sample_cb) {
                thmb02s_sample_t sample = {
                    .humidity = hum,
                    .temperature = tmp,
                    .cal_hum = cal_h,
                    .cal_temp = cal_t,
                    .address = addr,
                    .baud_rate = baud,
                };
                s_sample_cb(&sample, s_sample_ctx);
            }
        } else {
            ESP_LOGW(TAG, "Poll read failed: ht=%s cal=%s cfg=%s",
                     esp_err_to_name(e1), esp_err_to_name(e2), esp_err_to_name(e3));
        }

        vTaskDelay(delay_ticks);
    }
}

esp_err_t thmb02s_service_start(thmb02s_sample_cb_t cb, void *ctx) {
    if (s_service_started) {
        s_sample_cb = cb;
        s_sample_ctx = ctx;
        return ESP_OK;
    }

    esp_err_t err = rs485_uart_init();
    if (err != ESP_OK) {
        return err;
    }

    g_cmd_q = xQueueCreate(10, sizeof(mb_cmd_t));
    if (!g_cmd_q) {
        return ESP_ERR_NO_MEM;
    }

    TaskHandle_t bus_handle = NULL;
    TaskHandle_t poll_handle = NULL;
    BaseType_t bus_ok = xTaskCreate(modbus_bus_task, "modbus_bus", 4096, NULL, 12, &bus_handle);
    BaseType_t poll_ok = xTaskCreate(thmb02s_poll_task, "thmb02s_poll", 4096, NULL, 10, &poll_handle);

    if (bus_ok != pdPASS || poll_ok != pdPASS) {
        if (bus_handle) {
            vTaskDelete(bus_handle);
        }
        if (poll_handle) {
            vTaskDelete(poll_handle);
        }
        if (g_cmd_q) {
            vQueueDelete(g_cmd_q);
            g_cmd_q = NULL;
        }
        return ESP_ERR_NO_MEM;
    }

    s_sample_cb = cb;
    s_sample_ctx = ctx;
    s_service_started = true;
    return ESP_OK;
}
