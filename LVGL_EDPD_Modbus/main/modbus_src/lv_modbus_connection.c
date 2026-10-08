/**
 * @file lv_modbus_connection.c
 *
 * Modbus RTU Master over UART.
 * 외부에서 modbus_send_request()로 요청 파라미터를 넘기면
 * 내부 태스크가 UART 프레임 전송 → 응답 대기 → CRC 검증 → 파싱 후
 * modbus_chart_queue에 결과를 push한다.
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_modbus_connection.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <string.h>

/*********************
 *      DEFINES
 *********************/
#define TAG                  "MODBUS"
#define MODBUS_RESP_TIMEOUT  pdMS_TO_TICKS(300)  /* 슬레이브 응답 대기 최대 시간 */
#define MODBUS_MIN_FRAME_LEN 5

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void     uart_modbus_init(void);
static void     modbus_master_task(void *pvParameter);
static void     build_read_request(const modbus_request_t *req, uint8_t *frame_out);
static uint16_t modbus_crc16(const uint8_t *buf, uint16_t len);
static bool     parse_modbus_response(const uint8_t *buf, int len,
                                      const modbus_request_t *req,
                                      modbus_data_t *out);

/**********************
 *  GLOBAL VARIABLES
 **********************/
QueueHandle_t modbus_chart_queue = NULL;

/**********************
 *  STATIC VARIABLES
 **********************/
static QueueHandle_t s_req_queue = NULL;

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void modbus_connection_init(void)
{
    esp_log_level_set("MODBUS", ESP_LOG_DEBUG);
	esp_log_level_set("MODBUS_UI", ESP_LOG_DEBUG);
    ESP_LOGI(TAG, "modbus_connection_init called");
    modbus_chart_queue = xQueueCreate(MODBUS_QUEUE_SIZE, sizeof(modbus_data_t));
    s_req_queue        = xQueueCreate(2, sizeof(modbus_batch_t));
    uart_modbus_init();
    xTaskCreatePinnedToCore(modbus_master_task, "modbus_master", 4096, NULL, 5, NULL, 0);
}

void modbus_send_request(uint8_t slave_id, uint16_t reg_addr, uint16_t reg_count)
{
    modbus_send_read_request(slave_id, 0x03, reg_addr, reg_count);
}

void modbus_send_read_request(uint8_t slave_id, uint8_t function_code,
                              uint16_t reg_addr, uint16_t reg_count)
{
    modbus_send_tagged_read_request(slave_id, function_code, reg_addr, reg_count, 0);
}

void modbus_send_tagged_read_request(uint8_t slave_id, uint8_t function_code,
                                    uint16_t reg_addr, uint16_t reg_count, uint32_t request_id)
{
    modbus_send_read_request_to(modbus_chart_queue, slave_id, function_code,
                                reg_addr, reg_count, request_id);
}

void modbus_send_read_request_to(QueueHandle_t response_queue, uint8_t slave_id,
                                 uint8_t function_code, uint16_t reg_addr,
                                 uint16_t reg_count, uint32_t request_id)
{
    modbus_request_t req = {
        .response_queue = response_queue,
        .request_id = request_id,
        .slave_id  = slave_id,
        .function_code = function_code,
        .reg_addr  = reg_addr,
        .reg_count = reg_count,
    };
    modbus_send_batch(&req, 1);
}

bool modbus_send_batch(const modbus_request_t *requests, uint8_t count)
{
    return modbus_send_batch_at_baud(requests, count, MODBUS_UART_BAUD);
}

bool modbus_send_batch_at_baud(const modbus_request_t *requests, uint8_t count, uint32_t baud_rate)
{
    switch (baud_rate) {
        case 9600: case 19200: case 38400: case 57600:
        case 115200: case 230400: case 460800: case 921600: break;
        default: return false;
    }
    if (!s_req_queue || !requests || count == 0 || count > MODBUS_MAX_BATCH_REQUESTS) return false;
    modbus_batch_t batch = {.count = count, .baud_rate = baud_rate};
    for (unsigned i = 0; i < count; i++) {
        const modbus_request_t *req = &requests[i];
        if (!req->response_queue || req->function_code < 1 || req->function_code > 4 ||
            req->reg_count == 0 || req->reg_count > MODBUS_MAX_REGS ||
            (uint32_t)req->reg_addr + req->reg_count > 65536U) return false;
        batch.requests[i] = *req;
    }
    return xQueueSend(s_req_queue, &batch, 0) == pdTRUE;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void uart_modbus_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = MODBUS_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
	esp_err_t e1 = uart_param_config(MODBUS_UART_PORT, &cfg);
    esp_err_t e2 = uart_set_pin(MODBUS_UART_PORT, MODBUS_UART_TX_PIN, MODBUS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    esp_err_t e3 = uart_driver_install(MODBUS_UART_PORT, MODBUS_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    ESP_LOGI(TAG, "uart init: param=%d pin=%d install=%d", e1, e2, e3);
	
}

/* Read only the expected response, not the whole UART buffer. A fragmented
 * frame shares one overall deadline; an exact local request echo is skipped. */
static bool receive_response(const modbus_request_t *req, const uint8_t frame[8],
                             modbus_data_t *out)
{
    uint8_t buf[MODBUS_UART_BUF_SIZE];
    unsigned len = 0;
    TickType_t started = xTaskGetTickCount();
    while (1) {
        if (len >= 8 && memcmp(buf, frame, 8) == 0) {
            memmove(buf, buf + 8, len - 8);
            len -= 8;
        }
        unsigned target = 3;
        if (len >= 3) {
            unsigned bytes = req->function_code <= 2
                ? (req->reg_count + 7) / 8 : req->reg_count * 2;
            target = (buf[1] & 0x80) ? 5 : 5 + bytes;
            if (len >= target) {
                if (len == target && parse_modbus_response(buf, len, req, out)) return true;
                /* A single-register reply is 7 bytes; a request echo is 8. */
                if (len < 8 && memcmp(buf, frame, len) == 0) target = 8;
                else return false;
            }
        }
        TickType_t elapsed = xTaskGetTickCount() - started;
        if (elapsed >= MODBUS_RESP_TIMEOUT || target > sizeof(buf)) return false;
        int n = uart_read_bytes(MODBUS_UART_PORT, buf + len, target - len,
                                MODBUS_RESP_TIMEOUT - elapsed);
        if (n <= 0) return false;
        len += n;
    }
}

static void process_batch(const modbus_batch_t *batch)
{
    /* Only the UART worker changes speed, before any frame in this sweep. */
    uint32_t baud = batch->baud_rate ? batch->baud_rate : MODBUS_UART_BAUD;
    bool speed_ok = uart_set_baudrate(MODBUS_UART_PORT, baud) == 0;
    if (!speed_ok) ESP_LOGE(TAG, "Unable to set baud rate %u", (unsigned)baud);
    for (unsigned i = 0; i < batch->count; i++) {
        const modbus_request_t *req = &batch->requests[i];
        uint8_t frame[8];
        build_read_request(req, frame);
        if (speed_ok) {
            uart_flush_input(MODBUS_UART_PORT);
            uart_write_bytes(MODBUS_UART_PORT, (const char *)frame, sizeof(frame));
            uart_wait_tx_done(MODBUS_UART_PORT, pdMS_TO_TICKS(50));
        }
        modbus_data_t data = {
            .request_id = req->request_id,
            .slave_id = req->slave_id,
            .function_code = req->function_code,
            .reg_addr = req->reg_addr,
            .reg_count = req->reg_count,
        };
        data.valid = speed_ok && receive_response(req, frame, &data);
        data.batch_complete = i + 1 == batch->count;
        /* Always notify completion, including timeouts. The UI never guesses
         * that a still-running sweep has finished based on its own timer. */
        xQueueSend(req->response_queue, &data, portMAX_DELAY);
        /* Conservative RTU silence for all supported speeds (9600 and above). */
        vTaskDelay(pdMS_TO_TICKS(5) + 1);
    }
}

static void modbus_master_task(void *pvParameter)
{
    (void)pvParameter;
    modbus_batch_t batch;
    while (1) {
        if (xQueueReceive(s_req_queue, &batch, portMAX_DELAY) == pdTRUE)
            process_batch(&batch);
    }
}

/* Modbus RTU 읽기 요청 프레임 (8바이트) 생성 */
static void build_read_request(const modbus_request_t *req, uint8_t *frame_out)
{
    frame_out[0] = req->slave_id;
    frame_out[1] = req->function_code;
    frame_out[2] = (req->reg_addr  >> 8) & 0xFF;
    frame_out[3] =  req->reg_addr        & 0xFF;
    frame_out[4] = (req->reg_count >> 8) & 0xFF;
    frame_out[5] =  req->reg_count       & 0xFF;

    uint16_t crc = modbus_crc16(frame_out, 6);
    frame_out[6] =  crc        & 0xFF;   /* CRC low byte first */
    frame_out[7] = (crc >> 8)  & 0xFF;
}

/* CRC-16/IBM (Modbus 표준) */
static uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;
}

static bool parse_modbus_response(const uint8_t *buf, int len,
                                  const modbus_request_t *req,
                                  modbus_data_t *out)
{
    if (req->function_code < 0x01 || req->function_code > 0x04 ||
        req->reg_count == 0 || req->reg_count > MODBUS_MAX_REGS) return false;

    /* Only skip an exact copy of the transmitted request. */
    uint8_t request_frame[8];
    build_read_request(req, request_frame);
    if (len >= 8 + MODBUS_MIN_FRAME_LEN &&
        memcmp(buf, request_frame, sizeof(request_frame)) == 0) {
        buf += sizeof(request_frame);
        len -= sizeof(request_frame);
    }
    if (len < MODBUS_MIN_FRAME_LEN) return false;

    /* CRC 검증 */
    uint16_t received_crc = (uint16_t)buf[len - 2] | ((uint16_t)buf[len - 1] << 8);
    uint16_t calc_crc     = modbus_crc16(buf, len - 2);
    if (received_crc != calc_crc) {
        ESP_LOGW(TAG, "CRC mismatch: got 0x%04X, calc 0x%04X", received_crc, calc_crc);
        return false;
    }

    /* 슬레이브 ID 확인 */
    if (buf[0] != req->slave_id) {
        ESP_LOGW(TAG, "Slave ID mismatch: got 0x%02X, expected 0x%02X", buf[0], req->slave_id);
        return false;
    }

    /* Selected function code or its exception response */
    if (buf[1] == (req->function_code | 0x80)) {
        ESP_LOGE(TAG, "Modbus exception code: 0x%02X", buf[2]);
        return false;
    }
    if (buf[1] != req->function_code) {
        ESP_LOGW(TAG, "Unexpected function code: 0x%02X", buf[1]);
        return false;
    }

    bool bit_response = req->function_code <= 0x02;
    uint8_t data_bytes = bit_response ? (req->reg_count + 7) / 8 : req->reg_count * 2;
    /* Preserve the existing FC03 slave's nonstandard byte count. */
    bool legacy_count = req->function_code == 0x03 && buf[2] == req->reg_count;
    if ((buf[2] != data_bytes && !legacy_count) || len != 3 + data_bytes + 2) {
        ESP_LOGW(TAG, "Byte count mismatch: got %d, expected %d (len=%d)",
                 buf[2], data_bytes, len);
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->valid = true;
    out->request_id = req->request_id;
    out->slave_id  = buf[0];
    out->function_code = req->function_code;
    out->reg_addr  = req->reg_addr;
    out->reg_count = req->reg_count;
    for (uint16_t i = 0; i < req->reg_count; i++) {
        out->values[i] = bit_response ? ((buf[3 + i / 8] >> (i % 8)) & 1)
            : (int16_t)((buf[3 + i * 2] << 8) | buf[4 + i * 2]);
    }
    return true;
}
