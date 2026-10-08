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
    s_req_queue        = xQueueCreate(8, sizeof(modbus_request_t));
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
    if (!response_queue || !s_req_queue) return;
    if (function_code < 0x01 || function_code > 0x04) return;
    if ((uint32_t)reg_addr + reg_count > 65536U) return;
    if (reg_count == 0 || reg_count > MODBUS_MAX_REGS) {
        ESP_LOGE(TAG, "reg_count out of range: %d", reg_count);
        return;
    }
    modbus_request_t req = {
        .response_queue = response_queue,
        .request_id = request_id,
        .slave_id  = slave_id,
        .function_code = function_code,
        .reg_addr  = reg_addr,
        .reg_count = reg_count,
    };
    BaseType_t r = xQueueSend(s_req_queue, &req, 0);
    // ESP_LOGI	(TAG, "send_request slave=0x%02X reg=0x%04X cnt=%d queued=%s",
    //          slave_id, reg_addr, reg_count, r == pdTRUE ? "OK" : "FAIL(queue full)");
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
    uart_param_config(MODBUS_UART_PORT, &cfg);
    uart_set_pin(MODBUS_UART_PORT,
                 MODBUS_UART_TX_PIN, MODBUS_UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(MODBUS_UART_PORT, MODBUS_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
	esp_err_t e1 = uart_param_config(MODBUS_UART_PORT, &cfg);
    esp_err_t e2 = uart_set_pin(MODBUS_UART_PORT, MODBUS_UART_TX_PIN, MODBUS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    esp_err_t e3 = uart_driver_install(MODBUS_UART_PORT, MODBUS_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    ESP_LOGI(TAG, "uart init: param=%d pin=%d install=%d", e1, e2, e3);
	
}

static void modbus_master_task(void *pvParameter)
{
    modbus_request_t req;
    uint8_t          frame[8];
    uint8_t          buf[MODBUS_UART_BUF_SIZE];
	static uint32_t last_tick  = 0;
	static uint32_t now  = 0;
    ESP_LOGI(TAG, "modbus_master_task started");
    while (1) {
        /* 요청이 올 때까지 대기 */
        if (xQueueReceive(s_req_queue, &req, portMAX_DELAY) != pdTRUE) continue;
        /* 읽기 요청 프레임 빌드 후 전송 */
        build_read_request(&req, frame);
        uart_flush_input(MODBUS_UART_PORT);
        uart_write_bytes(MODBUS_UART_PORT, (const char *)frame, sizeof(frame));

        /* TX 완료 대기 — 자동방향 RS485 모듈이 TX→RX 전환하기 전에 읽기 시작하면 응답 놓침 */
        uart_wait_tx_done(MODBUS_UART_PORT, pdMS_TO_TICKS(50));

        /* 응답 대기 */
        int len = uart_read_bytes(MODBUS_UART_PORT, buf, sizeof(buf), MODBUS_RESP_TIMEOUT);
		now = xTaskGetTickCount();
        /* 디버그: 수신 raw 바이트 출력 */
        if (len > 0) {
            ESP_LOGI(TAG, "Delay:%d RX %d bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02x %02x %02x %02x %02x %02x %02x %02x %02X %02X %02X %02X %02X %02X %02X %02X %02x %02x %02x %02x %02x %02x %02x",
                     (now - last_tick), len,
                     len > 0 ? buf[0] : 0, len > 1 ? buf[1] : 0,
                     len > 2 ? buf[2] : 0, len > 3 ? buf[3] : 0,
                     len > 4 ? buf[4] : 0, len > 5 ? buf[5] : 0,
                     len > 6 ? buf[6] : 0, len > 7 ? buf[7] : 0,
					 len > 8 ? buf[8] : 0, len > 9 ? buf[9] : 0,
					 len > 10 ? buf[10] : 0, len > 11 ? buf[11] : 0,
					 len > 12 ? buf[12] : 0, len > 13 ? buf[13] : 0,
					 len > 14 ? buf[14] : 0, len > 15 ? buf[15] : 0,
					 len > 16 ? buf[16] : 0, len > 17 ? buf[17] : 0,
					 len > 18 ? buf[18] : 0, len > 19 ? buf[19] : 0,
					 len > 20 ? buf[20] : 0, len > 21 ? buf[21] : 0,
					 len > 22 ? buf[22] : 0, len > 23 ? buf[23] : 0,
					 len > 24 ? buf[24] : 0, len > 25 ? buf[25] : 0,
					 len > 26 ? buf[26] : 0, len > 27 ? buf[27] : 0,
					 len > 28 ? buf[28] : 0, len > 29 ? buf[29] : 0,
					 len > 30 ? buf[30] : 0);
        }
		last_tick = now;

        if (len < MODBUS_MIN_FRAME_LEN) {
            // ESP_LOGW(TAG, "Timeout or short response (len=%d) slave=0x%02X reg=0x%04X",
            //          len, req.slave_id, req.reg_addr);
            continue;
        }

        modbus_data_t data;
        if (parse_modbus_response(buf, len, &req, &data)) {
            xQueueSend(req.response_queue, &data, 0);
        }
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
