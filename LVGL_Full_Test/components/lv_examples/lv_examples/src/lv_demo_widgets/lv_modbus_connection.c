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
static void     build_fc03_request(const modbus_request_t *req, uint8_t *frame_out);
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
    if (reg_count == 0 || reg_count > MODBUS_MAX_REGS) {
        ESP_LOGE(TAG, "reg_count out of range: %d", reg_count);
        return;
    }
    modbus_request_t req = {
        .slave_id  = slave_id,
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
        /* FC 0x03 요청 프레임 빌드 후 전송 */
        build_fc03_request(&req, frame);
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
            xQueueSend(modbus_chart_queue, &data, 0);
        }
    }
}

/* Modbus RTU FC 0x03 요청 프레임 (8바이트) 생성 */
static void build_fc03_request(const modbus_request_t *req, uint8_t *frame_out)
{
    frame_out[0] = req->slave_id;
    frame_out[1] = 0x03;
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
    /* RS485 에코 건너뛰기 — 자동방향 모듈이 TX 프레임 8바이트를 RX로 돌려보내는 경우 */
    const int ECHO_LEN = 8; /* FC03 요청 프레임 크기 */
    if (len > ECHO_LEN && buf[0] == req->slave_id) {
        /* buf[1]이 0x03이고 buf[2]가 byte_count(26 또는 13)가 아니면 에코일 가능성 */
        uint8_t expected_bc = (uint8_t)(req->reg_count * 2);
        if (buf[1] == 0x03 && buf[2] != expected_bc && buf[2] != req->reg_count) {
            ESP_LOGI(TAG, "Echo detected, skipping %d bytes", ECHO_LEN);
            buf += ECHO_LEN;
            len -= ECHO_LEN;
        }
    }

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

    /* Function code 확인 — 0x83이면 에러 응답 */
    if (buf[1] == (0x03 | 0x80)) {
        ESP_LOGE(TAG, "Modbus exception code: 0x%02X", buf[2]);
        return false;
    }
    if (buf[1] != 0x03) {
        ESP_LOGW(TAG, "Unexpected function code: 0x%02X", buf[1]);
        return false;
    }

    /* 바이트 수 검증
     * 슬레이브 버그: byte_count 필드에 reg_count*2 대신 reg_count를 그대로 보냄.
     * 표준(reg_count*2)과 버그(reg_count) 둘 다 허용. 실제 데이터 길이는 reg_count*2. */
    uint8_t data_bytes = (uint8_t)(req->reg_count * 2);
    if ((buf[2] != data_bytes && buf[2] != req->reg_count) || len < 3 + data_bytes + 2) {
        ESP_LOGW(TAG, "Byte count mismatch: got %d, expected %d or %d (len=%d)",
                 buf[2], (uint8_t)req->reg_count, data_bytes, len);
        return false;
    }

    /* 레지스터 값 추출 */
    out->slave_id  = buf[0];
    out->reg_addr  = req->reg_addr;
    out->reg_count = req->reg_count;
    for (uint16_t i = 0; i < req->reg_count; i++) {
        out->values[i] = (int16_t)((buf[3 + i * 2] << 8) | buf[4 + i * 2]);
    }
    return true;
}
