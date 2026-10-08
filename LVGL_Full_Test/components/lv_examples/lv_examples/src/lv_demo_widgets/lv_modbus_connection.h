/**
 * @file lv_modbus_connection.h
 */

#ifndef LV_MODBUS_CONNECTION_H
#define LV_MODBUS_CONNECTION_H

#ifdef __cplusplus	
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include <stdint.h>
#include <stdbool.h>

/*********************
 *      DEFINES
 *********************/
#define MODBUS_UART_PORT        UART_NUM_2
#define MODBUS_UART_BAUD        9600
// #define MODBUS_UART_TX_PIN      GPIO_NUM_41
// #define MODBUS_UART_RX_PIN      GPIO_NUM_40
#define MODBUS_UART_TX_PIN      27	//GPIO_NUM_1
#define MODBUS_UART_RX_PIN      22	//GPIO_NUM_3
#define MODBUS_UART_BUF_SIZE    256
#define MODBUS_QUEUE_SIZE       32
#define MODBUS_MAX_REGS         13      /* 한 번에 읽을 수 있는 최대 레지스터 수 */

/**********************
 *      TYPEDEFS
 **********************/

/* 요청 파라미터 — modbus_send_request()에서 내부 큐로 전달 */
typedef struct {
    uint8_t  slave_id;
    uint16_t reg_addr;
    uint16_t reg_count;
} modbus_request_t;

/* 파싱 결과 — modbus_chart_queue로 전달 */
typedef struct {
    uint8_t  slave_id;
    uint16_t reg_addr;
    uint8_t  reg_count;
    int16_t  values[MODBUS_MAX_REGS];
} modbus_data_t;

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void modbus_connection_init(void);

/**
 * UI 태스크 등에서 호출 — 비동기로 요청 큐에 넣음
 * @param slave_id  슬레이브 ID (1~247)
 * @param reg_addr  시작 레지스터 주소
 * @param reg_count 읽을 레지스터 수 (최대 MODBUS_MAX_REGS)
 */
void modbus_send_request(uint8_t slave_id, uint16_t reg_addr, uint16_t reg_count);

/**********************
 *  EXTERN VARIABLES
 **********************/
extern QueueHandle_t modbus_chart_queue;

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LV_MODBUS_CONNECTION_H */
