#!/usr/bin/env python3
"""Test real Modbus receive/batch logic using a simulated UART; requires GCC."""
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="modbus-transport-") as folder:
    temp = Path(folder)
    for directory in ["driver", "freertos"]:
        (temp / directory).mkdir()
    headers = {
        "freertos/FreeRTOS.h": """#pragma once
#include <stdint.h>
#include <stddef.h>
typedef uint32_t TickType_t;
typedef int BaseType_t;
#define pdMS_TO_TICKS(x) (x)
#define pdTRUE 1
#define portMAX_DELAY UINT32_MAX
TickType_t xTaskGetTickCount(void);
""",
        "freertos/queue.h": """#pragma once
typedef void *QueueHandle_t;
int xQueueSend(QueueHandle_t, const void *, unsigned);
int xQueueReceive(QueueHandle_t, void *, unsigned);
QueueHandle_t xQueueCreate(unsigned, unsigned);
""",
        "freertos/task.h": """#pragma once
void vTaskDelay(TickType_t);
int xTaskCreatePinnedToCore(void (*)(void *), const char *, unsigned, void *, unsigned, void *, int);
""",
        "driver/gpio.h": "#pragma once\n",
        "driver/uart.h": """#pragma once
#include "freertos/FreeRTOS.h"
typedef int esp_err_t;
#define UART_NUM_2 2
#define UART_DATA_8_BITS 8
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_PIN_NO_CHANGE -1
typedef struct { int baud_rate, data_bits, parity, stop_bits, flow_ctrl; } uart_config_t;
int uart_set_baudrate(int, uint32_t);
int uart_read_bytes(int, void *, uint32_t, TickType_t);
int uart_write_bytes(int, const char *, size_t);
int uart_flush_input(int);
int uart_wait_tx_done(int, TickType_t);
int uart_param_config(int, const uart_config_t *);
int uart_set_pin(int, int, int, int, int);
int uart_driver_install(int, int, int, int, void *, int);
""",
        "esp_log.h": """#pragma once
#define ESP_LOG_DEBUG 4
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
void esp_log_level_set(const char *, int);
""",
    }
    for name, content in headers.items():
        (temp / name).write_text(content)
    executable = temp / "transport-test"
    subprocess.run(["gcc", "-std=gnu99", "-Werror=implicit-function-declaration",
                    "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections", f"-I{temp}",
                    str(ROOT / "tests/test_modbus_transport.c"), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], cwd=temp, check=True, timeout=10)
