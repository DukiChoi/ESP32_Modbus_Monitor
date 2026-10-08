#!/usr/bin/env python3
"""Host UI integration test with repository LVGL. Requires Python 3 and GCC.
Run: python3 tests/run_sensor_ui.py
This does not replace an ESP-IDF firmware build or hardware UART testing.
"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    with tempfile.TemporaryDirectory(prefix="lvgl-sensor-test-") as folder:
        temp = Path(folder)
        (temp / "freertos").mkdir()
        (temp / "driver").mkdir()
        (temp / "freertos/FreeRTOS.h").write_text(
            "#pragma once\n#include <stdint.h>\ntypedef uint32_t TickType_t;\n"
            "#define pdMS_TO_TICKS(x) (x)\n#define pdTRUE 1\nTickType_t xTaskGetTickCount(void);\n")
        (temp / "freertos/queue.h").write_text(
            "#pragma once\ntypedef void *QueueHandle_t;\n"
            "int xQueueReceive(QueueHandle_t, void *, unsigned);\n"
            "QueueHandle_t xQueueCreate(unsigned, unsigned);\n")
        (temp / "driver/uart.h").write_text("#pragma once\n")
        (temp / "esp_attr.h").write_text("#define IRAM_ATTR\n#define DRAM_ATTR\n")
        config = []
        for line in (ROOT / "sdkconfig").read_text().splitlines():
            if line.startswith("CONFIG_") and "=" in line:
                key, value = line.split("=", 1)
                config.append(f"#define {key} {'1' if value == 'y' else value}")
        (temp / "sdkconfig.h").write_text("\n".join(config) + "\n")
        flags = ["-std=gnu99", "-O0", "-g", "-Werror=implicit-function-declaration",
                 "-include", str(temp / "sdkconfig.h"), "-DLV_CONF_INCLUDE_SIMPLE=1",
                 f"-I{temp}", "-Icomponents/lvgl", "-Icomponents/lvgl/lvgl", "-Imain/modbus_src"]
        sources = [ROOT / "tests/test_sensor_ui.c", ROOT / "main/modbus_src/lv_sensor_profiles.c", ROOT / "main/modbus_src/lv_app_font.c"]
        sources += sorted((ROOT / "components/lvgl/lvgl/src").rglob("*.c"))

        def compile_source(item):
            index, source = item
            obj = temp / f"{index}.o"
            result = subprocess.run(["gcc", *flags, "-c", str(source), "-o", str(obj)],
                                    cwd=ROOT, capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError(f"{source}\n{result.stderr}")
            return str(obj)

        with ThreadPoolExecutor(max_workers=4) as pool:
            objects = list(pool.map(compile_source, enumerate(sources)))
        executable = temp / "sensor-ui-test"
        subprocess.run(["gcc", *objects, "-lm", "-Wl,--wrap=lv_debug_log_error", "-o", str(executable)],
                       cwd=ROOT, check=True)
        subprocess.run([str(executable)], cwd=temp, check=True, timeout=30)


if __name__ == "__main__":
    main()
