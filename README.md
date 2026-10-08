# ESP32 Modbus Monitor

ESP32와 LVGL v7 기반의 디스플레이·터치 UI 및 Modbus RTU 센서 모니터링 프로젝트입니다. 용도가 다른 두 프로그램을 각각 독립된 ESP-IDF 프로젝트 폴더로 관리합니다.

## 프로그램 한눈에 보기

| 프로그램 | 용도 | 주요 기능 |
|---|---|---|
| [LVGL_EDPD_Modbus](LVGL_EDPD_Modbus/) | 센서군별 Modbus 모니터링 | 센서 선택, 전체 채널 읽기, 현재값·통합 그래프, 통신 시작·중지, 보드레이트 선택 |
| [LVGL_Full_Test](LVGL_Full_Test/) | LVGL 화면과 Modbus 그래프 기능 시험 | 사인·스텝 파형, 채널 선택, Modbus 데이터 그래프, 채널별 최소·최대값 |

## LVGL_EDPD_Modbus — 센서 모니터

선택한 센서군의 채널을 Modbus RTU로 읽고, 단위 변환된 현재값과 여러 채널의 추이를 한 차트에 표시합니다.

- **화면:** Home과 LWTM / BTM / B-WACS / O-WACS / AVM 탭으로 구성합니다. Home에서 보드레이트와 통신 시작·중지를 제어합니다.
- **수집:** 선택한 센서군만 읽으며, 연속 주소를 묶어 FC03 요청을 순차 전송합니다. 전체 읽기는 500ms 시작 간격을 기준으로 하고, 응답이 늦으면 이전 읽기가 끝날 때까지 기다립니다.
- **표시:** 전체 읽기가 완료되면 현재값과 그래프를 함께 갱신합니다. 실패한 채널은 응답 없음과 그래프의 빈 구간으로 표시합니다.
- **설정:** 채널 주소, Scale/Offset, 단위, 그래프 범위는 `main/modbus_src/lv_sensor_profiles.c`에서 관리합니다. `ModbusTable.csv`는 참고 목록이며 실행 중 불러오는 설정 파일은 아닙니다.

| 센서군 | 채널 수 | 표시 단위 | 읽기 상태 |
|---|---:|---|---|
| LWTM | 10 | °C | 사용 |
| BTM | 12 | °C | 사용 |
| B-WACS | 10 | µm | 매핑·화면 유지, 현재 시리얼 읽기 제외 |
| O-WACS | 1 | Aw | 사용 |
| AVM | 1 | mm | 사용 |

기본 통신은 UART2, 9600 baud, 8N1이며 TX는 GPIO27, RX는 GPIO22입니다. 보드레이트는 Home에서 변경할 수 있습니다. 알람 판정 기능은 포함하지 않습니다.

상세 사용법과 통신 동작은 [프로그램 README](LVGL_EDPD_Modbus/README.md), 채널 목록은 [ModbusTable.csv](LVGL_EDPD_Modbus/ModbusTable.csv)를 참고하세요.

## LVGL_Full_Test — 화면·통신 시험 프로그램

LVGL ESP32 데모를 바탕으로 화면 출력, 터치 입력, 그래프 갱신과 Modbus 통신을 시험하는 프로그램입니다.

- **화면:** 홈과 EDPD 그래프 탭에서 제어 UI와 차트를 확인합니다.
- **파형 시험:** 사인 함수와 스텝 함수 모드로 센서 데이터 없이 그래프 동작을 확인합니다.
- **통신 시험:** Modbus 모드에서 수신 데이터를 채널별 그래프와 최소·최대값으로 표시합니다.
- **코드 위치:** 시작점은 `main/main.c`이며, UI와 통신 코드는 `components/lv_examples/lv_examples/src/lv_demo_widgets/`의 `lv_modbus.c`, `lv_modbus_connection.c`에 있습니다.

기본 통신은 UART2, 9600 baud, TX GPIO27 / RX GPIO22입니다. 디스플레이와 터치 컨트롤러는 프로젝트 설정에 맞춰 사용합니다.

[폴더 README](LVGL_Full_Test/README.md)는 원본 LVGL ESP32 포팅 프로젝트의 하드웨어·설정 안내를 담고 있습니다. 해당 문서의 원본 저장소 복제 명령 대신 아래 명령으로 이 저장소를 받으면 됩니다.

## 시작하기

```sh
git clone git@github.com:DukiChoi/ESP32_Modbus_Monitor.git
cd ESP32_Modbus_Monitor
```

각 폴더에 LVGL과 드라이버 소스가 포함되어 있습니다. 사용할 프로그램 폴더에서 ESP-IDF 환경을 활성화하고 빌드하세요. Modbus 프로그램의 기존 개발 환경은 ESP-IDF 4.4.3 / LVGL v7입니다.

```sh
# 센서 모니터
cd LVGL_EDPD_Modbus
idf.py -B build_modbus_src build
```

```sh
# 화면·통신 시험 프로그램 — 저장소 루트 기준
cd LVGL_Full_Test
idf.py menuconfig
idf.py build
```

디스플레이·터치 설정과 배선은 각 폴더의 `sdkconfig` 및 소스를 확인하세요. 두 프로그램은 각각 빌드하며, 하나의 펌웨어로 함께 실행하지 않습니다.

## 호스트 검증

`LVGL_EDPD_Modbus`는 Python 3와 GCC를 이용한 UI·Modbus 통신 테스트를 제공합니다.

```sh
cd LVGL_EDPD_Modbus
python3 tests/run_sensor_ui.py
python3 tests/run_modbus_transport.py
```

호스트 테스트는 ESP-IDF 펌웨어 전체 빌드나 실제 장비 통신 검증을 대신하지 않습니다. 검증 범위와 현재 상태는 [프로그램 README](LVGL_EDPD_Modbus/README.md#빌드-및-검증)를 참고하세요.
