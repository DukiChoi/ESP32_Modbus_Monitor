/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * History
  * 1. 2021.01.28  - Change the below temperature display in CalTemp()
  * 2. 2021.01.29  - Sensor Error Code Change in main()
  * 3. 2022.02.07  - Change the Board ID Set
  *                - Change the Sensor Set 2 & 3
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### */
#define REG_GPIOA_ODR	(*(volatile unsigned*)0x40020014)
#define USART_CR1		(*(volatile unsigned*)0x40020014)
/* ##################직접작성! 끝###################### *//* ##################직접작성! 끝###################### *//* ##################직접작성! 끝###################### */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### */

#define DEBUG_STATUS	1
#define MASTERIO

extern __IO uint32_t uwTick;
__IO uint32_t gTimer = 0;

unsigned char U2_Packet[64];
unsigned char U2_NewPacket = 0;
unsigned char u2_PacketSize = 0;
int           Resetplz = 0;
extern unsigned int ModbusCheck;
extern unsigned int ModbusReset;

#define COUNTOF(buf)	(sizeof(buf) / sizeof(*(buf)))
#define BUFFERSIZE(buf)	COUNTOF(buf)

uint8_t       tx_buffer[]    = "Hal GD!\r\n";
uint8_t       rx2_data[2];
uint8_t       rx2_buffer[255];
uint8_t       adc1_data[64];
unsigned char spi1_conversion = 0;

#define MASTER_TX_SIZE	20
#define MASTER_RX_SIZE	20

/* Sensor type codes */
#define RTD		1
#define TC		2
#define ONOFF	3
#define CURRENT	4
#define RTD1000	5
#define VOLTAGE	6

/* ADC state machine states */
#define ST_START_SEND	1
#define ST_START_END	2
#define ST_WAIT_CNVT	3
#define ST_READ_SEND0	4
#define ST_READ_SEND1	5
#define ST_READ_SEND2	6
#define ST_READ_SEND3	7
#define ST_READ_END0	8
#define ST_READ_END1	9
#define ST_READ_END2	10
#define ST_READ_END3	11

/* System state machine states */
#define SS_RUN			0
#define SS_RESET		1
#define SS_SET_START	2
#define SS_REC_ID_SET	3
#define SS_SEND_RES_ID	4
#define SS_SENSOR_SET	5
#define SS_WAIT			6

/* Sensor type identifiers */
#define SENSOR_TYPE_PT100	1
#define SENSOR_TYPE_TC		2
#define SENSOR_TYPE_MA		3
#define SENSOR_TYPE_CT		4
#define SENSOR_TYPE_PT1000	5
#define SENSOR_TYPE_VOLTAGE	6
#define SENSOR_TYPE_NC		0

int           TransmitComplete[4];
int           gSS = 0;
unsigned char DCUMODE = 0;
int           SPI_Complt[4] = {0, 0, 0, 0};

unsigned char gSensorType[5] = {
	SENSOR_TYPE_PT100, SENSOR_TYPE_PT100, SENSOR_TYPE_PT100, SENSOR_TYPE_PT100,
};

uint16_t gSensorValue[16];
uint8_t  gSensorStatus[16];
uint8_t  g_TempSensorStatus[16];

/* SPI3 multi-conversion register mask commands */
unsigned char gMultiConversionSet0[5] = {0x02, 0x00, 0xF7, 0x00,};
unsigned char gMultiConversionSet1[5] = {0x02, 0x00, 0xF6, 0x00,};
unsigned char gMultiConversionSet2[5] = {0x02, 0x00, 0xF5, 0x00,};
uint8_t       MultiConversion_start[5]= {0x02, 0x00, 0x00, 0x80, 0};

unsigned char spi1_interrupt_status = 1, spi2_interrupt_status = 1;
unsigned char spi3_interrupt_status = 1, spi4_interrupt_status = 1;
unsigned char spi1_conversing_end = 0, spi2_conversing_end = 0;
unsigned char spi3_conversing_end = 0, spi4_conversing_end = 0;
unsigned char BF_SendBuffer[4][4], BF_ReadBuffer[4][4];
unsigned char spi_saveBuffer[16][7];
unsigned char spi1_datacall = 0, spi2_datacall = 0;
unsigned char spi3_datacall = 0, spi4_datacall = 0;
unsigned char ADC_S_CMD_BF[20][4] = {};
unsigned char ADC_R_CMD_BF[20][4] = {};

int ST_ADC[4] = {ST_START_SEND, ST_START_SEND, ST_START_SEND, ST_START_SEND};
int tty = 0;
int gCnt[4] = {0, 0, 0, 0};

struct SLAVE {
	uint8_t          ModuleID;
	uint8_t          Flg_ADC_Setted;
	volatile uint8_t LED_COMM_RX, LED_MOD, LED_ST;
};

struct SLAVE gSlave;

#define DAQ_MOD		11
#define CONFIG_MOD	1

const char HextTable[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void shorttohex(int val, unsigned char *data)
{
	int indata = val;
	data[3] = HextTable[indata % 16]; indata >>= 4;
	data[2] = HextTable[indata % 16]; indata >>= 4;
	data[1] = HextTable[indata % 16]; indata >>= 4;
	data[0] = HextTable[indata % 16];
}

/* Channel address lookup tables */
int CHANNEL[25] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
	0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14,
};

int SetChannel[25] = {
	0,    0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24,
	0x28, 0x2C, 0x30, 0x34, 0x38, 0x3C, 0x40, 0x44, 0x48, 0x4C,
};

int ReadChannel[25] = {
	0,    0x10, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x30,
	0x34, 0x38, 0x3C, 0x40, 0x44, 0x48, 0x4C, 0x50, 0x54, 0x58, 0x5C,
};

/* RTD sensor configuration */
#define Two_External					0x0
#define Two_Internal					0x1
#define Three_External					0x4
#define Three_Internal					0x5
#define Four_External					0x8
#define Four_Internal_NotRotation		0x9
#define Four_Internal_Rotation			0xA
#define Four_Kelvin_External			0xC
#define Four_Kelvin_Internal_NotRotation	0xD
#define Four_Kelvin_Internal_Rotation		0xE

/* RTD excitation current */
#define Reserved			0x0
#define RTD_Current_5uA		0x1
#define RTD_Current_10uA	0x2
#define RTD_Current_25uA	0x3
#define RTD_Current_50uA	0x4
#define RTD_Current_100uA	0x5
#define RTD_Current_250uA	0x6
#define RTD_Current_500uA	0x7
#define RTD_Current_1mA		0x8

/* RTD curve */
#define Standard	0x0
#define American	0x1
#define Japanese	0x2
#define ITS_90		0x3

/* Sensor type IDs */
#define Type_J		0x1
#define Type_K		0x2
#define Type_E		0x3
#define Type_N		0x4
#define Type_R		0x5
#define Type_S		0x6
#define Type_T		0x7
#define Type_B		0x8
#define Type_C		0x9
#define PT10		0xA
#define PT50		0xB
#define PT100		0xC
#define PT200		0xD
#define PT500		0xE
#define PT1000		0xF
#define DIODE		0x1C
#define RESISTOR	0x1D
#define ADCC		0x1E

/* TC sensor configuration */
#define Diff_External	0x0
#define Diff_10uA		0x4
#define Diff_100uA		0x5
#define Diff_500uA		0x6
#define Diff_1mA		0x7
#define Single_External	0x8
#define Single_10uA		0xC
#define Single_100uA	0xD
#define Single_500uA	0xE
#define Single_1mA		0xF

/* TC diode configuration */
#define Single				0x1
#define Differential		0x0
#define Two_Conversion		0x0
#define Three_Conversion	0x1
#define Average				0x1

/* TC diode excitation current */
#define TC_Current_10uA		0x0
#define TC_Current_20uA		0x1
#define TC_Current_40uA		0x2
#define TC_Current_80uA		0x3

unsigned char return_data[10];
unsigned char write_data[10]       = {0x02, 0x02,};
unsigned char read_data1[10]       = {0x03, 0x00,};
unsigned char read_data2[10]       = {0x03, 0x00,};
unsigned char read_data3[10]       = {0x03, 0x00,};
unsigned char read_data4[10]       = {0x03, 0x00,};
unsigned char rtd_setting[5]       = {0x02, 0x02,};
unsigned char tc_setting[5]        = {0x02, 0x02,};
unsigned char conversion_start[5]  = {0x02, 0x00, 0x00,};
unsigned char conversion_start2[5] = {0x02, 0x00, 0x00,};
unsigned char conversion_start3[5] = {0x02, 0x00, 0x00,};
unsigned char conversion_start4[5] = {0x02, 0x00, 0x00,};

unsigned char mode = 0;
int read_channel1[20];
int read_channel2[20];
int read_channel4[20];
uint8_t spi_tx_buffer[5]    = {0,};
uint8_t spi_rx_buffer1[8]   = {0,};
uint8_t spi_rx_buffer2[8]   = {0,};
uint8_t spi_rx_buffer3[8]   = {0,};
uint8_t spi_rx_buffer4[8]   = {0,};
uint8_t spi_rx_buffer[5]    = {0,};
uint8_t spi_set_buffer1[5]  = {0,};
uint8_t spi_set_buffer2[5]  = {0,};
uint8_t spi_set_buffer3[5]  = {0,};
uint8_t spi_set_buffer4[5]  = {0,};
uint8_t uart_tx_buffer[20]  = {};
uint8_t uart_rx_buffer[255] = {};

uint8_t start_read[5]  = {0x03, 0x00, 0x00, 0x00, 0};
uint8_t ini_write_1[5] = {0x02, 0x02, 0x00, 0x00, 0};

int cnt = 0, r_cnt = 0;
int RecevPacketEnabe = 0, RequestPacketEnable = 0;
int PacketSize = 0, p_count = 0;
uint8_t set_buf[5][25];
uint8_t reply_buf[10];

/* ##################직접작성! 끝###################### *//* ##################직접작성! 끝###################### *//* ##################직접작성! 끝###################### */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### */
static void MX_USART2_UART_Init_GD(unsigned int Baudrate);

void RTD_Configuration(int type, uint8_t channel, int sensor, int current, int curve, uint8_t *return_data);
void RTD_Sense_Registor(int type, float registor, uint8_t *return_data);
void Setting_Write_Data(int channel, uint8_t *return_data, uint8_t count, uint8_t *write_data);
void RTD_Read_Data(int channel, uint8_t *read_data);
void TC_Configuration(int type, int channel, int configuration, uint8_t *return_data);
void TC_Assignment(int type, uint8_t in_type, uint8_t in_line, uint8_t average, uint8_t current, float Factor, uint8_t *return_data);
void TC_Setting(int channel, uint8_t *return_data, uint8_t count, uint8_t *tc_setting);
void TC_Read_Data(int channel, uint8_t *return_data);
void Channel_Setting(int adc, int ch, int type);
void ADC_MultiConversion_Set(void);
void SPI1_Write(int channel, uint8_t *return_data, uint8_t *write_data);
void SPI1_Read(int channel, int sensor, uint8_t *return_data);
void SPI2_Write(int channel, uint8_t *return_data, uint8_t *write_data);
void SPI2_Read(int channel, int sensor, uint8_t *return_data);
void SPI3_Write(int channel, uint8_t *return_data, uint8_t *write_data);
void SPI3_Read(int channel, int sensor, uint8_t *return_data);
void SPI4_Write(int channel, uint8_t *return_data, uint8_t *write_data);
void SPI4_Read(int channel, int sensor, uint8_t *return_data);
void SensorDataSend(int ADCNumber);

/* Converts 24-bit ADC result to temperature*10 + 500 (supports below-zero). */
uint16_t CalTemp(uint32_t a, uint32_t b, uint32_t c)
{
	double  deg;
	int32_t temp;

	if (a & 0x80)
		temp = (int32_t)(a * 256 * 256 + b * 256 + c) | 0xFF000000;  /* negative */
	else
		temp = a * 256 * 256 + b * 256 + c;

	deg = (double)temp / 1024.0;
	if (deg <= -50.0)
		deg = -50.0;
	//최소 -50도인 소수 1자리까지의 temp로 바꿔준 것.
	return (int)(deg * 10) + 500;
}

/* Converts 24-bit ADC result to current*20000 (4–20 mA range). */
uint16_t CalCurrent(uint32_t a, uint32_t b, uint32_t c)
{
	uint32_t temp = a * 256 * 256 + b * 256 + c;

	/* Negative raw value means sensor disconnected; clamp to zero. */
	if (a & 0x80)
		temp = 0;

	return (uint16_t)((double)temp * 0.0095367432);
}

/* ##################직접작성! 끝###################### *//* ##################직접작성! 끝###################### *//* ##################직접작성! 끝###################### */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### */

unsigned int Adc_ch_cnt = 0;
int      temp;
uint32_t prevTick = 0;
unsigned char U2_TX_buffer[64];
unsigned char U2_Tx_Cnt = 0, U2_Tx_Size = 0;
int temp1, temp2, temp3, temp4;
__IO int ggcnt = 0;

/* ID setting */
unsigned int Cur_ID_Set = 0, New_ID_Set = 0;
unsigned int TEST_Pin0 = 0, TEST_Pin1 = 0, TEST_Pin2 = 0, TEST_Pin3 = 0;

/* Channel setting */
unsigned int Cur_CH1_Set = 0, Cur_CH2_Set = 0, Cur_CH3_Set = 0, Cur_CH4_Set = 0;
unsigned int New_CH1_Set = 0, New_CH2_Set = 0, New_CH3_Set = 0, New_CH4_Set = 0;

/* RS-485 / Modbus */
unsigned int gSlave_ID, gSlave_ID_H, gSlave_ID_L;
unsigned int DataLength = 0, DataRevEnd = 0;
unsigned int StartAddress = 0, AddressLength = 0;
unsigned int BaudrateSet = 115200;

extern unsigned char U2_buffer[255];
extern unsigned int  ModbusTickCnt;
extern unsigned int  DataCnt, RevCheck;
unsigned char SendModbusData[256] = {0,};

uint16_t CalCRC, CheckCRC, TempCRC;

/* Modbus CRC16 lookup table – high byte */
const unsigned char auchCRCHi[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
	0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
	0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
	0x40
};

/* Modbus CRC16 lookup table – low byte */
const unsigned char auchCRCLo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
	0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
	0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
	0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
	0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
	0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
	0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
	0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
	0x40
};

uint16_t CRC16(unsigned char *puchMsg, uint16_t usDataLen)
{
	unsigned char uchCRCHi = 0xFF;
	unsigned char uchCRCLo = 0xFF;
	uint16_t uIndex;

	while (usDataLen--) {
		uIndex   = uchCRCLo ^ *puchMsg++;
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
		uchCRCHi = auchCRCLo[uIndex];
	}
	return (uchCRCHi << 8 | uchCRCLo);
}

uint32_t *gpio_c;
uint32_t *gpio_d;
uint32_t *gpio_e;
uint32_t *USART2_CR1;
uint32_t *USART2_CR3;

uint16_t gpio1 = 0, gpio2 = 0, gpio3 = 0, gpio4 = 0;
uint32_t bn;

/* ##################직접작성! 끝###################### *//* ##################직접작성! 끝###################### *//* ##################직접작성! 끝###################### */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */
	/* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### */
	unsigned char ADC3_interrupt_pin;
	int dcu_mode = 0;
	int i;
	gpio_d = (uint32_t *)0x40020C10;
	gpio_c = (uint32_t *)0x40020810;
	gpio_e = (uint32_t *)0x40021010;
	/* USER CODE END 1 */
	/* ##################직접작성! 끝###################### *//* ##################직접작성! 끝###################### *//* ##################직접작성! 끝###################### */
	/* MCU Configuration--------------------------------------------------------*/
	HAL_Init();



	SystemClock_Config();

	MX_GPIO_Init();
	MX_SPI3_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();

	/* USER CODE BEGIN 2 */
	/* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### */
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

	dcu_mode = CONFIG_MOD;
	gSlave.Flg_ADC_Setted = 1;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_Delay(250);
	gSlave.ModuleID = 0x0;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(1);

	/* Read board ID from GPIO pins */
	gSlave_ID_H = (*gpio_e) & 0xF;
	gSlave_ID_L = (*gpio_d >> 2) & 0xF;
	gSlave_ID   = (gSlave_ID_H * 10) + gSlave_ID_L;
	MX_USART2_UART_Init_GD(BaudrateSet);

	/* Read sensor type DIP switches */
	gSensorType[0] = ((*gpio_e) >> 8)  & 0xF;
	gSensorType[1] = ((*gpio_c) >> 4)  & 0xF;
	gSensorType[2] = (*gpio_c)         & 0xF;
	gSensorType[3] = ((*gpio_e) >> 12) & 0xF;

	Channel_Setting(3, 0, SENSOR_TYPE_PT100);
	Channel_Setting(3, 1, gSensorType[0]);
	Channel_Setting(3, 2, gSensorType[1]);
	Channel_Setting(3, 3, gSensorType[2]);
	Channel_Setting(3, 4, gSensorType[3]);

	ADC_MultiConversion_Set();

	dcu_mode = DAQ_MOD;
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

	while (1)
	{
		ggcnt++;

		/* UART overrun error recovery */
		if (Resetplz)
		{
			Resetplz = 0;
			bn = huart2.Instance->SR;
			bn = huart2.Instance->SR;
			bn = huart2.Instance->SR;
			bn = huart2.Instance->SR;
			bn = huart2.Instance->SR;
			bn = huart2.Instance->DR;
			bn = huart2.Instance->DR;
			bn = huart2.Instance->DR;
			bn = huart2.Instance->DR;
			MX_USART2_UART_Init();
			__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
		}

		gpio1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
		gpio2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
		gpio3 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
		gpio4 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);

		/* LED blink – 64 ms period */
		if (prevTick != uwTick)
		if ((uwTick % 64) == 0)
		{
			gSlave.LED_MOD = gSlave.Flg_ADC_Setted ? 1 : 0;
			gSlave.LED_ST  = 1;

			if ((uwTick % 512) == 0)
			{
				if (gSlave.LED_MOD)  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
				else                 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
			}

			if (gSlave.LED_ST)  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
			else                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

			gSlave.LED_COMM_RX = 0;
			prevTick = uwTick;
		}

		/* Periodic DIP switch re-read – every ~1 s */
		if ((uwTick % 1024) == 0)
		{
			HAL_Delay(1);
			gSlave_ID_H = (*gpio_e) & 0xF;
			gSlave_ID_L = (*gpio_d >> 2) & 0xF;
			gSlave_ID   = (gSlave_ID_H) * 10 + gSlave_ID_L;
			HAL_Delay(1);

			/* Channel 1 */
			gSensorType[0] = ((*gpio_e) >> 8) & 0xF;
			if (Cur_CH1_Set != gSensorType[0]) {
				Cur_CH1_Set = gSensorType[0];
				Channel_Setting(3, 1, gSensorType[0]);
				HAL_Delay(1);
				dcu_mode = CONFIG_MOD;
			}
			HAL_Delay(1);

			/* Channel 2 */
			gSensorType[1] = ((*gpio_c) >> 4) & 0xF;
			if (Cur_CH2_Set != gSensorType[1]) {
				Cur_CH2_Set = gSensorType[1];
				Channel_Setting(3, 2, gSensorType[1]);
				HAL_Delay(1);
				dcu_mode = CONFIG_MOD;
			}
			HAL_Delay(1);

			/* Channel 3 */
			gSensorType[2] = (*gpio_c) & 0xF;
			if (Cur_CH3_Set != gSensorType[2]) {
				Cur_CH3_Set = gSensorType[2];
				Channel_Setting(3, 3, gSensorType[2]);
				HAL_Delay(1);
				dcu_mode = CONFIG_MOD;
			}
			HAL_Delay(1);

			/* Channel 4 */
			gSensorType[3] = ((*gpio_e) >> 12) & 0xF;
			if (Cur_CH4_Set != gSensorType[3]) {
				Cur_CH4_Set = gSensorType[3];
				Channel_Setting(3, 4, gSensorType[3]);
				HAL_Delay(1);
				dcu_mode = CONFIG_MOD;
			}
		}

		/* ------------------------------------------------------------------ */
		/* Modbus RTU packet handling                                          */
		/* ------------------------------------------------------------------ */
		if (U2_NewPacket)
		{
			U2_NewPacket = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

			if (U2_Packet[0] == gSlave_ID)                       /* device ID match */
			{
				if (U2_Packet[1] == 0x03)                        /* Read Holding Registers */
				{
					StartAddress  = U2_Packet[2] * 256 + U2_Packet[3];
					AddressLength = U2_Packet[4] * 256 + U2_Packet[5];

					CheckCRC = CRC16(U2_Packet, 6);
					TempCRC  = (U2_Packet[7] * 256) + U2_Packet[6];

					if (CheckCRC == TempCRC)
					{
						SendModbusData[0] = U2_Packet[0];   /* Device ID */
						SendModbusData[1] = U2_Packet[1];   /* Function code */
						SendModbusData[2] = AddressLength;

						/* Sensor values (4 channels) */
						SendModbusData[3]  = gSensorValue[0] / 256;
						SendModbusData[4]  = gSensorValue[0] % 256;
						SendModbusData[5]  = gSensorValue[1] / 256;
						SendModbusData[6]  = gSensorValue[1] % 256;
						SendModbusData[7]  = gSensorValue[2] / 256;
						SendModbusData[8]  = gSensorValue[2] % 256;
						SendModbusData[9]  = gSensorValue[3] / 256;
						SendModbusData[10] = gSensorValue[3] % 256;

						/* Sensor status (4 channels) */
						SendModbusData[11] = 0; SendModbusData[12] = gSensorStatus[0];
						SendModbusData[13] = 0; SendModbusData[14] = gSensorStatus[1];
						SendModbusData[15] = 0; SendModbusData[16] = gSensorStatus[2];
						SendModbusData[17] = 0; SendModbusData[18] = gSensorStatus[3];

						/* Sensor type (4 channels) */
						SendModbusData[19] = 0; SendModbusData[20] = gSensorType[0];
						SendModbusData[21] = 0; SendModbusData[22] = gSensorType[1];
						SendModbusData[23] = 0; SendModbusData[24] = gSensorType[2];
						SendModbusData[25] = 0; SendModbusData[26] = gSensorType[3];

						/* Board ID */
						SendModbusData[27] = 0; SendModbusData[28] = gSlave_ID;

						CalCRC = CRC16(SendModbusData, AddressLength * 2 + 3);
						SendModbusData[(AddressLength * 2) + 3] = CalCRC & 0xFF;
						SendModbusData[(AddressLength * 2) + 4] = CalCRC >> 8;

						HAL_Delay(10);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
						HAL_UART_Transmit_IT(&huart2, SendModbusData, (AddressLength * 2) + 5);
						HAL_Delay(30);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

						for (i = 0; i <= (AddressLength + 3) * 2; i++)
							SendModbusData[i] = 0;
					}
				}
			}
			ModbusCheck = 0;
		}

		/* ------------------------------------------------------------------ */
		/* DAQ / CONFIG mode state machine                                     */
		/* ------------------------------------------------------------------ */
		if (gSlave.Flg_ADC_Setted)
		switch (dcu_mode)
		{
			case CONFIG_MOD:
				Channel_Setting(3, 1, gSensorType[0]);
				Channel_Setting(3, 2, gSensorType[1]);
				Channel_Setting(3, 3, gSensorType[2]);
				Channel_Setting(3, 4, gSensorType[3]);
				ADC_MultiConversion_Set();
				dcu_mode = DAQ_MOD;
				break;

			case DAQ_MOD:
			{
				if ((uwTick % 16) == 4)
				{
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);

					switch (ST_ADC[2])
					{
					/* ---- Conversion start ---- */
					case ST_START_SEND:
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
						TransmitComplete[2] = 0;
						HAL_SPI_Transmit_IT(&hspi3, MultiConversion_start, 4);
						ST_ADC[2] = TransmitComplete[2] ? ST_WAIT_CNVT : ST_START_END;
						break;

					case ST_START_END:
						if (HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_READY)
							ST_ADC[2] = ST_WAIT_CNVT;
						break;

					case ST_WAIT_CNVT:
						ADC3_interrupt_pin = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10);
						if (ADC3_interrupt_pin == 1)
							ST_ADC[2] = ST_READ_SEND0;
						break;

					/* ---- Channel 0 (ADC ch8) ---- */
					case ST_READ_SEND0:
						SPI_Complt[2] = 0;
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
						HAL_SPI_TransmitReceive_IT(&hspi3, ADC_R_CMD_BF[8], spi_rx_buffer3, 7);
						ST_ADC[2] = ST_READ_END0;
						break;

					case ST_READ_END0:
						if (HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_READY)
						{
							if (gSensorType[0] == SENSOR_TYPE_PT100)
							{
#if DEBUG_STATUS == 1
								if (spi_rx_buffer3[3] == 0x01) {
									//2바이트 인티져 gSensorValue에 넣어줌.
									gSensorValue[0] = CalTemp(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
									if      (gSensorValue[0] == 0)    gSensorStatus[0] = 0x02;  /* Over Range */
									else if (gSensorValue[0] > 6535)  gSensorStatus[0] = 0x02;  /* Over Range */
									else                              gSensorStatus[0] = 0x00;  /* Normal */
								}
								else if ((spi_rx_buffer3[3] == 0x81) || (spi_rx_buffer3[3] == 0x85) ||
								         (spi_rx_buffer3[3] == 0x89) || (spi_rx_buffer3[3] == 0x09)) {
									gSensorValue[0]  = 0;
									gSensorStatus[0] = 0x10;  /* Not Connected */
								}
								else if (spi_rx_buffer3[3] == 0xCB) {
									gSensorValue[0]  = 0;
									gSensorStatus[0] = 0x08;  /* Line Break */
								}
								else if (spi_rx_buffer3[3] == 0x8B) {
									gSensorValue[0]  = 0;
									gSensorStatus[0] = 0x08;  /* Line Break */
								}
								else if (spi_rx_buffer3[3] == 0x05) {
									gSensorValue[0]  = 0;
									gSensorStatus[0] = 0x04;  /* Config Mismatch */
								}
								else {
									gSensorValue[0]  = 0;
									gSensorStatus[0] = 0x01;  /* Fault */
								}
#else
								gSensorStatus[0] = spi_rx_buffer3[3];
#endif
							}
							else if (gSensorType[0] == SENSOR_TYPE_PT1000)
							{
								gSensorValue[0] = CalTemp(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
#if DEBUG_STATUS == 1
								if ((spi_rx_buffer3[3] == 0x01) || (spi_rx_buffer3[3] == 0x03)) {
									if (g_TempSensorStatus[8] == 0xCB) {
										if (gSensorValue[0] < 32775) {
											if (gSensorValue[0] > 6500) gSensorStatus[0] = 0x02;  /* Over Range */
											else                        gSensorStatus[0] = 0x00;  /* Normal */
										}
										else gSensorStatus[0] = 0x00;  /* Normal */
									}
									else if ((g_TempSensorStatus[8] == 0x81) || (g_TempSensorStatus[8] == 0x85))
										gSensorStatus[0] = 0x08;  /* Line Break */
									else if (g_TempSensorStatus[8] == 0x01)
										gSensorStatus[0] = 0x04;  /* Config Mismatch */
								}
								else if ((spi_rx_buffer3[3] == 0x81) || (spi_rx_buffer3[3] == 0x85)) {
									if ((g_TempSensorStatus[8] == 0x81) || (g_TempSensorStatus[8] == 0x85) ||
									    (g_TempSensorStatus[8] == 0x89) || (spi_rx_buffer3[3] == 0x09))
										gSensorStatus[0] = 0x18;  /* Line Break or Not Matched */
									else if (g_TempSensorStatus[8] == 0x8B)
										gSensorStatus[0] = 0x10;  /* Not Matched */
								}
								else if (spi_rx_buffer3[3] == 0xCB) {
									if ((g_TempSensorStatus[8] == 0xCB) || (g_TempSensorStatus[8] == 0xEB))
										gSensorStatus[0] = 0x08;  /* Line Break */
									else
										gSensorStatus[0] = 0x10;  /* Not Matched */
								}
								else if (spi_rx_buffer3[3] == 0x83) gSensorStatus[0] = 0x08;  /* Line Break */
								else if (spi_rx_buffer3[3] == 0x05) gSensorStatus[0] = 0x10;  /* Not Matched */
								else if (spi_rx_buffer3[3] == 0x8B) gSensorStatus[0] = 0x10;  /* Not Matched */
								else if (spi_rx_buffer3[3] == 0xC3) gSensorStatus[0] = 0x02;  /* Over Range */
								else                                gSensorStatus[0] = 0x01;  /* Fault */
#else
								gSensorStatus[0] = spi_rx_buffer3[3];
#endif
							}
							else if (gSensorType[0] == SENSOR_TYPE_TC)
							{
								if (spi_rx_buffer3[3] == 0x01) {
									gSensorValue[0]  = CalTemp(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
									gSensorStatus[0] = 0x00;
								}
								else if ((spi_rx_buffer3[3] == 0xFB) || (spi_rx_buffer3[3] == 0xEB) ||
								         (spi_rx_buffer3[3] == 0xCB) || (spi_rx_buffer3[3] == 0x09) ||
								         (spi_rx_buffer3[3] == 0x89)) {
									gSensorStatus[0] = 0x02;  /* Not Connected */
									gSensorValue[0]  = 0;
								}
								else if (spi_rx_buffer3[3] == 0xB5) {
									gSensorStatus[0] = 0x10;  /* ADC Error */
									gSensorValue[0]  = 0;
								}
								else if (spi_rx_buffer3[3] == 0x11) {
									gSensorStatus[0] = 0x10;
									gSensorValue[0]  = 0;
								}
								else {
									gSensorValue[0] = 0;
								}
							}
							else if (gSensorType[0] == SENSOR_TYPE_CT)
							{
								if ((spi_rx_buffer3[3] == 0x01) || (spi_rx_buffer3[3] == 0xCB) ||
								    (spi_rx_buffer3[3] == 0xB1)) {
									gSensorValue[0]  = 255;
									gSensorStatus[0] = 0x00;
								}
								else {
#if DEBUG_STATUS == 1
									gSensorValue[0]  = 0;
									gSensorStatus[0] = 0x00;
									if ((spi_rx_buffer3[3] == 0xBB) || (spi_rx_buffer3[3] == 0x8B))
										gSensorStatus[0] = 0x10;  /* Miss Connected */
#else
									gSensorStatus[0] = spi_rx_buffer3[3];
#endif
								}
							}
							else if (gSensorType[0] == SENSOR_TYPE_MA)
							{
								gSensorValue[0] = CalCurrent(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
#if DEBUG_STATUS == 1
								if (spi_rx_buffer3[3] == 0x01) {
									if ((spi_rx_buffer3[4] == 0x00) && (spi_rx_buffer3[5] == 0x00)) {
										gSensorStatus[0] = 0x08;  /* Not Connected */
										gSensorValue[0]  = 0;
									}
									else if ((spi_rx_buffer3[4] == 0xFF) && (spi_rx_buffer3[5] == 0xFF)) {
										gSensorStatus[0] = 0x08;  /* Not Connected */
										gSensorValue[0]  = 0;
									}
									else {
										if ((gSensorValue[0] > 44000) || (gSensorValue[0] < 4000))
											gSensorStatus[0] = 0x02;  /* Over Range */
										else
											gSensorStatus[0] = 0x00;  /* Normal */
									}
								}
								else if ((spi_rx_buffer3[3] == 0x0B) || (spi_rx_buffer3[3] == 0xCB)) {
									if ((gSensorValue[0] > 44000) || (gSensorValue[0] < 4000))
										gSensorStatus[0] = 0x02;  /* Over Range */
									else
										gSensorStatus[0] = 0x00;  /* Normal */
								}
								else if (spi_rx_buffer3[3] == 0xFE) {
									gSensorStatus[0] = 0x08;  /* Not Connected */
									gSensorValue[0]  = 0;
								}
#else
								gSensorStatus[0] = spi_rx_buffer3[3];
#endif
							}
							ST_ADC[2] = ST_READ_SEND1;
						}
						break;

					/* ---- Channel 1 (ADC ch9) ---- */
					case ST_READ_SEND1:
						SPI_Complt[2] = 0;
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
						HAL_SPI_TransmitReceive_IT(&hspi3, ADC_R_CMD_BF[9], spi_rx_buffer3, 7);
						ST_ADC[2] = ST_READ_END1;
						break;

					case ST_READ_END1:
						if (HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_READY)
						{
							if (gSensorType[1] == SENSOR_TYPE_PT100)
							{
#if DEBUG_STATUS == 1
								if (spi_rx_buffer3[3] == 0x01) {
									gSensorValue[1] = CalTemp(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
									if      (gSensorValue[1] == 0)    gSensorStatus[1] = 0x02;  /* Over Range */
									else if (gSensorValue[1] > 6535)  gSensorStatus[1] = 0x02;  /* Over Range */
									else                              gSensorStatus[1] = 0x00;  /* Normal */
								}
								else if ((spi_rx_buffer3[3] == 0x81) || (spi_rx_buffer3[3] == 0x85) ||
								         (spi_rx_buffer3[3] == 0x89) || (spi_rx_buffer3[3] == 0x09)) {
									gSensorValue[1]  = 0;
									gSensorStatus[1] = 0x10;  /* Not Connected */
								}
								else if (spi_rx_buffer3[3] == 0xCB) {
									gSensorValue[1]  = 0;
									gSensorStatus[1] = 0x08;  /* Line Break */
								}
								else if (spi_rx_buffer3[3] == 0x8B) {
									gSensorValue[1]  = 0;
									gSensorStatus[1] = 0x08;  /* Line Break */
								}
								else if (spi_rx_buffer3[3] == 0x05) {
									gSensorValue[1]  = 0;
									gSensorStatus[1] = 0x04;  /* Config Mismatch */
								}
								else {
									gSensorStatus[1] = 0x01;  /* Fault */
									gSensorValue[1]  = 0;
								}
#else
								gSensorStatus[1] = spi_rx_buffer3[3];
#endif
							}
							else if (gSensorType[1] == SENSOR_TYPE_PT1000)
							{
								gSensorValue[1] = CalTemp(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
#if DEBUG_STATUS == 1
								if ((spi_rx_buffer3[3] == 0x01) || (spi_rx_buffer3[3] == 0x03)) {
									if (g_TempSensorStatus[9] == 0xCB) {
										if (gSensorValue[1] < 32775) {
											if (gSensorValue[1] > 6500) gSensorStatus[1] = 0x02;  /* Over Range */
											else                        gSensorStatus[1] = 0x00;  /* Normal */
										}
										else gSensorStatus[1] = 0x00;  /* Normal */
									}
									else if ((g_TempSensorStatus[9] == 0x81) || (g_TempSensorStatus[9] == 0x85))
										gSensorStatus[1] = 0x08;  /* Line Break */
									else if (g_TempSensorStatus[9] == 0x01)
										gSensorStatus[1] = 0x04;  /* Config Mismatch */
								}
								else if ((spi_rx_buffer3[3] == 0x81) || (spi_rx_buffer3[3] == 0x85)) {
									if ((g_TempSensorStatus[9] == 0x81) || (g_TempSensorStatus[9] == 0x85) ||
									    (g_TempSensorStatus[9] == 0x89))
										gSensorStatus[1] = 0x18;  /* Line Break or Not Matched */
									else if (g_TempSensorStatus[9] == 0x8B)
										gSensorStatus[1] = 0x10;  /* Not Matched */
								}
								else if (spi_rx_buffer3[3] == 0xCB) {
									if ((g_TempSensorStatus[9] == 0xCB) || (g_TempSensorStatus[9] == 0xEB))
										gSensorStatus[1] = 0x08;  /* Line Break */
									else
										gSensorStatus[1] = 0x10;  /* Not Matched */
								}
								else if (spi_rx_buffer3[3] == 0x83) gSensorStatus[1] = 0x08;  /* Line Break */
								else if (spi_rx_buffer3[3] == 0x05) gSensorStatus[1] = 0x10;  /* Not Matched */
								else if (spi_rx_buffer3[3] == 0x8B) gSensorStatus[1] = 0x10;  /* Not Matched */
								else if (spi_rx_buffer3[3] == 0xC3) gSensorStatus[1] = 0x02;  /* Over Range */
								else                                gSensorStatus[1] = 0x01;  /* Fault */
#else
								gSensorStatus[1] = spi_rx_buffer3[3];
#endif
							}
							else if (gSensorType[1] == SENSOR_TYPE_TC)
							{
								if (spi_rx_buffer3[3] == 0x01) {
									gSensorValue[1]  = CalTemp(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
									gSensorStatus[1] = 0x00;
								}
								else if ((spi_rx_buffer3[3] == 0xFB) || (spi_rx_buffer3[3] == 0xEB) ||
								         (spi_rx_buffer3[3] == 0xCB) || (spi_rx_buffer3[3] == 0x09) ||
								         (spi_rx_buffer3[3] == 0x89)) {
									gSensorStatus[1] = 0x02;  /* Not Connected */
									gSensorValue[1]  = 0;
								}
								else if (spi_rx_buffer3[3] == 0xB5) {
									gSensorStatus[1] = 0x10;  /* ADC Error */
									gSensorValue[1]  = 0;
								}
							}
							else if (gSensorType[1] == SENSOR_TYPE_CT)
							{
								if ((spi_rx_buffer3[3] == 0x01) || (spi_rx_buffer3[3] == 0xCB) ||
								    (spi_rx_buffer3[3] == 0xB1)) {
									gSensorValue[1]  = 255;
									gSensorStatus[1] = 0x00;
								}
								else {
#if DEBUG_STATUS == 1
									gSensorValue[1]  = 0;
									gSensorStatus[1] = 0x00;
									if ((spi_rx_buffer3[3] == 0xBB) || (spi_rx_buffer3[3] == 0x8B))
										gSensorStatus[1] = 0x10;  /* Miss Connected */
#else
									gSensorStatus[1] = spi_rx_buffer3[3];
#endif
								}
							}
							else if (gSensorType[1] == SENSOR_TYPE_MA)
							{
								gSensorValue[1] = CalCurrent(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
#if DEBUG_STATUS == 1
								if (spi_rx_buffer3[3] == 0x01) {
									if ((spi_rx_buffer3[4] == 0x00) && (spi_rx_buffer3[5] == 0x00)) {
										gSensorStatus[1] = 0x08;  /* Not Connected */
										gSensorValue[1]  = 0;
									}
									else if ((spi_rx_buffer3[4] == 0xFF) && (spi_rx_buffer3[5] == 0xFF)) {
										gSensorStatus[1] = 0x08;  /* Not Connected */
										gSensorValue[1]  = 0;
									}
									else {
										if ((gSensorValue[1] > 44000) || (gSensorValue[1] < 4000))
											gSensorStatus[1] = 0x02;  /* Over Range */
										else
											gSensorStatus[1] = 0x00;  /* Normal */
									}
								}
								else if ((spi_rx_buffer3[3] == 0x0B) || (spi_rx_buffer3[3] == 0xCB)) {
									if ((gSensorValue[1] > 44000) || (gSensorValue[1] < 4000))
										gSensorStatus[1] = 0x02;  /* Over Range */
									else
										gSensorStatus[1] = 0x00;  /* Normal */
								}
								else if (spi_rx_buffer3[3] == 0xFE) {
									gSensorStatus[1] = 0x08;  /* Not Connected */
									gSensorValue[1]  = 0;
								}
#else
								gSensorStatus[1] = spi_rx_buffer3[3];
#endif
							}
							ST_ADC[2] = ST_READ_SEND2;
						}
						break;

					/* ---- Channel 2 (ADC ch10) ---- */
					case ST_READ_SEND2:
						SPI_Complt[2] = 0;
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
						HAL_SPI_TransmitReceive_IT(&hspi3, ADC_R_CMD_BF[10], spi_rx_buffer3, 7);
						ST_ADC[2] = ST_READ_END2;
						break;

					case ST_READ_END2:
						if (HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_READY)
						{
							if (gSensorType[2] == SENSOR_TYPE_PT100)
							{
#if DEBUG_STATUS == 1
								if (spi_rx_buffer3[3] == 0x01) {
									gSensorValue[2] = CalTemp(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
									if      (gSensorValue[2] == 0)    gSensorStatus[2] = 0x02;  /* Over Range */
									else if (gSensorValue[0] > 6535)  gSensorStatus[2] = 0x02;  /* Over Range */
									else                              gSensorStatus[2] = 0x00;  /* Normal */
								}
								else if ((spi_rx_buffer3[3] == 0x81) || (spi_rx_buffer3[3] == 0x85) ||
								         (spi_rx_buffer3[3] == 0x89) || (spi_rx_buffer3[3] == 0x09)) {
									gSensorValue[2]  = 0;
									gSensorStatus[2] = 0x10;  /* Not Connected */
								}
								else if (spi_rx_buffer3[3] == 0xCB) {
									gSensorValue[2]  = 0;
									gSensorStatus[2] = 0x08;  /* Line Break */
								}
								else if (spi_rx_buffer3[3] == 0x8B) {
									gSensorValue[2]  = 0;
									gSensorStatus[2] = 0x08;  /* Line Break */
								}
								else if (spi_rx_buffer3[3] == 0x05) {
									gSensorValue[2]  = 0;
									gSensorStatus[2] = 0x04;  /* Config Mismatch */
								}
								else {
									gSensorStatus[2] = 0x01;  /* Fault */
									gSensorValue[2]  = 0;
								}
#else
								gSensorStatus[2] = spi_rx_buffer3[3];
#endif
							}
							else if (gSensorType[2] == SENSOR_TYPE_PT1000)
							{
								gSensorValue[2] = CalTemp(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
#if DEBUG_STATUS == 1
								if ((spi_rx_buffer3[3] == 0x01) || (spi_rx_buffer3[3] == 0x03)) {
									if (g_TempSensorStatus[10] == 0xCB) {
										if (gSensorValue[2] < 32775) {
											if (gSensorValue[2] > 6500) gSensorStatus[2] = 0x02;  /* Over Range */
											else                        gSensorStatus[2] = 0x00;  /* Normal */
										}
										else gSensorStatus[2] = 0x00;  /* Normal */
									}
									else if ((g_TempSensorStatus[10] == 0x81) || (g_TempSensorStatus[10] == 0x85))
										gSensorStatus[2] = 0x08;  /* Line Break */
									else if (g_TempSensorStatus[10] == 0x01)
										gSensorStatus[2] = 0x04;  /* Config Mismatch */
								}
								else if ((spi_rx_buffer3[3] == 0x81) || (spi_rx_buffer3[3] == 0x85)) {
									if ((g_TempSensorStatus[10] == 0x81) || (g_TempSensorStatus[10] == 0x85) ||
									    (g_TempSensorStatus[10] == 0x89))
										gSensorStatus[2] = 0x18;  /* Line Break or Not Matched */
									else if (g_TempSensorStatus[10] == 0x8B)
										gSensorStatus[2] = 0x10;  /* Not Matched */
								}
								else if (spi_rx_buffer3[3] == 0xCB) {
									if ((g_TempSensorStatus[10] == 0xCB) || (g_TempSensorStatus[10] == 0xEB))
										gSensorStatus[2] = 0x08;  /* Line Break */
									else
										gSensorStatus[2] = 0x10;  /* Not Matched */
								}
								else if (spi_rx_buffer3[3] == 0x83) gSensorStatus[2] = 0x08;  /* Line Break */
								else if (spi_rx_buffer3[3] == 0x05) gSensorStatus[2] = 0x10;  /* Not Matched */
								else if (spi_rx_buffer3[3] == 0x8B) gSensorStatus[2] = 0x10;  /* Not Matched */
								else if (spi_rx_buffer3[3] == 0xC3) gSensorStatus[2] = 0x02;  /* Over Range */
								else                                gSensorStatus[2] = 0x01;  /* Fault */
#else
								gSensorStatus[2] = spi_rx_buffer3[3];
#endif
							}
							else if (gSensorType[2] == SENSOR_TYPE_TC)
							{
								if (spi_rx_buffer3[3] == 0x01) {
									gSensorValue[2]  = CalTemp(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
									gSensorStatus[2] = 0x00;
								}
								else if ((spi_rx_buffer3[3] == 0xFB) || (spi_rx_buffer3[3] == 0xEB) ||
								         (spi_rx_buffer3[3] == 0xCB) || (spi_rx_buffer3[3] == 0x09) ||
								         (spi_rx_buffer3[3] == 0x89)) {
									gSensorStatus[2] = 0x02;  /* Not Connected */
									gSensorValue[2]  = 0;
								}
								else if (spi_rx_buffer3[3] == 0xB5) {
									gSensorStatus[2] = 0x10;  /* ADC Error */
									gSensorValue[2]  = 0;
								}
							}
							else if (gSensorType[2] == SENSOR_TYPE_CT)
							{
								if ((spi_rx_buffer3[3] == 0x01) || (spi_rx_buffer3[3] == 0xCB) ||
								    (spi_rx_buffer3[3] == 0xB1)) {
									gSensorValue[2]  = 255;
									gSensorStatus[2] = 0x00;
								}
								else {
#if DEBUG_STATUS == 1
									gSensorValue[2]  = 0;
									gSensorStatus[2] = 0x00;
									if ((spi_rx_buffer3[3] == 0xBB) || (spi_rx_buffer3[3] == 0x8B))
										gSensorStatus[2] = 0x10;  /* Miss Connected */
#else
									gSensorStatus[2] = spi_rx_buffer3[3];
#endif
								}
							}
							else if (gSensorType[2] == SENSOR_TYPE_MA)
							{
								gSensorValue[2] = CalCurrent(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
#if DEBUG_STATUS == 1
								if (spi_rx_buffer3[3] == 0x01) {
									if ((spi_rx_buffer3[4] == 0x00) && (spi_rx_buffer3[5] == 0x00)) {
										gSensorStatus[2] = 0x08;  /* Not Connected */
										gSensorValue[2]  = 0;
									}
									else if ((spi_rx_buffer3[4] == 0xFF) && (spi_rx_buffer3[5] == 0xFF)) {
										gSensorStatus[2] = 0x08;  /* Not Connected */
										gSensorValue[2]  = 0;
									}
									else {
										if ((gSensorValue[2] > 44000) || (gSensorValue[2] < 4000))
											gSensorStatus[2] = 0x02;  /* Over Range */
										else
											gSensorStatus[2] = 0x00;  /* Normal */
									}
								}
								else if ((spi_rx_buffer3[3] == 0x0B) || (spi_rx_buffer3[3] == 0xCB)) {
									if ((gSensorValue[2] > 44000) || (gSensorValue[2] < 4000))
										gSensorStatus[2] = 0x02;  /* Over Range */
									else
										gSensorStatus[2] = 0x00;  /* Normal */
								}
								else if (spi_rx_buffer3[3] == 0xFE) {
									gSensorStatus[2] = 0x08;  /* Not Connected */
									gSensorValue[2]  = 0;
								}
#else
								gSensorStatus[2] = spi_rx_buffer3[3];
#endif
							}
							ST_ADC[2] = ST_READ_SEND3;
						}
						break;

					/* ---- Channel 3 (ADC ch11) ---- */
					case ST_READ_SEND3:
						SPI_Complt[2] = 0;
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
						HAL_SPI_TransmitReceive_IT(&hspi3, ADC_R_CMD_BF[11], spi_rx_buffer3, 7);
						ST_ADC[2] = ST_READ_END3;
						break;

					case ST_READ_END3:
						if (HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_READY)
						{
							if (gSensorType[3] == SENSOR_TYPE_PT100)
							{
#if DEBUG_STATUS == 1
								if (spi_rx_buffer3[3] == 0x01) {
									gSensorValue[3] = CalTemp(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
									if      (gSensorValue[3] == 0)    gSensorStatus[3] = 0x02;  /* Over Range */
									else if (gSensorValue[3] > 6535)  gSensorStatus[3] = 0x02;  /* Over Range */
									else                              gSensorStatus[3] = 0x00;  /* Normal */
								}
								else if ((spi_rx_buffer3[3] == 0x81) || (spi_rx_buffer3[3] == 0x85) ||
								         (spi_rx_buffer3[3] == 0x89)) {
									gSensorValue[3]  = 0;
									gSensorStatus[3] = 0x10;  /* Not Connected */
								}
								else if (spi_rx_buffer3[3] == 0xCB) {
									gSensorValue[3]  = 0;
									gSensorStatus[3] = 0x08;  /* Line Break */
								}
								else if (spi_rx_buffer3[3] == 0x8B) {
									gSensorValue[3]  = 0;
									gSensorStatus[3] = 0x08;  /* Line Break */
								}
								else if (spi_rx_buffer3[3] == 0x05) {
									gSensorValue[3]  = 0;
									gSensorStatus[3] = 0x04;  /* Config Mismatch */
								}
								else {
									gSensorStatus[3] = 0x01;  /* Fault */
									gSensorValue[3]  = 0;
								}
#else
								gSensorStatus[3] = spi_rx_buffer3[3];
#endif
							}
							else if (gSensorType[3] == SENSOR_TYPE_PT1000)
							{
								gSensorValue[3] = CalTemp(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
#if DEBUG_STATUS == 1
								if ((spi_rx_buffer3[3] == 0x01) || (spi_rx_buffer3[3] == 0x03)) {
									if (g_TempSensorStatus[11] == 0xCB) {
										if (gSensorValue[3] < 32775) {
											if (gSensorValue[3] > 6500) gSensorStatus[3] = 0x02;  /* Over Range */
											else                        gSensorStatus[3] = 0x00;  /* Normal */
										}
										else gSensorStatus[3] = 0x00;  /* Normal */
									}
									else if ((g_TempSensorStatus[11] == 0x81) || (g_TempSensorStatus[11] == 0x85))
										gSensorStatus[3] = 0x08;  /* Line Break */
									else if (g_TempSensorStatus[11] == 0x01)
										gSensorStatus[3] = 0x04;  /* Config Mismatch */
								}
								else if ((spi_rx_buffer3[3] == 0x81) || (spi_rx_buffer3[3] == 0x85)) {
									if ((g_TempSensorStatus[11] == 0x81) || (g_TempSensorStatus[11] == 0x85))
										gSensorStatus[3] = 0x18;  /* Line Break or Not Matched */
									else if (g_TempSensorStatus[11] == 0x8B)
										gSensorStatus[3] = 0x10;  /* Not Matched */
								}
								else if (spi_rx_buffer3[3] == 0xCB) {
									if ((g_TempSensorStatus[11] == 0xCB) || (g_TempSensorStatus[11] == 0xEB))
										gSensorStatus[3] = 0x08;  /* Line Break */
									else
										gSensorStatus[3] = 0x10;  /* Not Matched */
								}
								else if (spi_rx_buffer3[3] == 0x83) gSensorStatus[3] = 0x08;  /* Line Break */
								else if (spi_rx_buffer3[3] == 0x05) gSensorStatus[3] = 0x10;  /* Not Matched */
								else if (spi_rx_buffer3[3] == 0x8B) gSensorStatus[3] = 0x10;  /* Not Matched */
								else if (spi_rx_buffer3[3] == 0xC3) gSensorStatus[3] = 0x02;  /* Over Range */
								else                                gSensorStatus[3] = 0x01;  /* Fault */
#else
								gSensorStatus[3] = spi_rx_buffer3[3];
#endif
							}
							else if (gSensorType[3] == SENSOR_TYPE_TC)
							{
								if (spi_rx_buffer3[3] == 0x01) {
									gSensorValue[3]  = CalTemp(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
									gSensorStatus[3] = 0x00;
								}
								else if ((spi_rx_buffer3[3] == 0xFB) || (spi_rx_buffer3[3] == 0xEB) ||
								         (spi_rx_buffer3[3] == 0xCB) || (spi_rx_buffer3[3] == 0x09) ||
								         (spi_rx_buffer3[3] == 0x89)) {
									gSensorStatus[3] = 0x02;  /* Not Connected */
									gSensorValue[3]  = 0;
								}
								else if (spi_rx_buffer3[3] == 0xB5) {
									gSensorStatus[3] = 0x10;  /* ADC Error */
									gSensorValue[3]  = 0;
								}
							}
							else if (gSensorType[3] == SENSOR_TYPE_CT)
							{
								if ((spi_rx_buffer3[3] == 0x01) || (spi_rx_buffer3[3] == 0xCB) ||
								    (spi_rx_buffer3[3] == 0xB1)) {
									gSensorValue[3]  = 255;
									gSensorStatus[3] = 0x00;
								}
								else {
#if DEBUG_STATUS == 1
									gSensorValue[3]  = 0;
									gSensorStatus[3] = 0x00;
									if ((spi_rx_buffer3[3] == 0xBB) || (spi_rx_buffer3[3] == 0x8B))
										gSensorStatus[3] = 0x10;  /* Miss Connected */
#else
									gSensorStatus[3] = spi_rx_buffer3[3];
#endif
								}
							}
							else if (gSensorType[3] == SENSOR_TYPE_MA)
							{
								gSensorValue[3] = CalCurrent(spi_rx_buffer3[4], spi_rx_buffer3[5], spi_rx_buffer3[6]);
								gSensorStatus[3] = spi_rx_buffer3[3];
#if DEBUG_STATUS == 1
								if (spi_rx_buffer3[3] == 0x01) {
									if ((spi_rx_buffer3[4] == 0x00) && (spi_rx_buffer3[5] == 0x00)) {
										gSensorStatus[3] = 0x08;  /* Not Connected */
										gSensorValue[3]  = 0;
									}
									else if ((spi_rx_buffer3[4] == 0xFF) && (spi_rx_buffer3[5] == 0xFF)) {
										gSensorStatus[3] = 0x08;  /* Not Connected */
										gSensorValue[3]  = 0;
									}
									else {
										if ((gSensorValue[3] > 44000) || (gSensorValue[3] < 4000))
											gSensorStatus[3] = 0x02;  /* Over Range */
										else
											gSensorStatus[3] = 0x00;  /* Normal */
									}
								}
								else if ((spi_rx_buffer3[3] == 0x0B) || (spi_rx_buffer3[3] == 0xCB)) {
									if ((gSensorValue[3] > 44000) || (gSensorValue[3] < 4000))
										gSensorStatus[3] = 0x02;  /* Over Range */
									else
										gSensorStatus[3] = 0x00;  /* Normal */
								}
								else if (spi_rx_buffer3[3] == 0xFE) {
									gSensorStatus[3] = 0x08;  /* Not Connected */
									gSensorValue[3]  = 0;
								}
#else
								gSensorStatus[3] = spi_rx_buffer3[3];
#endif
							}
							ST_ADC[2] = ST_START_SEND;
							gCnt[2]++;
						}
						break;

					} /* switch ST_ADC[2] */

					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
				}
			} /* case DAQ_MOD */

		} /* switch dcu_mode */
	}
}
/* USER CODE END 3 */
/* ##################직접작성! 끝###################### *//* ##################직접작성! 끝###################### *//* ##################직접작성! 끝###################### */
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM       = 10;
	RCC_OscInitStruct.PLL.PLLN       = 80;
	RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ       = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		Error_Handler();

	RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                                 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
		Error_Handler();
}

/**
  * @brief SPI3 Initialization Function
  * @retval None
  */
static void MX_SPI3_Init(void)
{
	/* USER CODE BEGIN SPI3_Init 0 */
	/* USER CODE END SPI3_Init 0 */
	/* USER CODE BEGIN SPI3_Init 1 */
	/* USER CODE END SPI3_Init 1 */

	hspi3.Instance               = SPI3;
	hspi3.Init.Mode              = SPI_MODE_MASTER;
	hspi3.Init.Direction         = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize          = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity       = SPI_POLARITY_HIGH;
	hspi3.Init.CLKPhase          = SPI_PHASE_2EDGE;
	hspi3.Init.NSS               = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi3.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode            = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial     = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
		Error_Handler();

	/* USER CODE BEGIN SPI3_Init 2 */
	/* USER CODE END SPI3_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
	/* USER CODE BEGIN USART1_Init 0 */
	/* USER CODE END USART1_Init 0 */
	/* USER CODE BEGIN USART1_Init 1 */
	/* USER CODE END USART1_Init 1 */

	huart1.Instance          = USART1;
	huart1.Init.BaudRate     = 115200;
	huart1.Init.WordLength   = UART_WORDLENGTH_8B;
	huart1.Init.StopBits     = UART_STOPBITS_1;
	huart1.Init.Parity       = UART_PARITY_NONE;
	huart1.Init.Mode         = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
		Error_Handler();

	/* USER CODE BEGIN USART1_Init 2 */
	/* USER CODE END USART1_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
	/* USER CODE BEGIN USART2_Init 0 */
	/* USER CODE END USART2_Init 0 */
	/* USER CODE BEGIN USART2_Init 1 */
	/* USER CODE END USART2_Init 1 */

	huart2.Instance          = USART2;
	huart2.Init.BaudRate     = 115200;
	huart2.Init.WordLength   = UART_WORDLENGTH_8B;
	huart2.Init.StopBits     = UART_STOPBITS_1;
	huart2.Init.Parity       = UART_PARITY_NONE;
	huart2.Init.Mode         = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
		Error_Handler();

	/* USER CODE BEGIN USART2_Init 2 */
	/* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15 | GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

	/* PC0–PC7: input (sensor type DIP switches) */
	GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
	                     | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* PA1: RS-485 TX enable */
	GPIO_InitStruct.Pin   = GPIO_PIN_1;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* PE8–PE15: input (board ID / sensor type DIP) */
	GPIO_InitStruct.Pin  = GPIO_PIN_8  | GPIO_PIN_9  | GPIO_PIN_10 | GPIO_PIN_11
	                     | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* PB8, PB9, PB10: LED outputs */
	GPIO_InitStruct.Pin   = GPIO_PIN_10 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* PD10, PD11, PD0: ADC /DRDY interrupt inputs */
	GPIO_InitStruct.Pin  = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* PD15, PD1: outputs */
	GPIO_InitStruct.Pin   = GPIO_PIN_15 | GPIO_PIN_1;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* PC9: SPI3 CS (software-controlled, needs high speed) */
	GPIO_InitStruct.Pin   = GPIO_PIN_9;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* PD2–PD5: input */
	GPIO_InitStruct.Pin  = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### */

void RTD_Configuration(int type, uint8_t channel, int sensor, int current, int curve, uint8_t *return_data)
{
	return_data[0] = (type << 3) + (channel >> 2);
	return_data[1] = ((0x3 & channel) << 6) + (sensor << 2) + (current >> 2);
	return_data[2] = ((0x3 & current) << 6) + (curve << 4);
	return_data[3] = 0x00;
}

void RTD_Sense_Registor(int type, float registor, uint8_t *return_data)
{
	unsigned int SensorValue = (int)(registor / 0.0009765625);
	return_data[3] = SensorValue & 0xFF;
	return_data[2] = (SensorValue >> 8)  & 0xFF;
	return_data[1] = (SensorValue >> 16) & 0xFF;
	return_data[0] = (type << 3) | ((SensorValue >> 24) & 0x07);
}

void TC_Configuration(int type, int channel, int configuration, uint8_t *return_data)
{
	return_data[0] = (type << 3) + (channel >> 2);
	return_data[1] = ((0x3 & channel) << 6) + (configuration << 2);
	return_data[2] = 0x00;
	return_data[3] = 0x00;
}

void TC_Assignment(int type, uint8_t in_type, uint8_t in_line, uint8_t average, uint8_t current, float factor, uint8_t *return_data)
{
	unsigned int IdealityFactor = (int)(factor / 0.00000095367431640625);
	return_data[3] = IdealityFactor & 0xFF;
	return_data[2] = (IdealityFactor >> 8) & 0xFF;
	return_data[1] = (current << 6) | ((IdealityFactor >> 16) & 0xFF);
	return_data[0] = (type << 3) + (in_type * 4) + (in_line * 2) + average;
}

void Setting_Write_Data(int channel, uint8_t *return_data, uint8_t count, uint8_t *write_data)
{
	write_data[2] = channel + count;
	write_data[3] = return_data[count];
}

void ADC_MultiConversion_Set(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, gMultiConversionSet0, 4, 50);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, gMultiConversionSet1, 4, 50);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(2);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, gMultiConversionSet2, 4, 50);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(2);
}

void SPI3_Write(int channel, uint8_t *return_data, uint8_t *write_data)
{
	int i;
	for (i = 0; i < 4; i++) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		Setting_Write_Data(SetChannel[channel], return_data, i, write_data);
		HAL_SPI_Transmit(&hspi3, write_data, 4, 50);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_Delay(2);
	}
	/* Re-send word 0 to latch the configuration */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	Setting_Write_Data(SetChannel[channel], return_data, 0, write_data);
	HAL_SPI_Transmit(&hspi3, write_data, 4, 50);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(2);
}

void SPI3_Read(int channel, int sensor, uint8_t *return_data)
{
	int set_read3;

	channel = channel - 4;
	if      (sensor == 1)                       set_read3 = (channel * 5) - 1;
	else if ((sensor == 2) | (sensor == 3))     set_read3 = (channel * 5) - 2;
	else if  (sensor == 4)                      set_read3 = (channel * 5) - 4;

	conversion_start3[3] = 0x80 + set_read3;
	read_data3[2]        = ReadChannel[set_read3];

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, read_data3, 3, 50);
	HAL_SPI_Receive(&hspi3, spi_set_buffer3, 1, 50);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(1);

	read_data3[2] = ReadChannel[set_read3] + 1;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, read_data3, 3, 50);
	HAL_SPI_Receive(&hspi3, spi_set_buffer3 + 1, 1, 50);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(1);

	read_data3[2] = ReadChannel[set_read3] + 2;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, read_data3, 3, 50);
	HAL_SPI_Receive(&hspi3, spi_set_buffer3 + 2, 1, 50);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(1);

	read_data3[2] = ReadChannel[set_read3] + 3;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, read_data3, 3, 50);
	HAL_SPI_Receive(&hspi3, spi_set_buffer3 + 3, 1, 50);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(1);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, conversion_start3, 4, 50);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(3);

	temp3 = spi_set_buffer3[1] * 64 + spi_set_buffer3[2] / 4;
	HAL_Delay(1);
}

void Channel_Setting(int adc, int ch, int type)
{
	int channel_set, resistor_set;
	int Tadd = 0;

	/* Default channel 0 (cold-junction reference) */
	ADC_S_CMD_BF[(adc - 1) * 4 + (ch - 1)][0] = 0x02;
	ADC_S_CMD_BF[(adc - 1) * 4 + (ch - 1)][1] = 0x00;
	ADC_R_CMD_BF[(adc - 1) * 4 + (ch - 1)][0] = 0x03;
	ADC_R_CMD_BF[(adc - 1) * 4 + (ch - 1)][1] = 0x00;
	ADC_R_CMD_BF[(adc - 1) * 4 + (ch - 1)][3] = 0x00;

	resistor_set = 2;
	channel_set  = 4;
	RTD_Configuration(PT100, resistor_set, Three_External, RTD_Current_1mA, American, return_data);
	HAL_Delay(10);
	SPI3_Write(channel_set, return_data, write_data);
	RTD_Sense_Registor(RESISTOR, 100, return_data);
	HAL_Delay(10);
	SPI3_Write(resistor_set, return_data, write_data);
	HAL_Delay(10);
	gMultiConversionSet0[3] |= 0x08;
	Tadd = 0x10 + 4 * 3; //0x1C
	ADC_R_CMD_BF[(adc - 1) * 4 + (ch - 1)][2] = Tadd & 0xFF;

	switch (adc)
	{
	case 3:
		if ((type == SENSOR_TYPE_PT100) || (type == SENSOR_TYPE_PT1000))
		{
			resistor_set = (ch * 4) + 2;
			channel_set  = (ch * 4) + 4;

			if (type == SENSOR_TYPE_PT100)
				RTD_Configuration(PT100,  resistor_set, Three_External, RTD_Current_1mA, American, return_data);
			else
				RTD_Configuration(PT1000, resistor_set, Three_External, RTD_Current_1mA, American, return_data);

			HAL_Delay(10);
			SPI3_Write(channel_set, return_data, write_data);
			RTD_Sense_Registor(RESISTOR, 100, return_data);
			HAL_Delay(10);
			SPI3_Write(resistor_set, return_data, write_data);
			HAL_Delay(10);

			if      (ch == 1) gMultiConversionSet0[3] |= 0x80;
			else if (ch == 2) gMultiConversionSet1[3] |= 0x08;
			else if (ch == 3) gMultiConversionSet1[3] |= 0x80;
			else if (ch == 4) gMultiConversionSet2[3] |= 0x08;

			Tadd = 0x10 + 4 * (7 + (ch - 1) * 4);
			ADC_R_CMD_BF[(adc - 1) * 4 + (ch - 1)][2] = Tadd & 0xFF;
		}
		else if (type == SENSOR_TYPE_CT)
		{
			resistor_set = (ch * 4) + 2;
			channel_set  = (ch * 4) + 4;

			RTD_Configuration(PT100, resistor_set, Three_External, RTD_Current_1mA, American, return_data);
			HAL_Delay(10);
			SPI3_Write(channel_set, return_data, write_data);
			RTD_Sense_Registor(RESISTOR, 100, return_data);
			HAL_Delay(10);
			SPI3_Write(resistor_set, return_data, write_data);
			HAL_Delay(10);

			if      (ch == 1) gMultiConversionSet0[3] |= 0x80;
			else if (ch == 2) gMultiConversionSet1[3] |= 0x08;
			else if (ch == 3) gMultiConversionSet1[3] |= 0x80;
			else if (ch == 4) gMultiConversionSet2[3] |= 0x08;

			Tadd = 0x10 + 4 * (7 + (ch - 1) * 4);
			ADC_R_CMD_BF[(adc - 1) * 4 + (ch - 1)][2] = Tadd & 0xFF;
		}
		else if (type == SENSOR_TYPE_TC)
		{
			resistor_set = 4;
			channel_set  = (ch * 4) + 3;

			TC_Configuration(Type_K, resistor_set, Diff_100uA, return_data);
			HAL_Delay(10);
			SPI3_Write(channel_set, return_data, write_data);
			HAL_Delay(10);

			if      (ch == 1) gMultiConversionSet0[3] |= 0x40;
			else if (ch == 2) gMultiConversionSet1[3] |= 0x04;
			else if (ch == 3) gMultiConversionSet1[3] |= 0x40;
			else if (ch == 4) gMultiConversionSet2[3] |= 0x04;

			Tadd = 0x10 + 4 * (6 + (ch - 1) * 4);
			ADC_R_CMD_BF[(adc - 1) * 4 + (ch - 1)][2] = Tadd & 0xFF;
		}
		else if (type == SENSOR_TYPE_MA)
		{
			channel_set    = (ch * 4) + 2;
			return_data[0] = 0xF0;
			return_data[1] = 0x00;
			return_data[2] = 0x00;
			return_data[3] = 0x00;
			SPI3_Write(channel_set, return_data, write_data);

			if      (ch == 1) gMultiConversionSet0[3] |= 0x20;
			else if (ch == 2) gMultiConversionSet1[3] |= 0x02;
			else if (ch == 3) gMultiConversionSet1[3] |= 0x20;
			else if (ch == 4) gMultiConversionSet2[3] |= 0x02;

			Tadd = 0x10 + 4 * (5 + (ch - 1) * 4);
			ADC_R_CMD_BF[(adc - 1) * 4 + (ch - 1)][2] = Tadd & 0xFF;
		}
		break;
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI3)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		TransmitComplete[2] = 1;
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI3)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		SPI_Complt[2] = 1;
	}
}

void SensorDataSend(int ADCNumber)
{
	U2_TX_buffer[0]  = 'T';
	U2_TX_buffer[27] = 'X';
	U2_TX_buffer[28] = 'Y';
	U2_TX_buffer[29] = 'Z';

	temp1 = gSensorValue[ADCNumber * 4 + 0];
	temp2 = gSensorValue[ADCNumber * 4 + 1];
	temp3 = gSensorValue[ADCNumber * 4 + 2];
	temp4 = gSensorValue[ADCNumber * 4 + 3];

	U2_TX_buffer[7]  = HextTable[gSensorStatus[ADCNumber * 4 + 0] >> 4];
	U2_TX_buffer[8]  = HextTable[gSensorStatus[ADCNumber * 4 + 0] & 0x0F];
	U2_TX_buffer[13] = HextTable[gSensorStatus[ADCNumber * 4 + 1] >> 4];
	U2_TX_buffer[14] = HextTable[gSensorStatus[ADCNumber * 4 + 1] & 0x0F];
	U2_TX_buffer[19] = HextTable[gSensorStatus[ADCNumber * 4 + 2] >> 4];
	U2_TX_buffer[20] = HextTable[gSensorStatus[ADCNumber * 4 + 2] & 0x0F];
	U2_TX_buffer[25] = HextTable[gSensorStatus[ADCNumber * 4 + 3] >> 4];
	U2_TX_buffer[26] = HextTable[gSensorStatus[ADCNumber * 4 + 3] & 0x0F];

	shorttohex(temp1, &U2_TX_buffer[3]);
	shorttohex(temp2, &U2_TX_buffer[9]);
	shorttohex(temp3, &U2_TX_buffer[15]);
	shorttohex(temp4, &U2_TX_buffer[21]);

	HAL_UART_Transmit_IT(&huart2, U2_TX_buffer, 30);
	U2_NewPacket = 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	}
}

static void MX_USART2_UART_Init_GD(unsigned int Baudrate)
{
	/* USER CODE BEGIN USART2_Init 0 */
	/* USER CODE END USART2_Init 0 */
	/* USER CODE BEGIN USART2_Init 1 */
	/* USER CODE END USART2_Init 1 */

	huart2.Instance          = USART2;
	huart2.Init.BaudRate     = Baudrate;
	huart2.Init.WordLength   = UART_WORDLENGTH_8B;
	huart2.Init.StopBits     = UART_STOPBITS_1;
	huart2.Init.Parity       = UART_PARITY_NONE;
	huart2.Init.Mode         = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
		Error_Handler();

	/* USER CODE BEGIN USART2_Init 2 */
	/* USER CODE END USART2_Init 2 */
}

/* ##################직접작성! 끝###################### *//* ##################직접작성! 시작###################### *//* ##################직접작성! 시작###################### */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the source file name and line number where an assert failed.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
