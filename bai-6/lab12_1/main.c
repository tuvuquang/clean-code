/*******************************************************************************
 *				 _ _                                             _ _
				|   |                                           (_ _)
				|   |        _ _     _ _   _ _ _ _ _ _ _ _ _ _   _ _
				|   |       |   |   |   | |    _ _     _ _    | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |       |   |   |   | |   |   |   |   |   | |   |
				|   |_ _ _  |   |_ _|   | |   |   |   |   |   | |   |
				|_ _ _ _ _| |_ _ _ _ _ _| |_ _|   |_ _|   |_ _| |_ _|
								(C)2021 Lumi
 * Copyright (c) 2021
 * Lumi, JSC.
 * All Rights Reserved
 *
 * File name: bai-6
 *
 * Description: This code is used for clean code
 * This is main function in application layer.
 *
 * Author: TrungNT
 *
 * Last Changed By:  $Author: TuVQ
 * Revision:         $Revision: $
 * Last Changed:     $Date: $May 5, 2022
 *

 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <queue.h>
#include <typedefs.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define SIZE_QUEUE_DATA_RX 									256
#if (SIZE_QUEUE_DATA_RX &(SIZE_QUEUE_DATA_RX - 1) != 0)
#error "SIZE_QUEUE_DATA_RX must be a power of two"
#endif

#define RX_BUFFER_SIZE 				16

#define FRAME_SOF					0xB1

#define FRAME_ACK					0x06
#define FRAME_NACK					0x15

/* @brief check xor init */
#define CXOR_INIT_VAL				0xFF

/* @brief Field	type*/
#define CMD_TYPE_GET				0x00
#define CMD_TYPE_RES				0x01
#define CMD_TYPE_SET				0x02

/* @brief Field command id*/
#define CMD_ID_DEVICE				0x00
#define CMD_ID_LED					0x01
#define CMD_ID_BUZZER				0x04
#define CMD_ID_BUTTON				0x82
#define CMD_ID_TEMP_SENSOR			0x84
#define CMD_ID_HUMI_SENSOR			0x85
#define CMD_ID_LIGHT_SENSOR			0x86
#define CMD_ID_LCD					0x87

#define FRAME_SIZE_MAX				9

typedef enum {
	RX_STATE_START_BYTE,
	RX_STATE_DATA_BYTES,
	RX_STATE_CXOR_BYTE
} ReceiveState;

typedef enum {
	UART_STATE_IDLE,
	UART_STATE_DATA_RECEIVED,
	UART_STATE_ACK_RECEIVED,
	UART_STATE_NACK_RECEIVED,
	UART_STATE_ERROR,
	UART_STATE_RX_TIMEOUT,
} UartState;

//Khai báo các trường dữ liệu của các module bằng struct.

typedef struct{
	u8_t cmtID;
	u8_t type;
}FrmCommon_t;

typedef struct{
	u8_t cmtID;
	u8_t type;
	u8_t infor;
}FrmDeviceInfor_t;

typedef struct{
	u8_t cmtID;
	u8_t type;
	u8_t state;
}FrmBuzzerState_t;

typedef struct{
	u8_t cmtID;
	u8_t type;
	u8_t value;
}FrmLightSensor_t;

typedef struct{
	u8_t cmtID;
	u8_t type;
	u8_t value;
}FrmTemperatureSensor_t;

typedef struct{
	u8_t cmtID;
	u8_t type;
	u8_t value;
}FrmHumiditySensor_t;

typedef struct{
	u8_t cmtID;
	u8_t type;
	u8_t text[20];
}FrmLcdDisplay_t;

typedef struct{
	u8_t cmtID;
	u8_t type;
	u8_t numID;
	u8_t state;
}FrmLedIndicator_t;

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
static u8_t g_byRxBufState;
static u8_t g_byIndexRxBuf;
static u8_t g_byCheckXorRxBuf;
static buffqueue_t serialQueueRx;
static u8_t g_pbyBuffDataRx[SIZE_QUEUE_DATA_RX];
static u8_t g_byRxBuffer[RX_BUFFER_SIZE] = { 0 };

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void_t appInitCommon(void_t);
static void_t Led_Init(void_t);
static void_t Button_Init(void_t);
static void_t I2C_InitMaster(void_t);
static void_t I2C_InitSlave(void_t);
static void_t I2C_Start(void_t);
static void_t I2C_SendAddress(u8_t byAdsress);
static void_t I2C_TransmitData(u8_t byData);
static void_t I2C_Stop(void_t);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
u8_t serialPoll(void_t);
void_t uartCommandProcess(void_t *arg);
/******************************************************************************/

int main() {
	// Define Strings
	u8_t i;
	u8_t byState;
	unsigned i32_t frameArr[9];

	bufInit(pBuffDataRx, &serialQueueRx, sizeof(pBuffDataRx[0]),
			SIZE_QUEUE_DATA_RX);
	printf("Enter Frame: ");

	for (i = 0; i < 9; i++) {
		fflush(stdout);
		scanf("%x", &frameArr[i]);
		bufEnDat(&serialQueueRx, (u8_t*) &frameArr[i]);
	}

	byState = serialPoll();
	if (byState != UART_STATE_IDLE) {
		switch (byState) {
		case UART_STATE_ACK_RECEIVED:
			printf("UART_STATE_ACK_RECEIVED\r\n");
			break;

		case UART_STATE_NACK_RECEIVED:
			printf("UART_STATE_NACK_RECEIVED\r\n");
			break;

		case UART_STATE_DATA_RECEIVED:
			printf("UART_STATE_DATA_RECEIVED\n");
			uartCommandProcess(&byRxBuffer[2]);
			break;

		case UART_STATE_ERROR:
		case UART_STATE_RX_TIMEOUT:
			printf("UART_STATE_RX_TIMEOUT\r\n");
			break;

		default:
			break;
		}
	}
	getchar();
	getchar();
	return 0;
}
/**
 * @func 		serialPoll
 * @brief		Poll serial
 * @param		None
 * @retval		None
 */
u8_t serialPoll(void_t) {
	u8_t byRxData;
	u8_t byUartState = (u8_t) UART_STATE_IDLE;
	while ((bufNumItems(&serialQueueRx) != 0)
			&& (byUartState == UART_STATE_IDLE)) {
		bufDeDat(&serialQueueRx, &byRxData);
		switch (byRxBufState) {

		case RX_STATE_START_BYTE:
			if (byRxData == FRAME_SOF) {
				byIndexRxBuf = 0;
				byCheckXorRxBuf = CXOR_INIT_VAL;
				byRxBufState = RX_STATE_DATA_BYTES;
			} else if (byRxData == FRAME_ACK) {
				byUartState = UART_STATE_ACK_RECEIVED;
			} else if (byRxData == FRAME_NACK) {
				byUartState = UART_STATE_NACK_RECEIVED;
			} else {
				byUartState = UART_STATE_ERROR;
			}
			break;

		case RX_STATE_DATA_BYTES:
			if (byIndexRxBuf < RX_BUFFER_SIZE) {
				byRxBuffer[byIndexRxBuf] = byRxData;
				if (byIndexRxBuf > 0) {
					byCheckXorRxBuf ^= byRxData;
				}
				if (++byIndexRxBuf == *byRxBuffer) {
					byRxBufState = RX_STATE_CXOR_BYTE;
				}
			} else {
				byRxBufState = RX_STATE_START_BYTE;
				byUartState = UART_STATE_ERROR;
			}
			break;

		case RX_STATE_CXOR_BYTE: {
			if (byRxData == byCheckXorRxBuf) {
				byUartState = UART_STATE_DATA_RECEIVED;
			} else {
				byUartState = UART_STATE_ERROR;
			}
			default:
			byRxBufState = RX_STATE_START_BYTE;
			break;
		}

		}
	}
	return byUartState;
}
/**
 * @func 		uartCommandProcess
 * @brief		Process command uart
 * @param		None
 * @retval		None
 */
void_t uartCommandProcess(void_t *arg)
{
	FrmCommon_t *pCmd = (FrmCommon_t *) arg;

	switch(pCmd->cmtID)
	{
	case CMD_ID_LED:
		printf("Xu li tin hieu LED");
		break;

	case CMD_ID_BUZZER:
		printf("Xu li tin hieu BUZZER");
		break;

	case CMD_ID_LIGHT_SENSOR:
		printf("Xu li tin hieu LIGHT SENSOR");
		break;

	case CMD_ID_TEMP_SENSOR:
		printf("Xu li tin hieu TEMP SENSOR");
		break;

	case CMD_ID_HUMI_SENSOR:
		printf("Xu li tin hieu HUMI SENSOR");
		break;

	case CMD_ID_LCD:
		printf("Xu li tin hieu LCD");
		break;

	default:
		break;
	}
}
