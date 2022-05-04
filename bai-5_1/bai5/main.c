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
 * File name: Bai5.1
 *
 * Description: This code is used for clean code
 * This is main function in application layer.
 *
 * Author: Trungnt
 *
 * Last Changed By:  $Author: trungnt
 * Revision:         $Revision: $
 * Last Changed:     $Date: $May 3, 2021
 *

 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include "queue.h"
#include "typedefs.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/

#define SIZE_QUEUE_DATA_RX 						256
#define CMD_ID_FIR          					0x00
#define CMD_ID_SECOND    						0x01
#define CMD_ID_THIRD    						0x04
#define CMD_ID_FOURTH    						0x84
#define CMD_ID_FITH    							0x85
#define CMD_ID_SIXTH    						0x86
#define CMD_TYPE_FIRST							0x00
#define CMD_TYPE_SECOND							0x01
#define CMD_TYPE_THIRD							0x02

#define RX_BUFFER_SIZE							16

#define FRAME_SOF								0xB1
#define FRAME_ACK								0x06
#define FRAME_NACK								0x15

#define CXOR_INIT_VAL							0xFF

#define CMD_ID									byRxBuffer[2]
#define CMD_TYPE								byRxBuffer[3]
#define CMD_DATA_EPOINT							byRxBuffer[4]
#define CMD_DATA_STATE							byRxBuffer[5]

typedef enum {
	RX_STATE_START_BYTE, 
	RX_STATE_DATA_BYTES,
	RX_STATE_CXOR_BYTE,
} ReceiveState;

typedef enum {
	UART_STATE_IDLE,
	UART_STATE_DATA_RECEIVED,
	UART_STATE_ACK_RECEIVED,
	UART_STATE_NACK_RECEIVED,		
	UART_STATE_ERROR,
	UART_STATE_RX_TIMEOUT,
} UartState;

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
u8_t g_byDataReceive;
static u8_t g_byRxBufState;
static u8_t g_byIndexRxBuf;
static u8_t g_byCheckXorRxBuf;
static buffqueue_t serialQueueRx;
static u8_t pbyBuffDataRx[SIZE_QUEUE_DATA_RX];
u8_t byRxBuffer[RX_BUFFER_SIZE] = { 0 };
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
u8_t serialPoll(void_t);
static void_t checkDataTramsmit(void_t);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/

int main(void_t) {
	
	u8_t i;
	u8_t byStateRx;
	u32_t dwFrame[9];

	/* Initializes receive register buffer	*/
	bufInit(pbyBuffDataRx, &serialQueueRx, sizeof(pbyBuffDataRx[0]),SIZE_QUEUE_DATA_RX);
	printf("Enter Frame: ");

	for (i = 0; i < 9; ++i) 
	{
		//fflush(stdout);
		scanf("%x", &dwFrame[i]);
		bufEnDat(&serialQueueRx, (u8_t*) &dwFrame[i]);
	}
	byStateRx = serialPoll();
	if (byStateRx != UART_STATE_IDLE) 
	{
		switch (byStateRx) 
		{
			case UART_STATE_ACK_RECEIVED:
				printf("UART_STATE_ACK_RECEIVED\r\n");
				break;
			case UART_STATE_NACK_RECEIVED:
				printf("UART_STATE_NACK_RECEIVED\r\n");
				break;
			case UART_STATE_DATA_RECEIVED:
				printf("UART_STATE_DATA_RECEIVED\r\n");
				checkDataTramsmit();
				break;
			case UART_STATE_ERROR:
			case UART_STATE_RX_TIMEOUT:
				printf("UART_STATE_RX_TIMEOUT\r\n");
				break;
			default:
				break;
		}
	}
	while(getchar() != '\n')
	{

	}
	getchar();
	return 0;
}
/**
 * @func	serialPoll
 * @brief	serial Pool
 * @param	None
 * @retval	None
 */
u8_t serialPoll(void_t) {
	u8_t byRxData;
	u8_t byUartState = (u8_t) UART_STATE_IDLE;
	while ((bufNumItems(&serialQueueRx) != 0) && (byUartState == UART_STATE_IDLE)) 
	{
		bufDeDat(&serialQueueRx, &byRxData);
	}
	switch (g_byRxBufState) 
	{
		case RX_STATE_START_BYTE:
			if (byRxData == FRAME_SOF)
			{
				g_byIndexRxBuf = 0;
				g_byCheckXorRxBuf = CXOR_INIT_VAL;
				g_byRxBufState = RX_STATE_DATA_BYTES;
			} 
			else if (byRxData == FRAME_ACK) 
			{
				byUartState = UART_STATE_ACK_RECEIVED;

			} 
			else if (byRxData == FRAME_NACK) 
			{
				byUartState = UART_STATE_NACK_RECEIVED;
			} 
			else 
			{
				byUartState = UART_STATE_ERROR;
			}
			break;

		case RX_STATE_DATA_BYTES:
			if (g_byIndexRxBuf < RX_BUFFER_SIZE) 
			{
				byRxBuffer[g_byIndexRxBuf] = byRxData;
				if (g_byIndexRxBuf > 0) 
				{
					/* Caculate check xor */
					g_byCheckXorRxBuf ^= byRxData;
				}
				if (++g_byIndexRxBuf == *byRxBuffer) 
				{
				g_byRxBufState = RX_STATE_CXOR_BYTE;
				}
			} 
			else 
			{
				g_byRxBufState = RX_STATE_START_BYTE;
				byUartState = UART_STATE_ERROR;
			}
			break;
		case RX_STATE_CXOR_BYTE:
			if (byRxData == g_byCheckXorRxBuf) 
			{
				byUartState = UART_STATE_DATA_RECEIVED;
			} 
			else 
			{
				byUartState = UART_STATE_ERROR;
			}
		default:
			g_byRxBufState = RX_STATE_START_BYTE;
			break;
	}
	return byUartState;
}

/**
 * @func	checkDataTramsmit
 * @brief	check data received from terminal
 * @param	None
 * @retval	None
 */
static void_t checkDataTramsmit(void_t) {
	if (CMD_ID == CMD_ID_FIR )
	{
		if(CMD_TYPE == CMD_TYPE_FIRST)
		{
			printf("Board STM32F4 Nucleo\r\n");
		}
	}
	else if(CMD_ID == CMD_ID_SECOND)
	{
		if(CMD_TYPE == CMD_TYPE_THIRD)
		{
			if(CMD_DATA_EPOINT == CXOR_INIT_VAL)
			{
				printf("Led turned on\r\n");
			}
			else
			{
				printf("Led turned off\r\n");
			}
		}
	}
	else if(CMD_ID == CMD_ID_THIRD)
	{
		if(CMD_TYPE == CMD_TYPE_THIRD)
		{
			if(CMD_DATA_STATE == CXOR_INIT_VAL)
			{
				printf("buzzer turned on\r\n");
			}
			else
			{
				printf("buzzer turned off\r\n");
			}
		}
	}
	else if(CMD_ID == CMD_ID_FOURTH)
	{
		if(CMD_TYPE == CMD_TYPE_SECOND)
		{
			printf("Temperature of sensor is 33 oC\r\n");
		}
	}
	else if(CMD_ID == CMD_ID_FITTH)
	{
		if(CMD_TYPE == CMD_TYPE_SECOND)
		{
			printf("Humidity of sensor is 82 \r\n");
		}
	}
	else if(CMD_ID == CMD_ID_SIXTH)
	{
		if(CMD_TYPE == CMD_TYPE_SECOND)
		{
			printf("Light intensity of sensor is 50 lux\r\n");
		}
	}
}
