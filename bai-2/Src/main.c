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
 * File name: main.c
 *
 * Description: This code is used for clean code
 * This is main function in application layer.
 *
 * Author: Tu Vu Quang
 *
 * Last Changed By:  $Author: tuvq $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $Apr, 27 2021
 *
 
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include <eventman.h>
#include <stm32f401re_usart.h>
#include <stm32f401re_rcc.h>
#include <misc.h>
#include <stm32f401re_gpio.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define GPIO_PIN_LOW											0
#define GPIO_PIN_HIGH											1

#define LED														GPIO_Pin_0
#define BUTTON													GPIO_Pin_13

#define USART6_TX												GPIO_Pin_6
#define USART6_GPIO												GPIOC
#define USART6_GPIO_CLOCK										RCC_AHB1Periph_GPIOC
#define USART6_CLOCK											RCC_APB2Periph_USART6

#define USART1_TX												GPIO_Pin_10
#define USART1_GPIO												GPIOA
#define USART1_GPIO_CLOCK										RCC_AHB1Periph_GPIOA
#define USART1_CLOCK											RCC_APB2Periph_USART1

#define USARTX_BAUD												9600

#define CHECK_DATA												0x10
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
u16_t g_wData1 = 0;
u16_t g_wDataReceive = 0;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void_t buttonInit(void_t);
static void_t ledInit(void_t);
static void_t USART6_TX_Init(void_t);
static void_t USART1_RX_Init(void_t);
static void_t controlLedReceive(void_t);
static void_t delay();
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/

int main(void_t)
{
	ledInit();
	buttonInit();
	USART6_TX_Init();
	USART1_RX_Init();
	while(1)
	{
		controlLedReceive();
	}

}
static void_t delay()
{
	for(u32_t i = 0 ; i < 50000;i++)
	{
		for(u32_t j = 0; j<500000;j++);
	}
}
/*
 * @func	buttonInit
 * @brief	Initialization Button
 * @param	None
 * @retval	None
 */
static void_t buttonInit(void_t)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStruct.GPIO_Pin = BUTTON;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

}
/*
 * @func	ledInit
 * @brief	Initialization Led
 * @param	None
 * @retval	None
 */
static void_t ledInit(void_t)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStruct.GPIO_Pin = LED;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/*
 * @func	USART6_TX_Init
 * @brief	Transfer
 * @param	None
 * @retval	None
 */
static void_t USART6_TX_Init(void_t)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(USART6_GPIO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin = USART6_TX;
	GPIO_Init(USART6_GPIO, &GPIO_InitStructure);
	GPIO_PinAFConfig(USART6_GPIO, GPIO_PinSource6, GPIO_AF_USART6);

// Configure USART
	RCC_APB2PeriphClockCmd(USART6_CLOCK, ENABLE);

	USART_InitStructure.USART_BaudRate = USARTX_BAUD;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;

	USART_Init(USART6, &USART_InitStructure);
	USART_Cmd(USART6, ENABLE);

}

/*
 * @func	USART1_RX_Init
 * @brief	Receved
 * @param	None
 * @retval	None
 */
static
void_t USART1_RX_Init(void_t)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;


	RCC_AHB1PeriphClockCmd(USART1_GPIO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin = USART1_TX;
	GPIO_Init(USART1_GPIO, &GPIO_InitStructure);

	GPIO_PinAFConfig(USART1_GPIO, GPIO_PinSource1, GPIO_AF_USART1);

// Configure USART
	RCC_APB2PeriphClockCmd(USART1_CLOCK, ENABLE);

	USART_InitStructure.USART_BaudRate = USARTX_BAUD;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;

	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);

// NVIC configure
	// Enable USARTx Receive interrupts
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	// NVIC configuration
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
 * @func	USART1_IRQHandler
 * @brief	Interrupt
 * @param	None
 * @retval	None
 */

void_t USART1_IRQHandler(void_t)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		g_wData1 = USART_ReceiveData(USART1);
	}
}
/*
 * @func	controlLedReceive
 * @brief	Send data from USART6 to USART1
 * @param	None
 * @retval	None
 */
static void_t controlLedReceive(void_t)
{
	// Send data from USART6 to USART1
	if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)== 0)
	{
		USART_SendData(USART6, 0x10);
	}

	if(g_wDataReceive == CHECK_DATA)
	{
		for(i32_t i = 0; i<5; i++)
		{
			GPIO_SetBits(GPIOA, LED);
			delay();
			GPIO_ResetBits(GPIOA, LED);
			delay();
		}
	}
}
