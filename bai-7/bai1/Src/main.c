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
 * Author: TrungNT
 *
 * Last Changed By:  $Author: tuvq
 * Revision:         $Revision: $
 * Last Changed:     $Date: $May 6, 2022
 *

 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include <stm32f401re_gpio.h>
#include <stm32f401re_exti.h>
#include <stm32f401re_syscfg.h>
#include <stm32f401re_rcc.h>
#include <misc.h>
#include <typedefs.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
// Define Logic GPIO_PIN
#define GPIO_PIN_SET						1
#define GPIO_PIN_RESET						0
#define GPIO_PIN_LOW						0
#define GPIO_PIN_HIGH						1

// Define GPIO PIN
#define LED_GPIO_PORT						GPIOA
#define LED_GPIO_PIN						GPIO_Pin_5
#define LED_PIN9							5
#define LED_CONTROL_SET_CLOCK				RCC_AHB1Periph_GPIOA

#define BUTTON_GPIO_PORT					GPIOC
#define BUTTON_GPIO_PIN						GPIO_Pin_13
#define BUTTON_PIN3							13
#define BUTTON_CONTROL_SET_CLOCK			RCC_AHB1Periph_GPIOC

#define SYSFG_CLOCK							RCC_APB2Periph_SYSCFG
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
u8_t g_byStatus=1;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void_t delay();
static void_t ledInit(void_t);
static void_t interruptInit(void_t);
static void_t EXT15_IRQHandler(void_t);
static void_t ledControlSetStatus(GPIO_TypeDef *GPIOx, u16_t GPIO_PIN,
		u8_t byStatus);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/

int main(void_t)
{
	SystemCoreClockUpdate();
	ledInit();
	interruptInit();
	while(1)
	{
		if(g_byStatus % 2 != 0)
		{
			ledControlSetStatus(LED_GPIO_PORT, LED_GPIO_PIN, 0);
		}
		else
		{
			ledControlSetStatus(LED_GPIO_PORT, LED_GPIO_PIN, 1);
		}
	}
}

/**
 * @func 		ledInit
 * @brief		init led
 * @param		None
 * @retval		None
 */
static void_t ledInit(void_t)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(LED_CONTROL_SET_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
}

/**
 * @func 		interruptInit
 * @brief		init interrupt
 * @param		None
 * @retval		None
 */
static void_t interruptInit(void_t)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;


	/* Enable Clock Port C*/
	RCC_AHB1PeriphClockCmd(BUTTON_CONTROL_SET_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = BUTTON_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(BUTTON_GPIO_PORT, &GPIO_InitStructure);


	/*Enable Clock Syscfg, Connect EXTI Line 13 to PC13 pin*/

	RCC_APB2PeriphClockCmd(SYSFG_ClOCK, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);


	/*Configuration Interrupt*/

	EXTI_InitStructure.EXTI_Line = EXTI_Line13;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/*Configuration NVIC*/

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @func 		EXT15_IRQHandler
 * @brief		IRQ handler
 * @param		None
 * @retval		None
 */
static void_t EXT15_IRQHandler(void_t)
{
	if(EXTI_GetFlagStatus(EXTI_Line13) == SET)
	{
		g_byStatus++;
	}

	//xóa cờ ngắt sau khi thực hiện xong chương trình ngắt
	EXTI_ClearITPendingBit(EXTI_Line13);
}

/**
 * @func 		ledControlSetStatus
 * @brief		control led state
 * @param		None
 * @retval		None
 */
static void_t ledControlSetStatus(GPIO_TypeDef *GPIOx, u16_t GPIO_PIN,
		u8_t byStatus) {
	// SET bit in BSRR Registers

	if (byStatus == GPIO_PIN_SET) {
		GPIOx->BSRRL = GPIO_PIN;
	}
	if (byStatus == GPIO_PIN_RESET) {
		GPIOx->BSRRH = GPIO_PIN;
	}
}

/**
 * @func 		delay
 * @brief		time delay ~ 1ms
 * @param		None
 * @retval		None
 */
void_t delay()
{
	for(uint32_t i = 0;i<500000;i++);
}