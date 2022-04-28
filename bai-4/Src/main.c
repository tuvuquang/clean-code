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
 * Last Changed:     $Date: $Apr 4, 2021
 *
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include <stm32f401re_rcc.h>
#include <stm32f401re_tim.h>
#include <stm32f401re_gpio.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define LED_GREEN_PORT							GPIOA
#define LED_GREEN_PIN							GPIO_Pin_0
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void_t appInitCommon(void_t);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void_t ledInit(void_t);
void_t timerInit(void_t);
void_t delay(u32_t dwMilliSecond);
/******************************************************************************/

int main(void_t)
{
	appInitCommon();

	while(1)
	{
		GPIO_SetBits(LED_GREEN_PORT, LED_GREEN_PIN);
		delay(1000);

		GPIO_ResetBits(LED_GREEN_PORT, LED_GREEN_PIN);

		delay(1000);
	}
}
/*
 * @func   appInitCommon
 * @brief  Init app layer
 * @param  None
 * @retval None
 */
static void_t appInitCommon(void_t)
{
	SystemCoreClockUpdate();
	ledInit();
	timerInit();
}
/*
 * @func   ledInit
 * @brief  Init led
 * @param  None
 * @retval None
 */
void_t ledInit(void_t)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_InitStruct.GPIO_Pin = LED_GREEN_PIN;
	GPIO_Init(LED_GREEN_PORT, &GPIO_InitStruct);
}
/*
 * @func   timerInit
 * @brief  init timer 
 * @param  None
 * @retval None
 */
void_t timerInit(void_t)
{
	TIM_TimeBaseInitTypeDef Timer_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	Timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;

	Timer_InitStructure.TIM_Period = 999;

	Timer_InitStructure.TIM_Prescaler = 83;

	Timer_InitStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &Timer_InitStructure);
	TIM_Cmd(TIM1, ENABLE);
}
/*
 * @func   delay
 * @brief  delay 1 ms
 * @param  None
 * @retval None
 */
void_t delay(u32_t dwMilliSecond)
{
	while(dwMilliSecond !=  0 )
	{
		TIM_SetCounter(TIM1, 0);
		while(TIM_GetCounter(TIM1) != 999){};
			dwMilliSecond--;
	}
}
