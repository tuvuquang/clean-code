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
 * Last Changed:     $Date: $Apr 28, 2021
 *
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include <stm32f401re_adc.h>
#include <stm32f401re_rcc.h>
#include <stm32f401re_gpio.h>
#include <stm32f401re_usart.h>
#include <timer.h>
#include <stm32f401re_dma.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define USART2_Tx								GPIO_Pin_2
#define USART_Baud								9600

#define ADC_PORT								GPIOC
#define ADC_PIN									GPIO_Pin_5

#define ADC_DR_ADDRESS							((u32_t)0x4001204C)
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
u16_t g_wADC_ConvertedValue;
u16_t g_wLight;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void_t ADC_DMA(void_t);
static void_t USART2_Init(void_t);
u16_t lightSensorMeasureUseDMA(void_t);
static void_t MultilSensorScan(void_t);
static void_t applyCommon(void_t);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/

int main(void_t)
{
	applyCommon();

	while(1)
	{
		MultilSensorScan();
		processTimerScheduler();
	}
}

static void_t applyCommon(void_t)
{
	systemCoreClockUpdate();
	timerInit();
	ADC_DMA();
	USART2_Init();
}
static void_t ADC_DMA(void_t)
{
	ADC_InitTypeDef				ADC_InitStruct;
	ADC_CommonInitTypeDef		ADC_CommonInitStruct;
	DMA_InitTypeDef				DMA_InitStruct;
	GPIO_InitTypeDef			GPIO_InitStruct;

	// GPIO configure
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Pin = ADC_PIN;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	/**************************************************************************************
	// ADC_Common Configure
	Kh???i t???o c??c gi?? tr??? ADC chung.

	??? Ch???n ch??? ????? cho ADC l?? ADC_Mode_Independent.
	??? Ch???n h??? s??? chia t???n s??? l?? 1.
	??? Kh??ng s??? d???ng DMA.
	??? Ch???n th???i gian gi???a 2 l???n chuy???n ?????i l?? 5Cycles.
	***************************************************************************************/
	// Enable ADC peripheral clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_DeInit();

	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;

	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;

	ADC_CommonInit(&ADC_CommonInitStruct);
	/***************************************************************************************
	// ADC_Init Advance
	Kh???i t???o c??c gi?? tr??? cho ADC1.
	??? C???p xung Clock cho ADC1.
	??? Ch???n s??? bit chuy???n ?????i l?? 12 bit.
	??? Kh??ng cho ph??p s??? d???ng chuy???n ?????i kh??ng li??n t???c.
	??? Cho ph??p s??? d???ng chuy???n ?????i li??n t???c.
	??? Cho ph??p ghi d??? li???u t??? b??n ph???i.
	??? Ch???n s??? k??nh s??? d???ng l?? 1
	??? S??? d???ng h??m ADC_Init v???i tham s??? l?? b??? ADC ??ang s??? d???ng v?? bi???n
	d??? li???u thu???c ki???u c???u tr??c ADC ????? c??i ?????t c??c gi?? tr??? ???? kh???i t???o ???
	tr??n.
	 *****************************************************************************************/
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigInjecConv_T4_CC1;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStruct);
	/*****************************************************************************************
	// DMA Configure
	C???u h??nh s??? d???ng DMA.
	 */
	//??? RCC_AHB1PeriphClockCmd: C???p xung clock cho DMA.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	//??? DMA_DeInit: Kh???i t???o m???c ?????nh cho DMA b???ng h??m DMA_DeInit.
	DMA_DeInit(DMA2_Stream0);

	//??? DMA_Channel: C???u h??nh k??nh DMA.
	DMA_InitStruct.DMA_Channel = DMA_Channel_0;

	//??? DMA_PeripheralBaseAddr: C???u h??nh ?????a ch??? c???a thanh ghi ch???a Data trong b??? ADC.
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32_t)ADC_DR_ADDRESS;

	//??? DMA_Memory0BaseAddr: C???u h??nh ?????a ch??? c???a bi???n ch???a gi?? tr??? ADC ?????c v???.
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32_t) &uhADC_ConvertedValue;

	//??? DMA_DIR: C???u h??nh chi???u d??? li???u ???????c truy???n l?? t??? ngo???i vi.
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToMemory;

	//??? DMA_BufferSize: S??? l?????ng data truy???n l?? 1.
	DMA_InitStruct.DMA_BufferSize = 1;

	//??? DMA_PeripheralInc: Kh??ng cho ph??p t??ng ?????a ch??? c???a ngo???i vi v?? data ch??? c?? 1.
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

	//??? DMA_MemoryInc: Kh??ng cho ph??p t??ng ?????a ch??? c???a b??? nh??? v?? data ch??? c?? 1.
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable;

	//??? DMA_PeripheralDataSize:Ch???n k??ch th?????c m???ng d??? li???u ADCValue g???m : Byte, Haftword v?? Word
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;

	//??? DMA_MemoryDataSize: Ch???n s??? bit c???a data b??? nh???
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;

	//??? DMA_Mode: Ch???n ch??? ????? DMA chuy???n ?????i li??n t???c.
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;

	//??? DMA_Priority: X??c ?????nh ????? ??u ti??n c???a k??nh DMA.
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;

	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;


	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;


	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;

	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	//??? DMA_Init: C???u h??nh c??c gi?? tr??? ???? ???????c kh???i t???o ??? tr??n.
	DMA_Init(DMA2_Stream0, &DMA_InitStruct);

	//DMA_Cmd: Cho ph??p DMA ho???t ?????ng.
	DMA_Cmd(DMA2_Stream0, ENABLE);

	//Enable DMA request after last transfer (singer-ADC mode)
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	// ENABLE ADC1 DMA
	//??? ADC_DMACmd: Cho ph??p s??? d???ng ADC v???i ch???c n??ng DMA.
	ADC_DMACmd(ADC1, ENABLE);

	//C???u h??nh k??nh chuy???n ?????i, cho ph??p ADC1 ho???t ?????ng v?? chuy???n ?????i.
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_15Cycles);

	//Enable ADC1
	ADC_Cmd(ADC1, ENABLE);

	// Start ADC1 Software Conversion
	ADC_SoftwareStartConv(ADC1);
}

u16_t lightSensorMeasureUseDMA(void_t)
{
	return wADC_ConvertedValue;
}

/**
 * @func   USART2_Init
 * @brief  Initialize USART2
 * @param  None
 * @retval None
 */
static void_t USART2_Init(void_t)
{
	GPIO_InitTypeDef 					GPIO_InitStruct;
	USART_InitTypeDef					USART_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

	//Enable USART

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;

	USART_Init(USART2, &USART_InitStruct);

	USART_Cmd(USART2, ENABLE);
}


/**
 * @func   MultilSensorScan
 * @brief  Scan sensor
 * @param  None
 * @retval None
 */
static void_t MultilSensorScan(void_t)
{
	u32_t dwTimeCurrent;
	static u32_t dwTimeTotal, dwTimeInit;

	dwTimeCurrent = GetMilSecTick();

	if(dwTimeCurrent >= dwTimeInit)
	{
		dwTimeTotal += dwTimeCurrent - dwTimeInit;
	}
	else
	{
		dwTimeTotal += 0xFFFFFFFFU - dwTimeCurrent + dwTimeInit;
	}

	if(dwTimeTotal >= 1000)
	{
		dwTimeTotal = 0;
		Light = lightSensorMeasureUseDMA();
		USART_SendData(USART2, Light);
	}
	dwTimeInit = dwTimeCurrent;
}


