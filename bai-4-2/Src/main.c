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
	Khởi tạo các giá trị ADC chung.

	● Chọn chế độ cho ADC là ADC_Mode_Independent.
	● Chọn hệ số chia tần số là 1.
	● Không sử dụng DMA.
	● Chọn thời gian giữa 2 lần chuyển đổi là 5Cycles.
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
	Khởi tạo các giá trị cho ADC1.
	● Cấp xung Clock cho ADC1.
	● Chọn số bit chuyển đổi là 12 bit.
	● Không cho phép sử dụng chuyển đổi không liên tục.
	● Cho phép sử dụng chuyển đổi liên tục.
	● Cho phép ghi dữ liệu từ bên phải.
	● Chọn số kênh sử dụng là 1
	● Sử dụng hàm ADC_Init với tham số là bộ ADC đang sử dụng và biến
	dữ liệu thuộc kiểu cấu trúc ADC để cài đặt các giá trị đã khởi tạo ở
	trên.
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
	Cấu hình sử dụng DMA.
	 */
	//● RCC_AHB1PeriphClockCmd: Cấp xung clock cho DMA.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	//● DMA_DeInit: Khởi tạo mặc định cho DMA bằng hàm DMA_DeInit.
	DMA_DeInit(DMA2_Stream0);

	//● DMA_Channel: Cấu hình kênh DMA.
	DMA_InitStruct.DMA_Channel = DMA_Channel_0;

	//● DMA_PeripheralBaseAddr: Cấu hình địa chỉ của thanh ghi chứa Data trong bộ ADC.
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32_t)ADC_DR_ADDRESS;

	//● DMA_Memory0BaseAddr: Cấu hình địa chỉ của biến chứa giá trị ADC đọc về.
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32_t) &uhADC_ConvertedValue;

	//● DMA_DIR: Cấu hình chiều dữ liệu được truyền là từ ngoại vi.
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToMemory;

	//● DMA_BufferSize: Số lượng data truyền là 1.
	DMA_InitStruct.DMA_BufferSize = 1;

	//● DMA_PeripheralInc: Không cho phép tăng địa chỉ của ngoại vi vì data chỉ có 1.
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

	//● DMA_MemoryInc: Không cho phép tăng địa chỉ của bộ nhớ vì data chỉ có 1.
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Disable;

	//● DMA_PeripheralDataSize:Chọn kích thước mảng dữ liệu ADCValue gồm : Byte, Haftword và Word
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;

	//● DMA_MemoryDataSize: Chọn số bit của data bộ nhớ
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;

	//● DMA_Mode: Chọn chế độ DMA chuyển đổi liên tục.
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;

	//● DMA_Priority: Xác định độ ưu tiên của kênh DMA.
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;

	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;


	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;


	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;

	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	//● DMA_Init: Cấu hình các giá trị đã được khởi tạo ở trên.
	DMA_Init(DMA2_Stream0, &DMA_InitStruct);

	//DMA_Cmd: Cho phép DMA hoạt động.
	DMA_Cmd(DMA2_Stream0, ENABLE);

	//Enable DMA request after last transfer (singer-ADC mode)
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	// ENABLE ADC1 DMA
	//● ADC_DMACmd: Cho phép sử dụng ADC với chức năng DMA.
	ADC_DMACmd(ADC1, ENABLE);

	//Cấu hình kênh chuyển đổi, cho phép ADC1 hoạt động và chuyển đổi.
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


