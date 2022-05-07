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
 * File name: bai-8
 *
 * Description: This code is used for clean code
 * This is main function in application layer.
 *
 * Author: TrungNT
 *
 * Last Changed By:  $Author: tuvq
 * Revision:         $Revision: $
 * Last Changed:     $Date: $May 7, 2022
 *

 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdint.h>
#include <stm32f401re_rcc.h>
#include <stm32f401re_gpio.h>
#include <stm32f401re_spi.h>
#include <misc.h>
#include <typedefs.h>
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define GPIO_PIN_SET											1
#define GPIO_PIN_RESET											0
#define GPIO_PIN_LOW											0
#define GPIO_PIN_HIGH											1

#define LED														GPIO_Pin_0
#define PIN_LED											    	0
#define BUTTON													GPIO_Pin_13
#define PIN_BOTTON												13
#define Pin_NSS													12

#define SPI_MASTER_GPIO_CLOCK									RCC_AHB1Periph_GPIOB
#define SPI_MASTER_CLOCK										RCC_APB1Periph_SPI2
#define SPI_MASTER_GPIO											GPIOB
#define SPI_MASTER												SPI2
#define NSS_MASTER												GPIO_Pin_12
#define SCK_MASTER												GPIO_Pin_13
#define MISO_MASTER												GPIO_Pin_14
#define MOSI_MASTER												GPIO_Pin_15

#define SPI_SLAVE_GPIO_CLOCK									RCC_AHB1Periph_GPIOA
#define SPI_SLAVE_CLOCK											RCC_APB2Periph_SPI1
#define SPI_SLAVE_GPIO											GPIOA
#define SPI_SLAVE												SPI1
#define NSS_SLAVE												GPIO_Pin_4
#define SCK_SLAVE												GPIO_Pin_5
#define MISO_SLAVE												GPIO_Pin_6
#define MOSI_SLAVE												GPIO_Pin_7

#define CHECK_DATA_SLAVE									    0xB1
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
u8_t g_byReceiveData = 0;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void_t SPI2_MasterInit(void_t);
static void_t SPI1_SlaveInit(void_t);
static u8_t GPIO_ReadPin(GPIO_TypeDef * GPIOx, u32_t GPIO_PIN);
static void_t ledInit(void_t);
static void_t buttonInit(void_t);
static u8_t reviceDataSlave(SPI_TypeDef *SPIx);
static void_t sendData(SPI_TypeDef *SPIx, u8_t byData);
static void_t GPIO_PinControl(GPIO_TypeDef *GPIOx, u8_t GPIO_PIN, u8_t byStatus);
static void_t delay(u32_t ms);
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/

int main(void_t)
{
	SystemCoreClockUpdate();
	buttonInit();
	ledInit();
	SPI2_MasterInit();
	SPI1_SlaveInit();
	while(1)
	{
		//Send data when Button pressed-----------------------------------------
				if(GPIO_ReadPin(GPIOC, PIN_BOTTON) == 0)
				{
					sendData(SPI2, 0xB1);
				}
				//Turn on led when data = 0x20------------------------------------------
				if (g_byReceiveData == Check_DataSlave)
				{
				    for (int i = 0; i<5; i++)
				    {
				    	GPIO_PinControl(GPIOA, PIN_LED, GPIO_PIN_SET);
				    	delay(1000);
				    	GPIO_PinControl(GPIOA, PIN_LED, GPIO_PIN_RESET);
				    	delay(1000);
				    }
				    g_byReceiveData = 0;
				}
		    	GPIO_PinControl(GPIOA, PIN_LED, GPIO_PIN_RESET);
	}
}

/**
 * @func   Initializes SPI Master
 * @brief  SPI_InitMaster
 * @param  None
 * @retval None
 */
static void_t SPI2_MasterInit(void_t)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef	 SPI_InitStructure;

	RCC_AHB1PeriphClockCmd(SPI_MASTER_GPIO_CLOCK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

	GPIO_InitStructure.GPIO_Pin = SCK_MASTER| MISO_MASTER | MOSI_MASTER;
	GPIO_Init(SPI_MASTER_GPIO, &GPIO_InitStructure);

	// Connect SPI1 pins to SPI Alternate function
	GPIO_PinAFConfig(SPI_MASTER_GPIO, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(SPI_MASTER_GPIO, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(SPI_MASTER_GPIO, GPIO_PinSource15, GPIO_AF_SPI2);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = NSS_MASTER;
	GPIO_Init(SPI_MASTER_GPIO, &GPIO_InitStructure);

	//Enable peripheral clock----------------------------------------------
	RCC_APB1PeriphClockCmd(SPI_MASTER_CLOCK, ENABLE);

	//Set to full duplex mode, seperate MOSI and MISO----------------------
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;

	SPI_Init(SPI_Master, &SPI_InitStructure);
	SPI_Cmd(SPI_Master, ENABLE);
}

/**
 * @func   Initializes SPI Slave
 * @brief  SPI_InitSlave
 * @param  None
 * @retval None
 */
static void_t SPI1_SlaveInit(void_t)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef	 SPI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Connect Clock to GPIOB--------------------------------------------------
	RCC_AHB1PeriphClockCmd(SPI_SLAVE_GPIO_CLOCK, ENABLE);

	// Initialization GPIO use for SPI-----------------------------------------
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = SCK_SLAVE | MISO_SLAVE | MOSI_SLAVE;
	GPIO_Init(SPI_SLAVE_GPIO, &GPIO_InitStructure);

	// Connect SPI1 pins to SPI Alternate function
	GPIO_PinAFConfig(SPI_SLAVE_GPIO, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPI_SLAVE_GPIO, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(SPI_SLAVE_GPIO, GPIO_PinSource7, GPIO_AF_SPI1);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = NSS_SLAVE;
	GPIO_Init(SPI_SLAVE_GPIO, &GPIO_InitStructure);

	//Enable peripheral clock----------------------------------------------
	RCC_APB2PeriphClockCmd(SPI_SLAVE_CLOCK, ENABLE);

	//Set to full duplex mode, seperate MOSI and MISO----------------------
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;

	SPI_Init(SPI_Slave, &SPI_InitStructure);

	//Enable SPI1----------------------------------------------------------
	SPI_Cmd(SPI_Slave, ENABLE);

	SPI_I2S_ITConfig(SPI_Slave, SPI_I2S_IT_RXNE, ENABLE);
	//NVIC configuration---------------------------------------------------
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init( &NVIC_InitStructure);

	SPI_I2S_ITConfig(SPI_Slave, SPI_I2S_IT_RXNE, ENABLE);
}
/**
 * @func   Control Led Turn_On or Turn_Off
 * @brief  Led_Control
 * @param  None
 * @retval None
 */
static void_t GPIO_PinControl(GPIO_TypeDef *GPIOx, u8_t GPIO_PIN,
		u8_t byStatus) {
	// SET bit in BSRR Registers
	if (byStatus == GPIO_PIN_SET) {
		GPIOx->BSRRL |= 1 << GPIO_PIN;
	}
	if (byStatus == GPIO_PIN_RESET) {
		GPIOx->BSRRH |= 1 << GPIO_PIN;
	}
}
static
void_t sendData(SPI_TypeDef * SPIx, u8_t byData)
{
	//Allow Send Data to Slave--------------------------------------------------
	GPIO_PinControl (SPI_Master_GPIO, Pin_NSS, GPIO_PIN_RESET);
	SPI_I2S_SendData(SPIx,byData);

	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET){;}

	//Allow Send Data to Slave--------------------------------------------------
	GPIO_PinControl (SPI_Master_GPIO, Pin_NSS, GPIO_PIN_SET);
}
/**
 * @func   Recive Data From Slave
 * @brief  Revice_Data
 * @param  None
 * @retval None
 */
static
u8_t reviceDataSlave(SPI_TypeDef * SPIx)
{
	u8_t byDataReceive = 0;

	byDataReceive = SPI_I2S_ReceiveData(SPIx);

	return byDataReceive;
}
/**
 * @func   Read Logic On GPIO
 * @brief  GPIO_ReadPin
 * @param  None
 * @retval None
 */
static
u8_t GPIO_ReadPin(GPIO_TypeDef * GPIOx, u32_t GPIO_PIN)
{
	u32_t dwReadPin;

	//Read bit in IDR Registers-------------------------------------------------
	dwReadPin = (GPIOx->IDR) >> GPIO_PIN;
	dwReadPin = dwReadPin & 0x01;

	return dwReadPin;
}
/**
 * @func   The Function Executes The Interrupt
 * @brief  SPI1_IRQHandler
 * @param  None
 * @retval None
 */
void_t SPI1_IRQHandler(void_t)
{
	if(SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
	{
		g_byReceiveData = reviceDataSlave(SPI1);
	}
	SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_RXNE);
}
/**
 * @func   Initializes GPIO Use Led
 * @brief  ledInit
 * @param  None
 * @retval None
 */
static void_t ledInit(void_t)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = LED;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/**
 * @func   Initializes GPIO Use Button
 * @brief  buttonInit
 * @param  None
 * @retval None
 */
static void_t buttonInit(void_t)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_InitStructure.GPIO_Pin = BUTTON;

	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/**
 * @func   delay Time
 * @brief  delay
 * @param  None
 * @retval None
 */
static
void_t delay(u32_t ms)
{
	u32_t i,j;
	for (i = 0 ; i < ms ; i ++)
	{
		for (j = 0; j<5000; j++){;}
	}
}

