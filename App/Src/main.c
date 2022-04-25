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
 * File name: FINAL_EMBEDDED_SYSTEM
 *
 * Description: This code is used for tranning Lumi IOT member. It is the code form statandard.
 * This is main function in application layer.
 *
 * Author: Tuvuquang
 *
 * Last Changed By:  $Author: tuvq $
 * Revision:         $Revision: $
 * Last Changed:     $Date: $Aug 23, 2022
 *
 * Code sample:
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "lumi-types.h"
#include "typedefs.h"
#include "main.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define DEBUG                        1
#define BUFF_SIZE                    3

// LCD ADDRESS
#define SLAVE_ADDRESS_LCD            0x4E
#define LCD_SIZE                     100

//LCD COMMANDS
#define DISPLAY_OFF_CURSOR_OFF       0x08
#define DISPLAY_ON_CURSOR_OFF        0x0C
#define SEND_LOWER_NIBBLE            0xF0
#define LCD_ALL_SEG_ON               0x09
#define LCD_CLEAR                    0x0D
#define CURSOR_BEGIN_LINE_2          0xC0
#define CURSOR_BEGIN_LINE_1          0x80
#define LCD_INCREASE_CURSOR          0x06
#define LCD_CLEAR_ALL                0x01
#define INIT_WITH_8_BIT              0X30
#define ENABLE_4_BIT_MODE            0x28
#define SETTING_4_BIT_MODE           0x20

// DHT11 PORT & PIN
#define DHT11_PORT                   GPIOB
#define DHT11_PIN                    GPIO_PIN_10

//MPU6050 
#define RESET_VALUE                  0x00
#define XA_OFFSET_L_TC               0x07
#define SIGNAL_PATH_RES              104
#define DIVIDE_VALUE_ACCEL           16384
#define DIVIDE_VALUE_GYRO            131
#define MPU_SIZE                     1000

// TEM & ACCEL
typedef struct
{
	  i8_t type; 
	  i32_t dataAccelX;
	  i32_t dataAccelY;
	  i32_t dataAccelZ;
	  i32_t temp;
	  i8_t count;
} DataTemAndAccel_t;
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//PV_UART
i8_t g_ibBuff[8];
i8_t g_ibDataRx;
i8_t g_ibReceive;
i8_t g_ibReceiveData[BUFF_SIZE]
i8_t g_ibRxIndex = 0;
i8_t g_ibCountPrint = 0;
i8_t g_ibRxData;
//end PV_UART

u16_t g_wCount = 0;
i16_t g_iwAccelAxRaw, g_iwAccelAyRaw, g_iwAccelAzRaw;
i32_t g_idwAccelAx, g_idwAccelAy, g_idwAccelAz;
i16_t g_iwGyroAxRaw, g_iwGyroAyRaw, g_iwGyroAzRaw;
i32_t g_idwGyroAx, g_idwGyroAy, g_idwGyroAz;

i8_t g_ibHumidityIntegral, g_ibHumidityDecimal;
i8_t g_ibCelsiusIntegral, g_ibCelsiusDecimal, g_ibSum;
u32_t g_dwPreMillis, g_dwCurrentMillis;
i32_t g_idwCelsius = 0;
i32_t g_idwFahrenheit = 0;
i32_t g_idwHumidity = 0;


osMessageQId myQueue01Handle;
osThreadId readTempHandle;
osThreadId readAccelHandle;
osThreadId sendDataHandle;
osThreadId sendCOMHandle;   
osMailQId dataMailQueueHandle;

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
static void_t MX_GPIO_Init(void_t);
static void_t MX_USART2_UART_Init(void_t);
static void_t MX_I2C1_Init(void_t);
static void_t MX_TIM1_Init(void_t);
static void_t systemClockConfig(void_t);
static void_t startReadTempTask(void_t const * argument); 
static void_t startReadAccelTask(void_t const * argument); 
static void_t startSendDataTask(void_t const * argument);
static void_t startSendCOMTask(void_t const * argument);

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/

int main(void_t){
    HAL_Init();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
	  HAL_TIM_Base_Start(&htim1);
	  MPU6050_Init();
	  HAL_UART_Receive_IT(&huart2, &ibRxData, 1);

    systemClockConfig();
  	lcdInit();
	  lcdSendString("Initializing...");
	
  	osMailQueueDef(dataMailQueue, 16, array_ibDataString);
	  dataMailQueueHandle = osMailCreate(osMailQ(DataMailQueue), NULL); 
 
    /*===============================Begin: Khoi tao cac task==============================*/
	  osThreadDef(readAccelTask, startReadAccelTask, osPriorityNormal, 0, 96);
    readAccelHandle = osThreadCreate(osThread(readAccelTask), NULL);
	
	  osThreadDef(sendDataTask, startSendDataTask, osPriorityAboveNormal, 0, 96);
    sendDataHandle = osThreadCreate(osThread(sendDataTask), NULL);
	
	  osThreadDef(readTempTask, startReadTempTask, osPriorityNormal, 0, 96);
    readTempHandle = osThreadCreate(osThread(readTempTask), NULL);
	
	  osThreadDef(sendCOMTask, startSendCOMTask, osPriorityHigh, 0, 96);
    sendCOMHandle = osThreadCreate(osThread(sendCOMTask), NULL);
/*===============================End: Khoi tao cac task====================================*/

/**
 * @func   lcdSendCmd
 * @brief  LCD send command
 * @param  None
 * @retval None
 */
void_t lcdSendCmd (i8_t cmd)
{
  i8_t ibDataUpperNibble,ibDataLowerNibble;
	i8_t array_ibDataString[4];
	ibDataUpperNibble = (cmd&SEND_LOWER_NIBBLE);
	ibDataLowerNibble = ((cmd<<4)&SEND_LOWER_NIBBLE);
	array_ibDataString[0] = ibDataUpperNibble | DISPLAY_ON_CURSOR_OFF; 
	array_ibDataString[1] = ibDataUpperNibble | DISPLAY_OFF_CURSOR_OFF ;  
	array_ibDataString[2] = ibDataLowerNibble | DISPLAY_ON_CURSOR_OFF; 
	array_ibDataString[3] = ibDataLowerNibble | DISPLAY_OFF_CURSOR_OFF ;  
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(i8_t *) array_ibDataString, 4, LCD_SIZE);
}
/**
 * @func   lcdSendData
 * @brief  send dat to LCD
 * @param  None
 * @retval None
 */
void_t lcdSendData (i8_t data)
{
	i8_t ibDataUpperNibble, ibDataLowerNibble;
	i8_t array_ibDataString[4];
  ibDataUpperNibble = (data & SEND_LOWER_NIBBLE);
	ibDataLowerNibble = ((data <<4) & SEND_LOWER_NIBBLE);
	array_ibDataString[0] = ibDataUpperNibble | LCD_CLEAR; 
	array_ibDataString[1] = ibDataUpperNibble | LCD_ALL_SEG_ON; 
	array_ibDataString[2] = ibDataLowerNibble | LCD_CLEAR;  
	array_ibDataString[3] = ibDataLowerNibble | LCD_ALL_SEG_ON;  
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(i8_t *) array_ibDataString, 4, LCD_SIZE);
}
/**
 * @func   lcdClear
 * @brief  clear LCD
 * @param  None
 * @retval None
 */
void_t lcdClear (void_t)
{
	lcdSendCmd (CURSOR_BEGIN_LINE_1);
	for (i32_t i=0; i<16; i++)
	{
		lcdSendData (' ');
	}
	lcdSendCmd(CURSOR_BEGIN_LINE_1);    
}
/**
 * @func   lcdPutCur
 * @brief  
 * @param  None
 * @retval None
 */
void_t lcdPutCur(i32_t idwRow, i32_t idwCol)
{
    switch (idwRow){
        case 0:
            idwCol |= CURSOR_BEGIN_LINE_1;
            break;
        case 1:
            idwCol |= CURSOR_BEGIN_LINE_2;
            break;
    }
    lcdSendCmd (idwCol);
}
/**
 * @func   lcdInit
 * @brief  Init LCD
 * @param  None
 * @retval None
 */

void_t lcdInit (void_t){
	  // 4 bit initialisation
	  HAL_Delay(50);  
	  lcdSendCmd (INIT_WITH_8_BIT);
	  HAL_Delay(5);  
	  lcdSendCmd (INIT_WITH_8_BIT);
  	HAL_Delay(1); 
  	lcdSendCmd (INIT_WITH_8_BIT);
  	HAL_Delay(10);
  	lcdSendCmd (SETTING_4_BIT_MODE); 
  	HAL_Delay(10);

    // dislay initialisation
  	lcdSendCmd (ENABLE_4_BIT_MODE); 
  	HAL_Delay(1);
  	lcdSendCmd (DISPLAY_OFF_CURSOR_OFF ); 
  	HAL_Delay(1);
  	lcdSendCmd (LCD_CLEAR_ALL); 
  	HAL_Delay(1);
  	HAL_Delay(1);
  	lcdSendCmd (LCD_INCREASE_CURSOR );
  	HAL_Delay(1);
  	lcdSendCmd (DISPLAY_ON_CURSOR_OFF); 
}
/**
 * @func   lcdSendString
 * @brief  send string to LCD
 * @param  None
 * @retval None
 */
void_t lcdSendString (i8_t *str){
	while (*str) lcdSendData (*str++);
}

/*========================END  I2C_LCD ========================================*/

/*=============================READ MPU-6050 SENSOR============================*/
/**
 * @func   MPU_Init
 * @brief  Init MPU
 * @param  None
 * @retval None
 */
void_t MPU6050_Init(void_t){
    i8_t ibCheck, ibData;
	  HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,WHO_AM_I_REG,1,&ibCheck,1,MPU_SIZE);
	  if(ibCheck == SIGNAL_PATH_RES){
  	ibData =RESET_VALUE;	
  	HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,PWR_MGMT_1_REG,1,&ibData,1,MPU_SIZE);
	  ibData =XA_OFFSET_L_TC;
	  HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,SMPLRT_DIV_REG,1,&ibData,1,MPU_SIZE);
  	ibData =RESET_VALUE;
    HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,ACCE_CONFIG_REG,1,&ibData,1,MPU_SIZE);
    HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,GYRO_CONFIG_REG,1,&ibData,1,MPU_SIZE);		
  	}
}
/**
 * @func   MPU_ReadAccel
 * @brief  read Accel
 * @param  None
 * @retval None
 */
void_t MPU6050_readAccel(void_t){
    i8_t array_ibRecData[6];
    HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR, ACCE_XOUT_H_REG, 1, array_ibRecData,6,MPU_SIZE);
    iwAccelAxRaw = array_ibRecData[0] <<8 | array_ibRecData[1];
    g_idwAccelAx = iwAccelAxRaw / DIVIDE_VALUE_ACCEL;
    iwAccelAyRaw = array_ibRecData[2] <<8 | array_ibRecData[3];
    g_idwAccelAy = iwAccelAyRaw / DIVIDE_VALUE_ACCEL;
    iwAccelAzRaw = array_ibRecData[4] <<8 | array_ibRecData[5];
    g_idwAccelAz = iwAccelAzRaw / DIVIDE_VALUE_ACCEL;
}
/**
 * @func   MPU6050_readGyro
 * @brief  read Gyro
 * @param  None
 * @retval None
 */
void_t MPU6050_readGyro(void_t){
    i8_t array_ibRecData[6];
    HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,GYRO_XOUT_H_REG,1,array_ibRecData,6,MPU_SIZE);
    iwGyroAxRaw = array_ibRecData[0] <<8|array_ibRecData[1];
    g_idwGyryAx = iwGyroAxRaw / DIVIDE_VALUE_GYRO;
    iwGyroAyRaw = array_ibRecData[2] <<8|array_ibRecData[3];
    g_idwGyryAy = iwGyroAyRaw / DIVIDE_VALUE_GYRO;
    iwGyroAzRaw = array_ibRecData[4] <<8|array_ibRecData[5];
    g_idwGyryAz = iwGyroAzRaw / DIVIDE_VALUE_GYRO;
}
/*===================end MPU-6050==============================================*/

/*===================== DHT 11=================================================*/
/**
 * @func   microDelay
 * @brief  delay 1 ms
 * @param  None
 * @retval None
 */
void_t microDelay (u16_t wDelay){
    HAL_TIM_SET_COUNTER(&htim1, 0);
    while (HAL_TIM_GET_COUNTER(&htim1) < wDelay);
}
/**
 * @func   DHT11_start
 * @brief  read Gyro
 * @param  None
 * @retval None
 */
i8_t DHT11_start (void_t){
    i8_t Response = 0;
    GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
    GPIO_InitStructPrivate.Pin = DHT11_PIN;
    GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate);   // set the pin as output
    HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);         // pull the pin low
    HAL_Delay(20);                                        // wait for 20ms
    HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);         // pull the pin high
    microDelay (30);                                      // wait for 30us
    GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate);   // set the pin as input
    microDelay (40);
    if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))){
        microDelay (80);
        if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
    }
    g_dwPreMillis = HAL_GetTick();
    g_dwCurrentMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) &&g_dwPreMillis + 2 >g_dwCurrentMillis){
        g_dwCurrentMillis = HAL_GetTick();
    }
    return Response;
}
/**
 * @func   DHT11_read
 * @brief  read data from DHT11
 * @param  None
 * @retval None
 */
i8_t DHT11_Read (void_t){
    i8_t a,b;
    for (a=0;a<8;a++){
        g_dwPreMillis = HAL_GetTick();
        g_dwCurrentMillis = HAL_GetTick();
        while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) &&g_dwPreMillis + 2 >g_dwCurrentMillis){  
            g_dwCurrentMillis = HAL_GetTick();
            }
        microDelay (40);  
        if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))){
            b&= ~(1<<(7-a));
        }
        else{
            b|= (1<<(7-a));
            g_dwPreMillis = HAL_GetTick();
            g_dwCurrentMillis = HAL_GetTick();
        }
        while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) &&g_dwPreMillis + 2 >g_dwCurrentMillis){  
            g_dwCurrentMillis = HAL_GetTick();
        }
    }
    return b;
}
/*===============================END DHT-11==============================================*/

/*===============================DEFINE EXECUTION TASKS==================================*/
/**
 * @func   senToLCD
 * @brief  send data to LCD
 * @param  None
 * @retval None
 */
void_t sendToLCD(i32_t idwTemp, i32_t idwAccelAx, i32_t idwAccelAy, i32_t idwAccelAz){
	  //send data to LCD
		lcdClear();
		sprintf(buff, "%0.1f", idwAccelAx); 
		lcdSendString("AccelAx=");
		lcdSendString(g_ibBuff);
		lcdPutCur(0, 8);
		sprintf(buff, "%0.1f", idwAccelAy); 
		lcdSendString("AccelAy=");
		lcdSendString(buff);
		lcdPutCur(1,0);
		sprintf(buff, "%0.1f", idwAccelAz);
		lcdSendString("AccelAz=");
		lcdSendString(buff);
		lcdPutCur(1, 8);
		sprintf(buff, "%0.1f", idwTemp); 
		lcdSendString("T=");
		lcdSendString(buff);
}
/**
 * @func   getDataDHT
 * @brief  read data from DHT
 * @param  None
 * @retval None
 */
void_t getDataDHT(void_t){
		if(DHT11_Start()){
        g_ibHumidityIntegral = DHT11_Read(); 
        g_ibHumidityDecimal = DHT11_Read(); 
        g_ibCelsiusIntegral = DHT11_Read(); 
        g_ibCelsiusDecimal = DHT11_Read(); 
        g_ibSum = DHT11_Read(); 
        if (RHI + RHD + TCI + TCD == SUM){
            g_idwCelsius    = (i32_t)g_ibCelsiusIntegral + (i32_t)(g_ibCelsiusDecimal/10.0);
            g_idwFahrenheit =  g_idwCelsius  * 9/5 + 32;
            g_idwHumidity   = (i32_t)g_ibHumidityIntegral + (i32_t)(g_ibHumidityDecimal/10.0);
        }
    }
}
/**
 * @func   senToCOM
 * @brief  send data to heculus
 * @param  None
 * @retval None
 */
void_t sendToCOM(void_t){
	  printf("g_idwAccelAx: %0.1f, g_idwAccelAy:%0.1f, g_idwAccelAz:%0.1f, T: %0.1f\n", g_idwAccelAx, Ay, g_idwAccelAz, tCelsius);
}
/*============================END DEFINE EXECUTION TASKS===========================*/
/**
  * @brief System Clock Configuration
  * @retval None
  */
void_t systemClockConfig(void_t){
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

/** Initializes the RCC Oscillators according to the specified parameters
** in the RCC_OscInitTypeDef structure.*/
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        errorHandler();
    }
/** Initializes the CPU, AHB and APB buses clocks
*/
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK){
        errorHandler();
    }
}
/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void_t MX_I2C1_Init(void_t){
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        errorHandler();
    }
}
/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void_t MX_TIM1_Init(void_t){
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 71;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        errorHandler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK){
        errorHandler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK){
        errorHandler();
    } 
}
/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void_t MX_USART2_UART_Init(void_t)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK){
        errorHandler();
    }
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void_t MX_GPIO_Init(void_t){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

    /*Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PB10 */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/********************************************************************************
 *
 * 
 *  Task 1(P: Normal):      Doc cam bien DHT va put vao mail queue
 *                          Thoi gian doc cam bien DHT 1s/lan
 *
 *  Task 2(P: Normal):      Doc cam bien MPU6050 va put vao mail queue
 *                          Thoi gian doc cam bien MPU6050 50ms/lan 
 * 
 *  Task 3(P:AboveNormal):  Get du lieu tu mail queue va hien thi len LCD
 * 
 *  Task 4:(P: High)        In ra Hercules khi co ngat UART xay ra (bang cach gui ki tu 'h')
 * 
 * 
 ******************************************************************************/
/**
 * @func   HAL_UART_RxCpltCallback
 * @brief  xu ly ngat uart
 * @param  None
 * @retval None
 */
void_t HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
		BaseType_t xHigher;
		if (ibRxData == 'h') {
				xHigher = xTaskResumeFromISR(sendCOMHandle);
				portEND_SWITCHING_ISR(xHigher);
		}
		HAL_UART_Receive_IT(&huart2, &ibRxData, 1);
}
/**
 * @func   startSendCOMTask
 * @brief  gui data len herculus
 * @param  None
 * @retval None
 */
void_t startSendCOMTask(void_t const * argument) {
   	osEvent recvTaskData;//su kien 
		while(1) {
				vTaskSuspend(NULL);
				printf("Lay du lieu tu ngat\n");
				recvTaskData = osMailGet(DataMailQueueHandle, 100);
				array_ibDataString *data = recvTaskData.value.p;//chi ra rang thong bao la mot con tro
				printf("g_idwAccelAx: %0.1f, g_idwAccelAy: %0.1f, g_idwAccelAz: %0.1f, T: %0.1f \n", data->dataAccelX, data->dataAccelY, data->dataAccelZ, data->temp);
				osMailFree(DataMailQueueHandle, data);
		}
}
/**
 * @func   startReadTempTask
 * @brief  task doc nhiet do
 * @param  None
 * @retval None
 */
void_t startReadTempTask(void_t const * argument){
		while(1){
		   	getDataDHT();
	  		//phan bo hang doi thu va dien vao no du lieu
	  		array_ibDataString *ibTaskData = osMailAlloc(DataMailQueueHandle, 100);
	 	  	ibTaskData->type = 1;
	  		ibTaskData->temp = tCelsius;
				ibTaskData->dataAccelX = g_idwAccelAx;
				ibTaskData->dataAccelY = g_idwAccelAy;
				ibTaskData->dataAccelZ = g_idwAccelAz;
				ibTaskData->count = wCount;
				osMailPut(DataMailQueueHandle,(void_t *)ibTaskData);
				wCount++;
				osDelay(1000);
		}
}
/**
 * @func   startReadAccelTask
 * @brief  task doc gia toc
 * @param  None
 * @retval None
 */

void_t startReadAccelTask(void_t const * argument){
		while(1){
				MPU6050_readAccel();
				//phan bo hang doi thu va dien du lieu vao do
				array_ibDataString *ibTaskData = osMailAlloc(DataMailQueueHandle, 100);
				ibTaskData->type = 2;
				ibTaskData->temp = tCelsius;
				ibTaskData->dataAccelX = g_idwAccelAx;
				ibTaskData->dataAccelY = g_idwAccelAy;
				ibTaskData->dataAccelZ = g_idwAccelAz;
				ibTaskData->count = wCount;
				osMailPut(DataMailQueueHandle,(void_t *)ibTaskData);
				wCount++;
				osDelay(200);
		}
}
/**
 * @func   startSendDataTask
 * @brief  task hien thi LCD
 * @param  None
 * @retval None
 */
void_t startSendDataTask(void_t const * argument){
		osEvent recvTaskData;
		while(1){
				recvTaskData = osMailGet(DataMailQueueHandle, 100);
				array_ibDataString *data = recvTaskData.value.p;
				if (data->type == 1) {
				  	sendToLCD(data->temp, data->dataAccelX, data->dataAccelY, data->dataAccelZ);
				  	osMailFree(DataMailQueueHandle, data);
					  wCount--;
				} else if (data->type == 2) {
					  sendToLCD(data->temp, data->dataAccelX, data->dataAccelY, data->dataAccelZ); 
					  osMailFree(DataMailQueueHandle, data);
				  	wCount--;
					  osDelay(200);
				} 
		}
}
