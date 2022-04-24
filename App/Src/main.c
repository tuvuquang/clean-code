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
#define __DEBUG__
#define BUFF_SIZE                    3

// LCD ADDRESS
#define SLAVE_ADDRESS_LCD            0x4E

// DHT11 PORT & PIN
#define DHT11_PORT                   GPIOB
#define DHT11_PIN                    GPIO_PIN_10

/* USER CODE BEGIN PFP */
#ifdef _GNUC_
#define PUTi8_t_PROTOTYPE i32_t __io_putchar(i32_t ch)
#else
#define PUTCHAR_PROTOTYPE i32_t fputc(i32_t ch, FILE *f)
#endif /* _GNUC_ */

// TEM & ACCEL
typedef struct
{
	i8_t type; 
	float dataAccelX;
	float dataAccelY;
	float dataAccelZ;
	float temp;
	i8_t cnt;
} DataTemAndAccel_t;
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//PV_UART
i8_t ibBuff[8];
i8_t ibDataRx;
i8_t ibReceive;
i8_t ibReceiveData[BUFF_SIZE]
i8_t ibRxIndex = 0;
i8_t ibCountPrint = 0;
i8_t ibRxData;
//end PV_UART

u16_t wCount = 0;
i16_t iwAccelAxRaw, iwAccelAyRaw, iwAccelAzRaw;
float Ax, Ay, Az;
i16_t iwGyroAxRaw, iwGyroAyRaw, Gyro_Z_Raw;
float Gx, Gy, Gz;

i8_ RHI, RHD, TCI, TCD, SUM;
u32_t pMillis, cMillis;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;

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


//osThreadId defaultTaskHandle;
osMessageQId myQueue01Handle;
osThreadId readTempHandle;
osThreadId readAccelHandle;
osThreadId sendDataHandle;
osThreadId sendCOMHandle;   

osMailQId DataMailQueueHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
 // task hien thi len Hercules
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
/* Place your implementation of fputc here */
/* e.g. write a character to the USART */
HAL_UART_Transmit(&huart2, (i8_t *)&ch, 1, 100);
return ch;
}


/*==========================START I2C_LCD======================================*/
/**
 * @func   lcdSendCmd
 * @brief  LCD send command
 * @param  None
 * @retval None
 */
void_t lcdSendCmd (i8_t cmd)
{
  i8_t data_u, data_l;
	i8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(i8_t *) data_t, 4, 100);
}

/**
 * @func   lcdSendData
 * @brief  send dat to LCD
 * @param  None
 * @retval None
 */
void_t lcdSendData (i8_t data)
{
	i8_t data_u, data_l;
	i8_t data_t[4];
  data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(i8_t *) data_t, 4, 100);
}

/**
 * @func   lcdClear
 * @brief  clear LCD
 * @param  None
 * @retval None
 */
void_t lcdClear (void_t)
{
	lcdSendCmd (0x80);
	for (i32_t i=0; i<16; i++)
	{
		lcdSendData (' ');
	}
	lcdSendCmd(0x80);    
}

/**
 * @func   lcdPutCur
 * @brief  
 * @param  None
 * @retval None
 */

void_t lcdPutCur(i32_t idwRow, i32_t idwCol)
{
    switch (idwRow)
    {
        case 0:
            idwCol |= 0x80;
            break;
        case 1:
            idwCol |= 0xC0;
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

void_t lcdInit (void_t)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcdSendCmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcdSendCmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcdSendCmd (0x30);
	HAL_Delay(10);
	lcdSendCmd (0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	lcdSendCmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcdSendCmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcdSendCmd (0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcdSendCmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcdSendCmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

/**
 * @func   lcdSendString
 * @brief  send string to LCD
 * @param  None
 * @retval None
 */
void_t lcdSendString (i8_t *str)
{
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
i8_t check, data;
	HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,WHO_AM_I_REG,1,&check,1,1000);
	if(check == 104){
	data = 0x00;	
	HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,PWR_MGMT_1_REG,1,&data,1,1000);
	data = 0x07;
	HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,SMPLRT_DIV_REG,1,&data,1,1000);
	data = 0x00;
  HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,ACCE_CONFIG_REG,1,&data,1,1000);
  HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR,GYRO_CONFIG_REG,1,&data,1,1000);		
	}
}

/**
 * @func   MPU_ReadAccel
 * @brief  read Accel
 * @param  None
 * @retval None
 */
void_t MPU6050_readAccel(void_t){
i8_t Rec_data[6];
HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR, ACCE_XOUT_H_REG, 1, Rec_data,6,1000);
iwAccelAxRaw = Rec_data[0] <<8|Rec_data[1];
Ax = iwAccelAxRaw/16384.0;
iwAccelAyRaw = Rec_data[2] <<8|Rec_data[3];
Ay = iwAccelAyRaw/16384.0;
iwAccelAzRaw = Rec_data[4] <<8|Rec_data[5];
Az = iwAccelAzRaw/16384.0;
}

/**
 * @func   MPU6050_readGyro
 * @brief  read Gyro
 * @param  None
 * @retval None
 */

void_t MPU6050_readGyro(void_t){
i8_t Rec_data[6];
HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,GYRO_XOUT_H_REG,1,Rec_data,6,1000);
iwGyroAxRaw = Rec_data[0] <<8|Rec_data[1];
Gx = iwGyroAxRaw/131.0;
iwGyroAyRaw = Rec_data[2] <<8|Rec_data[3];
Gy = iwGyroAyRaw/131.0;
Gyro_Z_Raw = Rec_data[4] <<8|Rec_data[5];
Gz = Gyro_Z_Raw/131.0;
}
/*===================end MPU-6050==============================================*/


/*===================== DHT 11=================================================*/

/**
 * @func   microDelay
 * @brief  delay 1 ms
 * @param  None
 * @retval None
 */
void_t microDelay (u16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}
/**
 * @func   DHT11_start
 * @brief  read Gyro
 * @param  None
 * @retval None
 */
i8_t DHT11_start (void_t)
{
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
  if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
  {
    microDelay (80);
    if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
  {
    cMillis = HAL_GetTick();
  }
  return Response;
}
/**
 * @func   DHT11_read
 * @brief  read data from DHT11
 * @param  None
 * @retval None
 */
i8_t DHT11_Read (void_t)
{
  i8_t a,b;
  for (a=0;a<8;a++)
  {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go high
      cMillis = HAL_GetTick();
    }
    microDelay (40);   // wait for 40 us
    if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go low
      cMillis = HAL_GetTick();
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
void_t sendToLCD(float temp, float x, float y, float z)
{
	//send data to LCD
		lcdClear();
		sprintf(buff, "%0.1f", x); //gia toc truc x
		lcdSendString("Ax=");
		lcdSendString(ibBuff);
		lcdPutCur(0, 8);
		sprintf(buff, "%0.1f", y); //gia toc truc y
		lcdSendString("Ay=");
		lcdSendString(buff);
		lcdPutCur(1,0);
		sprintf(buff, "%0.1f", z); //gia toc truc z
		lcdSendString("Az=");
		lcdSendString(buff);
		lcdPutCur(1, 8);
		sprintf(buff, "%0.1f", temp); //nhiet do lay tu DHT-11
		lcdSendString("T=");
		lcdSendString(buff);
}

/**
 * @func   getDataDHT
 * @brief  read data from DHT
 * @param  None
 * @retval None
 */
void_t getDataDHT(void_t)
{
		if(DHT11_Start())
    {
      RHI = DHT11_Read(); // Relative humidity integral
      RHD = DHT11_Read(); // Relative humidity decimal
      TCI = DHT11_Read(); // Celsius integral
      TCD = DHT11_Read(); // Celsius decimal
      SUM = DHT11_Read(); // Check sum
      if (RHI + RHD + TCI + TCD == SUM)
      {
        // Can use RHI and TCI for any purposes if whole number only needed
        tCelsius = (float)TCI + (float)(TCD/10.0);
        tFahrenheit = tCelsius * 9/5 + 32;
        RH = (float)RHI + (float)(RHD/10.0);
        // Can use tCelsius, tFahrenheit and RH for any purposes
      }
    }
}

/**
 * @func   senToCOM
 * @brief  send data to heculus
 * @param  None
 * @retval None
 */
void_t sendToCOM(void_t)
{
	printf("Ax: %0.1f, Ay:%0.1f, Az:%0.1f, T: %0.1f\n", Ax, Ay, Az, tCelsius);
}
/*============================END DEFINE EXECUTION TASKS===========================*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void_t)
{
  HAL_Init();
  systemClockConfig();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	MPU6050_Init();
	HAL_UART_Receive_IT(&huart2, &ibRxData, 1);
	lcdInit();
	lcdSendString("Initializing...");
	
  osMessageQDef(myQueue01, 14, u32_t);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);
	osMailQDef(DataMailQueue, 16, data_t);
	DataMailQueueHandle = osMailCreate(osMailQ(DataMailQueue), NULL); // khoi tao mail queue
  /* USER CODE END RTOS_QUEUES */
  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /*===============================Begin: Khoi tao cac task==============================*/
	osThreadDef(readAccelTask, startReadAccelTask, osPriorityNormal, 0, 96);
  readAccelHandle = osThreadCreate(osThread(readAccelTask), NULL);
	
	osThreadDef(sendDataTask, startSendDataTask, osPriorityAboveNormal, 0, 96);
  sendDataHandle = osThreadCreate(osThread(sendDataTask), NULL);
	
	osThreadDef(readTempTask, startReadTempTask, osPriorityNormal, 0, 96);
  readTempHandle = osThreadCreate(osThread(readTempTask), NULL);
	
	osThreadDef(sendCOMTask, startSendCOMTask, osPriorityHigh, 0, 96);
  sendCOMHandle = osThreadCreate(osThread(sendCOMTask), NULL);
/*===============================End: Khoi tao cac task===================================*/

  osKernelStart();

  
  while (1)
  {
 

  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void_t systemClockConfig(void_t)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void_t MX_I2C1_Init(void_t)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void_t MX_TIM1_Init(void_t)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

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
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void_t MX_USART2_UART_Init(void_t)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void_t MX_GPIO_Init(void_t)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
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
/***************************************************************************
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
 * 
 * 
 ******************************************************************************/


/**
 * @func   HAL_UART_RxCpltCallback
 * @brief  xu ly ngat uart
 * @param  None
 * @retval None
 */
		void_t HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
		{
			BaseType_t xHigher;
			if (ibRxData == 'h') 
			{
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

		void_t startSendCOMTask(void_t const * argument) 
		{
			osEvent recv_task_data;//su kien 
			while(1) 
			{
				vTaskSuspend(NULL);
				printf("Lay du lieu tu ngat\n");
				recv_task_data = osMailGet(DataMailQueueHandle, 100);
				data_t *data = recv_task_data.value.p;//chi ra rang thong bao la mot con tro
				printf("Ax: %0.1f, Ay: %0.1f, Az: %0.1f, T: %0.1f \n", data->dataAccelX, data->dataAccelY, data->dataAccelZ, data->temp);
				osMailFree(DataMailQueueHandle, data);
			}
		}

/**
 * @func   startReadTempTask
 * @brief  task doc nhiet do
 * @param  None
 * @retval None
 */
		void_t startReadTempTask(void_t const * argument)
		{
			while(1)
			{
				getDataDHT();
				//phan bo hang doi thu va dien vao no du lieu
				data_t *task_data = osMailAlloc(DataMailQueueHandle, 100);
				task_data->type = 1;
				task_data->temp = tCelsius;
				task_data->dataAccelX = Ax;
				task_data->dataAccelY = Ay;
				task_data->dataAccelZ = Az;
				task_data->cnt = wCount;
				osMailPut(DataMailQueueHandle,(void_t *)task_data);
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

		void_t startReadAccelTask(void_t const * argument)
		{
			while(1)
			{
				MPU6050_readAccel();
				//phan bo hang doi thu va dien du lieu vao do
				data_t *task_data = osMailAlloc(DataMailQueueHandle, 100);//phan bo hang doi thu trong mot chuoi va dien vao no voi du lieu
				task_data->type = 2;
				task_data->temp = tCelsius;
				task_data->dataAccelX = Ax;
				task_data->dataAccelY = Ay;
				task_data->dataAccelZ = Az;
				task_data->cnt = wCount;
				osMailPut(DataMailQueueHandle,(void_t *)task_data);
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
		void_t startSendDataTask(void_t const * argument)
		{
			osEvent recv_task_data;//su kien nhan task data
			
		//	u32_t recv_ISR_data;
			while(1)
			{
				
				recv_task_data = osMailGet(DataMailQueueHandle, 100);
				data_t *data = recv_task_data.value.p;
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

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
//void_t StartDefaultTask(void_t const * argument)
//{
//  /* USER CODE BEGIN 5 */
//  /* Infinite loop */
//  for(;;)
//  {
//    
//  }
  /* USER CODE END 5 */


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void_t HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void_t Error_Handler(void_t)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void_t assert_failed(i8_t *file, u32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
