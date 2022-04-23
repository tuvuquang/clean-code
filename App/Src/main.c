/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint8_t type; // 1: doc nhiet do, do am || 2: doc accel 
	float data_x;
	float data_y;
	float data_z;
	float temp;
	uint8_t cnt;
} data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define __DEBUG__

#define buffSize 3

#define SLAVE_ADDRESS_LCD 0x4E

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCE_CONFIG_REG 0x1C
#define ACCE_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

//osThreadId defaultTaskHandle;
osMessageQId myQueue01Handle;
/* USER CODE BEGIN PV */
//PV_UART____________________________________
char buff[8];
uint8_t dataRx;
uint8_t rx_Data[buffSize];
uint8_t rx_Index = 0;
uint8_t count_print = 0;

uint8_t rx_data;
//end PV_UART________________________________

uint16_t count = 0;
int16_t Accel_X_Raw, Accel_Y_Raw, Accel_Z_Raw;
float Ax, Ay, Az;
int16_t Gyro_X_Raw, Gyro_Y_Raw, Gyro_Z_Raw;
float Gx, Gy, Gz;

uint8_t RHI, RHD, TCI, TCD, SUM;
uint32_t pMillis, cMillis;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;

osThreadId readTempHandle;
osThreadId readAccelHandle;
osThreadId sendDataHandle;
osThreadId sendCOMHandle;   

osMailQId DataMailQueueHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
#ifdef _GNUC_
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* _GNUC_ */

void StartReadTempTask(void const * argument); // task doc nhiet do

void StartReadAccelTask(void const * argument); // task doc cam bien gia toc

void StartSendDataTask(void const * argument); // task hien thi len LCD

void StartSendCOMTask(void const * argument); // task hien thi len Hercules
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
/* Place your implementation of fputc here */
/* e.g. write a character to the USART */
HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
return ch;
}

//I2C LCD...........................................
void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x80);
	for (int i=0; i<16; i++)
	{
		lcd_send_data (' ');
	}
	lcd_send_cmd(0x80);    
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}


void lcd_init (void)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}
//end i2c_LCD________________________________________________________________
//MPU-6050___________________________________________________________________

void MPU6050_Init(void){
uint8_t check, data;
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
void MPU6050_Read_Accel(void){
uint8_t Rec_data[6];
HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR, ACCE_XOUT_H_REG, 1, Rec_data,6,1000);
Accel_X_Raw = Rec_data[0] <<8|Rec_data[1];
Ax = Accel_X_Raw/16384.0;
Accel_Y_Raw = Rec_data[2] <<8|Rec_data[3];
Ay = Accel_Y_Raw/16384.0;
Accel_Z_Raw = Rec_data[4] <<8|Rec_data[5];
Az = Accel_Z_Raw/16384.0;
}
void MPU6050_Read_Gyro(void){
uint8_t Rec_data[6];
HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,GYRO_XOUT_H_REG,1,Rec_data,6,1000);
Gyro_X_Raw = Rec_data[0] <<8|Rec_data[1];
Gx = Gyro_X_Raw/131.0;
Gyro_Y_Raw = Rec_data[2] <<8|Rec_data[3];
Gy = Gyro_Y_Raw/131.0;
Gyro_Z_Raw = Rec_data[4] <<8|Rec_data[5];
Gz = Gyro_Z_Raw/131.0;
}
//end MPU-6050__________________________________________________________
//DHT-11________________________________________________________________
void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

uint8_t DHT11_Start (void)
{
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT11_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
  HAL_Delay(20);   // wait for 20ms
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
  microDelay (30);   // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as input
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

uint8_t DHT11_Read (void)
{
  uint8_t a,b;
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
//end DHT-11___________________________________________________________________
//Define execution tasks
void sendToLCD(float temp, float x, float y, float z)
{
	//send data to LCD
		lcd_clear();
		sprintf(buff, "%0.1f", x); //gia toc truc x
		lcd_send_string("Ax=");
		lcd_send_string(buff);
		lcd_put_cur(0, 8);
		sprintf(buff, "%0.1f", y); //gia toc truc y
		lcd_send_string("Ay=");
		lcd_send_string(buff);
		lcd_put_cur(1,0);
		sprintf(buff, "%0.1f", z); //gia toc truc z
		lcd_send_string("Az=");
		lcd_send_string(buff);
		lcd_put_cur(1, 8);
		sprintf(buff, "%0.1f", temp); //nhiet do lay tu DHT-11
		lcd_send_string("T=");
		lcd_send_string(buff);
}

//Doc du lieu tu DHT-11
void getDataDHT(void)
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

//gui du lieu len hercules
void sendToCOM(void)
{
	printf("Ax: %0.1f, Ay:%0.1f, Az:%0.1f, T: %0.1f\n", Ax, Ay, Az, tCelsius);
}
//End define execution tasks
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	MPU6050_Init();
	HAL_UART_Receive_IT(&huart2, &rx_data, 1);
	lcd_init();
	lcd_send_string("Initializing...");
	/* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 14, uint32_t);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	osMailQDef(DataMailQueue, 16, data_t);
	DataMailQueueHandle = osMailCreate(osMailQ(DataMailQueue), NULL); // khoi tao mail queue
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /*===============================Begin: Khoi tao cac task=============================*/
	osThreadDef(readAccelTask, StartReadAccelTask, osPriorityNormal, 0, 96);
  readAccelHandle = osThreadCreate(osThread(readAccelTask), NULL);
	
	osThreadDef(sendDataTask, StartSendDataTask, osPriorityAboveNormal, 0, 96);
  sendDataHandle = osThreadCreate(osThread(sendDataTask), NULL);
	
	osThreadDef(readTempTask, StartReadTempTask, osPriorityNormal, 0, 96);
  readTempHandle = osThreadCreate(osThread(readTempTask), NULL);
	
	osThreadDef(sendCOMTask, StartSendCOMTask, osPriorityHigh, 0, 96);
  sendCOMHandle = osThreadCreate(osThread(sendCOMTask), NULL);
/*===============================End: Khoi tao cac task=============================*/

	
	
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
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
static void MX_I2C1_Init(void)
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
static void MX_TIM1_Init(void)
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
static void MX_USART2_UART_Init(void)
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
static void MX_GPIO_Init(void)
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


/*============================Ham xu ly ngat UART=====================================*/
		void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
		{
			BaseType_t xHigher;
			if (rx_data == 'h') 
			{
				xHigher = xTaskResumeFromISR(sendCOMHandle);
				portEND_SWITCHING_ISR(xHigher);
			}
			HAL_UART_Receive_IT(&huart2, &rx_data, 1);
		}

/*============================Ham gui len Hercules=====================================*/

		void StartSendCOMTask(void const * argument) 
		{
			osEvent recv_task_data;//su kien 
			while(1) 
			{
				vTaskSuspend(NULL);
				printf("Lay du lieu tu ngat\n");
				recv_task_data = osMailGet(DataMailQueueHandle, 100);
				data_t *data = recv_task_data.value.p;//chi ra rang thong bao la mot con tro
				printf("Ax: %0.1f, Ay: %0.1f, Az: %0.1f, T: %0.1f \n", data->data_x, data->data_y, data->data_z, data->temp);
				osMailFree(DataMailQueueHandle, data);
			}
		}

/*============================Ham doc nhiet do=====================================*/
		void StartReadTempTask(void const * argument)
		{
			while(1)
			{
				getDataDHT();
				//phan bo hang doi thu va dien vao no du lieu
				data_t *task_data = osMailAlloc(DataMailQueueHandle, 100);
				task_data->type = 1;
				task_data->temp = tCelsius;
				task_data->data_x = Ax;
				task_data->data_y = Ay;
				task_data->data_z = Az;
				task_data->cnt = count;
				osMailPut(DataMailQueueHandle,(void *)task_data);
				count++;
				osDelay(1000);
			}
		}
/*============================Ham doc gia toc=====================================*/

		void StartReadAccelTask(void const * argument)
		{
			while(1)
			{
				MPU6050_Read_Accel();
				//phan bo hang doi thu va dien du lieu vao do
				data_t *task_data = osMailAlloc(DataMailQueueHandle, 100);//phan bo hang doi thu trong mot chuoi va dien vao no voi du lieu
				task_data->type = 2;
				task_data->temp = tCelsius;
				task_data->data_x = Ax;
				task_data->data_y = Ay;
				task_data->data_z = Az;
				task_data->cnt = count;
				osMailPut(DataMailQueueHandle,(void *)task_data);
				count++;
				osDelay(200);
			}
		}
/*============================Ham hien thi len LCD=====================================*/
		void StartSendDataTask(void const * argument)
		{
			osEvent recv_task_data;//su kien nhan task data
			
		//	uint32_t recv_ISR_data;
			while(1)
			{
				
				recv_task_data = osMailGet(DataMailQueueHandle, 100);
				data_t *data = recv_task_data.value.p;
				if (data->type == 1) {
					sendToLCD(data->temp, data->data_x, data->data_y, data->data_z);
					osMailFree(DataMailQueueHandle, data);
					count--;
				} else if (data->type == 2) {
					sendToLCD(data->temp, data->data_x, data->data_y, data->data_z); 
					osMailFree(DataMailQueueHandle, data);
					count--;
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
//void StartDefaultTask(void const * argument)
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
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
void Error_Handler(void)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
