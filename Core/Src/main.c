/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "string.h"
#include "adxl345.h"
#include "tla2024.h"
#include "dht22.h"
#include "apds9960.h"
#include "usart.h"
#include "utilites.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CV_EXT_INTERRUPT_ID 	0x05
#define TIM11_DELAY_ID			0x06


#define RASP_SHTDN_DELAY		180000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

osThreadId defaultTaskHandle;
osThreadId lightMeterHandle;
osThreadId accelHandle;
osThreadId tempMeasHandle;
osThreadId uartCommHandle;
osThreadId tempHumMeasHandle;
osMessageQId onOffQueueHandle;
osMutexId I2C2MutexHandle;
/* USER CODE BEGIN PV */
osMailQId qSensorsHandle;
uint16_t tickCounter = 0;
uint8_t currentVoltageRxBuf [CV_RX_BUF_SIZE];
uint8_t raspRxBuf [RASP_RX_BUF_SIZE];
uint8_t fanAutoFlag = 0;
int16_t accelX = 0;
int16_t accelY = 0;
int16_t accelZ = 0;

uint8_t intSource = 0;
float Xresult;
float Yresult;
float Zresult;

uint8_t shutdownFlag = 0;
uint32_t shutdownCounter = 0;
uint16_t gerconCounter = 0;
uint16_t cvRequestCounter = 0;
uint8_t gerconState = 0;

uint32_t osTickCounter = 0;
uint32_t osTickCounterOld = 0;

const uint8_t cvTimeoutResponse[8] = {0xAA, 0x0F, 0x08, 0x11, 0x01, 0, 0, 0x55};

extern uint8_t flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM13_Init(void);
void StartDefaultTask(void const * argument);
void lightMeterTask(void const * argument);
void accelTask(void const * argument);
void tempMeasTask(void const * argument);
void uartCommTask(void const * argument);
void TempHumTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM7_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(TXRX6_GPIO_Port, TXRX6_Pin, RESET);

  HAL_GPIO_WritePin(SENSORS_PWR_GPIO_Port, SENSORS_PWR_Pin, SET);
  HAL_TIM_Base_Start_IT(&htim11);

//  	HAL_I2C_DeInit(&hi2c2);
	TLA2024_Init();
	APDS9960_Init();
	HAL_TIM_Base_Stop_IT(&htim13);
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of I2C2Mutex */
  osMutexDef(I2C2Mutex);
  I2C2MutexHandle = osMutexCreate(osMutex(I2C2Mutex));

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
  /* definition and creation of onOffQueue */
  osMessageQDef(onOffQueue, 8, uint16_t);
  onOffQueueHandle = osMessageCreate(osMessageQ(onOffQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	osMailQDef(Qsensors, MAIL_SIZE, sensorsData);
	qSensorsHandle = osMailCreate(osMailQ(Qsensors), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of lightMeter */
  osThreadDef(lightMeter, lightMeterTask, osPriorityNormal, 0, 256);
  lightMeterHandle = osThreadCreate(osThread(lightMeter), NULL);

  /* definition and creation of accel */
  osThreadDef(accel, accelTask, osPriorityNormal, 0, 256);
  accelHandle = osThreadCreate(osThread(accel), NULL);

  /* definition and creation of tempMeas */
  osThreadDef(tempMeas, tempMeasTask, osPriorityNormal, 0, 256);
  tempMeasHandle = osThreadCreate(osThread(tempMeas), NULL);

  /* definition and creation of uartComm */
  osThreadDef(uartComm, uartCommTask, osPriorityNormal, 0, 256);
  uartCommHandle = osThreadCreate(osThread(uartComm), NULL);

  /* definition and creation of tempHumMeas */
  osThreadDef(tempHumMeas, TempHumTask, osPriorityNormal, 0, 128);
  tempHumMeasHandle = osThreadCreate(osThread(tempHumMeas), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 124;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 16799;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 167;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 167;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 19999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 19200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SENSORS_PWR_Pin|GPIO__12V_1_Pin|GPIO__5V_1_Pin|TXRX6_Pin
                          |DHT22_3_Pin|DHT22_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO__12V_2_Pin|TXRX2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FAN_2_Pin|FAN_Pin|GPIO__12V_3_Pin|RASP_KEY_Pin
                          |DHT22_1_Pin|ALT_KEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO3_RASP_Pin|CS1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : SENSORS_PWR_Pin GPIO__12V_1_Pin GPIO__5V_1_Pin TXRX6_Pin
                           DHT22_3_Pin DHT22_2_Pin */
  GPIO_InitStruct.Pin = SENSORS_PWR_Pin|GPIO__12V_1_Pin|GPIO__5V_1_Pin|TXRX6_Pin
                          |DHT22_3_Pin|DHT22_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ADXL2_INT_Pin */
  GPIO_InitStruct.Pin = ADXL2_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADXL2_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WKUP_Pin */
  GPIO_InitStruct.Pin = WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WKUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO__12V_2_Pin TXRX2_Pin GPIO3_RASP_Pin */
  GPIO_InitStruct.Pin = GPIO__12V_2_Pin|TXRX2_Pin|GPIO3_RASP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FAN_2_Pin FAN_Pin GPIO__12V_3_Pin RASP_KEY_Pin
                           DHT22_1_Pin ALT_KEY_Pin */
  GPIO_InitStruct.Pin = FAN_2_Pin|FAN_Pin|GPIO__12V_3_Pin|RASP_KEY_Pin
                          |DHT22_1_Pin|ALT_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS2_Pin */
  GPIO_InitStruct.Pin = CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO17_RASP_Pin */
  GPIO_InitStruct.Pin = GPIO17_RASP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO17_RASP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS1_Pin */
  GPIO_InitStruct.Pin = CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GERCON_Pin */
  GPIO_InitStruct.Pin = GERCON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GERCON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void allConsumersEnable(void) {

//	HAL_GPIO_WritePin(SENSORS_PWR_GPIO_Port, SENSORS_PWR_Pin, SET);
	HAL_GPIO_WritePin(RASP_KEY_GPIO_Port, RASP_KEY_Pin, SET);
	HAL_GPIO_WritePin(ALT_KEY_GPIO_Port, ALT_KEY_Pin, SET);
	HAL_GPIO_WritePin(GPIO__12V_1_GPIO_Port, GPIO__12V_1_Pin, SET);
	HAL_GPIO_WritePin(GPIO__12V_2_GPIO_Port, GPIO__12V_2_Pin, SET);
	HAL_GPIO_WritePin(GPIO__12V_3_GPIO_Port, GPIO__12V_3_Pin, SET);
	HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, SET);
	HAL_GPIO_WritePin(FAN_2_GPIO_Port, FAN_2_Pin, SET);
	HAL_GPIO_WritePin(GPIO__5V_1_GPIO_Port, GPIO__5V_1_Pin, SET);

}

void allConsumersDisable(void) {
//	HAL_GPIO_WritePin(SENSORS_PWR_GPIO_Port, SENSORS_PWR_Pin, RESET);
	HAL_GPIO_WritePin(ALT_KEY_GPIO_Port, ALT_KEY_Pin, RESET);
	HAL_GPIO_WritePin(GPIO__12V_1_GPIO_Port, GPIO__12V_1_Pin, RESET);
	HAL_GPIO_WritePin(GPIO__12V_2_GPIO_Port, GPIO__12V_2_Pin, RESET);
	HAL_GPIO_WritePin(GPIO__12V_3_GPIO_Port, GPIO__12V_3_Pin, RESET);
	HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, RESET);
	HAL_GPIO_WritePin(FAN_2_GPIO_Port, FAN_2_Pin, RESET);
	HAL_GPIO_WritePin(GPIO__5V_1_GPIO_Port, GPIO__5V_1_Pin, RESET);
}


void setTxMode (uint8_t uartNo){
	switch (uartNo){
	case 2:
		HAL_GPIO_WritePin(TXRX2_GPIO_Port, TXRX2_Pin, SET);
		break;
	case 6:
		HAL_GPIO_WritePin(TXRX6_GPIO_Port, TXRX6_Pin, SET);
		break;
	}
}

void setRxMode (uint8_t uartNo){
	switch (uartNo){
	case 2:
		HAL_GPIO_WritePin(TXRX2_GPIO_Port, TXRX2_Pin, RESET);
		break;
	case 6:
		HAL_GPIO_WritePin(TXRX6_GPIO_Port, TXRX6_Pin, RESET);
		break;
	default:
		break;
	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

	if (huart->Instance == USART2) {
		setRxMode(2);
	}
	if (huart->Instance == USART6) {
		setRxMode(6);
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//	if (GPIO_Pin == GPIO_PIN_0) {
//		osSignalSet(defaultTaskHandle, CV_EXT_INTERRUPT_ID);
//	}
	if (GPIO_Pin == GPIO_PIN_1) {
		osSignalSet(accelHandle, 0x01);
	}
	if (GPIO_Pin == GPIO_PIN_12) {
		osMessagePut(onOffQueueHandle, RASP_17_INT_ID, 0);
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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	osEvent event1;
	static uint8_t powerState = 0;

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, raspRxBuf, RASP_RX_BUF_SIZE);
	__HAL_UART_ENABLE_IT (&huart6, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart6, currentVoltageRxBuf, CV_RX_BUF_SIZE);

	/* Infinite loop */
	for (;;) {

		event1 = osMessageGet(onOffQueueHandle, osWaitForever);
		if (event1.status == osEventMessage){
			switch (powerState) {
			case DISABLED:

				if (event1.value.v == ENGINE_START_ID){
					allConsumersEnable();
					powerState = ENABLED;
				}
				break;

			case ENABLED:
				if (event1.value.v == ENGINE_STOP_ID){

					powerState = RASPBERRY_WAIT;
				}
				break;

			case RASPBERRY_WAIT:
				if (event1.value.v == RASP_17_INT_ID) {
					HAL_GPIO_WritePin(RASP_KEY_GPIO_Port, RASP_KEY_Pin, RESET);
					powerState = DISABLED;
				}
				break;
			}
		}
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_lightMeterTask */
/**
* @brief Function implementing the lightMeter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lightMeterTask */
__weak void lightMeterTask(void const * argument)
{
  /* USER CODE BEGIN lightMeterTask */
  /* Infinite loop */

  /* USER CODE END lightMeterTask */
}

/* USER CODE BEGIN Header_accelTask */
/**
* @brief Function implementing the accel thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_accelTask */
void accelTask(void const * argument)
{
  /* USER CODE BEGIN accelTask */

	osEvent evt;
	uint8_t buffer[6];
	sensorsData *sensors = {0};
//	osDelay(1000);
	HAL_I2C_Init(&hi2c2);
	ADXL345_Init();

	/* Infinite loop */
	for (;;) {
		evt = osSignalWait(0x01, osWaitForever);
		if (evt.status == osEventSignal) {
			if (osMutexWait(I2C2MutexHandle, 50) != osOK){
			}
			intSource = getInterruptSource();
			ADXL345_Read(buffer);
			osMutexRelease(I2C2MutexHandle);
			accelX =(int16_t) buffer[1] << 8;
			accelX |=(int16_t) buffer[0];
			Xresult = (float)accelX * 0.0156;
			accelY = (int16_t) buffer[3] << 8;
			accelY |= (int16_t)buffer[2];
			Yresult = (float)accelY * 0.0156;
			accelZ = (int16_t)buffer[5] << 8;
			accelZ |= (int16_t)buffer[4];
			Zresult = (float)accelZ * 0.0156;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			osDelay(200);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
//			sensors = osMailAlloc(qSensorsHandle, osWaitForever);
//			sensors->source = ADXL_TASK;
//			sensors->size = ADXL_SIZE;
//			memcpy (sensors->payload, buffer, 6);
//			osMailPut(qSensorsHandle, sensors);
		}
	}
  /* USER CODE END accelTask */
}

/* USER CODE BEGIN Header_tempMeasTask */
/**
* @brief Function implementing the tempMeas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tempMeasTask */
void tempMeasTask(void const * argument)
{
  /* USER CODE BEGIN tempMeasTask */
	static uint8_t buffer0[2] = {0};
	static uint8_t buffer1[2] = {0};
	sensorsData *sensors = {0};

	osDelay(500);
	/* Infinite loop */
	for (;;) {
		if (osMutexWait(I2C2MutexHandle, 50) != osOK){
		}
		TLA2024_Read(0, buffer0);
//		TLA2024_Read(1, buffer1);
		osMutexRelease(I2C2MutexHandle);
		sensors = osMailAlloc(qSensorsHandle, osWaitForever);
		sensors->source = TLA2024_TASK_SOURCE;
		sensors->size = TLA2024_SIZE;
		memset(sensors->payload, 0, 16);
		memcpy (sensors->payload, buffer0, 2);
		memcpy (sensors->payload+2, buffer1, 2);
		osMailPut(qSensorsHandle, sensors);
		osDelay(500);
	}
  /* USER CODE END tempMeasTask */
}

/* USER CODE BEGIN Header_uartCommTask */
/**
* @brief Function implementing the uartComm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uartCommTask */
__weak void uartCommTask(void const * argument)
{
  /* USER CODE BEGIN uartCommTask */

  /* USER CODE END uartCommTask */
}

/* USER CODE BEGIN Header_TempHumTask */
/**
* @brief Function implementing the tempHumMeas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TempHumTask */
void TempHumTask(void const * argument)
{
  /* USER CODE BEGIN TempHumTask */
	sensorsData *sensors = {0};

	osDelay(200);
	/* Infinite loop */
	for (;;) {
//		DHT22_Read(0);
//		sensors = osMailAlloc(qSensorsHandle, osWaitForever);
//		sensors->source = DHT22_TASK;
//		sensors->size = DHT22_SIZE;
//		memset(sensors->payload, 0, 16);
//		sensors->payload[0] = dht22_buf.tMSB; sensors->payload[1] = dht22_buf.tLSB;
//		sensors->payload[2] = dht22_buf.hMSB; sensors->payload[3] = dht22_buf.hLSB;
//		osMailPut(qSensorsHandle, sensors);
		osDelay(200);
	}
  /* USER CODE END TempHumTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	sensorsData *sensor, *sensor1;
	uint8_t request[6] = {PACKET_HEADER, CV_REQ_PACK_ID, CV_REQ_SIZE, CMD_CV_REQUEST, 0, PACKET_FOOTER};
	request[4] = get_check_sum(request, 6);

  if (htim->Instance == TIM11){


	  if (tickCounter < 2000){
		  tickCounter++;
	  } else {
		  tickCounter = 0;
		  osSignalSet(uartCommHandle, 0x02);
	  }

	  if (cvRequestCounter < 1000){
		  cvRequestCounter++;
	  } else {
		  cvRequestCounter = 0;
		  sensor = osMailAlloc(qSensorsHandle, 0);
		  sensor->source = CV_REQ_SOURCE;
		  sensor->size = CV_REQ_SIZE;
		  memcpy(sensor->payload, request, CV_REQ_SIZE);
		  osMailPut(qSensorsHandle, sensor);
	  }

//	  switch (gerconState){
//	  case 0:
//		  if (!HAL_GPIO_ReadPin(GERCON_GPIO_Port, GERCON_Pin)){
//			  if (gerconCounter < 50){
// 				  gerconCounter++;
//			  } else {
//				  gerconCounter = 0;
//				  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//				  gerconState++;
//			  }
//		  } else {
//			  gerconCounter = 0;
//		  }
//		  break;
//	  case 1:
//		  if (HAL_GPIO_ReadPin(GERCON_GPIO_Port, GERCON_Pin)){
//			  if (gerconCounter < 500){
//				  gerconCounter++;
//			  } else {
//				  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
//				  gerconCounter = 0;
//				  gerconState = 0;
//			  }
//		  } else {
//			  gerconCounter = 0;
//		  }
//		  break;
//	  }
  }

  if (htim->Instance == TIM13) {
	  HAL_TIM_Base_Stop_IT(&htim13);
	  __HAL_TIM_CLEAR_IT(&htim13, TIM_IT_UPDATE);
	  sensor1 = osMailAlloc(qSensorsHandle, 0);
	  sensor1->source = CV_RESP_SOURCE;
	  sensor1->size = RASP_RESP_SIZE;
	  sensor1->payload[6] = get_check_sum(sensor1->payload, 8);
	  memcpy (sensor1->payload, cvTimeoutResponse,8);

	  osTickCounter = xTaskGetTickCountFromISR() - osTickCounterOld;
	  osMailPut(qSensorsHandle, sensor1);
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
