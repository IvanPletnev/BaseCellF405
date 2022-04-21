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
#include "eeprom.h"
#include "dimmer.h"
#include "CANSPI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CV_EXT_INTERRUPT_ID 	0x05
#define TIM11_DELAY_ID			0x06

#define RASP_OFF_COUNTER		500
#define RASP_SHTDN_DELAY		45000

#define TEMPERATURE				400
#define TEMP_HYSTERESE			20
#define TEMP_SENS_NORM			1
#define TEMP_SENSOR_FAIL		0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

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
osThreadId dimmerHandle;
osThreadId canRxHandle;
osThreadId canTxHandle;
osMessageQId onOffQueueHandle;
osMessageQId watchDogQHandle;
osMutexId I2C2MutexHandle;
osMutexId spiMutexHandle;
/* USER CODE BEGIN PV */

QueueHandle_t qSensorsHandle;
uint8_t qSensorsBuffer[ MAIL_SIZE * sizeof( sensorsData ) ];
StaticQueue_t qSensorsQueueBuffer;

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


uint32_t raspOffCounter = 0;
uint16_t stateChangeCounter = 0;
uint8_t raspOffState = 0;

uint16_t wakeUpPinCounter = 0;
uint8_t wakeUpFlag = 0;

uint32_t secondCounter = 0;

uint16_t pulseDuration = 0;
const uint8_t cvTimeoutResponse[8] = {0xAA, 0x0F, 0x08, 0x11, 0x01, 0, 0, 0x55};

extern uint8_t engineSwitchFlag;
extern uint16_t onBoardVoltage;
extern uint8_t engineState;
extern uint8_t misStatusByte0;
extern uint8_t misStatusByte1;
extern uint8_t cvStatusByteExtern;
extern uint8_t breaksStateTelem;
extern uint8_t queueStatusByte;
extern uint8_t queueStatusByte1;
//extern UBaseType_t mailInQueue;

uint16_t VirtAddVarTab[NB_OF_VAR];
uint8_t tempSensorState = 0;
uint8_t sensorsOnFlag = 1;

volatile unsigned long ulHighFrequencyTimerTicks;

int16_t debugTemp0;
int16_t debugTemp1;
int16_t debugTemp2;

__IO int32_t osMailFreeCounter = 0;
__IO uint32_t queueErrorCnt = 0;

static volatile sensorsData xSensor, xSensor1;
static volatile sensorsData *sensor = &xSensor;
static volatile sensorsData *sensor1 = &xSensor1;

uint8_t resetFlag = 0;
uint8_t gpio17State = 0;
uint8_t turnOffBreaksFlag = 0;

volatile uint8_t timeOutFlag = 0;
volatile uint32_t timeoutCnt = 0;
volatile uint8_t raspRestartFlag = 0;

static volatile uint8_t cvCounter = 0;

float pwmValue;
uint16_t pwm_Value;
uint16_t targetValue;
BH1750_device_t* monitor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM8_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);
void lightMeterTask(void const * argument);
void accelTask(void const * argument);
void tempMeasTask(void const * argument);
void uartCommTask(void const * argument);
extern void dimmerTask(void const * argument);
extern void canRxTask(void const * argument);
extern void canTxTask(void const * argument);

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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM7_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM10_Init();
  MX_TIM8_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(TXRX6_GPIO_Port, TXRX6_Pin, RESET);

  HAL_GPIO_WritePin(SENSORS_PWR_GPIO_Port, SENSORS_PWR_Pin, SET);

  HAL_TIM_Base_Start_IT(&htim11);

	TLA2024_Init();
	HAL_TIM_Base_Stop_IT(&htim13);
	HAL_TIM_Base_Stop_IT(&htim10);
	monitor = BH1750_init_dev_struct(&hi2c3, "monitor", true);
	BH1750_init_dev(monitor);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of I2C2Mutex */
  osMutexDef(I2C2Mutex);
  I2C2MutexHandle = osMutexCreate(osMutex(I2C2Mutex));

  /* definition and creation of spiMutex */
  osMutexDef(spiMutex);
  spiMutexHandle = osMutexCreate(osMutex(spiMutex));

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

  /* definition and creation of watchDogQ */
  osMessageQDef(watchDogQ, 4, uint16_t);
  watchDogQHandle = osMessageCreate(osMessageQ(watchDogQ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  qSensorsHandle = xQueueCreateStatic(MAIL_SIZE, sizeof (sensorsData), &qSensorsBuffer[0], &qSensorsQueueBuffer);

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

  /* definition and creation of dimmer */
  osThreadDef(dimmer, dimmerTask, osPriorityNormal, 0, 256);
  dimmerHandle = osThreadCreate(osThread(dimmer), NULL);

  /* definition and creation of canRx */
  osThreadDef(canRx, canRxTask, osPriorityNormal, 0, 256);
  canRxHandle = osThreadCreate(osThread(canRx), NULL);

  /* definition and creation of canTx */
  osThreadDef(canTx, canTxTask, osPriorityNormal, 0, 128);
  canTxHandle = osThreadCreate(osThread(canTx), NULL);

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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim7.Init.Period = 65535;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 16799;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 167;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 83;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 99;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart6.Init.BaudRate = 115200;
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
  HAL_GPIO_WritePin(GPIOA, GPIO__12V_2_Pin|TXRX2_Pin|CAM_ON_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : MCPINT_Pin */
  GPIO_InitStruct.Pin = MCPINT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MCPINT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WKUP_Pin GPIO17_Pin */
  GPIO_InitStruct.Pin = WKUP_Pin|GPIO17_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO__12V_2_Pin TXRX2_Pin CAM_ON_Pin GPIO3_RASP_Pin */
  GPIO_InitStruct.Pin = GPIO__12V_2_Pin|TXRX2_Pin|CAM_ON_Pin|GPIO3_RASP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ADXL2_INT_Pin */
  GPIO_InitStruct.Pin = ADXL2_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADXL2_INT_GPIO_Port, &GPIO_InitStruct);

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

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

void configureTimerForRunTimeStats(void)
{
	ulHighFrequencyTimerTicks = 0;
	HAL_TIM_Base_Start_IT(&htim14);
}

unsigned long getRunTimeCounterValue(void)
{
return ulHighFrequencyTimerTicks;
}

void setStatusBytes (void) {

	if (HAL_GPIO_ReadPin(SENSORS_PWR_GPIO_Port, SENSORS_PWR_Pin) == GPIO_PIN_SET){
		misStatusByte0 |= 0x01;
	} else {
		misStatusByte0 &= ~0x01;
	}
	if (HAL_GPIO_ReadPin(ALT_KEY_GPIO_Port, ALT_KEY_Pin) == GPIO_PIN_SET){
		misStatusByte0 |= 0x02;
	} else {
		misStatusByte0 &= ~0x02;
	}
	if (HAL_GPIO_ReadPin(GPIO__12V_1_GPIO_Port, GPIO__12V_1_Pin) == GPIO_PIN_SET){
		misStatusByte0 |= 0x04;
	} else {
		misStatusByte0 &= ~0x04;
	}
	if (HAL_GPIO_ReadPin(GPIO__12V_2_GPIO_Port, GPIO__12V_2_Pin) == GPIO_PIN_SET){
		misStatusByte0 |= 0x08;
	} else {
		misStatusByte0 &= ~0x08;
	}
	if (HAL_GPIO_ReadPin(GPIO__12V_3_GPIO_Port, GPIO__12V_3_Pin) == GPIO_PIN_SET){
		misStatusByte0 |= 0x10;
	} else {
		misStatusByte0 &= ~0x10;
	}
	if (HAL_GPIO_ReadPin(FAN_GPIO_Port, FAN_Pin) == GPIO_PIN_SET){
		misStatusByte0 |= 0x20;
	} else {
		misStatusByte0 &= ~0x20;
	}
	if (HAL_GPIO_ReadPin(FAN_2_GPIO_Port, FAN_2_Pin) == GPIO_PIN_SET){
		misStatusByte0 |= 0x40;
	} else {
		misStatusByte0 &= ~0x40;
	}
	if (HAL_GPIO_ReadPin(GPIO__5V_1_GPIO_Port, GPIO__5V_1_Pin) == GPIO_PIN_SET){
		misStatusByte0 |= 0x80;
	} else {
		misStatusByte0 &= ~0x80;
	}
	if (HAL_GPIO_ReadPin(CAM_ON_GPIO_Port, CAM_ON_Pin) == GPIO_PIN_SET){
		misStatusByte1 |= 0x01;
	} else {
		misStatusByte1 &= ~0x01;
	}
}

void allConsumersEnable(void) {

	HAL_GPIO_WritePin(RASP_KEY_GPIO_Port, RASP_KEY_Pin, SET);
	HAL_GPIO_WritePin(ALT_KEY_GPIO_Port, ALT_KEY_Pin, SET);
	HAL_GPIO_WritePin(GPIO__12V_1_GPIO_Port, GPIO__12V_1_Pin, SET);
	HAL_GPIO_WritePin(GPIO__12V_2_GPIO_Port, GPIO__12V_2_Pin, SET);
	HAL_GPIO_WritePin(GPIO__12V_3_GPIO_Port, GPIO__12V_3_Pin, SET);
	HAL_GPIO_WritePin(FAN_2_GPIO_Port, FAN_2_Pin, SET);
//	HAL_GPIO_WritePin(GPIO__5V_1_GPIO_Port, GPIO__5V_1_Pin, SET);
	HAL_GPIO_WritePin(CAM_ON_GPIO_Port, CAM_ON_Pin, SET);
}

void allConsumersDisable(void) {
	HAL_GPIO_WritePin(GPIO__12V_1_GPIO_Port, GPIO__12V_1_Pin, RESET);
	HAL_GPIO_WritePin(GPIO__12V_2_GPIO_Port, GPIO__12V_2_Pin, RESET);
	HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, RESET);
	HAL_GPIO_WritePin(FAN_2_GPIO_Port, FAN_2_Pin, RESET);
//	HAL_GPIO_WritePin(GPIO__5V_1_GPIO_Port, GPIO__5V_1_Pin, RESET);
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
//	uint8_t i  =0;

	if (huart->Instance == USART2) {
		setRxMode(2);
	}
	if (huart->Instance == USART6) {
		setRxMode(6);
		if (resetFlag) {
			resetFlag = 0;
			NVIC_SystemReset();
		}
	}
//	if (huart->Instance == USART1) {
//		for (i = 0; i < STD_PACK_SIZE; i++) {
//			raspTxBuf[i] = 0;
//		}
//	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == GPIO_PIN_4) {
		osSignalSet(accelHandle, 0x01);
	}
	if (GPIO_Pin == GPIO_PIN_1) {
		osSignalSet (canRxHandle, 0x0A);
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

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1, raspRxBuf, RASP_RX_BUF_SIZE);
	__HAL_UART_ENABLE_IT (&huart6, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart6, currentVoltageRxBuf, CV_RX_BUF_SIZE);

	/* Infinite loop */
	for (;;) {

		event1 = osMessageGet(onOffQueueHandle, osWaitForever);
		if (event1.status == osEventMessage){
			switch (event1.value.v) {

			case ENGINE_START_ID:

				allConsumersEnable();
				break;

			case ENGINE_STOP_ID:

				allConsumersDisable();
				osDelay(20000);
				HAL_GPIO_WritePin(RASP_KEY_GPIO_Port, RASP_KEY_Pin, RESET);
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
	sensorsData xSensors;
	sensorsData *sensors = &xSensors;
	HAL_I2C_Init(&hi2c2);
	ADXL345_Init();

	/* Infinite loop */
	for (;;) {
		evt = osSignalWait(0x01, osWaitForever);
		if (evt.status == osEventSignal) {
			if (osMutexWait(I2C2MutexHandle, 50) == osOK){
				intSource = getInterruptSource();
				ADXL345_Read(buffer);
				osMutexRelease(I2C2MutexHandle);
			}
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
			sensors->source = ADXL_TASK;
			sensors->size = ADXL_SIZE;
			memcpy (sensors->payload, buffer, 6);
			if (xQueueSend(qSensorsHandle, (void*) &sensors, 1) != pdTRUE) {
			}
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
	static uint8_t buffer2[2] = {0};

	int16_t temperature0 = 0;
	int16_t temperature2 = 0;
	static uint8_t fanState = 0;
	osStatus tempMutexStatus0;

	sensorsData xSensors;
	sensorsData *sensors = &xSensors;

	osDelay(500);
	/* Infinite loop */
	for (;;) {

		tempMutexStatus0 = osMutexWait(I2C2MutexHandle, 1);

		if (tempMutexStatus0 == osOK) {
			tempSensorState = TLA2024_Read(0, buffer0);
			TLA2024_Read(2, buffer2);
			osMutexRelease(I2C2MutexHandle);
		}

		temperature0 = (int16_t)buffer0[0] << 8;
		temperature0 |= buffer0[1];
		temperature2 = (int16_t)buffer2[0] << 8;
		temperature2 |= buffer2[1];

		debugTemp0 = temperature0;
		debugTemp2 = temperature2;

		if (temperature0 < 0) {
			temperature0 = -temperature0;
			temperature0 |= 0x8000;
		}

		if (temperature2 < 0) {
			temperature2 = -temperature2;
			temperature2 |= 0x8000;
		}

		buffer0[0] = (uint8_t)((temperature0 & 0xFF00) >> 8);
		buffer0[1] = (uint8_t)(temperature0 & 0x00FF);
		buffer2[0] = (uint8_t)((temperature2 & 0xFF00) >> 8);
		buffer2[1] = (uint8_t)(temperature2 & 0x00FF);

		if ((tempSensorState == TEMP_SENSOR_FAIL) && (cvStatusByteExtern & 0x06)) {

			HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, SET);

		} else {
			switch (fanState) {
			case 0:
				if (temperature0 > TEMPERATURE) {
					HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, SET);
					fanState++;
				}
				break;
			case 1:
				if (temperature0 < (TEMPERATURE-TEMP_HYSTERESE)){
					HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, RESET);
					fanState = 0;
				}
				break;
			}
		}

		sensors->source = TLA2024_TASK_SOURCE;
		sensors->size = TLA2024_SIZE;
		memset(sensors->payload, 0, 16);
		memcpy (sensors->payload, buffer0, 2);
		memcpy (sensors->payload+4, buffer2, 2);
		if (xQueueSend(qSensorsHandle, (void*) &sensors, 5) != pdTRUE) {

		}

		if (sensorsOnFlag) {
			HAL_GPIO_WritePin(SENSORS_PWR_GPIO_Port, SENSORS_PWR_Pin, SET);
		} else {
			HAL_GPIO_WritePin(SENSORS_PWR_GPIO_Port, SENSORS_PWR_Pin, RESET);
		}

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

	BaseType_t xHigherPriorityTaskWoken = (BaseType_t) 0;

	uint8_t request[6] = { PACKET_HEADER, CV_REQ_PACK_ID, CV_REQ_SIZE,
			CMD_CV_REQUEST, 0, PACKET_FOOTER };
	request[4] = get_check_sum(request, 6);

	if (htim->Instance == TIM11) {

		if (tickCounter < 1000) {
			tickCounter++;
		} else {
			tickCounter = 0;
			osSignalSet(uartCommHandle, 0x02);
			HAL_GPIO_TogglePin(GPIO__5V_1_GPIO_Port, GPIO__5V_1_Pin);
		}

/*------------------------------------------------------------------------------------------*/

		if (cvRequestCounter < 500) {
			cvRequestCounter++;

		} else {
			cvRequestCounter = 0;
			sensor->source = CV_REQ_SOURCE;
			sensor->size = CV_REQ_SIZE;
			memcpy((uint8_t*)sensor->payload, request, CV_REQ_SIZE);
			if (xQueueSendFromISR(qSensorsHandle, (void *)&sensor, &xHigherPriorityTaskWoken) != pdTRUE){
			}
			portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

			if (engineState == ENGINE_STOPPED)  {
				if (!breaksStateTelem){
					secondCounter++;
				} else {
					secondCounter = 0;
				}
			} else {
				secondCounter = 0;
			}
		}

/*------------------------------------------------------------------------------------------*/

		if (secondCounter >= 120){
			secondCounter = 0;
			if (engineState == ENGINE_STOPPED)  {
				HAL_GPIO_WritePin(ALT_KEY_GPIO_Port, ALT_KEY_Pin, RESET);
				HAL_GPIO_WritePin(GPIO__12V_3_GPIO_Port, GPIO__12V_3_Pin, RESET);
			}
		}

/*------------------------------------------------------------------------------------------*/

	switch (raspOffState) {

	case 0:
		if ((gpio17State = HAL_GPIO_ReadPin(GPIO17_GPIO_Port, GPIO17_Pin)) == GPIO_PIN_SET){
			if (raspOffCounter < 20){
				raspOffCounter++;
			} else {
				raspOffCounter = 0;
				raspOffState++;
			}
		} else {
			if (timeOutFlag) {
				if ((HAL_GetTick() - timeoutCnt) >= TIMEOUT_GPIO17) {
					timeOutFlag = 0;
					raspRestartFlag = 1;
				}
			}
			raspOffCounter = 0;
		}
		break;

	case 1:
		if ((gpio17State = HAL_GPIO_ReadPin(GPIO17_GPIO_Port, GPIO17_Pin)) == GPIO_PIN_RESET) {

			if (raspOffCounter < 50) {
				raspOffCounter++;
			} else {
				raspOffCounter = 0;
				sensor->source = RASP_UART_SRC;
				sensor->size = CV_REQ_SIZE;
				sensor->payload[0] = 0xAA;
				sensor->payload[1] = RASP_IN_PACK_ID;
				sensor->payload[2] = CV_REQ_SIZE;
				sensor->payload[3] = CMD_BACKLIGHT_OFF;
				sensor->payload[4] = get_check_sum((uint8_t*)(sensor->payload), CV_REQ_SIZE);
				sensor->payload[5] = 0x55;
				if (xQueueSendFromISR(qSensorsHandle, (void *) &sensor, &xHigherPriorityTaskWoken) != pdTRUE) {
				}
				portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
				if (breaksStateTelem) {
					turnOffBreaksFlag = 1;
					raspOffState = 3;
				} else {
					turnOffBreaksFlag = 0;
					raspOffState++;
				}
			}
		} else {
			raspOffCounter = 0;
		}
		break;

	case 2:
		if ((gpio17State = HAL_GPIO_ReadPin(GPIO17_GPIO_Port, GPIO17_Pin)) == GPIO_PIN_RESET) {
			stateChangeCounter = 0;
			if (raspOffCounter < 119950) {
				if (!breaksStateTelem){
					raspOffCounter++;
				} else {
					raspOffCounter = 0;
				}
			} else {
				raspOffCounter = 0;
				sensor->source = RASP_UART_SRC;
				sensor->size = CV_REQ_SIZE;
				sensor->payload[0] = 0xAA;
				sensor->payload[1] = RASP_IN_PACK_ID;
				sensor->payload[2] = CV_REQ_SIZE;
				sensor->payload[3] = CMD_PWR_OFF;
				sensor->payload[4] = get_check_sum((uint8_t*)sensor->payload, CV_REQ_SIZE);
				sensor->payload[5] = 0x55;
				allConsumersDisable();
				if (xQueueSendFromISR(qSensorsHandle, (void *)&sensor, &xHigherPriorityTaskWoken) != pdTRUE) {
				}
				portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
				HAL_GPIO_WritePin(RASP_KEY_GPIO_Port, RASP_KEY_Pin, RESET);
				raspOffState = 0;
			}
		} else {
			raspOffCounter = 0;
			if (stateChangeCounter < 300) {
				stateChangeCounter++;
			} else {
				stateChangeCounter = 0;
				raspOffState = 0;
			}
		}
		break;

	case 3:

		if ((gpio17State = HAL_GPIO_ReadPin(GPIO17_GPIO_Port, GPIO17_Pin)) == GPIO_PIN_RESET) {
			if (raspOffCounter < 119950) {
					raspOffCounter++;
			} else {
				raspOffCounter = 0;
				sensor->source = RASP_UART_SRC;
				sensor->size = CV_REQ_SIZE;
				sensor->payload[0] = 0xAA;
				sensor->payload[1] = RASP_IN_PACK_ID;
				sensor->payload[2] = CV_REQ_SIZE;
				sensor->payload[3] = CMD_PWR_OFF;
				sensor->payload[4] = get_check_sum((uint8_t*)sensor->payload, CV_REQ_SIZE);
				sensor->payload[5] = 0x55;
				allConsumersDisable();
				if (xQueueSendFromISR(qSensorsHandle, (void *)&sensor, &xHigherPriorityTaskWoken) != pdTRUE) {
				}
				portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
				HAL_GPIO_WritePin(RASP_KEY_GPIO_Port, RASP_KEY_Pin, RESET);
				raspOffState = 0;
			}
		} else {
			raspOffCounter = 0;
		}
		break;
	}

/*------------------------------------------------------------------------------------------*/
	
	  switch (gerconState){
	  case 0:
		  if (!HAL_GPIO_ReadPin(GERCON_GPIO_Port, GERCON_Pin)){
			  if (gerconCounter < 50){
 				  gerconCounter++;
			  } else {
				  gerconCounter = 0;
				  gerconState++;
			  }
		  } else {
			  gerconCounter = 0;
		  }
		  break;
	  case 1:
		  if (HAL_GPIO_ReadPin(GERCON_GPIO_Port, GERCON_Pin)){
			  if (gerconCounter < 500){
				  gerconCounter++;
			  } else {
				  gerconCounter = 0;
				  gerconState = 0;
			  }
		  } else {
			  gerconCounter = 0;
		  }
		  break;
	  }

/*------------------------------------------------------------------------------------------*/

	  if (HAL_GPIO_ReadPin(WKUP_GPIO_Port, WKUP_Pin) == GPIO_PIN_SET) {
		  if (wakeUpPinCounter < 100) {
			  wakeUpPinCounter++;
		  } else {
			  wakeUpPinCounter = 0;
			  wakeUpFlag = 1;
		  }
	  } else {
		  wakeUpPinCounter = 0;
	  }

	}
	
/*------------------------------------------------------------------------------------------*/


	if (htim->Instance == TIM13) { //Таймаут команды управления SourceSelector
		HAL_TIM_Base_Stop_IT(&htim13);
		__HAL_TIM_CLEAR_IT(&htim13, TIM_IT_UPDATE);
		sensor1->source = CV_RESP_SOURCE;
		sensor1->size = RASP_RESP_SIZE;
		sensor1->payload[6] = get_check_sum((uint8_t*)sensor1->payload, 8);
		memcpy((uint8_t*)sensor1->payload, cvTimeoutResponse, 8);
		if (xQueueSendFromISR(qSensorsHandle, (void *)&sensor1, &xHigherPriorityTaskWoken) != pdTRUE) {
		}
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}

	/*------------------------------------------------------------------------------------------*/

	if (htim->Instance == TIM8) { //Сюда мы попадаем, если 200ms нет ответа от SourceSelector
		HAL_TIM_Base_Stop_IT(&htim8);
		__HAL_TIM_CLEAR_IT(&htim8, TIM_IT_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim8, 0);
		__HAL_UART_DISABLE_IT(&huart6, UART_IT_IDLE);
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);
		HAL_UART_AbortTransmit(&huart6);
		HAL_UART_DMAStop(&huart6);
		HAL_UART_AbortReceive(&huart6);
		HAL_UART_DeInit(&huart6);
		MX_USART6_UART_Init();
		__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart6, currentVoltageRxBuf, CV_RX_BUF_SIZE);
		for (cvCounter = 0; cvCounter < CV_RX_BUF_SIZE; cvCounter++) {
			currentVoltageRxBuf[cvCounter] = 0;
		}
		queueStatusByte1 |= 0x80;
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
