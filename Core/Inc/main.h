/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct _sensorsData {
	uint8_t source;
	uint8_t size;
	uint8_t payload[64];
}sensorsData;

typedef enum _powerStateType {
	DISABLED,
	ENABLED,
	RASPBERRY_WAIT
}powerStateType;

typedef struct os_mailQ_cb {
  const osMailQDef_t *queue_def;
  QueueHandle_t handle;
  osPoolId pool;
} os_mailQ_cb_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void setTxMode (uint8_t uartNo);
void allConsumersEnable (void);
void allConsumersDisable(void);
void setStatusBytes (void);

extern __IO int32_t osMailAllocCounter;
extern __IO int32_t osMailFreeCounter;
extern __IO uint32_t queueErrorCnt;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SENSORS_PWR_Pin GPIO_PIN_13
#define SENSORS_PWR_GPIO_Port GPIOC
#define GPIO__12V_1_Pin GPIO_PIN_0
#define GPIO__12V_1_GPIO_Port GPIOC
#define MCPINT_Pin GPIO_PIN_1
#define MCPINT_GPIO_Port GPIOC
#define MCPINT_EXTI_IRQn EXTI1_IRQn
#define WKUP_Pin GPIO_PIN_0
#define WKUP_GPIO_Port GPIOA
#define GPIO__12V_2_Pin GPIO_PIN_1
#define GPIO__12V_2_GPIO_Port GPIOA
#define TXRX2_Pin GPIO_PIN_4
#define TXRX2_GPIO_Port GPIOA
#define CAM_ON_Pin GPIO_PIN_7
#define CAM_ON_GPIO_Port GPIOA
#define ADXL2_INT_Pin GPIO_PIN_4
#define ADXL2_INT_GPIO_Port GPIOC
#define ADXL2_INT_EXTI_IRQn EXTI4_IRQn
#define GPIO__5V_1_Pin GPIO_PIN_5
#define GPIO__5V_1_GPIO_Port GPIOC
#define FAN_2_Pin GPIO_PIN_0
#define FAN_2_GPIO_Port GPIOB
#define FAN_Pin GPIO_PIN_1
#define FAN_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_12
#define CS2_GPIO_Port GPIOB
#define GPIO__12V_3_Pin GPIO_PIN_14
#define GPIO__12V_3_GPIO_Port GPIOB
#define RASP_KEY_Pin GPIO_PIN_15
#define RASP_KEY_GPIO_Port GPIOB
#define TXRX6_Pin GPIO_PIN_8
#define TXRX6_GPIO_Port GPIOC
#define GPIO3_RASP_Pin GPIO_PIN_11
#define GPIO3_RASP_GPIO_Port GPIOA
#define GPIO17_Pin GPIO_PIN_12
#define GPIO17_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_15
#define CS1_GPIO_Port GPIOA
#define DHT22_3_Pin GPIO_PIN_10
#define DHT22_3_GPIO_Port GPIOC
#define DHT22_2_Pin GPIO_PIN_11
#define DHT22_2_GPIO_Port GPIOC
#define DHT22_1_Pin GPIO_PIN_3
#define DHT22_1_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOB
#define GERCON_Pin GPIO_PIN_5
#define GERCON_GPIO_Port GPIOB
#define ALT_KEY_Pin GPIO_PIN_9
#define ALT_KEY_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define ENGINE_STOPPED		0
#define ENGINE_STARTED		1
#define ENGINE_START_ID		0x0001
#define ENGINE_STOP_ID		0x0002
#define RASP_17_INT_ID		0x0003
#define RASP_TIMEOUT_ID		0x0004
#define WATCHDOG_ID			0x0005

#define MAIL_SIZE			24
#define APDS_TASK_SOURCE	0
#define ADXL_TASK			1
#define DHT22_TASK_SOURCE	2
#define	TLA2024_TASK_SOURCE	3
#define	CV_USART_SRC		4
#define	RASP_UART_SRC		5
#define	BL_AUTO_CONTROL_SRC	6
#define	CV_REQ_SOURCE		7
#define CV_RESP_SOURCE		8
#define CV_RESP_SIZE		8
#define RESERVE_SIZE		3
#define CV_STATUS_SIZE		2
#define MIS_STATUS_SIZE		2
#define BL_AUTO_CTL_SIZE	12
#define	APDS_SIZE			16
#define	ADXL_SIZE			6
#define DHT22_SIZE			12
#define	TLA2024_SIZE		6
#define	CV_SIZE				16
#define GERCON_SIZE			1
#define ENGINE_STATE_SIZE	1
#define DISCRETE_IN_SIZE	2
#define CV_FIRMWARE_SIZE	2
#define MIS_FIRMWARE_SIZE	2
#define PACKET_HEADER		0xAA
#define PACKET_FOOTER		0x55
#define PACK_HEADER_SIZE	3 //0xAA + PACK_ID + SIZE
#define PACK_FOOTER_SIZE	2 //checksum + 0x55
#define STD_PACK_ID			1
#define	ADXL_PACK_ID		2
#define RASP_IN_PACK_ID		3
#define BL_OUT_PACK_ID		4
#define RASP_RESP_PACK_ID	0x0F
#define RASP_RESP_SIZE		8
#define CV_PACK_ID			5
#define CV_REQ_PACK_ID		7
#define CV_RESP_PACK_ID		8
#define CV_REQ_SIZE			6
#define	APDS_OFFSET			3
#define TEMP_OFFSET			APDS_SIZE+PACK_HEADER_SIZE //19
#define DHT22_OFFSET		APDS_SIZE+TLA2024_SIZE+PACK_HEADER_SIZE //25
#define CV_OFFSET			APDS_SIZE+TLA2024_SIZE+DHT22_SIZE+PACK_HEADER_SIZE//37
#define GERCON_OFFSET		APDS_SIZE+TLA2024_SIZE+DHT22_SIZE+CV_SIZE+PACK_HEADER_SIZE
#define ENGINE_STATE_OFFSET	APDS_SIZE+TLA2024_SIZE+DHT22_SIZE+CV_SIZE+GERCON_SIZE+PACK_HEADER_SIZE
#define CV_STATUS_OFFSET	APDS_SIZE+TLA2024_SIZE+DHT22_SIZE+CV_SIZE+GERCON_SIZE+ENGINE_STATE_SIZE+PACK_HEADER_SIZE
#define MIS_STATUS_OFFSET	APDS_SIZE+TLA2024_SIZE+DHT22_SIZE+CV_SIZE+GERCON_SIZE+ENGINE_STATE_SIZE+CV_STATUS_SIZE+PACK_HEADER_SIZE
#define DISCR_INPUT_OFFSET	APDS_SIZE+TLA2024_SIZE+DHT22_SIZE+CV_SIZE+GERCON_SIZE+ENGINE_STATE_SIZE+CV_STATUS_SIZE+MIS_STATUS_SIZE+PACK_HEADER_SIZE
#define CV_FIRMWARE_OFFSET	APDS_SIZE+TLA2024_SIZE+DHT22_SIZE+CV_SIZE+GERCON_SIZE+ENGINE_STATE_SIZE+CV_STATUS_SIZE+MIS_STATUS_SIZE+DISCRETE_IN_SIZE+PACK_HEADER_SIZE
#define MIS_FIRMWARE_OFFSET	APDS_SIZE+TLA2024_SIZE+DHT22_SIZE+CV_SIZE+GERCON_SIZE+ENGINE_STATE_SIZE+CV_STATUS_SIZE+MIS_STATUS_SIZE+DISCRETE_IN_SIZE+CV_FIRMWARE_SIZE+PACK_HEADER_SIZE
#define STD_PACK_SIZE		APDS_SIZE+TLA2024_SIZE+DHT22_SIZE+CV_SIZE+GERCON_SIZE+ENGINE_STATE_SIZE+CV_STATUS_SIZE+MIS_STATUS_SIZE+DISCRETE_IN_SIZE+CV_FIRMWARE_SIZE+MIS_FIRMWARE_SIZE+RESERVE_SIZE+PACK_HEADER_SIZE+PACK_FOOTER_SIZE //70
#define ADXL_PACK_SIZE		ADXL_SIZE+PACK_HEADER_SIZE+PACK_FOOTER_SIZE //10
#define CV_RX_BUF_SIZE		30
#define RASP_RX_BUF_SIZE	64

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
