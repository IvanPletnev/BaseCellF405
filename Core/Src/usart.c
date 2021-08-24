/*
 * usart.c
 *
 *  Created on: Jun 25, 2021
 *      Author: ivan
 */

#include "usart.h"

#define ENGINE_START_LEVEL   	1300
#define ENGINE_STOP_LEVEL		1280

extern osMailQId qSensorsHandle;
extern osThreadId defaultTaskHandle;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern uint8_t currentVoltageRxBuf[CV_RX_BUF_SIZE];
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint8_t raspRxBuf[RASP_RX_BUF_SIZE];
extern uint8_t gerconState;
extern osMessageQId onOffQueueHandle;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim13;
extern uint32_t osTickCounter;
extern uint32_t osTickCounterOld;

uint8_t raspTxBuf[STD_PACK_SIZE];

uint8_t dimmingTime = 0x32;
uint8_t engineSwitchFlag = 0;

usartErrT usartError = 0;

uint8_t autoBacklightflags[4] = { 1,1,1,1 };
uint8_t brightnessValues[4] = { 10,10,10,10 };
uint8_t engineState = 0;

uint8_t chksum = 0;

uint16_t onBoardVoltage = 0;
uint8_t breaksState = 0;


void USER_UART_IDLECallback(UART_HandleTypeDef *huart) {

//	static uint8_t flag;
	sensorsData *sensors;
	uint8_t state = 0;

	HAL_UART_DMAStop(huart);

	if (huart->Instance == USART6) {
		sensors = osMailAlloc(qSensorsHandle, 0);
		sensors->size = CV_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);

		switch (state) {
		case 0:
			if (currentVoltageRxBuf[0] == 0xAA){
				state++;
			} else {
				return;
			}
		case 1:
			switch (currentVoltageRxBuf[1]) {
			case CV_PACK_ID:
				sensors->source = CV_USART_SRC;
				memcpy(sensors->payload, currentVoltageRxBuf, CV_RX_BUF_SIZE);
				break;
			case RASP_RESP_PACK_ID:
				sensors->source = CV_RESP_SOURCE;
				if (currentVoltageRxBuf[3] == CMD_PWR_OFF){
					osMessagePut(onOffQueueHandle, ENGINE_STOP_ID, 0);
				}
				memcpy(sensors->payload, currentVoltageRxBuf, CV_RESP_SIZE);
				HAL_TIM_Base_Stop_IT(&htim13);
				__HAL_TIM_CLEAR_IT(&htim13, TIM_IT_UPDATE);
				__HAL_TIM_SET_COUNTER(&htim13, 0);
				break;
			default:
				break;
			}
			break;
		}

		osMailPut(qSensorsHandle, sensors);
		HAL_UART_Receive_DMA(&huart6, currentVoltageRxBuf, CV_RX_BUF_SIZE);

	} else if (huart->Instance == USART1) {
		sensors = (sensorsData *) osMailAlloc(qSensorsHandle, 0);
		sensors->size = RASP_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
		sensors->source = RASP_UART_SRC;
		memcpy (sensors->payload, raspRxBuf, sensors->size);
		osMailPut(qSensorsHandle, sensors);
		HAL_UART_Receive_DMA(&huart1, raspRxBuf, RASP_RX_BUF_SIZE);
//		if (get_check_sum(raspRxBuf, size) != raspRxBuf[size - 3]) {
//			sendRespToRasp(0, RESPONSE_ERROR);
//			return;
//		}
	}
}

void USER_UART_IRQHandler(UART_HandleTypeDef *huart) {
	if ( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		USER_UART_IDLECallback(huart);
	}
}


void sendRespToRasp(uint8_t cmd, uint8_t response) {
	uint8_t responseBuf[8] = { 0 };
	responseBuf[0] = 0xAA;
	responseBuf[1] = RASP_RESP_PACK_ID;
	responseBuf[2] = 8;
	responseBuf[3] = cmd;
	responseBuf[4] = response;
	responseBuf[5] = 0;
	responseBuf[6] = get_check_sum(responseBuf, 8);
	responseBuf[7] = 0x55;
	HAL_UART_Transmit_DMA(&huart1, responseBuf, 8);
}


usartErrT cmdHandler (uint8_t *source, uint8_t size) {

	uint8_t destTempBuf[16] = {0};
	uint8_t state = 0;
	uint8_t i;

	switch (state) {

	case 0:

		if (source[0] == 0xAA) {
			state++;
			destTempBuf[0] = 0xAA;
		} else {
			return WRONG_HEADER;
		}
	case 1:

		if (source[1] == RASP_IN_PACK_ID) {
			state++;
			destTempBuf[1] = BL_OUT_PACK_ID;
			destTempBuf[2] = size = source[2];
			destTempBuf[size-1] = 0x55;
		} else {
			return WRONG_PACK_ID;
		}
	case 2:

		switch (source[3]) {
		case CMD_START_IMG:
			for (i = 0; i < 4; i++) {
				if (source[i + 6]) {
					if (source[4]) {
						autoBacklightflags[i] = source[4];
					} else {
						autoBacklightflags[i] = BL_MODE_MAN;
					}
				} else {
					autoBacklightflags[i] = BL_MODE_MAN;
				}
				brightnessValues[i] = source[i + 6];
			}
			dimmingTime = source[5];
			memcpy(destTempBuf + 3, source + 3, 7);
			destTempBuf[10] = get_check_sum(destTempBuf, size);
			setTxMode(2);
			HAL_UART_Transmit_DMA(&huart2, destTempBuf, size);
			sendRespToRasp(CMD_START_IMG, RESPONSE_OK);
			state = 0;
			break;

		case CMD_STOP_IMG:

			memcpy(destTempBuf + 3, source + 3, 2);
			setTxMode(2);
			HAL_UART_Transmit_DMA(&huart2, destTempBuf, size);
			sendRespToRasp(CMD_STOP_IMG, RESPONSE_OK);
			state = 0;
			break;

		case CMD_BACKLIGHT_MODE:

			if (source[4] == BROADCAST_ADDR) {
				for (i = 0; i < 4; i++) {
					autoBacklightflags[i] = BL_MODE_AUTO;
				}
			} else if (source[5] == 1) {
				autoBacklightflags[source[4]] = BL_MODE_AUTO;
			} else if (source[5] == 0) {
				autoBacklightflags[source[4]] = BL_MODE_MAN;
			}
			memcpy(destTempBuf + 3, source + 3, 3);
			destTempBuf[6] = get_check_sum(destTempBuf, size);
			setTxMode(2);
			HAL_UART_Transmit_DMA(&huart2, destTempBuf, size);
			sendRespToRasp(CMD_BACKLIGHT_MODE, RESPONSE_OK);
			state = 0;
			break;

		case CMD_SET_BRIGHTNESS: // Установка яркости подсветки для конкретного адреса, либо для всех драйверов, если адрес 0xFF

			if (source[5] == BROADCAST_ADDR) { //если адрес широковещательный
				for (i = 0; i < 4; i++) {
					brightnessValues[i] = source[5]; //сохраняем значения яркости всех ячеек подсветки
					autoBacklightflags[i] = BL_MODE_MAN; // выключаем авторегулирование подсветки
				}
			} else if (source[5]) { //если значение яркости ненулевое
				brightnessValues[source[4]] = source[5]; //сохраняем значение яркости для указанного адреса
				autoBacklightflags[source[4]] = BL_MODE_MAN; //и выключаем авторегулирование яркости
			} else if (source[5] == 0) { //если яркость нулевая
				brightnessValues[source[4]] = 0;
				autoBacklightflags[source[4]] = BL_MODE_MAN; //выключаем авторегулирование
			}
			memcpy(destTempBuf + 3, source + 3, 3);
			destTempBuf[6] = get_check_sum(destTempBuf, size);
			setTxMode(2);
			HAL_UART_Transmit_DMA(&huart2, destTempBuf, size);
			sendRespToRasp(CMD_SET_BRIGHTNESS, RESPONSE_OK);
			state = 0;
			break;

		case CMD_BL_MODE_BY_ADDR:

			memcpy(autoBacklightflags, source + 4, 4);
			memcpy(destTempBuf + 3, source + 3, 5);
			destTempBuf[8] = get_check_sum(destTempBuf, size);
			setTxMode(2);
			HAL_UART_Transmit_DMA(&huart2, destTempBuf, size);
			sendRespToRasp(CMD_BL_MODE_BY_ADDR, RESPONSE_OK);
			state = 0;
			break;

		case CMD_SET_BR_BY_ADDR:

			memcpy(brightnessValues, source + 4, 4);
			memcpy(destTempBuf + 3, source + 3, 5);
			destTempBuf[8] = get_check_sum(destTempBuf, size);
			setTxMode(2);
			HAL_UART_Transmit_DMA(&huart2, destTempBuf, size);
			sendRespToRasp(CMD_SET_BR_BY_ADDR, RESPONSE_OK);
			state = 0;
			break;

		case CMD_PWR_OFF:
			destTempBuf[1] = CV_REQ_PACK_ID;
			destTempBuf[2] = CV_REQ_SIZE;
			destTempBuf[3] = CMD_PWR_OFF;
			destTempBuf[4] = get_check_sum(destTempBuf, CV_REQ_SIZE);
			setTxMode(6);
			HAL_UART_Transmit_DMA(&huart6, destTempBuf, CV_REQ_SIZE);

			__HAL_TIM_CLEAR_IT(&htim13, TIM_IT_UPDATE);
			__HAL_TIM_SET_COUNTER(&htim13, 0);
			HAL_TIM_Base_Start_IT(&htim13);
			break;

		case CMD_CHARGER_OFF:
			destTempBuf[1] = CV_REQ_PACK_ID;
			destTempBuf[2] = CV_REQ_SIZE;
			destTempBuf[3] = CMD_CHARGER_OFF;
			destTempBuf[4] = get_check_sum(destTempBuf, CV_REQ_SIZE);
			setTxMode(6);
			HAL_UART_Transmit_DMA(&huart6, destTempBuf, CV_REQ_SIZE);
			__HAL_TIM_CLEAR_IT(&htim13, TIM_IT_UPDATE);
			__HAL_TIM_SET_COUNTER(&htim13, 0);
			HAL_TIM_Base_Start_IT(&htim13);
			break;

		case CMD_BACKLIGHT_OFF:
			destTempBuf[1] = CV_REQ_PACK_ID;
			destTempBuf[2] = CV_REQ_SIZE;
			destTempBuf[3] = CMD_BACKLIGHT_OFF;
			destTempBuf[4] = get_check_sum(destTempBuf, CV_REQ_SIZE);
			setTxMode(6);
			HAL_UART_Transmit_DMA(&huart6, destTempBuf, CV_REQ_SIZE);
			__HAL_TIM_CLEAR_IT(&htim13, TIM_IT_UPDATE);
			__HAL_TIM_SET_COUNTER(&htim13, 0);
			HAL_TIM_Base_Start_IT(&htim13);
			break;

		case CMD_ZERO_OFF:
			HAL_GPIO_WritePin(ALT_KEY_GPIO_Port, ALT_KEY_Pin, RESET);
			sendRespToRasp(CMD_ZERO_OFF, RESPONSE_OK);
			break;

		case CMD_CHARGER_ON:

			destTempBuf[1] = CV_REQ_PACK_ID;
			destTempBuf[2] = CV_REQ_SIZE;
			destTempBuf[3] = CMD_CHARGER_ON;
			destTempBuf[4] = get_check_sum(destTempBuf, CV_REQ_SIZE);
			setTxMode(6);
			HAL_UART_Transmit_DMA(&huart6, destTempBuf, CV_REQ_SIZE);
			__HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
			__HAL_TIM_SET_COUNTER(&htim7, 0);
			HAL_TIM_Base_Start_IT(&htim7);
			break;

		case CMD_BACKLIGHT_ON:

			destTempBuf[1] = CV_REQ_PACK_ID;
			destTempBuf[2] = CV_REQ_SIZE;
			destTempBuf[3] = CMD_BACKLIGHT_ON;
			destTempBuf[4] = get_check_sum(destTempBuf, CV_REQ_SIZE);
			setTxMode(6);
			HAL_UART_Transmit_DMA(&huart6, destTempBuf, CV_REQ_SIZE);
			__HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
			__HAL_TIM_SET_COUNTER(&htim7, 0);
			HAL_TIM_Base_Start_IT(&htim7);
			break;

		case CMD_ZERO_ON:

			HAL_GPIO_WritePin(ALT_KEY_GPIO_Port, ALT_KEY_Pin, SET);
			sendRespToRasp(CMD_ZERO_OFF, RESPONSE_OK);
			break;

		case CMD_SET_DIMMING_TIME:
		case CMD_FX:

			memcpy(destTempBuf + 3, source + 3, 2);
			destTempBuf[5] = get_check_sum(destTempBuf, size);
			setTxMode(2);
			HAL_UART_Transmit_DMA(&huart2, destTempBuf, size);
			sendRespToRasp(source[3], RESPONSE_OK);
			state = 0;
			break;
		}
	break;
	}
	memset (destTempBuf, 0,16);
	return ERR_OK;
}

void uartCommTask(void const *argument) {
	/* USER CODE BEGIN uartCommTask */
	sensorsData *sensors;
	osEvent event, evt;
//	uint8_t destTempBuf[6] = {0};

	osDelay(200);

	/* Infinite loop */
	for (;;) {
		event = osMailGet(qSensorsHandle, 1);

		if (event.status == osEventMail) {
			sensors = event.value.p;
			raspTxBuf[0] = 0xAA;

			if (sensors->source == ADXL_TASK) { //пакет от акселерометра, сразу отправляем в Raspberry, не дожидаясь таймера
				raspTxBuf[1] = ADXL_PACK_ID;
				raspTxBuf[2] = ADXL_PACK_SIZE;
				memcpy(raspTxBuf + PACK_HEADER_SIZE, sensors->payload,
						ADXL_SIZE);
				raspTxBuf[ADXL_PACK_SIZE] = 0x55;
				HAL_UART_Transmit_DMA(&huart1, raspTxBuf, ADXL_PACK_SIZE); // -> В Raspberry

			} else if (sensors->source == RASP_UART_SRC) { //пакет от RaspberryPi - управление подсветкой и платой управления питанием
				cmdHandler(sensors->payload, sensors->size); //также пакет, имитирующий CMD_PWR_OFF по таймауту выключения Raspberry

			} else if (sensors->source == BL_AUTO_CONTROL_SRC) { //пакет авторегулирования подсветки
				setTxMode(2);
				HAL_UART_Transmit_DMA(&huart2, sensors->payload, sensors->size);

			} else if (sensors->source == CV_REQ_SOURCE) { //запрос к SourceSelector на телеметрию UART6
				setTxMode(6);
				HAL_UART_Transmit_DMA(&huart6, sensors->payload, sensors->size);

			} else if (sensors->source == CV_RESP_SOURCE) { //ответы от SourceSelector на команды управления питанием UART6

//				if (sensors->payload[4] == 1){ //Если сработал таймаут, то есть ответ от SorceSelector не получен,
//					destTempBuf[0] = 0xAA; destTempBuf[1] = CV_REQ_PACK_ID; destTempBuf[2] = CV_REQ_SIZE;
//					destTempBuf[3] = sensors->payload[3]; destTempBuf[4] = get_check_sum(destTempBuf, CV_REQ_SIZE); destTempBuf[5] = 0x55;
//					setTxMode(6);
//					HAL_UART_Transmit_DMA(&huart6, destTempBuf, CV_REQ_SIZE); //Отправляем повторную команду CMD_PWR_OFF в SourceSelector
//					__HAL_TIM_CLEAR_IT(&htim13, TIM_IT_UPDATE);
//					__HAL_TIM_SET_COUNTER(&htim13, 0);
//					HAL_TIM_Base_Start_IT(&htim13);
//				}
				HAL_UART_Transmit_DMA(&huart1, sensors->payload, sensors->size);//транслируем в Raspberry

			} else{

				switch (sensors->source) {

				case APDS_TASK_SOURCE: //пакет от датчика освещенности
					memcpy(raspTxBuf + APDS_OFFSET, sensors->payload,
							APDS_SIZE);
					break;

				case TLA2024_TASK_SOURCE: // пакет от датчиков температуры
					memcpy(raspTxBuf + TEMP_OFFSET, sensors->payload,
							TLA2024_SIZE);
					break;

				case DHT22_TASK_SOURCE: // пакет от датчиков температуры и влажности
					memcpy(raspTxBuf + DHT22_OFFSET, sensors->payload,
							DHT22_SIZE);
					break;

				case CV_USART_SRC: //в порт пришел пакет от ячейки управления питанием

					chksum = get_check_sum(sensors->payload, CV_RX_BUF_SIZE);
					onBoardVoltage = (uint16_t)(sensors->payload[3] << 8);
					onBoardVoltage |= (uint16_t)sensors->payload[4];

					if ((onBoardVoltage > ENGINE_START_LEVEL) && ( engineState == ENGINE_STOPPED)) {
						engineState = ENGINE_STARTED;
						HAL_GPIO_WritePin(ALT_KEY_GPIO_Port, ALT_KEY_Pin, SET);
						HAL_GPIO_WritePin(RASP_KEY_GPIO_Port, RASP_KEY_Pin, SET);
						HAL_GPIO_WritePin(GPIO__12V_3_GPIO_Port, GPIO__12V_3_Pin, SET);
						HAL_GPIO_WritePin(CAM_ON_GPIO_Port, CAM_ON_Pin, SET);
						osMessagePut(onOffQueueHandle, ENGINE_START_ID, 0);

					} else if (onBoardVoltage < ENGINE_STOP_LEVEL)
					{
						engineState = ENGINE_STOPPED;
					}

					breaksState = sensors->payload[19];
					if (breaksState) {
						breaksState = 0;
						HAL_GPIO_WritePin(ALT_KEY_GPIO_Port, ALT_KEY_Pin, SET);
						HAL_GPIO_WritePin(RASP_KEY_GPIO_Port, RASP_KEY_Pin, SET);
						HAL_GPIO_WritePin(GPIO__12V_3_GPIO_Port, GPIO__12V_3_Pin, SET);
						HAL_GPIO_WritePin(CAM_ON_GPIO_Port, CAM_ON_Pin, SET);
						if (engineState == ENGINE_STOPPED){
							osMessagePut(onOffQueueHandle, ENGINE_START_ID, 0);
						}
					}

					memcpy(raspTxBuf + CV_OFFSET, sensors->payload+PACK_HEADER_SIZE, CV_SIZE);
					break;
				}
			}
			raspTxBuf[1] = STD_PACK_ID;
			raspTxBuf[2] = STD_PACK_SIZE;
			raspTxBuf[GERCON_OFFSET] = gerconState;
			raspTxBuf[ENGINE_STATE_OFFSET] = engineState;
			raspTxBuf[STD_PACK_SIZE-2] = get_check_sum(raspTxBuf, STD_PACK_SIZE);
			raspTxBuf[STD_PACK_SIZE-1] = 0x55;
			osMailFree(qSensorsHandle, sensors); //освобождаем память под очередью
		}
		evt = osSignalWait(0x02, 1); //отправляем по прерыванию от таймера
		if (evt.status == osEventSignal) {

			HAL_UART_Transmit_DMA(&huart1, raspTxBuf, STD_PACK_SIZE);
		}
	}
	/* USER CODE END uartCommTask */
}

uint8_t get_check_sum(uint8_t *source, uint8_t size) {
	uint8_t i, uSum = 0;
	for (i = 0; i < size - 3; i++) {
		uSum = uSum + source[i];
	}
	uSum = (~uSum) + 1;
	return uSum;
}
