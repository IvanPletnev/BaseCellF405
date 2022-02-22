/*
 * usart.c
 *
 *  Created on: Jun 25, 2021
 *      Author: ivan
 */

#include "usart.h"

#define ENGINE_START_LEVEL   	1300
#define ENGINE_STOP_LEVEL		1280
#define ONBOARD_CRITICAL_LEVEL	1230

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
extern uint8_t wakeUpFlag;
extern uint8_t lightMeterStatusByte;
extern int16_t rawCurrent[3];
uint8_t raspTxBuf[STD_PACK_SIZE];


uint8_t dimmingTime = 0x32;

usartErrT usartError = 0;

uint8_t autoBacklightflags[4] = { 1,1,1,1 };
uint8_t brightnessValues[4] = { 2,2,2,2 };
uint8_t autoBacklightflagsBackUp[4] = {0};
uint8_t brightnessValuesBackUp[4] = {0};
uint8_t backLightOffFlag = 0;

uint8_t engineState = 0;
uint8_t onboardAlarmFlag = 0;

uint8_t chksum = 0;

uint16_t onBoardVoltage = 0;
uint16_t bat1Voltage = 0;
uint16_t bat2Voltage = 0;
uint8_t breaksState = 0;
uint8_t breaksStateTelem = 0;
uint8_t discreteInputState = 0;

uint8_t cvStatusByte = 0;
uint8_t cvStatusByte1 = 0;

uint8_t misStatusByte0 = 0;
uint8_t misStatusByte1 = 0;

uint8_t cvFirmwareVersion0 = 0;
uint8_t cvFirmwareVersion1 = 0;

uint8_t misFirmwareVersion0 = 7;
uint8_t misFirmwareVersion1 = 15;

extern uint8_t raspOffState;
extern osMailQId qEepromHandle;
extern uint8_t wakeUpState;


void USER_UART_IDLECallback(UART_HandleTypeDef *huart) {
	sensorsData *sensors;
	uint8_t state = 0;

	HAL_UART_DMAStop(huart);
	HAL_UART_AbortReceive(huart);

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
						autoBacklightflags[i] = source[4]; //Если в команде включен авторежим, сохраняем флаги для тех драйверов, y которых значения яркости ненулевые
					} else {
						autoBacklightflags[i] = BL_MODE_MAN; //Иначе ставим autoBackLightFlag в ноль
					}
				} else {
					autoBacklightflags[i] = BL_MODE_MAN; // Если значения яркости нулевые, обнуляем autoBacklightFlag
				}
				brightnessValues[i] = source[i + 6]; //и сохраняем последнюю яркость для каждого драйвера
			}
			dimmingTime = source[5]; //запоминаем время диммирования
			memcpy(destTempBuf + 3, source + 3, 7); //копируем все это в отправляемый буфер
			destTempBuf[10] = get_check_sum(destTempBuf, size);//считаем CRC
			setTxMode(2); //в передачу
			HAL_UART_Transmit_DMA(&huart2, destTempBuf, size); //отправляем в драйвер
			sendRespToRasp(CMD_START_IMG, RESPONSE_OK);
			state = 0;
			break;

		case CMD_STOP_IMG: //Здесь мы подменяем команду CMD_STOP_IMG командой CMD_START_IMG с нулями

			destTempBuf[2] = 12;
			destTempBuf[3] = 0x0F;
			destTempBuf[4] = BL_MODE_MAN;
			destTempBuf[5] = source[4]; //берем значение dimmingTime из команды
			destTempBuf[10] = get_check_sum(destTempBuf, 12);
			destTempBuf[11] = 0x55;

			for (i = 0; i < 4; i++) {
				autoBacklightflags[i] = BL_MODE_MAN; //выключение авторежима для всех драйверов
				brightnessValues[i] = 0; //Нулевое значение яркости для всех драйверов
				destTempBuf[i+6] = 0; // Заполняем нулями значения яркости пакета, отправляемого в драйвер
			}

			setTxMode(2); // В передачу
			HAL_UART_Transmit_DMA(&huart2, destTempBuf, 12);
			sendRespToRasp(CMD_STOP_IMG, RESPONSE_OK); //Отправляем в Raspberry ответ, что всё хорошо
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
			osMessagePut(onOffQueueHandle, ENGINE_STOP_ID, 0);
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
			HAL_GPIO_WritePin(GPIO__12V_1_GPIO_Port, GPIO__12V_1_Pin, RESET);
			cvStatusByte &= ~0x04;
			setTxMode(6);
			HAL_UART_Transmit_DMA(&huart6, destTempBuf, CV_REQ_SIZE);

			destTempBuf[1] = BL_OUT_PACK_ID;
			destTempBuf[2] = 12;
			destTempBuf[3] = 0x0F;
			destTempBuf[4] = BL_MODE_MAN;
			destTempBuf[5] = 0x32; //берем значение dimmingTime из команды
			destTempBuf[10] = get_check_sum(destTempBuf, 12);
			destTempBuf[11] = 0x55;
			memcpy((uint8_t*)brightnessValuesBackUp, (uint8_t*) brightnessValues, 4);
			memcpy((uint8_t*)autoBacklightflagsBackUp, (uint8_t*) autoBacklightflags, 4);
			backLightOffFlag = 1;

			for (i = 0; i < 4; i++) {
				autoBacklightflags[i] = BL_MODE_MAN; //выключение авторежима для всех драйверов
				brightnessValues[i] = 0; //Нулевое значение яркости для всех драйверов
				destTempBuf[i+6] = 0; // Заполняем нулями значения яркости пакета, отправляемого в драйвер
			}
			setTxMode(2); // В передачу
			HAL_UART_Transmit_DMA(&huart2, destTempBuf, 12);
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
			HAL_GPIO_WritePin(GPIO__12V_1_GPIO_Port, GPIO__12V_1_Pin, SET);
			cvStatusByte |= 0x04;
			setTxMode(6);
			HAL_UART_Transmit_DMA(&huart6, destTempBuf, CV_REQ_SIZE);
			__HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
			__HAL_TIM_SET_COUNTER(&htim7, 0);
			HAL_TIM_Base_Start_IT(&htim7);
			break;

		case CMD_PWR_ON:

			destTempBuf[1] = CV_REQ_PACK_ID;
			destTempBuf[2] = CV_REQ_SIZE;
			destTempBuf[3] = CMD_PWR_ON;
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

		case CMD_LIGHT_TABLE:

			for (i = 0; i < TAB_ENTRY_COUNT; i++) {
				lightTable[i].apdsValue = (uint16_t)source[i*3 + 4] << 8;
				lightTable[i].apdsValue |= (uint16_t) source[i*3 + 5];
				lightTable[i].brightness = source[i*3 + 6];
			}
			sendRespToRasp(CMD_LIGHT_TABLE, RESPONSE_OK);
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

	sensorsData *sensors;
	osEvent event, evt;
//	uint8_t raspTxBuf[STD_PACK_SIZE];
//	uint8_t i = 0;

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
				raspTxBuf[ADXL_PACK_SIZE-2] = get_check_sum(raspTxBuf, ADXL_PACK_SIZE);
				raspTxBuf[ADXL_PACK_SIZE-1] = 0x55;
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

				HAL_UART_Transmit_DMA(&huart1, sensors->payload, sensors->size);//транслируем в Raspberry

			} else if (sensors->source == CV_INA219_SOURCE) { //ответы от SourceSelector на команды управления питанием UART6

				raspTxBuf[CV_OFFSET] = sensors->payload[0];
				raspTxBuf[1 + CV_OFFSET] = sensors->payload[1];
				raspTxBuf[2 + CV_OFFSET] = sensors->payload[2];
				raspTxBuf[3 + CV_OFFSET] = sensors->payload[3];
				raspTxBuf[4 + CV_OFFSET] = sensors->payload[4];
				raspTxBuf[5 + CV_OFFSET] = sensors->payload[5];
				raspTxBuf[6 + CV_OFFSET] = sensors->payload[6];
				raspTxBuf[7 + CV_OFFSET] = sensors->payload[7];
				raspTxBuf[8 + CV_OFFSET] = sensors->payload[8];
				raspTxBuf[9 + CV_OFFSET] = sensors->payload[9];
				raspTxBuf[10 + CV_OFFSET] = sensors->payload[10];
				raspTxBuf[11 + CV_OFFSET] = sensors->payload[11];

			} else {

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

					breaksStateTelem = sensors->payload[19];
					breaksState = sensors->payload[19];
					memcpy(raspTxBuf + CV_OFFSET, sensors->payload + PACK_HEADER_SIZE, CV_SIZE);
					break;
				}
			}

			osMailFree(qSensorsHandle, sensors); //освобождаем память под очередью
		}

		evt = osSignalWait(0x02, 1); //отправляем по прерыванию от таймера

		if (evt.status == osEventSignal) {

			setStatusBytes();
			if (HAL_GPIO_ReadPin(GPIO__12V_1_GPIO_Port, GPIO__12V_1_Pin) == GPIO_PIN_SET) {
				cvStatusByte |= 0x04;
			} else {
				cvStatusByte &= ~0x04;
			}

//			if (HAL_GPIO_ReadPin(GPIO__12V_3_GPIO_Port, GPIO__12V_3_Pin) == GPIO_PIN_SET) {
//				cvStatusByte |= 0x08;
//			} else {
//				cvStatusByte &= ~0x08;
//			}

			if (rawCurrent[1] > 0) {
				cvStatusByte |= 0x08;
			} else {
				cvStatusByte &= ~0x08;
			}

			raspTxBuf[1] = STD_PACK_ID;
			raspTxBuf[2] = STD_PACK_SIZE;
			raspTxBuf[GERCON_OFFSET] = gerconState;
			raspTxBuf[ENGINE_STATE_OFFSET] = engineState;
			raspTxBuf[BAT2_STATE_OFFSET] = 0x05;
			raspTxBuf[BAT2_ERR_OFFSET] = 0x05;
			raspTxBuf[CV_STATUS_OFFSET] = cvStatusByte;
			raspTxBuf[CV_STATUS_OFFSET + 1] = cvStatusByte1;
			raspTxBuf[MIS_STATUS_OFFSET] = misStatusByte0;
			raspTxBuf[MIS_STATUS_OFFSET + 1] = misStatusByte1;
			raspTxBuf[DISCR_INPUT_OFFSET] = engineState;
			raspTxBuf[DISCR_INPUT_OFFSET + 1] = discreteInputState;
			raspTxBuf[CV_FIRMWARE_OFFSET] = cvFirmwareVersion0;
			raspTxBuf[CV_FIRMWARE_OFFSET + 1] = cvFirmwareVersion1;
			raspTxBuf[MIS_FIRMWARE_OFFSET] = misFirmwareVersion0;
			raspTxBuf[MIS_FIRMWARE_OFFSET + 1] = misFirmwareVersion1;
			raspTxBuf[MIS_FIRMWARE_OFFSET + 2] = lightMeterStatusByte;
			raspTxBuf[STD_PACK_SIZE-2] = get_check_sum(raspTxBuf, STD_PACK_SIZE);
			raspTxBuf[STD_PACK_SIZE-1] = 0x55;

			HAL_UART_Transmit_DMA(&huart1, raspTxBuf, STD_PACK_SIZE);
		}
	}
}

uint8_t get_check_sum(uint8_t *source, uint8_t size) {
	uint8_t i, uSum = 0;
	for (i = 0; i < size - 3; i++) {
		uSum = uSum + source[i];
	}
	uSum = (~uSum) + 1;
	return uSum;
}
