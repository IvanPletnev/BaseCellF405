/*
 * lightmeter.c
 *
 *  Created on: Jul 27, 2021
 *      Author: ivan
 */

#include "lightmeter.h"

#define DISCRETE		10

extern osMailQId qSensorsHandle;
extern I2C_HandleTypeDef hi2c1;
extern uint8_t queueStatusByte;

uint16_t debugLightLevel0 = 0;
uint16_t debugLightLevel1 = 0;

uint8_t result;
lightData lightTable[TAB_ENTRY_COUNT] = {

	{0, 2},
	{100, 10},
	{300, 15},
	{700, 25},
	{1000, 30},
	{10000, 45},
	{19662, 50},
	{26216, 55},
	{32768, 60},
	{45875, 60},
	{58982, 60},
	{65535, 60}

};

uint8_t getAutoBrightness (uint16_t apds){
	uint8_t i = 0;

	if (apds == 0){
		return 2;
	}

	for (i = 0; i < TAB_ENTRY_COUNT-1; i++) {
		if ((apds > lightTable[i].apdsValue) && (apds <= lightTable[i+1].apdsValue)) {
			return lightTable[i].brightness;
		}
	}
	return 0;
}

uint8_t isAutoBrightnessEnable (void){

	uint8_t i = 0;
	uint8_t value = 0;

	for (i = 0; i < 4; i++){
		if (autoBacklightflags[i]) {
			value++;
		}
	}
	return value;
}

void setAutoBrightnessPacket (sensorsData *arg, uint32_t light){

	uint8_t i = 0;
	uint8_t autoBrValue = 0;

	autoBrValue = getAutoBrightness((uint16_t)light);

	arg->payload[0] = PACKET_HEADER;
	arg->payload[1] = BL_OUT_PACK_ID;
	arg->payload[2] = BL_AUTO_CTL_SIZE;
	arg->payload[3] = CMD_START_IMG;
	arg->payload[4] = BL_MODE_MAN;
	arg->payload[5] = 0x32;

	for (i = 0; i < 4; i++){
		if (brightnessValues[i] != 0){
			if (autoBacklightflags[i]){
				arg->payload[i+6] = autoBrValue;
			} else {
				arg->payload[i+6] = brightnessValues[i];
			}
		} else {
			arg->payload[i+6] = 0;
		}
	}
	arg->payload[10] = get_check_sum(arg->payload, BL_AUTO_CTL_SIZE);
	arg->payload[11] = 0x55;
}

void lightMeterTask(void const * argument) {

	//ligtMeterStatusByte 							7   6   5   4   3   2   1   0
	// Ошибка чтения значения с датчика 0					|   |   |   |	|	1  (0x01)
	// Ошибка чтения значения с датчика 1					|	|	|	|	1	   (0x02)
	// Магические числа на датчике 0, reinit				|	|	|	1		   (0x04)
	// Магические числа на датчике 0, reinit				|	|	1			   (0x08)
	// Потеря инициализации датчик 0, reinit				|	1				   (0x10)
	// Потеря инициализации датчик 1, reinit				1					   (0x20)

	uint8_t light0[2] = {0};
	uint8_t light1[2] = {0};
	uint8_t red0[2];
	uint8_t red1[2];
	uint8_t green0[2];
	uint8_t green1[2];
	uint8_t blue0[2];
	uint8_t blue1[2];

	uint16_t lightLevel = 0;
	uint16_t lightLevel1 = 0;
	uint32_t lightSum = 0;
	static uint32_t lightSumFiltered = 0;
	sensorsData *sensors = {0};
	sensorsData *autoBlQueue = {0};
	uint8_t aTime0 = 0;
	uint8_t aTime1 = 0;
	uint8_t status0 = 0;
	uint8_t status1 = 0;

	osDelay(500);
	APDS9960_Init();
	osDelay(100);
	/* Infinite loop */

	for (;;) {

		if((status0 = APDS9960_ReadLight(0, light0)) != STATUS_OK){
			light0[0] = 0; light0[1] = 0; red0[0] = 0; red0[1] = 0; green0[0] = 0; green0[1] = 0; blue0[1] = 0;
		}

		if ((status1 = APDS9960_ReadLight(1, light1)) != STATUS_OK){
			light1[0] = 0; light1[1] = 0; red1[0] = 0; red1[1] = 0; green1[0] = 0; green1[1] = 0; blue1[1] = 0;
		}

		if (status0) {
			APDS9960_ReadRedLight(0, red0);
			APDS9960_ReadGreenLight(0, green0);
			APDS9960_ReadBlueLight(0, blue0);
		}

		if (status1) {
			APDS9960_ReadRedLight(1, red1);
			APDS9960_ReadGreenLight(1, green1);
			APDS9960_ReadBlueLight(1, blue1);
		}

		if (!status0 && !status1) {
			HAL_I2C_DeInit(&hi2c1);
			osDelay(10);
			HAL_I2C_Init(&hi2c1);
			APDS9960_Init();
		}

		lightLevel = (uint16_t)light0[0] << 8;
		lightLevel |= light0[1];

		lightLevel1 = (uint16_t)light1[0] << 8;
		lightLevel1 |= light1[1];
		debugLightLevel0 = lightLevel;
		debugLightLevel1 = lightLevel1;

		APDS9960_SetActiveChan(0);
		sensorReadDataByte(APDS9960_ATIME, &aTime0);
		if (aTime0 != 0xC0) {
			disableLightSensor();
			disablePower();
			osDelay(10);
			enablePower();
			init();
			osDelay(20);
		}

		APDS9960_SetActiveChan(1);
		sensorReadDataByte(APDS9960_ATIME, &aTime1);
		if (aTime1 != 0xC0) {
			disableLightSensor();
			disablePower();
			osDelay(10);
			enablePower();
			init();
			osDelay(20);
		}

		lightSum = ((uint32_t)lightLevel + (uint32_t)lightLevel1) / 2;
		lightSumFiltered = filtering(lightSum, &currentFilter[0]);

		sensors = osMailAlloc(qSensorsHandle, 10);
		if (sensors != NULL){
			sensors->source = APDS_TASK_SOURCE;
			sensors->size = APDS_SIZE;
			sensors->payload[0] = light0[0]; sensors->payload[1] = light0[1]; sensors->payload[2] = light1[0]; sensors->payload[3] = light1[1];
			sensors->payload[4] = red0[0]; sensors->payload[5] = red0[1]; sensors->payload[6] = red1[0]; sensors->payload[7] = red1[1];
			sensors->payload[8] = green0[0]; sensors->payload[9] = green0[1]; sensors->payload[10] = green1[0]; sensors->payload[11] = green1[1];
			sensors->payload[12] = blue0[0]; sensors->payload[13] = blue0[1]; sensors->payload[14] = blue1[0]; sensors->payload[15] = blue1[1];
			osMailPut(qSensorsHandle, sensors);
		} else {
			queueStatusByte |= 0x04;
		}

		autoBlQueue = osMailAlloc(qSensorsHandle, 10);
		if (autoBlQueue != NULL){
			autoBlQueue->source = BL_AUTO_CONTROL_SRC;
			autoBlQueue->size = BL_AUTO_CTL_SIZE;
			setAutoBrightnessPacket(autoBlQueue, lightSumFiltered);
			if (!isAutoBrightnessEnable()){
				autoBlQueue->payload[5] = dimmingTime;
			}
			osMailPut(qSensorsHandle, autoBlQueue);
		} else {
			queueStatusByte |= 0x08;
		}

		osDelay(500);
	}
}
