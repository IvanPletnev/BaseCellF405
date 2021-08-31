/*
 * lightmeter.c
 *
 *  Created on: Jul 27, 2021
 *      Author: ivan
 */

#include "lightmeter.h"

#define TAB_ENTRY_COUNT	11
#define DISCRETE		10

uint16_t lightLevel = 0;
uint16_t lightLevel1 = 0;

uint32_t lightSum = 0;
extern osMailQId qSensorsHandle;

const lightData lightTable[TAB_ENTRY_COUNT] = {
		{0, 2},
		{6554, 5},
		{13107, 5},
		{19662, 10},
		{26216, 20},
		{32768, 30},
		{39322, 40},
		{45875, 50},
		{52428, 60},
		{58982, 70},
		{65535, 70}
};

uint8_t getAutoBrightness (uint16_t apds){
	uint8_t i = 0;
	for (i = 0; i < TAB_ENTRY_COUNT-1; i++) {
		if ((apds >= lightTable[i].apdsValue) && (apds < lightTable[i+1].apdsValue)) {
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

void setAutoBrightnessPacket (sensorsData *arg){

	uint8_t i = 0;
	uint8_t autoBrValue = 0;

	autoBrValue = getAutoBrightness((uint16_t)lightSum);

//	autoBrValue = (autoBrValue + (DISCRETE / 2)) / DISCRETE;
//	autoBrValue *= DISCRETE;

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
			}
		} else {
			arg->payload[i+6] = 0;
		}
	}
	arg->payload[10] = get_check_sum(arg->payload, BL_AUTO_CTL_SIZE);
	arg->payload[11] = 0x55;
}

void lightMeterTask(void const * argument) {

	static uint8_t light0[2];
	static uint8_t light1[2];
	static uint8_t red0[2];
	static uint8_t red1[2];
	static uint8_t green0[2];
	static uint8_t green1[2];
	static uint8_t blue0[2];
	static uint8_t blue1[2];
	sensorsData *sensors = {0};
	sensorsData *autoBlQueue = {0};
	osDelay(200);
	/* Infinite loop */

	for (;;) {

		APDS9960_ReadLight(0, light0);
		APDS9960_ReadLight(1, light1);
		APDS9960_ReadRedLight(0, red0);
		APDS9960_ReadRedLight(1, red1);
		APDS9960_ReadGreenLight(0, green0);
		APDS9960_ReadGreenLight(1, green1);
		APDS9960_ReadBlueLight(0, blue0);
		APDS9960_ReadBlueLight(1, blue1);

		lightLevel = (uint16_t)light0[0] << 8;
		lightLevel |= light0[1];
		lightLevel1 = (uint16_t)light1[0] << 8;
		lightLevel1 |= light1[1];
		lightSum = (lightLevel + lightLevel1) / 2;


		sensors = osMailAlloc(qSensorsHandle, osWaitForever);
		sensors->source = APDS_TASK_SOURCE;
		sensors->size = APDS_SIZE;
//		memset(sensors->payload, 0, 16);
		sensors->payload[0] = light0[0]; sensors->payload[1] = light0[1]; sensors->payload[2] = light1[0]; sensors->payload[3] = light1[1];
		sensors->payload[4] = red0[0]; sensors->payload[5] = red0[1]; sensors->payload[6] = red1[0]; sensors->payload[7] = red1[1];
		sensors->payload[8] = green0[0]; sensors->payload[9] = green0[1]; sensors->payload[10] = green1[0]; sensors->payload[11] = green1[1];
		sensors->payload[12] = blue0[0]; sensors->payload[13] = blue0[1]; sensors->payload[14] = blue1[0]; sensors->payload[15] = blue1[1];
		osMailPut(qSensorsHandle, sensors);

		if (isAutoBrightnessEnable() != 0){
			autoBlQueue = osMailAlloc(qSensorsHandle, osWaitForever);
			autoBlQueue->source = BL_AUTO_CONTROL_SRC;
			autoBlQueue->size = BL_AUTO_CTL_SIZE;
			setAutoBrightnessPacket(autoBlQueue);
			osMailPut(qSensorsHandle, autoBlQueue);
		}
		osDelay(1000);
	}
}
