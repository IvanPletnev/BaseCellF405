/*
 * current_voltage.c
 *
 *  Created on: Jan 27, 2022
 *      Author: ivan
 */

#include "main.h"
#include "current_voltage.h"
#include "utilites.h"
#include "cmsis_os.h"

int16_t rawCurrent[3];
int16_t rawVoltage[3];
int16_t averageCurrent[3];
int16_t averageVoltage[3];

extern osMutexId I2C2MutexHandle;
extern osMailQId qSensorsHandle;

void cvTask(void const * argument){


	sensorsData *sensors;

	ina219Init();
	osDelay(200);

	for(;;){

			if (osMutexWait(I2C2MutexHandle, 200) != osOK){
			}
			rawCurrent[0] = ina219GetCurrent_raw(0); // ток потребления бортовой сети. отрицательное значение
			rawCurrent[1] = ina219GetCurrent_raw(1); // ток первого аккумулятора. "+" зарядка, "-" потребление
			rawCurrent[2] = ina219GetCurrent_raw(2); // ток первого аккумулятора. "+" зарядка, "-" потребление
			rawVoltage[0] = ina219GetBusVoltage_raw(0); //получаем значения напряжения
			rawVoltage[1] = ina219GetBusVoltage_raw(1);
			rawVoltage[2] = ina219GetBusVoltage_raw(2);

			osMutexRelease(I2C2MutexHandle);

			averageCurrent[0] = filtering(rawCurrent[0], &currentFilter[0]);
			averageCurrent[1] = filtering(rawCurrent[1], &currentFilter[1]);
			averageCurrent[2] = filtering(rawCurrent[2], &currentFilter[2]);
			averageVoltage[0] = filtering(rawVoltage[0], &voltageFilter[0]);
			averageVoltage[1] = filtering(rawVoltage[1], &voltageFilter[1]);
			averageVoltage[2] = filtering(rawVoltage[2], &voltageFilter[2]);


			sensors = osMailAlloc(qSensorsHandle, 1);
			sensors->source = CV_INA219_SOURCE;
			sensors->size = CV_INA219_SIZE;
			sensors->payload[0] = (uint8_t)((averageVoltage[0] & 0xFF00) >> 8);
			sensors->payload[1] = (uint8_t)(averageVoltage[0] & 0x00FF);
			sensors->payload[2] = (uint8_t)((averageVoltage[1] & 0xFF00) >> 8);
			sensors->payload[3] = (uint8_t)(averageVoltage[1] & 0x00FF);
			sensors->payload[4] = (uint8_t)((averageVoltage[2] & 0xFF00) >> 8);
			sensors->payload[5] = (uint8_t)(averageVoltage[2] & 0x00FF);
			sensors->payload[6] = (uint8_t)((averageCurrent[0] & 0xFF00) >> 8);
			sensors->payload[7] = (uint8_t)(averageCurrent[0] & 0x00FF);
			sensors->payload[8] = (uint8_t)((averageCurrent[1] & 0xFF00) >> 8);
			sensors->payload[9] = (uint8_t)(averageCurrent[1] & 0x00FF);
			sensors->payload[10] = (uint8_t)((averageCurrent[2] & 0xFF00) >> 8);
			sensors->payload[11] = (uint8_t)(averageCurrent[2] & 0x00FF);

			osMailPut(qSensorsHandle, sensors);
			osDelay(300);
		}
}
