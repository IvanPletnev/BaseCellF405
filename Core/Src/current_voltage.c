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
#include "apds9960.h"


int16_t rawCurrent[3];
int16_t rawVoltage[3];
int16_t averageCurrent[3];
int16_t averageVoltage[3];
osStatus cvMutexStatus0;
osStatus cvMutexStatus1;
uint32_t i2cErrorCode = 0;

extern osMutexId I2C2MutexHandle;
extern osMailQId qSensorsHandle;
extern I2C_HandleTypeDef hi2c2;

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C2) {
		i2cErrorCode = HAL_I2C_GetError(&hi2c2);
	}
}


void cvTask(void const * argument){

	sensorsData *sensors;
	uint16_t regValue = 0;
	ina219Init();
	osDelay(200);
	static uint8_t errorsCounter = 0;

	for(;;){

			cvMutexStatus0 = osMutexWait(I2C2MutexHandle, 10);
			if (cvMutexStatus0 == osOK){

				if (ina219ReadRegister(0, INA219_REG_CONFIG, &regValue) != STATUS_FAIL) {
					if (regValue != 0x0C67){
						ina219SetCalibration_16V_80A_075mOhm(0);
						regValue = 0;
					}
					rawCurrent[0] = ina219GetCurrent_raw(0); // ток потребления бортовой сети. отрицательное значение
					rawVoltage[0] = ina219GetBusVoltage_raw(0); //получаем значения напряжения
				} else {
					rawCurrent[0] = 0;
					rawVoltage[0] = 0;
					++errorsCounter;
				}


				if (ina219ReadRegister(1, INA219_REG_CONFIG, &regValue) != STATUS_FAIL) {
					if (regValue != 0x0C67){
						ina219SetCalibration_16V_80A_05mOhm(1);
						regValue = 0;
					}
					rawCurrent[1] = ina219GetCurrent_raw(1); // ток первого аккумулятора. "+" зарядка, "-" потребление
					rawVoltage[1] = ina219GetBusVoltage_raw(1);
				} else {
					rawCurrent[1] = 0;
					rawVoltage[1] = 0;
					++errorsCounter;
				}


				if (ina219ReadRegister(2, INA219_REG_CONFIG, &regValue) != STATUS_FAIL) {
					if (regValue != 0x0C67){
						ina219SetCalibration_16V_80A(2);
						regValue = 0;
					}
					rawCurrent[2] = ina219GetCurrent_raw(2); // ток первого аккумулятора. "+" зарядка, "-" потребление
					rawVoltage[2] = ina219GetBusVoltage_raw(2);
				} else {
					rawCurrent[2] = 0;
					rawVoltage[2] = 0;
					++errorsCounter;
				}
				if (errorsCounter == 3){
					HAL_I2C_DeInit(&hi2c2);
					osDelay(5);
					HAL_I2C_Init(&hi2c2);
					ina219Init();
				}
				cvMutexStatus1 = osMutexRelease(I2C2MutexHandle);
				errorsCounter = 0;
			}



			averageCurrent[0] = filtering(rawCurrent[0], &currentFilter[0]);
			averageCurrent[1] = filtering(rawCurrent[1], &currentFilter[1]);
			averageCurrent[2] = filtering(rawCurrent[2], &currentFilter[2]);
			averageVoltage[0] = filtering(rawVoltage[0], &voltageFilter[0]);
			averageVoltage[1] = filtering(rawVoltage[1], &voltageFilter[1]);
			averageVoltage[2] = filtering(rawVoltage[2], &voltageFilter[2]);


			sensors = osMailAlloc(qSensorsHandle, 1);
			sensors->source = CV_INA219_SOURCE;
			sensors->size = CV_INA219_SIZE;
			sensors->payload[0] = (uint8_t)((rawVoltage[0] & 0xFF00) >> 8);
			sensors->payload[1] = (uint8_t)(rawVoltage[0] & 0x00FF);
			sensors->payload[2] = (uint8_t)((rawVoltage[1] & 0xFF00) >> 8);
			sensors->payload[3] = (uint8_t)(rawVoltage[1] & 0x00FF);
			sensors->payload[4] = (uint8_t)((rawVoltage[2] & 0xFF00) >> 8);
			sensors->payload[5] = (uint8_t)(rawVoltage[2] & 0x00FF);
			sensors->payload[6] = (uint8_t)((rawCurrent[0] & 0xFF00) >> 8);
			sensors->payload[7] = (uint8_t)(rawCurrent[0] & 0x00FF);
			sensors->payload[8] = (uint8_t)((rawCurrent[1] & 0xFF00) >> 8);
			sensors->payload[9] = (uint8_t)(rawCurrent[1] & 0x00FF);
			sensors->payload[10] = (uint8_t)((rawCurrent[2] & 0xFF00) >> 8);
			sensors->payload[11] = (uint8_t)(rawCurrent[2] & 0x00FF);

			osMailPut(qSensorsHandle, sensors);
			osDelay(300);
		}
}
