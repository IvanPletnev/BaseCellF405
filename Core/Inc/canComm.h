/*
 * canComm.h
 *
 *  Created on: Jan 31, 2022
 *      Author: ivan
 */

#ifndef INC_CANCOMM_H_
#define INC_CANCOMM_H_

#include "main.h"
#include "cmsis_os.h"
#include "CANSPI.h"

typedef struct _obdParamType {
	int16_t coolantTemp;
	uint16_t rpm;
	uint8_t speed;
	uint8_t throttleRel;
	uint16_t timeFromeStart;
	uint16_t chekLampDist;
	uint16_t onboardVoltage;
	uint8_t throttleAbs;
	uint8_t engineLoad;
	uint8_t fuelLevel;
} obdParamType;

void canRxTask(void const * argument);
void canTxTask(void const * argument);

#define OBD2_REQUEST_ID		0x7E0 //7DF
#define OBD2_RESPONSE_ID	0x7E8

#define COOLANT_TEMP_REQ	0x05 // A-40
#define RPM					0x0C //((A*256)+B)/4
#define SPEED				0x0D //A
#define THROTTLE_RELATIVE	0x11 //A*100/255
#define TIME_FROM_START		0x1F //(A*256)+B
#define CHECK_LAMP_DIST		0x21 //(A*256)+B
#define ONBOARD_VOLTAGE		0x42 //((A*256)+B)/1000
#define THROTTLE_ABS		0x45 //A*100/255
#define ENGINE_LOAD			0x04 //A*100/255
#define FUEL_LEVEL			0x2F //A*100/255



#endif /* INC_CANCOMM_H_ */
