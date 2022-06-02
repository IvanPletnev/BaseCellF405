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


void canRxTask(void const * argument);
void canTxTask(void const * argument);

#define OBD2_REQUEST_ID		0x7E0 //7DF
#define OBD2_RESPONSE_ID	0x7E8

#define COOLANT_TEMP_REQ	0x05 // A-40
#define RPM					0x0C //((A*256)+B)/4
#define SPEED				0x0D //A
#define TIME_FROM_START		0x1F //(A*256)+B
#define CHECK_LAMP_DIST		0x21 //(A*256)+B
#define ONBOARD_VOLTAGE		0x42 //((A*256)+B)/1000
#define ENGINE_LOAD			0x04 //A*100/255
#define FUEL_LEVEL			0x2F //A*100/255
#define INTAKE_AIR_TEMP		0x0F //A-40
#define ENGINE_OIL_TEMP		0x5C //A-40

//typedef enum _doorState {
//	OPEN,
//	CLOSED
//}doorState;
//
//typedef enum _onOffState {
//	ON,
//	OFF
//}onOffState;

typedef struct _obdParamType {
	int8_t coolantTemp;
	uint16_t rpm;
	uint8_t speed;
	uint16_t timeFromeStart;
	uint16_t chekLampDist;
	uint16_t onboardVoltage;
	uint8_t engineLoad;
	uint8_t fuelLevel;
	int8_t intakeAirTemp;
	int8_t engineOilTemp;
	uint16_t brakeForce;
	int16_t steeringAngle;
	uint8_t throttleLevel;

} __attribute__((packed)) obdParamType;

typedef union {
	struct{
		unsigned ignition 				: 1;
		unsigned engineStatus			: 1;
		unsigned starterStatus			: 1;
		unsigned windScreenVipers		: 3;
		unsigned reserved				: 2;
	};
	uint8_t canByte0;
} canStatusByte0;

typedef union {
	struct{
		unsigned parkingLights				: 1;
		unsigned dippedHeadLights			: 1;
		unsigned highBeamHeadlights			: 1;
		unsigned directionIndicatorLeft		: 1;
		unsigned directionIndicatorRight	: 1;
		unsigned hazardLights				: 1;
		unsigned reserved					: 2;
	};
	uint8_t canByte1;
} canStatusByte1;

typedef union {
	struct{
		unsigned ignitionLock				: 3;
		unsigned reserved					: 5;
	};
	uint8_t canByte2;
} canStatusByte2;

typedef union {
	struct{
		unsigned driverDoor					: 1;
		unsigned rightFrontDoor				: 1;
		unsigned leftRearDoor				: 1;
		unsigned rightRearDoor				: 1;
		unsigned handBrake					: 1;
		unsigned driverSeatBelt				: 1;
		unsigned trunk						: 1;
		unsigned reserved					: 1;
	};
	uint8_t canByte3;
} canStatusByte3;






#endif /* INC_CANCOMM_H_ */
