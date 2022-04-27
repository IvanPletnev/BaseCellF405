/*
 * canComm.c
 *
 *  Created on: Jan 31, 2022
 *      Author: ivan
 */

#include "canComm.h"
#include "main.h"
#include "string.h"

uCAN_MSG canRxMessage;
uCAN_MSG canTxMessage;
obdParamType obdParameters = {
		0
//		.coolantTemp = 70,
//		.rpm = 2000,
//		.speed = 50,
//		.timeFromeStart = 600,
//		.chekLampDist = 1000,
//		.onboardVoltage = 14500,
//		.engineLoad = 25,
//		.fuelLevel = 10,
//		.intakeAirTemp = 40,
//		.brakeForce = 3000,
//		.steeringAngle = -2500,
//		.throttleLevel = 60
};

uint32_t canPacketCounter = 0;
const uint8_t canObdIdSet[] = {COOLANT_TEMP_REQ, RPM, SPEED, TIME_FROM_START, CHECK_LAMP_DIST,
		ONBOARD_VOLTAGE, ENGINE_LOAD, FUEL_LEVEL, INTAKE_AIR_TEMP};

volatile canStatusByte0 can_status_byte_0 = {
		0
//		.ignition = 1,
//		.engineStatus = 1,
//		.starterStatus = 0,
//		.windScreenVipers = 2,
};
volatile canStatusByte1 can_status_byte_1 = {
		0
//		.parkingLights = 1,
//		.dippedHeadLights = 0,
//		.highBeamHeadlights = 1,
//		.directionIndicatorRight = 0,
//		.directionIndicatorLeft = 1,
//		.hazardLights = 1,
};
volatile canStatusByte2 can_status_byte_2 = {
		0
//		.driverDoor = 1,
//		.rightFrontDoor = 1,
//		.leftRearDoor = 0,
//		.rightRearDoor = 0,
//		.handBrake = 1,
//		.driverSeatBelt = 1,
//		.trunk = 1
};

volatile canStatusByte3 can_status_byte_3 = {
		0
//		.ignitionLock = 3,
};

extern osMutexId spiMutexHandle;


uint8_t obdMessageParcer(uCAN_MSG *message, obdParamType *param) {

	switch (message->frame.id) {

	case OBD2_RESPONSE_ID: {

		switch (message->frame.data2) {
		case COOLANT_TEMP_REQ:
			param->coolantTemp = message->frame.data3 - 40;
			break;
		case RPM:
			param->rpm = ((((uint16_t)(message->frame.data3)) << 8) | (uint16_t)message->frame.data4) >> 2;
			break;
		case SPEED:
			param->speed = message->frame.data3;
			break;
		case TIME_FROM_START:
			param->timeFromeStart = (((uint16_t)(message->frame.data3)) << 8) | (uint16_t)message->frame.data4;
			break;
		case CHECK_LAMP_DIST:
			param->chekLampDist = (((uint16_t)(message->frame.data3)) << 8) | (uint16_t)message->frame.data4;
			break;
		case ONBOARD_VOLTAGE:
			param->onboardVoltage = (((uint16_t)(message->frame.data3)) << 8) | (uint16_t)message->frame.data4;
			break;
		case ENGINE_LOAD:
			param->engineLoad  = (message->frame.data3) * 100 / 255;
			break;
		case FUEL_LEVEL:
			param->fuelLevel = (message->frame.data3) * 100 / 255;
			break;
		case INTAKE_AIR_TEMP:
			param->intakeAirTemp = message->frame.data3 - 40;
			break;
		default:
			return 0;
		}
		break;
	}

	case 0x541: {

		if (message->frame.data1 & 0x40) {
			can_status_byte_0.ignition = 1;
		} else {
			can_status_byte_0.ignition = 0;
		}

		if (message->frame.data1 & 0x01) {
			can_status_byte_2.driverDoor = 1;
		} else {
			can_status_byte_2.driverDoor = 0;
		}

		if (message->frame.data4 & 0x08) {
			can_status_byte_2.rightFrontDoor = 1;
		} else {
			can_status_byte_2.rightFrontDoor = 0;
		}

		if (message->frame.data1 & 0x10) {
			can_status_byte_2.trunk = 1;
		} else {
			can_status_byte_2.trunk = 0;
		}

		if (message->frame.data3 & 0x80) {
			can_status_byte_1.dippedHeadLights = 1;
		} else {
			can_status_byte_1.dippedHeadLights = 0;
		}

		if (message->frame.data4 & 0x01) {
			can_status_byte_1.highBeamHeadlights = 1;
		} else {
			can_status_byte_1.highBeamHeadlights = 0;
		}

		if (message->frame.data2 & 0x08) {
			can_status_byte_1.directionIndicatorLeft = 1;
		} else {
			can_status_byte_1.directionIndicatorLeft = 0;
		}

		if (message->frame.data7 & 0x40) {
			can_status_byte_1.directionIndicatorRight = 1;
		} else {
			can_status_byte_1.directionIndicatorRight = 0;
		}

		if (message->frame.data4 & 0x02) {
			can_status_byte_1.hazardLights = 1;
		} else {
			can_status_byte_1.hazardLights = 0;
		}

		can_status_byte_0.windScreenVipers = message->frame.data3 & 0x07;

		if (message->frame.data1 & 0x04) {
			can_status_byte_2.driverSeatBelt = 1;
		} else {
			can_status_byte_2.driverSeatBelt = 0;
		}

		if (message->frame.data7 & 0x10) {
			can_status_byte_2.handBrake = 1;
		} else {
			can_status_byte_2.handBrake = 0;
		}

		can_status_byte_3.ignitionLock = message->frame.data0 & 0x07;

		break;
	}

	case 0x329: {

		param->throttleLevel = message->frame.data6;

		break;
	}

//	case 0x553: {
//
//		if (message->frame.data3 & 0x01) {
//			can_status_byte_2.rightRearDoor = 1;
//		} else {
//			can_status_byte_2.rightRearDoor = 0;
//		}
//
//		if (message->frame.data2 & 0x80) {
//			can_status_byte_2.leftRearDoor = 1;
//		} else {
//			can_status_byte_2.leftRearDoor = 0;
//		}
//
//		if (message->frame.data2 & 0x40) {
//			can_status_byte_1.parkingLights = 1;
//		} else {
//			can_status_byte_1.parkingLights = 0;
//		}
//
//		break;
//	}

	case 0x260: {

		if (message->frame.data3 & 0x20) {
			can_status_byte_0.engineStatus = 1;
		} else {
			can_status_byte_0.engineStatus = 0;
		}

		break;
	}


	case 0x220: {

		param->brakeForce = (uint16_t)(message->frame.data4 << 8) | (uint16_t)(message->frame.data3);
		break;
	}

	case 0x2B0:
		param->steeringAngle = (int16_t) ((uint16_t)(message->frame.data1 << 8) | (uint16_t) (message->frame.data0));
		break;

	default:
		return 0;
	}
	return 1;
}


void canRxTask(void const * argument){

	osDelay(50);
	MCP2515_Reset();
	CANSPI_Initialize();
	osDelay(50);
	osEvent evt;


	for (;;){
		evt = osSignalWait(0x0A, osWaitForever);
		if (evt.status == osEventSignal) {
			if (osMutexWait(spiMutexHandle, 1) == osOK) {
				CANSPI_Receive(&canRxMessage);
				MCP2515_BitModify(MCP2515_CANINTF, 255, 0);
				osMutexRelease(spiMutexHandle);
				taskENTER_CRITICAL();
				obdMessageParcer(&canRxMessage, &obdParameters);
				++canPacketCounter;
				taskEXIT_CRITICAL();
			}
			osThreadYield();
		}
	}
}

void canTxTask(void const * argument){

	uint8_t i = 0;
	sensorsData xSensors;
	sensorsData *sensors = &xSensors;
	uint8_t offset = sizeof (obdParameters);

	canTxMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
	canTxMessage.frame.id = 0x7E0;
	canTxMessage.frame.dlc = 8;
	canTxMessage.frame.data0 = 2;
	canTxMessage.frame.data1 = 1;
	canTxMessage.frame.data3 = 0;
	canTxMessage.frame.data4 = 0;
	canTxMessage.frame.data5 = 0;
	canTxMessage.frame.data6 = 0;
	canTxMessage.frame.data7 = 0;

	for (;;){

		if (i < sizeof(canObdIdSet)) {
			canTxMessage.frame.data2 = canObdIdSet[i];
			if (osMutexWait(spiMutexHandle, 5) == osOK) {
				CANSPI_Transmit(&canTxMessage);
				MCP2515_BitModify(MCP2515_CANINTF, 255, 0);
				osMutexRelease(spiMutexHandle);
			}
			i++;
			osDelay(10);
		} else {
			taskENTER_CRITICAL();
			sensors->source = CAN_SOURCE;
			sensors->size = CAN_SIZE;
			memcpy (sensors->payload, (uint8_t*) & obdParameters, sizeof (obdParameters));
			sensors->payload[offset] = can_status_byte_0.canByte0;
			sensors->payload[offset + 1] = can_status_byte_1.canByte1;
			sensors->payload[offset + 2] = can_status_byte_2.canByte2;
			sensors->payload[offset + 3] = can_status_byte_3.canByte3;
			taskEXIT_CRITICAL();
			if (xQueueSend(qSensorsHandle, (void*)&sensors, 1) != pdTRUE) {
			}
			i = 0;
			osDelay(100);
		}
//		osDelay(200);
	}
}
