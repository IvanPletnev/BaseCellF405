/*
 * canComm.c
 *
 *  Created on: Jan 31, 2022
 *      Author: ivan
 */

#include "canComm.h"


uCAN_MSG canRxMessage;
uCAN_MSG canTxMessage;
obdParamType obdParameters;
uint32_t canPacketCounter = 0;
const uint8_t canObdIdSet[] = {COOLANT_TEMP_REQ, RPM, SPEED, THROTTLE_RELATIVE, TIME_FROM_START, CHECK_LAMP_DIST,
		ONBOARD_VOLTAGE, THROTTLE_ABS, ENGINE_LOAD, FUEL_LEVEL};

extern osMutexId spiMutexHandle;

uint8_t obdMessageParcer(uCAN_MSG *message, obdParamType *param) {

	switch (message->frame.id) {
	case OBD2_RESPONSE_ID:
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
		case THROTTLE_RELATIVE:
			param->throttleRel = (message->frame.data3) * 100 / 255;
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
		case THROTTLE_ABS:
			param->throttleAbs = (message->frame.data3) * 100 / 255;
			break;
		case ENGINE_LOAD:
			param->engineLoad  = (message->frame.data3) * 100 / 255;
			break;
		case FUEL_LEVEL:
			param->fuelLevel = (message->frame.data3) * 100 / 255;
			break;
		default:
			return 0;
		}
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
			if (osMutexWait(spiMutexHandle, 5) == osOK) {
				CANSPI_Receive(&canRxMessage);
				MCP2515_BitModify(MCP2515_CANINTF, 255, 0);
				osMutexRelease(spiMutexHandle);
			}
			obdMessageParcer(&canRxMessage, &obdParameters);
			++canPacketCounter;
			osThreadYield();
		}
	}
}

void canTxTask(void const * argument){

	uint8_t i = 0;
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
			i = 0;
			osDelay(200);
		}
	}
}
