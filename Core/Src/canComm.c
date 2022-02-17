/*
 * canComm.c
 *
 *  Created on: Jan 31, 2022
 *      Author: ivan
 */

#include "canComm.h"

uCAN_MSG canMessage;
uint32_t canPacketCounter = 0;

void canRxTask(void const * argument){
	osDelay(50);
	MCP2515_Reset();
	CANSPI_Initialize();
	osDelay(50);
	osEvent evt;

	for (;;){
		evt = osSignalWait(0x0A, osWaitForever);
		if (evt.status == osEventSignal) {
			CANSPI_Receive(&canMessage);
			MCP2515_BitModify(MCP2515_CANINTF, 255, 0);
			++canPacketCounter;
			osThreadYield();
		}

	}
}

void canTxTask(void const * argument){

	for (;;){

		osDelay(1);
	}
}
