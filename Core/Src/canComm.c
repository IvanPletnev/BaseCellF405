/*
 * canComm.c
 *
 *  Created on: Jan 31, 2022
 *      Author: ivan
 */

#include "canComm.h"

uCAN_MSG canMessage;

void canRxTask(void const * argument){
	osDelay(50);
	MCP2515_Reset();
	CANSPI_Initialize();
	osDelay(50);
	osEvent evt;

	for (;;){
		evt = osSignalWait(0x0A, osWaitForever);
		if (evt.status == osEventSignal) {
			taskENTER_CRITICAL();
			CANSPI_Receive(&canMessage);
			taskEXIT_CRITICAL();
			asm ("NOP");
		}

	}
}

void canTxTask(void const * argument){

	for (;;){

		osDelay(1);
	}
}
