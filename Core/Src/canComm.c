/*
 * canComm.c
 *
 *  Created on: Jan 31, 2022
 *      Author: ivan
 */

#include "canComm.h"

uCAN_MSG canMessage;

void canRxTask(void const * argument){
	osEvent evt;

	for (;;){
		evt = osSignalWait(0x0A, osWaitForever);
		if (evt.status == osEventSignal) {
			CANSPI_Receive(&canMessage);
			asm ("NOP");
		}

	}
}

void canTxTask(void const * argument){

	for (;;){

		osDelay(1);
	}
}
