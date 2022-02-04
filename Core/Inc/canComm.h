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


#endif /* INC_CANCOMM_H_ */
