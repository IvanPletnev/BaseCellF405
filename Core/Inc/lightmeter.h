/*
 * lightmeter.h
 *
 *  Created on: Jul 27, 2021
 *      Author: ivan
 */

#ifndef INC_LIGHTMETER_H_
#define INC_LIGHTMETER_H_

#include "main.h"
#include "apds9960.h"
#include "usart.h"

typedef struct _lightdata {
	uint16_t apdsValue;
	uint8_t brightness;
}lightData;

#endif /* INC_LIGHTMETER_H_ */
