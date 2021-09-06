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

#define TAB_ENTRY_COUNT	12

typedef struct _lightdata {
	uint16_t apdsValue;
	uint8_t brightness;
}lightData;

uint8_t isAutoBrightnessEnable (void);

#endif /* INC_LIGHTMETER_H_ */
