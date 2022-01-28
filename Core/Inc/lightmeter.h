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
#include "utilites.h"

#define TAB_ENTRY_COUNT	12

typedef struct _lightdata {
	uint16_t apdsValue;
	uint8_t brightness;
}lightData;

uint8_t isAutoBrightnessEnable (void);
extern lightData lightTable[TAB_ENTRY_COUNT];

#endif /* INC_LIGHTMETER_H_ */
