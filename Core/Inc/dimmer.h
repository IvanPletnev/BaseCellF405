/*
 * dimmer.h
 *
 *  Created on: Apr 11, 2022
 *      Author: Artem K.
 */

#ifndef INC_SENSORS_DIMMER_H_
#define INC_SENSORS_DIMMER_H_

#include "main.h"
#include "bh1750.h"

#define DEFAULT_PWM 500
#define MIN_PWM 100
#define MAX_PWM 900
#define DIMM_MULTIPLER 80

extern BH1750_device_t* monitor;
extern float pwmValue;
extern uint16_t targetValue;
extern uint16_t pwm_Value;

uint16_t SetMonitorBackligt(void);
uint16_t GetDimPwm(void);

#endif /* INC_SENSORS_DIMMER_H_ */
