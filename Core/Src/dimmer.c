/*
 * dimmer.c
 *
 *  Created on: Apr 11, 2022
 *      Author: Artem K.
 */

#include "dimmer.h"


//monitor = BH1750_init_dev_struct(&hi2c3, "monitor", true);


uint16_t GetDimPwm(void)
{
	pwmValue = MAX_PWM - ((((float)monitor->value / 65353.0) * 1000) * DIMM_MULTIPLER);
	return (uint16_t)pwmValue;
}

uint16_t SetMonitorBackligt(void)
{


	if(GetDimPwm() < MIN_PWM)
	{
		targetValue=MIN_PWM;
	}
	else if(GetDimPwm() > MAX_PWM)
	{
		targetValue=MAX_PWM;
	}
	else
	{
		targetValue=GetDimPwm();
	}

	if(targetValue > pwm_Value)
	{
		pwm_Value++;
		//	osDelay(10);
	}
	if(targetValue < pwm_Value)
	{
		pwm_Value--;
		//	osDelay(10);
	}

	return pwm_Value;

}
