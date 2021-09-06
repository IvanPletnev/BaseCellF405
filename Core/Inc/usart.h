/*
 * usart.h
 *
 *  Created on: Jun 25, 2021
 *      Author: ivan
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "string.h"
#include "lightmeter.h"

typedef enum _usartErrT {
	ERR_OK,
	WRONG_HEADER,
	WRONG_PACK_ID,
	WRONG_CHKSUM
}usartErrT;


#define CMD_BL_MODE_BY_ADDR			0x09
#define CMD_BACKLIGHT_MODE			0x0A //AUTO / MANUAL
#define BL_MODE_AUTO				0x01
#define BL_MODE_ADAPTIVE			0x02
#define BL_MODE_MAN					0x00

#define CMD_SET_BRIGHTNESS			0x0B
#define CMD_SET_BR_BY_ADDR			0x0C

#define CMD_ENABLE_DRIVERS			0x0D
#define CMD_SET_DIMMING_TIME		0x0E
#define CMD_START_IMG 				0x0F
#define CMD_STOP_IMG				0x10
#define CMD_FX				    	0x13
//#define CMD_SET_FAN_MODE			20
//#define	CMD_SET_FAN_SPEED			21
#define CMD_PWR_OFF					0x11
#define CMD_CV_REQUEST				0x12
#define CMD_CHARGER_OFF				0x14
#define CMD_BACKLIGHT_OFF			0x15
#define CMD_ZERO_OFF				0x16
#define CMD_CHARGER_ON				0x17
#define CMD_BACKLIGHT_ON			0x18
#define CMD_ZERO_ON					0x19
#define CMD_LIGHT_TABLE				0x1A

#define LED_DRIVER_0_ADDR			0x00
#define LED_DRIVER_1_ADDR			0x01
#define LED_DRIVER_2_ADDR			0x02
#define LED_DRIVER_3_ADDR			0x03
#define BROADCAST_ADDR				0xFF

#define RESPONSE_OK					0x00
#define RESPONSE_ERROR				0x01


extern uint8_t autoBacklightflags[4];
extern uint8_t brightnessValues[4];
extern uint8_t dimmingTime;



void uartCommTask(void const * argument);
void USER_UART_IRQHandler (UART_HandleTypeDef *huart);
uint8_t get_check_sum(uint8_t *source, uint8_t size);
void sendRespToRasp(uint8_t cmd, uint8_t response);


#endif /* INC_USART_H_ */
