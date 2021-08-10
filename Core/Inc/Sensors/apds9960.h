#ifndef __APDS9960_H
#define __APDS9960_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
/* Debug */
#define DEBUG_APDS                   0

extern I2C_HandleTypeDef hi2c1;

extern uint8_t APDS9960_Init(void);
extern uint8_t APDS9960_SetActiveChan(uint8_t n_apds9960);
extern uint8_t APDS9960_ReadLight(uint8_t nSensor, uint8_t *dest);
extern uint8_t APDS9960_ReadRedLight(uint8_t nSensor, uint8_t *dest);
extern uint8_t APDS9960_ReadGreenLight(uint8_t nSensor, uint8_t *dest);
extern uint8_t APDS9960_ReadBlueLight(uint8_t nSensor, uint8_t *dest);
//extern uint8_t APDS9960_ReadClrTemp(uint8_t nSensor, uint8_t nClr);

#endif


