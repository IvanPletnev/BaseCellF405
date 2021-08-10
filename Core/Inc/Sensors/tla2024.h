#ifndef __TLA2024_H
#define __TLA2024_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Debug */

extern I2C_HandleTypeDef hi2c2;
extern void TLA2024_Init(void);
uint8_t TLA2024_Read(uint8_t nSensor, uint8_t *dest);

#endif


