#ifndef __DHT22_H
#define __DHT22_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
/* Debug */

typedef struct dht22_data
{
  volatile uint8_t rcv_response;
  volatile float temperature;
  volatile float humidity;
  uint8_t parity;
  uint8_t parity_rcv;
  uint8_t hMSB;
  uint8_t hLSB;
  uint8_t tMSB;
  uint8_t tLSB;
  uint8_t bits[40];
} dht22_data;

extern dht22_data dht22_buf;

extern void DHT22_Init(uint8_t nSensor);
extern uint8_t DHT22_Read(uint8_t nSensor);

#endif


