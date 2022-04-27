#include "dht22.h"
#include <stdlib.h>
#include <math.h>

#define STATUS_OK					0x01
#define STATUS_FAIL					0x00

/* Ports and pins with DHT22 sensor*/
#define DHT22_GPIO_PORT_1			GPIOB
#define DHT22_GPIO_PIN_1			GPIO_PIN_3

#define DHT22_GPIO_PORT_2			GPIOC
#define DHT22_GPIO_PIN_2			GPIO_PIN_10

#define DHT22_GPIO_PORT_3			GPIOC
#define DHT22_GPIO_PIN_3			GPIO_PIN_11

#define DHT22_MAX_DELAY				100
#define DHT22_MIN_DELAY_H			20

/* DHT22_GetReadings response codes */
#define DHT22_RCV_OK				0 // Return with no error
#define DHT22_RCV_NO_RESPONSE		1 // No response from sensor
#define DHT22_RCV_BAD_ACK1			2 // Bad first half length of ACK impulse
#define DHT22_RCV_BAD_ACK2			3 // Bad second half length of ACK impulse
#define DHT22_RCV_TIMEOUT			4	// Timeout while receiving bits
#define DHT22_BAD_DATA				5 // Bad data received

GPIO_InitTypeDef DHT22_GPIO_Struct = { 0 };
extern TIM_HandleTypeDef htim7;
dht22_data dht22_buf;
uint32_t dht_pins[3] = { DHT22_GPIO_PIN_1, DHT22_GPIO_PIN_2, DHT22_GPIO_PIN_3 };
GPIO_TypeDef *dht_ports[3] = { DHT22_GPIO_PORT_1, DHT22_GPIO_PORT_2, DHT22_GPIO_PORT_3 };
//dht22_data *dht22_buf = &dht22_dat;
//uint16_t c;

static uint8_t DHT22_GetReadings(uint8_t nSensor);
static void DHT22_DecodeReadings(void);

void DHT22_Init(uint8_t nSensor) {
	/*Configure GPIO pins : PCPin PCPin PCPin PCPin */
	DHT22_GPIO_Struct.Pin = dht_pins[nSensor];
	DHT22_GPIO_Struct.Mode = GPIO_MODE_OUTPUT_OD;
	DHT22_GPIO_Struct.Pull = GPIO_PULLDOWN;
	DHT22_GPIO_Struct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(dht_ports[nSensor], &DHT22_GPIO_Struct);
	dht_ports[nSensor]->BSRR = dht_pins[nSensor] << 16U; // Pull down SDA (Bit_SET)
	osDelay(10); // Host start signal at least 1ms
	dht_ports[nSensor]->BSRR = dht_pins[nSensor]; // Release SDA (Bit_RESET)


}

uint8_t DHT22_Read(uint8_t nSensor) {
	uint16_t tmp_temp, tmp_humid;

	if (DHT22_GetReadings(nSensor) == STATUS_FAIL) {

		return STATUS_FAIL;
	}

	DHT22_DecodeReadings();

	if (dht22_buf.parity != dht22_buf.parity_rcv) {
		dht22_buf.rcv_response = DHT22_BAD_DATA;
		return STATUS_FAIL;
	}

	tmp_humid = (dht22_buf.hMSB << 8) | dht22_buf.hLSB;
	tmp_temp = (dht22_buf.tMSB << 8) | dht22_buf.tLSB;

	dht22_buf.humidity = (float) tmp_humid / 10.0f;
	dht22_buf.temperature = ((float) (tmp_temp & 0x7fff)) / 10.0f;

//	if (tmp_temp & 0x8000) {
//		dht22_buf.temperature = -dht22_buf.temperature;
//	}


	return STATUS_OK;
}


void Delay_us(uint16_t nTime) {
	TIM7->CNT = 0;

	while (TIM7->CNT < nTime)
		;
}

static uint8_t DHT22_GetReadings(uint8_t nSensor) {
	uint8_t i;
	uint16_t c;

	dht22_buf.rcv_response = DHT22_RCV_OK;

	// Switch pin to output
	DHT22_GPIO_Struct.Mode = GPIO_MODE_OUTPUT_OD;
	HAL_GPIO_Init(dht_ports[nSensor], &DHT22_GPIO_Struct);

	dht_ports[nSensor]->BSRR = dht_pins[nSensor]; // Release SDA (Bit_RESET)
	TIM7->CNT = 0;
	while (((c = TIM7->CNT) < DHT22_MAX_DELAY)
			&& (dht_ports[nSensor]->IDR & dht_pins[nSensor]))
		;

	// Generate start impulse
	dht_ports[nSensor]->BSRR = dht_pins[nSensor] << 16U; // Pull down SDA (Bit_SET)
	osDelay(2); // Host start signal at least 1ms
	dht_ports[nSensor]->BSRR = dht_pins[nSensor]; // Release SDA (Bit_RESET)

	// Switch pin to input with Pull-Up
	DHT22_GPIO_Struct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(dht_ports[nSensor], &DHT22_GPIO_Struct);

	// Wait for AM2302 to begin communication (20-40us)
	TIM7->CNT = 0;
	while (((c = TIM7->CNT) < DHT22_MAX_DELAY)
			&& (dht_ports[nSensor]->IDR & dht_pins[nSensor]))
		;

	TIM7->CNT = 0;

	if (c >= DHT22_MAX_DELAY) {
		dht22_buf.rcv_response = DHT22_RCV_NO_RESPONSE;
		return STATUS_FAIL;
	}

	// Check ACK strobe from sensor
	while (((c = TIM7->CNT) < DHT22_MAX_DELAY)
			&& !(dht_ports[nSensor]->IDR & dht_pins[nSensor]))
		;

	TIM7->CNT = 0;

	if ((c < DHT22_MIN_DELAY_H) || (c > DHT22_MAX_DELAY - 5)) {
		dht22_buf.rcv_response = DHT22_RCV_BAD_ACK1;
		return STATUS_FAIL;
	}

	while (((c = TIM7->CNT) < DHT22_MAX_DELAY)
			&& (dht_ports[nSensor]->IDR & dht_pins[nSensor]))
		;

	TIM7->CNT = 0;

	if ((c < DHT22_MIN_DELAY_H) || (c > DHT22_MAX_DELAY - 5)) {
		dht22_buf.rcv_response = DHT22_RCV_BAD_ACK2;
		return STATUS_FAIL;
	}

	// ACK strobe received --> receive 40 bits
	i = 0;

	while (i < 40) {
		dht22_buf.bits[i] = 0;

		// Measure bit start impulse (T_low = 50us)
		while (((c = TIM7->CNT) < DHT22_MAX_DELAY)
				&& !(dht_ports[nSensor]->IDR & dht_pins[nSensor]))
			;

		if (c > 75) {
			// invalid bit start impulse length
			dht22_buf.bits[i] = 0xff;
			while (((c = TIM7->CNT) < DHT22_MAX_DELAY)
					&& (dht_ports[nSensor]->IDR & dht_pins[nSensor]))
				;

			TIM7->CNT = 0;
		}

		else

		{
			// Measure bit impulse length (T_h0 = 25us, T_h1 = 70us)
			TIM7->CNT = 0;
			while (((c = TIM7->CNT) < DHT22_MAX_DELAY)
					&& (dht_ports[nSensor]->IDR & dht_pins[nSensor]))
				;

			TIM7->CNT = 0;
			dht22_buf.bits[i] = (c < DHT22_MAX_DELAY) ? (uint8_t) c : 0xff;
		}
		i++;
	}

	for (i = 0; i < 40; i++) {
		if (dht22_buf.bits[i] == 0xff) {
			dht22_buf.rcv_response = DHT22_RCV_TIMEOUT;
			return STATUS_FAIL;
		}
	}

	return STATUS_OK;
}

static void DHT22_DecodeReadings(void) {
	uint8_t i = 0;

	dht22_buf.hMSB = 0;
	dht22_buf.hLSB = 0;
	dht22_buf.tMSB = 0;
	dht22_buf.tLSB = 0;
	dht22_buf.parity = 0;
	dht22_buf.parity_rcv = 0;

	for (; i < 8; i++) {
		dht22_buf.hMSB <<= 1;
		if (dht22_buf.bits[i] > DHT22_MIN_DELAY_H)
			dht22_buf.hMSB |= 1;
	}

	for (; i < 16; i++) {
		dht22_buf.hLSB <<= 1;
		if (dht22_buf.bits[i] > DHT22_MIN_DELAY_H)
			dht22_buf.hLSB |= 1;
	}

	for (; i < 24; i++) {
		dht22_buf.tMSB <<= 1;
		if (dht22_buf.bits[i] > DHT22_MIN_DELAY_H)
			dht22_buf.tMSB |= 1;
	}

	for (; i < 32; i++) {
		dht22_buf.tLSB <<= 1;
		if (dht22_buf.bits[i] > DHT22_MIN_DELAY_H)
			dht22_buf.tLSB |= 1;
	}

	for (; i < 40; i++) {
		dht22_buf.parity_rcv <<= 1;
		if (dht22_buf.bits[i] > DHT22_MIN_DELAY_H)
			dht22_buf.parity_rcv |= 1;
	}

	dht22_buf.parity = dht22_buf.hMSB + dht22_buf.hLSB + dht22_buf.tMSB
			+ dht22_buf.tLSB;

}
