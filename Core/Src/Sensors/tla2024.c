#include "tla2024.h"
#include <stdlib.h>
#include <math.h>
#include "stdio.h"
#include "cmsis_os.h"



#define STATUS_OK							0x01
#define STATUS_FAIL							0x00

#define TLA2024_ADDRESS_1					0x48	//pull down
#define TLA2024_ADDRESS_2					0x49	//pull up
#define TLA2024_ADDRESS_3					0x4B	//pull scl
#define pulse 								49 		//50ms
#define TLA2024_CONV_RETRY					10

#define TLA2024_REG_DATA_POINT				0x00
#define TLA2024_REG_CONF_POINT				0x01

#define MEASURER_mode_not_active			0x00
#define MEASURER_mode_temperature			0x01
#define MEASURER_mode_voltage				0x02
#define MEASURER_mode_current_left			0x03
#define MEASURER_mode_current_right			0x04

#define TLA2024_state_not_active			0x00
#define TLA2024_state_write_conf			0x01
#define TLA2024_state_wait_os				0x02
#define TLA2024_state_set_point				0x03
#define TLA2024_state_read_data				0x04

#define TLA2024_timeout_k					2
#define TLA2024_CONF 						0x01

#define TLA2024_attempt						3

#define TEMP_TAB_COUNT						30


enum TLA2024_MUX {
	TLA2024_AIN0_AIN1	= 0x00,
	TLA2024_AIN0_AIN3   = 0x01,
	TLA2024_AIN1_AIN3   = 0x02,
	TLA2024_AIN2_AIN3   = 0x03,
	TLA2024_AIN0_GND	= 0x04,
	TLA2024_AIN1_GND	= 0x05,
	TLA2024_AIN2_GND	= 0x06,
	TLA2024_AIN3_GND	= 0x07
};	 
	 
enum TLA2024_DR {
	TLA2024_DR_128		= 0x00,
	TLA2024_DR_250		= 0x01,
	TLA2024_DR_490		= 0x02,
	TLA2024_DR_920		= 0x03,
	TLA2024_DR_1600		= 0x04,	//default
	TLA2024_DR_2400		= 0x05,
	TLA2024_DR_3300		= 0x06,
	TLA2024_DR_max		= 0x07
};	 

enum TLA2024_PGA {
	TLA2024_PGA_6144	= 0x00,
	TLA2024_PGA_4096	= 0x01, //максимальное напряжение, которое мы можем померять этим АЦП. 4096мВ.
	TLA2024_PGA_2048	= 0x02,	//default
	TLA2024_PGA_1024	= 0x03,
	TLA2024_PGA_512 	= 0x04,
	TLA2024_PGA_256		= 0x05,
	TLA2024_PGA_256_	= 0x06,
	TLA2024_PGA_256__	= 0x07
};	 

typedef struct 
{
	int	mask;
	uint8_t	shift;
} TLA2024_reg;

TLA2024_reg TLA2024_CONF_OS 	= {0x8000, 16};
TLA2024_reg TLA2024_CONF_MUX 	= {0x7000, 12};
TLA2024_reg TLA2024_CONF_PGA 	= {0x0c00, 9};
TLA2024_reg TLA2024_CONF_MODE	= {0x0100, 8};
TLA2024_reg TLA2024_CONF_DR		= {0x01c0, 5};
TLA2024_reg TLA2024_DATA_REG	= {0xf000, 4};


uint8_t data_buf[2];
uint8_t send_buf1[3], send_buf2[3], send_buf3[3];
uint8_t * sensors_send_buf[] = {send_buf1, send_buf2, send_buf3};
uint8_t sensors[3] = {TLA2024_ADDRESS_2, TLA2024_ADDRESS_1, TLA2024_ADDRESS_3};
uint8_t counter_try = 0;


float tempVar;


void TLA2024_Init(void);
void init_send_buffer(uint8_t nSensor, uint8_t mux, uint8_t dr, uint8_t pga);
uint8_t TLA2024_single_shot_conv_start(uint8_t nSensor);
uint8_t TLA2024_single_shot_conv_check_os(uint8_t nSensor);
uint8_t TLA2024_single_shot_conv_set_point(uint8_t nSensor, uint8_t point_reg);
uint8_t TLA2024_single_shot_conv_read_data(uint8_t nSensor);

typedef struct _point{
	int x;
	int y;
}tPoint;
	
tPoint line[TEMP_TAB_COUNT] = {// x - температура, y - значение АЦП, умноженное на 10
		{-400, 12410},
		{-350, 12380},
		{-300, 12340},
		{-250, 12280},
		{-200, 12210},
		{-150, 12110},
		{-100, 11990},
		{-50, 11850},
		{0, 11660},
		{50, 11450},
		{100, 11180},
		{150, 10880},
		{200, 10530},
		{250, 10130},
		{300, 9690},
		{350, 9200},
		{400, 8680},
		{450, 8140},
		{500, 7580},
		{550, 7010},
		{600, 6440},
		{650, 5890},
		{700, 5350},
		{750, 4840},
		{800, 4370},
		{850, 3930},
		{900, 3520},
		{950, 3150},
		{1000, 2810},
		{1100, 2480}
};




int16_t getTemp (uint16_t adc){ //Метод кусочно-линейной аппроксимации

	uint8_t i=0;
	uint16_t adc10;
	float k; //наклон характеристики
	float c; //смещение относительно 0 по Y
	float value;
	adc10 = adc*10; //умножаем код АЦП на 10, в таблице также значения, умноженные на 10

	for (i = 0; i < TEMP_TAB_COUNT-1; i++){
		if ((adc10 <= line[i].y)&&(adc10 >= line[i+1].y)) {
			k = ((float)line[i].x - (float)line[i+1].x)/((float)line[i].y - (float)line[i+1].y);
			c = (float)line[i].x - k*(float)line[i].y;
			value = k * adc10 + c; //уравнение текущего (i) отрезка
//			if (value >= 0){
//				return (uint16_t) value;
//			} else {
//				return ((uint16_t) value) | 0x8000;//Добавляем признак отрицательного числа
//			}
			return (int16_t) value;
		}
	}
	return 0;
}

void TLA2024_Init() 
{
	init_send_buffer(0, TLA2024_AIN0_GND, TLA2024_DR_3300, TLA2024_PGA_4096);
	init_send_buffer(1, TLA2024_AIN0_GND, TLA2024_DR_3300, TLA2024_PGA_4096);
	init_send_buffer(2, TLA2024_AIN0_GND, TLA2024_DR_3300, TLA2024_PGA_4096);
}

uint8_t TLA2024_Read(uint8_t nSensor, uint8_t *dest)
{
	int16_t temperature;
	uint16_t tmp_val;

	if (TLA2024_single_shot_conv_start(nSensor) == STATUS_FAIL)
		return STATUS_FAIL;
	
	if (TLA2024_single_shot_conv_check_os(nSensor) == STATUS_FAIL)
		return STATUS_FAIL;
		
	if (TLA2024_single_shot_conv_set_point(nSensor, 0x00) == STATUS_FAIL)
		return STATUS_FAIL;
	
	if (TLA2024_single_shot_conv_read_data(nSensor) == STATUS_FAIL)
		return STATUS_FAIL;
	
	tmp_val = ((uint16_t) data_buf[0]) << 4;
	tmp_val += (uint16_t) data_buf[1] >> 4;

	temperature = getTemp(tmp_val);

	dest[0]	= (uint8_t) (temperature >> 8);
	dest[1]	= (uint8_t) (temperature & 0xFF);

	return STATUS_OK;
}

void init_send_buffer(uint8_t nSensor, uint8_t mux, uint8_t dr, uint8_t pga)
{
	int tmp_buf = TLA2024_CONF_OS.mask | TLA2024_CONF_MODE.mask;
	
	tmp_buf |= (mux << TLA2024_CONF_MUX.shift);
	tmp_buf |= (dr 	<< TLA2024_CONF_DR.shift);
	tmp_buf |= (pga << TLA2024_CONF_PGA.shift);
	
	sensors_send_buf[nSensor][0] = 0x01;
	sensors_send_buf[nSensor][1] = (uint8_t) (tmp_buf >> 8); 
	sensors_send_buf[nSensor][2] = (uint8_t) (tmp_buf & 0xFF); 	
}

uint8_t TLA2024_single_shot_conv_start(uint8_t nSensor)
{
	if (HAL_I2C_Master_Transmit(&hi2c2, sensors[nSensor]<<1 , sensors_send_buf[nSensor], 3, 10) == HAL_BUSY)
	{
		HAL_I2C_MspInit(&hi2c2);
		return STATUS_FAIL;
	}

	osDelay(10);
	return STATUS_OK;
}

uint8_t TLA2024_single_shot_conv_check_os(uint8_t nSensor)
{
	for (uint8_t i=0; i < TLA2024_attempt; i++) 
	{
		data_buf[0] = 0;
		data_buf[1] = 0;
		HAL_I2C_Master_Receive(&hi2c2, sensors[nSensor]<<1, data_buf, 2, 10);
		if ((data_buf[0] & (TLA2024_CONF_OS.mask >> 8)) == (TLA2024_CONF_OS.mask >> 8))
		{
			return STATUS_OK;
		}
	}
	
	return STATUS_FAIL;
}

uint8_t TLA2024_single_shot_conv_set_point(uint8_t nSensor, uint8_t point_reg)
{
	data_buf[0] = point_reg;
	data_buf[1] = 0;
	if (HAL_I2C_Master_Transmit(&hi2c2, sensors[nSensor]<<1, data_buf, 1, 10) == HAL_BUSY)
	{
		return STATUS_FAIL;
	}


	return STATUS_OK;
}

uint8_t TLA2024_single_shot_conv_read_data(uint8_t nSensor)
{

		data_buf[0] = 0;
		data_buf[1] = 0;
		HAL_I2C_Master_Receive(&hi2c2, sensors[nSensor]<<1, data_buf, 2, 10);
		if (data_buf[0]||data_buf[0])
		{
			return STATUS_OK;	
		}
		
	return STATUS_FAIL;
}

