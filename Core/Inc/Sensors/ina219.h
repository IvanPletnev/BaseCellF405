/**************************************************************************/
/*!
 @file     INA219B.h
 @author   William Garrido
 @license  BSD (see license.txt)

 This is a library for the ChiBiOS running on the USB Tester STM32
 ----> https://friedcircuits.us

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 @section  HISTORY

 v1.0  - First release
 */
#include "main.h"
/**************************************************************************/

/*=========================================================================
 I2C ADDRESS/BITS
 -----------------------------------------------------------------------*/
#define INA219_ADDRESS_1                       	(0x40)    // 1000000 (A0+A1=GND)
#define INA219_ADDRESS_2						(0x41)
#define INA219_ADDRESS_3						(0x42)
#define INA219_READ                            	(0x01)
/*=========================================================================*/

/*=========================================================================
 CONFIG REGISTER (R/W)
 -----------------------------------------------------------------------*/
#define INA219_REG_CONFIG                      	(0x00)
/*---------------------------------------------------------------------*/
#define INA219_CONFIG_RESET                    	(0x8000)  // Reset Bit

#define INA219_CONFIG_BVOLTAGERANGE_MASK       	(0x2000)  // Bus Voltage Range Mask
#define INA219_CONFIG_BVOLTAGERANGE_16V       	(0x0000)  // 0-16V Range
#define INA219_CONFIG_BVOLTAGERANGE_32V        	(0x2000)  // 0-32V Range

#define INA219_CONFIG_GAIN_MASK                	(0x1800)  // Gain Mask
#define INA219_CONFIG_GAIN_1_40MV              	(0x0000)  // Gain 1, 40mV Range
#define INA219_CONFIG_GAIN_2_80MV              	(0x0800)  // Gain 2, 80mV Range
#define INA219_CONFIG_GAIN_4_160MV             	(0x1000)  // Gain 4, 160mV Range
#define INA219_CONFIG_GAIN_8_320MV             	(0x1800)  // Gain 8, 320mV Range

#define INA219_CONFIG_BADCRES_MASK             	(0x0780)  // Bus ADC Resolution Mask
#define INA219_CONFIG_BADCRES_9BIT             	(0x0080)  // 9-bit bus res = 0..511
#define INA219_CONFIG_BADCRES_10BIT            	(0x0100)  // 10-bit bus res = 0..1023
#define INA219_CONFIG_BADCRES_11BIT            	(0x0200)  // 11-bit bus res = 0..2047
#define INA219_CONFIG_BADCRES_12BIT            	(0x0400)  // 12-bit bus res = 0..4097

#define INA219_CONFIG_SADCRES_MASK             	(0x0078)  // Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_SADCRES_9BIT_1S_84US     	(0x0000)  // 1 x 9-bit shunt sample
#define INA219_CONFIG_SADCRES_10BIT_1S_148US   	(0x0008)  // 1 x 10-bit shunt sample
#define INA219_CONFIG_SADCRES_11BIT_1S_276US   	(0x0010)  // 1 x 11-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_1S_532US   	(0x0018)  // 1 x 12-bit shunt sample
#define INA219_CONFIG_SADCRES_12BIT_2S_1060US  	(0x0048)	 // 2 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_4S_2130US  	(0x0050)  // 4 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_8S_4260US  	(0x0058)  // 8 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_16S_8510US 	(0x0060)  // 16 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_32S_17MS   	(0x0068)  // 32 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_64S_34MS   	(0x0070)  // 64 x 12-bit shunt samples averaged together
#define INA219_CONFIG_SADCRES_12BIT_128S_69MS  	(0x0078)  // 128 x 12-bit shunt samples averaged together

#define INA219_CONFIG_MODE_MASK                	(0x0007)  // Operating Mode Mask
#define INA219_CONFIG_MODE_POWERDOWN           	(0x0000)
#define INA219_CONFIG_MODE_SVOLT_TRIGGERED     	(0x0001)
#define INA219_CONFIG_MODE_BVOLT_TRIGGERED     	(0x0002)
#define INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED 	(0x0003)
#define INA219_CONFIG_MODE_ADCOFF              	(0x0004)
#define INA219_CONFIG_MODE_SVOLT_CONTINUOUS    	(0x0005)
#define INA219_CONFIG_MODE_BVOLT_CONTINUOUS    	(0x0006)
#define INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS (0x0007)
/*=========================================================================*/

/*=========================================================================
 SHUNT VOLTAGE REGISTER (R)
 -----------------------------------------------------------------------*/
#define INA219_REG_SHUNTVOLTAGE                	(0x01)
/*=========================================================================*/

/*=========================================================================
 BUS VOLTAGE REGISTER (R)
 -----------------------------------------------------------------------*/
#define INA219_REG_BUSVOLTAGE                  	(0x02)
/*=========================================================================*/

/*=========================================================================
 POWER REGISTER (R)
 -----------------------------------------------------------------------*/
#define INA219_REG_POWER                       	(0x03)
/*=========================================================================*/

/*=========================================================================
 CURRENT REGISTER (R)
 -----------------------------------------------------------------------*/
#define INA219_REG_CURRENT                     	(0x04)
/*=========================================================================*/

/*=========================================================================
 CALIBRATION REGISTER (R/W)
 -----------------------------------------------------------------------*/
#define INA219_REG_CALIBRATION                 	(0x05)
/*=========================================================================*/

/*=========================================================================
 Maximum timeout
 -----------------------------------------------------------------------*/
#define INA219_TIMEOUT                			(0x3000)
/*=========================================================================*/


extern int16_t rawVoltage[3];
extern int16_t rawCurrent[3];
extern float fVoltage[3];
extern float fCurrent[3];
extern float shuntVoltage[3];
void ina219Init(void);
uint8_t ina219WriteRegister(uint8_t nSensor, uint8_t reg, uint16_t value);
uint8_t ina219ReadRegister(uint8_t nSensor, uint8_t reg, uint16_t *value);
int16_t ina219GetBusVoltage_raw(uint8_t nSensor);
float ina219GetBusVoltage_V(uint8_t nSensor);
void ina219SetCalibration_32V_2A(uint8_t nSensor);
void ina219SetCalibration_16V_80A (uint8_t nSensor);
void ina219SetCalibration_16V_80A_05mOhm (uint8_t nSensor);
void ina219SetCalibration_16V_80A_075mOhm (uint8_t nSensor);
void ina219SetCalibration_32V_1A(uint8_t nSensor);
void ina219SetCalibration_16V_400mA(uint8_t nSensor);
int16_t ina219GetCurrent_raw(uint8_t nSensor);
float ina219GetCurrent_mA(uint8_t nSensor, uint16_t offset);
int16_t ina219GetShuntVoltage_raw(uint8_t nSensor);
float ina219GetShuntVoltage_mV(uint8_t nSensor);

