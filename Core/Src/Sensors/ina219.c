/**************************************************************************/
/*!
 @file     Adafruit_INA219.cpp
 @author   K.Townsend (Adafruit Industries)
 @license  BSD (see license.txt)

 Driver for the INA219 current sensor

 This is a library for the Adafruit INA219 breakout
 ----> https://www.adafruit.com/products/???

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 @section  HISTORY

 v1.0 - First release
 v2.0  - Ported to STM32 for ChibiOS by William Garrido
 */
/**************************************************************************/

#include "ina219.h"
#include "cmsis_os.h"
#include "apds9960.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

#define INA_DRIVER  	&hi2c2


uint32_t ina219CurrentDivider_mA;
uint32_t ina219PowerDivider_mW;
static uint8_t sensors[3] = { INA219_ADDRESS_2, INA219_ADDRESS_3, INA219_ADDRESS_1};
int16_t rawVoltage[3];
int16_t rawCurrent[3];
float fVoltage[3];
float fCurrent[3];
float shuntVoltage[3];


/**************************************************************************/
/*!
 @brief  INA219B Init
 */
/**************************************************************************/
void ina219Init(void) {
	ina219CurrentDivider_mA = 0;
	ina219PowerDivider_mW = 0;
	ina219SetCalibration_16V_80A_05mOhm(1);
	osDelay(10);
	ina219SetCalibration_16V_80A(2);
	osDelay(10);
	ina219SetCalibration_16V_80A_075mOhm(0);
	osDelay(10);
}

/**************************************************************************/
/*!
 @brief  Sends a single command byte over I2C
 */
/**************************************************************************/
uint8_t ina219WriteRegister(uint8_t nSensor, uint8_t reg, uint16_t value) {
	uint8_t txbuf[3];
	txbuf[0] = reg;
	txbuf[1] = ((value >> 8) & 0xFF);
	txbuf[2] = (value & 0xFF);

	if (HAL_I2C_Master_Transmit(INA_DRIVER, sensors[nSensor]<<1, txbuf, 3, 10) != HAL_OK ){
		return STATUS_FAIL;
	}
	return STATUS_OK;
}

/**************************************************************************/
/*!
 @brief  Reads a 16 bit values over I2C
 */
/**************************************************************************/
uint8_t ina219ReadRegister(uint8_t nSensor, uint8_t reg, uint16_t *value) {
	uint8_t rxbuf[2], txbuf[2];
	rxbuf[0] = 0;
	rxbuf[1] = 0;
	txbuf[0] = reg;
	txbuf[1] = 0;
	if (HAL_I2C_Master_Transmit(INA_DRIVER, sensors[nSensor]<<1, txbuf, 1, 10) != HAL_OK){
		return STATUS_FAIL;
	}
	if (HAL_I2C_Master_Receive(INA_DRIVER, sensors[nSensor]<<1, rxbuf, 2, 10) != HAL_OK) {
		return STATUS_FAIL;
	}
	while (HAL_I2C_GetState(INA_DRIVER) != HAL_I2C_STATE_READY){
	}

	*value = (((uint16_t) rxbuf[0]) << 8) | rxbuf[1];
	return STATUS_OK;
}

/**************************************************************************/
/*!
 @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
 */
/**************************************************************************/
int16_t ina219GetBusVoltage_raw(uint8_t nSensor) {
	uint16_t value;
	ina219ReadRegister(nSensor, INA219_REG_BUSVOLTAGE, &value);

	// Shift to the right 3 to drop CNVR and OVF and multiply by LSB
	return (int16_t) ((value >> 3) * 4 * 0.1003);
}
/**************************************************************************/
/*!
 @brief  Gets the shunt voltage in volts
 */
/**************************************************************************/
float ina219GetBusVoltage_V(uint8_t nSensor) {
	int16_t value = ina219GetBusVoltage_raw(nSensor);
	return value * 0.001003;
}
/**************************************************************************/
/*!
 @brief  Configures to INA219 to be able to measure up to 32V and 2A
 of current.  Each unit of current corresponds to 100uA, and
 each unit of power corresponds to 2mW. Counter overflow
 occurs at 3.2A.

 @note   These calculations assume a 0.1 ohm resistor is present
 */
/**************************************************************************/
void ina219SetCalibration_32V_2A(uint8_t nSensor) {
	ina219CurrentDivider_mA = 10000; // Current LSB = 100uA per bit (1000/100 = 10)
	ina219PowerDivider_mW = 2;     // Power LSB = 1mW per bit (2/1)

	// Set Calibration register to 'Cal' calculated above
	ina219WriteRegister(nSensor, INA219_REG_CALIBRATION, 0x1000);

	// Set Config register to take into account the settings above
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
	INA219_CONFIG_GAIN_8_320MV |
	INA219_CONFIG_BADCRES_12BIT |
	INA219_CONFIG_SADCRES_12BIT_8S_4260US |
	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	ina219WriteRegister(nSensor,INA219_REG_CONFIG, config);
}

void ina219SetCalibration_16V_80A (uint8_t nSensor){
	ina219CurrentDivider_mA = 100; // Current LSB = 100uA per bit (1000/100 = 10)
	ina219PowerDivider_mW = 2;     // Power LSB = 1mW per bit (2/1)

	// Set Calibration register to 'Cal' calculated above
	ina219WriteRegister(nSensor, INA219_REG_CALIBRATION, 0x1000);

	// Set Config register to take into account the settings above
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
	INA219_CONFIG_GAIN_2_80MV |
	INA219_CONFIG_BADCRES_12BIT |
	INA219_CONFIG_SADCRES_12BIT_16S_8510US |
	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	ina219WriteRegister(nSensor,INA219_REG_CONFIG, config);
}

void ina219SetCalibration_16V_80A_05mOhm (uint8_t nSensor){
	ina219CurrentDivider_mA = 100; // Current LSB = 100uA per bit (1000/100 = 10)
	ina219PowerDivider_mW = 2;     // Power LSB = 1mW per bit (2/1)

	// Set Calibration register to 'Cal' calculated above
	ina219WriteRegister(nSensor, INA219_REG_CALIBRATION, 0x2000);

	// Set Config register to take into account the settings above
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
	INA219_CONFIG_GAIN_2_80MV |
	INA219_CONFIG_BADCRES_12BIT |
	INA219_CONFIG_SADCRES_12BIT_16S_8510US |
	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	ina219WriteRegister(nSensor,INA219_REG_CONFIG, config);
}

void ina219SetCalibration_16V_80A_075mOhm (uint8_t nSensor){
	ina219CurrentDivider_mA = 100; // Current LSB = 100uA per bit (1000/100 = 10)
	ina219PowerDivider_mW = 2;     // Power LSB = 1mW per bit (2/1)

	// Set Calibration register to 'Cal' calculated above
	ina219WriteRegister(nSensor, INA219_REG_CALIBRATION, 0x1555);

	// Set Config register to take into account the settings above
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
	INA219_CONFIG_GAIN_2_80MV |
	INA219_CONFIG_BADCRES_12BIT |
	INA219_CONFIG_SADCRES_12BIT_16S_8510US |
	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	ina219WriteRegister(nSensor,INA219_REG_CONFIG, config);
}



/**************************************************************************/
/*!
 @brief  Configures to INA219 to be able to measure up to 32V and 1A
 of current.  Each unit of current corresponds to 40uA, and each
 unit of power corresponds to 800uW. Counter overflow occurs at
 1.3A.

 @note   These calculations assume a 0.1 ohm resistor is present
 */
/**************************************************************************/
void ina219SetCalibration_32V_1A(uint8_t nSensor) {
	// Set multipliers to convert raw current/power values
	ina219CurrentDivider_mA = 25;   // Current LSB = 40uA per bit (1000/40 = 25)
	ina219PowerDivider_mW = 1;         // Power LSB = 800uW per bit

	// Set Calibration register to 'Cal' calculated above
	ina219WriteRegister(nSensor, INA219_REG_CALIBRATION, 0x2800);

	// Set Config register to take into account the settings above
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
	INA219_CONFIG_GAIN_8_320MV |
	INA219_CONFIG_BADCRES_12BIT |
	INA219_CONFIG_SADCRES_12BIT_1S_532US |
	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	ina219WriteRegister(nSensor, INA219_REG_CONFIG, config);
}

//Testing lower current measurements
//http://cpre.kmutnb.ac.th/esl/learning/ina219b-current-sensor/ina219_sensor_demo.ino
void ina219SetCalibration_16V_400mA(uint8_t nSensor) {
	// Set multipliers to convert raw current/power values
	ina219CurrentDivider_mA = 20;   // Current LSB = 40uA per bit (1000/40 = 25)
	ina219PowerDivider_mW = 1;         // Power LSB = 800uW per bit

	// Set Calibration register to 'Cal' calculated above
	ina219WriteRegister(nSensor, INA219_REG_CALIBRATION, 8192);

	// Set Config register to take into account the settings above
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
	INA219_CONFIG_GAIN_1_40MV |
	INA219_CONFIG_BADCRES_12BIT |
	INA219_CONFIG_SADCRES_12BIT_1S_532US |
	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	ina219WriteRegister(nSensor, INA219_REG_CONFIG, config);
}

/**************************************************************************/
/*!
 @brief  Gets the raw current value (16-bit signed integer, so +-32767)
 */
/**************************************************************************/
int16_t ina219GetCurrent_raw(uint8_t nSensor) {
	uint16_t value;
	ina219ReadRegister(nSensor, INA219_REG_CURRENT, &value);
	return (int16_t) value;
}

/**************************************************************************/
/*!
 @brief  Gets the current value in mA, taking into account the
 config settings and current LSB
 */
/**************************************************************************/
float ina219GetCurrent_mA(uint8_t nSensor, uint16_t offset) {
	float valueDec = ina219GetCurrent_raw(nSensor) - offset; //
	valueDec /= ina219CurrentDivider_mA;
//	valueDec *=0.8189845;
	return valueDec;
}

/**************************************************************************/
/*!
 @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
 */
/**************************************************************************/
int16_t ina219GetShuntVoltage_raw(uint8_t nSensor) {
	uint16_t value;
	ina219ReadRegister(nSensor, INA219_REG_SHUNTVOLTAGE, &value);
	return (int16_t) value;
}

/**************************************************************************/
/*!
 @brief  Gets the shunt voltage in mV (so +-327mV)
 */
/**************************************************************************/
float ina219GetShuntVoltage_mV(uint8_t nSensor) {
	int16_t value;
	value = ina219GetShuntVoltage_raw(nSensor);
	return value * 0.01;
}

