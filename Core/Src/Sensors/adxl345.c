#include "adxl345.h"
#include "stm32f4xx_hal.h"
#define ADXL_ADDRESS 0x53<<1

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern uint8_t intSource;
uint8_t threshold = 0;
uint8_t duration = 0;
uint8_t axises = 0;

void ADXL345_Init(void);
	
void ADXL345_Init(void) {

    setPowerControl(0x00);
    setDataRate(ADXL345_800HZ);
    //Full resolution, +/-16g, 4mg/LSB.
//    setDataFormatControl(0x01);
    setDataFormatControl(0x02);
    setTapThreshold(200);
    setTapDuration(50);
    setTapAxisControl(0x07);
    setInterruptMappingControl(0x40);
    setInterruptEnableControl(0x40);
    intSource = getInterruptSource();
    
//    threshold =getTapThreshold();
//    duration = getTapDuration();
//    axises = getTapAxisControl();
    //Measurement mode.
    setPowerControl(0x08);
}

uint8_t getDevId(void)
{
	return oneByteRead(ADXL345_DEVID_REG);
}

uint8_t getTapThreshold(void)
{
	return oneByteRead(ADXL345_THRESH_TAP_REG);
}

void setTapThreshold(uint8_t threshold)
{
	oneByteWrite(ADXL345_THRESH_TAP_REG, threshold);
}

uint8_t getOffset(uint8_t axis)
{
	uint8_t address = 0;

	if (axis == ADXL345_X) {
			address = ADXL345_OFSX_REG;
	} else if (axis == ADXL345_Y) {
			address = ADXL345_OFSY_REG;
	} else if (axis == ADXL345_Z) {
			address = ADXL345_OFSZ_REG;
	}

	return oneByteRead(address);
}

void setOffset(uint8_t axis, uint8_t offset)
{
	uint8_t address = 0;

	if (axis == ADXL345_X) {
			address = ADXL345_OFSX_REG;
	} else if (axis == ADXL345_Y) {
			address = ADXL345_OFSY_REG;
	} else if (axis == ADXL345_Z) {
			address = ADXL345_OFSZ_REG;
	}

	oneByteWrite(address, offset);
}

uint8_t getTapDuration(void)
{
	return oneByteRead(ADXL345_DUR_REG)*625;
}

void setTapDuration(uint8_t duration)
{

	oneByteWrite(ADXL345_DUR_REG, duration);
}

float getTapLatency(void)
{
	return oneByteRead(ADXL345_LATENT_REG)*1.25;
}

void setTapLatency(uint8_t latency_ms)
{
	uint8_t tapLatency = latency_ms / 1.25;

	oneByteWrite(ADXL345_LATENT_REG, tapLatency);
}

float getWindowTime(void)
{
	return oneByteRead(ADXL345_WINDOW_REG)*1.25;
}

void setWindowTime(uint8_t window_ms)
{
	uint8_t windowTime = window_ms / 1.25;

	oneByteWrite(ADXL345_WINDOW_REG, windowTime);
}

uint8_t getActivityThreshold(void)
{
	return oneByteRead(ADXL345_THRESH_ACT_REG);
}

void setActivityThreshold(uint8_t threshold)
{
	oneByteWrite(ADXL345_THRESH_ACT_REG, threshold);
}

uint8_t getInactivityThreshold(void)
{
	return oneByteRead(ADXL345_THRESH_INACT_REG);
}

void setInactivityThreshold(uint8_t threshold)
{
	oneByteWrite(ADXL345_THRESH_INACT_REG, threshold);
}

uint8_t getTimeInactivity(void)
{
	return oneByteRead(ADXL345_TIME_INACT_REG);
}

void setTimeInactivity(uint8_t timeInactivity)
{
	oneByteWrite(ADXL345_TIME_INACT_REG, timeInactivity);
}

uint8_t getActivityInactivityControl(void)
{
	return oneByteRead(ADXL345_ACT_INACT_CTL_REG);
}

void setActivityInactivityControl(uint8_t settings)
{
	oneByteWrite(ADXL345_ACT_INACT_CTL_REG, settings);
}

uint8_t getFreefallThreshold(void)
{
	return oneByteRead(ADXL345_THRESH_FF_REG);
}

void setFreefallThreshold(uint8_t threshold)
{
	oneByteWrite(ADXL345_THRESH_FF_REG, threshold);
}

uint8_t getFreefallTime(void)
{
	return oneByteRead(ADXL345_TIME_FF_REG)*5;
}

void setFreefallTime(uint8_t freefallTime_ms)
{
	uint8_t freefallTime = freefallTime_ms / 5;
	oneByteWrite(ADXL345_TIME_FF_REG, freefallTime);
}

uint8_t getTapAxisControl(void)
{
	return oneByteRead(ADXL345_TAP_AXES_REG);
}

void setTapAxisControl(uint8_t settings)
{
	oneByteWrite(ADXL345_TAP_AXES_REG, settings);
}

uint8_t getTapSource(void)
{
	return oneByteRead(ADXL345_ACT_TAP_STATUS_REG);
}

void setPowerMode(uint8_t mode)
{
	//Get the current register contents, so we don't clobber the rate value.
	uint8_t registerContents = oneByteRead(ADXL345_BW_RATE_REG);

	registerContents = (mode << 4) | registerContents;

	oneByteWrite(ADXL345_BW_RATE_REG, registerContents);
}

uint8_t getPowerControl(void)
{
	return oneByteRead(ADXL345_POWER_CTL_REG);
}

void setPowerControl(uint8_t settings)
{
	oneByteWrite(ADXL345_POWER_CTL_REG, settings);
}

uint8_t getuint8_terruptEnableControl(void)
{
	return oneByteRead(ADXL345_INT_ENABLE_REG);
}

void setInterruptEnableControl(uint8_t settings)
{
	oneByteWrite(ADXL345_INT_ENABLE_REG, settings);
}

uint8_t getuint8_terruptMappingControl(void)
{
	return oneByteRead(ADXL345_INT_MAP_REG);
}

void setInterruptMappingControl(uint8_t settings)
{
	oneByteWrite(ADXL345_INT_MAP_REG, settings);
}

uint8_t getInterruptSource(void)
{
	return oneByteRead(ADXL345_INT_SOURCE_REG);
}

uint8_t getDataFormatControl(void)
{
	return oneByteRead(ADXL345_DATA_FORMAT_REG);
}

void setDataFormatControl(uint8_t settings)
{
	oneByteWrite(ADXL345_DATA_FORMAT_REG, settings);
}

void setDataRate(uint8_t rate)
{
	//Get the current register contents, so we don't clobber the power bit.
	uint8_t registerContents = oneByteRead(ADXL345_BW_RATE_REG);

	registerContents &= ~0x10;
	registerContents |= rate;

	oneByteWrite(ADXL345_BW_RATE_REG, registerContents);
}


uint8_t ADXL345_Read(uint8_t *buffer) //buffer - 6 byte destination conversion buffer
{

	multiByteRead(ADXL345_DATAX0_REG, buffer, 6);
	
	return 1;
}

uint8_t getFifoControl(void)
{
	return oneByteRead(ADXL345_FIFO_CTL);
}

void setFifoControl(uint8_t settings)
{
	oneByteWrite(ADXL345_FIFO_STATUS, settings);
}

uint8_t getFifoStatus(void)
{
	return oneByteRead(ADXL345_FIFO_STATUS);
}

uint8_t oneByteRead(uint8_t address)
{
	uint8_t rx[1] = {0};
	HAL_I2C_Mem_Read(&hi2c2, ADXL_ADDRESS, address, 1, rx, 1, 100);

	return rx[0];
}

void oneByteWrite(uint8_t address, uint8_t data)
{
	uint8_t databuf[2] = {0};
	databuf[0] = address;
	databuf[1] = data;

	HAL_I2C_Master_Transmit(&hi2c2, ADXL_ADDRESS, databuf, 2, 100);
	
}

void multiByteRead(uint8_t startAddress, uint8_t* buffer, uint8_t size)
{
	HAL_I2C_Mem_Read(&hi2c2, ADXL_ADDRESS, startAddress, 1, buffer, size, 100);
}

void multiByteWrite(uint8_t startAddress, uint8_t* buffer, uint8_t size)
{
	HAL_I2C_Mem_Write(&hi2c2, ADXL_ADDRESS, startAddress, 1, buffer, size, 100);
}

