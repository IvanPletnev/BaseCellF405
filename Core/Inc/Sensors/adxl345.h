#ifndef __ADXL345_H
#define __ADXL345_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
 * Defines
 */
//Registers.
#define ADXL345_DEVID_REG          0x00
#define ADXL345_THRESH_TAP_REG     0x1D
#define ADXL345_OFSX_REG           0x1E
#define ADXL345_OFSY_REG           0x1F
#define ADXL345_OFSZ_REG           0x20
#define ADXL345_DUR_REG            0x21
#define ADXL345_LATENT_REG         0x22
#define ADXL345_WINDOW_REG         0x23
#define ADXL345_THRESH_ACT_REG     0x24
#define ADXL345_THRESH_INACT_REG   0x25
#define ADXL345_TIME_INACT_REG     0x26
#define ADXL345_ACT_INACT_CTL_REG  0x27
#define ADXL345_THRESH_FF_REG      0x28
#define ADXL345_TIME_FF_REG        0x29
#define ADXL345_TAP_AXES_REG       0x2A
#define ADXL345_ACT_TAP_STATUS_REG 0x2B
#define ADXL345_BW_RATE_REG        0x2C
#define ADXL345_POWER_CTL_REG      0x2D
#define ADXL345_INT_ENABLE_REG     0x2E
#define ADXL345_INT_MAP_REG        0x2F
#define ADXL345_INT_SOURCE_REG     0x30
#define ADXL345_DATA_FORMAT_REG    0x31
#define ADXL345_DATAX0_REG         0x32
#define ADXL345_DATAX1_REG         0x33
#define ADXL345_DATAY0_REG         0x34
#define ADXL345_DATAY1_REG         0x35
#define ADXL345_DATAZ0_REG         0x36
#define ADXL345_DATAZ1_REG         0x37
#define ADXL345_FIFO_CTL           0x38
#define ADXL345_FIFO_STATUS        0x39

//Data rate codes.
#define ADXL345_3200HZ      0x0F
#define ADXL345_1600HZ      0x0E
#define ADXL345_800HZ       0x0D
#define ADXL345_400HZ       0x0C
#define ADXL345_200HZ       0x0B
#define ADXL345_100HZ       0x0A
#define ADXL345_50HZ        0x09
#define ADXL345_25HZ        0x08
#define ADXL345_12HZ5       0x07
#define ADXL345_6HZ25       0x06

#define ADXL345_SPI_READ    0x80
#define ADXL345_SPI_WRITE   0x00
#define ADXL345_MULTI_BYTE  0x60

#define ADXL345_X           0x00
#define ADXL345_Y           0x01
#define ADXL345_Z           0x02

extern void ADXL345_Init(void);
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
/**
 * ADXL345 triple axis, digital interface, accelerometer.
 */
/**
 * Constructor.
 *
 * @param mosi mbed pin to use for MOSI line of SPI interface.
 * @param miso mbed pin to use for MISO line of SPI interface.
 * @param sck mbed pin to use for SCK line of SPI interface.
 * @param cs mbed pin to use for not chip select line of SPI interface.
 */
/**
 * Read the device ID register on the device.
 *
 * @return The device ID code [0xE5]
 */
uint8_t getDevId(void);

/**
 * Read the tap threshold on the device.
 *
 * @return The tap threshold as an 8-bit number with a scale factor of
 *         62.5mg/LSB.
 */
uint8_t getTapThreshold(void);

/**
 * Set the tap threshold.
 *
 * @param The tap threshold as an 8-bit number with a scale factor of
 *        62.5mg/LSB.
 */
void setTapThreshold(uint8_t threshold);

/**
 * Get the current offset for a particular axis.
 *
 * @param axis 0x00 -> X-axis
 *             0x01 -> Y-axis
 *             0x02 -> Z-axis
 * @return The current offset as an 8-bit 2's complement number with scale
 *         factor 15.6mg/LSB.
 */
uint8_t getOffset(uint8_t axis);

/**
 * Set the offset for a particular axis.
 *
 * @param axis 0x00 -> X-axis
 *             0x01 -> Y-axis
 *             0x02 -> Z-axis
 * @param offset The offset as an 8-bit 2's complement number with scale
 *               factor 15.6mg/LSB.
 */
void setOffset(uint8_t axis, uint8_t offset);

/**
 * Get the tap duration required to trigger an event.
 *
 * @return The max time that an event must be above the tap threshold to
 *         qualify as a tap event, in microseconds.
 */
uint8_t getTapDuration(void);

/**
 * Set the tap duration required to trigger an event.
 *
 * @param duration_us The max time that an event must be above the tap
 *                    threshold to qualify as a tap event, in microseconds.
 *                    Time will be normalized by the scale factor which is
 *                    625us/LSB. A value of 0 disables the single/double
 *                    tap functions.
 */
void setTapDuration(uint8_t duration_us);

/**
 * Get the tap latency between the detection of a tap and the time window.
 *
 * @return The wait time from the detection of a tap event to the start of
 *         the time window during which a possible second tap event can be
 *         detected in milliseconds.
 */
float getTapLatency(void);

/**
 * Set the tap latency between the detection of a tap and the time window.
 *
 * @param latency_ms The wait time from the detection of a tap event to the
 *                   start of the time window during which a possible
 *                   second tap event can be detected in milliseconds.
 *                   A value of 0 disables the double tap function.
 */
void setTapLatency(uint8_t latency_ms);

/**
 * Get the time of window between tap latency and a double tap.
 *
 * @return The amount of time after the expiration of the latency time
 *         during which a second valid tap can begin, in milliseconds.
 */
float getWindowTime(void);

/**
 * Set the time of the window between tap latency and a double tap.
 *
 * @param window_ms The amount of time after the expiration of the latency
 *                  time during which a second valid tap can begin,
 *                  in milliseconds.
 */
void setWindowTime(uint8_t window_ms);

/**
 * Get the threshold value for detecting activity.
 *
 * @return The threshold value for detecting activity as an 8-bit number.
 *         Scale factor is 62.5mg/LSB.
 */
uint8_t getActivityThreshold(void);

/**
 * Set the threshold value for detecting activity.
 *
 * @param threshold The threshold value for detecting activity as an 8-bit
 *                  number. Scale factor is 62.5mg/LSB. A value of 0 may
 *                  result in undesirable behavior if the activity
 *                  interrupt is enabled.
 */
void setActivityThreshold(uint8_t threshold);

/**
 * Get the threshold value for detecting inactivity.
 *
 * @return The threshold value for detecting inactivity as an 8-bit number.
 *         Scale factor is 62.5mg/LSB.
 */
uint8_t getInactivityThreshold(void);

/**
 * Set the threshold value for detecting inactivity.
 *
 * @param threshold The threshold value for detecting inactivity as an
 *                  8-bit number. Scale factor is 62.5mg/LSB.
 */
void setInactivityThreshold(uint8_t threshold);

/**
 * Get the time required for inactivity to be declared.
 *
 * @return The amount of time that acceleration must be less than the
 *         inactivity threshold for inactivity to be declared, in
 *         seconds.
 */
uint8_t getTimeInactivity(void);

/**
 * Set the time required for inactivity to be declared.
 *
 * @param inactivity The amount of time that acceleration must be less than
 *                   the inactivity threshold for inactivity to be
 *                   declared, in seconds. A value of 0 results in an
 *                   interrupt when the output data is less than the
 *                   threshold inactivity.
 */
void setTimeInactivity(uint8_t timeInactivity);

/**
 * Get the activity/inactivity control settings.
 *
 *      D7            D6             D5            D4
 * +-----------+--------------+--------------+--------------+
 * | ACT ac/dc | ACT_X enable | ACT_Y enable | ACT_Z enable |
 * +-----------+--------------+--------------+--------------+
 *
 *        D3             D2               D1              D0
 * +-------------+----------------+----------------+----------------+
 * | INACT ac/dc | INACT_X enable | INACT_Y enable | INACT_Z enable |
 * +-------------+----------------+----------------+----------------+
 *
 * See datasheet for details.
 *
 * @return The contents of the ACT_INACT_CTL register.
 */
uint8_t getActivityInactivityControl(void);

/**
 * Set the activity/inactivity control settings.
 *
 *      D7            D6             D5            D4
 * +-----------+--------------+--------------+--------------+
 * | ACT ac/dc | ACT_X enable | ACT_Y enable | ACT_Z enable |
 * +-----------+--------------+--------------+--------------+
 *
 *        D3             D2               D1              D0
 * +-------------+----------------+----------------+----------------+
 * | INACT ac/dc | INACT_X enable | INACT_Y enable | INACT_Z enable |
 * +-------------+----------------+----------------+----------------+
 *
 * See datasheet for details.
 *
 * @param settings The control byte to write to the ACT_INACT_CTL register.
 */
void setActivityInactivityControl(uint8_t settings);

/**
 * Get the threshold for free fall detection.
 *
 * @return The threshold value for free-fall detection, as an 8-bit number,
 *         with scale factor 62.5mg/LSB.
 */
uint8_t getFreefallThreshold(void);

/**
 * Set the threshold for free fall detection.
 *
 * @return The threshold value for free-fall detection, as an 8-bit number,
 *         with scale factor 62.5mg/LSB. A value of 0 may result in 
 *         undesirable behavior if the free-fall interrupt is enabled.
 *         Values between 300 mg and 600 mg (0x05 to 0x09) are recommended.
 */
void setFreefallThreshold(uint8_t threshold);

/**
 * Get the time required to generate a free fall interrupt.
 *
 * @return The minimum time that the value of all axes must be less than
 *         the freefall threshold to generate a free-fall interrupt, in
 *         milliseconds.
 */
uint8_t getFreefallTime(void);

/**
 * Set the time required to generate a free fall interrupt.
 *
 * @return The minimum time that the value of all axes must be less than
 *         the freefall threshold to generate a free-fall interrupt, in
 *         milliseconds. A value of 0 may result in undesirable behavior
 *         if the free-fall interrupt is enabled. Values between 100 ms 
 *         and 350 ms (0x14 to 0x46) are recommended.
 */
void setFreefallTime(uint8_t freefallTime_ms);

/**
 * Get the axis tap settings.
 *
 *      D3           D2            D1             D0
 * +----------+--------------+--------------+--------------+
 * | Suppress | TAP_X enable | TAP_Y enable | TAP_Z enable |
 * +----------+--------------+--------------+--------------+
 *
 * (D7-D4 are 0s).
 *
 * See datasheet for more details.
 *
 * @return The contents of the TAP_AXES register.
 */ 
uint8_t getTapAxisControl(void);

/**
 * Set the axis tap settings.
 *
 *      D3           D2            D1             D0
 * +----------+--------------+--------------+--------------+
 * | Suppress | TAP_X enable | TAP_Y enable | TAP_Z enable |
 * +----------+--------------+--------------+--------------+
 *
 * (D7-D4 are 0s).
 *
 * See datasheet for more details.
 *
 * @param The control byte to write to the TAP_AXES register.
 */
void setTapAxisControl(uint8_t settings);

/**
 * Get the source of a tap.
 *
 * @return The contents of the ACT_TAP_STATUS register.
 */
uint8_t getTapSource(void);

/**
 * Set the power mode.
 *
 * @param mode 0 -> Normal operation.
 *             1 -> Reduced power operation.
 */
void setPowerMode(uint8_t mode);

/**
 * Set the data rate.
 *
 * @param rate The rate code (see #defines or datasheet).
 */
void setDataRate(uint8_t rate);

/**
 * Get the power control settings.
 *
 * See datasheet for details.
 *
 * @return The contents of the POWER_CTL register.
 */
uint8_t getPowerControl(void);

/**
 * Set the power control settings.
 *
 * See datasheet for details.
 *
 * @param The control byte to write to the POWER_CTL register.
 */
void setPowerControl(uint8_t settings);

/**
 * Get the interrupt enable settings.
 *
 * @return The contents of the INT_ENABLE register.
 */
uint8_t getInterruptEnableControl(void);

/**
 * Set the interrupt enable settings.
 *
 * @param settings The control byte to write to the INT_ENABLE register.
 */
void setInterruptEnableControl(uint8_t settings);

/**
 * Get the interrupt mapping settings.
 *
 * @return The contents of the INT_MAP register.
 */
uint8_t getInterruptMappingControl(void);

/**
 * Set the interrupt mapping settings.
 *
 * @param settings The control byte to write to the INT_MAP register.
 */
void setInterruptMappingControl(uint8_t settings);

/**
 * Get the interrupt source.
 *
 * @return The contents of the INT_SOURCE register.
 */
uint8_t getInterruptSource(void);

/**
 * Get the data format settings.
 *
 * @return The contents of the DATA_FORMAT register.
 */
uint8_t getDataFormatControl(void);

/**
 * Set the data format settings.
 *
 * @param settings The control byte to write to the DATA_FORMAT register.
 */
void setDataFormatControl(uint8_t settings);

/**
 * Get the output of all three axes.
 *
 * @param Pointer to a buffer to hold the accelerometer value for the
 *        x-axis, y-axis and z-axis [in that order].
 */
uint8_t ADXL345_Read(uint8_t *buffer);

/**
 * Get the FIFO control settings.
 *
 * @return The contents of the FIFO_CTL register.
 */
uint8_t getFifoControl(void);

/**
 * Set the FIFO control settings.
 *
 * @param The control byte to write to the FIFO_CTL register.
 */
void setFifoControl(uint8_t settings);

/**
 * Get FIFO status.
 *
 * @return The contents of the FIFO_STATUS register.
 */
uint8_t getFifoStatus(void);
    
/**
 * Read one byte from a register on the device.
 *
 * @param address Address of the register to read.
 *
 * @return The contents of the register address.
 */
uint8_t oneByteRead(uint8_t address);

/**
 * Write one byte to a register on the device.
 *
 * @param address Address of the register to write to.
 * @param data The data to write into the register.
 */
void oneByteWrite(uint8_t address, uint8_t data);

/**
 * Read several consecutive bytes on the device.
 *
 * @param startAddress The address of the first register to read from.
 * @param buffer Pointer to a buffer to store data read from the device.
 * @param size The number of bytes to read.
 */
void multiByteRead(uint8_t startAddress, uint8_t* buffer, uint8_t size);

/**
 * Write several consecutive bytes on the device.
 *
 * @param startAddress The address of the first register to write to.
 * @param buffer Pointer to a buffer which contains the data to write.
 * @param size The number of bytes to write.
 */
void multiByteWrite(uint8_t startAddress, uint8_t* buffer, uint8_t size);

#endif /* ADXL345_H */
