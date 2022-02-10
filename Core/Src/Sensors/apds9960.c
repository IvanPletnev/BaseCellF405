#include "apds9960.h"
#include <stdlib.h>
#include <math.h>



/* Direction definitions */
enum {
	DIR_NONE, DIR_LEFT, DIR_RIGHT, DIR_UP, DIR_DOWN, DIR_NEAR, DIR_FAR, DIR_ALL
};

/* State definitions */
enum {
	NA_STATE, NEAR_STATE, FAR_STATE, ALL_STATE
};

/* Container for gesture data */
typedef struct gesture_data_type {
	uint8_t u_data[32];
	uint8_t d_data[32];
	uint8_t l_data[32];
	uint8_t r_data[32];
	uint8_t index;
	uint8_t total_gestures;
	uint8_t in_threshold;
	uint8_t out_threshold;
} gesture_data_type;

uint8_t APDS9960_SetActiveChan(uint8_t n_apds9960);
uint8_t mpxSetReg(uint8_t Register_Addr);
uint8_t mpxPowerOn(void);

uint8_t sensorWriteDataByte(uint16_t Register_Addr, uint8_t Register_Data);
uint8_t sensorReadDataByte(uint16_t Register_Addr, uint8_t *Register_Data);
uint8_t sensorReadDataBlock(uint16_t Register_Addr, uint8_t *Register_Data,
		uint16_t Size);

uint8_t init(void);
uint8_t getStatusRegister(void);
uint8_t getMode(void);
uint8_t setMode(uint8_t mode, uint8_t enable);

/* Turn the APDS-9960 on and off */
uint8_t enablePower(void);
uint8_t disablePower(void);

/* Enable or disable specific sensors */
uint8_t enableLightSensor(uint8_t interrupts);
uint8_t disableLightSensor(void);
uint8_t enableProximitySensor(uint8_t interrupts);
uint8_t disableProximitySensor(void);
uint8_t enableGestureSensor(uint8_t interrupts);
uint8_t disableGestureSensor(void);

/* LED drive strength control */
uint8_t getLEDDrive(void);
uint8_t setLEDDrive(uint8_t drive);
uint8_t getGestureLEDDrive(void);
uint8_t setGestureLEDDrive(uint8_t drive);

/* Gain control */
uint8_t getAmbientLightGain(void);
uint8_t setAmbientLightGain(uint8_t gain);
uint8_t getProximityGain(void);
uint8_t setProximityGain(uint8_t gain);
uint8_t getGestureGain(void);
uint8_t setGestureGain(uint8_t gain);

/* Get and set light interrupt thresholds */
uint8_t getLightIntLowThreshold(uint16_t *threshold);
uint8_t setLightIntLowThreshold(uint16_t threshold);
uint8_t getLightIntHighThreshold(uint16_t *threshold);
uint8_t setLightIntHighThreshold(uint16_t threshold);

/* Get and set proximity interrupt thresholds */
uint8_t getProximityIntLowThreshold(uint8_t *threshold);
uint8_t setProximityIntLowThreshold(uint8_t threshold);
uint8_t getProximityIntHighThreshold(uint8_t *threshold);
uint8_t setProximityIntHighThreshold(uint8_t threshold);

/* Get and set interrupt enables */
uint8_t getAmbientLightIntEnable(void);
uint8_t setAmbientLightIntEnable(uint8_t enable);
uint8_t getProximityIntEnable(void);
uint8_t setProximityIntEnable(uint8_t enable);
uint8_t getGestureIntEnable(void);
uint8_t setGestureIntEnable(uint8_t enable);

/* Clear interrupts */
uint8_t clearAmbientLightInt(void);
uint8_t clearProximityInt(void);

/* Ambient light methods */

/* Proximity methods */
uint8_t readProximity(uint8_t *val);

/* Gesture methods */
uint8_t isGestureAvailable(void);
int readGesture(void);

/* Gesture processing */
void resetGestureParameters(void);
uint8_t processGestureData(void);
uint8_t decodeGesture(void);

/* Proximity Interrupt Threshold */
uint8_t getProxIntLowThresh(void);
uint8_t setProxIntLowThresh(uint8_t threshold);
uint8_t getProxIntHighThresh(void);
uint8_t setProxIntHighThresh(uint8_t threshold);

/* LED Boost Control */
uint8_t getLEDBoost(void);
uint8_t setLEDBoost(uint8_t boost);

/* Proximity photodiode select */
uint8_t getProxGainCompEnable(void);
uint8_t setProxGainCompEnable(uint8_t enable);
uint8_t getProxPhotoMask(void);
uint8_t setProxPhotoMask(uint8_t mask);

/* Gesture threshold control */
uint8_t getGestureEnterThresh(void);
uint8_t setGestureEnterThresh(uint8_t threshold);
uint8_t getGestureExitThresh(void);
uint8_t setGestureExitThresh(uint8_t threshold);

/* Gesture LED, gain, and time control */
uint8_t getGestureWaitTime(void);
uint8_t setGestureWaitTime(uint8_t time);

/* Gesture mode */
uint8_t getGestureMode(void);
uint8_t setGestureMode(uint8_t mode);





/* Members */
gesture_data_type gesture_data_;
int gesture_ud_delta_;
int gesture_lr_delta_;
int gesture_ud_count_;
int gesture_lr_count_;
int gesture_near_count_;
int gesture_far_count_;
int gesture_state_;
int gesture_motion_;

uint8_t mpx_reg[2] = { MPX_CHAN_0, MPX_CHAN_1 };
uint8_t apds9960_id[2] = { 0, 0 }, nAPDS9960 = 0;


uint8_t APDS9960_Init(void) {
	uint8_t i = 0;

	gesture_ud_delta_ = 0;
	gesture_lr_delta_ = 0;

	gesture_ud_count_ = 0;
	gesture_lr_count_ = 0;

	gesture_near_count_ = 0;
	gesture_far_count_ = 0;

	gesture_state_ = 0;
	gesture_motion_ = DIR_NONE;

	if (!mpxPowerOn()) {
		return STATUS_FAIL;
	}

	for (i = 0; i < 2; i++) {
		if (!APDS9960_SetActiveChan(i)) {
			continue;
		}

		osDelay(20);

		/* Read ID register and check against known values for APDS-9960 */
		if (!sensorReadDataByte(APDS9960_ID, &apds9960_id[i])) {
			continue;
		}

		if (!((apds9960_id[i] == APDS9960_ID_1)
				|| (apds9960_id[i] == APDS9960_ID_2))) {
			apds9960_id[i] = 0;
			continue;
		}

		if (init() == STATUS_OK) {

		}
	}

	return STATUS_OK;
}

uint8_t mpxPowerOn(void) {
	return mpxSetReg(MPX_PWR_ON);
}

uint8_t APDS9960_SetActiveChan(uint8_t n_chan) {
	return mpxSetReg(mpx_reg[n_chan]);
}

/**
 * @brief Configures I2C communications and initializes registers to defaults
 *
 * @return STATUS_OK if initialized successfully. STATUS_FAIL otherwise.
 */
uint8_t init(void) {

	/* Set ENABLE register to 0 (disable all features) */
	if (!setMode(ALL, OFF)) {
		return STATUS_FAIL;
	}

	/* Set default values for ambient light and proximity registers */
	if (!sensorWriteDataByte(APDS9960_ATIME, DEFAULT_ATIME)) {
		return STATUS_FAIL;
	}
	if (!sensorWriteDataByte(APDS9960_WTIME, DEFAULT_WTIME)) {
		return STATUS_FAIL;
	}
	if (!sensorWriteDataByte(APDS9960_PPULSE, DEFAULT_PROX_PPULSE)) {
		return STATUS_FAIL;
	}
	if (!sensorWriteDataByte(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR)) {
		return STATUS_FAIL;
	}
	if (!sensorWriteDataByte(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL)) {
		return STATUS_FAIL;
	}
	if (!sensorWriteDataByte(APDS9960_CONFIG1, DEFAULT_CONFIG1)) {
		return STATUS_FAIL;
	}
	if (!setLEDDrive(DEFAULT_LDRIVE)) {
		return STATUS_FAIL;
	}
	if (!setProximityGain(DEFAULT_PGAIN)) {
		return STATUS_FAIL;
	}
	if (!setAmbientLightGain(DEFAULT_AGAIN)) {
		return STATUS_FAIL;
	}
	if (!setProxIntLowThresh(DEFAULT_PILT)) {
		return STATUS_FAIL;
	}
	if (!setProxIntHighThresh(DEFAULT_PIHT)) {
		return STATUS_FAIL;
	}
	if (!setLightIntLowThreshold(DEFAULT_AILT)) {
		return STATUS_FAIL;
	}
	if (!setLightIntHighThreshold(DEFAULT_AIHT)) {
		return STATUS_FAIL;
	}
	if (!sensorWriteDataByte(APDS9960_PERS, DEFAULT_PERS)) {
		return STATUS_FAIL;
	}
	if (!sensorWriteDataByte(APDS9960_CONFIG2, DEFAULT_CONFIG2)) {
		return STATUS_FAIL;
	}
	if (!sensorWriteDataByte(APDS9960_CONFIG3, DEFAULT_CONFIG3)) {
		return STATUS_FAIL;
	}

	/* Set default values for gesture sense registers */
//	if (!setGestureEnterThresh(DEFAULT_GPENTH)) {
//		return STATUS_FAIL;
//	}
//	if (!setGestureExitThresh(DEFAULT_GEXTH)) {
//		return STATUS_FAIL;
//	}
//	if (!sensorWriteDataByte(APDS9960_GCONF1, DEFAULT_GCONF1)) {
//		return STATUS_FAIL;
//	}
//	if (!setGestureGain(DEFAULT_GGAIN)) {
//		return STATUS_FAIL;
//	}
//	if (!setGestureLEDDrive(DEFAULT_GLDRIVE)) {
//		return STATUS_FAIL;
//	}
//	if (!setGestureWaitTime(DEFAULT_GWTIME)) {
//		return STATUS_FAIL;
//	}
//	if (!sensorWriteDataByte(APDS9960_GOFFSET_U, DEFAULT_GOFFSET)) {
//		return STATUS_FAIL;
//	}
//	if (!sensorWriteDataByte(APDS9960_GOFFSET_D, DEFAULT_GOFFSET)) {
//		return STATUS_FAIL;
//	}
//	if (!sensorWriteDataByte(APDS9960_GOFFSET_L, DEFAULT_GOFFSET)) {
//		return STATUS_FAIL;
//	}
//	if (!sensorWriteDataByte(APDS9960_GOFFSET_R, DEFAULT_GOFFSET)) {
//		return STATUS_FAIL;
//	}
//	if (!sensorWriteDataByte(APDS9960_GPULSE, DEFAULT_GPULSE)) {
//		return STATUS_FAIL;
//	}
//	if (!sensorWriteDataByte(APDS9960_GCONF3, DEFAULT_GCONF3)) {
//		return STATUS_FAIL;
//	}
//	if (!setGestureIntEnable(DEFAULT_GIEN)) {
//		return STATUS_FAIL;
//	}
	if (!enableLightSensor(0)) {
		return STATUS_FAIL;
	}

#if 0
    /* Gesture config register dump */
    uint8_t reg;
    uint8_t val;
  
    for(reg = 0x80; reg <= 0xAF; reg++) {
        if( (reg != 0x82) && \
            (reg != 0x8A) && \
            (reg != 0x91) && \
            (reg != 0xA8) && \
            (reg != 0xAC) && \
            (reg != 0xAD) )
        {
            sensorReadDataByte(Register_Addr, val);
            Serial.print(Register_Addr, HEX);
            Serial.print(": 0x");
            Serial.println(val, HEX);
        }
    }

    for(reg = 0xE4; reg <= 0xE7; reg++) {
        sensorReadDataByte(Register_Addr, val);
        Serial.print(Register_Addr, HEX);
        Serial.print(": 0x");
        Serial.println(val, HEX);
    }
#endif

	return STATUS_OK;
}

/*******************************************************************************
 * Public methods for controlling the APDS-9960
 ******************************************************************************/

/**
 * @brief Reads and returns the contents of the STATUS register
 *
 * @return Contents of the STATUS register. 0xFF if error.
 */
uint8_t getStatusRegister(void) {
	uint8_t status_value;

	/* Read current ENABLE register */
	if (!sensorReadDataByte(APDS9960_STATUS, &status_value)) {
		return ERROR;
	}

	return status_value;
}

/**
 * @brief Reads and returns the contents of the ENABLE register
 *
 * @return Contents of the ENABLE register. 0xFF if error.
 */
uint8_t getMode() {
	uint8_t enable_value;

	/* Read current ENABLE register */
	if (!sensorReadDataByte(APDS9960_ENABLE, &enable_value)) {
		return ERROR;
	}

	return enable_value;
}

/**
 * @brief Enables or disables a feature in the APDS-9960
 *
 * @param[in] mode which feature to enable
 * @param[in] enable ON (1) or OFF (0)
 * @return STATUS_OK if operation success. STATUS_FAIL otherwise.
 */
uint8_t setMode(uint8_t pmode, uint8_t enable) {
	uint8_t reg_val;

	/* Read current ENABLE register */
	reg_val = getMode();
	if (reg_val == ERROR) {
		return STATUS_FAIL;
	}

	/* Change bit(s) in ENABLE register */
	enable = enable & 0x01;
	if (pmode <= 6) {
		if (enable) {
			reg_val |= (1 << pmode);
		} else {
			reg_val &= ~(1 << pmode);
		}
	} else if (pmode == ALL) {
		if (enable) {
			reg_val = 0x7F;
		} else {
			reg_val = 0x00;
		}
	}

	/* Write value back to ENABLE register */
	if (!sensorWriteDataByte(APDS9960_ENABLE, reg_val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Starts the light (R/G/B/Ambient) sensor on the APDS-9960
 *
 * @param[in] interrupts STATUS_OK to enable hardware interrupt on high or low light
 * @return STATUS_OK if sensor enabled correctly. STATUS_FAIL on error.
 */
uint8_t enableLightSensor(uint8_t interrupts) {

	/* Set default gain, interrupts, enable power, and enable sensor */
	if (!setAmbientLightGain(AGAIN_1X)) {
		return STATUS_FAIL;
	}
	if (interrupts) {
		if (!setAmbientLightIntEnable(1)) {
			return STATUS_FAIL;
		}
	} else {
		if (!setAmbientLightIntEnable(0)) {
			return STATUS_FAIL;
		}
	}
	if (!enablePower()) {
		return STATUS_FAIL;
	}
	if (!setMode(AMBIENT_LIGHT, 1)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;

}

/**
 * @brief Ends the light sensor on the APDS-9960
 *
 * @return STATUS_OK if sensor disabled correctly. STATUS_FAIL on error.
 */
uint8_t disableLightSensor() {
	if (!setAmbientLightIntEnable(0)) {
		return STATUS_FAIL;
	}
	if (!setMode(AMBIENT_LIGHT, 0)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Starts the proximity sensor on the APDS-9960
 *
 * @param[in] interrupts STATUS_OK to enable hardware external interrupt on proximity
 * @return STATUS_OK if sensor enabled correctly. STATUS_FAIL on error.
 */
uint8_t enableProximitySensor(uint8_t interrupts) {
	/* Set default gain, LED, interrupts, enable power, and enable sensor */
	if (!setProximityGain(DEFAULT_PGAIN)) {
		return STATUS_FAIL;
	}
	if (!setLEDDrive(DEFAULT_LDRIVE)) {
		return STATUS_FAIL;
	}
	if (interrupts) {
		if (!setProximityIntEnable(1)) {
			return STATUS_FAIL;
		}
	} else {
		if (!setProximityIntEnable(0)) {
			return STATUS_FAIL;
		}
	}
	if (!enablePower()) {
		return STATUS_FAIL;
	}
	if (!setMode(PROXIMITY, 1)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Ends the proximity sensor on the APDS-9960
 *
 * @return STATUS_OK if sensor disabled correctly. STATUS_FAIL on error.
 */
uint8_t disableProximitySensor() {
	if (!setProximityIntEnable(0)) {
		return STATUS_FAIL;
	}
	if (!setMode(PROXIMITY, 0)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Starts the gesture recognition engine on the APDS-9960
 *
 * @param[in] interrupts STATUS_OK to enable hardware external interrupt on gesture
 * @return STATUS_OK if engine enabled correctly. STATUS_FAIL on error.
 */
uint8_t enableGestureSensor(uint8_t interrupts) {

	/* Enable gesture mode
	 Set ENABLE to 0 (power off)
	 Set WTIME to 0xFF
	 Set AUX to LED_BOOST_300
	 Enable PON, WEN, PEN, GEN in ENABLE
	 */
	resetGestureParameters();
	if (!sensorWriteDataByte(APDS9960_WTIME, 0xFF)) {
		return STATUS_FAIL;
	}
	if (!sensorWriteDataByte(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE)) {
		return STATUS_FAIL;
	}
	if (!setLEDBoost(LED_BOOST_300)) {
		return STATUS_FAIL;
	}
	if (interrupts) {
		if (!setGestureIntEnable(1)) {
			return STATUS_FAIL;
		}
	} else {
		if (!setGestureIntEnable(0)) {
			return STATUS_FAIL;
		}
	}
	if (!setGestureMode(1)) {
		return STATUS_FAIL;
	}
	if (!enablePower()) {
		return STATUS_FAIL;
	}
	if (!setMode(WAIT, 1)) {
		return STATUS_FAIL;
	}
	if (!setMode(PROXIMITY, 1)) {
		return STATUS_FAIL;
	}
	if (!setMode(GESTURE, 1)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Ends the gesture recognition engine on the APDS-9960
 *
 * @return STATUS_OK if engine disabled correctly. STATUS_FAIL on error.
 */
uint8_t disableGestureSensor() {
	resetGestureParameters();
	if (!setGestureIntEnable(0)) {
		return STATUS_FAIL;
	}
	if (!setGestureMode(0)) {
		return STATUS_FAIL;
	}
	if (!setMode(GESTURE, 0)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Determines if there is a gesture available for reading
 *
 * @return STATUS_OK if gesture available. STATUS_FAIL otherwise.
 */
uint8_t isGestureAvailable() {
	uint8_t val;

	/* Read value from GSTATUS register */
	if (!sensorReadDataByte(APDS9960_GSTATUS, &val)) {
		return ERROR;
	}

	/* Shift and mask out GVALID bit */
	val &= APDS9960_GVALID;

	/* Return STATUS_OK/STATUS_FAIL based on GVALID bit */
	if (val == 1) {
		return STATUS_OK;
	} else {
		return STATUS_FAIL;
	}
}

/**
 * @brief Processes a gesture event and returns best guessed gesture
 *
 * @return Number corresponding to gesture. -1 on error.
 */
int readGesture() {
	uint8_t fifo_level = 0;
	uint8_t bytes_read = 0;
	uint8_t fifo_data[128];
	uint8_t gstatus;
	int motion;
	int i;

	/* Make sure that power and gesture is on and data is valid */
	if (!isGestureAvailable() || !(getMode() & 0x41)) {
		return DIR_NONE;
	}

	/* Keep looping as long as gesture data is valid */
	while (1) {

		/* Wait some time to collect next batch of FIFO data */
		osDelay(FIFO_PAUSE_TIME);

		/* Get the contents of the STATUS register. Is data still valid? */
		if (!sensorReadDataByte(APDS9960_GSTATUS, &gstatus)) {
			return ERROR;
		}

		/* If we have valid data, read in FIFO */
		if ((gstatus & APDS9960_GVALID) == APDS9960_GVALID) {

			/* Read the current FIFO level */
			if (!sensorReadDataByte(APDS9960_GFLVL, &fifo_level)) {
				return ERROR;
			}

#if DEBUG_APDS
            Serial.print("FIFO Level: ");
            Serial.println(fifo_level);
#endif

			/* If there's stuff in the FIFO, read it into our data block */
			if (fifo_level > 0) {
				bytes_read = sensorReadDataBlock( APDS9960_GFIFO_U,
						(uint8_t*) fifo_data, (fifo_level * 4));
				if (!bytes_read) {
					return ERROR;
				}
#if DEBUG_APDS
                Serial.print("FIFO Dump: ");
                for ( i = 0; i < bytes_read; i++ ) {
                    Serial.print(fifo_data[i]);
                    Serial.print(" ");
                }
                Serial.println();
#endif

				/* If at least 1 set of data, sort the data into U/D/L/R */
				if (bytes_read >= 4) {
					for (i = 0; i < bytes_read; i += 4) {
						gesture_data_.u_data[gesture_data_.index] = fifo_data[i
								+ 0];
						gesture_data_.d_data[gesture_data_.index] = fifo_data[i
								+ 1];
						gesture_data_.l_data[gesture_data_.index] = fifo_data[i
								+ 2];
						gesture_data_.r_data[gesture_data_.index] = fifo_data[i
								+ 3];
						gesture_data_.index++;
						gesture_data_.total_gestures++;
					}

#if DEBUG_APDS
                Serial.print("Up Data: ");
                for ( i = 0; i < gesture_data_.total_gestures; i++ ) {
                    Serial.print(gesture_data_.u_data[i]);
                    Serial.print(" ");
                }
                Serial.println();
#endif

					/* Filter and process gesture data. Decode near/far state */
					if (processGestureData()) {
						if (decodeGesture()) {
							//***TODO: U-Turn Gestures
#if DEBUG_APDS
                            //Serial.println(gesture_motion_);
#endif
						}
					}

					/* Reset data */
					gesture_data_.index = 0;
					gesture_data_.total_gestures = 0;
				}
			}
		} else {

			/* Determine best guessed gesture and clean up */
			osDelay(FIFO_PAUSE_TIME);
			decodeGesture();
			motion = gesture_motion_;
#if DEBUG_APDS
            Serial.print("END: ");
            Serial.println(gesture_motion_);
#endif
			resetGestureParameters();
			return motion;
		}
	}
}

/**
 * Turn the APDS-9960 on
 *
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t enablePower() {
	if (!setMode(POWER, 1)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * Turn the APDS-9960 off
 *
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t disablePower() {
	if (!setMode(POWER, 0)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/*******************************************************************************
 * Ambient light and color sensor controls
 ******************************************************************************/

/**
 * @brief Reads the ambient (clear) light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t APDS9960_ReadLight(uint8_t nSensor, uint8_t *dest) {
	uint8_t buffer[2];

	if (!APDS9960_SetActiveChan(nSensor)){
		return STATUS_FAIL;
	}

	/* Read value from clear channel, low byte register */
	if (!sensorReadDataByte(APDS9960_CDATAL, &buffer[1])) {
		return STATUS_FAIL;
	}
	dest[1] = buffer[1];
	/* Read value from clear channel, high byte register */
	if (!sensorReadDataByte(APDS9960_CDATAH, &buffer[0])) {
		return STATUS_FAIL;
	}
	dest[0] = buffer[0];

	return STATUS_OK;
}


/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t APDS9960_ReadRedLight(uint8_t nSensor, uint8_t *dest) {
	uint8_t buffer[2];

	APDS9960_SetActiveChan(nSensor);

	/* Read value from clear channel, low byte register */
	if (!sensorReadDataByte(APDS9960_RDATAL, &buffer[1])) {
		return STATUS_FAIL;
	}
	dest[1] = buffer[1];
	/* Read value from clear channel, high byte register */
	if (!sensorReadDataByte(APDS9960_RDATAH, &buffer[0])) {
		return STATUS_FAIL;
	}
	dest[0] = buffer[0];
	return STATUS_OK;
}

/**
 * @brief Reads the green light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t APDS9960_ReadGreenLight(uint8_t nSensor, uint8_t *dest) {
	uint8_t buffer[2];

	APDS9960_SetActiveChan(nSensor);

	/* Read value from clear channel, low byte register */
	if (!sensorReadDataByte(APDS9960_GDATAL, &buffer[1])) {
		return STATUS_FAIL;
	}
	dest[1] = buffer[1];
	/* Read value from clear channel, high byte register */
	if (!sensorReadDataByte(APDS9960_GDATAH, &buffer[0])) {
		return STATUS_FAIL;
	}
	dest[0] = buffer[0];

	return STATUS_OK;
}

/**
 * @brief Reads the red light level as a 16-bit value
 *
 * @param[out] val value of the light sensor.
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t APDS9960_ReadBlueLight(uint8_t nSensor, uint8_t *dest) {
	uint8_t buffer[2];

	APDS9960_SetActiveChan(nSensor);

	/* Read value from clear channel, low byte register */
	if (!sensorReadDataByte(APDS9960_BDATAL, &buffer[1])) {
		return STATUS_FAIL;
	}
	dest[1] = buffer[1];
	/* Read value from clear channel, high byte register */
	if (!sensorReadDataByte(APDS9960_BDATAH, &buffer[0])) {
		return STATUS_FAIL;
	}
	dest[0] = buffer[0];
	return STATUS_OK;
}

/*******************************************************************************
 * Proximity sensor controls
 ******************************************************************************/

/**
 * @brief Reads the proximity level as an 8-bit value
 *
 * @param[out] val value of the proximity sensor.
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t readProximity(uint8_t *val) {
	val = 0;

	/* Read value from proximity data register */
	if (!sensorReadDataByte(APDS9960_PDATA, val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/*******************************************************************************
 * High-level gesture controls
 ******************************************************************************/

/**
 * @brief Resets all the parameters in the gesture data member
 */
void resetGestureParameters() {
	gesture_data_.index = 0;
	gesture_data_.total_gestures = 0;

	gesture_ud_delta_ = 0;
	gesture_lr_delta_ = 0;

	gesture_ud_count_ = 0;
	gesture_lr_count_ = 0;

	gesture_near_count_ = 0;
	gesture_far_count_ = 0;

	gesture_state_ = 0;
	gesture_motion_ = DIR_NONE;
}

/**
 * @brief Processes the raw gesture data to determine swipe direction
 *
 * @return STATUS_OK if near or far state seen. STATUS_FAIL otherwise.
 */
uint8_t processGestureData() {
	uint8_t u_first = 0;
	uint8_t d_first = 0;
	uint8_t l_first = 0;
	uint8_t r_first = 0;
	uint8_t u_last = 0;
	uint8_t d_last = 0;
	uint8_t l_last = 0;
	uint8_t r_last = 0;
	int ud_ratio_first;
	int lr_ratio_first;
	int ud_ratio_last;
	int lr_ratio_last;
	int ud_delta;
	int lr_delta;
	int i;

	/* If we have less than 4 total gestures, that's not enough */
	if (gesture_data_.total_gestures <= 4) {
		return STATUS_FAIL;
	}

	/* Check to make sure our data isn't out of bounds */
	if ((gesture_data_.total_gestures <= 32)
			&& (gesture_data_.total_gestures > 0)) {

		/* Find the first value in U/D/L/R above the threshold */
		for (i = 0; i < gesture_data_.total_gestures; i++) {
			if ((gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT)
					&& (gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT)
					&& (gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT)
					&& (gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT)) {

				u_first = gesture_data_.u_data[i];
				d_first = gesture_data_.d_data[i];
				l_first = gesture_data_.l_data[i];
				r_first = gesture_data_.r_data[i];
				break;
			}
		}

		/* If one of the _first values is 0, then there is no good data */
		if ((u_first == 0) || (d_first == 0) || (l_first == 0)
				|| (r_first == 0)) {

			return STATUS_FAIL;
		}
		/* Find the last value in U/D/L/R above the threshold */
		for (i = gesture_data_.total_gestures - 1; i >= 0; i--) {
#if DEBUG_APDS
            Serial.print(F("Finding last: "));
            Serial.print(F("U:"));
            Serial.print(gesture_data_.u_data[i]);
            Serial.print(F(" D:"));
            Serial.print(gesture_data_.d_data[i]);
            Serial.print(F(" L:"));
            Serial.print(gesture_data_.l_data[i]);
            Serial.print(F(" R:"));
            Serial.println(gesture_data_.r_data[i]);
#endif
			if ((gesture_data_.u_data[i] > GESTURE_THRESHOLD_OUT)
					&& (gesture_data_.d_data[i] > GESTURE_THRESHOLD_OUT)
					&& (gesture_data_.l_data[i] > GESTURE_THRESHOLD_OUT)
					&& (gesture_data_.r_data[i] > GESTURE_THRESHOLD_OUT)) {

				u_last = gesture_data_.u_data[i];
				d_last = gesture_data_.d_data[i];
				l_last = gesture_data_.l_data[i];
				r_last = gesture_data_.r_data[i];
				break;
			}
		}
	}

	/* Calculate the first vs. last ratio of up/down and left/right */
	ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
	lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
	ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
	lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);

#if DEBUG_APDS
    Serial.print(F("Last Values: "));
    Serial.print(F("U:"));
    Serial.print(u_last);
    Serial.print(F(" D:"));
    Serial.print(d_last);
    Serial.print(F(" L:"));
    Serial.print(l_last);
    Serial.print(F(" R:"));
    Serial.println(r_last);

    Serial.print(F("Ratios: "));
    Serial.print(F("UD Fi: "));
    Serial.print(ud_ratio_first);
    Serial.print(F(" UD La: "));
    Serial.print(ud_ratio_last);
    Serial.print(F(" LR Fi: "));
    Serial.print(lr_ratio_first);
    Serial.print(F(" LR La: "));
    Serial.println(lr_ratio_last);
#endif

	/* Determine the difference between the first and last ratios */
	ud_delta = ud_ratio_last - ud_ratio_first;
	lr_delta = lr_ratio_last - lr_ratio_first;

#if DEBUG_APDS
    Serial.print("Deltas: ");
    Serial.print("UD: ");
    Serial.print(ud_delta);
    Serial.print(" LR: ");
    Serial.println(lr_delta);
#endif

	/* Accumulate the UD and LR delta values */
	gesture_ud_delta_ += ud_delta;
	gesture_lr_delta_ += lr_delta;

#if DEBUG_APDS
    Serial.print("Accumulations: ");
    Serial.print("UD: ");
    Serial.print(gesture_ud_delta_);
    Serial.print(" LR: ");
    Serial.println(gesture_lr_delta_);
#endif

	/* Determine U/D gesture */
	if (gesture_ud_delta_ >= GESTURE_SENSITIVITY_1) {
		gesture_ud_count_ = 1;
	} else if (gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1) {
		gesture_ud_count_ = -1;
	} else {
		gesture_ud_count_ = 0;
	}

	/* Determine L/R gesture */
	if (gesture_lr_delta_ >= GESTURE_SENSITIVITY_1) {
		gesture_lr_count_ = 1;
	} else if (gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1) {
		gesture_lr_count_ = -1;
	} else {
		gesture_lr_count_ = 0;
	}

	/* Determine Near/Far gesture */
	if ((gesture_ud_count_ == 0) && (gesture_lr_count_ == 0)) {
		if ((abs(ud_delta) < GESTURE_SENSITIVITY_2)
				&& (abs(lr_delta) < GESTURE_SENSITIVITY_2)) {

			if ((ud_delta == 0) && (lr_delta == 0)) {
				gesture_near_count_++;
			} else if ((ud_delta != 0) || (lr_delta != 0)) {
				gesture_far_count_++;
			}

			if ((gesture_near_count_ >= 10) && (gesture_far_count_ >= 2)) {
				if ((ud_delta == 0) && (lr_delta == 0)) {
					gesture_state_ = NEAR_STATE;
				} else if ((ud_delta != 0) && (lr_delta != 0)) {
					gesture_state_ = FAR_STATE;
				}
				return STATUS_OK;
			}
		}
	} else {
		if ((abs(ud_delta) < GESTURE_SENSITIVITY_2)
				&& (abs(lr_delta) < GESTURE_SENSITIVITY_2)) {

			if ((ud_delta == 0) && (lr_delta == 0)) {
				gesture_near_count_++;
			}

			if (gesture_near_count_ >= 10) {
				gesture_ud_count_ = 0;
				gesture_lr_count_ = 0;
				gesture_ud_delta_ = 0;
				gesture_lr_delta_ = 0;
			}
		}
	}

#if DEBUG_APDS
    Serial.print("UD_CT: ");
    Serial.print(gesture_ud_count_);
    Serial.print(" LR_CT: ");
    Serial.print(gesture_lr_count_);
    Serial.print(" NEAR_CT: ");
    Serial.print(gesture_near_count_);
    Serial.print(" FAR_CT: ");
    Serial.println(gesture_far_count_);
    Serial.println("----------");
#endif

	return STATUS_FAIL;
}

/**
 * @brief Determines swipe direction or near/far state
 *
 * @return STATUS_OK if near/far event. STATUS_FAIL otherwise.
 */
uint8_t decodeGesture() {
	/* Return if near or far event is detected */
	if (gesture_state_ == NEAR_STATE) {
		gesture_motion_ = DIR_NEAR;
		return STATUS_OK;
	} else if (gesture_state_ == FAR_STATE) {
		gesture_motion_ = DIR_FAR;
		return STATUS_OK;
	}

	/* Determine swipe direction */
	if ((gesture_ud_count_ == -1) && (gesture_lr_count_ == 0)) {
		gesture_motion_ = DIR_UP;
	} else if ((gesture_ud_count_ == 1) && (gesture_lr_count_ == 0)) {
		gesture_motion_ = DIR_DOWN;
	} else if ((gesture_ud_count_ == 0) && (gesture_lr_count_ == 1)) {
		gesture_motion_ = DIR_RIGHT;
	} else if ((gesture_ud_count_ == 0) && (gesture_lr_count_ == -1)) {
		gesture_motion_ = DIR_LEFT;
	} else if ((gesture_ud_count_ == -1) && (gesture_lr_count_ == 1)) {
		if (abs(gesture_ud_delta_) > abs(gesture_lr_delta_)) {
			gesture_motion_ = DIR_UP;
		} else {
			gesture_motion_ = DIR_RIGHT;
		}
	} else if ((gesture_ud_count_ == 1) && (gesture_lr_count_ == -1)) {
		if (abs(gesture_ud_delta_) > abs(gesture_lr_delta_)) {
			gesture_motion_ = DIR_DOWN;
		} else {
			gesture_motion_ = DIR_LEFT;
		}
	} else if ((gesture_ud_count_ == -1) && (gesture_lr_count_ == -1)) {
		if (abs(gesture_ud_delta_) > abs(gesture_lr_delta_)) {
			gesture_motion_ = DIR_UP;
		} else {
			gesture_motion_ = DIR_LEFT;
		}
	} else if ((gesture_ud_count_ == 1) && (gesture_lr_count_ == 1)) {
		if (abs(gesture_ud_delta_) > abs(gesture_lr_delta_)) {
			gesture_motion_ = DIR_DOWN;
		} else {
			gesture_motion_ = DIR_RIGHT;
		}
	} else {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/*******************************************************************************
 * Getters and setters for register values
 ******************************************************************************/

/**
 * @brief Returns the lower threshold for proximity detection
 *
 * @return lower threshold
 */
uint8_t getProxIntLowThresh() {
	uint8_t val;

	/* Read value from PILT register */
	if (!sensorReadDataByte(APDS9960_PILT, &val)) {
		val = 0;
	}

	return val;
}

/**
 * @brief Sets the lower threshold for proximity detection
 *
 * @param[in] threshold the lower proximity threshold
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setProxIntLowThresh(uint8_t threshold) {
	if (!sensorWriteDataByte(APDS9960_PILT, threshold)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Returns the high threshold for proximity detection
 *
 * @return high threshold
 */
uint8_t getProxIntHighThresh() {
	uint8_t val;

	/* Read value from PIHT register */
	if (!sensorReadDataByte(APDS9960_PIHT, &val)) {
		val = 0;
	}

	return val;
}

/**
 * @brief Sets the high threshold for proximity detection
 *
 * @param[in] threshold the high proximity threshold
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setProxIntHighThresh(uint8_t threshold) {
	if (!sensorWriteDataByte(APDS9960_PIHT, threshold)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Returns LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the value of the LED drive strength. 0xFF on failure.
 */
uint8_t getLEDDrive() {
	uint8_t val;

	/* Read value from CONTROL register */
	if (!sensorReadDataByte(APDS9960_CONTROL, &val)) {
		return ERROR;
	}

	/* Shift and mask out LED drive bits */
	val = (val >> 6) & 0x03;

	return val;
}

/**
 * @brief Sets the LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value (0-3) for the LED drive strength
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setLEDDrive(uint8_t drive) {
	uint8_t val;

	/* Read value from CONTROL register */
	if (!sensorReadDataByte(APDS9960_CONTROL, &val)) {
		return STATUS_FAIL;
	}

	/* Set bits in register to given value */
	drive &= 0x03;
	drive = drive << 6;
	val &= 0x3F;
	val |= drive;

	/* Write register value back into CONTROL register */
	if (!sensorWriteDataByte(APDS9960_CONTROL, val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Returns receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the value of the proximity gain. 0xFF on failure.
 */
uint8_t getProximityGain() {
	uint8_t val;

	/* Read value from CONTROL register */
	if (!sensorReadDataByte(APDS9960_CONTROL, &val)) {
		return ERROR;
	}

	/* Shift and mask out PDRIVE bits */
	val = (val >> 2) & 0x03;

	return val;
}

/**
 * @brief Sets the receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setProximityGain(uint8_t drive) {
	uint8_t val;

	/* Read value from CONTROL register */
	if (!sensorReadDataByte(APDS9960_CONTROL, &val)) {
		return STATUS_FAIL;
	}

	/* Set bits in register to given value */
	drive &= 0x03;
	drive = drive << 2;
	val &= 0xF3;
	val |= drive;

	/* Write register value back into CONTROL register */
	if (!sensorWriteDataByte(APDS9960_CONTROL, val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Returns receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @return the value of the ALS gain. 0xFF on failure.
 */
uint8_t getAmbientLightGain() {
	uint8_t val;

	/* Read value from CONTROL register */
	if (!sensorReadDataByte(APDS9960_CONTROL, &val)) {
		return ERROR;
	}

	/* Shift and mask out ADRIVE bits */
	val &= 0x03;

	return val;
}

/**
 * @brief Sets the receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @param[in] drive the value (0-3) for the gain
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setAmbientLightGain(uint8_t drive) {
	uint8_t val;

	/* Read value from CONTROL register */
	if (!sensorReadDataByte(APDS9960_CONTROL, &val)) {
		return STATUS_FAIL;
	}

	/* Set bits in register to given value */
	drive &= 0x03;
	val &= 0xFC;
	val |= drive;

	/* Write register value back into CONTROL register */
	if (!sensorWriteDataByte(APDS9960_CONTROL, val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Get the current LED boost value
 * 
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @return The LED boost value. 0xFF on failure.
 */
uint8_t getLEDBoost() {
	uint8_t val;

	/* Read value from CONFIG2 register */
	if (!sensorReadDataByte(APDS9960_CONFIG2, &val)) {
		return ERROR;
	}

	/* Shift and mask out LED_BOOST bits */
	val = (val >> 4) & 0x03;

	return val;
}

/**
 * @brief Sets the LED current boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @param[in] drive the value (0-3) for current boost (100-300%)
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setLEDBoost(uint8_t boost) {
	uint8_t val;

	/* Read value from CONFIG2 register */
	if (!sensorReadDataByte(APDS9960_CONFIG2, &val)) {
		return STATUS_FAIL;
	}

	/* Set bits in register to given value */
	boost &= 0x03;
	boost = boost << 4;
	val &= 0xCF;
	val |= boost;

	/* Write register value back into CONFIG2 register */
	if (!sensorWriteDataByte(APDS9960_CONFIG2, val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Gets proximity gain compensation enable
 *
 * @return 1 if compensation is enabled. 0 if not. 0xFF on error.
 */
uint8_t getProxGainCompEnable() {
	uint8_t val;

	/* Read value from CONFIG3 register */
	if (!sensorReadDataByte(APDS9960_CONFIG3, &val)) {
		return ERROR;
	}

	/* Shift and mask out PCMP bits */
	val = (val >> 5) & 0x01;

	return val;
}

/**
 * @brief Sets the proximity gain compensation enable
 *
 * @param[in] enable 1 to enable compensation. 0 to disable compensation.
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setProxGainCompEnable(uint8_t enable) {
	uint8_t val;

	/* Read value from CONFIG3 register */
	if (!sensorReadDataByte(APDS9960_CONFIG3, &val)) {
		return STATUS_FAIL;
	}

	/* Set bits in register to given value */
	enable &= 0x01;
	enable = enable << 5;
	val &= 0xDF;
	val |= enable;

	/* Write register value back into CONFIG3 register */
	if (!sensorWriteDataByte(APDS9960_CONFIG3, val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Gets the current mask for enabled/disabled proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @return Current proximity mask for photodiodes. 0xFF on error.
 */
uint8_t getProxPhotoMask() {
	uint8_t val;

	/* Read value from CONFIG3 register */
	if (!sensorReadDataByte(APDS9960_CONFIG3, &val)) {
		return ERROR;
	}

	/* Mask out photodiode enable mask bits */
	val &= 0x0F;

	return val;
}

/**
 * @brief Sets the mask for enabling/disabling proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @param[in] mask 4-bit mask value
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setProxPhotoMask(uint8_t mask) {
	uint8_t val;

	/* Read value from CONFIG3 register */
	if (!sensorReadDataByte(APDS9960_CONFIG3, &val)) {
		return STATUS_FAIL;
	}

	/* Set bits in register to given value */
	mask &= 0x0F;
	val &= 0xF0;
	val |= mask;

	/* Write register value back into CONFIG3 register */
	if (!sensorWriteDataByte(APDS9960_CONFIG3, val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Gets the entry proximity threshold for gesture sensing
 *
 * @return Current entry proximity threshold.
 */
uint8_t getGestureEnterThresh() {
	uint8_t val;

	/* Read value from GPENTH register */
	if (!sensorReadDataByte(APDS9960_GPENTH, &val)) {
		val = 0;
	}

	return val;
}

/**
 * @brief Sets the entry proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to start gesture mode
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setGestureEnterThresh(uint8_t threshold) {
	if (!sensorWriteDataByte(APDS9960_GPENTH, threshold)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Gets the exit proximity threshold for gesture sensing
 *
 * @return Current exit proximity threshold.
 */
uint8_t getGestureExitThresh() {
	uint8_t val;

	/* Read value from GEXTH register */
	if (!sensorReadDataByte(APDS9960_GEXTH, &val)) {
		val = 0;
	}

	return val;
}

/**
 * @brief Sets the exit proximity threshold for gesture sensing
 *
 * @param[in] threshold proximity value needed to end gesture mode
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setGestureExitThresh(uint8_t threshold) {
	if (!sensorWriteDataByte(APDS9960_GEXTH, threshold)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Gets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @return the current photodiode gain. 0xFF on error.
 */
uint8_t getGestureGain() {
	uint8_t val;

	/* Read value from GCONF2 register */
	if (!sensorReadDataByte(APDS9960_GCONF2, &val)) {
		return ERROR;
	}

	/* Shift and mask out GGAIN bits */
	val = (val >> 5) & 0x03;

	return val;
}

/**
 * @brief Sets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] gain the value for the photodiode gain
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setGestureGain(uint8_t gain) {
	uint8_t val;

	/* Read value from GCONF2 register */
	if (!sensorReadDataByte(APDS9960_GCONF2, &val)) {
		return STATUS_FAIL;
	}

	/* Set bits in register to given value */
	gain &= 0x03;
	gain = gain << 5;
	val &= 0x9F;
	val |= gain;

	/* Write register value back into GCONF2 register */
	if (!sensorWriteDataByte(APDS9960_GCONF2, val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Gets the drive current of the LED during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @return the LED drive current value. 0xFF on error.
 */
uint8_t getGestureLEDDrive() {
	uint8_t val;

	/* Read value from GCONF2 register */
	if (!sensorReadDataByte(APDS9960_GCONF2, &val)) {
		return ERROR;
	}

	/* Shift and mask out GLDRIVE bits */
	val = (val >> 3) & 0x03;

	return val;
}

/**
 * @brief Sets the LED drive current during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] drive the value for the LED drive current
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setGestureLEDDrive(uint8_t drive) {
	uint8_t val;

	/* Read value from GCONF2 register */
	if (!sensorReadDataByte(APDS9960_GCONF2, &val)) {
		return STATUS_FAIL;
	}

	/* Set bits in register to given value */
	drive &= 0x03;
	drive = drive << 3;
	val &= 0xE7;
	val |= drive;

	/* Write register value back into GCONF2 register */
	if (!sensorWriteDataByte(APDS9960_GCONF2, val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Gets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @return the current wait time between gestures. 0xFF on error.
 */
uint8_t getGestureWaitTime() {
	uint8_t val;

	/* Read value from GCONF2 register */
	if (!sensorReadDataByte(APDS9960_GCONF2, &val)) {
		return ERROR;
	}

	/* Mask out GWTIME bits */
	val &= 0x07;

	return val;
}

/**
 * @brief Sets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @param[in] the value for the wait time
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setGestureWaitTime(uint8_t time) {
	uint8_t val;

	/* Read value from GCONF2 register */
	if (!sensorReadDataByte(APDS9960_GCONF2, &val)) {
		return STATUS_FAIL;
	}

	/* Set bits in register to given value */
	time &= 0x07;
	val &= 0xF8;
	val |= time;

	/* Write register value back into GCONF2 register */
	if (!sensorWriteDataByte(APDS9960_GCONF2, val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Gets the low threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t getLightIntLowThreshold(uint16_t *threshold) {
	uint8_t val_byte;
	threshold = 0;

	/* Read value from ambient light low threshold, low byte register */
	if (!sensorReadDataByte(APDS9960_AILTL, &val_byte)) {
		return STATUS_FAIL;
	}
	threshold = threshold + (uint16_t) val_byte;

	/* Read value from ambient light low threshold, high byte register */
	if (!sensorReadDataByte(APDS9960_AILTH, &val_byte)) {
		return STATUS_FAIL;
	}
	threshold = threshold + ((uint16_t) val_byte << 8);

	return STATUS_OK;
}

/**
 * @brief Sets the low threshold for ambient light interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setLightIntLowThreshold(uint16_t threshold) {
	uint8_t val_low;
	uint8_t val_high;

	/* Break 16-bit threshold into 2 8-bit values */
	val_low = threshold & 0x00FF;
	val_high = (threshold & 0xFF00) >> 8;

	/* Write low byte */
	if (!sensorWriteDataByte(APDS9960_AILTL, val_low)) {
		return STATUS_FAIL;
	}

	/* Write high byte */
	if (!sensorWriteDataByte(APDS9960_AILTH, val_high)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Gets the high threshold for ambient light interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t getLightIntHighThreshold(uint16_t *threshold) {
	uint8_t val_byte;
	threshold = 0;

	/* Read value from ambient light high threshold, low byte register */
	if (!sensorReadDataByte(APDS9960_AIHTL, &val_byte)) {
		return STATUS_FAIL;
	}
	threshold = threshold + ((uint16_t) val_byte);

	/* Read value from ambient light high threshold, high byte register */
	if (!sensorReadDataByte(APDS9960_AIHTH, &val_byte)) {
		return STATUS_FAIL;
	}
	threshold = threshold + ((uint16_t) val_byte << 8);

	return STATUS_OK;
}

/**
 * @brief Sets the high threshold for ambient light interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setLightIntHighThreshold(uint16_t threshold) {
	uint8_t val_low;
	uint8_t val_high;

	/* Break 16-bit threshold into 2 8-bit values */
	val_low = threshold & 0x00FF;
	val_high = (threshold & 0xFF00) >> 8;

	/* Write low byte */
	if (!sensorWriteDataByte(APDS9960_AIHTL, val_low)) {
		return STATUS_FAIL;
	}

	/* Write high byte */
	if (!sensorWriteDataByte(APDS9960_AIHTH, val_high)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Gets the low threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t getProximityIntLowThreshold(uint8_t *threshold) {
	threshold = 0;

	/* Read value from proximity low threshold register */
	if (!sensorReadDataByte(APDS9960_PILT, threshold)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Sets the low threshold for proximity interrupts
 *
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setProximityIntLowThreshold(uint8_t threshold) {

	/* Write threshold value to register */
	if (!sensorWriteDataByte(APDS9960_PILT, threshold)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Gets the high threshold for proximity interrupts
 *
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t getProximityIntHighThreshold(uint8_t *threshold) {
	threshold = 0;

	/* Read value from proximity low threshold register */
	if (!sensorReadDataByte(APDS9960_PIHT, threshold)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Sets the high threshold for proximity interrupts
 *
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setProximityIntHighThreshold(uint8_t threshold) {

	/* Write threshold value to register */
	if (!sensorWriteDataByte(APDS9960_PIHT, threshold)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Gets if ambient light interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t getAmbientLightIntEnable() {
	uint8_t val;

	/* Read value from ENABLE register */
	if (!sensorReadDataByte(APDS9960_ENABLE, &val)) {
		return ERROR;
	}

	/* Shift and mask out AIEN bit */
	val = (val >> 4) & 0x01;

	return val;
}

/**
 * @brief Turns ambient light interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setAmbientLightIntEnable(uint8_t enable) {
	uint8_t val;

	/* Read value from ENABLE register */
	if (!sensorReadDataByte(APDS9960_ENABLE, &val)) {
		return STATUS_FAIL;
	}

	/* Set bits in register to given value */
	enable &= 0x01;
	enable = enable << 4;
	val &= 0xEF;
	val |= enable;

	/* Write register value back into ENABLE register */
	if (!sensorWriteDataByte(APDS9960_ENABLE, val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Gets if proximity interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t getProximityIntEnable() {
	uint8_t val;

	/* Read value from ENABLE register */
	if (!sensorReadDataByte(APDS9960_ENABLE, &val)) {
		return ERROR;
	}

	/* Shift and mask out PIEN bit */
	val = (val >> 5) & 0x01;

	return val;
}

/**
 * @brief Turns proximity interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setProximityIntEnable(uint8_t enable) {
	uint8_t val;

	/* Read value from ENABLE register */
	if (!sensorReadDataByte(APDS9960_ENABLE, &val)) {
		return STATUS_FAIL;
	}

	/* Set bits in register to given value */
	enable &= 0x01;
	enable = enable << 5;
	val &= 0xDF;
	val |= enable;

	/* Write register value back into ENABLE register */
	if (!sensorWriteDataByte(APDS9960_ENABLE, val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Gets if gesture interrupts are enabled or not
 *
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t getGestureIntEnable() {
	uint8_t val;

	/* Read value from GCONF4 register */
	if (!sensorReadDataByte(APDS9960_GCONF4, &val)) {
		return ERROR;
	}

	/* Shift and mask out GIEN bit */
	val = (val >> 1) & 0x01;

	return val;
}

/**
 * @brief Turns gesture-related interrupts on or off
 *
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setGestureIntEnable(uint8_t enable) {
	uint8_t val;

	/* Read value from GCONF4 register */
	if (!sensorReadDataByte(APDS9960_GCONF4, &val)) {
		return STATUS_FAIL;
	}

	/* Set bits in register to given value */
	enable &= 0x01;
	enable = enable << 1;
	val &= 0xFD;
	val |= enable;

	/* Write register value back into GCONF4 register */
	if (!sensorWriteDataByte(APDS9960_GCONF4, val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Clears the ambient light interrupt
 *
 * @return STATUS_OK if operation completed successfully. STATUS_FAIL otherwise.
 */
uint8_t clearAmbientLightInt() {
	uint8_t throwaway;
	if (!sensorReadDataByte(APDS9960_AICLEAR, &throwaway)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Clears the proximity interrupt
 *
 * @return STATUS_OK if operation completed successfully. STATUS_FAIL otherwise.
 */
uint8_t clearProximityInt() {
	uint8_t throwaway;
	if (!sensorReadDataByte(APDS9960_PICLEAR, &throwaway)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/**
 * @brief Tells if the gesture state machine is currently running
 *
 * @return 1 if gesture state machine is running, 0 if not. 0xFF on error.
 */
uint8_t getGestureMode() {
	uint8_t val;

	/* Read value from GCONF4 register */
	if (!sensorReadDataByte(APDS9960_GCONF4, &val)) {
		return ERROR;
	}

	/* Mask out GMODE bit */
	val &= 0x01;

	return val;
}

/**
 * @brief Tells the state machine to either enter or exit gesture state machine
 *
 * @param[in] mode 1 to enter gesture state machine, 0 to exit.
 * @return STATUS_OK if operation successful. STATUS_FAIL otherwise.
 */
uint8_t setGestureMode(uint8_t mode) {
	uint8_t val;

	/* Read value from GCONF4 register */
	if (!sensorReadDataByte(APDS9960_GCONF4, &val)) {
		return STATUS_FAIL;
	}

	/* Set bits in register to given value */
	mode &= 0x01;
	val &= 0xFE;
	val |= mode;

	/* Write register value back into GCONF4 register */
	if (!sensorWriteDataByte(APDS9960_GCONF4, val)) {
		return STATUS_FAIL;
	}

	return STATUS_OK;
}

/*******************************************************************************
 * Raw I2C Reads and Writes
 ******************************************************************************/

/**********************************************************************************************
 * @brief  sensorWriteDataByte(uint16_t Register_Addr,uint8_t Register_Data)
 * @param  Register_Addr : Register Address
 * @param  Register_Data : Register data
 * @param  return Write state,Write success is 1,Write fail is 0
 **********************************************************************************************/
uint8_t mpxSetReg(uint8_t Register_Addr) {
	if (HAL_I2C_Master_Transmit(&hi2c1, MPX_I2C_ADDR, &Register_Addr, 1, 5)
			== HAL_OK)
		return STATUS_OK;
	else
		return STATUS_FAIL;
}

/**********************************************************************************************
 * @brief  sensorWriteDataByte(uint16_t Register_Addr,uint8_t Register_Data)
 * @param  Register_Addr : Register Address
 * @param  Register_Data : Register data
 * @param  return Write state,Write success is 1,Write fail is 0
 **********************************************************************************************/
uint8_t sensorWriteDataByte(uint16_t Register_Addr, uint8_t Register_Data) {
	if (HAL_I2C_Mem_Write(&hi2c1, APDS9960_I2C_ADDR, Register_Addr,
			I2C_MEMADD_SIZE_8BIT, &Register_Data, 1, 5) == HAL_OK)
		return STATUS_OK;
	else
		return STATUS_FAIL;
}

/**********************************************************************************************
 * @brief  sensorReadDataByte(uint16_t Register_Addr)
 * @param  Register_Addr : Register Address
 * @param
 * @param  return read data (uchar)
 **********************************************************************************************/
uint8_t sensorReadDataByte(uint16_t Register_Addr, uint8_t *Register_Data) {
	if (HAL_I2C_Mem_Read(&hi2c1, APDS9960_I2C_ADDR, Register_Addr,
			I2C_MEMADD_SIZE_8BIT, Register_Data, 1, 5) == HAL_OK) {
		return STATUS_OK;
	}

	else {
		return STATUS_FAIL;
	}

}

/**********************************************************************************************
 * @brief  sensor_read(uint16_t Register_Addr)
 * @param  Register_Addr : Register Address
 * @param
 * @param  return read data (uchar)
 **********************************************************************************************/
uint8_t sensorReadDataBlock(uint16_t Register_Addr, uint8_t *Register_Data,
		uint16_t Size) {
	if (HAL_I2C_Mem_Read(&hi2c1, APDS9960_I2C_ADDR, Register_Addr,
			I2C_MEMADD_SIZE_8BIT, Register_Data, Size, 5) == HAL_OK)
		return STATUS_OK;
	else
		return STATUS_FAIL;
}
