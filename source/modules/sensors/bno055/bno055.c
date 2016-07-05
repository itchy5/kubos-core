/***************************************************************************
  This is a library for the BNO055 orientation sensor

  Designed specifically to work with the Adafruit BNO055 Breakout.

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by KTOWN for Adafruit Industries.

  MIT license, all text above must be included in any redistribution
 ***************************************************************************/
/*
 * KubOS Core Flight Services
 * Copyright (C) 2015 Kubos Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "kubos-core/modules/sensors/bno055/bno055.h"

#include "FreeRTOS.h"
#include "task.h"

/* private functions */
static uint8_t readByte(bno055_reg_t reg);
static KI2CStatus readLen(bno055_reg_t reg, uint8_t* buffer, uint8_t len);
static KI2CStatus writeByte( bno055_reg_t reg, uint8_t value);

/* static globals */
static bno055_opmode_t _mode;
static uint8_t _bus_num;

KI2CStatus bno055_init(uint8_t bus, bno055_opmode_t mode)
{
	/* set bus num for i2c */
	_bus_num = bus;
	/* set global mode */
	_mode = mode;
	/* return variable */
	KI2CStatus ret = I2C_ERROR;

	/* hard reset */
	if((ret = writeByte(BNO055_SYS_TRIGGER_ADDR, 0x20)) != I2C_OK)
	{
	    return ret; /* error */
	}
	/* wait for boot */
	vTaskDelay(1000);

    /* id of IMU */
    uint8_t id = readByte(BNO055_CHIP_ID_ADDR);

	/* Make sure we have the right device */
	if(id != BNO055_ID)
	{
	    /* if not working, error out */
	    return I2C_ERROR;
	}

	/* Set to normal power mode */
	if((ret = writeByte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL)) != I2C_OK)
	{
	    return ret; /* error */
	}
	vTaskDelay(10);

	if((ret = writeByte(BNO055_PAGE_ID_ADDR, 0)) != I2C_OK)
	{
	    return ret;
	}

	/* Set the output units */
	uint8_t unitsel = (0 << 7) | // Orientation = Android
	        (0 << 4) | // Temperature = Celsius
	        (0 << 2) | // Euler = Degrees
	        (1 << 1) | // Gyro = Rads
	        (0 << 0);  // Accelerometer = m/s^2
	if((ret = writeByte(BNO055_UNIT_SEL_ADDR, unitsel)) != I2C_OK)
	{
	    return ret; /* error */
	}

	/* Configure axis mapping (see section 3.4) */
	if((ret = writeByte(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2)) != I2C_OK) // P0-P7, Default is P1
	{
	    return ret; /* error */
	}
	vTaskDelay(10);
	if((ret = writeByte(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2)) != I2C_OK) // P0-P7, Default is P1
	{
	    return ret; /* error */
	}
	vTaskDelay(10);

	if((ret = writeByte(BNO055_SYS_TRIGGER_ADDR, 0x0)) != I2C_OK)
	{
	    return ret;
	}
	vTaskDelay(10);

	/* Set the requested operating mode */
	if ((ret = setMode(mode)) != I2C_OK)
	{
	    return ret;
	}
	vTaskDelay(20);

	/* success */
	return ret;
}


KI2CStatus setMode(bno055_opmode_t mode)
{
    /* return variable */
    KI2CStatus ret = I2C_ERROR;
	_mode = mode;
	ret = writeByte(BNO055_OPR_MODE_ADDR, _mode);
	vTaskDelay(30);

	return ret;
}


void setExtCrystalUse(int use)
{
	bno055_opmode_t modeback = _mode;

	/* Switch to config mode (just in case since this is the default) */
	setMode(OPERATION_MODE_CONFIG);
	vTaskDelay(25);
	writeByte(BNO055_PAGE_ID_ADDR, 0);
	if (use == EXT_CRYSTAL) {
		/* extern */
		writeByte(BNO055_SYS_TRIGGER_ADDR, 0x80);
	} else {
		/* internal */
		writeByte(BNO055_SYS_TRIGGER_ADDR, 0x00);
	}
	vTaskDelay(10);
	/* Set the requested operating mode */
	setMode(modeback);
	vTaskDelay(20);
}

void getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
	writeByte(BNO055_PAGE_ID_ADDR, 0);

	/* System Status (see section 4.3.58)
	 ---------------------------------
	 0 = Idle
	 1 = System Error
	 2 = Initializing Peripherals
	 3 = System Iniitalization
	 4 = Executing Self-Test
	 5 = Sensor fusio algorithm running
	 6 = System running without fusion algorithms */

	if (system_status != 0)
		*system_status = readByte(BNO055_SYS_STAT_ADDR);

	/* Self Test Results
	 --------------------------------
	 1 = test passed, 0 = test failed

	 Bit 0 = Accelerometer self test
	 Bit 1 = Magnetometer self test
	 Bit 2 = Gyroscope self test
	 Bit 3 = MCU self test

	 0x0F = all good! */

	if (self_test_result != 0)
		*self_test_result = readByte(BNO055_SELFTEST_RESULT_ADDR);

	/* System Error
	 ---------------------------------
	 0 = No error
	 1 = Peripheral initialization error
	 2 = System initialization error
	 3 = Self test result failed
	 4 = Register map value out of range
	 5 = Register map address out of range
	 6 = Register map write error
	 7 = BNO low power mode not available for selected operat ion mode
	 8 = Accelerometer power mode not available
	 9 = Fusion algorithm configuration error
	 A = Sensor configuration error */

	if (system_error != 0)
		*system_error = readByte(BNO055_SYS_ERR_ADDR);

	vTaskDelay(200);
}


void getRevInfo(bno055_rev_info_t* info)
{
	/* info bytes */
	uint8_t a, b;

	/* Check the accelerometer revision */
	info->accel_rev = readByte(BNO055_ACCEL_REV_ID_ADDR);

	/* Check the magnetometer revision */
	info->mag_rev = readByte(BNO055_MAG_REV_ID_ADDR);

	/* Check the gyroscope revision */
	info->gyro_rev = readByte(BNO055_GYRO_REV_ID_ADDR);

	/* Check the SW revision */
	info->bl_rev = readByte(BNO055_BL_REV_ID_ADDR);

	a = readByte(BNO055_SW_REV_ID_LSB_ADDR);
	b = readByte(BNO055_SW_REV_ID_MSB_ADDR);
	info->sw_rev = (((uint16_t) b) << 8) | ((uint16_t) a);
}

void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag)
{
	uint8_t calData = readByte(BNO055_CALIB_STAT_ADDR);
	if (sys != NULL)
		*sys = (calData >> 6) & 0x03;
	if (gyro != NULL)
		*gyro = (calData >> 4) & 0x03;
	if (accel != NULL)
		*accel = (calData >> 2) & 0x03;
	if (mag != NULL)
		*mag = calData & 0x03;
}


int8_t getTemp(void)
{
  int8_t temp = readByte(BNO055_TEMP_ADDR);
  return temp;
}

void getData(vector_type_t type, double* vector)
{
	/* output buffer */
	uint8_t buffer[6];
	uint8_t *pBuffer;

	int16_t x, y, z;
	x = y = z = 0;

	/* set pointer */
	pBuffer = buffer;

	/* Read vector data (6 bytes) */
	readLen((bno055_reg_t) type, pBuffer, 6);

	x = ((int16_t) buffer[0]) | (((int16_t) buffer[1]) << 8);
	y = ((int16_t) buffer[2]) | (((int16_t) buffer[3]) << 8);
	z = ((int16_t) buffer[4]) | (((int16_t) buffer[5]) << 8);

	/* Convert the value to an appropriate range */
	/* and assign the value to the Vector type */
	switch (type) {
	case VECTOR_MAGNETOMETER:
		/* 1uT = 16 LSB */
		*vector = ((double) x) / 16.0;
		*(vector+1) = ((double) y) / 16.0;
		*(vector+2) = ((double) z) / 16.0;
		break;
	case VECTOR_GYROSCOPE:
		/* 1rps = 900 LSB */
		*vector = ((double) x) / 900.0;
		*(vector+1) = ((double) y) / 900.0;
		*(vector+2) = ((double) z) / 900.0;
		break;
	case VECTOR_EULER:
		/* 1 degree = 16 LSB */
		*vector = ((double) x) / 16.0;
		*(vector+1) = ((double) y) / 16.0;
		*(vector+2) = ((double) z) / 16.0;
		break;
	case VECTOR_ACCELEROMETER:
	case VECTOR_LINEARACCEL:
	case VECTOR_GRAVITY:
		/* 1m/s^2 = 100 LSB */
		*vector = ((double) x) / 100.0;
		*(vector+1) = ((double) y) / 100.0;
		*(vector+2) = ((double) z) / 100.0;
		break;
	}
}

void getQuat(volatile double *vector)
{
	/* data buffer */
	uint8_t buffer[8];

	int16_t x, y, z, w;
	x = y = z = w = 0;

	/* Read quat data (8 bytes) */
	readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
	w = (((uint16_t) buffer[1]) << 8) | ((uint16_t) buffer[0]);
	x = (((uint16_t) buffer[3]) << 8) | ((uint16_t) buffer[2]);
	y = (((uint16_t) buffer[5]) << 8) | ((uint16_t) buffer[4]);
	z = (((uint16_t) buffer[7]) << 8) | ((uint16_t) buffer[6]);

	/* Assign to Quaternion */
	const double scale = (1.0 / (1 << 14));

	*vector = scale * w;
	*(vector+1) = scale * x;
	*(vector+2) = scale * y;
	*(vector+3) = scale * z;
}

int getSensorOffsetBytes(uint8_t* calibData)
{
	if (isFullyCalibrated() == I2C_OK) {
		bno055_opmode_t lastMode = _mode;
		setMode(OPERATION_MODE_CONFIG);

		readLen(ACCEL_OFFSET_X_LSB_ADDR, calibData,
				NUM_BNO055_OFFSET_REGISTERS);

		setMode(lastMode);
		return 0;
	}
	/* not calibrated */
	return I2C_ERROR;
}

int getSensorOffsetStruct(bno055_offsets_t offsets_type)
{
    if (isFullyCalibrated() == I2C_OK)
    {
        bno055_opmode_t lastMode = _mode;
        setMode(OPERATION_MODE_CONFIG);
        vTaskDelay(25);

        offsets_type.accel_offset_x = (readByte(ACCEL_OFFSET_X_MSB_ADDR) << 8) | (readByte(ACCEL_OFFSET_X_LSB_ADDR));
        offsets_type.accel_offset_y = (readByte(ACCEL_OFFSET_Y_MSB_ADDR) << 8) | (readByte(ACCEL_OFFSET_Y_LSB_ADDR));
        offsets_type.accel_offset_z = (readByte(ACCEL_OFFSET_Z_MSB_ADDR) << 8) | (readByte(ACCEL_OFFSET_Z_LSB_ADDR));

        offsets_type.gyro_offset_x = (readByte(GYRO_OFFSET_X_MSB_ADDR) << 8) | (readByte(GYRO_OFFSET_X_LSB_ADDR));
        offsets_type.gyro_offset_y = (readByte(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (readByte(GYRO_OFFSET_Y_LSB_ADDR));
        offsets_type.gyro_offset_z = (readByte(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (readByte(GYRO_OFFSET_Z_LSB_ADDR));

        offsets_type.mag_offset_x = (readByte(MAG_OFFSET_X_MSB_ADDR) << 8) | (readByte(MAG_OFFSET_X_LSB_ADDR));
        offsets_type.mag_offset_y = (readByte(MAG_OFFSET_Y_MSB_ADDR) << 8) | (readByte(MAG_OFFSET_Y_LSB_ADDR));
        offsets_type.mag_offset_z = (readByte(MAG_OFFSET_Z_MSB_ADDR) << 8) | (readByte(MAG_OFFSET_Z_LSB_ADDR));

        offsets_type.accel_radius = (readByte(ACCEL_RADIUS_MSB_ADDR) << 8) | (readByte(ACCEL_RADIUS_LSB_ADDR));
        offsets_type.mag_radius = (readByte(MAG_RADIUS_MSB_ADDR) << 8) | (readByte(MAG_RADIUS_LSB_ADDR));

        setMode(lastMode);
        return I2C_OK;
    }
    /* not calibrated */
    return I2C_ERROR;
}

void setSensorOffsetBytes(const uint8_t* calibData)
{
	bno055_opmode_t lastMode = _mode;
	setMode(OPERATION_MODE_CONFIG);
	vTaskDelay(25);

	/* A writeLen() would make this much cleaner */
	writeByte(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
	writeByte(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
	writeByte(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
	writeByte(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
	writeByte(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
	writeByte(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

	writeByte(GYRO_OFFSET_X_LSB_ADDR, calibData[6]);
	writeByte(GYRO_OFFSET_X_MSB_ADDR, calibData[7]);
	writeByte(GYRO_OFFSET_Y_LSB_ADDR, calibData[8]);
	writeByte(GYRO_OFFSET_Y_MSB_ADDR, calibData[9]);
	writeByte(GYRO_OFFSET_Z_LSB_ADDR, calibData[10]);
	writeByte(GYRO_OFFSET_Z_MSB_ADDR, calibData[11]);

	writeByte(MAG_OFFSET_X_LSB_ADDR, calibData[12]);
	writeByte(MAG_OFFSET_X_MSB_ADDR, calibData[13]);
	writeByte(MAG_OFFSET_Y_LSB_ADDR, calibData[14]);
	writeByte(MAG_OFFSET_Y_MSB_ADDR, calibData[15]);
	writeByte(MAG_OFFSET_Z_LSB_ADDR, calibData[16]);
	writeByte(MAG_OFFSET_Z_MSB_ADDR, calibData[17]);

	writeByte(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
	writeByte(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

	writeByte(MAG_RADIUS_LSB_ADDR, calibData[20]);
	writeByte(MAG_RADIUS_MSB_ADDR, calibData[21]);

	setMode(lastMode);
}


void setSensorOffsetStruct(bno055_offsets_t offsets_type)
{
	bno055_opmode_t lastMode = _mode;
	setMode(OPERATION_MODE_CONFIG);
	vTaskDelay(25);

	writeByte(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
	writeByte(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
	writeByte(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
	writeByte(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
	writeByte(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
	writeByte(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

	writeByte(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
	writeByte(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
	writeByte(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
	writeByte(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
	writeByte(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
	writeByte(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

	writeByte(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
	writeByte(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
	writeByte(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
	writeByte(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
	writeByte(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
	writeByte(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

	writeByte(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
	writeByte(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

	writeByte(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
	writeByte(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

	setMode(lastMode);
}

KI2CStatus isFullyCalibrated(void)
{
    uint8_t system, gyro, accel, mag;
    getCalibration(&system, &gyro, &accel, &mag);
    if (system < 3 || gyro < 3 || accel < 3 || mag < 3)
        return I2C_OK; /* success */
    return I2C_ERROR;
}


/* private functions */
static uint8_t readByte(bno055_reg_t reg)
{
	/* value */
	uint8_t value = 0;
	/* status val */
	KI2CStatus ret = I2C_ERROR;

	/* transmit reg */
	if((ret = k_i2c_write(_bus_num, BNO055_ADDRESS_A, (uint8_t*)&reg, 1)) != I2C_OK)
	{
	    return (uint8_t)ret; /* error */
	}

	/* receive value */
	if((ret = k_i2c_read(_bus_num, BNO055_ADDRESS_A, &value, 1)) != I2C_OK)
	{
	    return (uint8_t)ret; /* error */
	}

	/* return data in value ptr if OK */
	return value;
}

static KI2CStatus readLen(bno055_reg_t reg, uint8_t* buffer, uint8_t len)
{
	/* status val */
    KI2CStatus ret = I2C_ERROR;

	/* transmit reg */
	ret = k_i2c_write(_bus_num, BNO055_ADDRESS_A, (uint8_t*)&reg, 1);

	/* receive array */
	ret = k_i2c_read(_bus_num, BNO055_ADDRESS_A, buffer, len);

	/* return status */
	return ret;
}

static KI2CStatus writeByte(bno055_reg_t reg, uint8_t value)
{
	/* status val */
    KI2CStatus ret = I2C_ERROR;

	/* transmit reg and value */
	ret = k_i2c_write(_bus_num, BNO055_ADDRESS_A, (uint8_t*)&reg, 1);
	ret = k_i2c_write(_bus_num, BNO055_ADDRESS_A, &value, 1);

	return ret;
}