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

#include "kubos-hal/I2C.h"
#include "kubos-core/modules/sensors/htu21d/htu21d.h"

#include "FreeRTOS.h"
#include "task.h"

double readHumidity()
{
	uint8_t I2Cptr;
	uint8_t array[HTU21D_RX_SIZE];
	uint8_t* arrayp;

	I2Cptr = TRIGGER_HUMD_MEASURE_NOHOLD;

	/* transmit to sensor */
	k_master_transmit_i2c(HTDU21D_DEV_NUM, &I2Cptr, HTU21D_TX_SIZE);

	/* wait for computation */
	vTaskDelay(55);

	/* set array pointer */
	arrayp = array;
	/* get three uint8_ts, data(MSB) / data(LSB) / Checksum */
	k_master_receive_i2c(HTDU21D_DEV_NUM, arrayp, HTU21D_RX_SIZE);

	unsigned int rawHumidity = ((unsigned int) array[0] << 8) | (unsigned int) array[1];

	//sensorStatus = rawHumidity & 0x0003; //Grab only the right two bits
	rawHumidity &= 0xFFFC; //Zero out the status bits but keep them in place

	/* Given the raw humidity data, calculate the actual relative humidity */
	double tempRH = rawHumidity / (double)65536; //2^16 = 65536
	double rh = -6 + (125 * tempRH); //From page 14

	return rh;
}

double readTemperature()
{
	uint8_t I2Cptr;
	uint8_t array[HTU21D_RX_SIZE];
	uint8_t* arrayp;

	I2Cptr = TRIGGER_TEMP_MEASURE_NOHOLD;

	/* transmit to sensor */
	k_master_transmit_i2c(HTDU21D_DEV_NUM, &I2Cptr, HTU21D_TX_SIZE);

	/* wait for computation */
	vTaskDelay(55);

	/* set array pointer */
	arrayp = array;
	/* get three uint8_ts, data(MSB) / data(LSB) / Checksum */
	k_master_receive_i2c(HTDU21D_DEV_NUM, arrayp, HTU21D_RX_SIZE);

	unsigned int rawTemperature = ((unsigned int) array[0] << 8)
			| (unsigned int) array[1];

	//sensorStatus = rawTemperature & 0x0003; //Grab only the right two bits
	rawTemperature &= 0xFFFC; //Zero out the status bits but keep them in place

	//Given the raw temperature data, calculate the actual temperature
	double tempTemperature = rawTemperature / (double) 65536; //2^16 = 65536
	double realTemperature = (double) (-46.85 + (175.72 * tempTemperature)); //From page 14

	return (realTemperature);
}

void setResolution(uint8_t resolution)
{
	uint8_t userRegister = read_user_register(); //Go get the current register state
	uint8_t I2Cptr;
	userRegister &= 0x7E; //Turn off the resolution bits B01111110 = 0x7E
	resolution &= 0x81; //Turn off all other bits but resolution bits B10000001 = 0x81
	userRegister |= resolution; //Mask in the requested resolution bits

	/* set value */
	I2Cptr = WRITE_USER_REG;

	/* transmit to sensor */
	k_master_transmit_i2c(HTDU21D_DEV_NUM, &I2Cptr, HTU21D_TX_SIZE);
	k_master_transmit_i2c(HTDU21D_DEV_NUM, &userRegister, HTU21D_TX_SIZE);
}

//Read the user register
uint8_t read_user_register(void)
{
	uint8_t I2Cptr;
	uint8_t userRegister;

	/* set value */
	I2Cptr = READ_USER_REG;

	/* transmit to sensor */
	k_master_transmit_i2c(HTDU21D_DEV_NUM, &I2Cptr, HTU21D_TX_SIZE);
	/* receive result */
	k_master_receive_i2c(HTDU21D_DEV_NUM, &userRegister, HTU21D_RX_SIZE);

	return (userRegister);
}
