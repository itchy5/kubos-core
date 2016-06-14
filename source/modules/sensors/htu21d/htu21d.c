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
#include "task.h"

static uint8_t check_crc(uint16_t message_from_sensor, uint8_t check_value_from_sensor);

double readHumidity()
{
	uint8_t I2Cptr;
	KI2C *k_i2c = kprv_i2c_get(DEFAULT_I2C);
	uint8_t array[HTU21D_RX_SIZE];

	I2Cptr = TRIGGER_HUMD_MEASURE_NOHOLD;

	/* transmit to sensor */
	k_master_transmit_interrupt_i2c(DEFAULT_I2C, HTDU21D_ADDRESS, &I2Cptr, 1);

	/* wait for computation */
	vTaskDelay(55);

	/* get three uint8_ts, data(MSB) / data(LSB) / Checksum */
	k_master_receive_interrupt_i2c(DEFAULT_I2C, HTDU21D_ADDRESS);

	int i = 0;
	uint8_t* p = array;

	/* get data from I2C queue */
	uint8_t test;
	for (; i < HTU21D_RX_SIZE; i++, p++)
	{
		xQueueReceive(k_i2c->rx_queue, &test, 100);
	}

	unsigned int rawHumidity = ((unsigned int) array[0] << 8) | (unsigned int) array[1];

	/*
	if(check_crc(rawHumidity, array[2]) != 0)
	{
		return(-1); //Error out
	}
	*/

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
	KI2C *k_i2c = kprv_i2c_get(DEFAULT_I2C);
	uint8_t array[HTU21D_RX_SIZE];

	I2Cptr = TRIGGER_TEMP_MEASURE_NOHOLD;

	/* transmit to sensor */
	k_master_transmit_interrupt_i2c(DEFAULT_I2C, HTDU21D_ADDRESS, &I2Cptr, 1);

	/* wait for computation */
	vTaskDelay(55);

	/* get three uint8_ts, data(MSB) / data(LSB) / Checksum */
	k_master_receive_interrupt_i2c(DEFAULT_I2C, HTDU21D_ADDRESS);

	int i = 0;
	uint8_t* p = array;

	/* get data from I2C queue */
	for (; i < HTU21D_RX_SIZE; i++, p++)
		xQueueReceive(k_i2c->rx_queue, p, portMAX_DELAY);

	unsigned int rawTemperature = ((unsigned int) array[0] << 8) | (unsigned int) array[1];

	if(check_crc(rawTemperature, array[2]) != 0) return(-1); //Error out

		//sensorStatus = rawTemperature & 0x0003; //Grab only the right two bits
	rawTemperature &= 0xFFFC; //Zero out the status bits but keep them in place

		//Given the raw temperature data, calculate the actual temperature
	double tempTemperature = rawTemperature / (double)65536; //2^16 = 65536
	double realTemperature = (double)(-46.85 + (175.72 * tempTemperature)); //From page 14

	return(realTemperature);
}

#define SHIFTED_DIVISOR 0x988000
static uint8_t check_crc(uint16_t message_from_sensor, uint8_t check_value_from_sensor)
{
	uint32_t remainder = (uint32_t)message_from_sensor << 8; //Pad with 8 bits because we have to add in the check value
	remainder |= check_value_from_sensor; //Add on the check value

	uint32_t divsor = (uint32_t)SHIFTED_DIVISOR;

	int i = 0;
	for (; i < 16 ; i++) //Operate on only 16 positions of max 24. The remaining 8 are our remainder and should be zero when we're done.
	{

	if( remainder & (uint32_t)1<<(23 - i) ) //Check if there is a one in the left position
		remainder ^= divsor;

	    divsor >>= 1; //Rotate the divsor max 16 times so that we have 8 bits left of a remainder
	}

	return (uint8_t)remainder;
}
