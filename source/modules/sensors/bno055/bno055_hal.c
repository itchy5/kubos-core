/*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* bno055_support.c
* Date: 2016/03/14
* Revision: 1.0.4 $
*
* Usage: Sensor Driver support file for BNO055 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/

#include "kubos-core/modules/sensors/bno055/bno055_hal.h"
/* kubos HAL */
#include "kubos-hal/I2C.h" /* hal header */

/*---------------------------------------------------------------------------*/
struct bno055_t bno055;

s32 bno055_data_init(void)
{
	/* Variable used to return value of
		communication routine*/
	s32 comres = BNO055_ERROR;
	/* variable used to set the power mode of the sensor*/
	u8 power_mode = BNO055_INIT_VALUE;

	I2C_routine();
	comres = bno055_init(&bno055);

	power_mode = BNO055_POWER_MODE_NORMAL;
		/* set the power mode as NORMAL*/
	comres += bno055_set_power_mode(power_mode);

	return comres;
}

s8 I2C_routine(void)
{

	bno055.bus_write = BNO055_I2C_bus_write;
	bno055.bus_read = BNO055_I2C_bus_read;
	bno055.delay_msec = BNO055_delay_msek;
	bno055.dev_addr = BNO055_I2C_ADDR1;

	return BNO055_INIT_VALUE;
}

double bno055_get_data(void)
{
	/* Variable used to return value of
			communication routine*/
	double comres = BNO055_ERROR;

	/*************read accel converted data***************/
	/* variable used to read the accel x data output as m/s2 or mg */
	double d_accel_datax = BNO055_INIT_VALUE;
	/* variable used to read the accel y data output as m/s2 or mg */
	double d_accel_datay = BNO055_INIT_VALUE;
	/* variable used to read the accel z data output as m/s2 or mg */
	double d_accel_dataz = BNO055_INIT_VALUE;
	/* structure used to read the accel xyz data output as m/s2 or mg */
	struct bno055_accel_double_t d_accel_xyz;


	/*	API used to read accel data output as double  - m/s2 and mg
		float functions also available in the BNO055 API */
		comres += bno055_convert_double_accel_x_msq(&d_accel_datax);
		comres += bno055_convert_double_accel_x_mg(&d_accel_datax);
		comres += bno055_convert_double_accel_y_msq(&d_accel_datay);
		comres += bno055_convert_double_accel_y_mg(&d_accel_datay);
		comres += bno055_convert_double_accel_z_msq(&d_accel_dataz);
		comres += bno055_convert_double_accel_z_mg(&d_accel_dataz);
		comres += bno055_convert_double_accel_xyz_msq(&d_accel_xyz);
		comres += bno055_convert_double_accel_xyz_mg(&d_accel_xyz);

	return comres;
}

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 BNO055_iERROR = BNO055_INIT_VALUE;
	u8 array[I2C_QUEUE_LEN];
	u8 stringpos = BNO055_INIT_VALUE;
	u8 *p;

	array[BNO055_INIT_VALUE] = reg_addr;
	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++){
		array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] =
			*(reg_data + stringpos);
	}

	/* set array pointer */
	p = array;

	/* kubos API call */
	BNO055_iERROR = k_master_transmit_interrupt_i2c(0, dev_addr, p, cnt);

	return (s8)BNO055_iERROR;
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	KI2C *k_i2c = kprv_i2c_get(DEFAULT_I2C);

	s32 BNO055_iERROR = BNO055_INIT_VALUE;
	u8 stringpos = BNO055_INIT_VALUE;

	/* push reg_addr first */
	queue_push(k_i2c->rx_queue,reg_addr,I2C_TIMEOUT_FLAG,0);

	/* kubos API call */
	BNO055_iERROR = k_master_receive_interrupt_i2c(0, dev_addr);

	/* save received FRTOS queue data */
	for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++, reg_data++)
		BNO055_iERROR = xQueueReceive(k_i2c->rx_queue, reg_data, portMAX_DELAY);
	return (s8)BNO055_iERROR;
}
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BNO055_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
	/* use FRTOS */
	vTaskDelay(msek / portTICK_RATE_MS);
}
