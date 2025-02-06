/***************************************************************************//**
 *   @file   basic_example_main.c
 *   @brief  Main program for ADXL382 & AD5421 Example
 *   @author Brandon Hurst (brandon.hurst@analog.com)
********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "basic_example_main.h"
#include "common_data.h"
#include "no_os_delay.h"
#include "no_os_print_log.h"
#include <errno.h>
#include <assert.h>

#include "mxc_sys.h"
#include "ad5421.h"
#include "adxl345.h"
#include "adxl38x.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define CTRL_VLOOP_ALRT (1 << 5)
#define NO_ALARM_FORCED_CURRENT (1 << 10)


/***** AD5421 FUNCTIONS  *****/
// Write a DAC register directly
void ad5421_set_reg(struct ad5421_dev *dev, int32_t cmd, int32_t val) {
	assert(dev);
	assert(cmd & ~(0x80));

	int32_t spi_data = 0;

	spi_data = AD5421_CMD(cmd);
	/* Set certain bits. */
	spi_data += (val);
	/* Send data via SPI. */
	ad5421_set(dev,
		   &spi_data);
}

// Read a DAC register directly
int32_t ad5421_get_reg(struct ad5421_dev *dev, int32_t cmd, int32_t val) {
	assert(dev);
	assert(cmd & (0x80));

	int32_t spi_data = 0;

	/* Read from the control register. */
	spi_data = AD5421_CMD(cmd);
	ad5421_set(dev,
		   &spi_data);
	/* Receive data via SPI. */
	spi_data = ad5421_get(dev);
	return spi_data;
}

// Output loop for the DAC
int dac_test(struct ad5421_dev *ad5421_desc, int32_t dac) {
	assert(ad5421_desc);
	int32_t val;

	// DAC TEST
	ad5421_set_dac(ad5421_desc, dac);
	val = ad5421_get_fault(ad5421_desc);
	if (val) {
		pr_info("DAC Fault Status Reg: 0x%X\n", val);
		return -1;
	}
	val = ad5421_get_dac(ad5421_desc);
	pr_info("DAC VALUE: \t0x%X \n", dac);
	if (val != dac) {
		pr_info("ERR: Failed to set DAC Value \n");
		return -1;
	}

	return 0;
}


/***** ADXL FUNCTIONS  *****/
// Convert No-OS Accelerometer struct data into floating point
float adxl38x_struct_to_float(struct adxl38x_fractional_val *acc_struct)
{
	return (float)(acc_struct->integer + (float)acc_struct->fractional / ADXL38X_ACC_SCALE_FACTOR_GEE_DIV);
}

// Convert 1 dimension of accelerometer data to the 16-bit DAC code
void accel_1d_to_dac(uint16_t* dac, float accel_1d) {
	assert(dac);
	uint16_t ret=0;

	// float val comes back normalized -1.0 to 1.0
	// convert the range to 0x0000 to 0xFFFF uint16
	// (-1.0 to 1.0 will be offset without calibration)
	//
	// -1.0 --> 0x0000
	// 0.0 	--> 0x8000
	// 1.0	--> 0xFFFF

	// start with a range of 0.0 to 2.0
	accel_1d += 1.0;

	// add clipping
	accel_1d = MIN(2.0, accel_1d);
	accel_1d = MAX(0.0, accel_1d);

	*dac = (uint16_t)( (accel_1d / 2.0) * 0xFFFF );
}

// Measurement loop for accelerometer
int adxl_test(struct adxl38x_dev *adxl_desc, float* x, float* y, float* z) {
	assert(adxl_desc);

	int ret;
	uint16_t temp1, temp2;
	uint16_t x_int,y_int,z_int = 0;
	struct adxl38x_fractional_val
		xf,
		yf,
		zf = {0, 0};

	ret = adxl38x_get_xyz_gees(adxl_desc, ADXL38X_CH_EN_XYZT,
					   &xf, &yf, &zf);

	*x = adxl38x_struct_to_float(&xf);
	*y = adxl38x_struct_to_float(&yf);
	*z = adxl38x_struct_to_float(&zf);
	pr_info(" x = %f g, y = %f g, z = %f g\n", *x, *y, *z);
	pr_info("accel_z: \t%f\n", *z);

	return 0;
}

/***************************************************************************//**
 * @brief Example main execution.
 *
 * @return ret - Result of the example execution. If working correctly, will
 *               execute continuously the while(1) loop and will not return.
 *
 * @NOTE: 	DAC kits may have issues on arrival. For now, DAC errors are left
 * 			as NON-BLOCKING. So DAC errors will print, but not stop the
 * 			application.
*******************************************************************************/
int basic_example_main()
{
	struct ad5421_dev *ad5421_desc;
	// struct adxl345_dev *adxl345_desc;
	struct adxl38x_dev *adxl38x_desc;
	union adxl38x_sts_reg_flags device_flags;
	uint8_t device_id;
	uint8_t device_range;
	uint8_t opmode;

	int32_t ret = 0;
	int32_t spi_data = 0;
	uint8_t reg_value[10] = {0};
	struct no_os_uart_desc *uart_desc;

	// Delay to prevent lockouts
	no_os_mdelay(2000);

	// Print version
	pr_info("ADXL38x ACCEL + AD5421 4-20mA DAC\n");
	pr_info("v1.1.0\n");

	/* Initializing the device(s) */
	pr_info("START: ad5421_init...\n");
	ret = ad5421_init(&ad5421_desc, ad5421_ip);
	if (ret) {
		pr_info("ERR during ad5421_init \tERR=%d\n", ret);
		// goto error;
	}
	spi_data += (
			CTRL_AUTO_FAULT_RDBK |
			CTRL_ONCHIP_ADC |
			CTRL_SPI_WATCHDOG |
			NO_ALARM_FORCED_CURRENT // SPI fault doesn't force loop current alarm
			);
	ad5421_set_reg(ad5421_desc, AD5421_CMDWRCTRL, spi_data);
	ret = ad5421_get_reg(ad5421_desc, AD5421_CMDRDCTRL, spi_data);
	if (ret != spi_data) {
		pr_info("ERR during ad5421 status write", ret);
		// goto error;
	}
	else{ pr_info("Success!\n"); }


	pr_info("\nSTART: adxl_init...\n");
	ret = adxl38x_init(&adxl38x_desc, adxl38x_ip);
	if (ret)
		goto error;
	ret = adxl38x_soft_reset(adxl38x_desc);
	if (ret == -EAGAIN)
		pr_info("ADXL reset was not successful\n");
	else if (ret)
		goto error;
	ret = adxl38x_get_sts_reg(adxl38x_desc, &device_flags);
	if (ret)
		goto error;
	pr_info(" ADXL status value = 0x%08lx\n", (uint32_t)device_flags.value);

	ret = adxl38x_set_op_mode(adxl38x_desc, ADXL38X_MODE_HP);
	if (ret)
		goto error;
	ret = adxl38x_set_range(adxl38x_desc, ADXL382_RANGE_15G);
	if (ret)
		goto error;
	ret = adxl38x_get_range(adxl38x_desc, &device_range);
	if (ret)
		goto error;
	ret = adxl38x_get_deviceID(adxl38x_desc, &device_id);
	if (adxl38x_desc->dev_type == ID_ADXL382)
		pr_info(" ADXL Device Type = ADXL382\n");
	else
		pr_info(" ADXL Device Type = ADXL380\n");
	no_os_mdelay(500);

	int32_t dac = 0x0;
	while(1) {
		int32_t ret = 0;
		float xf, yf, zf = 0.0;

		pr_info("\n");

		ret = adxl_test(adxl38x_desc, &xf, &yf, &zf);
		if (ret) {goto error;}

		accel_1d_to_dac(&dac, zf);
		pr_info("AD5421 DAC Value: 0x%X\n", dac);

		ret = dac_test(ad5421_desc, dac);
		// if (ret) {goto error;}

		// no_os_mdelay(1000);
	}

error:
	pr_info("Error!\n");
	ad5421_remove(ad5421_desc);
	return 0;
}
