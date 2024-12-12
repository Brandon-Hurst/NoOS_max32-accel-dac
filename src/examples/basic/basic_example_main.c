/***************************************************************************//**
 *   @file   basic_example_main.c
 *   @brief  Main program for basic example eval-adxl38x project
 *   @author BRajendran (balarupini.rajendran@analog.com)
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

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define CTRL_VLOOP_ALRT (1 << 5)
#define NO_ALARM_FORCED_CURRENT (1 << 10)

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

int dac_test(struct ad5421_dev *ad5421_desc, int32_t dac) {
	assert(ad5421_desc);
	int32_t val;

	// pr_info("Testing AD5421 dac...\n");

	// DAC TEST
	ad5421_set_dac(ad5421_desc, dac);
	val = ad5421_get_fault(ad5421_desc);
	if (val) {
		pr_info("DAC FAULT! 0x%X\n", val);
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

int adxl_test(struct adxl345_dev *adxl345_desc, float* x, float* y, float* z) {
	assert(adxl345_desc);
	uint16_t temp1, temp2;
	uint16_t x_int,y_int,z_int = 0;

	// pr_info("Testing adxl345 accelerometer...\n");

	// adxl345_get_xyz(adxl345_desc, &x_int, &y_int , &z_int);
	adxl345_get_g_xyz(adxl345_desc, x, y, z);

	// pr_info("x: %d y: %d z: %d\n", x_int, y_int, z_int);
	// pr_info("x: %f y: %f z: %f\n", *x, *y, *z);
	pr_info("accel_z: \t%f g\n", *z);

	return 0;
}

void accel_1d_to_dac(uint16_t* dac, float accel_1d) {
	assert(dac);
	uint16_t ret=0;

	// float val comes back normalized -1.0 to 1.0
	// convert the range to 0x0000 to 0xFFFF uint16
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

/***************************************************************************//**
 * @brief Example main execution.
 *
 * @return ret - Result of the example execution. If working correctly, will
 *               execute continuously the while(1) loop and will not return.
*******************************************************************************/
int basic_example_main()
{
	struct ad5421_dev *ad5421_desc;
	struct adxl345_dev *adxl345_desc;

	int32_t ret, spi_data = 0;
	uint8_t reg_value[10] = {0};
	uint8_t device_id;
	struct no_os_uart_desc *uart_desc;

	/* Initializing the device(s) */
	pr_info("Starting ad5421_init...");
	ret = ad5421_init(&ad5421_desc, ad5421_ip);
	if (ret) {
		pr_info("ERR during ad5421_init \tERR=%d\n", ret);
		goto error;
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
		pr_info("ERR during ad5421_init \tERR=%d\n", ret);
		goto error;
	}
	else{ pr_info("Success!\n"); }


	// pr_info("Starting adxl345_init...\n");
	ret = adxl345_init(&adxl345_desc, adxl345_ip);
	adxl345_set_power_mode(adxl345_desc, 0x1);
	if (ret) {
		pr_info("ERR during adxl345_init \tERR=%d\n", ret);
		goto error;
	}
	else{ pr_info("Success!\n"); }

	// Run basic initialization tasks here
	ad5421_reset(ad5421_desc);
	no_os_mdelay(500);

	int32_t dac = 0x0;
	while(1) {
		int32_t ret = 0;
		float x, y, z = 0.0;

		pr_info("\n");

		ret = adxl_test(adxl345_desc, &x, &y, &z);
		if (ret) {goto error;}

		accel_1d_to_dac(&dac, z);
		// no_os_mdelay(2000);

		ret = dac_test(ad5421_desc, dac);
		if (ret) {goto error;}

		// dac = (dac + 0x1111) % 0xFFFF;
		no_os_mdelay(1000);
	}

error:
	pr_info("Error!\n");
	ad5421_remove(ad5421_desc);
	return 0;
}
