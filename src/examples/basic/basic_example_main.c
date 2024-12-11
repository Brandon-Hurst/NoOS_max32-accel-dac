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

#include "ad5421.h"
#include "adxl355.h"


int dac_test(struct ad5421_dev *ad5421_desc, int32_t dac) {
	int32_t val;

	assert(ad5421_desc);

	pr_info("Testing AD5421 dac...\n");

	// DAC TEST
	ad5421_set_dac(ad5421_desc, dac);
	val = ad5421_get_dac(ad5421_desc);
	pr_info("DAC VALUE: \t0x%X \n", dac);
	if (val != dac) {
		pr_info("ERR: Failed to set DAC Value \n");
		return -1;
	}

	val = ad5421_get_temp(ad5421_desc);
	pr_info("TEMP VAL: \t0x%X \n", val);

	return 0;
}

int adxl_test(struct adxl355_dev *adxl355_desc) {
	assert(adxl355_desc);

	pr_info("Testing ADXL355 accelerometer...\n");

	return 0;
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
	struct adxl355_dev *adxl355_desc;

	int ret;
	uint8_t reg_value[10] = {0};
	uint8_t device_id;
	struct no_os_uart_desc *uart_desc;

	/* Initializing the device(s) */
	pr_info("Starting ad5421_init...\n");
	ret = ad5421_init(&ad5421_desc, ad5421_ip);
	if (ret) {
		pr_info("ERR during ad5421_init \tERR=%d", ret);
	}

	// pr_info("Starting adxl355_init...\n");
	// ret = adxl355_init(&adxl355_desc, adxl355_ip);
	if (ret) {
		pr_info("ERR during adxl355_init \tERR=%d", ret);
	}

	if (ret){
		goto error;
	}
	else {
		pr_info("...ad5421_init succeeded!\n");
	}

	// Run basic initialization tasks here
	ad5421_reset(ad5421_desc);
	// adxl355_soft_reset(adxl355_desc);
	no_os_mdelay(500);

	int32_t dac = 0x0;
	while(1) {
		int32_t ret;
		ret = dac_test(ad5421_desc, dac);
		if (ret) {goto error;}

		dac = (dac + 0x1111) % 0x10000;
		no_os_mdelay(500);
	}

error:
	pr_info("Error!\n");
	ad5421_remove(ad5421_desc);
	return 0;
}
