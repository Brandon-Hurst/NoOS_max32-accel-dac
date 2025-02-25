/***************************************************************************//**
 *   @file   common_data.c
 *   @brief  Defines common data to be used by ADXL382/AD5421 examples.
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
#include "common_data.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
struct no_os_uart_init_param uart_ip = {
	.device_id = UART_DEVICE_ID,
	.irq_id = UART_IRQ_ID,
	.asynchronous_rx = true,
	.baud_rate = UART_BAUDRATE,
	.size = NO_OS_UART_CS_8,
	.parity = NO_OS_UART_PAR_NO,
	.stop = NO_OS_UART_STOP_1_BIT,
	.extra = UART_EXTRA,
	.platform_ops = UART_OPS,
};


/***** AD5421 DEVICE SETUP *****/
struct no_os_spi_init_param ad5421_spi_ip = {
	.device_id = AD5421_SPI_DEVICE_ID,
	.max_speed_hz = SPI_BAUDRATE,
	.bit_order = NO_OS_SPI_BIT_ORDER_MSB_FIRST,
	.mode = NO_OS_SPI_MODE_1,
	.platform_ops = SPI_OPS,
	.chip_select = AD5421_SPI_CS,
	.extra = SPI_EXTRA,
};

struct no_os_gpio_init_param ad5421_ldac_ip = {
	.port = GPIO_LDAC_PORT_NUM,
	.number = GPIO_LDAC_PIN_NUM,
	.pull = NO_OS_PULL_NONE,
	.platform_ops = GPIO_OPS,
	.extra = GPIO_EXTRA_LDAC,
};

struct no_os_gpio_init_param ad5421_faultin_ip = {
	.port = GPIO_FAULTIN_PORT_NUM,
	.number = GPIO_FAULTIN_PIN_NUM,
	.pull = NO_OS_PULL_NONE,
	.platform_ops = GPIO_OPS,
	.extra = GPIO_EXTRA_FAULTIN,
};

struct ad5421_init_param ad5421_ip = {
	// .spi_init = &ad5421_spi_ip,
	.gpio_ldac = &ad5421_ldac_ip,
	.gpio_faultin = &ad5421_faultin_ip,
};

/***** ADXL355 DEVICE SETUP *****/
struct no_os_spi_init_param adxl_spi_ip = {
	.device_id = ADXL_SPI_DEVICE_ID,
	.max_speed_hz = SPI_BAUDRATE,
	.bit_order = NO_OS_SPI_BIT_ORDER_MSB_FIRST,
	.mode = NO_OS_SPI_MODE_0,
	.platform_ops = SPI_OPS,
	.chip_select = ADXL_SPI_CS,
	.extra = SPI_EXTRA,
};

struct no_os_i2c_init_param adxl_i2c_ip = {
	.device_id = I2C_PORT,
	.max_speed_hz = 400000,
	.platform_ops = I2C_OPS,
	.extra = I2C_EXTRA,
	// no slave addr b/c acting as master
	.slave_address = ADXL345_ADDRESS,
};

struct adxl345_init_param adxl345_ip = {
	.i2c_init = &adxl_i2c_ip,
	.spi_init = &adxl_spi_ip,
	.dev_type = ID_ADXL345,
	.communication_type = ADXL345_SPI_COMM,
	.selected_range =  ADXL345_RANGE_PM_16G, // 2, 4, 8 , or 16G
	.full_resolution_set = 1
};

/***** ADXL382 DEVICE SETUP *****/
struct adxl38x_init_param adxl38x_ip = {
	/** Device Communication initialization structure: either SPI or I2C */
	// .comm_init.spi_init = adxl_spi_ip,
	/** Device Communication type: ADXL38X_SPI_COMM, ADXL38X_I2C_COMM */
	.comm_type = ADXL38X_SPI_COMM,
	/** Device type: ADXL380 or 382 */
	.dev_type = ID_ADXL382,
};
