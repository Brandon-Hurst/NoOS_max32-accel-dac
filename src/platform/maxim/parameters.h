/***************************************************************************//**
 *   @file   parameters.h
 *   @brief  Definitions specific to Maxim platform used by eval-adxl38x
 *           project.
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
#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "maxim_irq.h"
#include "maxim_spi.h"
#include "maxim_gpio.h"
#include "maxim_uart.h"
#include "maxim_uart_stdio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

// Pins setup for different targets
#if (TARGET_NUM == 78000)
    #define UART_DEVICE_ID  0
    #define UART_IRQ_ID     UART0_IRQn
    #define SPI_DEVICE_ID   1
    #define SPI_CS          1
    #define GPIO_FAULTIN_PORT_NUM   1
    #define GPIO_FAULTIN_PIN_NUM    6
    #define GPIO_LDAC_PORT_NUM      0
    #define GPIO_LDAC_PIN_NUM       19
#elif (TARGET_NUM == 32675)
    #define UART_DEVICE_ID  0
    #define UART_IRQ_ID     UART0_IRQn
    #define SPI_DEVICE_ID   1 // SPI1A: 0.14 MISO 0.15 MOSI 0.16 SCLK 0.17 SS0
    #define SPI_CS          0
    #define GPIO_FAULTIN_PORT_NUM   1
    #define GPIO_FAULTIN_PIN_NUM    7
    #define GPIO_LDAC_PORT_NUM      1
    #define GPIO_LDAC_PIN_NUM       8
#elif (TARGET_NUM == 32670)
    #define UART_DEVICE_ID  0
    #define UART_IRQ_ID     UART0_IRQn
    #define SPI_DEVICE_ID   0
    #define AD5421_SPI_CS   0
    #define ADXL355_SPI_CS  1
    #define GPIO_FAULTIN_PORT_NUM   0
    #define GPIO_FAULTIN_PIN_NUM    29
    #define GPIO_LDAC_PORT_NUM      0
    #define GPIO_LDAC_PIN_NUM       30
#endif

// Platform setup for UART
#define UART_BAUDRATE   115200
#define UART_OPS        &max_uart_ops
#define UART_EXTRA      &uart_extra_ip

// Platform setup for SPI
#define SPI_BAUDRATE    1000000
#define SPI_EXTRA       &spi_extra_ip
#define SPI_OPS         &max_spi_ops

// Platform setup for GPIOs
#define GPIO_OPS        &max_gpio_ops
#define GPIO_EXTRA_LDAC &ad5421_gpio_extra_ldac_ip
#define GPIO_EXTRA_FAULTIN  &ad5421_gpio_extra_faultin_ip

extern struct max_uart_init_param uart_extra_ip;
extern struct max_spi_init_param spi_extra_ip;
extern struct max_gpio_init_param ad5421_gpio_extra_ldac_ip;
extern struct max_gpio_init_param ad5421_gpio_extra_faultin_ip;

#endif /* __PARAMETERS_H__ */
