

#ifndef NOOS_DATA_H
#define NOOS_DATA_H

// Platform includes
#include "maxim_gpio.h"
#include "maxim_uart.h"
#include "maxim_uart_stdio.h"
#include "maxim_spi.h"

// Drivers
#include "adxl38x.h"
#include "ad5421.h"

/*** Definitions ***/
#define UART_DEVICE_ID  0
#define UART_IRQ_ID     UART0_IRQn

// SPI Device for AD5421
#define DAC_SPI_DEVICE_ID       1
#define DAC_FREQ                1000000
#define DAC_SPI_CS              0
#define DAC_FAULTIN_PORT_NUM    0
#define DAC_FAULTIN_PIN_NUM     29
#define DAC_LDAC_PORT_NUM       0
#define DAC_LDAC_PIN_NUM        30

// SPI setup for ADXL (10 MHz max)
#define ADXL_SPI_DEVICE_ID      0
#define ADXL_SPI_FREQ           1000000
#define ADXL_SPI_CS             0

/*********************************/
/***** NO-OS DATA STRUCTURES *****/
/*********************************/

/*** Common Init Params ***/
static struct max_uart_init_param max_uart_extra =
{
    .flow = MXC_UART_FLOW_DIS,
};
// UART Init Parameters. Used for STDIO
static struct no_os_uart_init_param uart_ip =
{
    .device_id = 0, //UART 0 Gets to the USB Port
    .irq_id = UART0_IRQn,
    .asynchronous_rx = true,
    .baud_rate = 115200,
    .size = NO_OS_UART_CS_8,
    .parity = NO_OS_UART_PAR_NO,
    .stop = NO_OS_UART_STOP_1_BIT,
    .platform_ops = &max_uart_ops, //Use Maxim Platform Ops
    .extra = &max_uart_extra, //Maxim specific parameters
};

// SPI Initialization Parameters
static struct max_spi_init_param max_spi_param =
{
    .num_slaves = 1,
    .polarity = SPI_SS_POL_LOW,
};

/***** ADXL382 DEVICE SETUP *****/
const struct no_os_spi_init_param adxl_spi_ip = {
	.device_id = ADXL_SPI_DEVICE_ID,
	.max_speed_hz = ADXL_SPI_FREQ,
	.bit_order = NO_OS_SPI_BIT_ORDER_MSB_FIRST,
	.mode = NO_OS_SPI_MODE_0,
	.platform_ops = &max_spi_ops,
	.chip_select = ADXL_SPI_CS,
	.extra = &max_spi_param,
};
const struct adxl38x_init_param adxl38x_ip = {
	/** Device Communication initialization structure: either SPI or I2C */
	.comm_init.spi_init = adxl_spi_ip,
	/** Device Communication type: ADXL38X_SPI_COMM, ADXL38X_I2C_COMM */
	.comm_type = ADXL38X_SPI_COMM,
	/** Device type: ADXL380 or 382 */
	.dev_type = ID_ADXL382,
};

/***** AD5421 DAC Init Params *****/
struct max_gpio_init_param ad5421_gpio_extra_ldac = {
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};
struct max_gpio_init_param ad5421_gpio_extra_faultin = {
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};
const struct no_os_gpio_init_param ad5421_ldac_ip = {
	.port = DAC_LDAC_PORT_NUM,
	.number = DAC_LDAC_PIN_NUM,
	.pull = NO_OS_PULL_NONE,
	.platform_ops = &max_gpio_ops,
	.extra = &ad5421_gpio_extra_ldac,
};
const struct no_os_gpio_init_param ad5421_faultin_ip = {
	.port = DAC_FAULTIN_PORT_NUM,
	.number = DAC_FAULTIN_PIN_NUM,
	.pull = NO_OS_PULL_NONE,
	.platform_ops = &max_gpio_ops,
	.extra = &ad5421_gpio_extra_faultin,
};

const struct no_os_spi_init_param ad5421_spi_ip = {
	.device_id = DAC_SPI_DEVICE_ID,
	.max_speed_hz = DAC_FREQ,
	.bit_order = NO_OS_SPI_BIT_ORDER_MSB_FIRST,
	.mode = NO_OS_SPI_MODE_1,
	.platform_ops = &max_spi_ops,
	.chip_select = DAC_SPI_CS,
	.extra = &max_spi_param,
};

const struct ad5421_init_param ad5421_ip = {
	.spi_init = ad5421_spi_ip,
	.gpio_ldac = ad5421_ldac_ip,
	.gpio_faultin = ad5421_faultin_ip,
};

#endif // NOOS_DATA_H