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

#include <assert.h>
#include <errno.h>
#include "no_os_print_log.h"
#include "no_os_delay.h"
#include "ad5421.h"
#include "adxl38x.h"

#include "mxc_sys.h"
#include "wdt.h"
#include "gpio.h"

#include "no-os_app-data.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define CTRL_VLOOP_ALRT (1 << 5)
#define NO_ALARM_FORCED_CURRENT (1 << 10)

// TODO (low prior): LP mode not yet implemented
#define DO_SLEEP    0
#define USE_WDT     0

/***** FUNCTIONS ******/
// *****************************************************************************
#if USE_WDT
void WDT0_IRQHandler(void)
{
    MXC_WDT_ClearIntFlag(MXC_WDT0);
    printf("\n WATCHDOG TIMEOUT! \n");

    // Attempt to reset the WDT for mitigation before reset
    // MXC_WDT_ResetTimer(MXC_WDT0);
}

void WDT_Setup(void)
{
    static mxc_wdt_cfg_t cfg = {};

    // Configure WDT with defaults taken from MSDK example
    cfg.mode = MXC_WDT_COMPATIBILITY;
    cfg.upperResetPeriod = MXC_WDT_PERIOD_2_26;
    cfg.upperIntPeriod = MXC_WDT_PERIOD_2_25;
    MXC_WDT_Init(MXC_WDT0, &cfg);
    MXC_WDT_Disable(MXC_WDT0);

    MXC_WDT_SetResetPeriod(MXC_WDT0, &cfg);
    MXC_WDT_SetIntPeriod(MXC_WDT0, &cfg);
    MXC_WDT_ResetTimer(MXC_WDT0);
    MXC_WDT_EnableInt(MXC_WDT0);
    MXC_WDT_EnableReset(MXC_WDT0);

    // Setup interrupts
    NVIC_SetVector(WDT0_IRQn, (uint32_t)WDT0_IRQHandler);
    MXC_WDT_ClearResetFlag(MXC_WDT0);
    MXC_WDT_ClearIntFlag(MXC_WDT0);
    NVIC_EnableIRQ(WDT0_IRQn);
    MXC_WDT_Enable(MXC_WDT0);
}
#endif

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
        printf("DAC Fault Status Reg: 0x%X\n", val);
        return -1;
    }
    val = ad5421_get_dac(ad5421_desc);
    printf("DAC VALUE: \t0x%X \n", dac);
    if (val != dac) {
        printf("ERR: Failed to set DAC Value \n");
        return -1;
    }

    return 0;
}


/***** ADXL FUNCTIONS  *****/
// Convert No-OS Accelerometer struct data into floating point
double adxl38x_struct_to_float(struct adxl38x_fractional_val *acc_struct)
{
    return (double)(acc_struct->integer + (double)acc_struct->fractional / ADXL38X_ACC_SCALE_FACTOR_GEE_DIV);
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
int adxl_test(struct adxl38x_dev *adxl_desc, double* x, double* y, double* z) {
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
    printf(" x = %f g, y = %f g, z = %f g\n", *x, *y, *z);
    printf("accel_z: \t%f\n", *z);

    return 0;
}

/******************************************************************************/
/**************************** Main Application ********************************/
/******************************************************************************/
/*******************************************************************************
 * @brief Example main execution.
 *
 * @return ret - Result of the example execution. If working correctly, will
 *               execute continuously the while(1) loop and will not return.
 *
 * @NOTE: 	DAC kits may have issues on arrival. For now, DAC errors are left
 * 			as NON-BLOCKING. So DAC errors will print, but not stop the
 * 			application.
*******************************************************************************/
int main(void)
{
    int32_t ret = 0;
    int32_t spi_data = 0;
    uint8_t reg_value[10] = {0};
    uint8_t device_id, device_range, opmode;

    struct ad5421_dev *ad5421_desc;
    struct adxl38x_dev *adxl38x_desc;
    union adxl38x_sts_reg_flags device_flags;

    // Delay to prevent lockouts if using Low Power modes
    #if DO_SLEEP
    no_os_mdelay(2000);
    #endif

    #if USE_WDT
    WDT_Setup();
    #endif

    /* Init UART & bind to STDIO */
	struct no_os_uart_desc *uart_desc;
	ret = no_os_uart_init(&uart_desc, &uart_ip);
	if (ret)
		return ret;
	no_os_uart_stdio(uart_desc);

    /* Print version */
    printf("ADXL38x ACCEL + AD5421 4-20mA DAC\n");
    printf("v1.2.0\n");

    /* Initialize the AD5421 DAC */
    printf("START: ad5421_init...\n");
    ret = ad5421_init(&ad5421_desc, ad5421_ip);
    if (ret) {
        printf("ERR during ad5421_init \tERR=%d\n", ret);
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
        printf("ERR during ad5421 status write", ret);
        // goto error;
    }
    else {
        printf("Success!\n");
    }

    /* Print AD5421 Registers */
    printf("AD5421 Registers:\n");
    for (int i = 0; i < 5; i++) {
        ret = ad5421_get_reg(ad5421_desc, AD5421_CMDRDDAC + i, spi_data);
        printf("0x%X ==> 0x%X\n", AD5421_CMDRDDAC + i, ret);
    }
    printf("\n\n");
    printf("Delaying 1 sec...\n");
    no_os_mdelay(1000);

    /*** Initialize the ADXL38x accelerometer ***/
    printf("\nSTART: adxl_init...\n");
    ret = adxl38x_init(&adxl38x_desc, adxl38x_ip);
    if (ret)
        goto error;
    ret = adxl38x_soft_reset(adxl38x_desc);
    if (ret == -EAGAIN)
        printf("ADXL reset was not successful\n");
    else if (ret)
        goto error;
    ret = adxl38x_get_sts_reg(adxl38x_desc, &device_flags);
    if (ret)
        goto error;
    printf(" ADXL status value = 0x%08lx\n", (uint32_t)device_flags.value);

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
        printf(" ADXL Device Type = ADXL382\n");
    else
        printf(" ADXL Device Type = ADXL380\n");
    no_os_mdelay(500);

    /* Configure LED2 for running indicator */
    mxc_gpio_cfg_t led2_cfg = {
        .port = MXC_GPIO0,
        .mask = MXC_GPIO_PIN_23,
        .func = MXC_GPIO_FUNC_OUT,
        .vssel = MXC_GPIO_VSSEL_VDDIOH
    };
    MXC_GPIO_Config(&led2_cfg);

    /* MAIN LOOP : Read accelerometer and program DAC 4-20mA loop current */
    static uint16_t dac = 0x0;
    static double xf, yf, zf = 0.0;
    while(1) {

        /* Reset the Watchdog Timer (if used) */
        #if USE_WDT
        __disable_irq();
        MXC_WDT_ResetTimer(MXC_WDT0);
        __enable_irq();
        #endif

        printf("\n");

        /* Read the accelerometer */
        ret = adxl_test(adxl38x_desc, &xf, &yf, &zf);
        if (ret) {
            goto error;
        }

        /* Convert single-axis accelerometer data to DAC Code and print */
        accel_1d_to_dac(&dac, zf);
        printf("AD5421 DAC Value: 0x%X\n", dac);

        /* Program DAC with converted value */
        ret = dac_test(ad5421_desc, dac);
        // if (ret) {goto error;}

        /* Delay and toggle a "running" LED */
        no_os_mdelay(500);
        MXC_GPIO_OutToggle(MXC_GPIO0, MXC_GPIO_PIN_23);
    }

error:
    printf("Error!\n");
    ad5421_remove(ad5421_desc);
    adxl38x_remove(adxl38x_desc);
    return 0;
}
