/***************************************************************************//**
 *   @file    app_config.c
 *   @brief   Application configurations module
 *   @details This module contains the configurations needed for IIO application
********************************************************************************
 * Copyright (c) 2021-23 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdbool.h>

#include "app_config.h"
#include "ad77681_iio.h"
#include "ltc26x6.h"

/******************************************************************************/
/************************ Macros/Constants ************************************/
/******************************************************************************/

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/* UART init parameters */
struct no_os_uart_init_param uart_init_params = {
	.device_id = NULL,
	.baud_rate = IIO_UART_BAUD_RATE,
	.size = NO_OS_UART_CS_8,
	.parity = NO_OS_UART_PAR_NO,
	.stop = NO_OS_UART_STOP_1_BIT,
	.platform_ops = &uart_ops,
	.extra = &uart_extra_init_params
};

/* Trigger GPIO init parameters */
struct no_os_gpio_init_param trigger_gpio_param = {
	.port = TRIGGER_GPIO_PORT,
	.number = TRIGGER_GPIO_PIN,
	.pull = NO_OS_PULL_NONE,
	.platform_ops = &trigger_gpio_ops,
	.extra = &trigger_gpio_extra_init_params
};

/* Trigger GPIO IRQ parameters */
struct no_os_irq_init_param trigger_gpio_irq_params = {
	.irq_ctrl_id = 0,
	.platform_ops = &trigger_gpio_irq_ops,
	.extra = &trigger_gpio_irq_extra_params
};

// Initial parameters for the DAC LTC2606 itself
struct ltc26x6_init_param init_params_dac = { 
	{   .device_id = 0,
		.platform_ops = &i2c_ops,
		.slave_address = LTC2606_I2C_ADDRESS,
		.max_speed_hz = 100000,
		.extra = &i2c_extra_init_params,
	},   // I2C parameters
	16,    // Resolution (LTC2606)
	2.5,   // Reference Voltage
	- 0.001			 // Typical offset
};

/* UART descriptor */
struct no_os_uart_desc *uart_desc;

/* Trigger GPIO descriptor */
struct no_os_gpio_desc *trigger_gpio_desc;

/* Trigger GPIO interrupt descriptor */
struct no_os_irq_ctrl_desc *trigger_irq_desc;

/******************************************************************************/
/************************ Functions Prototypes ********************************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/
/* External interrupt callback descriptor */
static struct no_os_callback_desc ext_int_callback_desc = {
	.callback = burst_capture_callback,
	.ctx = NULL,
	.event = NO_OS_EVT_GPIO,
	.peripheral = NO_OS_GPIO_IRQ
};

/**
 * @brief Initialize the trigger GPIO and associated IRQ event
 * @return 0 in case of success, negative error code otherwise
 */
static int32_t gpio_trigger_init(void)
{
	int32_t ret;

	/* Configure GPIO as input */
	ret = no_os_gpio_get(&trigger_gpio_desc, &trigger_gpio_param);
	if (ret) {
		return ret;
	}

	ret = no_os_gpio_direction_input(trigger_gpio_desc);
	if (ret) {
		return ret;
	}

#if (DATA_CAPTURE_MODE == BURST_DATA_CAPTURE)
    	/* Initialize the IRQ controller */
	ret = no_os_irq_ctrl_init(&trigger_irq_desc, &trigger_gpio_irq_params);
	if (ret) {
		return ret;
	}

    ret = no_os_irq_register_callback(trigger_irq_desc, 0,
					  &ext_int_callback_desc);
	if (ret) {
		return ret;
	}

	ret = no_os_irq_trigger_level_set(trigger_irq_desc, 0,
					  NO_OS_IRQ_EDGE_FALLING);
	if (ret) {
		return ret;
	}

	ret = no_os_irq_disable(trigger_irq_desc, 0);
	if (ret) {
		return ret;
	}

#else
	/* Init interrupt controller for external interrupt */
	ret = no_os_irq_ctrl_init(&trigger_irq_desc, &trigger_gpio_irq_params);
	if (ret) {
		return ret;
	}
#endif

	return 0;
}

/**
 * @brief 	Initialize the UART peripheral
 * @return	0 in case of success, negative error code otherwise
 */
static int32_t init_uart(void)
{
	return no_os_uart_init(&uart_desc, &uart_init_params);
}

/**
 * @brief 	Initialize the system peripherals
 * @return	0 in case of success, negative error code otherwise
 */
int32_t init_system(void)
{
	int32_t ret;

	ret = init_uart();
	if (ret) {
		return ret;
	}

	ret = gpio_trigger_init();
	if (ret) {
		return ret;
	}

#if defined(USE_SDRAM)
	ret = sdram_init();
	if (ret) {
		return ret;
	}
#endif

	return 0;
}
