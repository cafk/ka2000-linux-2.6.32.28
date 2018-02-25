/*
 * mach-ka2000/devices.c
 *
 * KeyASIC Ka2000 platform device setup/initialization
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>

#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/i2c.h>

//static struct resource i2c_resources[] = {
//	{
//		.start		= KA2000_I2C_BASE,
//		.end		= KA2000_I2C_BASE + 0x40,
//		.flags		= IORESOURCE_MEM,
//	},
//	{
//		.start		= IRQ_I2C,
//		.flags		= IORESOURCE_IRQ,
//	},
//};
//
//static struct platform_device ka2000_i2c_device = {
//	.name           = "i2c_ka2000",
//	.id             = 1,
//	.num_resources	= ARRAY_SIZE(i2c_resources),
//	.resource	= i2c_resources,
//};

void __init ka2000_init_i2c(struct ka2000_i2c_platform_data *pdata)
{
//this is mark by kathy
//	ka2000_i2c_device.dev.platform_data = pdata;
//	(void) platform_device_register(&ka2000_i2c_device);
}

