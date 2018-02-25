/*
 * KeyASIC Ka2000 pin multiplexing configurations
 *
 * Copyright (C) 2013 KeyASIC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/io.h>
#include <linux/spinlock.h>

#include <mach/hardware.h>

#include <mach/mux.h>

/* System control register offsets */
#define PINMUX0         0x00
#define PINMUX1         0x04

static DEFINE_SPINLOCK(mux_lock);

void ka2000_mux_peripheral(unsigned int mux, unsigned int enable)
{
	u32 pinmux, muxreg = PINMUX0;

	if (mux >= KA2000_MUX_LEVEL2) {
		muxreg = PINMUX1;
		mux -= KA2000_MUX_LEVEL2;
	}

	spin_lock(&mux_lock);
	pinmux = ka2000_readl(KA2000_SYSTEM_MODULE_BASE + muxreg);
	if (enable)
		pinmux |= (1 << mux);
	else
		pinmux &= ~(1 << mux);
	ka2000_writel(pinmux, KA2000_SYSTEM_MODULE_BASE + muxreg);
	spin_unlock(&mux_lock);
}
