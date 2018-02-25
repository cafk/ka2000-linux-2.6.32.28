/*
 * KeyASIC Ka2000 I2C controller platfrom_device info
 *
 * Author: Vladimir Barinov, MontaVista Software, Inc. <source@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
*/

#ifndef __ASM_ARCH_I2C_H
#define __ASM_ARCH_I2C_H

/* All frequencies are expressed in kHz */
struct ka2000_i2c_platform_data {
	unsigned int	bus_freq;	/* standard bus frequency (kHz) */
	unsigned int	bus_delay;	/* post-transaction delay (usec) */
};

/* for board setup code */
void ka2000_init_i2c(struct ka2000_i2c_platform_data *);

#endif /* __ASM_ARCH_I2C_H */
