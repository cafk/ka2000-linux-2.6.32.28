/*
 * arch/arm/mach-ka2000/include/mach/clock.h
 *
 * Clock control driver for KeyASIC Ka2000 - header file
 *
 * Authors: Vladimir Barinov <source@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef __ASM_ARCH_KA2000_CLOCK_H
#define __ASM_ARCH_KA2000_CLOCK_H

struct clk {
	struct list_head	node;
	struct module		*owner;
	struct clk           *parent;

	const char		*name;
	unsigned int		*rate;

	//unsigned long         rate;
	unsigned long         ctrlbit;
	int			id;
	__s8			usecount;
	__u8			flags;
	__u8			lpsc;
	int		    (*set_rate)(struct clk *c, unsigned long rate);
	unsigned long	    (*get_rate)(struct clk *c);
};


/* Clock flags */
#define RATE_CKCTL		1
#define RATE_FIXED		2
#define RATE_PROPAGATES		4
#define VIRTUAL_CLOCK		8
#define ALWAYS_ENABLED		16
#define ENABLE_REG_32BIT	32

extern int clk_register(struct clk *clk);
extern void clk_unregister(struct clk *clk);
extern int ka2000_clk_init(void);

#endif
