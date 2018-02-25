/*
 * KeyASIC Ka2000 pin multiplexing defines
 *
 * Author: Vladimir Barinov, MontaVista Software, Inc. <source@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef __ASM_ARCH_MUX_H
#define __ASM_ARCH_MUX_H

#define KA2000_MUX_AEAW0	0
#define KA2000_MUX_AEAW1	1
#define KA2000_MUX_AEAW2	2
#define KA2000_MUX_AEAW3	3
#define KA2000_MUX_AEAW4	4
#define KA2000_MUX_AECS4	10
#define KA2000_MUX_AECS5	11
#define KA2000_MUX_VLYNQWD0	12
#define KA2000_MUX_VLYNQWD1	13
#define KA2000_MUX_VLSCREN	14
#define KA2000_MUX_VLYNQEN	15
#define KA2000_MUX_HDIREN	16
#define KA2000_MUX_ATAEN	17
#define KA2000_MUX_RGB666	22
#define KA2000_MUX_RGB888	23
#define KA2000_MUX_LOEEN	24
#define KA2000_MUX_LFLDEN	25
#define KA2000_MUX_CWEN	26
#define KA2000_MUX_CFLDEN	27
#define KA2000_MUX_HPIEN	29
#define KA2000_MUX_1394EN	30
#define KA2000_MUX_EMACEN	31

#define KA2000_MUX_LEVEL2	32
#define KA2000_MUX_UART0	(KA2000_MUX_LEVEL2 + 0)
#define KA2000_MUX_UART1	(KA2000_MUX_LEVEL2 + 1)
#define KA2000_MUX_UART2	(KA2000_MUX_LEVEL2 + 2)
#define KA2000_MUX_U2FLO	(KA2000_MUX_LEVEL2 + 3)
#define KA2000_MUX_PWM0	(KA2000_MUX_LEVEL2 + 4)
#define KA2000_MUX_PWM1	(KA2000_MUX_LEVEL2 + 5)
#define KA2000_MUX_PWM2	(KA2000_MUX_LEVEL2 + 6)
#define KA2000_MUX_I2C		(KA2000_MUX_LEVEL2 + 7)
#define KA2000_MUX_SPI		(KA2000_MUX_LEVEL2 + 8)
#define KA2000_MUX_MSTK	(KA2000_MUX_LEVEL2 + 9)
#define KA2000_MUX_ASP		(KA2000_MUX_LEVEL2 + 10)
#define KA2000_MUX_CLK0	(KA2000_MUX_LEVEL2 + 16)
#define KA2000_MUX_CLK1	(KA2000_MUX_LEVEL2 + 17)
#define KA2000_MUX_TIMIN	(KA2000_MUX_LEVEL2 + 18)

extern void ka2000_mux_peripheral(unsigned int mux, unsigned int enable);

#endif /* __ASM_ARCH_MUX_H */
