/*
 * KeyASIC Ka2000 system defines
 *
 * Author: Kevin Hilman, MontaVista Software, Inc. <source@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H
#include <linux/version.h>
#include <linux/io.h>
#include <mach/hardware.h>

extern void ka2000_system_reset(void);
extern int cpu_arm926_do_idle(void);
static void arch_idle(void)
{
        //asm("ldr     r2, =0xf5005008");
        //asm("ldr     r0, =0x77");
        //asm("str     r0, [r2]");
		
        cpu_arm926_do_idle();
}
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
static void arch_reset(char mode, const char *cmd)
#else
static void arch_reset(char mode)
#endif
{
	ka2000_system_reset();
}

#endif /* __ASM_ARCH_SYSTEM_H */
