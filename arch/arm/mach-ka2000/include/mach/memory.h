/*
 * KeyASIC Ka2000 memory space definitions
 *
 * Author: Kevin Hilman, MontaVista Software, Inc. <source@mvista.com>
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/**************************************************************************
 * Included Files
 **************************************************************************/
#include <asm/page.h>
#include <asm/sizes.h>

/**************************************************************************
 * Definitions
 **************************************************************************/

#define KA2000_DDR_BASE    UL(0x00000000)

#define PHYS_OFFSET KA2000_DDR_BASE

/*
 * Increase size of DMA-consistent memory region
 */

/*
#define CONSISTENT_DMA_SIZE ( 2<<20 )    //kathy ...default dma size for kernel.
*/
#define CONSISTENT_DMA_SIZE SZ_4M  // Athena

/*
 * Bus address is physical address
 */
#define __virt_to_bus(x)	__virt_to_phys(x)
#define __bus_to_virt(x)	__phys_to_virt(x)

#endif /* __ASM_ARCH_MEMORY_H */
