/*
 * KeyASIC Ka2000 I/O mapping code
 *
 * Copyright (C) 2010 KeyASIC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>

#include <asm/tlb.h>
#include <asm/memory.h>

#include <asm/mach/map.h>
#include <mach/clock.h>

extern void ka2000_check_revision(void);

/*
 * The machine specific code may provide the extra mapping besides the
 * default mapping provided here.
 */
 /*
#define IO_PHYS		0xa0000000
#define IO_OFFSET	0x55000000
#define IO_SIZE		0x00100000

#define IO_VIRT		(IO_PHYS + IO_OFFSET)
#define io_v2p(va)	((va) - IO_OFFSET)
#define __IO_ADDRESS(x)	((x) + IO_OFFSET)
*/
static struct map_desc ka2000_io_desc[] __initdata =
{
    {
        .virtual	= IO_VIRT,
        .pfn		= __phys_to_pfn(IO_PHYS),
        .length		= IO_SIZE,
        .type		= MT_DEVICE,
    },
};


void __init ka2000_map_common_io(void)
{

    iotable_init(ka2000_io_desc, ARRAY_SIZE(ka2000_io_desc));

    /* Normally devicemaps_init() would flush caches and tlb after
     * mdesc->map_io(), but we must also do it here because of the CPU
     * revision check below.
     */
    //local_flush_tlb_all();  this is mark by kathy
    //flush_cache_all();

    /* We want to check CPU revision early for cpu_is_xxxx() macros.
     * IO space mapping must be initialized before we can do that.
     */
    //ka2000_check_revision();   this is mark by kathy
    //printk("ka2000_map_common_io done\n");
}

void __init ka2000_init_common_hw(void)
{
    ka2000_clk_init();
}
