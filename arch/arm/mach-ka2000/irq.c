/*
 * Interrupt handler for KeyASIC Ka2000 boards.
 *
 * Copyright (C) 2013 KeyASIC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/mach/irq.h>

#if 1//CONFIG_ARCH_KA2000
void ka2000_putstr(const char *ptr);
void ka2000_dbg(const char *fmt, ...);
#define T(); ka2000_dbg("%s %d\n", __FUNCTION__,__LINE__);

#endif

#define IRQ_BIT(irq)		((irq) & 0x1f)

//this is for trek2k project, add by kathy*/
#define INTC_SRC1               0x00
#define INTC_SRC2               0x04
#define INTC_MASK1              0x10
#define INTC_MASK2              0x14
#define INTC_CLR                0x30
#define INTC_PND1               0x18
#define INTC_PND2               0x1c
/**********************************/

#define IRQ_SRCP_REG1_OFFSET	0x0000
#define IRQ_SRCP_REG2_OFFSET	0x0004
#define IRQ_MODE_REG1_OFFSET	0x0008
#define IRQ_MODE_REG2_OFFSET	0x000C
#define IRQ_MASK_REG1_OFFSET	0x0010
#define IRQ_MASK_REG2_OFFSET	0x0014
#define IRQ_PEND_REG1_OFFSET	0x0018
#define IRQ_PEND_REG2_OFFSET	0x001C
#define IRQ_PRIO_REG1_OFFSET	0x0020
#define IRQ_PRIO_REG2_OFFSET	0x0024
#define IRQ_PROG_REG1_OFFSET	0x0028
#define IRQ_PROG_REG2_OFFSET	0x002C
#define IRQ_CLR_REG_OFFSET	0x0030
#define IRQ_OFFSET_REG_OFFSET	0x0034



#define ka2000_irq_readl(offset) ka2000_readl(KA2000_ARM_INTC_BASE + offset)
#define ka2000_irq_writel(value, offset)  ka2000_writel(value, KA2000_ARM_INTC_BASE + offset)

static int check_irq(int irq)
{
    if (irq >= KA2000_N_AINTC_IRQ)
    {
        //printk("irq %d >= KA2000_N_AINTC_IRQ %d\n", irq, KA2000_N_AINTC_IRQ);
        return -1;
    }
    return 0;
}


/* enable interrupt */
static void ka2000_unmask_irq(unsigned int irq)
{
    unsigned int mask;
    u32 l;
    if (check_irq(irq))
        return;

    mask = 1 << IRQ_BIT(irq); // IRQ_BIT is & 0x1f, ie < 32

    if (irq > 31)
    {
        l = ka2000_irq_readl(IRQ_MASK_REG2_OFFSET);
        l &= ~mask;
		barrier();
        ka2000_irq_writel(l, IRQ_MASK_REG2_OFFSET);
    }
    else
    {
        l = ka2000_irq_readl(IRQ_MASK_REG1_OFFSET);
        l &= ~mask;
		barrier();
        ka2000_irq_writel(l, IRQ_MASK_REG1_OFFSET);
    }
}
/* disable interrupt */
static void ka2000_mask_irq(unsigned int irq)
{
    unsigned int mask;
    u32 l;

   if (check_irq(irq))
        return;

    mask = 1 << IRQ_BIT(irq);  // IRQ_BIT is & 0x1f, ie < 32


    //mask = 1 << IRQ_BIT(irq);
    //ka2000_dbg("mask irq %d mask %x\n",irq,mask);

    if (irq > 31)
    {
        l = ka2000_irq_readl(IRQ_MASK_REG2_OFFSET);
        l |= mask;
        barrier();
        ka2000_irq_writel(l, IRQ_MASK_REG2_OFFSET);
    }
    else
    {
        l = ka2000_irq_readl(IRQ_MASK_REG1_OFFSET);
        l |= mask;
        barrier();
        ka2000_irq_writel(l, IRQ_MASK_REG1_OFFSET);
    }
}
/* EOI interrupt */
static void ka2000_ack_irq(unsigned int irq)
{
    unsigned int mask;
    if (check_irq(irq))
        return;

    mask = 1 << IRQ_BIT(irq);

    //if (irq != 22)
    //printk("ack irq %d,",irq);
    //printk("ack irq %d mask %x\n",irq, mask);
    //if (irq == 26)
    //  printk("A");
    //if(irq==IRQ_PWM0){	//IRQ_PWM0
    //   //PWM Timer Configuration Register 1
    //   ka2000_writel(0x00100000, 0xa0002000 + 0x04); //Timer0 in DMA mode, Timer1 & 2 in Interrupt mode
    //   ka2000_writel(0x00000000, 0xa0002000 + 0x04); //Timer0, 1, 2 in Interrupt mode
    //}
    if (irq > 31)
        ka2000_irq_writel(irq, IRQ_PEND_REG2_OFFSET);
    else
        ka2000_irq_writel(irq, IRQ_PEND_REG1_OFFSET);
    barrier();
    //Interrupt Request Clear Register - clear global interrupt signal to start next
    ka2000_irq_writel(0, IRQ_CLR_REG_OFFSET);
}
static struct irq_chip ka2000_irq_chip_0 =
{
    .name	= "AINTC",
    .ack	= ka2000_ack_irq,
    .mask	= ka2000_mask_irq,
    .unmask = ka2000_unmask_irq,
};


/* FIQ are pri 0-1; otherwise 2-7, with 7 lowest priority */
static const u8 default_priorities[KA2000_N_AINTC_IRQ] __initdata =
{
    [IRQ_WDT0]			        = 7,
    [IRQ_UARTINT0]			    = 7,
    [IRQ_SSI]			        = 7,
    [IRQ_PWM0]			        = 7,
    [IRQ_DMA]			        = 7,
    [IRQ_EXT_INT]			    = 7,
    [IRQ_IIC]			        = 7,
    [7]				            = 7,
    [IRQ_GPI0_0]			    = 7,
    [IRQ_GPI0_1]			    = 7,
    [IRQ_GPI0_2]			    = 7,
    [IRQ_GPI0_3]			    = 7,
    [IRQ_GPI0_4]			    = 7,
    [IRQ_GPI0_5]			    = 7,
    [IRQ_GPI0_6]			    = 7,
    [IRQ_GPI0_7]			    = 3,
    [IRQ_GPI1_0]			    = 7,
    [IRQ_GPI1_1]			    = 7,
    [IRQ_GPI1_2]			    = 7,
    [IRQ_GPI1_3]			    = 7,
    [IRQ_GPI1_4]			    = 7,
    [IRQ_GPI1_5]			    = 7,
    [IRQ_PWM_T1]			    = 2,    /* clockevent */
    [IRQ_PWM_T2]			    = 2,    /* clocksource */
    [IRQ_sdm_buf_tran_finish]	= 7,
    [IRQ_sdm_data_bound_int]	= 7,
    [IRQ_sdm_tran_done_int]		= 7,
    [IRQ_sdm_cmd_done_int]		= 7,
    [IRQ_sdm_card_error_int]	= 7,
    [IRQ_sdm_dma_int]		    = 7,
    [30]				        = 7,
    [31]				        = 7,
    [IRQ_sdio_buf_tran_finish_int]	= 7,
    [IRQ_sdio_data_bound_int]	= 4,
    [IRQ_sdio_tran_done_int]	= 7,
    [IRQ_sdio_cmd_done_int]		= 7,
    [IRQ_sdio_card_error_int]	= 4,
    [IRQ_sdio_dma_int]		    = 7,
    [IRQ_card_int]			    = 5,
    [39]				        = 7,
    [IRQ_switch_int0]		    = 5, //5    /*7,7d,16,17,18,24,25,42,a6*/
    [IRQ_switch_int1]		    = 5, //7    /*0,3,4,15,27,32,33,38,56,a13,a22,a23,a41,a42,a51*/
    [IRQ_switch_int2]		    = 4,        /*bomb*/
    [IRQ_switch_int3]		    = 3,        /*6,7,7d,38,42,56*/
    [IRQ_switch_int4]		    = 7,        /*17,18*/
    [IRQ_switch_int5]		    = 7,        /*24,25*/
    [IRQ_switch_int6]		    = 4,        /*bomb slp*/
    [IRQ_switch_int7]		    = 3,        /*50,51*/
    [IRQ_switch_int8]		    = 3,        /*50,51 slp*/
    [49]				        = 7,        /*dma?*/
    [IRQ_switch_int_b2_normal]  = 7,        /*bomb2*/
    [IRQ_switch_int_b2_sleep]	= 7,        /*bomb2 slp*/
    [IRQ_switch_int12]			= 7,        /*2,8,9,10,12,13,27*/
    [IRQ_switch_int13]			= 7,        /*a18,a25,a26,a38,a43,a44,a45,a46,a47,a48*/
    [54]				        = 7,
    [55]				        = 7,
    [56]				        = 7,
    [57]				        = 7,
    [58]				        = 7,
    [59]				        = 7,
    [60]				        = 7,
    [61]				        = 7,
    [62]				        = 7,
    [63]				        = 7,
};



/* ARM Interrupt Controller Initialization */
void __init ka2000_irq_init(void)
{
    unsigned i;
    const u8 *priority = default_priorities;
    u32 m1, m2;
    u64 type = 0;

    /* Clear all interrupt requests */
    ka2000_irq_writel(~0x0, IRQ_PEND_REG1_OFFSET);
    ka2000_irq_writel(~0x0, IRQ_PEND_REG2_OFFSET);
    barrier();
    /* Disable all interrupts */
    ka2000_irq_writel(~0x0, IRQ_MASK_REG1_OFFSET);
    ka2000_irq_writel(~0x0, IRQ_MASK_REG2_OFFSET);
    barrier();
//      /* Interrupts disabled immediately, IRQ entry reflects all */
//	ka2000_irq_writel(0x0, IRQ_INCTL_REG_OFFSET);

    /* we don't use the hardware vector table, just its entry addresses */
//	ka2000_irq_writel(0, IRQ_EABASE_REG_OFFSET);
    /* Clear all interrupt requests - again*/
    ka2000_irq_writel(~0x0, IRQ_PEND_REG1_OFFSET);
    ka2000_irq_writel(~0x0, IRQ_PEND_REG2_OFFSET);
    barrier();
    /* Setup Priority, Mode, Type ?? (eee:TODO)*/
	for (i = IRQ_PRIO_REG1_OFFSET; i <= IRQ_PRIO_REG2_OFFSET; i += 4)
	{
		unsigned	j;
		u32		pri;

		for (j = 0, pri = 0; j < 32; j += 4, priority++)
			pri |= (*priority & 0x01) << j;
        barrier();
		ka2000_irq_writel(pri, i);
	}

    /* set up genirq dispatch for ARM INTC */
    m1 = ka2000_irq_readl(IRQ_PROG_REG1_OFFSET);	//level(0), edge(1)
    m2 = ka2000_irq_readl(IRQ_PROG_REG2_OFFSET);
	barrier();
    type = m2;
    type <<= 32;
    type |= m1;

    for (i = 0; i < KA2000_N_AINTC_IRQ; i++)
    {
        set_irq_chip(i, &ka2000_irq_chip_0);
        set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
        if(0 == (type & (1 << i)))
        {
            //ka2000_dbg("set_irq_handler %d handle_level_irq\n", i);
            set_irq_handler(i, handle_level_irq);
        }
        else
        {
            //ka2000_dbg("set_irq_handler %d handle_edge_irq\n", i);
            set_irq_handler(i, handle_edge_irq);
        }

    }
    barrier();
    /* Clear global interrupt signal */
    ka2000_irq_writel(0, IRQ_CLR_REG_OFFSET);
}
