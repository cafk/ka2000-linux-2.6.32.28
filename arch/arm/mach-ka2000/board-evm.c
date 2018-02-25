/*
 * KeyASIC KA2000 EVM board support
 *
 * Copyright (C) 2013 KeyASIC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/leds.h>

#include <linux/i2c.h>
#include <linux/i2c/pcf857x.h>
#include <linux/i2c/at24.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/io.h>

#include <asm/setup.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <linux/spi/flash.h>

#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/i2c.h>
#include <mach/ka2000_define.h>
#include <mach/ka2000.h>
#include <linux/spi/spi.h>
#include <mach/irqs.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <mach/serial.h>
#include <mach/irqs.h>
#include <linux/serial_8250.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/serial_core.h>

#ifndef CONFIG_KA2000_PRINTK_ENABLE
#define printk dprintk
static inline int dprintk(const char *fmt, ...)
{
      return 0;
}
#endif

/* other misc. init functions */
void __init ka2000_psc_init(void);
void __init ka2000_irq_init(void);
void __init ka2000_map_common_io(void);
void __init ka2000_init_common_hw(void);

#define word_write(a, v) 	__raw_writel(v, IO_ADDRESS(a))

extern void arm926_dma_flush_range(const void *start, const void *end);
extern void arm926_dma_inv_range(const void *start, const void *end);

extern void printascii(const char *);
#if CONFIG_KA2000_CHIP_VERSION == 0xA

void ka2000_dma_flush_all(void)
{
    //arm926_dma_flush_range(start, end);
    local_flush_tlb_all();
    flush_cache_all();
   null_delay(30);
}

void ka2000_dma_flush_range(const void *start, const void *end)
{
    //arm926_dma_flush_range(start, end);
    local_flush_tlb_all();
    flush_cache_all();
   null_delay(20);
}
void ka2000_dma_inv_range(const void *start, const void *end)
{
    //printk("start %x, end %x\n", (int)start, (int)end);
    arm926_dma_inv_range(start, end);
    //local_flush_tlb_all();
    //flush_cache_all();
    null_delay(20);
}
#else

void ka2000_dma_flush_all(void)
{
    flush_cache_all();
}

void ka2000_dma_flush_range(const void *start, const void *end)
{
    arm926_dma_flush_range(start, end);
}
void ka2000_dma_inv_range(const void *start, const void *end)
{
    //printk("start %x, end %x\n", (int)start, (int)end);
    arm926_dma_inv_range(start, end);
}

#endif
EXPORT_SYMBOL(ka2000_dma_flush_all);
EXPORT_SYMBOL(ka2000_dma_flush_range);
EXPORT_SYMBOL(ka2000_dma_inv_range);

#define UARTC(ch);	*(volatile u32 __force *) (0x55000000 + 0xa0004000) = (ch);

void ka2000_putstr(const char *s)
{
#if 0
	int i;

	for (i = 0; i < 256; i++)
	{
		if (s[i] == 0)
			break;

		UARTC(s[i]);
	}
#endif
}


void ka2000_dbg(const char *fmt, ...)
{
#if 0
	va_list va;
	char buff[256];
	ka2000_putstr(fmt);

	va_start(va, fmt);
	vsprintf(buff, fmt, va);
	va_end(va);

	ka2000_putstr(buff);
#endif
}

EXPORT_SYMBOL(ka2000_putstr);
EXPORT_SYMBOL(ka2000_dbg);

#define KA_PA_SSI_SPI	(0xa0001000)
#define KA_SZ_SSI_SPI	SZ_4K

static struct mtd_partition ka2000_spi_flash_partitions[] = {
    {
        .name   = "JffS2",
        .offset = 0x080000,
        .size   = 0x180000,
    },
    {
        .name   = "Kernel",
        .offset = 0x200000,
        .size   = 0x300000,
    },
	{
		.name	= "Ramdisk",
		.offset = 0x500000,
		.size	= 0x300000,
	},
	{
		.name	= "JFFS2_Kernel_RFS",
		.offset = 0x080000,
		.size	= 0x780000,
	},	
};

#if 1 //def CONFIG_MMC_KA2000
#ifndef KA_SD_CTRL_BASE
#define KA_SD_CTRL_BASE 0xa000b000
#endif
static struct resource ka_sdmmc_resources[] = {
	[0] = {
		.start = KA_SD_CTRL_BASE,
		.end = KA_SD_CTRL_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_sdm_buf_tran_finish,
		//.end = IRQ_sdm_buf_tran_finish,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_sdm_data_bound_int,
		//.end = IRQ_sdm_data_bound_int,
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.start = IRQ_sdm_tran_done_int,
		//.end = IRQ_sdm_tran_done_int,
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = IRQ_sdm_cmd_done_int,
		//.end = IRQ_sdm_cmd_done_int,
		.flags = IORESOURCE_IRQ,
	},
	[5] = {
		.start = IRQ_sdm_card_error_int,
		//.end = IRQ_sdm_card_error_int,
		.flags = IORESOURCE_IRQ,
	},
	[6] = {
		.start = IRQ_sdm_dma_int,
		//.end = IRQ_sdm_dma_int,
		.flags = IORESOURCE_IRQ,
	}
};
struct platform_device ka_sdmmc_device = {
	.name 	= "ka_sdmmc",
	.id 	= -1,
	.dev 	= {
		.platform_data = NULL,
	},
	.resource = ka_sdmmc_resources,
	.num_resources = ARRAY_SIZE(ka_sdmmc_resources),
};
#endif


/*----------------------------------------------------------------------*/
/*
	SDIO platform define data for sdio driver usage, some field maybe no need
*/
#include <linux/mmc/host.h>
#include "ka2000-sdio.h"

struct platform_device;
struct mmc_host;
struct mmc_card;
struct mmc_ios;

char *ka_sdio_clksrcs[1] = {
	[0] = "SDIOCLK",
};

static struct resource ka_sdio_resource[] = {
	[0] = {
		.start = KA_SDIO_BASE,
		.end   = KA_SDIO_BASE + 0x100 - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_sdio_buf_tran_finish_int, // 32,
		.end   = IRQ_sdio_buf_tran_finish_int, // 32,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_sdio_data_bound_int, // 33,
		.end   = IRQ_sdio_data_bound_int, // 33,
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.start = IRQ_sdio_tran_done_int, // 34,
		.end   = IRQ_sdio_tran_done_int, // 34,
		.flags = IORESOURCE_IRQ,
	},
	[4] = {
		.start = IRQ_sdio_cmd_done_int, // 35,
		.end   = IRQ_sdio_cmd_done_int, // 35,
		.flags = IORESOURCE_IRQ,
	},
	[5] = {
		.start = IRQ_sdio_card_error_int, // 36,
		.end   = IRQ_sdio_card_error_int, // 36,
		.flags = IORESOURCE_IRQ,
	},
	[6] = {
		.start = IRQ_sdio_dma_int, // 37,
		.end   = IRQ_sdio_dma_int, // 37,
		.flags = IORESOURCE_IRQ,
	},
	[7] = {
		.start = IRQ_card_int, // 38,
		.end   = IRQ_card_int, // 38,
		.flags = IORESOURCE_IRQ,
	},
};


static u64 ka_sdio_dmamask = 0xffffffffUL;


struct ka_sdio_platdata ka_sdio_platdata = {
	.max_width	= 4,
	.clocks 	= ka_sdio_clksrcs,
	.irq_num	= 7,
};

struct platform_device ka_sdio_device = {
	.name		= "KeyAsic_sdhci",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(ka_sdio_resource),
	.resource	= ka_sdio_resource,
	.dev		= {
		.dma_mask		= &ka_sdio_dmamask,  // KATODO : check  dmamask usage
		.coherent_dma_mask	= 0xffffffffUL,
		.platform_data		= &ka_sdio_platdata,
	},
};

/*----------------------------------------------------------------------*/
/*
	SPI NOR Flash /Roger
*/

#define KA_PA_SSI_SPI	(0xa0001000)
#define KA_SZ_SSI_SPI	SZ_4K

static struct resource ka_ssi_spi_resource[] = {

	[0] = {
		.start = KA_PA_SSI_SPI,
		.end   = KA_PA_SSI_SPI + KA_SZ_SSI_SPI - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SSI,
		.end   = IRQ_SSI,
		.flags = IORESOURCE_IRQ,
	},

};

struct ka_spi_info {
	unsigned long		 pin_clk;
	unsigned long		 pin_mosi;
	unsigned long		 pin_miso;

	int			 num_chipselect;
	int			 bus_num;

	void (*chip_select)(struct ka_spi_info *spi, int cs);
};

static struct ka_spi_info ka_ssi_spi_data = {

	.pin_clk=NULL,
	.pin_mosi=NULL,
	.pin_miso=NULL,
	.num_chipselect=1,
	.bus_num=0,
};

/*
static struct device ka_ssi_spi_data = {

	.pdev = NULL,
	.clk = NULL,
	.prnt_clk = NULL,
	.num_slaves = 0,

}
*/
struct platform_device device_ka_ssi_spi = {

	.name				= "spi_KeyAsic_ssi",
	.id					= 0,
	.num_resources		= ARRAY_SIZE(ka_ssi_spi_resource),
	.resource			= ka_ssi_spi_resource,
	.dev 				= {
							.platform_data = &ka_ssi_spi_data,
						}

};



static struct flash_platform_data ka2000_spi_flashdata = { 

  .name="mx25l6405d",
  .type="mx25l6405d",
  .parts=&ka2000_spi_flash_partitions,
  .nr_parts=ARRAY_SIZE(ka2000_spi_flash_partitions),
}; 


static struct spi_board_info __initdata spi_nor_flash_info[] = {

	[0] = {
 		.modalias 		= "m25p80",
 		.platform_data = &ka2000_spi_flashdata,
    	.mode	    	= SPI_MODE_3,
    	.max_speed_hz 	= KA2000_OSC_CLOCK,
		.bus_num	 	= 0,
		.irq		 	= IRQ_SSI,
    	.chip_select 	= 0, //0:1 ff,1:1 no,1:0 err,
	},

};

#ifdef CONFIG_SERIAL_8250
/*
 * Only the second or "console" uart is connected on the ka2000.
 */
static struct resource ns16550_uart_resources[] = {
	{
		.start	= KA2000_UART0_BASE,
		.end	= KA2000_UART0_BASE + 0x0fff,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_UARTINT0,
		.end	= IRQ_UARTINT0,
		.flags	= IORESOURCE_IRQ,
	},
	{ },
};

static struct plat_serial8250_port ns16550_uart_platform_data[] = {
	{

		.mapbase	= KA2000_UART0_BASE,
		.membase	= (char *)IO_ADDRESS(KA2000_UART0_BASE),
		.irq		= IRQ_UARTINT0,
		//.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.flags = UPF_BOOT_AUTOCONF | UPF_IOREMAP | UPF_SHARE_IRQ | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
#if HCLK_RATE==1
		.uartclk	= KA2000_OSC_CLOCK,
#elif HCLK_RATE==2
        .uartclk	= KA2000_PLL_CLOCK/2,
#else
        .uartclk	= KA2000_PLL_CLOCK/(1<<HCLK_RATE),
#endif

	},

	{ },
};

static struct platform_device ns16550_uart_device = {
	.name		= "serial8250",
	.id		= 0,
	.dev			= {
		.platform_data	= ns16550_uart_platform_data,
	},
	.num_resources	= 2,
	.resource	= ns16550_uart_resources,
};
#endif



static void __init
ka2000_evm_map_io(void)
{
	ka2000_map_common_io();
}


static __init void ka2000_evm_init(void)
{
#ifdef CONFIG_SERIAL_8250
    platform_device_register(&ns16550_uart_device);
#endif
	platform_device_register(&ka_sdio_device);
	platform_device_register(&device_ka_ssi_spi);
	spi_register_board_info(&spi_nor_flash_info, 1) ; //ARRAY_SIZE(spi_nor_flash_info));

	platform_device_register(&ka_sdmmc_device);



	//ka2000_psc_init();   this is mark by kathy


	//platform_add_devices(ka2000_evm_devices,ARRAY_SIZE(ka2000_evm_devices));  this is mark by kathy
	//evm_init_i2c();   this is mark by kathy

	/* irlml6401 sustains over 3A, switches 5V in under 8 msec */
	//setup_usb(500, 8);   this is mark by kathy
}

void ka2000_system_reset(void)
{

}

int ka_platform_driver_probe(struct platform_driver *driver,
                int (*probe)(struct platform_device *))
{
	return platform_driver_probe(driver, probe);
}
EXPORT_SYMBOL(ka_platform_driver_probe);

struct resource *ka_platform_get_resource(struct platform_device *d, unsigned int t, unsigned int s)
{
	return platform_get_resource(d, t, s);
}
EXPORT_SYMBOL(ka_platform_get_resource);

void ka_platform_driver_unregister(struct platform_driver *d)
{
	platform_driver_unregister(d);
}
EXPORT_SYMBOL(ka_platform_driver_unregister);


struct workqueue_struct *ka_create_workqueue(const char *lock_name)
{
	return create_workqueue(lock_name);
}
EXPORT_SYMBOL(ka_create_workqueue);

int ka_queue_delayed_work(struct workqueue_struct *wq,
                        struct delayed_work *work, unsigned long delay)
{
	queue_delayed_work(wq, work, delay);
}
EXPORT_SYMBOL(ka_queue_delayed_work);


struct pid *ka_find_vpid(int nr)
{
	return find_vpid(nr);
}
EXPORT_SYMBOL(ka_find_vpid);

#if 0
#include <linux/elfcore.h>

/* * Capture the user space registers if the task is not running (in user space) */
int dump_task_regs(struct task_struct *tsk, elf_gregset_t *regs)
{
	struct pt_regs ptregs = *task_pt_regs(tsk);
	elf_core_copy_regs(regs, &ptregs);
	return 1;
}
#endif
static __init void ka2000_evm_irq_init(void)
{
	ka2000_init_common_hw();
	ka2000_irq_init();
}

MACHINE_START(KA2000_EVM, "KeyASIC Ka2000 EVM")
	/* Maintainer: MontaVista Software <source@mvista.com> */
	.phys_io      = IO_PHYS,
	.io_pg_offst  = (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params  = 0x100, //(KA2000_DDR_BASE + 0x200),
	.map_io	      = ka2000_evm_map_io,
	.init_irq     = ka2000_evm_irq_init,
	.timer	      = &ka2000_timer,           //kathy
	.init_machine = ka2000_evm_init,
MACHINE_END

