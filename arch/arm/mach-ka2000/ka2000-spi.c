/*
 * KeyASIC KA2000 series software
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
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <mach/ka2000.h>
#include <linux/autoconf.h>
#ifndef CONFIG_KA2000_PRINTK_ENABLE
#define printk dprintk
static inline int dprintk(const char *fmt, ...)
{
      return 0;
}
#endif
#define MULTE 					(0x1 << 14)
#define MIMSK 					(0x1 << 13)
#define TAGD  					(0x1 << 12)
#define SPIMW 					(0x1 << 11)
#define MSTR  					(0x1 << 10)
#define CSO   					(0x1 << 9)
#define ENSCK 					(0x1 << 8)
#define SMOD_PO 				(0x0 << 6)
#define SMOD_INT 				(0x1 << 6)
#define SMOD_DMA 				(0x2 << 6)
#define DRD   					(0x1 << 5)
#define DTD   					(0x1 << 4)
#define CSLV  					(0x1 << 3)
#define KEEP  					(0x1 << 2)
#define CPOL  					(0x1 << 1)
#define CPHA  					(0x1 << 0)
#define TMOD_BYTE  				(0x0 << 4)
#define TMOD_HWORD 				(0x1 << 4)
#define TMOD_WORD  				(0x2 << 4)
#define TMOD_MARK  				(0x2 << 4)
#define SPI_CLK_SRC 			(0x7 << 20)
#define SPI_ProgramMode			(0x1 << 29)
#define REGIF_BASE              0xA0000000
#define SCU_CLK_SRC_CTL			IO_ADDRESS( 0xA0000000 )
#define CLK_SRC_BASE			IO_ADDRESS( 0xA0000000 )
#define SSI_BASE                IO_ADDRESS( 0xA0000000 + 0x1000 )
#define SSI_PRE                 IO_ADDRESS( 0xA0000000 + 0x1000 + 0x00)
#define SSI_CON                 IO_ADDRESS( 0xA0000000 + 0x1000 + 0x04)
#define SSI_STA                 IO_ADDRESS( 0xA0000000 + 0x1000 + 0x08)
#define SSI_TDAT                IO_ADDRESS( 0xA0000000 + 0x1000 + 0x0c)
#define SSI_RDAT                IO_ADDRESS( 0xA0000000 + 0x1000 + 0x10)
#define SCU_PLL_FREQ_SEL1	 	IO_ADDRESS( 0xA0000000 + 0x04 )
#define SCU_PLL_FREQ_SEL2	 	IO_ADDRESS( 0xA0000000 + 0x08 )
#define spidelay(x) ndelay(x)
static inline void setsck(struct spi_device *dev, int on){}
static inline void setmosi(struct spi_device *dev, int on){}
static inline u32 getmiso(struct spi_device *dev){u32 dummy;return dummy;}
struct
ka_spi_info {
	unsigned long		 pin_clk;
	unsigned long		 pin_mosi;
	unsigned long		 pin_miso;
	int			 num_chipselect;
	int			 bus_num;
	void (*chip_select)(struct ka_spi_info *spi, int cs);
};
struct
ka_spi {
	struct spi_bitbang		 	bitbang;
	struct ka_spi_info			*info;
	struct platform_device		*dev;
};

#define ssi_write(val, addr) 	__raw_writel((val), (addr))
#define ssi_read(addr)  	    __raw_readl((addr))

static void ssi_wait_ready()
{
    int time_out = 40000;
	while((ssi_read(SSI_STA) & 0x1) == 0)
	{
	    if (time_out-- <= 0)
            break;

	    barrier();
	}
}

static inline void ssi_send(u32 word)
{
	ssi_wait_ready();
	ssi_write(word,SSI_TDAT);
}
static inline u32 ssi_recv(void)
{
	ssi_wait_ready(); 
	return ssi_read(SSI_RDAT);
}

static inline struct ka_spi *spidev_to_sg(struct spi_device *spi)
{
	return spi_master_get_devdata(spi->master);
}

/* set spi transfer bus width */
static ssi_bits_per_word = 0;
static int ka_ssi_setup_TMOD(u8 bits_per_word)
{
    volatile u32 dat;

    if (ssi_bits_per_word == bits_per_word)
        return 0;

    ssi_bits_per_word = bits_per_word;

	if(bits_per_word != 8)
	{
	    printk("TMOD Not 8 Recv %x\n",bits_per_word);
	    bits_per_word = 8;
	}

    ssi_wait_ready();

    dat = ssi_read(SSI_STA);
    dat &= ~TMOD_MARK;

	if (bits_per_word <= 8)
	{
		dat |= TMOD_BYTE;
	}
	else if (bits_per_word <= 16)
	{
		dat |= TMOD_HWORD;
	}
	else if (bits_per_word <= 32)
	{
		dat |= TMOD_WORD;
	}
	else
	{
		return -EINVAL;
	}

    ssi_wait_ready();
    ssi_write(dat, SSI_STA);

	return 0;
}
static u32 ka_spi_txrx_mode0(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	ssi_wait_ready(); 
	ssi_write(
		SPIMW+MSTR+ENSCK+SMOD_PO+CSO,
		SSI_CON
	);
	ka_ssi_setup_TMOD(bits);
	ssi_send(word);

	return ssi_recv();
}
static u32 ka_spi_txrx_mode1(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	ssi_wait_ready(); 
	ssi_write(
		SPIMW+MSTR+ENSCK+SMOD_PO+CSO+CPHA,
		SSI_CON
	);
	ka_ssi_setup_TMOD(bits);
	ssi_send(word);

	return ssi_recv();
}
static u32 ka_spi_txrx_mode2(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
	ssi_wait_ready(); 
	ssi_write(
		SPIMW+MSTR+ENSCK+SMOD_PO+CSO+CPOL,
		SSI_CON
	);
	ka_ssi_setup_TMOD(bits);
	ssi_send(word);

	return ssi_recv();
}
static u32 ka_spi_txrx_mode3(struct spi_device *spi,
				      unsigned nsecs, u32 word, u8 bits)
{
    u32 val;

    //preempt_disable();
	ssi_wait_ready();

	ssi_write(
		SPIMW+MSTR+ENSCK+SMOD_PO+CSO+CPOL+CPHA,
		SSI_CON
	);

    if (ssi_bits_per_word != bits)
        ka_ssi_setup_TMOD(bits);

	ssi_send(word);
	val = ssi_recv();
	//preempt_enable();
	
	return val;
}
static void ka_spi_chipselect(struct spi_device *dev, int value)
{
    struct ka_spi *sg = spidev_to_sg(dev);
	if(value){
		ssi_wait_ready();
		ssi_write(
			SPIMW+MSTR+ENSCK+SMOD_PO+CSO+CPOL+CPHA,
			SSI_CON
		);
	}else{
		ssi_wait_ready();
		ssi_write(
			SPIMW+MSTR+ENSCK+SMOD_PO+CPOL+CPHA,
			SSI_CON
		);
	}

	if (sg->info && sg->info->chip_select)
		(sg->info->chip_select)(sg->info, value);
}

/* spi clock selection define at register 0xa0000 0000 bit 20 */
static int ka_spi_probe(struct platform_device *dev)
{
	struct ka_spi_info 		*info;
	struct spi_master		*master;
	struct ka_spi  			*sp;
	int ret;
    struct clk *spi_clk;
    int clk_rate = 0;
    u32 val;
    //printk("ka_spi_probe\n");
    /* initial at u-boot */
	//ssi_write(0x00000000,SCU_PLL_FREQ_SEL1);
	//ssi_write(0x00000000,SCU_PLL_FREQ_SEL2);
	//ssi_write(0x01c22200,SCU_CLK_SRC_CTL);
	/* enable spi clock */
    //ssi_write(ssi_read(SCU_CLK_SRC_CTL) | (1 << 21),SCU_CLK_SRC_CTL);

    spi_clk = clk_get(NULL, "SPICLK");
    //printk("spi clock rate %d\n", clk_get_rate(spi_clk));
	/*This field contains the prescaler value for the baud rate
        Baud rate = PCLK / 2 / (SSIPRE+1)
    */
    ssi_wait_ready();
    clk_rate = (clk_get_rate(spi_clk) / 48000000) - 1;
    if (clk_rate < 0)
    {
        clk_rate = 0;
    }
    //printk("SSI_PRE %d\n", clk_rate);

    ssi_write(clk_rate, SSI_PRE );
    //Initialize SSI Controller (NOR Flash)
    //ssi_wait_ready(); //Transfer Ready
    //ssi_write(0x0, SSI_PRE); //baud rate = PCLK/2/(SSI_PRE+1)

    //printk("spi_alloc_master\n");
	master = spi_alloc_master(&dev->dev, sizeof(struct ka_spi));
	if (master == NULL)
	{
		dev_err(&dev->dev, "failed to allocate spi master\n");
		ret = -ENOMEM;
		goto err;
	}
	sp = spi_master_get_devdata(master);

    //printk("platform_set_drvdata dev %p, sp %d\n", dev, sp);
	platform_set_drvdata(dev, sp);
	info = sp->info = dev->dev.platform_data;
	sp->bitbang.master = spi_master_get(master);
	sp->bitbang.master->bus_num = info->bus_num;
	sp->bitbang.master->num_chipselect = info->num_chipselect;
	sp->bitbang.chipselect = ka_spi_chipselect;
	sp->bitbang.txrx_word[SPI_MODE_0] = ka_spi_txrx_mode0;
	sp->bitbang.txrx_word[SPI_MODE_1] = ka_spi_txrx_mode1;
	sp->bitbang.txrx_word[SPI_MODE_2] = ka_spi_txrx_mode2;
	sp->bitbang.txrx_word[SPI_MODE_3] = ka_spi_txrx_mode3;
	ret = spi_bitbang_start(&sp->bitbang);
	//printk("spi_bitbang_start ret %d\n", spi_bitbang_start);
	if (ret)
	{
	    goto err_no_bitbang;
	}

	return 0;
 err_no_bitbang:
	spi_master_put(sp->bitbang.master);
 err:
	return ret;
}

static int ka_spi_remove(struct platform_device *dev)
{
	struct ka_spi *sp = platform_get_drvdata(dev);
	spi_bitbang_stop(&sp->bitbang);
	spi_master_put(sp->bitbang.master);
	return 0;
}
static struct platform_driver ka_spi_drv = {
		.probe		= ka_spi_probe,
        .remove		= ka_spi_remove,
        .suspend	= NULL,
        .resume		= NULL,
        .driver		= {
		.name	= "spi_KeyAsic_ssi",
		.owner	= THIS_MODULE,
},};
static int __init ka_spi_init(void)
{
    //printk("ka_spi_init\n");
    return platform_driver_register(&ka_spi_drv);
}
static void __exit ka_spi_exit(void)
{
    platform_driver_unregister(&ka_spi_drv);
}
module_init(ka_spi_init);
module_exit(ka_spi_exit);
MODULE_ALIAS("platform:spi_ka_ssi");
MODULE_DESCRIPTION("KeyAsic SPI Driver(SSI)");
MODULE_AUTHOR("Apikbiz");
MODULE_LICENSE("GPL");
