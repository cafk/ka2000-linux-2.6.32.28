/*
 * KeyASIC KA2000 clock config file
 *
 * Copyright (C) 2010 KeyASIC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

Get Clock Example:
    struct clk *clock = clk_get(NULL, "SPICLK");
    unsigned int clk_rate = clk_get_rate(clock);

 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <asm/cacheflush.h>
#include <mach/psc.h>
#include <mach/ka2000.h>
#include <mach/clock.h>
#include <mach/system.h>
#include <asm/processor.h>
#include <asm/system.h>

#define SDIO_OSC_CLOCK_ONLY 1
#define DELAY_CHANGE_CLOCK 1

#if 0//ndef CONFIG_KA2000_PRINTK_ENABLE
#define printk dprintk
static inline int dprintk(const char *fmt, ...)
{
      return 0;
}
#else 
#undef printk
#endif
int go_sleep_count = 50; //300;
int go_sleep_enabled = 1;
EXPORT_SYMBOL(go_sleep_count);
EXPORT_SYMBOL(go_sleep_enabled);
int detect_cmd = 0;
int slow_mode = 0;
EXPORT_SYMBOL(detect_cmd);
EXPORT_SYMBOL(slow_mode);
int sdio_wakeup = 0;
EXPORT_SYMBOL(sdio_wakeup);
#define UARTC(ch);        *(volatile u32 __force *) (0x55000000 + 0xa0004000) = (ch);

#define word_write(a,v) 	__raw_writel(v, IO_ADDRESS(a))
#define word_read(a)  		__raw_readl(IO_ADDRESS(a))
#define byte_write(a,v) 	__raw_writeb(v, IO_ADDRESS(a))
#define byte_read(a)  		__raw_readb(IO_ADDRESS(a))

/* scu */
#define KA_SCU_BASE  0xa0000000
#define SCU_CLK_SRC_CTL		 KA_SCU_BASE + 0x00
#define SCU_PLL_FREQ_SEL1	 KA_SCU_BASE + 0x04
#define SCU_PLL_FREQ_SEL2	 KA_SCU_BASE + 0x08

static LIST_HEAD(clocks);
static DEFINE_MUTEX(clocks_mutex);
static DEFINE_SPINLOCK(clockfw_lock);

static unsigned int commonrate = KA2000_PLL_CLOCK;
static unsigned int armrate    = KA2000_PLL_CLOCK;

/* these module by pass PLL */
static unsigned int fixedrate  = KA2000_OSC_CLOCK;
static unsigned int mmcrate    = KA2000_OSC_CLOCK;
static unsigned int pwmrate    = KA2000_OSC_CLOCK;

/* SPI also have to set SSI_PRE */
static unsigned int spirate    = KA2000_OSC_CLOCK; //KA2000_PLL_CLOCK/8;

static unsigned int uartrate   = KA2000_PLL_CLOCK/HCLK_RATE;
static unsigned int pclkrate   = KA2000_PLL_CLOCK/HCLK_RATE;
static unsigned int scaler0rate= KA2000_PLL_CLOCK;
static unsigned int scaler1rate= KA2000_PLL_CLOCK;

static unsigned int sdiorate = KA2000_OSC_CLOCK / 4;
static unsigned int hsmmc = KA2000_OSC_CLOCK;
static unsigned int hsmmc2 = KA2000_OSC_CLOCK;
static unsigned int mmc_bus = KA2000_OSC_CLOCK;

//static int chip_version = 'D';
extern void ka2000_psc_config(unsigned int domain, unsigned int id, char enable);
extern void pwm_set(int cmd);
void set_clock_set(int set);
void set_osc_clock(void);
static u32 ka2000_get_pll_clock(u32 *hclk);
void ka2000_set_sdio_clock(u32 rate);
void update_clocks(u32 pll_clock, u32 hclk);

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>


#define PLL_INIT_FINISH (1 << 28)           
#define HIGH_SPEED_CLKVAL   0x01c33703 //0x01c33717 //org:0x01c33303
#define PANA_CLKVAL         0x10c33700 //0x10c26700 //0x10c33703

#define OSC 0
#define PLL 1
#define BIT9 (1 << 9)

#define CLK_24_24_24  0
#define CLK_96_48_48  1
#define CLK_192_96_96 2
#define CLK_192_192_96 3
#define CLK_96_96_96 4
#define CLK_192_48_48 5
#define CLK_192_96_48 6
#define CLK_192_24_24 7
#define CLK_210_210_105 8
#define CLK_156_78_78 9
#define CLK_12_12_12 10
#define CLK_96_96_48 11
//#define CLK_DEFAULT CLK_192_192_96 //CLK_192_192_96 //CLK_192_96_96
//#define CLK_DEFAULT 0
#if defined(CONFIG_PLL_CPU_HCLK_24_24_24)
	#define CLK_DEFAULT  CLK_24_24_24
#elif defined(CONFIG_PLL_CPU_HCLK_96_48_48)
       #define CLK_DEFAULT  CLK_96_48_48
#elif defined(CONFIG_PLL_CPU_HCLK_192_96_96)
       #define CLK_DEFAULT CLK_192_96_96
#elif defined(CONFIG_PLL_CPU_HCLK_192_192_96)
       #define CLK_DEFAULT CLK_192_192_96
#elif defined(CONFIG_PLL_CPU_HCLK_96_96_96)
       #define CLK_DEFAULT CLK_96_96_96
#elif defined(CONFIG_PLL_CPU_HCLK_192_48_48)
       #define CLK_DEFAULT CLK_192_48_48
#elif defined(CONFIG_PLL_CPU_HCLK_192_96_48)
       #define CLK_DEFAULT CLK_192_96_48
#elif defined(CONFIG_PLL_CPU_HCLK_192_24_24)
       #define CLK_DEFAULT CLK_192_24_24
#elif defined(CONFIG_PLL_CPU_HCLK_210_210_105)
       #define CLK_DEFAULT CLK_210_210_105
#elif defined(CONFIG_PLL_CPU_HCLK_156_78_78)
       #define CLK_DEFAULT CLK_156_78_78
#elif defined(CONFIG_PLL_CPU_HCLK_12_12_12_FPGA)
       #define CLK_DEFAULT CLK_12_12_12
#elif defined(CONFIG_PLL_CPU_HCLK_96_96_48)
       #define CLK_DEFAULT CLK_96_96_48
#endif
// 0 : OSC/PLL, 1 : pll_clock, 2 : armrate, 3 : hclk
// 4 : 0xa0000000, 5 : 0xa0000004, 6 : 0xa0000008, 7: 0xa000000c, 8 : 0xa0004000
/*
	PLL Frequency Select Register 2 (PLL_FREQ_SEL2)
	0xa0000000 + 0x8
	sdio_pll_sel_reg [12:8] R/W SDIO PLL Divider select
	5・b00001 : PLL is div 2
	5・b00010 : PLL is div 3
	5・b00100 : PLL is div 4
	5・b01000 : PLL is div 6
	5・b10000 : PLL is div 8
*/

#define OSC_R0 0x10c22610
#define PLL_R0 0x11c22601  //0x11c33601


#if 0
#define DIV_PLL_024 0x00000000
#define DIV_PLL_096 0x00010401
#define DIV_PLL_192 0x00041004
#else
#define DIV_PLL_024 0x00000000
#define DIV_PLL_096 0x00040404
#define DIV_PLL_192 0x00101010
#define DIV_PLL_240 0x00101010
#define DIV_PLL_156 0x00808080
#endif

u32 clock_regs[][9] = {
    {OSC,  24, 24, 1, OSC_R0 | 0, 0x00000000, DIV_PLL_024, 0x00000001, 0x26},   // pll-cpu-hclk 24-24-24 
    {PLL,  96, 48, 1, PLL_R0 | 1, 0x00000001, DIV_PLL_096, 0x0000df00, 0xa1/2},   // pll-cpu-hclk 96-48-48 
                     
	{PLL, 192, 96, 1, PLL_R0 | 1, 0x00000001, DIV_PLL_192, 0x00009f00, 0xa1},   // pll-cpu-hclk 192-96-96    
	{PLL, 192,192, 2, PLL_R0 | 3, 0x00000000, DIV_PLL_192, 0x00009f00, 0xa1},   // pll-cpu-hclk 192-192-96	   
	{PLL,  96, 96, 1, PLL_R0 | 1, 0x00000000, DIV_PLL_096, 0x0000df00, 0xa1},   // pll-cpu-hclk 96-96-96
    {PLL, 192, 48, 1, PLL_R0 | 1, 0x00000004, DIV_PLL_192, 0x00009f00, 0xa1/2},   // pll-cpu-hclk 192-48-48       
    
    {PLL, 192, 96, 2, PLL_R0 | 3, 0x00000001, DIV_PLL_192, 0x00009f00, 0xa1/2},   // pll-cpu-hclk 192-96-48    
    {PLL, 192, 24, 1, PLL_R0 | 1, 0x00000010, DIV_PLL_192, 0x00009f00, 0xa1/4},   // pll-cpu-hclk 192-24-24 
    {PLL, 210, 210, 2, PLL_R0 | 3, 0x00000000, DIV_PLL_240, 0x0000a200, 0xa2},   // pll-cpu-hclk 210-210-105
    {PLL, 156, 78, 1, PLL_R0 | 1, 0x00000001, DIV_PLL_156, 0x00009900, 0x7a},   // pll-cpu-hclk 156-78-78
    {OSC,  12, 12, 1, OSC_R0 | 0, 0x00000000, DIV_PLL_024, 0x00000001, 0x26},   // FPGA
    {PLL,  96, 96, 2, PLL_R0 | 3, 0x00000000, DIV_PLL_096, 0x0000df00, 0xa1/2},   // pll-cpu-hclk 96-96-48
};

/*
u32 clock_regs[][9] = {
    {OSC,  24, 24, 1, 0x10c22610, 0x00000000, 0x00101010, 0x00000001, 0x26},   // pll-cpu-hclk 24-24-24 
    {PLL,  96, 48, 1, 0x11c33601, 0x00000004, 0x00040404, 0x0000df00, 0xa1/4},   // pll-cpu-hclk 96-48-48 
                     
        {PLL, 192, 96, 1, 0x11c33601, 0x00000001, 0x00101010, 0x00009f00, 0xa1},   // pll-cpu-hclk 192-96-96    
        {PLL, 192,192, 2, 0x11c33603, 0x00000000, 0x00101010, 0x00009f00, 0xa1},   // pll-cpu-hclk 192-192-96    
        {PLL,  96, 48, 1, 0x11c33601, 0x00000004, 0x00040404, 0x0000df00, 0xa1/4},   // pll-cpu-hclk 96-48-48    
        {PLL,  96, 96, 2, 0x11c33603, 0x00000000, 0x00041004, 0x0000df00, 0xa1/2},   // pll-cpu-hclk 96-96-96
    {PLL, 192, 48, 1, 0x11c33601, 0x00000004, 0x00101010, 0x00009f00, 0xa1/2},   // pll-cpu-hclk 192-48-48       
    
    {PLL, 192, 96, 2, 0x11c33603, 0x00000001, 0x00041004, 0x00009f00, 0xa1/2},   // pll-cpu-hclk 192-96-96    
    {PLL, 192, 24, 1, 0x11c33601, 0x00000010, 0x00101010, 0x00009f00, 0xa1/4},   // pll-cpu-hclk 192-24-24 
    {OSC,  24, 24, 1, 0x10c22610, 0x00000000, 0x00000000, 0x00000001, 0x26},   // pll-cpu-hclk 24-24-24 
    {OSC,  12, 12, 1, 0x10c22610, 0x00000000, 0x00000000, 0x00000001, 0x26},   // FPGA
};

*/
#if DELAY_CHANGE_CLOCK                  
int cur_set = 0;


extern struct mmc_host *sd_host;
extern struct mmc_host *sdio_host;
#define SLOW_MODE 1
#define HIGH_MODE 0
struct timer_list chclk_timer;
static void callback(unsigned long);
struct clock_settings {
    int inited;
    int mode;
    int new_mode;
    u64 jiffies;
};
static struct clock_settings clock_setting;

#define SDSW_M1_CURR_CMD_REG 0xa000a040
extern int sdio_busy;
static void ka2000_clk_callback(unsigned long data)
{
    struct clock_settings *clk_set = (struct clock_settings*) data;
    u32 pll_clock, hclk = 0;
	u32 cmd, m1_busy = 0;
	
	//UARTC(clk_set->mode ? 's' : 'h');     

	cmd = word_read(SDSW_M1_CURR_CMD_REG) & 0x7f;
	if (detect_cmd)
		m1_busy = cmd > 16; //(cmd == 18 || cmd == 24 || cmd == 25 || cmd == 86);
	else
		m1_busy = (cmd == 25);

	if (m1_busy)
	{
		//UARTC('M');
		clk_set->jiffies = get_jiffies_64() + HZ*2;
		clk_set->new_mode = SLOW_MODE;	
		slow_mode = 1;
	}			
	//if (m1_busy)
	//	printk("%d,", cmd);
    if (clk_set->mode == clk_set->new_mode)
    {         
        //if (clk_set->mode == HIGH_MODE && (get_jiffies_64() >= clk_set->jiffies))
        //{
        //    clk_set->new_mode = SLOW_MODE;    
        //}

    	mod_timer(&chclk_timer, get_jiffies_64() + ((clk_set->mode==SLOW_MODE) ? 20 : 2));
    	return; 

    }	    

    if (clk_set->mode == SLOW_MODE)
    {     	
		if (m1_busy || get_jiffies_64() < clk_set->jiffies)
        {
        	//UARTC('N');
            mod_timer(&chclk_timer, get_jiffies_64() + 10);
            return;        
        }
    }	
                  
	local_irq_disable();
	preempt_disable();	

    if (clk_set->new_mode == SLOW_MODE)
        set_clock_set(CLK_24_24_24);
    else
        set_clock_set(CLK_DEFAULT);
    clk_set->mode = clk_set->new_mode;    

	preempt_enable();
	local_irq_enable();    
	    
    //printk("WAKE:%08x %08x %08x %08x\n", ka2000_readl(0xa0000000),ka2000_readl(0xa0000004),ka2000_readl(0xa0000008),ka2000_readl(0xa000000c));
    UARTC(clk_set->new_mode ? 'S' : 'H');   

	mod_timer(&chclk_timer, get_jiffies_64() + 2);   

}
                                        
static void ka2000_clk_change_mode(int mode)
{
    clock_setting.new_mode = mode;

    //UARTC(mode ? 'S' : 'H');            
  
    /* if change to slow mode again, delay the next change to highter mode, keep at slow mode at least 5 sec */
    if (clock_setting.new_mode == SLOW_MODE)
        clock_setting.jiffies = get_jiffies_64() + HZ*2;
}

static int init_clock_timer(void)
{
    if (clock_setting.inited)
        return 0;
    clock_setting.inited = 1;
    init_timer(&chclk_timer);
    chclk_timer.expires = get_jiffies_64() + 5 * HZ;
    chclk_timer.function = &ka2000_clk_callback;
    chclk_timer.data = (unsigned long) &clock_setting;
    add_timer(&chclk_timer);
    
    mod_timer(&chclk_timer, get_jiffies_64() + 30);
    return 0;
}

static void del_clock_timer(void)
{
    del_timer(&chclk_timer);
}
#endif
void ka2000_set_sdio_clock(u32 rate)
{
#if SDIO_OSC_CLOCK_ONLY
	u32 val;
	return;

	val = ka2000_readl(0xa0000000);
	//val &= ~(1 << 10);  
	val &= ~(1 << 8);
	ka2000_writel(val | (1 << 9), 0xa0000000);
#else
    /*
        PLL Frequency Select Register 2 (PLL_FREQ_SEL2)
        0xa0000000 + 0x8
        sdio_pll_sel_reg [12:8] R/W SDIO PLL Divider select
        5・b00001 : PLL is div 2
        5・b00010 : PLL is div 3
        5・b00100 : PLL is div 4
        5・b01000 : PLL is div 6
        5・b10000 : PLL is div 8
    */
    u32 val;
    u32 div, b;
    u8 v[] = {0, 1, 2, 4, 4, 8, 8, 0x10};

	//if (sdiorate == rate)
	//	return;

    if (rate < commonrate / 8)
        rate = commonrate / 8;
    if (rate > commonrate)
        rate = commonrate;

    div = commonrate / rate;

    if (div > 8)
        div = 8;
	if (div == 0)
		div = 1;

    b = v[div-1];
    /* read PLL_FREQ_SEL2 */
    val = ka2000_readl(0xa0000008);
    val &= ~(0x1f << 8);
    val |= b << 8;
	barrier();
    ka2000_writel(val, 0xa0000008);

    /*
        Clock Source Control Register (CLK_SRC_CTL)
        0xa0000000 + 0x0
        sdio_clk_out_sel_reg [10] R/W SDIO_CLK output select
        1・b0: select sdio_clk
        1・b1: select sdio_clk_b

        sdio_clk_en_reg [9] R/W SDIO_CLK enable
        1・b0: SDIO_CLK disable
        1・b1: SDIO_CLK enable

        sdio_clk_sel_reg [8] R/W SDIO_CLK source select
        1・b0: select OSC source
        1・b1: select PLL source
    */

    val = ka2000_readl(0xa0000000);
    printk("pre-wifi clk:%x, %d\n", val, div);
	//val &= ~(1 << 10);

    /*if (val & (1 << 8))
        printk("SDIO - PLL,");
    else
        printk("SDIO - OSC,");

    if (val & (1 << 10))
        printk("clock b,");
    else
        printk("clock a,"); */

    //printk("rate %d, div %d, b %d\n", rate, div, b);
    barrier();
    ka2000_writel(val | (1 << 9) /*| (1 << 8)*/, 0xa0000000);

    sdiorate   = rate;
	printk("set_sdio_clock to %d(%d)(%08x %08x %08x %08x)\n", rate,commonrate,ka2000_readl(0xa0000000),ka2000_readl(0xa0000004),ka2000_readl(0xa0000008),ka2000_readl(0xa000000c));
#endif	
}


void ka2000_set_sdc_clock(u32 rate)
{
    /*
        PLL Frequency Select Register 2 (PLL_FREQ_SEL2)
        0xa0000000 + 0x8
        sdio_pll_sel_reg [4:0] R/W SDIO PLL Divider select
        5・b00001 : PLL is div 2
        5・b00010 : PLL is div 3
        5・b00100 : PLL is div 4
        5・b01000 : PLL is div 6
        5・b10000 : PLL is div 8
    */
    static int clk;
#define sdsw_clk_en_reg  (1 << 17)
#define	sdsw_clk_sel_pll (1 << 16)
#define sdhost_clk_en_reg (1 << 13)
#define sdhost_clk_sel_pll (1 << 12)

    u32 val;
    u32 div, b;
    u8 v[] = {0, 1, 2, 4, 4, 8, 8, 0x10};

return;
    //if (clk == rate)
    //    return;

	clk = rate;
	//val = ka2000_readl(0xa0000000);
	//val &= ~(sdsw_clk_en_reg | sdhost_clk_en_reg);
    //ka2000_writel(val, 0xa0000000);

    if (rate < commonrate / 8)
        rate = commonrate / 8;
    if (rate > commonrate)
        rate = commonrate;

    div = commonrate / rate;

    if (div > 8)
        div = 8;
	if (div == 0)
		div = 1;

    b = v[div-1];
    /* read PLL_FREQ_SEL2 */
    val = ka2000_readl(0xa0000008);
    val &= ~((0x1f << 16) |(0x1f << 0));
    val |= (b << 16) | (b << 0);

    ka2000_writel(val, 0xa0000008);

    /*
        Clock Source Control Register (CLK_SRC_CTL)
        0xa0000000 + 0x0
        sdm_clk_out_sel_reg [14] R/W SDHOST_CLK output select
        1・b0: select sdio_clk
        1・b1: select sdio_clk_b

        sdhhost_clk_en_reg [13] R/W SDHOST_CLK enable
        1・b0: SDhost_CLK disable
        1・b1: SDhost_CLK enable

        sdhost_clk_sel_reg [12] R/W SDHOST_CLK source select
        1・b0: select OSC source
        1・b1: select PLL source
    */

    val = ka2000_readl(0xa0000000);
	val &= ~(1 << 14);
    //if (val & sdhost_clk_sel_pll)
    //    printk("SDC - PLL source\n");
    //else
    //    printk("SDC - OSC source\n");

    printk("SD-%d,div %d,b %d\n", rate, div, b);

    ka2000_writel(val | sdsw_clk_sel_pll | sdsw_clk_en_reg | sdhost_clk_sel_pll | sdhost_clk_en_reg, 0xa0000000);

    hsmmc = rate;
    hsmmc2 = rate;
    mmc_bus = rate;
}

void ka2000_set_spi_clock(u32 rate)
{
    /*
        PLL Frequency Select Register 1 (PLL_FREQ_SEL1)
        0xa0000000 + 0x4
        spi_pll_sel_reg [20:16] R/W SPI PLL Divider select
        5・b00001 : PLL is div 2
        5・b00010 : PLL is div 3
        5・b00100 : PLL is div 4
        5・b01000 : PLL is div 6
        5・b10000 : PLL is div 8
    */
    u32 val;
    u32 div, b;
    u8 v[] = {0, 0, 1, 2, 4, 4, 8, 0x10};

    if (rate < commonrate / 8)
        rate = commonrate / 8;
    if (rate > commonrate)
        rate = commonrate;

    div = commonrate / rate;
    if (div == 5)
        div = 6;

    if (div > 7)
        div = 7;

    b = v[div];
    /* read PLL_FREQ_SEL2 */
    val = ka2000_readl(0xa0000004);
    val &= ~(0x1f << 16);
    val |= b << 16;

    ka2000_writel(val, 0xa0000004);

    /*
        Clock Source Control Register (CLK_SRC_CTL)
        0xa0000000 + 0x0

        spi_clk_en_reg [21] R/W SPI_CLK enable
        1・b0: SPI_CLK disable
        1・b1: SPI_CLK enable

        spi_clk_sel_reg [20] R/W SPI_CLK source select
        1・b0: select OSC source
        1・b1: select PLL source
    */

    val = ka2000_readl(0xa0000000);
    if (val & (1 << 20))
        printk("SPI - PLL source\n");
    else
        printk("SPI - OSC source\n");

    printk("SPI - Set PLL source, rate %d, div %d, b %d\n", rate, div, b);

    ka2000_writel(val | (1 << 20), 0xa0000000);

    spirate   = rate;
}

/*
 * Returns a clock. Note that we first try to use device id on the bus
 * and clock name. If this fails, we try to use clock name only.
 */
struct clk *clk_get(struct device *dev, const char *id)
{
    struct clk *p, *clk = ERR_PTR(-ENOENT);
    int idno;

    if (dev == NULL || dev->bus != &platform_bus_type)
        idno = -1;
    else
        idno = to_platform_device(dev)->id;

    mutex_lock(&clocks_mutex);

    list_for_each_entry(p, &clocks, node)
    {
        if (p->id == idno &&
                strcmp(id, p->name) == 0 && try_module_get(p->owner))
        {
            clk = p;
            goto found;
        }
    }

    list_for_each_entry(p, &clocks, node)
    {
        if (strcmp(id, p->name) == 0 && try_module_get(p->owner))
        {
            clk = p;
            break;
        }
    }

found:
    mutex_unlock(&clocks_mutex);

    return clk;
}
EXPORT_SYMBOL(clk_get);

void clk_put(struct clk *clk)
{
    if (clk && !IS_ERR(clk))
        module_put(clk->owner);
}
EXPORT_SYMBOL(clk_put);


/* at KA2000, all clock default enabled */
static int __clk_enable(struct clk *clk)
{
    if (clk->flags & ALWAYS_ENABLED)
        return 0;

    //ka2000_psc_config(KA2000_GPSC_ARMDOMAIN, clk->lpsc, 1); mark by kathy
    return 0;
}

static void __clk_disable(struct clk *clk)
{
    if (clk->usecount)
        return;

    //ka2000_psc_config(KA2000_GPSC_ARMDOMAIN, clk->lpsc, 0); mark by kathy
}

int clk_enable(struct clk *clk)
{
    unsigned long flags;
    int ret = 0;

    if (clk == NULL || IS_ERR(clk))
        return -EINVAL;

    if (clk->usecount++ == 0)
    {
        spin_lock_irqsave(&clockfw_lock, flags);
        ret = __clk_enable(clk);
        spin_unlock_irqrestore(&clockfw_lock, flags);
    }

    return ret;
}

EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
    unsigned long flags;

    if (clk == NULL || IS_ERR(clk))
        return;

    if (clk->usecount > 0 && !(--clk->usecount))
    {
        spin_lock_irqsave(&clockfw_lock, flags);
        __clk_disable(clk);
        spin_unlock_irqrestore(&clockfw_lock, flags);
    }
}
EXPORT_SYMBOL(clk_disable);

unsigned long clk_get_rate(struct clk *clk)
{
    if (clk == NULL || IS_ERR(clk))
        return -EINVAL;

    return *(clk->rate);
}
EXPORT_SYMBOL(clk_get_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
    if (clk == NULL || IS_ERR(clk))
        return -EINVAL;

    return *(clk->rate);
}
EXPORT_SYMBOL(clk_round_rate);

/* TODO : clock dynamic set rate */
int clk_set_rate(struct clk *clk, unsigned long rate)
{
    //static unsigned int fixedrate = KA2000_OSC_CLOCK;
    if (clk == NULL || IS_ERR(clk))
        return -EINVAL;

    /* changing the clk rate is not supported */

    if (clk->lpsc == KA2000_LPSC_MMC_SD)
    {
       ka2000_set_sdc_clock(rate);
       return 0;
    }
    if (clk->lpsc == KA2000_LPSC_MMC_SDIO)
    {
       ka2000_set_sdio_clock(rate);
       return 0;
    }
    return -EINVAL;
}
EXPORT_SYMBOL(clk_set_rate);

int clk_register(struct clk *clk)
{
    if (clk == NULL || IS_ERR(clk))
        return -EINVAL;

    mutex_lock(&clocks_mutex);
    //printk("clk_register : name %s, rate %d\n", clk->name, *(clk->rate));
    list_add(&clk->node, &clocks);
    mutex_unlock(&clocks_mutex);

    return 0;
}
EXPORT_SYMBOL(clk_register);

void clk_unregister(struct clk *clk)
{
    if (clk == NULL || IS_ERR(clk))
        return;

    mutex_lock(&clocks_mutex);
    list_del(&clk->node);
    mutex_unlock(&clocks_mutex);
}
EXPORT_SYMBOL(clk_unregister);


unsigned long ka2000_clk_get_rate(void)
{
    return armrate;
}

EXPORT_SYMBOL(ka2000_clk_get_rate);

#if 1
//==============================================================================
static unsigned long clk_pwm_scaler_getrate(struct clk *clk)
{
    //return clk_get_rate(clk->parent) / (tcfg0 + 1);
    return clk_get_rate(clk->parent);
}

static int clk_default_setrate(struct clk *clk, unsigned long rate)
{
    //clk->rate = rate;
    return 0;
}
//==============================================================================
#endif




static struct clk clk_timer_scaler[] = {
    [0]	= {
        .name		= "pwm-scaler0",
        .id			= -1,
        .rate		= &scaler0rate,
        .lpsc 		= -1,
        .get_rate	= clk_get_rate,
        .flags 		= ALWAYS_ENABLED,
    },
    [1]	= {
        .name		= "pwm-scaler1",
        .id			= -1,
        .rate		= &scaler1rate,
        .lpsc 		= -1,
        .get_rate	= clk_get_rate,
        .flags 		= ALWAYS_ENABLED,
    },
};


static struct clk clk_hsmmc[] = {

	{
		.name		= "hsmmc",
		.id			= -1,
		.rate		= &hsmmc,
		.lpsc 		= -1,
		.get_rate	= clk_get_rate,
		.flags 		= ALWAYS_ENABLED,
	}, {
		.name		= "hsmmc2",
		.id			= -1,
		.rate		= &hsmmc2,
		.lpsc 		= -1,
		.get_rate	= clk_get_rate,
		.flags 		= ALWAYS_ENABLED,
	}, {
		.name		= "mmc_bus",
		.id			= -1,
		.rate		= &mmc_bus,
		.lpsc 		= -1,
		.get_rate	= clk_get_rate,
		.flags 		= ALWAYS_ENABLED,
	}
};


struct clk clk_p =
{
    .name		= "pclk",
    .id			= -1,
    .rate		= &pclkrate,
    .lpsc 		= -1,
    .parent		= NULL,
    .ctrlbit	= 0,
    .set_rate	= clk_set_rate,
    .flags 		= ALWAYS_ENABLED,
};


static struct clk ka2000_clks[] =
{
    {
        .name = "ARMCLK",
        .id			= -1,
        .rate = &armrate,
        .lpsc = -1,
        .flags = ALWAYS_ENABLED,
    },
    {
        .name = "PWM",
        .id = -1,
        .rate = &pwmrate,
        .lpsc 	= -1,
        .flags 	= ALWAYS_ENABLED,
    },
    {
        .name = "UART",
        .id = -1,
        .rate = &uartrate,
        .lpsc 		= -1,
        .flags 		= ALWAYS_ENABLED,
    },
    {
        .name = "MMCSDCLK",
        .id = -1,
        .rate = &mmcrate,
        .lpsc = KA2000_LPSC_MMC_SD,
    },
    {
        .name = "SPICLK",
        .id = -1,
        .rate = &spirate,
        .lpsc = KA2000_LPSC_SPI,
    },

    {
        .name = "I2CCLK",
        .rate = &fixedrate,
        .lpsc = KA2000_LPSC_I2C,
    },
    {
        .name = "SDIOCLK",
        .id = -1,
        .rate = &sdiorate,
        .lpsc = KA2000_LPSC_MMC_SDIO,
    },
};





static u32 ka2000_get_pll_clock(u32 *hclk)
{
    u32 val;
    u32 div_m, div_n, div_k;
    u32 mhz;
    u8 v[] = {1, 2, 3, 3, 4, 4, 4, 4, 6, 6, 6, 6, 6, 6, 6, 6, 8};

    /* read PLL setting register */
    if((ka2000_readl(0xa0000000) & 1) == 0)
    {
        *hclk = 1;
        return KA2000_OSC_CLOCK;
    }
    val = ka2000_readl(0xa000000c);

    //PLLDIVK [15:14] R/W Output divider
    //2・b00 : divided by 1
    //2・b01 : divided by 2
    //2・b10 : divided by 4
    //2・b11 : divided by 8
    div_k = (val >> 14) & 0x3;

    //PLLDIVN [13:8] R/W Feedback divider, the divisor range will be 1~63
    div_n = (val >> 8) & 0x1f;

    //PLLDIVM [2] R/W Pre-driver
    //1・b0 : divided by 1
    //1・b1 : divided by 2
    div_m = (val >> 2) & 0x1;

    /* get mhz */
    mhz = (KA2000_OSC_CLOCK * (div_n + 1)) / (1 << div_k);
    //*hclk = ((ka2000_readl(0xa0000000) >> 1) & 0x7) + 1;
    *hclk = (((ka2000_readl(0xa0000000) >> 1) & 0x7) + 1);//  * v[ka2000_readl(0xa0000004) & 0x1f];
  //  printk("PLL %d, div m %d, n %d, k %d\n", mhz, div_m, div_n, div_k);

    return mhz;
}

void update_clocks(u32 pll_clock, u32 hclk);

void set_clock_set(int set)
{
    u32 val, reg0, reg8, pll_clock, hclk;
    int t = 1000000;
    int p0, p1, div, mul;
    
   // if (cur_set == set) //for Dynamic colck setting
	//	return;
	
    reg0 = word_read(0xa0000000); // 11c33303
    reg8 = word_read(0xa0000008);// 40404
   // printk("%x\n",reg0);
//	printk("%x\n",reg8);
    /* set all clock from PLL to OSC */ 
    word_write(0xa0000000, OSC_R0 & (~(SDIO_OSC_CLOCK_ONLY << 8))); //clock_regs[set][4] & ~(0xff));
    
    p0 = clock_regs[cur_set][1]; // 24
    p1 = clock_regs[set][1]; // 192 
//    printk("p0 %d p1 %d\n",p0,p1);
    if (p1 > p0)
    {
        word_write(0xa0000008, (reg8 & (~0xffffff)) | 0x00101010);    
    }
    val = clock_regs[set][7]; // 0x00009f00
//    printk("%x\n",val); //9f00
    //printf("Changing OSC... \n");
    null_delay(200);
    /* Setup PLL freq change */  
//	printk("%x\n",3 | val);// 9f03
//	printk("%x\n",1| val);// 9f01
    word_write(0xa000000c, 3 | val); //val =  0x00009f03
    word_write(0xa000000c, 1 | val); // val =  0x00009f01

    null_delay(1000);
    
    /* doing PLL change */
    word_write(0xa000000c, val);  // 0x00009f00
    
    /* wait PLL init finish */
    while((word_read(0xa0000000) & PLL_INIT_FINISH) != PLL_INIT_FINISH)
    {
        null_delay(2);
        if (t-- <= 0)
            break;
    };      
    
    word_write(0xa0000004, clock_regs[set][5]);  //   0x00000001
    word_write(0xa0000008, (reg8 & (~0xffffff)) | clock_regs[set][6]);// DIV_PLL_192 0x00101010
 //    printk("%x\n",(reg8 & (~0xffffff)));// 0
//	 printk("%x \n ",((reg8 & (~0xffffff)) | clock_regs[set][6]));//101010
    //ka2000_set_sdio_clock(24000000);
    
    /* set UART */
    word_write(0xa000400c, 0x83);
    word_write(0xa0004004, 0x00);
    word_write(0xa0004000, clock_regs[set][8]);
    word_write(0xa000400c, 0x03);

    /* set reg 0 */
    word_write(0xa0000000, clock_regs[set][4] & (~(SDIO_OSC_CLOCK_ONLY << 8)) ); // 0x11c22601
        //    printk("%x\n",~(SDIO_OSC_CLOCK_ONLY << 8));  //fffffeff      
       //     printk("%x\n",clock_regs[set][4] & (~(SDIO_OSC_CLOCK_ONLY << 8)) );
	//		printk("%x\n",clock_regs[set][4] );
    pll_clock = clock_regs[set][1] * 1000000;
    armrate   = clock_regs[set][2] * 1000000;
    hclk = clock_regs[set][3];
    cur_set = set;  
    update_clocks(pll_clock, hclk);

    null_delay(10);
     
    //printk("PLL %d, ARM %d, HCLK %d\n", pll_clock/1000000, armrate/1000000, hclk);
    printk("(%d-%d-%d)", pll_clock/1000000, armrate/1000000, hclk);
}


unsigned int clk_div = 0;
unsigned int clk_val;

void update_clocks(u32 pll_clock, u32 hclk)
{
	u8 v[] = {1, 2, 3, 3, 4, 4, 4, 4, 6, 6, 6, 6, 6, 6, 6, 6, 8};

    commonrate = pll_clock;
    hclk = (((ka2000_readl(0xa0000000) >> 1) & 0x7) + 1);
    if ((word_read(0xa0000004) & 0x1f))
        armrate    = pll_clock/(v[word_read(0xa0000004) & 0x1f]);
    else
        armrate    = pll_clock;

    /* SPI also have to set SSI_PRE */
    //spirate    = pll_clock/8;
    
    uartrate   = armrate/hclk;
    pclkrate   = armrate/hclk;
    scaler0rate= pll_clock;
    scaler1rate= pll_clock;
    pwmrate = armrate/hclk; //pll_clock/hclk;
    
    /* SPI also have to set SSI_PRE */
    spirate 	= pclkrate; 		// SPI clock is PCLK
   // printk("pll_clock %d, ARM %d, HCLK ratio %d\n", pll_clock, armrate, hclk);
}

void set_osc_clock(void)
{
    /* switch to osc */
    word_write(0xa0000000, 0);

    null_delay(100);
    /* turn off PLL */
    word_write(0xa000000c, 3);

    word_write(0xa0000000, 0x10c26600);
    //pll_clock = ka2000_get_pll_clock();
    update_clocks(KA2000_OSC_CLOCK, 1);

    /* set 12MHz baud rate 38400 */
    word_write(0xa000400c, 0x83);
    word_write(0xa0004004, 0x00);
    word_write(0xa0004000, 0x27);  //0xa1
    word_write(0xa000400c, 0x03);
}


/* clock and PLL init at u-boot, kernel don't to change PLL, but set clock rate for drvier to get clock rate */
int __init ka2000_clk_init(void)
{
    struct clk *clkp;
    int count = 0;
    u32 pll_clock, hclk;

	/*word_write(0xa0000018, 0xff);
	barrier();
	if (word_read(0xa0000018) == 0xff)
		chip_version = 'C'; */

    //printk("SCU_PLL_FREQ_SEL1 %08x\n", ka2000_readl(SCU_PLL_FREQ_SEL1));
    //printk("SCU_PLL_FREQ_SEL2 %08x\n", ka2000_readl(SCU_PLL_FREQ_SEL2));
    //printk("SCU_CLK_SRC_CTL   %08x\n", ka2000_readl(SCU_CLK_SRC_CTL));

    // don't set here, set at u-boot
    //Enter Slow Mode: PLL 0MHz, ARM 12MHz, HCLK 12MHz, PCLK 12MHz, SD Card 12MHz, SDRAM 12MHz
    //word_write(SCU_PLL_FREQ_SEL1, 0x00000000);
    //word_write(SCU_PLL_FREQ_SEL2, 0x00000000);
    //word_write(SCU_CLK_SRC_CTL,   0x01c22200); //sel OSC, arm ratio 0, arm clk=hclk=12MHz, en clk:sdio,sdhost,sdsw, sel SPI, en spi idle
#if KA2000_MHZ > OSC_MHZ  //12
    set_clock_set(CLK_DEFAULT);
    
    pll_clock = ka2000_get_pll_clock(&hclk);

    //printk("PLL %d, ARM %d\n", pll_clock, armrate);
#else
	//set_osc_clock();
#endif

    //printk("set sdio clock \n");
    ka2000_set_sdio_clock(12000000);
#if 0 //et: disable temporarily to let u-boot control the clock speed
    ka2000_set_sdc_clock(12000000);
    ka2000_set_spi_clock(48000000);
#endif

    //for (clkp = ka2000_clks; count < ARRAY_SIZE(ka2000_clks);
    clk_register(&clk_p);
    clk_enable(&clk_p);

    clk_register(&clk_timer_scaler[0]);
    clk_enable(&clk_timer_scaler[0]);

    clk_register(&clk_timer_scaler[1]);
    clk_enable(&clk_timer_scaler[1]);
	clk_register(&clk_hsmmc[0]);
	clk_enable(&clk_hsmmc[0]);

	clk_register(&clk_hsmmc[1]);
	clk_enable(&clk_hsmmc[1]);

	clk_register(&clk_hsmmc[2]);
	clk_enable(&clk_hsmmc[2]);

    for (clkp = ka2000_clks; count < 7; count++, clkp++)
    {
        clk_register(clkp);

        /* Turn on clocks that have been enabled in the
         * table above */
        clk_enable(clkp);
    }                    
    return 0;
}

extern void arch_idle();
#if CONFIG_KA2000_CHIP_VERSION == 0xA
#define SLOW_HCLK 1
#define SLOW_CLKVAL 0x10c26602
#define SLOW_UART 0x13
#else
#define SLOW_HCLK 0
#define SLOW_CLKVAL 0x10c26600
#define SLOW_UART 0x26
#endif
void ka2000_clk_sleep(void)
{
    //printk("ka2000_clk_sleep \n");
    printk("SLP: %08x %08x %08x %08x\n", ka2000_readl(0xa0000000),ka2000_readl(0xa0000004),ka2000_readl(0xa0000008),ka2000_readl(0xa000000c));

    if( (word_read(0xa0000000) & SLOW_CLKVAL) != SLOW_CLKVAL){
        printk("ka sleep clk\n");

        clk_val = word_read(0xa0000000);
        clk_div = word_read(0xa000000c) & 0xfffffffc;
        null_delay(100);
        
        set_osc_clock();
	}

    update_clocks(KA2000_OSC_CLOCK, SLOW_HCLK+1);

    if(slow_mode)
        return;

    //null_delay(200);
	if ((word_read(0xa000a088) & 0x01) == 0x01)
	    word_write(0xa000a088, 1); /* clear bit 0 */
	flush_cache_all();
	null_delay(200);
	pwm_set(0);

#if  CONFIG_KA2000_CHIP_VERSION == 0xA
		enable_irq(IRQ_switch_int6);
#endif

    //__asm__ __volatile__ ("mov r0, #0x0");
    //__asm__ __volatile__ ("MCR p15, 0, r0,c7,c0,4" ::"r"(0));
    if (word_read(0xa000a088) == 0)
		arch_idle();

}



void ka2000_clk_wakeup(void)
{
#if 0

#if  DELAY_CHANGE_CLOCK
    init_clock_timer();
    //printk("ka2000_clk_wakeup %d\n", slow_mode);   
    if (slow_mode)
        ka2000_clk_change_mode(1);
    else
        ka2000_clk_change_mode(0);    
#else
	local_irq_disable();
	preempt_disable();	

	if (slow_mode)
		set_clock_set(CLK_24_24_24);
	else
		set_clock_set(CLK_DEFAULT);  

	preempt_enable();
	local_irq_enable();    
#endif
#endif

}


void ka2000_enable_sleep(void)
{
#ifdef CONFIG_KA2000_POWER_SLEEP_ENABLE
	go_sleep_enabled = 1;
#endif
}

void ka2000_disable_sleep(void)
{
#ifdef CONFIG_KA2000_POWER_SLEEP_ENABLE
	go_sleep_enabled = 0;
#endif
}

void ka2000_sdio_data_transfer(void)
{
	if (slow_mode)
	{
		//UARTC('T');
		slow_mode = 0;
		ka2000_clk_change_mode(0);
	}
}


void ka2000_sdio_on(void)
{
	sdio_wakeup = 1;

	printk("sdio wakeup");
#ifdef CFG_SYS_LP8
	if(detect_cmd)
#endif
        enable_irq(IRQ_switch_int0);
}

void ka2000_sdio_off(void)
{
	sdio_wakeup = 0;
}


EXPORT_SYMBOL(ka2000_clk_sleep);
EXPORT_SYMBOL(ka2000_clk_wakeup);
EXPORT_SYMBOL(ka2000_disable_sleep);
EXPORT_SYMBOL(ka2000_enable_sleep);
EXPORT_SYMBOL(ka2000_sdio_on);
EXPORT_SYMBOL(ka2000_sdio_off);
EXPORT_SYMBOL(ka2000_sdio_data_transfer);



#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static void *ka2000_ck_start(struct seq_file *m, loff_t *pos)
{
    return *pos < 1 ? (void *)1 : NULL;
}

static void *ka2000_ck_next(struct seq_file *m, void *v, loff_t *pos)
{
    ++*pos;
    return NULL;
}

static void ka2000_ck_stop(struct seq_file *m, void *v)
{
}

static int ka2000_ck_show(struct seq_file *m, void *v)
{
    struct clk *cp;

    list_for_each_entry(cp, &clocks, node)
    seq_printf(m,"%s %d %d\n", cp->name, *(cp->rate), cp->usecount);

    return 0;
}

static const struct seq_operations ka2000_ck_op =
{
    .start	= ka2000_ck_start,
    .next	= ka2000_ck_next,
    .stop	= ka2000_ck_stop,
    .show	= ka2000_ck_show
};

static int ka2000_ck_open(struct inode *inode, struct file *file)
{
    return seq_open(file, &ka2000_ck_op);
}

static const struct file_operations proc_ka2000_ck_operations =
{
    .open		= ka2000_ck_open,
    .read		= seq_read,
    .llseek		= seq_lseek,
    .release	= seq_release,
};

static int __init ka2000_ck_proc_init(void)
{
    proc_create("ka2000_clocks", 0, NULL, &proc_ka2000_ck_operations);
    return 0;
}

__initcall(ka2000_ck_proc_init);
#endif	/* CONFIG_DEBUG_PROC_FS */
