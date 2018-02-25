/*
 * KeyASIC Ka2000 timer subsystem
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
#include <linux/types.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/tick.h>
#include <linux/clk.h>
#include <mach/hardware.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/errno.h>
#include <mach/io.h>
#include <mach/ka2000.h>

#ifndef CONFIG_KA2000_PRINTK_ENABLE
#define printk dprintk
static inline int dprintk(const char *fmt, ...)
{
      return 0;
}
#endif

#define TIMER0_MAP_TIMER1 1

#if NR_CPUS > 1
ERR NR_CPUS > 1
#endif


#ifndef CONFIG_KA2000_12M_FPGA_TEST

    /* 192M mode for test faster timer */
#define M 1

#define PRESCALE_V (33*M)
#define INT_COUNT (100/M)
#define T1_MUX 0
#define T2_MUX 0

#define CLOCK_SOURCE_DIV (PRESCALE_V*2)
#define CLOCK_EVENT_DIV  (PRESCALE_V*INT_COUNT)

#define TIMER_PERIOD   (0x1d4c) 
#define TIMER_PERIOD_2 0xffff

#else

/* 12M or 24M mode */
#define PRESCALE_V (33)
#define INT_COUNT (100)
#define T1_MUX 0
#define T2_MUX 0
#define CLOCK_SOURCE_DIV (PRESCALE_V)
#define CLOCK_EVENT_DIV  (PRESCALE_V*50)

#define TIMER_PERIOD   (0x1d4c)
#define TIMER_PERIOD_2 0xffff
#endif

#define KA2000_TIMER0_BASE 	(IO_PHYS + 0x2000)  //in pwm timer
#define NUM_TIMERS          	2
#define KA2000_WDOG_BASE   	(IO_PHYS + 0x3000)

#ifdef KA2000_DISABLE_PRINTK
#define DBG(...); {}
#else
#define DBG printk
#endif

u32 ka_tick = 0;
u32 ka_tick1 = 0;

static int timer_irqs[NUM_TIMERS] =
{
    IRQ_PWM_T1,
    IRQ_PWM_T2,
};

/*
 * This driver configures the 2 64-bit count-up timers as 2 independent
 * 32-bit count-up timers used as follows:
 *
 * Timer 0:  clockevent source for buzzer
 * Timer 1:  clocksource for generic timekeeping
 */
#define TID_CLOCKEVENT      0
#define TID_CLOCKSOURCE 	1

/* Timer register offsets */
#define TCFG0                        0x00
#define TCFG1                        0x04
#define TCON0                        0x08
#define TCON1                        0x0c
#define TCNTB0                       0x10
#define TCMPB0			             0x14
#define TCNTO0                       0x18
#define TCNTB1                       0x40
#define TCNTO1                       0x44
#define TCNTB2                       0x48
#define TCNTO2                       0x4c

/* Timer register bitfields */
#define TCR_ENAMODE_DISABLE          0x0
#define TCR_ENAMODE_ONESHOT          0x0
#define TCR_ENAMODE_PERIODIC         0x1

#define TCR_START_TIMER0	     (1 << 0)
#define TCR_START_TIMER1	     (1 << 20)

#define TCR_RELOAD_TIMER0	     (1 << 3)
#define TCR_RELOAD_TIMER1	     (1 << 22)

#define TCR_START_TIMER2	     (1 << 0)
#define TCR_RELOAD_TIMER2	     (1 << 2)
/* values for 'opts' field of struct timer_s */
#define TIMER_OPTS_DISABLED   0x00
#define TIMER_OPTS_ONESHOT    0x01
#define TIMER_OPTS_PERIODIC   0x02

#define INTC_INTMSK1_ADDR 0xa0006010
#define TIMER_INTR		(0x1 << 22)
#define TIMER_TRIGGER_DMA	(0x1 << 24)

#define GPIO_OUTPUT 			 0xa0005008
#define INTC_INTPND1_ADDR		 0xa0006018

#define PRESCALER_MASK_T     (0xff << 16)
#define PRESCALER_VALUE_T(v)    ((v) << 16)
#define TRIGGER_MODE_MASK_T  (1 << 24)	//0x01000000
#define MUX0_MASK            (3 << 8)

#define TCR_START_TIMER TCR_START_TIMER1
#define TCR_RELOAD_TIMER TCR_RELOAD_TIMER1

struct timer_s
{
    char *name;
    unsigned int id;
    unsigned long period;
    unsigned long opts;
    unsigned long reg_base;
    unsigned long tim_reg;
    unsigned long prd_reg;
    struct irqaction irqaction;
};
static struct timer_s timers[];
spinlock_t		timer_lock;


static struct clock_event_device clockevent_ka2000;
unsigned int get_ka_tick(void)
{
    int t = (TIMER_PERIOD - (ka2000_readl(0xa0002000 + TCNTO1) & 0x1fff));

	return ka_tick + t;
}
EXPORT_SYMBOL(get_ka_tick);

static unsigned  int update_ka_tick(void)
{
    int t = (TIMER_PERIOD - (ka2000_readl(0xa0002000 + TCNTO1) & 0x1fff));

	return ka_tick + t;
}
#if 0
/* only for test purpose, to make sure ka_kick with correct increasment */
void check_ka_tick(void)
{
    int i, t0, t1;
    int t[1000];

    t0 = update_ka_tick();
    printk("check ka_tick\n");
    for (i = 0; i < 1000; i++)
    {
        t1 = update_ka_tick();
        //printk("%4d,", t1 - t0);
        null_delay(100);
        t[i] = t1 - t0;
        t0 = t1;
    }

    for (i = 0; i < 1000; i++)
        printk("%4d,", t[i]);
    printk("check ka_tick done\n");
}
#endif


#if TRACE_T_ENABLED
#define TRACE_T_COUNT 1024
#define TRACE_STR_LEN 20
static u32 trace_t_time[TRACE_T_COUNT];
static int trace_t_line[TRACE_T_COUNT];
static char trace_t_msg[TRACE_T_COUNT][TRACE_STR_LEN];
static int trace_t_i = 1;
static int trace_t_busy = 0;


void trace_t(char *msg, int line)
{
    int i, t;
    if (trace_t_busy)
        return;
    trace_t_busy = 1;

    trace_t_time[trace_t_i] = update_ka_tick(); //ka_tick + TIMER_PERIOD_2 - ka2000_readl(0xa0002000 + TCNTO2);
    memcpy(trace_t_msg[trace_t_i], msg, TRACE_STR_LEN);

    trace_t_msg[trace_t_i][TRACE_STR_LEN-1] = 0;
    trace_t_line[trace_t_i] = line;
    trace_t_i++;
    if (trace_t_i >= TRACE_T_COUNT)
    {
        trace_t_dump();
    }
    trace_t_busy = 0;
}

void trace_t_dump(void)
{
    int i = 0, j = 0;
    unsigned int t = 0;
    //printk("[%5d,%8s]\n",  0, trace_t_msg[0]);
    for (i = 1; i < trace_t_i; i++)
    {
        if (trace_t_time[i] >= trace_t_time[i-1])
        {
            t = trace_t_time[i] - trace_t_time[i-1];
        }
        else //if (trace_t_time[i] < 0xff && trace_t_time[i-1] > 0xf000)
        {
            //t = (unsigned int)0xffffffff - trace_t_time[i-1];

            //printk("%x,", trace_t_time[i-1]);
            //t = (0x10000 - (trace_t_time[i-1] & 0xffff)) + trace_t_time[i];
            t = trace_t_time[i];
        }
        //else
        //{
        //    printk("[t0 %d, t1 %d]", trace_t_time[i], trace_t_time[i-1]);
        //    t = trace_t_time[i-1] - trace_t_time[i];
        //}
        if (trace_t_msg[i][0] != 0)
            printk("%5d:%5d:%s\n", t, trace_t_line[i], trace_t_msg[i]);
        if (j > 10)
        {
           // schedule_timeout(10);
            j = 0;
        }
        j++;

    }
    trace_t_i = 1;
    trace_t_time[0] = update_ka_tick();
}

//EXPORT_SYMBOL(trace_t_reset);
EXPORT_SYMBOL(trace_t_dump);
EXPORT_SYMBOL(trace_t);
#endif


//--------------------------------------------------------------------------------------------------------
static int timer32_config(struct timer_s *t)
{
    u32 tcr=0, tcfg0=0, tcfg1=0;

    //ka2000_dbg("timer32_config %d\n", t->id);

    tcfg0 = ka2000_readl(t->reg_base + TCFG0);
    tcfg1 = ka2000_readl(t->reg_base + TCFG1);

    switch(t->id)
    {
    case 0:
        tcr = ~(TCR_START_TIMER1) & ka2000_readl(t->reg_base + TCON0);
        ka2000_writel(tcr, t->reg_base + TCON0);			//disable timer
        ka2000_writel(t->period, t->prd_reg);					//reset counter to 0 (TCNTB0)

        /* set prescaler of T1 and T2 */
        ka2000_writel((tcfg0 & ~PRESCALER_MASK_T) | PRESCALER_VALUE_T(PRESCALE_V), t->reg_base + TCFG0);   	//setup prescaler (128) or T1 and T2
        //ka2000_dbg("TCFG0 %08x -> %08x\n", tcfg0, (tcfg0 & ~PRESCALER_MASK_T) | PRESCALER_VALUE_T);

        /* set trigger output mode and MUX */
        ka2000_writel((ka2000_readl(0xa0002000+TCFG1) | (T1_MUX << 8) | (T2_MUX << 10) | TRIGGER_MODE_MASK_T), 0xa0002000 + TCFG1);  //timer 1 dma mode

        nop();nop();nop();nop();nop();nop();mb();

        ka2000_writel(ka2000_readl(0xa0002000 + TCFG1) & ~(TRIGGER_MODE_MASK_T), 0xa0002000 + TCFG1);  //timer 1 interrupt mode

        tcr = ka2000_readl(t->reg_base + TCON0);
        //ka2000_dbg("TCON0 %08x -> %08x\n", tcr, tcr | TCR_START_TIMER | TCR_RELOAD_TIMER);
        tcr |= TCR_START_TIMER1 | TCR_RELOAD_TIMER1;
        ka2000_writel(tcr, t->reg_base + TCON0);
        break;

    case 1:
        tcr = ~(TCR_START_TIMER2) & ka2000_readl(t->reg_base + TCON1);
        ka2000_writel(tcr, t->reg_base + TCON1);
        ka2000_writel(t->period, t->prd_reg);

        /* set prescaler of T1 and T2 */
        ka2000_writel((tcfg0 & ~PRESCALER_MASK_T) | PRESCALER_VALUE_T(PRESCALE_V), t->reg_base + TCFG0);   	//setup prescaler (128) or T1 and T2

        /* set trigger output mode and MUX */
        ka2000_writel((ka2000_readl(0xa0002000+TCFG1) | (T1_MUX << 8) | (T2_MUX << 10) | 0x2000000), 0xa0002000 + TCFG1);  //timer 2 dma mode

        //nop();nop();nop();nop();nop();nop();mb();

        //ka2000_writel(ka2000_readl(0xa0002000 + TCFG1) & ~(0x2000000), 0xa0002000 + TCFG1);  //timer 2 interrupt mode

        //ka2000_writel((tcfg1 & 0xfdfff3ff) | 0x02000000, t->reg_base + TCFG1);	//setup clk input & mode (def clk, dma)
        //tcr = ka2000_readl(t->reg_base + TCON1) | TCR_START_TIMER2 | TCR_RELOAD_TIMER2;
        //ka2000_writel(tcr, t->reg_base + TCON1);
        break;

    default:
        break;
    }
    return 0;
}
//--------------------------------------------------------------------------------------------------------

//static u32 t1_tick = 0;
#if 1//CONFIG_KA2000_CHIP_VERSION == 0xA

int kcard_call_back_init = 0;
kcard_call_back_t kcard_call_back;

void kcard_set_call_back(kcard_call_back_t handler)
{
	kcard_call_back = handler;
	kcard_call_back_init = 1;
}

EXPORT_SYMBOL(kcard_set_call_back);
#endif

//--------------------------------------------------------------------------------------------------------
static irqreturn_t timer_interrupt(int irq, void *dev_id)
{
    unsigned int tcfg1;

//#ifdef CONFIG_KA2000_WITH_RESISTER		
#if 0//CONFIG_KA2000_CHIP_VERSION == 0xA

	if (kcard_call_back_init)
		kcard_call_back();
#endif
//#endif
    clockevent_ka2000.event_handler(&clockevent_ka2000);

	/* timer 1 dma mode */
    tcfg1 = ka2000_readl(0xa0002000 + TCFG1) | TRIGGER_MODE_MASK_T;
    ka2000_writel(tcfg1, 0xa0002000 + TCFG1);	//timer 1 dma mode
	
    nop();  nop();  nop();  nop();
    ka_tick += TIMER_PERIOD;
    /* timer 1 interrupt mode */
    tcfg1 = ka2000_readl(0xa0002000 + TCFG1) & ~(TRIGGER_MODE_MASK_T);
    ka2000_writel(tcfg1, 0xa0002000 + TCFG1);	//timer 1 interrupt mode

    return IRQ_HANDLED;
}
//--------------------------------------------------------------------------------------------------------
/* called when 32-bit counter wraps */
//#define T2_INT

#ifdef T2_INT
static irqreturn_t freerun_interrupt_T2(int irq, void *dev_id)
{
    unsigned int tcfg1;

    tcfg1 = ka2000_readl(0xa0002000+TCFG1);
    ka2000_writel((tcfg1 | 0x2000000), 0xa0002000 + TCFG1);  //timer 2 dma mode
	/* add delay */
    nop();  nop();  nop();//mb();
    tcfg1 = ka2000_readl(0xa0002000+TCFG1) & ~(0x2000000);
    ka2000_writel(tcfg1, 0xa0002000 + TCFG1);  //timer 2 interrupt mode

    return IRQ_HANDLED;
}
#endif
//--------------------------------------------------------------------------------------------------------
static struct timer_s timers[] =
{
    [TID_CLOCKEVENT] = {

        .name      = "clockevent PWM T1",
        .opts      = TIMER_OPTS_PERIODIC, //TIMER_OPTS_DISABLED,
        .period    = TIMER_PERIOD, //0x1fff,
        .irqaction = {
            .flags   = IRQF_DISABLED | IRQF_TIMER, // | IRQF_IRQPOLL,
            .handler = timer_interrupt,
        }
    },
    [TID_CLOCKSOURCE] = {
        .name       = "free-run counter PWM T2",
        .period     = TIMER_PERIOD_2,
#ifndef T2_INT
        .opts       = TIMER_OPTS_ONESHOT,
#else
        .opts       = TIMER_OPTS_PERIODIC,
        .irqaction = {
            .flags   = IRQF_DISABLED | IRQF_TIMER,
            .handler = freerun_interrupt_T2,
        }
#endif
    },

};
//--------------------------------------------------------------------------------------------------------
static void __init timer_init(void)
{
    u32 bases[] = {KA2000_TIMER0_BASE};
    int i;

    /* Init of each timer as a 32-bit timer */
    for (i=0; i< ARRAY_SIZE(timers); i++)
    {
        struct timer_s *t = &timers[i];
        if (t->name)
        {
            t->id = i;
            t->reg_base = KA2000_TIMER0_BASE;
            if(t->id == 0)
            {
                t->tim_reg = t->reg_base + TCNTO1;
                t->prd_reg = t->reg_base + TCNTB1;
            }
            else if(t->id == 1)
            {
                t->tim_reg = t->reg_base + TCNTO2;
                t->prd_reg = t->reg_base + TCNTB2;
            }

            /* Register interrupt */
            //if(t->id == 0)
            if (t->irqaction.handler != NULL)
            {
                t->irqaction.name = t->name;
                t->irqaction.dev_id = (void *)t;
                if (t->irqaction.handler != NULL)
                {
                    setup_irq(timer_irqs[t->id], &t->irqaction);
                    DBG("time setup irq %s\n",t->name);
                    DBG("time setup irq %d\n",timer_irqs[t->id]);
                }
            }
            timer32_config(&timers[i]);
            DBG("time32_config %d done\n",i);
        }
    }
}
//--------------------------------------------------------------------------------------------------------
/*
 * clocksource
 */
static cycle_t read_cycles(void)
{
    return ka_tick;
}
//--------------------------------------------------------------------------------------------------------
static struct clocksource clocksource_ka2000 =
{
    .name		= "timer0_1",
    .rating		= HZ, //200
    .read		= read_cycles,
    .mask		= CLOCKSOURCE_MASK(16),
    .shift		= 24,
    .flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};
//--------------------------------------------------------------------------------------------------------
/*
 * clockevent
 */
static int ka2000_set_next_event(unsigned long cycles, struct clock_event_device *evt)
{
    struct timer_s *t = &timers[TID_CLOCKEVENT];

    t->period = cycles;
    timer32_config(t);
    return 0;
}
//--------------------------------------------------------------------------------------------------------
static void ka2000_set_mode(enum clock_event_mode mode, struct clock_event_device *evt)
{
    struct timer_s *t = &timers[TID_CLOCKEVENT];

    DBG("ka2000_set_mode %d\n", mode);

    switch (mode)
    {
    case CLOCK_EVT_MODE_PERIODIC:
        t->period = TIMER_PERIOD; //0x1f5a;
        t->opts = TIMER_OPTS_PERIODIC;
        timer32_config(t);

        break;
    case CLOCK_EVT_MODE_ONESHOT:
        t->opts = TIMER_OPTS_ONESHOT;
        break;
    case CLOCK_EVT_MODE_UNUSED:
    case CLOCK_EVT_MODE_SHUTDOWN:
        t->opts = TIMER_OPTS_DISABLED;
        break;
    case CLOCK_EVT_MODE_RESUME:
        break;
    }
}
//--------------------------------------------------------------------------------------------------------
static struct clock_event_device clockevent_ka2000 =
{
    .name		= "timer0_0",
    .features   = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
    .shift		= 32,
    .set_next_event	= ka2000_set_next_event,
    .set_mode	    = ka2000_set_mode,
};
//--------------------------------------------------------------------------------------------------------
void clocks_calc_mult_shift(u32 *mult, u32 *shift, u32 from, u32 to, u32 minsec)
{
        u64 tmp;
        u32 sft, sftacc= 32;

        /*
         * Calculate the shift factor which is limiting the conversion
         * range:
         */
        tmp = ((u64)minsec * from) >> 32;
        while (tmp) {
                tmp >>=1;
                sftacc--;
        }

        /*
         * Find the conversion shift/mult pair which has the best
         * accuracy and fits the maxsec conversion range:
         */
        for (sft = 32; sft > 0; sft--) {
                tmp = (u64) to << sft;
                do_div(tmp, from);
                if ((tmp >> sftacc) == 0)
                        break;
        }
        *mult = tmp;
        *shift = sft;
}
//--------------------------------------------------------------------------------------------------------
static void __init ka2000_timer_init(void)
{
    u32 mult, shift;
    static char err[] __initdata = KERN_ERR
                                   "%s: can't register clocksource!\n";
	struct clk *clk;
	unsigned long rate = KA2000_OSC_CLOCK;

    /* clock source */
	clk = clk_get(NULL, "pclk");

    /* get pclk rate */
	rate = clk_get_rate(clk);
    printk("pclk rate %d\n", rate);

    //clockevent_ka2000.period = TIMER_PERIOD * (pclk / KA2000_OSC_CLOCK);

    /* init timer hw */
    timer_init();

    /* setup clocksource */
    //clockevent_ka2000.mult = clocksource_khz2mult(rate/128,  clocksource_ka2000.shift);
    //clocks_calc_mult_shift(&mult, &shift, CLOCK_TICK_RATE*128, NSEC_PER_SEC, 5);
    clocks_calc_mult_shift(&mult, &shift, rate/CLOCK_SOURCE_DIV, NSEC_PER_SEC, 5);
    clocksource_ka2000.mult = mult;
    clocksource_ka2000.shift = shift;
    if (clocksource_register(&clocksource_ka2000))
        printk(err, clocksource_ka2000.name);

    DBG("clocksource mult %d shift %d\n", clocksource_ka2000.mult, clocksource_ka2000.shift);

    /* setup clockevent */
    //clockevent_ka2000.mult = div_sc(rate/128, NSEC_PER_SEC,
    //			 clockevent_ka2000.shift);
    clocks_calc_mult_shift(&mult, &shift, NSEC_PER_SEC, rate/CLOCK_EVENT_DIV, 5);
    clockevent_ka2000.mult = mult;
    clockevent_ka2000.shift = shift;

    DBG("clocksevent mult %d shift %d\n", clockevent_ka2000.mult, clockevent_ka2000.shift);

    clockevent_ka2000.max_delta_ns =
        clockevent_delta2ns(0xffffffff, &clockevent_ka2000);
    clockevent_ka2000.min_delta_ns =
        clockevent_delta2ns(1, &clockevent_ka2000);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
    clockevent_ka2000.cpumask = cpumask_of(0);
#else
    clockevent_ka2000.cpumask = cpumask_of_cpu(0);
#endif

    clockevents_register_device(&clockevent_ka2000);

}
//--------------------------------------------------------------------------------------------------------
struct sys_timer ka2000_timer =
{
    .init   = ka2000_timer_init,
};

//--------------------------------------------------------------------------------------------------------
/* reset board using watchdog timer */
void ka2000_watchdog_reset(void)
{
}

//--------------------------------------------------------------------------------------------------------
