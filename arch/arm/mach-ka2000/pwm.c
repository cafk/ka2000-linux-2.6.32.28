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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <mach/clock.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <mach/irqs.h>
#include <mach/ka2000.h>

//=========================================================================
// Address
//=========================================================================
#define TCFG0					0xA0002000 + 0x55000000
#define TCFG1					0xA0002004 + 0x55000000
#define TCON0					0xA0002008 + 0x55000000
#define TCNTB0					0xA0002010 + 0x55000000
#define TCMPB0					0xA0002014 + 0x55000000
#define TCNTO0					0xA0002018 + 0x55000000
//=========================================================================
// Register
//=========================================================================
#define TOUT0   				(0x1 << 2)
#define TOUT1   				(0x1 << 20)
#define TCFG1_MUX0_DIV16  		        (0x3 << 0)
#define TCFG1_MUX0_MASK     	                (0x3 << 0)
#define TCFG_PRESCALER0_MASK 	                (0xFF << 0)
#define MODE_TRIGGER_DMA	                (0x1 << 20)
#define KA2000_CLOCK_FREQ                       KA2000_PLL_CLOCK
//=========================================================================

#define PWM_TIMER_ENABLED

int pwm_start = 0;
int pwm_stopped = 0;
extern struct clk *clk_get(struct device *dev, const char *id);
extern unsigned long clk_get_rate(struct clk *clk);
extern void clk_put(struct clk *clk);
extern int clk_set_rate(struct clk *clk, unsigned long rate);

static struct timer_list	pwm_timer;


int queue_tone[20];
int index;
int total_rings=0;
struct clk *clk_get_parent(struct clk *clk)
{
    return ERR_PTR(-ENOSYS);
}

struct pwm_device
{
    struct list_head	 list;
    struct platform_device	*pdev;

    struct clk		*clk_div;
    struct clk		*clk;
    const char		*label;

    unsigned int		 period_ns;
    unsigned int		 duty_ns;

    unsigned char		 tcon_base;
    unsigned char		 running;
    unsigned char		 use_count;
    unsigned char		 pwm_id;
    int                      irq;
    unsigned long            pclk;
};

struct pwm_arg
{
    unsigned short          frequency;      //[15:0]
    unsigned char           volume;         //[23:16]
    unsigned char           duration;       //[31:24]
};

#define pwm_dbg(_pwm, msg...) dev_dbg(&(_pwm)->pdev->dev, msg)

static struct clk *clk_scaler[2];
int pwm_interval;
int pwm_run=0;
void pwm_set(int cmd);
void pwm_stop(void);
static u32 pwm_value = 0;
/* Standard setup for a timer block. */

#define TIMER_RESOURCE_SIZE (1)

#define TIMER_RESOURCE(_tmr, _irq)			\
	(struct resource [TIMER_RESOURCE_SIZE]) {	\
		[0] = {					\
			.start	= _irq,			\
			.end	= _irq,			\
			.flags	= IORESOURCE_IRQ	\
		}					\
	}

#define DEFINE_KeyAsic_TIMER(_tmr_no, _irq)			\
	.name		= "KeyAsic-pwm",		\
	.id		= _tmr_no,			\
	.num_resources	= TIMER_RESOURCE_SIZE,		\
	.resource	= TIMER_RESOURCE(_tmr_no, _irq),	\

/* since we already have an static mapping for the timer, we do not
 * bother setting any IO resource for the base.
 */

#if 1
struct platform_device ka2000pwm_device[] =
{
    [0] = { DEFINE_KeyAsic_TIMER(0, IRQ_PWM0) },
    /*	[1] = { DEFINE_KeyAsic_TIMER(1, IRQ_PWM0) },
    	[2] = { DEFINE_KeyAsic_TIMER(2, IRQ_PWM0) },
    	[3] = { DEFINE_KeyAsic_TIMER(3, IRQ_PWM0) },
    	[4] = { DEFINE_KeyAsic_TIMER(4, IRQ_PWM0) },
    */
};
#endif

static inline int pwm_is_tdiv(struct pwm_device *pwm)
{
    return clk_get_parent(pwm->clk) == pwm->clk_div;
}

static DEFINE_MUTEX(pwm_lock);
static LIST_HEAD(pwm_list);

struct pwm_device *pwm_request(int pwm_id, const char *label)
{
    struct pwm_device *pwm;
    int found = 0;

    mutex_lock(&pwm_lock);

    list_for_each_entry(pwm, &pwm_list, list)
    {
        if (pwm->pwm_id == pwm_id)
        {
            found = 1;
            break;
        }
    }

    if (found)
    {
        if (pwm->use_count == 0)
        {
            pwm->use_count = 1;
            pwm->label = label;
        }
        else
            pwm = ERR_PTR(-EBUSY);
    }
    else
        pwm = ERR_PTR(-ENOENT);

    mutex_unlock(&pwm_lock);
    return pwm;
}

EXPORT_SYMBOL(pwm_request);


void pwm_free(struct pwm_device *pwm)
{
    mutex_lock(&pwm_lock);

    if (pwm->use_count)
    {
        pwm->use_count--;
        pwm->label = NULL;
    }
    else
        printk(KERN_ERR "PWM%d device already freed\n", pwm->pwm_id);

    mutex_unlock(&pwm_lock);
}

EXPORT_SYMBOL(pwm_free);

#define pwm_tcon_start(pwm) (1 << (pwm->tcon_base + 0))
#define pwm_tcon_invert(pwm) (1 << (pwm->tcon_base + 2))
#define pwm_tcon_autoreload(pwm) (1 << (pwm->tcon_base + 3))
#define pwm_tcon_manulupdate(pwm) (1 << (pwm->tcon_base + 1))

int pwm_enable(struct pwm_device *pwm)
{
    unsigned long flags;
    unsigned long tcon;

    local_irq_save(flags);

    tcon = __raw_readl(TCON0);
    tcon |= pwm_tcon_start(pwm);
    __raw_writel(tcon, TCON0);

    local_irq_restore(flags);

    pwm->running = 1;
    return 0;
}

EXPORT_SYMBOL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
    unsigned long flags;
    unsigned long tcon;

    local_irq_save(flags);

    tcon = __raw_readl(TCON0);
    tcon &= ~pwm_tcon_start(pwm);
    __raw_writel(tcon, TCON0);

    local_irq_restore(flags);

    pwm->running = 0;
}

EXPORT_SYMBOL(pwm_disable);

static unsigned long pwm_calc_tin(struct pwm_device *pwm, unsigned long freq)
{
    unsigned long tin_parent_rate;
    unsigned int div;

    tin_parent_rate = clk_get_rate(clk_get_parent(pwm->clk_div));
    pwm_dbg(pwm, "tin parent at %lu\n", tin_parent_rate);

    for (div = 2; div <= 16; div *= 2)
    {
        if ((tin_parent_rate / (div << 16)) < freq)
            return tin_parent_rate / div;
    }

    return tin_parent_rate / 16;
}

#define NS_IN_HZ (1000000000UL)

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
    unsigned long tin_rate;
    unsigned long tin_ns;
    unsigned long period;
    unsigned long flags;
    unsigned long tcon;
    unsigned long tcnt;
    long tcmp;

    /* We currently avoid using 64bit arithmetic by using the
     * fact that anything faster than 1Hz is easily representable
     * by 32bits. */

    if (period_ns > NS_IN_HZ || duty_ns > NS_IN_HZ)
        return -ERANGE;

    if (duty_ns > period_ns)
        return -EINVAL;

    if (period_ns == pwm->period_ns &&
            duty_ns == pwm->duty_ns)
        return 0;

    /* The TCMP and TCNT can be read without a lock, they're not
     * shared between the timers. */

    /* KeyAsic only TCMPB0 TCNTB0*/
    //tcmp = __raw_readl(TCMPB(pwm->pwm_id));
    //tcnt = __raw_readl(TCNTB(pwm->pwm_id));
    tcmp = __raw_readl(TCMPB0);
    tcnt = __raw_readl(TCNTB0);

    period = NS_IN_HZ / period_ns;

    pwm_dbg(pwm, "duty_ns=%d, period_ns=%d (%lu)\n",
            duty_ns, period_ns, period);

    /* Check to see if we are changing the clock rate of the PWM */

    if (pwm->period_ns != period_ns)
    {
        if (pwm_is_tdiv(pwm))
        {
            tin_rate = pwm_calc_tin(pwm, period);
            clk_set_rate(pwm->clk_div, tin_rate);
        }
        else
            tin_rate = clk_get_rate(pwm->clk);

        pwm->period_ns = period_ns;

        pwm_dbg(pwm, "tin_rate=%lu\n", tin_rate);

        tin_ns = NS_IN_HZ / tin_rate;
        tcnt = period_ns / tin_ns;
    }
    else
        tin_ns = NS_IN_HZ / clk_get_rate(pwm->clk);

    /* Note, counters count down */

    tcmp = duty_ns / tin_ns;
    tcmp = tcnt - tcmp;

    pwm_dbg(pwm, "tin_ns=%lu, tcmp=%ld/%lu\n", tin_ns, tcmp, tcnt);

    if (tcmp < 0)
        tcmp = 0;

    /* Update the PWM register block. */

    local_irq_save(flags);

    /* KeyAsic only TCMPB0 TCNTB0 */
    //__raw_writel(tcmp, S3C2410_TCMPB(pwm->pwm_id));
    //__raw_writel(tcnt, S3C2410_TCNTB(pwm->pwm_id));
    __raw_writel(tcmp, TCMPB0);
    __raw_writel(tcnt, TCNTB0);

    tcon = __raw_readl(TCON0);
    tcon |= pwm_tcon_manulupdate(pwm);
    tcon |= pwm_tcon_autoreload(pwm);
    __raw_writel(tcon, TCON0);

    tcon &= ~pwm_tcon_manulupdate(pwm);
    __raw_writel(tcon, TCON0);

    local_irq_restore(flags);

    return 0;
}

EXPORT_SYMBOL(pwm_config);

static int pwm_register(struct pwm_device *pwm)
{
    pwm->duty_ns = -1;
    pwm->period_ns = -1;

    mutex_lock(&pwm_lock);
    list_add_tail(&pwm->list, &pwm_list);
    mutex_unlock(&pwm_lock);

    return 0;
}

void queue_reset()
{

    int i;
    for(i =0; i < 20; i++)
        queue_tone[i] = 0;

    index = 0;

}

void queue_construction(int cmd)
{
    int i;
#ifdef CONFIG_KA2000_PWM_ENABLE
    if (cmd == 16842753)
    {
        queue_tone[index] = cmd;
        total_rings=index;
        index = 0;
        i = queue_tone[index];
        index++;
        pwm_stopped = 0;
        pwm_set(i);

    }
    else if( cmd == 0 || cmd == 33685506)
    {
        pwm_stopped = 1;
		pwm_set(0);
        queue_reset();
        index = 0;
    }
    else
    {
        queue_tone[index] = cmd;
        index++;
    }
#endif	
}


static void pwm_set_value(unsigned long data)
{
    //mutex_lock(&pwm_lock);
    //printk("?");
    if (pwm_stopped)
        return;

    pwm_set(queue_tone[index]);
    if(index >= total_rings)
        index = 0;
    else
        index++;
    //mutex_unlock(&pwm_lock);
}


static int pwm_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct pwm_device *pwm;
    unsigned long flags;
    unsigned long tcon;
    unsigned int id = pdev->id;
    int ret;
    unsigned int tmp;

    //printk("KeyAsic_PWM_Driver_Loading .... \n");

    pwm = kzalloc(sizeof(struct pwm_device), GFP_KERNEL);
    if (pwm == NULL)
    {
        dev_err(dev, "failed to allocate pwm_device\n");
        return -ENOMEM;
    }

    pwm->pdev = pdev;
    pwm->pwm_id = id;

    /* calculate base of control bits in TCON */
    //pwm->tcon_base = id == 0 ? 0 : (id * 4) + 4;

    /*pwm->clk = clk_get(dev, "pwm-tin");
    if (IS_ERR(pwm->clk)) {
    	dev_err(dev, "failed to get pwm tin clk\n");
    	ret = PTR_ERR(pwm->clk);
    	goto err_alloc;
    }

    pwm->clk_div = clk_get(dev, "pwm-tdiv");
    if (IS_ERR(pwm->clk_div)) {
    	dev_err(dev, "failed to get pwm tdiv clk\n");
    	ret = PTR_ERR(pwm->clk_div);
    	goto err_clk_tin;
    }*/



    platform_set_drvdata(pdev, pwm);

    /*pwm_set(0x033201f7);
    pwm_set(0x0332020c);
    pwm_set(0x0a50024b);
    pwm_set(0x0a500296);
    pwm_set(0x0a5002bd);
    pwm_set(0x0a500313);
    pwm_set(0x0a50036e);
    pwm_set(0x0a5003ec);
    pwm_set(0x0a500418);*/

    //printk("KeyAsic_PWM_Driver_Loaded .... \n");

    return 0;

    /*err_clk_tdiv:
    clk_put(pwm->clk_div);

    err_clk_tin:
    clk_put(pwm->clk);

    */
err_reqirq:
    free_irq(pwm->irq, pwm);

err_alloc:
    kfree(pwm);
    return ret;
}

static int pwm_remove(struct platform_device *pdev)
{
    struct pwm_device *pwm = platform_get_drvdata(pdev);

    clk_put(pwm->clk_div);
    clk_put(pwm->clk);
    free_irq(pwm->irq, pwm);
    kfree(pwm);

    return 0;
}

static struct platform_driver pwm_driver =
{
    .driver		= {
        .name	= "KeyAsic-pwm",
        .owner	= THIS_MODULE,
    },
    .probe		= pwm_probe,
                  .remove		= __devexit_p(pwm_remove),
                            };

void pwm_stop(void)
{
    unsigned long tcon;
    //KA_DBG("pwm stop\n");
    tcon = __raw_readl (TCON0);
    __raw_writel (tcon & ~1, TCON0);
	pwm_stopped = 1;
}
void pwm_set(int cmd)
{
    unsigned long tcon;
    unsigned long tcnt;
    unsigned long tcfg1;
    unsigned long tcfg0;
    unsigned long pclk = KA2000_OSC_CLOCK;
    unsigned int tmp;
    int i=0;
    struct pwm_arg *p = (struct pwm_arg *)&cmd;
    struct clk *clk_p;
    int n = 18;

    //KA_DBG("pwm_set cmd %d\n",cmd);
    //printk("pwm %08x\n", cmd);
    mutex_lock(&pwm_lock);

    if (cmd <= 0)//if the input parameter is less than or equal to 0, then let the buzzer stopped working
    {
        pwm_stop();
        mutex_unlock(&pwm_lock);
        return;
    }

    //The following operations on the register to start a talk with the above steps and 2440 PWM timer PWM register operations manual easier to understand some terms
    tcfg1 = __raw_readl (TCFG1);//read the value of the timer configuration register 1
    tcfg0 = __raw_readl (TCFG0);//read the value of the timer configuration register 0
    tcfg0 &= ~TCFG_PRESCALER0_MASK;
    tcfg0 |= (50 - 1);//set the value of 49 tcfg0
    tcfg1 &= ~TCFG1_MUX0_MASK;
    tcfg1 |= TCFG1_MUX0_DIV16;//set the value of 0x0011 tcfg1 namely: 1 / 16

    __raw_writel (tcfg1, TCFG1);//write the value tcfg1 Timer 1 configuration register
    __raw_writel (tcfg0, TCFG0);//write the value tcfg0 timer configuration register 0

    //struct clk *clk_p;
    clk_p = clk_get(NULL, "PWM");
    pclk = clk_get_rate(clk_p)/2;//the queue from the system clock to get pclk platform clock frequency, in include / linux / clk.h defined
    n = (9 * pclk) / 12000000;
    if(n == 0)
    {
        n = 9;
        printk("pclk %d\n", pclk);
    }

    //if the input parameter is greater than 0, and let the buzzer, and different parameters, not the same frequency buzzer
    if(p->frequency == 0)
        p->frequency = 524;     //default to DO
    tcnt = (pclk / (50 * 16 * p->frequency)) - 1;//calculate the output of Timer 0 clock frequency (pclk / {prescaler0 1} / divider value)

    __raw_writel (tcnt, TCNTB0);//Set Timer 0 count buffer register value
    if(p->volume > 50)
        p->volume -= 50;
    //eee: lower volume
    p->volume = 1;
    __raw_writel (tcnt * p->volume / 100, TCMPB0);//Set Timer 0 compare buffer register value

    //pwm_interval = p->duration * pclk / tcnt / 50 / 16;
    //printk("PWM TCNTB0=%d, TCMPB0=%d, Interval=%d\n", __raw_readl(TCNTB0), __raw_readl(TCMPB0), pwm_interval);

    tcon = __raw_readl (TCON0);//read the value of the timer control register
    tcon &= ~0x1f;
    tcon |= 0xb;//close the dead zone, automatic overload, the inverter off, update TCNTB0 & TCMPB0, start timer 0
    __raw_writel (tcon, TCON0);//set the timer control register bits 0-4, ie the control of Timer 0
    tcon &= ~2;
    __raw_writel (tcon, TCON0);//Clear Timer 0 manual update bit
    barrier();

    //if (!pwm_stopped)
    /*if (pclk == 6000000)
        n = 9;
    else
        n = (18 * pclk) / 12000000;*/
#if CONFIG_KA2000_CHIP_VERSION == 0xA
    if(mod_timer(&pwm_timer, jiffies + (p->duration * n)))
#else
    if(mod_timer(&pwm_timer, jiffies + (p->duration * n * 2)))
#endif
        printk("pwm timer active!\n");

    {
        //mod_timer(&pwm_timer, jiffies + (p->duration * n / 2));  //37
    }
    mutex_unlock(&pwm_lock);
}
EXPORT_SYMBOL(pwm_set);

static int __init pwm_init(void)
{
    int ret;
    int i;

    index = 0;
    //printk("PWM_init \n");

    /*	clk_scaler[0] = clk_get(NULL, "pwm-scaler0");
    	clk_scaler[1] = clk_get(NULL, "pwm-scaler1");

    	if (IS_ERR(clk_scaler[0]) || IS_ERR(clk_scaler[1])) {
    		printk(KERN_ERR "%s: failed to get scaler clocks\n", __func__);
    		return -EINVAL;
    	}
    	printk("clk scaler=%x %x\n", clk_scaler[0], clk_scaler[1]);
    */

    init_timer(&pwm_timer);
    pwm_timer.function = pwm_set_value;
    pwm_timer.data     = (unsigned long)&pwm_value;


    platform_device_register(&ka2000pwm_device);

    ret = platform_driver_register(&pwm_driver);
    if (ret)
        printk(KERN_ERR "%s: failed to add pwm driver\n", __func__);

    return ret;
}




arch_initcall(pwm_init);
