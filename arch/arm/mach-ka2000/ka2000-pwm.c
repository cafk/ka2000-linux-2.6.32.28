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
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
//#include <linux/clk.h>
#include <mach/clock.h>
#include <linux/device.h>
#include <asm/io.h>

#include <linux/autoconf.h>
#ifndef CONFIG_KA2000_PRINTK_ENABLE
#define printk dprintk
static inline int dprintk(const char *fmt, ...)
{
      return 0;
}
#endif
extern struct clk *clk_get(struct device *dev, const char *id);
extern unsigned long clk_get_rate(struct clk *clk);
extern void clk_put(struct clk *clk);
extern int clk_set_rate(struct clk *clk, unsigned long rate);

//=========================================================================
// Address
//=========================================================================
#define TCFG0					0xA0002000 + 0x55000000
#define TCFG1					0xA0002004 + 0x55000000
#define TCON0					0xA0002008 + 0x55000000
#define TCNTB0					0xA0002010 + 0x55000000
#define TCMPB0					0xA0002014 + 0x55000000
//=========================================================================
// Register
//=========================================================================
#define TOUT0   				(0x2 << 0)
#define TCFG1_MUX0_DIV16  		(0x3 << 0)
#define TCFG1_MUX0_MASK     	(0x3 << 0)
#define TCFG_PRESCALER0_MASK 	(0xFF << 0)
//=========================================================================


#define PWM_MAJOR 125           //major device number
#define PWM_NAME "ka-pwm"       //device name

static int device_major = PWM_MAJOR;//system, the major number dynamically generated

//Open the device
static int pwm_open(struct inode *inode, struct file *file)
{
    return 0;
}

//Turn off the device
static int pwm_close(struct inode *inode, struct file *file)
{
#if 0
    // Added by Adam 2010-11-05
    //===============================================	
    unsigned int tmp;
    tmp = __raw_readl(TCON0);
    tmp &= ~TOUT0;
    //__raw_writel(tmp, TCON0); 
	__raw_writel(tmp, 0xf5005008); 
	//===============================================
#endif   
    return 0;
}

void queue_construction(int cmd);
//Control equipment
static int pwm_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    printk("pwm set %d\n", cmd);
    queue_construction(cmd);
    //printk("KeyAsic_PWM_IOCTL .... \n");
    return 0;

}

//Device operations structure
static struct file_operations pwm_fops =
{
    .owner   = THIS_MODULE,
    .open    = pwm_open,
    .release = pwm_close,
    .ioctl   = pwm_ioctl,
};

//Define a device class
static struct class *pwm_class;

static int __init pwm_init(void)
{
    //printk("KeyAsic_PWM_Driver_Init \n");

    //Registered as a character device, major number assigned to 0, the system automatically, device, called my2440_pwm, up successful return to the major number dynamically generated
    device_major = register_chrdev(PWM_MAJOR, PWM_NAME, &pwm_fops);
    if(device_major < 0)
    {
        printk(PWM_NAME " register falid!\n");
        return device_major;
    }

    //printk(PWM_NAME " register ok!\n");

    //Register a device class, so that you can mdev / dev / directory automatically create a device node
    pwm_class = class_create(THIS_MODULE, PWM_NAME);

    if(IS_ERR(pwm_class))
    {
        printk(PWM_NAME " register class falid!\n");
        return -1;
    }

    //printk(PWM_NAME " register class ok!\n");

    //Create a device node, device, called PWM_NAME
    //device_create(pwm_class, NULL, MKDEV(device_major, 0), NULL, PWM_NAME);

    //printk(PWM_NAME " create ok!\n");

    //printk("KeyAsic_PWM_Driver_Init finish \n");

#if 0
    // Added By Adam
    //==================================================================
    //Define some local variables
    unsigned long tcon;
    unsigned long tcnt;
    unsigned long tcfg1;
    unsigned long tcfg0;
    struct clk *clk_p;
    unsigned long pclk;
    unsigned int tmp;
    unsigned int cmd = 1000;

    //The following operations on the register to start a talk with the above steps and 2440 PWM timer PWM register operations manual easier to understand some terms
    tcfg1 = __raw_readl (TCFG1);//read the value of the timer configuration register 1
    tcfg0 = __raw_readl (TCFG0);//read the value of the timer configuration register 0
    tcfg0 &= ~TCFG_PRESCALER0_MASK;
    tcfg0 | = (50 - 1);//set the value of 49 tcfg0
    tcfg1 &= ~TCFG1_MUX0_MASK;
    tcfg1 | = TCFG1_MUX0_DIV16;//set the value of 0x0011 tcfg1 namely: 1 / 16
    __raw_writel (tcfg1, TCFG1);//write the value tcfg1 Timer 1 configuration register
    __raw_writel (tcfg0, TCFG0);//write the value tcfg0 timer configuration register 0
    clk_p = clk_get(NULL, "pclk");

    if (cmd <= 0)//if the input parameter is less than or equal to 0, then let the buzzer stopped working
    {
        pclk = 0;
        tcnt = 0;
    }
    else//if the input parameter is greater than 0, and let the buzzer, and different parameters, not the same frequency buzzer
    {
        pclk = clk_get_rate (clk_p);//the queue from the system clock to get pclk platform clock frequency, in include / linux / clk.h defined
        tcnt = (pclk/50/16) / cmd;//calculate the output of Timer 0 clock frequency (pclk / {prescaler0 1} / divider value)
    }
    __raw_writel (tcnt, TCNTB0);//Set Timer 0 count buffer register value
    __raw_writel (tcnt / 2, TCMPB0);//Set Timer 0 compare buffer register value
    tcon = __raw_readl (TCON0);//read the value of the timer control register
    tcon &= ~0x1f;
    tcon | = 0xb;//close the dead zone, automatic overload, the inverter off, update TCNTB0 & TCMPB0, start timer 0
    __raw_writel (tcon, TCON0);//set the timer control register bits 0-4, ie the control of Timer 0
    tcon &= ~2;
    __raw_writel (tcon, TCON0);//Clear Timer 0 manual update bit

    if (cmd <= 0)//if the input parameter is less than or equal to 0, then let the buzzer stopped working
    {
        // Modified By Adam 2010-11-05
        //==========================================================
        //Here resumed GPB0 IO port port for output, we can see from the diagram directly to the low level enables the buzzer to stop working
        tmp = __raw_readl(TCON0);
        tmp &= ~TOUT0;
        __raw_writel(tmp, TCON0);
        //==========================================================
    }
    else
    {
        // Modified By Adam 2010-11-04
        //==========================================================
        //Port of GPB0 multiplexed multiplexing feature set, set to TOUT0 PWM output
        tmp = __raw_readl(TCON0);
        tmp |= TOUT0;
        __raw_writel(tmp, TCON0);
        //==========================================================
    }
    //===================================================================
#else
    // Added by Adam 2010-11-05
    //===============================================
    unsigned int tmp;
    tmp = __raw_readl(TCON0);
    tmp &= ~TOUT0;
    __raw_writel(tmp, TCON0);
    //===============================================

#endif


    return 0;
}

static void __exit pwm_exit(void)
{
    //Write-off equipment
    unregister_chrdev(device_major, PWM_NAME);

    //Removing the device
    //device_destroy(pwm_class, MKDEV(device_major, 0));

    //Write-off Equipment
    class_destroy(pwm_class);
}

module_init(pwm_init);
module_exit(pwm_exit);
MODULE_LICENSE("PGL");
MODULE_AUTHOR("Apik");
MODULE_DESCRIPTION("PWM API");
