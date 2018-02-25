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
#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/iobuf.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <linux/vmalloc.h>

#define KA2000_GPO 0xa0005008

#define GPIO_MAJOR 220         //major device number
#define GPIO_NAME "gpio"       //device name

static int device_major = GPIO_MAJOR; //system, the major number dynamically generated


static void ka2000_set_gpo(int pin, char on)
{
    volatile u32 *addr;
    addr = 0xf5005008;
    if (on)
    	*addr |= (1 << pin);
    else
		*addr &= ~(1 << pin);    	
}

static char ka2000_get_gpo(int pin)
{
    volatile u32 *addr;
    
    addr = 0xf5005008;
    
	return (((*addr) >> pin) & 1);		  	
}

/*
 *  The read routine is generic, it relies on the preallocated rbuffer
 *  to supply the data.
 */
static ssize_t ka2000_gpio_read( struct file *file, 
			  char __user *buffer,
			  size_t len,
			  loff_t *offset )
{
	*buffer = ka2000_get_gpo(*offset);
	return 1;
}


/*
 *  The write routine is generic, it fills in a preallocated rbuffer
 *  to supply the data.
 */
static ssize_t ka2000_gpio_write( struct file *file,
			   const char __user *buffer,
			   size_t len,
			   loff_t *offset )
{
		
	ka2000_set_gpo(*offset, *buffer);
	
	return 1;
}


static struct file_operations ka2000_gpio_fops={
	.read    = ka2000_gpio_read,
	.write   = ka2000_gpio_write,
	.open	 = ka2000_gpio_open,
	.release = ka2000_gpio_release,
}; 
              


void __init ka2000_gpio_init(void)
{
	/* turn off all GPO */
	ka2000_writel(0x00, KA2000_GPO);	
	
	/* register a device for user mode control gpio, mknod gpio c 220 0 */
    device_major = register_chrdev(GPIO_MAJOR, GPIO_NAME, &ka2000_gpio_fops);
    if(device_major < 0)
    {
        printk(GPIO_NAME " register falid!\n");
        //return device_major;
    }	
}
