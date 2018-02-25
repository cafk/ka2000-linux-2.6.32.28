/*
 * KeyASIC KA2000 GPIO driver
 *
 *  Copyright (C) 2013 KeyASIC.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */


//#include <linux/config.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
//#include <linux/iobuf.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <asm/uaccess.h>
//#include <asm/hardware.h>
#include <asm/io.h>
#include <linux/vmalloc.h>
#include <linux/seq_file.h>
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#define IO_OFFSET	0x55000000
#define __IO_ADDRESS(x)	((x) + IO_OFFSET)
#define IO_ADDRESS(pa)          IOMEM(__IO_ADDRESS(pa))

#ifdef __ASSEMBLER__
#define IOMEM(x)                x
#else
#define IOMEM(x)                ((void __force __iomem *)(x))

#define ka2000_readb(a)	__raw_readb(IO_ADDRESS(a))
#define ka2000_readw(a)	__raw_readw(IO_ADDRESS(a))
#define ka2000_readl(a)	__raw_readl(IO_ADDRESS(a))

#define ka2000_writeb(a, v)	__raw_writeb(v, IO_ADDRESS(a))// 1 char
#define ka2000_writew(a, v)	__raw_writew(v, IO_ADDRESS(a))// 2 short
#define ka2000_writel(a, v)	__raw_writel(v, IO_ADDRESS(a))// 4 int
#endif /* __ASSEMBLER__ */

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/



#define KA2000_GPIO_OEN 0xa0005000
#define KA2000_GPIO_IN	0xa0005004
#define KA2000_GPIO_OUT	0xa0005008

#define GPIO_MAJOR 220         //major device number
#define GPIO_NAME "ka-gpio"       //device name
#define word_write(a,v) 	__raw_writel(v, IO_ADDRESS(a))

static int device_major = GPIO_MAJOR; //system, the major number dynamically generated


static void ka2000_set_gpo(unsigned int pin ,unsigned char on)
{

   if(on)
       pin = (1 << pin);
   else
   	pin=((ka2000_readl(KA2000_GPIO_OUT))&(~(1<<pin)));
   
	ka2000_writel(KA2000_GPIO_OUT,pin);
}

static char ka2000_get_gpo(unsigned int pin)
{
	
	return (ka2000_readl(KA2000_GPIO_OUT)>>pin);	  	
}

/*
 *  The read routine is generic, it relies on the preallocated rbuffer
 *  to supply the data.
 */
//static ssize_t ka2000_gpio_read( struct file *file, 
//			  char __user *buffer,
//			  size_t len,
//			  loff_t *offset )
static ssize_t ka2000_gpio_read( struct file *file,char *buffer, size_t len, loff_t *offset )

{
	return 	ka2000_get_gpo(buffer[0]);
}


/*
 *  The write routine is generic, it fills in a preallocated rbuffer
 *  to supply the data.
 */
//static ssize_t ka2000_gpio_write( struct file *file,
//			   const char __user *buffer,
//			   size_t len,
//			   loff_t *offset )
static ssize_t ka2000_gpio_write( struct file *file, char *buffer, size_t len, loff_t *offset )

{
	ka2000_set_gpo(buffer[0],buffer[1]);
	return 0;
}

	
static int ka2000_gpio_open(struct inode *inode, struct file *file)
{
       int ret;
       ret = nonseekable_open(inode, file);
       return ret;

}
 

static int ka2000_gpio_release(struct inode *inode, struct file *file)
{
       return 0;
}
 
static loff_t ka2000_gpio_lseek(struct file *file, loff_t offset, int origin)
{
          return -ESPIPE;
}

static struct file_operations ka2000_gpio_fops={
	.read    = ka2000_gpio_read,
	.write   = ka2000_gpio_write,
	.open	 = ka2000_gpio_open,
	.llseek	 = ka2000_gpio_lseek,
	.release = ka2000_gpio_release,
}; 
              


static int  __init ka2000_gpio_init(void)
{

    ka2000_writel(KA2000_GPIO_OEN,0xff);	
		ka2000_writel(0xa0005008,0x0);
	/* register a device for user mode control gpio, mknod gpio c 220 0 */
    device_major = register_chrdev(GPIO_MAJOR, GPIO_NAME, &ka2000_gpio_fops);
    if(device_major < 0)
    {
        printk(GPIO_NAME " register falid!\n");
        //return device_major;
    }	
	return device_major;
}

static void __exit ka2000_gpio_exit(void)
{
	
	unregister_chrdev(GPIO_MAJOR,GPIO_NAME);
	
}

module_init(ka2000_gpio_init);
module_exit(ka2000_gpio_exit);


MODULE_DESCRIPTION("ka-gpio");
MODULE_AUTHOR("KeyASIC");
MODULE_LICENSE("GPL");