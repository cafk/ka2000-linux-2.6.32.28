/*==================================================================================================

     Header Name: i2c_interface.c

     General Description: Implementation of I2C communication protocol with accelerometer.
==================================================================================================*/
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/pid.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <asm/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <mach/ka2000.h>
#include <asm/irq.h>
#include <mach/io.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#include "i2c_interface.h"

spinlock_t *lock;



static int major = 250;

#define NAME "gpio_i2c"
#define SENSOR_GPIO_RG 33   	/*define io range 33*/
#define DEVNAME "gpio_i2c"

#define MAX_RETRY 4


MODULE_AUTHOR("Alex");
MODULE_DESCRIPTION("gpio i2c");
MODULE_LICENSE("GPL");

module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");

static int gpio_i2c_open(struct inode *inode, struct file *file);
static int gpio_i2c_release(struct inode *inode, struct file *file);
static int gpio_i2c_ioctl(struct inode *inode, struct file *file,
                     unsigned int cmd, unsigned long arg);
static ssize_t gpio_i2c_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos);

static ssize_t gpio_i2c_read(struct file *filp, const char __user *buf, size_t count, loff_t *pos);

static irqreturn_t motion_sensor_ext_int_irq (int irq, void * dev_id);

static bool polling_iic_ready(void);




static struct cdev gpio_i2c_cdev;
static struct platform_device *pdev;

static struct file_operations gpio_i2c_ops = {
	.owner	= THIS_MODULE,
	.open	= gpio_i2c_open,
	.read   = gpio_i2c_read,
	.write  = gpio_i2c_write,
	.ioctl	= gpio_i2c_ioctl,
	.release= gpio_i2c_release,
};

#define word_write(a, v) 	__raw_writel(v, IO_ADDRESS(a))
#define word_read(a)  		__raw_readl(IO_ADDRESS(a))

#define KXTE9_I2C_SLV_ADDR 0x0f
#define KXTE9_I2C_INT_SRC_REG1	0x16
#define KXTE9_I2C_INT_SRC_REG2	0x17
#define KXTE9_I2C_INT_REL		0x1A
/*==================================================================================================

FUNCTION: i2c_write

DESCRIPTION: THIS IS PSEUDO CODE
   this function is the main interface to write a byte to the slave.

ARGUMENTS PASSED:
   None

return VALUE:
   None

PRE-CONDITIONS:

POST-CONDITIONS:
   None

IMPORTANT NOTES:

1) This function is follows the protocol to access the Kionix Accelerometer.
2) This function is implemented to only write one byte to the Accelerometer.


Term Definition
S    = Start Condition
Sr   = Repeated Start Condition
SAD  = Slave Address
W    = Write Bit
R    = Read Bit
ACK  = Acknowledge
NACK = Not Acknowledge
RA   = Register Address
Data = Transmitted/Received Data
P    = Stop Condition


       -----------------------------------------------------------
 Master|START| SAD + W |      |  RA   |      | DATA |       |  P  |
       |----------------------------------------------------------
 Slave |     |         | ACK  |       | ACK  |      |  ACK  |     |
       -----------------------------------------------------------
==================================================================================================*/
static ssize_t gpio_i2c_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos)
{
    uint8 ack=1;
	uint8 retry=MAX_RETRY; // max retry
	i2c_rw_cmd_type *i2c_rw=(i2c_rw_cmd_type *) buf;
    
	while (retry>0)
	{
			if (ack==0) // success and done
			{

				spin_unlock_irq(lock);
				break;
			}
			else
			{
				spin_lock_irq(lock);
				retry--;
				if (retry < MAX_RETRY-1)
				{
					printk ("retry iic\n");
					udelay(10);					
				}

			}

			//printk ("send start condition with slave addr\n");
			word_write(IIC_DR_ADDR,i2c_rw->slave_addr<<1); // set kionix id = 0f/write mode
			word_write(IIC_CR_ADDR,0x5804);     // Set Bit Level , Set Enable i2c controller, start write.
			ack =polling_iic_ready();

			if(ack)
			{
				spin_unlock_irq(lock);
				//printk("error:send slave_addr, slave no ack!\n");
		//	printk("slave_addr is : 0x%X\n", i2c_rw->slave_addr);
				continue;
			}

			//printk ("send register address for write 0x%x\n",i2c_rw->slave_reg_addr);
			word_write(IIC_DR_ADDR,i2c_rw->slave_reg_addr); // kionix register address
			word_write(IIC_CR_ADDR,0x5808);     // Set Bit Level , Set Enable i2c controller, start write.
			ack =polling_iic_ready();
			if(ack)
			{
				spin_unlock_irq(lock);
				//printk("error:send slave_reg_addr, no ack!; addr is 0x%x\n", i2c_rw->slave_reg_addr);
				continue;
			}

			//printk ("write data : 0x%x\n", *i2c_rw->data_ptr);
			word_write(IIC_DR_ADDR,*i2c_rw->data_ptr); // data
			//printk ("send  data 0x%x\n",*i2c_rw->data_ptr);
			word_write(IIC_CR_ADDR,0x5808);   // Set Bit Level , Set Enable i2c controller.
			ack =polling_iic_ready();
			if(ack)
			{
				spin_unlock_irq(lock);
			//	printk("error:send data_ptr no ack!\n");
				continue;
			}

			//stop condition
			//printk("send stop condition\n");
			word_write(IIC_CR_ADDR,0x5802);     // Set Bit Level , Set Enable i2c controller.

	}
	if (ack==0)
	{
		return 1; // success
	}
	if (retry ==0)
	{
		printk ("iic max retry (%d) reached at slave_reg_addr 0x%x. might be connection problem \n",MAX_RETRY, i2c_rw->slave_reg_addr );
	}
    return(0);
}
/*==================================================================================================

FUNCTION: i2c_read

DESCRIPTION: THIS IS PSEUDO CODE
   this function is the main interface to read multiple bytes from the slave.

ARGUMENTS PASSED:
   None

return VALUE:
   None

PRE-CONDITIONS:

POST-CONDITIONS:
   None

IMPORTANT NOTES:

This function is follows the protocol to read multiple bytes from the Kionix Accelerometer.

Term Definition
S    = Start Condition
Sr   = Repeated Start Condition
SAD  = Slave Address
W    = Write Bit
R    = Read Bit
ACK  = Acknowledge
NACK = Not Acknowledge
RA   = Register Address
Data = Transmitted/Received Data
P    = Stop Condition

Read one byte from Slave
       ----------------------------------------------------------------------------------------
 Master|START| SAD + W |      |  RA   |      | Sr   | SAD + R |       |       |  NACK  |   P   |
       |---------------------------------------------------------------------------------------
 Slave |     |         | ACK  |       | ACK  |      |         |  ACK  | DATA  |        |       |
       ----------------------------------------------------------------------------------------


Read Multiple Bytes from slaves
       ----------------------------------------------------------------------------------------------------------
 Master|START| SAD + W |      |  RA   |      | Sr   | SAD + R |       |       |  ACK   |        | NACK   |   P   |
       |---------------------------------------------------------------------------------------------------------
 Slave |     |         | ACK  |       | ACK  |      |         |  ACK  | DATA  |        | DATA   |        |       |
       ----------------------------------------------------------------------------------------------------------


==================================================================================================*/
static ssize_t gpio_i2c_read(struct file *filp, const char __user *buf, size_t count, loff_t *pos)
{
	uint8 i;
	uint8 ack =1;
	uint8 retry=MAX_RETRY; // max retry
	i2c_rw_cmd_type *i2c_rw=(i2c_rw_cmd_type *) buf;
    
	while (retry>0)
	{
			if (ack==0) // success and done
			{

				spin_unlock_irq(lock);
				break;
			}
			else
			{
				spin_lock_irq(lock);
				retry--;
				if (retry < MAX_RETRY-1)
				{
					//printk ("retry iic\n");
					udelay(10);					
				}

			}

			word_write(IIC_DR_ADDR,i2c_rw->slave_addr<<1); // set kionix id = 0f/write mode
			word_write(IIC_CR_ADDR,0x5804);     // Set Bit Level , Set Enable i2c controller, start write.
			ack =polling_iic_ready();
			if(ack)
			{
				spin_unlock_irq(lock);
				//printk("error:send slave_addr, slave no ack. addr is 0x%x!\n",i2c_rw->slave_addr );
				continue;
			}

			word_write(IIC_DR_ADDR,i2c_rw->slave_reg_addr); // kionix register address
			word_write(IIC_CR_ADDR,0x5808);     // Set Bit Level , Set Enable i2c controller, start write.
			ack =polling_iic_ready();
			if(ack)
			{
				spin_unlock_irq(lock);
				//printk("error:send slave_reg_addr, no ack! reg_addr is 0x%x\n", i2c_rw->slave_reg_addr);
				continue;
			}

			word_write(IIC_DR_ADDR,(i2c_rw->slave_addr<<1 | 0x00000001)); // set kionix id = 0f/write mode
			word_write(IIC_CR_ADDR,0x5804);     // Set Bit Level , Set Enable i2c controller, start write.
			ack =polling_iic_ready();
			if(ack)
			{
				spin_unlock_irq(lock);
				//printk("error:send slave_addr+1, slave no ack!\n");
				continue;
			}

			for(i=0;i<i2c_rw->length ;i++)
			{

				word_write(IIC_CR_ADDR,0x5008);   // Set Bit Level , Set Enable i2c controller.
				ack = polling_iic_ready();

				if(ack)
				{
					spin_unlock_irq(lock);
					//printk("error: reading data\n");
					continue; //this is for the 'for' loop
				}

				*(i2c_rw->data_ptr+i) = word_read(IIC_DR_ADDR) ;
				//printk("read data byte = 0x%x\n", *(i2c_rw->data_ptr+i));
			}
			if (ack)
			{
				continue; // 1 more time for the while loop
			}


			//stop condition
			word_write(IIC_CR_ADDR,0x5802);     // Set Bit Level , Set Enable i2c controller.

	}

	if (ack==0)
	{
		return 1; // success
	}
    return(0);
}
static int gpio_i2c_open(struct inode *inode, struct file *file)
{
	printk("movtion sensor gpio i2c interface open now\n");
	return 0;
}

static int gpio_i2c_release(struct inode *inode, struct file *file)
{
	printk("movtion sensor gpio i2c interface release now\n");
	return 0;
}


//static char *argv_usermodehelper[ ] = {"motion_app","-o","/var/log/motion_log.txt"};
// do not  add -o option at the beginning because there is problem of stdout redirected to
// photolist.txt file in sender. this problem only happen when calling via
// call_usermodehelper

#if 0
//static char *argv_usermodehelper[ ] = {"",};
static char *argv_usermodehelper[ ] = {"motion_app",};
static char *envp[]=
{
    "HOME=/",
    "PATH=/sbin:/usr/sbin:/bin:/usr/bin",
    NULL,
};
static struct work_struct workqueue_struct;

static void workqueue_func(void *ptr)
{
	printk ("call motion_app at user space \n");
	if (call_usermodehelper("/bin/busybox", argv_usermodehelper,  envp, 0)==0) // no wait for process
	{
		printk ("successfully call_usermodehelper motion_app \n");
	}
	else
	{
		printk ("error call_usermodehelper motion_app\n");
	}
}
#endif

static int motion_app_pid=-1;
static DEFINE_SPINLOCK(i2c_irq_lock);

static irqreturn_t motion_sensor_ext_int_irq (int irq, void * dev_id)
{
#if 1    

    unsigned long irq_flags;
	
	spin_lock_irqsave(&i2c_irq_lock, irq_flags);
	
	disable_irq_nosync(irq); // disable irq and  let user space app (motion_app) enable back
	{		
		printk (" from gpio_i2c motion_sensor_ext_int_irq :  interrupt !\n");				
		
#if 0 
		INIT_WORK(&workqueue_struct, workqueue_func);
		schedule_work (&workqueue_struct);		
#endif 
		
		// send signal to motion_app
		if (motion_app_pid!=-1)
		{
		    printk("kill_pid %d\n", motion_app_pid);
			kill_pid(find_get_pid(motion_app_pid), SIGUSR2, 1);
		}
		else
		{
			//enable_irq(irq);
		}

		//disable_irq(irq);  
	}
#endif	    
    //disable_irq_nosync(irq);
	spin_unlock_irqrestore(&i2c_irq_lock, irq_flags);	
	
	return IRQ_HANDLED;

}

// the following are ioctl commands
#define DISABLE_IRQ_HANDLING 0
#define ENABLE_IRQ_HANDLING 1
#define INIT_AGAIN 2
#define MOTION_SET_PID 3

static int gpio_i2c_ioctl(struct inode *inode, struct file *file,
                     unsigned int cmd, unsigned long arg)
{

    //printk("gpio_i2c_ioctl: cmd=%u, arg=%ld\n", cmd, arg);


    switch(cmd)
    {

		case ENABLE_IRQ_HANDLING:
			printk ("enable ext_int irq\n");
			enable_irq(IRQ_EXT_INT);
			break;

		case DISABLE_IRQ_HANDLING:
			printk ("disable ext_int irq\n");
			disable_irq(IRQ_EXT_INT);

			break;
		case INIT_AGAIN:
			printk ("initialize iic module again\n");

			word_write(IIC_TR_ADDR, 0x7C78); // if tp is 83ns, iiclock = 10khz
											 // if tp is 10ns, iiclock = 83khz

			word_write(IIC_CR_ADDR, 0x5800); // enable iic control and set master mode
			break;
		case MOTION_SET_PID:
			motion_app_pid=(int)(*((int*)arg));
			printk ("motion_app pid is %d\n", motion_app_pid);
			break;
        default:
            break;

    }

	return 0;
}

static bool polling_iic_ready(void)
{
   volatile long timeout = 0xffffffff;
   while( ( word_read(IIC_SR_ADDR) & 0x00000001)  != 1) { // wait interupt happen
        timeout--;          //add some delay
        if (timeout == 0 )   //add some delay
            break;
		
	    //else
	    //   if( timeout % 1000 == 0)
        //       schedule();
    }

   if ((word_read(IIC_SR_ADDR) & 0x00000008) == 0x00000008) {    // bus error
      word_write(IIC_SR_ADDR,0x0000);     // clear interrupt
      return 1 ;
   }
   word_write(IIC_SR_ADDR,0x0000);     // clear interrupt
   return 0 ;
}

static bool polling_iic_ready_sched(void)
{
   volatile long timeout = 0xffffffff;
   while( ( word_read(IIC_SR_ADDR) & 0x00000001)  != 1) { // wait interupt happen
        timeout--;          //add some delay
        if (timeout == 0 )   //add some delay
            break;
	    //else
	    //   if( timeout % 1000 == 0)
        //       schedule();
    }

   if ((word_read(IIC_SR_ADDR) & 0x00000008) == 0x00000008) {    // bus error
      word_write(IIC_SR_ADDR,0x0000);     // clear interrupt
      return 1 ;
   }
   word_write(IIC_SR_ADDR,0x0000);     // clear interrupt
   return 0 ;
}

static int __init gpio_i2c_init(void)
{
	int rc;

	dev_t devid;
	pdev = platform_device_alloc(DEVNAME, 0);
	if (!pdev)
		return -ENOMEM;

	rc = platform_device_add(pdev);

	if (rc)
		goto undo_malloc;

	if (major) {
		devid = MKDEV(major, 0);
		rc = register_chrdev_region(devid, SENSOR_GPIO_RG, DEVNAME);
	} else {
		rc = alloc_chrdev_region(&devid, 0, SENSOR_GPIO_RG, DEVNAME);
		major = MAJOR(devid);
	}

	if (rc < 0) {
		dev_err(&pdev->dev, "trek chrdev_region err: %d\n", rc);
		goto undo_platform_device_add;
	}

	if (!major) {
		major = rc;
		printk("got dynamic major %d\n", major);
	}

	/* ignore minor errs, and succeed */
	cdev_init(&gpio_i2c_cdev, &gpio_i2c_ops);
	cdev_add(&gpio_i2c_cdev, devid, SENSOR_GPIO_RG);

	printk ("initialize iic  timer register and set master mode\n");
	word_write(IIC_TR_ADDR, 0x7C78); // alex - PCLK/4
									// if tp is 83ns, iiclock = 10khz
									 // if tp is 10ns, iiclock = 83khz

	word_write(IIC_CR_ADDR, 0x5800); // enable iic control and set master mode


	// register interrupt handling for ext_int
	if (request_irq (IRQ_EXT_INT, motion_sensor_ext_int_irq, IRQF_DISABLED, NAME, NULL)==0)
	{
		printk("successfully registered irq handling for ext int\n");
	} else
	{
		printk("cannot registered irq handling for ext int!!\n");

	}
	printk("iic driver - disable irq at the beginning \n");
	disable_irq(IRQ_EXT_INT); // disable irq by default, only enabled at ioctl once triggered from userspace

	return 0;

undo_platform_device_add:
	platform_device_del(pdev);
undo_malloc:
	platform_device_put(pdev);

	return rc;
}

static void __exit gpio_i2c_cleanup(void)
{
	cdev_del(&gpio_i2c_cdev);

	unregister_chrdev_region(MKDEV(major, 0), 32);
	platform_device_unregister(pdev);
}

module_init(gpio_i2c_init);
module_exit(gpio_i2c_cleanup);

