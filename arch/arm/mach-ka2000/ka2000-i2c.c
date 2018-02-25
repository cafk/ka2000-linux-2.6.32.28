//#define DEBUG
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/irqs.h>
#include <mach/ka2000.h>
//#include <mach/ka2000_define.h>
#include <mach/io.h>

#define DEBUG_RW 0

/* define I2c register address */
#ifndef KA_I2C_BASE
#define KA_I2C_BASE 0xA000D000
#endif
#define KA_I2C_CR  (KA_I2C_BASE +  0x0) /* Control Register */
#define KA_I2C_SR  (KA_I2C_BASE +  0x4) /* Status Register */
#define KA_I2C_DR  (KA_I2C_BASE +  0x8) /* Bus Data Register, bit0-5 data */
#define KA_I2C_AR  (KA_I2C_BASE +  0xC) /* Slave Address Register,15:slave addr mapped,14:mode, 0-9:slave addr*/
#define KA_I2C_TR  (KA_I2C_BASE + 0x10) /* Timer Register, prescaler, SCL Low/High time counter */

/* define I2c control register bits content */
#define _CR_BIT_MODE_      (1 << 15)  /* 0 = byte level, 1 = bit level */
#define _CR_BYTE_MODE_     (0 << 15)   
#define _CR_I2C_ENABLE_    (1 << 14)
#define _CR_SOFT_RESET_    (1 << 13)
#define _CR_MASTER_MODE_   (1 << 12)  
#define _CR_SLAVE_MODE_    (0 << 12)  
#define _CR_TRANS_TX_MODE_ (1 << 11)  /* 0 = Receiver ,     1 = Transmitter    */
#define _CR_TRANS_RX_MODE_ (0 << 11) 
#define _CR_SPEED_        (1 << 10)
#define _CR_INT_EN_       (1 << 9)
#define _CR_GCE_          (1 << 8)
#define _CR_HSE_          (1 << 7)
#define _CR_POLL_EN_      (1 << 6)
#define _CR_RESUME_CMD_   (1 << 3)
#define _CR_START_CMD_    (1 << 2)
#define _CR_STOP_CMD_     (1 << 1)
#define _CR_ACK_          (1 << 0)

/* define I2c status register bits content */
#define _SR_TRANS_DIR_    (1 << 15)  /* 0 = byte level, 1 = bit level */
#define _SR_PAUSE_        (1 << 8)
#define _SR_GEN_CALL_     (1 << 7)
#define _SR_ADDR_MATCH_   (1 << 6)
#define _SR_HS_MODE_      (1 << 5)
#define _SR_BUS_BUSY_     (1 << 4)
#define _SR_BUS_ERR_      (1 << 3)
#define _SR_ARB_FAIL_     (1 << 2)
#define _SR_END_OF_TRANS_ (1 << 1)
#define _SR_INT_PENDING_  (1 << 0)


#define KA_I2C_CLOCK        12000000      /* Hz. max 400 Kbits/sec */
#define KA_I2C_TIMEOUT        msecs_to_jiffies(100)   /* transfer timeout */


static struct clk *i2c_clk;

#define word_read(reg)      __raw_readl(IO_ADDRESS(reg))
#define word_write(reg, val)    __raw_writel((val), IO_ADDRESS(reg))


/*-----------------------------------------------------------------------------------------*/
static int _i2c_check_error_(void)
{
    if (word_read(KA_I2C_SR) & _SR_BUS_ERR_) {
        /* clear status register */
        word_write(KA_I2C_SR, 0x0000);   
		printk("i2c bus err\n");
        return 1;
    }
    
    return 0;
}


/*
 * Poll the i2c status register until the specified bit is set.
 * Returns 0 if timed out (100 msec).
 */
static int _i2c_poll_status_(unsigned long bit)
{
    int timeout = 100000000;
    int err = 0;
    
    do {
        udelay(1000);
    } while (!(word_read(KA_I2C_SR) & bit) && (--timeout > 0));

    err = _i2c_check_error_() | (timeout == 0);
    if (timeout == 0)
    {
    	printk("_i2c_poll_status_ timeout\n");
    }
    return err;
}


/*-----------------------------------------------------------------------------------------*/
static int xfer_read(struct i2c_adapter *adap, unsigned char *buf, int length)
{
    /* all bit send at same time - 0x5000 */
    //unsigned int cr_cmd = _CR_I2C_ENABLE_|_CR_INT_EN_|_CR_MASTER_MODE_|_CR_TRANS_RX_MODE_;
    unsigned int cr_cmd = _CR_I2C_ENABLE_|_CR_INT_EN_|_CR_MASTER_MODE_|_CR_TRANS_RX_MODE_|_CR_POLL_EN_;
    int i;

    for (i = 0; i < length; i++) {
		word_write(KA_I2C_CR, cr_cmd | _CR_RESUME_CMD_);   // Set Bit Level , Set Enable i2c controller.

        if (_i2c_poll_status_(_SR_INT_PENDING_)) {
            dev_dbg(&adap->dev, "TXRDY timeout\n");
            return -ETIMEDOUT;
        }
		buf[i] = word_read(KA_I2C_DR) ;
    } 
	
	/* Send Stop */
	word_write(KA_I2C_CR, cr_cmd | _CR_STOP_CMD_);

    return 0;
}

static int xfer_write(struct i2c_adapter *adap, unsigned char *buf, int length)
{
    /* all bit send at same time 0x5800 */
    //unsigned int cr_cmd = _CR_I2C_ENABLE_|_CR_INT_EN_|_CR_MASTER_MODE_|_CR_TRANS_TX_MODE_;
    unsigned int cr_cmd = _CR_I2C_ENABLE_|_CR_INT_EN_|_CR_MASTER_MODE_|_CR_TRANS_TX_MODE_|_CR_POLL_EN_;
    int i;

    for (i = 0; i < length; i++) {
        word_write(KA_I2C_DR, buf[i]);
		word_write(KA_I2C_CR, cr_cmd | _CR_RESUME_CMD_);	
		
        if (_i2c_poll_status_(_SR_INT_PENDING_)) {
            dev_dbg(&adap->dev, "TXRDY timeout\n");
            return -ETIMEDOUT;
        }
    } 

	if (length > 1)
	{
    	/* Send Stop */
	    word_write(KA_I2C_CR, cr_cmd | _CR_STOP_CMD_);
	}
    return 0;
}

static int xfer_addr(struct i2c_adapter *adap, struct i2c_msg *pmsg)
{
    /* all bit send at same time 0x5804 */
    unsigned int cr_cmd = _CR_I2C_ENABLE_|_CR_INT_EN_|_CR_MASTER_MODE_|_CR_TRANS_TX_MODE_;
    
    /* store address at data register */
    if (pmsg->flags & I2C_M_RD)
        word_write(KA_I2C_DR, (pmsg->addr << 1) | 1);
    else
        word_write(KA_I2C_DR, pmsg->addr << 1);
    
    /* send start command 0x5804 */
    word_write(KA_I2C_CR, cr_cmd | _CR_START_CMD_); 
    
    if (_i2c_poll_status_(_SR_INT_PENDING_)) {
        dev_dbg(&adap->dev, "TXRDY timeout\n");
        return -ETIMEDOUT;
    }

    return 0;
}

/*-----------------------------------------------------------------------------------------*/
/*
 * Generic i2c master transfer entrypoint.
 */

static void _debug_print_write_pmsg_buf_(struct i2c_msg *pmsg)
{
#if (DEBUG_RW&1)
	if (!(pmsg->flags & I2C_M_RD))
	{
		int j;
		printk("I2C W ");
		for (j = 0; j < pmsg->len; j++)
			printk("%x, ", pmsg->buf[j]);
		printk("\n");
	}
#endif	
}

static void _debug_print_read_pmsg_buf_(struct i2c_msg *pmsg)
{
#if (DEBUG_RW&2)
	if ((pmsg->flags & I2C_M_RD))
	{
		int j;
		printk("I2C R ");
		for (j = 0; j < pmsg->len; j++)
			printk("%x, ", pmsg->buf[j]);
		printk("\n");
	}	
#endif	
}

static int ka2000_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *pmsg, int num)
{
    int i, ret = num;
	
    dev_dbg(&adap->dev, "i2c_xfer: proc %d mesg:\n", num);

    for (i = 0; i < num; i++) {
        dev_dbg(&adap->dev, " #%d: %sing %d byte%s %s 0x%02x\n", i,
            pmsg->flags & I2C_M_RD ? "read" : "writ",
            pmsg->len, pmsg->len > 1 ? "s" : "",
            pmsg->flags & I2C_M_RD ? "from" : "to", pmsg->addr);

/*
	printk(" #%d: %sing %d byte%s %s 0x%02x\n", i,
            pmsg->flags & I2C_M_RD ? "read" : "writ",
            pmsg->len, pmsg->len > 1 ? "s" : "",
            pmsg->flags & I2C_M_RD ? "from" : "to", pmsg->addr);
*/
		_debug_print_write_pmsg_buf_(pmsg);
		mdelay(10);

        ret = xfer_addr(adap, pmsg);                
		if (ret)
			return ret;
		
        if (pmsg->len && pmsg->buf) {   /* sanity check */
            if (pmsg->flags & I2C_M_RD)
                ret = xfer_read(adap, pmsg->buf, pmsg->len);
            else
                ret = xfer_write(adap, pmsg->buf, pmsg->len);

            if (ret < 0)
                return ret;
			
			_debug_print_read_pmsg_buf_(pmsg);
		mdelay(10);
        }
//        printk("xfer complete\n");
        dev_dbg(&adap->dev, "xfer complete\n");
        pmsg++;     /* next message */
    }
    return i;
}

/*-----------------------------------------------------------------------------------------*/

/*
 * Initialize the I2C hardware registers.
 */
static void _i2c_softe_reset_(void)
{
    word_write(KA_I2C_CR, _CR_SOFT_RESET_); /* Reset peripheral */
}
static void _i2c_set_master_mode_(void)
{
    word_write(KA_I2C_CR, 0x5800); /* Set Master mode */
}

static void _i2c_set_clock_(void)
{
    unsigned long i2c_clk_rate;
    unsigned long cdiv, ckdiv;  
    unsigned long scl_low_time = 0x78;
    unsigned long scl_high_time = 0x78;

    /* get i2c clock rate */
    i2c_clk_rate = clk_get_rate(i2c_clk);

    /* Calcuate clock prescaler dividers bits 15-14 */
    cdiv = ((i2c_clk_rate * 1) / KA_I2C_CLOCK);
    
    if (cdiv <= 1)
        ckdiv = 0;  /* 00 = PCLK */
    else if (cdiv <= 4)
        ckdiv = 1;  /* 01 = PCLK/4 */
    else if (cdiv <= 16)
        ckdiv = 2;  /* 10 = PCLK/16 */
    else 
        ckdiv = 3;  /* 11 = PCLK/64 */

    //word_write(KA_I2C_TR, (ckdiv << 14) | (scl_low_time << 7) | scl_high_time);
     word_write(KA_I2C_TR, (ckdiv << 14) | (scl_low_time << 7) | scl_high_time);
}
#define KXTE9_I2C_INT_SRC_REG1	0x16

static void __devinit ka2000_i2c_hwinit(void)
{     
    _i2c_softe_reset_();        
    _i2c_set_clock_();
	_i2c_set_master_mode_();
}


/*
 * Return list of supported functionality.
 */
static u32 ka2000_i2c_func(struct i2c_adapter *adapter)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm ka2000_algorithm = {
    .master_xfer    = ka2000_i2c_xfer,
    .functionality  = ka2000_i2c_func,
};

/*
 * Main initialization routine.
 */
static int __devinit ka2000_i2c_probe(struct platform_device *pdev)
{
    struct i2c_adapter *adapter;
    int rc;
	printk("ka2000_i2c_probe\n");

    i2c_clk = clk_get(NULL, "I2CCLK");
    if (IS_ERR(i2c_clk)) {
        dev_err(&pdev->dev, "no clock defined\n");
        rc = -ENODEV;
        goto fail1;
    }

    adapter = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
    if (adapter == NULL) {
        dev_err(&pdev->dev, "can't allocate inteface!\n");
        rc = -ENOMEM;
        goto fail2;
    }
    snprintf(adapter->name, sizeof(adapter->name), "KA2000");
    adapter->algo = &ka2000_algorithm;
    adapter->class = I2C_CLASS_HWMON;
    adapter->dev.parent = &pdev->dev;
	platform_set_drvdata(pdev, adapter);

    clk_enable(i2c_clk);        /* enable peripheral clock */
    ka2000_i2c_hwinit();        /* initialize I2C controller */

    rc = i2c_add_numbered_adapter(adapter);
    if (rc) {
        dev_err(&pdev->dev, "Adapter %s registration failed\n",
                adapter->name);
        goto fail3;
    }

    dev_info(&pdev->dev, "KA2000 i2c bus driver.\n");
    return 0;

fail3:
    platform_set_drvdata(pdev, NULL);
    kfree(adapter);
    clk_disable(i2c_clk);
fail2:
    clk_put(i2c_clk);
fail1:
    return rc;
}

static int __devexit ka2000_i2c_remove(struct platform_device *pdev)
{
    struct i2c_adapter *adapter = platform_get_drvdata(pdev);
    int rc;

    rc = i2c_del_adapter(adapter);
    platform_set_drvdata(pdev, NULL);

    clk_disable(i2c_clk);       /* disable peripheral clock */
    clk_put(i2c_clk);

    return rc;
}


/* NOTE: could save a few mA by keeping clock off outside of ka2000_xfer... */

static int ka2000_i2c_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    clk_disable(i2c_clk);
    return 0;
}

static int ka2000_i2c_resume(struct platform_device *pdev)
{
    return clk_enable(i2c_clk);
}

static struct platform_driver ka2000_i2c_driver = {
    .probe      = ka2000_i2c_probe,
    .remove     = __devexit_p(ka2000_i2c_remove),
    .suspend    = ka2000_i2c_suspend,
    .resume     = ka2000_i2c_resume,
    .driver     = {
        .name   = "ka2000_i2c",
        .owner  = THIS_MODULE,
    },
};

//module_platform_driver(ka2000_i2c_driver);
static int __init ka_i2c_init(void)
{
    int rc;
    if((rc=platform_driver_register(&ka2000_i2c_driver))==0){
	printk("i2c_driver register succeed\n");
	return 0;
    }
    else
	return rc;
}
static void __exit ka_i2c_exit(void)
{
	printk("i2c_driver unregister\n");
    platform_driver_unregister(&ka2000_i2c_driver);
}
module_init(ka_i2c_init);
module_exit(ka_i2c_exit);

MODULE_AUTHOR("KeyASIC");
MODULE_DESCRIPTION("I2C driver for KeyASIC KA2000");
MODULE_LICENSE("GPL");
