/*
 * KA2000 serial driver
 *
 * Copyright (C) 2010 KeyASIC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/serial_8250.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/serial.h>
#include <mach/irqs.h>
#include <mach/ka2000.h>

//#define KA2000_UART0_BASE 0xa0004000
#define PORT_KA2000 34



/* modem lines */
#define TIOCM_LE	0x001
#define TIOCM_DTR	0x002
#define TIOCM_RTS	0x004
#define TIOCM_ST	0x008
#define TIOCM_SR	0x010
#define TIOCM_CTS	0x020
#define TIOCM_CAR	0x040
#define TIOCM_RNG	0x080
#define TIOCM_DSR	0x100
#define TIOCM_CD	TIOCM_CAR
#define TIOCM_RI	TIOCM_RNG
#define TIOCM_OUT1	0x2000
#define TIOCM_OUT2	0x4000
#define TIOCM_LOOP	0x8000

struct ka2000_uart_port
{
    struct uart_port	port;
    //struct timer_list	timer;
    unsigned int		old_status;
};

#if 0
#define PORT_TIMER_ENABLED
struct timer_list	port_timer;
#endif
//struct timer_list	tx_timer;
/*
 * The work queue structure for this task, from workqueue.h
 */
static struct workqueue_struct *tx_workqueue;

struct delayed_work tx_work;

unsigned int		old_status;

static struct uart_port * ka2000_get_port();


static struct plat_serial8250_port serial_platform_data[] =
{
    {
        .membase	= (char *)IO_ADDRESS(KA2000_UART0_BASE),
        .mapbase	= (unsigned long)KA2000_UART0_BASE,
        .irq		= IRQ_UARTINT0,
        .flags		= UPF_BOOT_AUTOCONF|UPF_AUTO_IRQ, // | UPF_SKIP_TEST,
        .iotype		= UPIO_MEM,
        .regshift	= 2,
        .uartclk	= KA2000_PLL_CLOCK,

    },
    {
        .flags		= 0
    },
};


static struct platform_device serial_device =
{
    .name			= "UART_16550A",
    .id			= PLAT8250_DEV_PLATFORM,
    .dev			= {
        .platform_data	= serial_platform_data,
    },
};



#if CONFIG_ARCH_KA2000

#define KA2000_PORT_SIZE 0x100
#define SERIAL_KA2000_NR        1
#define KA2000_URRB             0
#define KA2000_URTH             0
#define KA2000_URINTR           4

#define KA2000_URFC             8
#define KA2000_URMS             0x18
#define KA2000_URLS             0x14
#define KA2000_URLC             0x0c
#define KA2000_URMC             0x10
#define KA2000_URBD             0x00
#define URLS_URTHRE             0x20
#define SERIAL_KA2000_MAJOR     4
#define SERIAL_KA2000_MINOR     64
#define SERIAL_KA2000_DEVNAME   "ttyS"
#define URLS_URTE               0x20
#define URLS_URDR               (0x01 << 0)
#define URLS_URBI               (0x01 << 4)
#define URLS_URPE               (0x01 << 2)
#define URLS_URFE               (0x01 << 3)
#define URLS_URROE              (0x01 << 1)

#define UART_DUMMY_LSR_RX	0x100


/* FIFO Control Register */
#define URFC_URFRT	(3 << 6)	/* Receive FIFO Trigger Level */
#define		URFC_URFRT_1	(0 << 6)
#define		URFC_URFRT_4	(1 << 6)
#define		URFC_URFRT_8	(2 << 6)
#define		URFC_URFRT_14	(3 << 6)
#define URFC_URTFR	(1 << 2)	/* Transmit FIFO Reset */
#define URFC_URRFR	(1 << 1)	/* Receive FIFO Reset */
#define URFC_URFE	(1 << 0)	/* FIFO Enable ???? */


/* Line Control Register */
#define URLC_URSBC	(1 << 6)	/* Set Break Condition */
#define URLC_PARITY	(7 << 3)	/* Parity */
#define		URPE_NONE	(0 << 3)
#define		URPE_ODD	(1 << 3)
#define		URPE_EVEN	(3 << 3)
#define		URPE_MARK	(5 << 3)
#define		URPE_SPACE	(7 << 3)
#define URLC_URSB	(1 << 2)	/* Stop Bits */
#define URLC_URCL	(3 << 0)	/* Character Length */
#define		URCL_5		(0 << 0)
#define		URCL_6		(1 << 0)
#define		URCL_7		(2 << 0)
#define		URCL_8		(3 << 0)



/*
 * Access macros for the KA2000 UART
 */

#define UART_GET_CHAR(p)        (__raw_readl(IO_ADDRESS(KA2000_UART0_BASE) + KA2000_URRB) & 0xFF)
#define UART_PUT_CHAR(p, c)     __raw_writel((c), IO_ADDRESS(KA2000_UART0_BASE) + KA2000_URTH)

#define UART_GET_FCR(p)         __raw_readl(IO_ADDRESS(KA2000_UART0_BASE) + KA2000_URFC)
#define UART_PUT_FCR(p, c)      __raw_writel((c), IO_ADDRESS(KA2000_UART0_BASE) + KA2000_URFC)

#define UART_GET_MSR(p)         __raw_readl(IO_ADDRESS(KA2000_UART0_BASE) + KA2000_URMS)

#define UART_GET_LSR(p)         __raw_readl(IO_ADDRESS(KA2000_UART0_BASE) + KA2000_URLS)

#define UART_GET_LCR(p)         __raw_readl(IO_ADDRESS(KA2000_UART0_BASE) + KA2000_URLC)
#define UART_PUT_LCR(p, c)      __raw_writel((c), IO_ADDRESS(KA2000_UART0_BASE) + KA2000_URLC)

#define UART_GET_MCR(p)         __raw_readl(IO_ADDRESS(KA2000_UART0_BASE) + KA2000_URMC)
#define UART_PUT_MCR(p, c)      __raw_writel((c), IO_ADDRESS(KA2000_UART0_BASE) + KA2000_URMC)

//for setting baud rate
#define UART_GET_BRDR(p)        __raw_readl(IO_ADDRESS(KA2000_UART0_BASE) + KA2000_URBD)
#define UART_PUT_BRDR(p, c)     __raw_writel((c), IO_ADDRESS(KA2000_UART0_BASE) + KA2000_URBD)


#define UART_GET_INTR(p)         __raw_readl(IO_ADDRESS(KA2000_UART0_BASE) + KA2000_URINTR)
#define UART_PUT_INTR(p, c)      __raw_writel((c), IO_ADDRESS(KA2000_UART0_BASE) + KA2000_URINTR)

u32 tx_rx = 0;
static struct uart_driver *ka2000_get_driver();

static inline int tx_enabled(struct uart_port *port)
{
    return port->unused[0] & 1;
}
static inline void tx_enable(struct uart_port *port, int enabled)
{
    if(enabled)
        port->unused[0] |= 1;
    else
        port->unused[0] &= ~1;
}
static inline int rx_enabled(struct uart_port *port)
{
    return port->unused[0] & 2;
}
static inline void rx_enable(struct uart_port *port, int enabled)
{
    if(enabled)
        port->unused[0] |= 2;
    else
        port->unused[0] &= ~2;
}
static inline int ms_enabled(struct uart_port *port)
{
    return port->unused[0] & 4;
}

static inline void ms_enable(struct uart_port *port, int enabled)
{
    if(enabled)
        port->unused[0] |= 4;
    else
        port->unused[0] &= ~4;
}


/*
 * Handle any change of modem status signal since we were last called.
 */
static void ka2000uart_mctrl_check(struct uart_port *port)
{
    unsigned int status, changed;

    status = port->ops->get_mctrl(port);
    changed = status ^ old_status;

    if (changed == 0)
        return;

    old_status = status;

    if (changed & TIOCM_RI)
        port->icount.rng++;
    if (changed & TIOCM_DSR)
        port->icount.dsr++;
    if (changed & TIOCM_CAR)
        uart_handle_dcd_change(port, status & TIOCM_CAR);
    if (changed & TIOCM_CTS)
        uart_handle_cts_change(port, status & TIOCM_CTS);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
    wake_up_interruptible(&port->state->port.delta_msr_wait);
#else
    wake_up_interruptible(&port->info->delta_msr_wait);
#endif
}


static unsigned int ka2000uart_tx_empty(struct uart_port *port)
{
    unsigned long flags;
    unsigned int status;
    spin_lock_irqsave(&port->lock, flags);
    status= (UART_GET_LSR(port) & URLS_URTE) ? TIOCSER_TEMT : 0;
    spin_unlock_irqrestore(&port->lock, flags);
    return status;
}

static void ka2000uart_stop_tx(struct uart_port *port)
{
    if (tx_enabled(port))
    {
        tx_enable(port, 0);
        //cancel_delayed_work(&tx_work);
    }
}
#if 1
#define UART_LSR_TEMT		(1<<6) /* Transmitter empty */
 /* URLS_URTHRE (1<<5) : Transmit-hold-register empty */
#define BOTH_EMPTY (UART_LSR_TEMT | URLS_URTHRE)

static void wait_for_xmitr(struct uart_port *port)
{
    unsigned int status;
    int i;

    for (i = 0; i < 10000; i++)
    {
        status = UART_GET_LSR(port);  // Read Line Status Register
        if ((status & URLS_URTHRE) == URLS_URTHRE)
            return;
        //if ((status & BOTH_EMPTY) == BOTH_EMPTY)
        //    return;
        //barrier();
    }
}
#else
static void wait_for_xmitr(struct uart_port *port)
{
    volatile int time_out = 10000;
    while (!(UART_GET_LSR(port) & URLS_URTHRE)) // != URLS_URTHRE)
    {
        //barrier();
        if (time_out-- < 0)
            break;
    }
}
#endif

static void serial_out(struct uart_port *port, const char ch)
{

    wait_for_xmitr(port);
    UART_PUT_CHAR(port, ch);
}

static int transmit_chars(struct uart_port *port)
{
    static int busy;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
    struct circ_buf *xmit = &port->state->xmit;
#else
    struct circ_buf *xmit = &port->info->xmit;
#endif

    int count;

    if (busy)
        port->x_char = '^';

    busy = 1;
    if (port->x_char)
    {
        serial_out(port, port->x_char);
        port->icount.tx++;
        port->x_char = 0;
        busy = 0;
        return 1;
    }
    if (uart_circ_empty(xmit) || uart_tx_stopped(port))
    {
        ka2000uart_stop_tx(port);
        busy = 0;
        return 0;
    }

    count = 256; //xmit->tail - xmit->head;
    do
    {
        serial_out(port, xmit->buf[xmit->tail]);
        xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
        port->icount.tx++;
        if (uart_circ_empty(xmit))
            break;
    }
    while (--count > 0);

    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);

    if (uart_circ_empty(xmit))
        ka2000uart_stop_tx(port);

    busy = 0;
    return 1;
}

static int idle = 0;
static void ka2000_next_tx(struct work_struct *work)
{
    struct uart_port *port;
    //struct circ_buf *xmit = &port->state->xmit;
    unsigned long flags;
    int ret = 0;
    port = (struct uart_port *)ka2000_get_port();
    spin_lock_irqsave(&port->lock, flags);
    //disable_irq(port->irq);
    //if (tx_rx == 0)
    {
        tx_rx |= 1;
        ret = transmit_chars(port);
        tx_rx &= ~1;
    }

    //enable_irq(port->irq);

    spin_unlock_irqrestore(&port->lock, flags);

    if (tx_enabled(port))
    {
        queue_delayed_work(tx_workqueue, &tx_work, 30);
    }
}

static void ka2000uart_start_tx(struct uart_port *port)
{
    //ka2000_next_tx((unsigned long)port);
    if (!tx_enabled(port))
    {
        tx_enable(port, 1);
        ka2000_next_tx((unsigned long)port);
    }
}

static void ka2000uart_stop_rx(struct uart_port *port)
{
    if (rx_enabled(port))
    {
        disable_irq(port->irq);
        rx_enable(port, 0);
    }
}

static void ka2000uart_break_ctl(struct uart_port *port, int break_state)
{
    unsigned int lcr;

    lcr = UART_GET_LCR(port);

    if (break_state == -1)
        lcr |= URLC_URSBC;
    else
        lcr &= ~URLC_URSBC;

    UART_PUT_LCR(port, lcr);
}

static irqreturn_t ka2000uart_irq(int irq, void *dev_id)
{
    struct uart_port *port = dev_id;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
    struct tty_struct *tty = port->state->port.tty;
#else
    struct tty_struct *tty = port->info->port.tty;
#endif

    unsigned int status, ch, lsr, flg, max_count = 1;
    unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

#if 0
    if (tx_rx == 0)
    {
    tx_rx |= 2;
#endif

    //disable_irq(port->irq);
    //ka2000_dbg("^");
    //ka2000_set_gpio(0);
    status = UART_GET_LSR(port);            /* clears pending LSR interrupts */
    while ((status & URLS_URDR) && max_count--)
    {
        if (tx_rx & 1)
            break;
        //ka2000_dbg("@");
        // ka2000_set_gpio(0xf);
        ch = UART_GET_CHAR(port);
        flg = TTY_NORMAL;

        port->icount.rx++;

        /*
         * Note that the error handling code is
         * out of the main execution path
         */
        lsr = UART_GET_LSR(port);
        if (unlikely(lsr & (URLS_URBI | URLS_URPE | URLS_URFE | URLS_URROE)))
        {
            if (lsr & URLS_URBI)
            {
                lsr &= ~(URLS_URFE | URLS_URPE);
                port->icount.brk++;
                if (uart_handle_break(port))
                    goto ignore_char;
            }
            if (lsr & URLS_URPE)
                port->icount.parity++;
            if (lsr & URLS_URFE)
                port->icount.frame++;
            if (lsr & URLS_URROE)
                port->icount.overrun++;

            lsr &= port->read_status_mask;

            if (lsr & URLS_URBI)
                flg = TTY_BREAK;
            else if (lsr & URLS_URPE)
                flg = TTY_PARITY;
            else if (lsr & URLS_URFE)
                flg = TTY_FRAME;
        }

        if (uart_handle_sysrq_char(port, ch))
            goto ignore_char;

        uart_insert_char(port, lsr, URLS_URROE, ch, flg);

ignore_char:
        status = UART_GET_LSR(port);
    }
	tty_flip_buffer_push(tty);

#if 0
    }
	tx_rx &= ~2;
#endif

    //enable_irq(port->irq);
    spin_unlock_irqrestore(&port->lock, flags);
    return IRQ_HANDLED;
}

static irqreturn_t ka2000uart_irq_1(int irq, void *dev_id)
{
    struct uart_port *port = dev_id;
    unsigned int status, ch, flg;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
    struct tty_struct *tty = port->state->port.tty;
#else
    struct tty_struct *tty = port->info->port.tty;
#endif
    unsigned long flags;

    spin_lock_irqsave(&port->lock, flags);

    UART_PUT_INTR(port,0x00);               /* clear interrupts*/

    status = UART_GET_LSR(port);
    ch = UART_GET_CHAR(port);
    //ka2000_set_gpio(ch);
    flg = TTY_NORMAL;
    port->icount.rx++;
#if 1
    /*
     * Note that the error handling code is
     * out of the main execution path
     */
    if (unlikely(status & (URLS_URBI | URLS_URPE | URLS_URFE | URLS_URROE)))
    {
        if (status & URLS_URBI)
        {
            status &= ~(URLS_URFE | URLS_URPE);
            flg = TTY_BREAK;
            port->icount.brk++;
        }
        if (status & URLS_URPE)
        {
            port->icount.parity++;
            flg = TTY_PARITY;
        }
        if (status & URLS_URFE)
        {
            port->icount.frame++;
            flg = TTY_FRAME;
        }
        if (status & URLS_URROE)
            port->icount.overrun++;
    }
#endif

    uart_insert_char(port, status, 0, ch, flg);
    tty_flip_buffer_push(tty);
    UART_PUT_INTR(port, 0x01);               /* re-enable rx interrupts*/

    spin_unlock_irqrestore(&port->lock, flags);
    return IRQ_HANDLED;
}

static int ka2000uart_startup(struct uart_port *port)
{
    int retval;
    unsigned long flags;

    spin_lock_irqsave(&port->lock, flags);

    tx_enable(port, 0);
    rx_enable(port, 1);
    ms_enable(port, 1);

    //ka2000_dbg("fcr vale %x\n",UART_GET_FCR(port));
    /*
     * Allocate the IRQ
     */
    retval = request_irq(port->irq, ka2000uart_irq, 0, "UART RX", port);

    if (retval)
        goto err_rx;
	
	UART_PUT_FCR(port, URFC_URFRT_1 | URFC_URTFR | URFC_URRFR | URFC_URFE);
	barrier();
    //open rx intr in KA2000uart
    UART_PUT_INTR(port, 0x01);
	
    spin_unlock_irqrestore(&port->lock, flags);
    return 0;

err_rx:
    free_irq(port->irq, port);
    spin_unlock_irqrestore(&port->lock, flags);
    return retval;
}

static void ka2000uart_shutdown(struct uart_port *port)
{
    /*
     * Free the interrupt for tx
     */
    free_irq(port->irq, port);

    /* disable break condition and fifos */
    UART_PUT_LCR(port, UART_GET_LCR(port) & ~URLC_URSBC);
}

static void	ka2000uart_enable_ms(struct uart_port *port)
{
    ms_enable(port,1);
    //mod_timer(&port_timer, jiffies+10);

}
static void ka2000uart_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
    unsigned int lcr, fcr = 0;
    unsigned long flags;
    unsigned int baud, quot;

    spin_lock_irqsave(&port->lock, flags);
    /*
     * Ask the core to calculate the divisor for us.
     */
    baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
    //ka2000_dbg("uart2000_set_baud %x\n",baud);
    quot = uart_get_divisor(port, baud);
    //ka2000_dbg("uart2000_set_quot %x\n",quot);
    lcr  = URCL_8;

    /* stop bits */
    lcr |= URLC_URSB;

    /* none parity */

    /* fcr control */
    if (port->fifosize > 1)
        fcr = URFC_URFRT_1;
#ifdef PORT_TIMER_ENABLED
    del_timer_sync(&port_timer);
#endif
    //printk("uart2000_c_flag %x\n",termios->c_iflag);
    /*
     * Update the per-port timeout.
     */
    uart_update_timeout(port, termios->c_cflag, baud);
    port->read_status_mask = URLS_URROE;
    if (termios->c_iflag & INPCK)
        port->read_status_mask |= (URLS_URFE | URLS_URPE);
    if (termios->c_iflag & (BRKINT | PARMRK))
        port->read_status_mask |= URLS_URBI;

    /*
     * Characters to ignore
     */
    port->ignore_status_mask = 0;
    if (termios->c_iflag & IGNPAR)
        port->ignore_status_mask |= (URLS_URFE | URLS_URPE);
    if (termios->c_iflag & IGNBRK)
    {
        port->ignore_status_mask |= URLS_URBI;
        /*
         * If we're ignoring parity and break indicators,
         * ignore overruns too (for real raw support).
         */
        if (termios->c_iflag & IGNPAR)
            port->ignore_status_mask |= URLS_URROE;
    }

    /*
     * Ignore all characters if CREAD is not set.
     */
    if ((termios->c_cflag & CREAD) == 0)
        port->ignore_status_mask |= UART_DUMMY_LSR_RX;

    if (UART_ENABLE_MS(port, termios->c_cflag))
        ka2000uart_enable_ms(port);


    UART_PUT_LCR(port, lcr);
    UART_PUT_FCR(port, fcr);

    if (tty_termios_baud_rate(termios))
        tty_termios_encode_baud_rate(termios, baud, baud);

    spin_unlock_irqrestore(&port->lock, flags);
}

static const char *ka2000uart_type(struct uart_port *port)
{
    return port->type == PORT_KA2000 ? "KA2000" : NULL;
}
/*
 * Release the memory region(s) being used by 'port'
 */
static void ka2000uart_release_port(struct uart_port *port)
{
    release_mem_region(port->mapbase, KA2000_PORT_SIZE);
}
/*
 * Request the memory region(s) being used by 'port'
 */
static int ka2000uart_request_port(struct uart_port *port)
{
    return request_mem_region(port->mapbase, KA2000_PORT_SIZE,
                              "serial_ka2000") != NULL ? 0 : -EBUSY;
}
/*
 * Configure/autoconfigure the port.
 */
static void ka2000uart_config_port(struct uart_port *port, int flags)
{
    if (flags & UART_CONFIG_TYPE)
    {
        port->type = PORT_KA2000;
        ka2000uart_request_port(port);
    }
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int ka2000uart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
    int ret = 0;
#if 0
    if (ser->type != PORT_KA2000)
        ret = -EINVAL;
    if (ser->irq != port->irq)
        ret = -EINVAL;
    if (ser->baud_base < 9600)
        ret = -EINVAL;
#endif
    return ret;
}


/* Modem Control Register */
#define URMC_URLB       (1 << 4)        /* Loop-back mode */
#define URMC_UROUT2     (1 << 3)        /* OUT2 signal */
#define URMC_UROUT1     (1 << 2)        /* OUT1 signal */
#define URMC_URRTS      (1 << 1)        /* Request to Send */
#define URMC_URDTR      (1 << 0)        /* Data Terminal Ready */

/* Modem Status Register */
#define URMS_URTERI 	(1 << 7)		/* Trailing Edge Ring Indicator */
#define URMS_URDCD      (1 << 6)        /* Data Carrier Detect */
#define URMS_URDSR      (1 << 5)        /* Data Set Ready */
#define URMS_URCTS      (1 << 4)        /*?????  Clear to Send, not sure */
#define URMS_URRI       (1 << 3)        /* Ring Indicator */
#define URMS_URDDCD 	(1 << 2)		/* Delta Data Carrier Detect */
#define URMS_URDCTS     (1 << 1)        /* Delta Clear to Send */
#define URMS_URDDST 	(1 << 0)		/* Delta Data Set Ready */


static void ka2000uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
#if 0
    unsigned int mcr;

    mcr = UART_GET_MCR(port);
    if (mctrl & TIOCM_RTS)
        mcr |= URMC_URRTS;
    else
        mcr &= ~URMC_URRTS;

    if (mctrl & TIOCM_DTR)
        mcr |= URMC_URDTR;
    else
        mcr &= ~URMC_URDTR;

    UART_PUT_MCR(port, mcr);
#endif
}

static unsigned int	ka2000uart_get_mctrl(struct uart_port *port)
{
#if 1
    /* Not implemented */
    return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
#else
    unsigned int result = 0;
    unsigned int status;

    status = UART_GET_MSR(port);
    if (status & URMS_URDCD)
        result |= TIOCM_CAR;
    if (status & URMS_URDSR)
        result |= TIOCM_DSR;
    if (status & URMS_URCTS)
        result |= TIOCM_CTS;
    if (status & URMS_URRI)
        result |= TIOCM_RI;

    return result;
#endif
}

/*
 * This is our per-port timeout handler, for checking the
 * modem status signals.
 */

#define MCTRL_TIMEOUT	(250*HZ/1000)
#ifdef PORT_TIMER_ENABLED
static void ka2000uart_timeout(unsigned long data)
{
    struct uart_port *port = (struct uart_port *)data;
    unsigned long flags;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,28)
    if (port->state)
#else
    if (port->info)
#endif
    {
        spin_lock_irqsave(&port->lock, flags);
        ka2000uart_mctrl_check(port);
        spin_unlock_irqrestore(&port->lock, flags);
        //ka2000uart_irq(0, port);

        //mod_timer(&port_timer, jiffies + MCTRL_TIMEOUT);
    }
}
#endif


static struct uart_ops ka2000uart_pops =
{
    .tx_empty       = ka2000uart_tx_empty,
    .stop_tx        = ka2000uart_stop_tx,
    .start_tx       = ka2000uart_start_tx,
    .stop_rx        = ka2000uart_stop_rx,
    .break_ctl      = ka2000uart_break_ctl,
    .startup        = ka2000uart_startup,
    .shutdown       = ka2000uart_shutdown,
    .set_termios    = ka2000uart_set_termios,
    .type           = ka2000uart_type,
    .release_port   = ka2000uart_release_port,
    .request_port   = ka2000uart_request_port,
    .config_port    = ka2000uart_config_port,
    .verify_port    = ka2000uart_verify_port,
    .set_mctrl      = ka2000uart_set_mctrl,

    .get_mctrl	= ka2000uart_get_mctrl,
    .enable_ms	= ka2000uart_enable_ms,


};
static struct uart_port ka2000uart_ports[SERIAL_KA2000_NR] =
{
    {
        .iobase         = (unsigned long) KA2000_UART0_BASE,
        .membase        = (char *)IO_ADDRESS(KA2000_UART0_BASE),
        .mapbase        = (unsigned long)KA2000_UART0_BASE,
        .iotype         = UPIO_MEM,
        .irq            = IRQ_UARTINT0,
        .uartclk        = KA2000_PLL_CLOCK/HCLK_RATE,
        .fifosize       = 256, //14,
        .ops            = &ka2000uart_pops,
        .flags          = UPF_BOOT_AUTOCONF | UPF_AUTO_IRQ,
        .line           = 0,
    }
};


static struct uart_port *ka2000_get_port()
{
    return &ka2000uart_ports[0];
}

static void ka2000_console_putchar(struct uart_port *port, int ch)
{
    unsigned int status;
    int i;

    for (i = 0; i < 10000; i++)
    {
        status = UART_GET_LSR(port);  // Read Line Status Register
        barrier();
        if ((status & URLS_URTHRE))
            break;
    }
    UART_PUT_CHAR(port, ch);
}

static void ka2000_console_write(struct console *co, const char *s, u_int count)
{
    struct uart_port *port = &ka2000uart_ports[0];
    uart_console_write(port, s, count, ka2000_console_putchar);
}


static void __init ka2000_console_get_options(struct uart_port *port, int *baud, int *parity, int *bits)
{
    unsigned int lcr;

    lcr = UART_GET_LCR(port);
    *parity = 'n';
    *bits = 8;
    *baud = port->uartclk / (UART_GET_BRDR(port) & 0x0FFF);
    *baud /= 16;
    *baud &= 0xFFFFFFF0;
}

static int __init ka2000_console_setup(struct console *co, char *options)
{
    struct uart_port *port;
    int baud = UART_BAUD_RATE;
    int bits = 8;
    int parity = 'n';
    int flow = 'n';

    /*
     * Check whether an invalid uart number has been specified, and
     * if so, search for the first available port that does have
     * console support.
     */
    port = uart_get_console(&ka2000uart_ports[0], SERIAL_KA2000_NR, co);

    if (options)
        uart_parse_options(options, &baud, &parity, &bits, &flow);
    else
        ka2000_console_get_options(port, &baud, &parity, &bits);

    return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver ka2000_reg;

static struct console ka2000_console =
{
    .name           = SERIAL_KA2000_DEVNAME,
    .write          = ka2000_console_write,
    .device         = uart_console_device,
    .setup          = ka2000_console_setup,
    .flags          = CON_PRINTBUFFER,
    .index          = -1,
    .data           = &ka2000_reg,
};

static int __init ka2000_console_init(void)
{
    //printk("ka2000_reg_console \n");
    register_console(&ka2000_console);
    return 0;
}

console_initcall(ka2000_console_init);
#define KA2000_CONSOLE  &ka2000_console

static struct uart_driver ka2000_reg =
{
    .owner                  = THIS_MODULE,
    .driver_name            = "serial_ka2000",
    .dev_name               = SERIAL_KA2000_DEVNAME,
    .major                  = TTY_MAJOR,
    .minor                  = SERIAL_KA2000_MINOR,
    .nr                     = SERIAL_KA2000_NR,
    .cons                   = KA2000_CONSOLE,
};

static struct uart_driver *ka2000_get_driver()
{
    return &ka2000_reg;
}

static int __init ka2000uart_init(void)
{
    int ret;
    struct clk *clk_p;
    //printk("Serial: KA2000 UART 16550 driver\n");

    ret = uart_register_driver(&ka2000_reg);
    if (ret)
        return ret;

    clk_p = clk_get(NULL, "UART");
    ka2000uart_ports[0].uartclk = clk_get_rate(clk_p);
    //printk("uart_add_one_port %x %x, %d\n", (unsigned long)&ka2000_reg, (unsigned long)&ka2000uart_ports[0], ka2000uart_ports[0].uartclk);


    uart_add_one_port(&ka2000_reg, &ka2000uart_ports[0]);
#ifdef PORT_TIMER_ENABLED
    init_timer(&port_timer);
    port_timer.function = ka2000uart_timeout;
    port_timer.data     = (unsigned long)&ka2000uart_ports[0];
#endif
	tx_workqueue = create_workqueue("uart tx");
    INIT_DELAYED_WORK(&tx_work, ka2000_next_tx);
    //init_timer(&tx_timer);
    //tx_timer.function = ka2000_next_tx;
    //tx_timer.data     = (unsigned long)&ka2000uart_ports[0];

    //printk("ka2000uart init ok\n");
    return 0;
}

static void __exit ka2000uart_exit(void)
{
    int i;
    /*
     * Stop our timer.
     */
    //del_timer_sync(&tx_timer);
    cancel_delayed_work_sync(&tx_work);
#ifdef PORT_TIMER_ENABLED
    del_timer_sync(&port_timer);
#endif
    for (i = 0; i < SERIAL_KA2000_NR; i++)
        uart_remove_one_port(&ka2000_reg, &ka2000uart_ports[0]);
    uart_unregister_driver(&ka2000_reg);
}

module_init(ka2000uart_init);
module_exit(ka2000uart_exit);

MODULE_DESCRIPTION("KA2000 serial port driver");
MODULE_AUTHOR("KeyASIC Inc.");
MODULE_LICENSE("GPL");
#endif

