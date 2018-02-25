
#include <linux/types.h>
#include <linux/serial_reg.h>
#include <mach/serial.h>

/* PORT_16C550A, in polled non-fifo mode */

static void putc(char c)
{

	volatile u32 *uart = (volatile void *) KA2000_UART0_BASE;
    int timeout = 10000;

	while (!(uart[UART_LSR] & UART_LSR_THRE))
    {
        if (timeout-- == 0)
            break;
        barrier();
    }

	uart[UART_TX] = c;

}

static inline void flush(void)
{
	volatile u32 *uart = (volatile void *) KA2000_UART0_BASE;
    int timeout = 10000;
	while (!(uart[UART_LSR] & UART_LSR_THRE))
    {
        if (timeout-- == 0)
            break;
        barrier();
    }
}

#define arch_decomp_setup()
#define arch_decomp_wdog()
