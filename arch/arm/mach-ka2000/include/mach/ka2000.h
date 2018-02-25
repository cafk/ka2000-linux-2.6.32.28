#ifndef KA2000_H
#define KA2000_H

#define _KA2000_DEBUG_ 0

#include <linux/kernel.h>

#ifndef CONFIG_KA2000_12M_FPGA_TEST
#define OSC_MHZ   24
#if CONFIG_KA2000_CHIP_VERSION == 0xA
#define HCLK_RATE   4
#else
#define HCLK_RATE   2
#endif
#define KA2000_OSC_CLOCK (24000000)
#define UART_BAUD_RATE 38400
#define KA2000_MHZ 192 //96  /* default value only, the real clock will get from 0xa000000c at clock init */

#else
#define OSC_MHZ   12
#define HCLK_RATE  1
#define KA2000_OSC_CLOCK (12000000)
#define UART_BAUD_RATE 19200
#define KA2000_MHZ 12  /* default value only, the real clock will get from 0xa000000c at clock init */

#endif


//#define KA2000_PLL_CLOCK 12000000
#define KA2000_PLL_CLOCK ((KA2000_MHZ) * 1000000) /* default value only, the real clock will get from 0xa000000c at clock init */

#if _KA2000_DEBUG_
void ka2000_putstr(const char *ptr);
void ka2000_dbg(const char *fmt, ...);
#define T(); printk("%s line=%d\n", __FUNCTION__, __LINE__);
#endif

//#define KA2000_DISABLE_PRINTK
#ifdef KA2000_DISABLE_PRINTK
#define KA_DBG(...); {}
#define printk dprintk
static inline int dprintk(const char *fmt, ...)
{
      return 0;
}
#else
#define KA_DBG printk
#endif

//------------------------------------------------------------------------------
#define ka2000_set_gpio(value); { *(u32 *)(0xf5005008) = value; }
#define KA_UARTC(ch);	*(volatile u32 __force *) (0x55000000 + 0xa0004000) = (ch);


//------------------------------------------------------------------------------
static inline void null_delay(int t)
{
    int i,j;
#if KA2000_MHZ==12
    t /= 2;
#endif
    for (i = 0; i < t; i++)
    {
        for (j = 0; j < 5; j++)
            nop();
    }
}
//------------------------------------------------------------------------------
//extern int ka_tick;
//#define ka2000_get_ticks() (ka_tick+ka2000_readl(0xa0002000+TCNTO2))
void ka2000_dma_flush_all(void);
void ka2000_dma_flush_range(const void *start, const void *end);
//unsigned long ka2000_clk_get_rate(void);

#define TRACE_T_ENABLED 0

#if TRACE_T_ENABLED
//void trace_t_reset(void);
void trace_t(char *msg, int line);
void trace_t_dump(void);
#else
//#define trace_t_reset(); {}
#define trace_t(msg, line); {}
#define trace_t_dump(); {}
#endif

unsigned int get_ka_tick(void);

#define CHECK_M1_IN_WQ 1

extern int kcard_check_m1_cmd_flags(void);
extern int kcard_check_m1_special_flags(void);

typedef int (*kcard_call_back_t)(void);
void kcard_set_call_back(kcard_call_back_t handler);

#endif
