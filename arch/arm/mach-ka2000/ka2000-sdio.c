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
#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/leds.h>
#include <linux/mmc/host.h>
#include <linux/init.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>

#include <mach/ka2000.h>
#include <mach/ka2000_define.h>
#include "ka2000-sdio.h"
#define IRQ_ENABLED 0
#define BURST_SIZE_R 0x30	// 0x3f,0x33 : incr 16, 0x2f,0x23 : incr 16,  0x1f,0x13 : incr 4
#define BURST_SIZE_W 0x30	// 0x3f,0x33 : incr 16, 0x2f,0x23 : incr 16,  0x1f,0x13 : incr 4
#define SINGLE_BLOCK_READ_DMA 1
#define SINGLE_BLOCK_WRITE_DMA 1

#define MAX_BLOCK_COUNT 16 

//#define DEBUG_BLOCKS 0
//#define DEBUG_REGS 1


#define sdio_word_write(a,v) 	__raw_writel(v, IO_ADDRESS(a))
#define sdio_word_read(a)  		__raw_readl(IO_ADDRESS(a))

#define byte_write(a,v) 	__raw_writeb(v, IO_ADDRESS(a))
#define byte_read(a)  		__raw_readb(IO_ADDRESS(a))

#define UARTC(ch)	 __raw_writeb(ch, IO_ADDRESS(0xa0004000))

void trace_t1(char *msg, int line) {}
#define trace_w trace_t1  
#define trace_r trace_t1  
#define trace_rw trace_t1 //trace_t

#define FIFO_DONE		0x1
#define DMA_DONE		(3<<0)
#define DATA_BOUND		0x8
#define TRANS_DONE		0x2
#define TRANS_CARD_ERR  0x10
#define TRANS_CARD_INT  0x20

#define TRAN_READ  		(0<<1)
#define TRAN_WRITE 		(1<<1)
#define TRAN_START 		(1<<2)

//#define Tran_Start		(1<<2)
#define SDIO_ERR 		(1<<4)

#define IRQMASK 1
#define IRQUNMASK 0

#define ERR_CMD_CRC_END_IND	(CMD_CRC_ERR | CMD_END_ERR | CMD_IND_ERR)
#define ERR_ENABLE_ALL (0x6F)

#define SDIO_SOFT_RESET (1 << 6)
#define SDIO_HARD_RESET (1 << 7)


#if 0
//------------------------------------------------------------------------------
static void dump_sdio_regs()
{

    printk("\n------------------------- Dump All Registers -------------------\n");
    printk("SDIO_CARD_BLOCK_SET_REG %08X\n", sdio_word_read(SDIO_CARD_BLOCK_SET_REG));
    printk("SDIO_CTRL_REG           %08X\n", sdio_word_read(SDIO_CTRL_REG));
    printk("SDIO_CMD_ARGUMENT_REG   %08X\n", sdio_word_read(SDIO_CMD_ARGUMENT_REG));
    printk("SDIO_SPECIAL_CTRL_REG   %08X\n", sdio_word_read(SDIO_SPECIAL_CTRL_REG));
    printk("SDIO_STATUS_REG         %08X\n", sdio_word_read(SDIO_STATUS_REG));
    printk("SDIO_ERROR_ENABLE_REG   %08X\n\n", sdio_word_read(SDIO_ERROR_ENABLE_REG));
    printk("SDIO_RESPONSE1_REG   %08X\n", sdio_word_read(SDIO_RESPONSE1_REG));
    printk("SDIO_RESPONSE2_REG   %08X\n", sdio_word_read(SDIO_RESPONSE2_REG));
    printk("SDIO_RESPONSE3_REG   %08X\n", sdio_word_read(SDIO_RESPONSE3_REG));
    printk("SDIO_RESPONSE4_REG   %08X\n\n", sdio_word_read(SDIO_RESPONSE4_REG));
    printk("SDIO_BUF_TRAN_RESP_REG  %08X\n", sdio_word_read(SDIO_BUF_TRAN_RESP_REG));
    printk("SDIO_BUF_TRAN_CTRL_REG  %08X\n\n", sdio_word_read(SDIO_BUF_TRAN_CTRL_REG));

    printk("SDIO_DMA_TCCH1_REG  %08X\n", sdio_word_read(SDIO_DMA_TCCH1_REG));
    printk("SDIO_DMA_DACH1_REG  %08X\n", sdio_word_read(SDIO_DMA_DACH1_REG));
    printk("SDIO_DMA_CTRCH1_REG %08X\n", sdio_word_read(SDIO_DMA_CTRCH1_REG));
    printk("SDIO_DMA_INTS_REG   %08X\n", sdio_word_read(SDIO_DMA_INTS_REG));

    printk("----------------------------------------------------------------\n");

}
#endif

//-------------------------------------------------------------------------------------------------------------------------------
static void _debug_regs_(char *msg, int t)
{
#ifdef DEBUG_REGS
    u32 reg0 = SDIO_BUF_TRAN_RESP_REG;
    u32 reg1 = SDIO_DMA_INTS_REG;
    u32 reg2 = SDIO_STATUS_REG;
    static u32 a[512];
    static u32 b[512];
    static u32 c[512];
    int i;
    
    if (t > 512)
        t = 512;

    a[0] = sdio_word_read(reg0);
    b[0] = sdio_word_read(reg1);
    c[0] = sdio_word_read(reg2);
    for (i = 1; i < t; i++)
    {
        a[i] = sdio_word_read(reg0);
        b[i] = sdio_word_read(reg1);
        c[i] = sdio_word_read(reg2);
        if (a[i] != a[i-1] || b[i] != b[i-1])
            break;
    }

    printk("%s %x,%x, %x,%x, %x,%x\n", msg, a[0], sdio_word_read(reg0), b[0], sdio_word_read(reg1), c[0], sdio_word_read(reg2));
#endif    
}

static inline void _debug_regs_level_(int level, char *msg, int t)
{
#ifdef DEBUG_REGS
    if (level & DEBUG_REGS)
        _debug_regs_(msg, t);
#endif
}


//-------------------------------------------------------------------------------------------------
static inline void _sdio_disable_cmd52_abort_cmd53_(void)
{
    sdio_word_write(SDIO_SPECIAL_COMMAND_REG, 0x00);
}

static inline void _sdio_resume_next_block_rw_(void)
{
    sdio_word_write(SDIO_SPECIAL_COMMAND_REG, 0x02);
}

static void sdio_clear_dma_regs(void)
{
    sdio_word_write(SDIO_BUF_TRAN_RESP_REG, 0x1f); //clear all flags
    sdio_word_write(SDIO_DMA_INTS_REG, 1);                         //clear interrupts
    sdio_word_write(SDIO_DMA_INTS_REG, 2);
    sdio_word_write(SDIO_BUF_TRAN_CTRL_REG,  0);
    sdio_word_write(SDIO_DMA_CTRCH0_REG, 0);
    sdio_word_write(SDIO_DMA_CTRCH1_REG, 0); 
}

static void sdio_check_error(struct ka_sdio_host *host, struct mmc_request *mrq)
{
    u32 err_states = sdio_word_read(SDIO_STATUS_REG);

    if (mrq->cmd != NULL) {
        if (err_states & CMD_TIME_ERR ) {
            mrq->cmd->error = -ETIMEDOUT;
            printk("CMD_TIMEOUT_ERR\n");

        }
        else if (err_states & ERR_CMD_CRC_END_IND) {
            mrq->cmd->error = -EILSEQ;
            printk("CMD_CRC_END_IND_ERR\n");
        }
    }

    if (mrq->cmd->data != NULL) {
        if (err_states & (DATA_CRC_ERR | DATA_END_ERR)) {
            mrq->cmd->data->error = -EILSEQ;
            printk("DAT_CRC_END_ERR\n");
        }

        if (mrq->cmd->data->error) {
            //ka_sdio_data_done(host);
        }
    }
}//sdio_check_error

static inline int sdio_check_resp(u32 reg, u32 mask, int timeout, int clean, int line)
{
    int timeoutcount = 0;

    while(1) {
        barrier();

        if( (sdio_word_read(reg) & mask) ) {
            barrier();
            if(clean) {
                  sdio_word_write( reg , mask );
            }
            return 0;
        }

        if( timeoutcount++ > timeout ) {
           if (timeout > 100)
                printk("%x:%x->%x timeout L%d\n", reg, mask, sdio_word_read(reg), line);
            return -1;
        }
    }
    return -1;
}//sdio_check_resp

//-------------------------------------------------------------------------------------------------
static int sdio_wait_trans_done(int line)
{
    int err = 0;
    err = sdio_check_resp(SDIO_BUF_TRAN_RESP_REG, TRANS_DONE, 100000, 1, line);

    /* check if transfer error */
    if( (sdio_word_read(SDIO_BUF_TRAN_RESP_REG) & TRANS_CARD_ERR) != 0x0) {               
        /* write to clear transfer error  */
        sdio_word_write(SDIO_BUF_TRAN_RESP_REG, TRANS_CARD_ERR);
        /* check if end bit error */
        if (sdio_word_read(SDIO_STATUS_REG) & 0x200000)    {
            //printk("endbit err!\n");  /* ignore end bit error */
            sdio_word_write(SDIO_STATUS_REG, sdio_word_read(SDIO_STATUS_REG) & 0xfffff);            
        } else {
            printk("sdio trans err!");
            return 1;
        }    
    }

    return err;
}

static inline int sdio_wait_bus_ready(void)
{
    return sdio_check_resp(SDIO_STATUS_REG, 0x0c00, 3000, 0, __LINE__);  
}

static inline int  sdio_wait_bus_idle(int timeout)
{
    /* wait until bus busy */
    return sdio_check_resp( SDIO_STATUS_REG, 0x0400, timeout, 0, __LINE__);
}

static void sdio_reset_controller(u32 flag)
{
    sdio_clear_dma_regs();
    sdio_word_write( SDIO_ERROR_ENABLE_REG, ERR_ENABLE_ALL );
}
//------------------------------------------------------------------------------
static void sdio_enable_gpio0_power_on_sdio_wifi(void)
{
    sdio_word_write(0xa0005008, 3);  /* power on */
    null_delay(2);barrier();
    sdio_word_write(0xa0005000, 3);  /* set gpio output */
    null_delay(2);barrier();
    sdio_word_write(0xa0005008, 1);  /* reset */
    null_delay(2);barrier();
    sdio_word_write(0xa0005008, 3);  /* power on */
    null_delay(2);barrier();
}
//------------------------------------------------------------------------------
static void sdio_disable_gpio0_power_off_sdio_wifi(void)
{
    printk("sdio_disable_gpio0_power_off_sdio_wifi\n");
    sdio_word_write(0xa0005008, 0);  /* power off */
    null_delay(2);barrier();
    sdio_word_write(0xa0005000, 3);  /* set gpio output */
    null_delay(2);barrier();
    sdio_word_write(0xa0005008, 0);  /* power off */
    null_delay(2);barrier();
}
//-------------------------------------------------------------------------------------------------------------------------------
void sdio_set_clock(void)
{
#define SDIO_CLOCK_PLL            (1 << 8)
#define SDIO_CLOCK_ENABLE        (1 << 9)
#define SDIO_CLOCK_B            (1 << 10)
#define SDIO_CLOCK (SDIO_CLOCK_PLL | SDIO_CLOCK_ENABLE | SDIO_CLOCK_B)
     sdio_word_write(SCU_CLK_SRC_CTL, sdio_word_read(SCU_CLK_SRC_CTL) | SDIO_CLOCK); // should defined at clock.c
}

static void sdio_enable_all_error_flags(void)
{
    //setup SDIO error enable reg
    sdio_word_write(SDIO_ERROR_ENABLE_REG, 0x6f);
}
//-------------------------------------------------------------------------------------------------------------------------------

static inline void _set_block_size_(unsigned int blocks, unsigned int blksz)
{
    u32 block_set;
    
    if (blksz != 512)
        block_set = ( (blksz<<20) | blocks );
    else
        block_set = ( (1<<16) | blocks );
    
    sdio_word_write( SDIO_CARD_BLOCK_SET_REG , block_set );
}    

//-------------------------------------------------------------------------------------------------------------------------------
static inline void _clear_read_dma_flag_(void)
{
    sdio_word_write(SDIO_DMA_INTS_REG, 2);
}

static inline void _trigger_read_dma_(void)
{
    sdio_word_write(SDIO_DMA_CTRCH1_REG, 0x0f | BURST_SIZE_R);       // 0x3f,0x33 : incr 16, 0x2f,0x23 : incr 16,  0x1f,0x13 : incr 4
    sdio_word_write(SDIO_DMA_CTRCH1_REG, 0x03 | BURST_SIZE_R);       
}

static inline void _trigger_read_transfer_from_bus_to_fifo_(unsigned int blocks)
{
    sdio_word_write(SDIO_BUF_TRAN_CTRL_REG, ((blocks << 16) | TRAN_START));
}

static inline void _set_read_dma_size_(unsigned int size)
{
    sdio_word_write(SDIO_DMA_TCCH1_REG, size);         /* Transfer Count */
}

static inline void _set_read_dma_address_(unsigned int addr)
{
    sdio_word_write(SDIO_DMA_DACH1_REG, addr);         /* Start Addr, DMA_WADDR */
}

//-------------------------------------------------------------------------------------------------------------------------------
/* 

1. set dma addr and size 
2. start transfer from bus to fifo 
3. start dma 

(2) and (3) don't have to care about sequence

*/
static inline void _start_read_dma_(u32 addr, int blocks, int blksz)
{
    _clear_read_dma_flag_();
    _set_read_dma_address_(addr);    
    _set_read_dma_size_(blocks * blksz);    
    _trigger_read_transfer_from_bus_to_fifo_(blocks);
    _trigger_read_dma_();    
}

//-------------------------------------------------------------------------------------------------------------------------------
static inline void _clear_write_dma_flag_(void)
{
    sdio_word_write(SDIO_DMA_INTS_REG, 1);
}

static inline void _trigger_write_dma_(void)
{
    sdio_word_write( SDIO_DMA_CTRCH0_REG, 0x0f | BURST_SIZE_W);     
    sdio_word_write( SDIO_DMA_CTRCH0_REG, 0x03 | BURST_SIZE_W);
}

static inline void _trigger_write_transfer_from_fifo_to_bus_(unsigned int blocks)
{
    sdio_word_write(SDIO_BUF_TRAN_CTRL_REG , (blocks << 16) | TRAN_WRITE | TRAN_START);
}

static inline void _set_write_dma_size_(unsigned int size)
{
    sdio_word_write(SDIO_DMA_TCCH0_REG, size);         /* Transfer Count */
}

static inline void _set_write_dma_address_(unsigned int addr)
{
    sdio_word_write(SDIO_DMA_SACH0_REG, addr);         /* Start Addr, DMA_WADDR */
}

//-------------------------------------------------------------------------------------------------------------------------------

static inline void _start_write_dma_(u32 addr, int blocks, int blksz)
{
    _clear_write_dma_flag_();
    _set_write_dma_address_(addr);
    _set_write_dma_size_(blocks * blksz);    
    _trigger_write_dma_();    
    _trigger_write_transfer_from_fifo_to_bus_(blocks);
}

//-------------------------------------------------------------------------------------------------------------------------------

static inline void sdio_start_write_dma(u32 addr, int blocks, int blksz, int blk_index)
{
#ifdef SINGLE_BLOCK_WRITE_DMA
    _start_write_dma_(addr + (blk_index * blksz), 1, blksz);
#else
    if (blk_index == 0)
    {
        _start_write_dma_(addr, blocks, blksz);
    }
#endif        
}
//-------------------------------------------------------------------------------------------------------------------------------

static inline void sdio_start_read_dma(u32 addr, int blocks, int blksz, int blk_index)
{
#ifdef SINGLE_BLOCK_READ_DMA
    _start_read_dma_(addr + (blk_index * blksz), 1, blksz);
#else
    if (blk_index == 0)
    {
        _start_read_dma_(addr, blocks, blksz);
    }
#endif        
}

//-------------------------------------------------------------------------------------------------------------------------------
static int sdio_start_data_write(struct mmc_request *mrq, struct mmc_data *data)
{
    u32 addr = sg_dma_address(data->sg);        
    int err = 0;
    /* start data dma */
    sdio_start_write_dma(addr, data->blocks, data->blksz, 0);

    if (data->blocks == 1)
    {
        err |= sdio_check_resp(SDIO_DMA_INTS_REG, 1, 10000, 1, __LINE__);         
    }
        
    err |= sdio_check_resp(SDIO_BUF_TRAN_RESP_REG, FIFO_DONE, 50000, 1, __LINE__);    

    return err;
}

//-------------------------------------------------------------------------------------------------------------------------------
static int sdio_wait_single_block_write(void)
{
    int err = sdio_wait_trans_done(__LINE__);

    /* wait until bus not busy */
    sdio_wait_bus_idle(100); //sdio_check_resp( SDIO_STATUS_REG, 0x0400, 100, 0, __LINE__);
    return err;
}

//-------------------------------------------------------------------------------------------------------------------------------
static int sdio_multi_block_write(struct mmc_data *data)
{
    u32 addr = sg_dma_address(data->sg);
    int i;
    int err = 0;

    /* before send cmd53, there is 1 block dma done, so i = 1 */
    for (i = 1; i < data->blocks; i++)
    {
        /* start current block (i) dma */
        sdio_start_write_dma(addr, data->blocks, data->blksz, i);
        /* wait for current block dma done */
        err |= sdio_check_resp(SDIO_DMA_INTS_REG, 0x1, 1000, 1, __LINE__); 
        /* wait for data write to bus */
        err |= sdio_check_resp(SDIO_BUF_TRAN_RESP_REG, FIFO_DONE, 1000, 1, __LINE__);
        
        /* wait for previous block (i - 1) data transfer from bus */
        err |= sdio_check_resp(SDIO_BUF_TRAN_RESP_REG, DATA_BOUND, 30000, 1, __LINE__);        

        /* resume bus transfer of current block (i) */
        _sdio_resume_next_block_rw_();
    }

    /* latest block wait for data transfer to sdio bus */
    err |= sdio_wait_trans_done(__LINE__);

    /* wait until bus busy */
    sdio_wait_bus_idle(10000); //sdio_check_resp( SDIO_STATUS_REG, 0x0400, 10000, 0, __LINE__);
    return err;
}

//-------------------------------------------------------------------------------------------------------------------------------
/* 1 block : trans done -> dma start -> fifo done -> dma done */
static int sdio_single_block_read(struct ka_sdio_host *host, struct mmc_request *mrq, struct mmc_data *data)
{
    int err = 0;
    
    _debug_regs_level_(1, "1r0", 200);
    
    /* wait for data transfer from sdio bus */
    err |= sdio_wait_trans_done(__LINE__);
    
    /* wait until bus not busy */
    //err |= sdio_wait_bus_idle(1000);

    /* start current block dma */
    sdio_start_read_dma(sg_dma_address(data->sg), 1, data->blksz, 0);
    
    _debug_regs_level_(2, "1rd", 100);

    /* wait for current block data move to fifo */
    err |= sdio_check_resp(SDIO_BUF_TRAN_RESP_REG, FIFO_DONE, 1000, 1, __LINE__);

    /* wait for current block dma done */
    err |= sdio_check_resp(SDIO_DMA_INTS_REG, 0x2, 1000, 1, __LINE__); 
    
    return err;
}    

//-------------------------------------------------------------------------------------------------------------------------------
/* n blocks : data bound 0 -> start dma 0 -> trans done 0 -> fifo done 0 -> data bound 1 -> start dma 1 -> trans done 1 -> fifo done 1 */
static int sdio_multi_block_read(struct ka_sdio_host *host, struct mmc_request *mrq, struct mmc_data *data)
{
    int i;
    u32 addr = sg_dma_address(data->sg);
    int err = 0;    
    
    _debug_regs_level_(4, "nr0", 100);
    
    for (i = 0; i < data->blocks; i++)
    {        
        _debug_regs_level_(8, "nr1", 100);
        if (i < data->blocks - 1)
        {
            /* wait for current block data transfer from bus */
            err |= sdio_check_resp(SDIO_BUF_TRAN_RESP_REG, DATA_BOUND, 30000, 1, __LINE__);    
        }
        else
        {
            /* latest block is different flow, no data bound, only transfer done */
            err |= sdio_wait_trans_done(__LINE__);                
        }    
        
        /* resume next block move from bus to fifo */
        _sdio_resume_next_block_rw_();        
        
        /* start current block dma */
        sdio_start_read_dma(addr, data->blocks, data->blksz, i);    
        
        _debug_regs_level_(16, "nr2", 100);
        
        /* wait for current block dma done */
        err |= sdio_check_resp(SDIO_BUF_TRAN_RESP_REG, FIFO_DONE, 50000, 1, __LINE__);
        err |= sdio_check_resp(SDIO_DMA_INTS_REG, 2, 1000, 1, __LINE__);            
    }

    /* wait until bus not busy */
    err |= sdio_wait_bus_idle(1000); //sdio_check_resp( SDIO_STATUS_REG, 0x0400, 1000, 0, __LINE__);
    return err;
}

//-------------------------------------------------------------------------------------------------------------------------------
static inline void sdio_prepare_data_dma(struct ka_sdio_host *host, struct mmc_request *mrq, struct mmc_data *data)
{
    int sg_cnt;

    _set_block_size_(data->blocks, data->blksz);
    
    BUG_ON(data->sg_len > 1);
    
    sg_cnt = dma_map_sg(mmc_dev(host->mmc),data->sg,data->sg_len,
                        (data->flags & MMC_DATA_READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
}

//-------------------------------------------------------------------------------------------------------------------------------
static inline int sdio_data_write(struct ka_sdio_host *host, struct mmc_request *mrq, struct mmc_command *cmd, struct mmc_data *data)
{             
    if (data->blocks == 1) {
        return sdio_wait_single_block_write();
    }
    
       return sdio_multi_block_write(data);
}

//-------------------------------------------------------------------------------------------------------------------------------
static inline int sdio_data_read(struct ka_sdio_host *host, struct mmc_request *mrq, struct mmc_command *cmd, struct mmc_data *data)
{             
    if (data->blocks == 1)   {
        return sdio_single_block_read(host, mrq, data);
    }
    
       return sdio_multi_block_read(host, mrq, data);
}

//-------------------------------------------------------------------------------------------------------------------------------
static inline void sdio_data_done(struct ka_sdio_host *host, struct mmc_request *mrq)
{
    struct mmc_data *data = mrq->data;
    
    if (data->sg) {
        dma_unmap_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
            (data->flags & MMC_DATA_READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
    }

    if (data->error)
        data->bytes_xfered = 0;
    else
        data->bytes_xfered = data->blksz * data->blocks;
}


//-------------------------------------------------------------------------------------------------
static inline int sdio_wait_command_done(unsigned int cmd)
{    
    int err = 0;

    /* wait for command done flag */
    err = sdio_check_resp(SDIO_BUF_TRAN_RESP_REG, CMD_DONE, 10000, 1, __LINE__);
    
    /* check if error happened */
    if((sdio_word_read(SDIO_BUF_TRAN_RESP_REG) & TRANS_CARD_ERR ) != 0)  {
        barrier();
        
        /* clear error flag, write to clear */
        sdio_word_write(SDIO_BUF_TRAN_RESP_REG, TRANS_CARD_ERR);

        printk("cmd %d error\n", cmd);
        if (cmd != 8)
            err = 1;
    }

    return err;
}

static inline void sdio_store_command_response(struct mmc_command *cmd)
{
    if (cmd->flags & MMC_RSP_PRESENT)
    {
        cmd->resp[0] = sdio_word_read(SDIO_RESPONSE1_REG);
    }
}

static inline u32 sdio_get_command_flag(struct ka_sdio_host *host, struct mmc_command *cmd)
{
    u32 ctrl_set = 0;
    ctrl_set |= (cmd->opcode << 16);

    if (!(cmd->flags & MMC_RSP_PRESENT))
        ctrl_set |= (0x0<<10);
    else
        ctrl_set |= (0x2<<10);

    if (cmd->flags & MMC_RSP_CRC)
        ctrl_set |= (1<<13);
    if (cmd->flags & MMC_RSP_OPCODE)
        ctrl_set |= (1<<12);

    if (cmd->data != NULL)
    {
        if ( cmd->data->flags & MMC_DATA_READ )
            ctrl_set |= (1<<8); /* bit 8 - 1: read data, 0:write data */

        ctrl_set |= (1<<14);    /* bit14 - 1: with data */
    }

#if IRQ_ENABLED
    ctrl_set |= (host->sdio_card_int_flag << 25);
#endif
    ctrl_set |= host->bus_width_4_flag;
    ctrl_set |= (1<<4);         /* bit 4 - 1:set force clock */
    
    return ctrl_set;    
}

static inline int sdio_send_command(struct ka_sdio_host *host, struct mmc_request *mrq)
{
    u32 ctrl_set = 0;
    struct mmc_command *cmd;
    
    cmd = mrq->cmd;

    if ((cmd->flags & MMC_RSP_136) && (cmd->flags & MMC_RSP_BUSY))
    {
        printk(KERN_ERR "%s: Unsupported response type!\n", mmc_hostname(host->mmc));
        cmd->error = -EINVAL;
        return -EINVAL;
    }

    ctrl_set = sdio_get_command_flag(host, cmd); 
    
    sdio_word_write(SDIO_CMD_ARGUMENT_REG, cmd->arg);
    sdio_word_write(SDIO_CTRL_REG, ctrl_set);
    
    _debug_regs_level_(0, "cmd", 200);
    
    if (sdio_wait_command_done(cmd->opcode))
        sdio_check_error(host, mrq);
    
    sdio_store_command_response(cmd);
    return 0;
}


//------------------------------------------------------------------------------

static void _debug_print_blocks_(struct mmc_request *mrq)
{
#ifdef DEBUG_BLOCKS 
    if (mrq->cmd->data && mrq->cmd->data->blocks > DEBUG_BLOCKS)
    {
        printk("%s %d %d\n", (mrq->cmd->data->flags & MMC_DATA_WRITE) ? "W" : "R", mrq->cmd->data->blocks, mrq->cmd->data->blksz);
    }
#endif    
}

//------------------------------------------------------------------------------

static inline void sdio_request_with_data(struct ka_sdio_host *host, struct mmc_request *mrq)
{
    _debug_print_blocks_(mrq);
    
    /* set dma map */
    sdio_prepare_data_dma(host, mrq, mrq->cmd->data);
    
    //local_irq_disable();
    if (mrq->cmd->data->flags & MMC_DATA_WRITE) {            
        /* write 1 block -> send command -> write the rest blocks */
        sdio_start_data_write(mrq, mrq->cmd->data);         
        sdio_send_command(host, mrq);            
        sdio_data_write(host, mrq, mrq->cmd, mrq->cmd->data);            
    } else {        
        /* send read command -> read all blocks */
        sdio_send_command(host, mrq);                    
        sdio_data_read(host, mrq, mrq->cmd, mrq->cmd->data);                
    }
    //local_irq_enable();
    
    /* set dma unmap */
    sdio_data_done(host, mrq);

}
//------------------------------------------------------------------------------

/* sdio request entry function */
static void ka_sdio_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
    struct ka_sdio_host *host = mmc_priv(mmc);
    
    WARN_ON(host->mrq != NULL); /* re-entry */
    spin_lock(&host->lock);
    
    host->mrq = mrq;
    
    sdio_wait_bus_ready();    
    sdio_clear_dma_regs();

    _sdio_disable_cmd52_abort_cmd53_(); 

    /* deal with request */
    if (mrq->cmd->data) {
        sdio_request_with_data(host, mrq);
    } else {
        /* command only, no data */
        sdio_send_command(host, mrq);
    }
    
    /* check if any error at last */
    if (sdio_word_read(SDIO_STATUS_REG) & 0xc0000)
    {
        sdio_check_error(host, mrq);
    }

    /* check if error, do sdio reset */
    if (mrq->cmd->error || (mrq->data && mrq->data->error)) {
        sdio_reset_controller(SDIO_SOFT_RESET);
    }
    
    sdio_clear_dma_regs();
    
    if (host->mrq == mrq)
        host->mrq = NULL;

    mmc_request_done(mmc, mrq);
    spin_unlock(&host->lock);
}


//------------------------------------------------------------------------------
static void ka_sdio_set_clock(struct ka_sdio_host *host, unsigned int clock)
{
    if (clock == 0)
        return;

    if (host->clock != clock)
        clk_set_rate(host->clk_io, clock);
    host->clock = clock;
}
//------------------------------------------------------------------------------
static void ka_sdio_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
    struct ka_sdio_host *host = (struct ka_sdio_host *)mmc_priv(mmc);

    if (host->flags & SDHCI_DEVICE_DEAD)
        return;
    
    //printk("ka_sdio_set_ios bus_wd %d, clock %d\n", ios->bus_width, ios->clock);

    ka_sdio_set_clock(host, ios->clock);
    if (ios && ios->bus_width == MMC_BUS_WIDTH_4)
        host->bus_width_4_flag = 1;
    else
        host->bus_width_4_flag = 0;
    
}
//------------------------------------------------------------------------------

#if IRQ_ENABLED
extern int sdio_irq_wakeup(struct mmc_host *mmc);

static irqreturn_t ka_sdio_card_int_irq(int irq, void *dev_id)
{
    struct ka_sdio_host *host = (struct ka_sdio_host *)dev_id;

    if(sdio_word_read(SDIO_BUF_TRAN_RESP_REG) & TRANS_CARD_INT)
    {
        sdio_word_write(SDIO_BUF_TRAN_RESP_REG, TRANS_CARD_INT);
        if (host->sdio_card_int_flag && host && host->mmc)
        {
            sdio_irq_wakeup(host->mmc);
            host->sdio_card_int_flag = 0;
        }
    }

    return IRQ_HANDLED;
}
#endif
//------------------------------------------------------------------------------

static void ka_sdio_enable_irq(struct mmc_host *mmc, int enable)
{
    struct ka_sdio_host *host;
    
    //printk("ka_sdio_enable_sdio_irq (%s)\n", enable ? "enable" : "disable");
    host = (struct ka_sdio_host *)mmc_priv(mmc);
    host->sdio_card_int_flag = enable;

}
//------------------------------------------------------------------------------

static const struct mmc_host_ops ka_sdio_mmc_ops =
{
    .request    = ka_sdio_request,
    .set_ios    = ka_sdio_set_ios,
    .enable_sdio_irq = ka_sdio_enable_irq,
};


//------------------------------------------------------------------------------
// allocate/free/add/remove host
//------------------------------------------------------------------------------
struct ka_sdio_host *ka_sdio_alloc_host(struct device *dev)
{
    struct mmc_host *mmc;
    struct ka_sdio_host *host;

    WARN_ON(dev == NULL);

    /* mmc private area first item will be ka_sdio_host */
    mmc = mmc_alloc_host(sizeof(struct ka_sdio_host), dev);
    if (!mmc)
        return ERR_PTR(-ENOMEM);

    host = mmc_priv(mmc);
    
    host->mmc = mmc;
    host->mmc->sdio_type = 1;  //bit0: sdio, bit 1: switch;
    
    spin_lock_init(&host->lock);
    return host;
}
//------------------------------------------------------------------------------
void ka_sdio_free_host(struct ka_sdio_host *host)
{
    mmc_free_host(host->mmc);
}
//------------------------------------------------------------------------------
int ka_sdio_add_host(struct ka_sdio_host *host)
{
    struct mmc_host *mmc;

    WARN_ON(host == NULL);
    if (host == NULL)
        return -EINVAL;

    mmc = host->mmc;

    mmc_add_host(mmc);
#if 0
    printk(KERN_INFO "%s: SDHCI controller on %s [%s] using %s%s\n",
           mmc_hostname(mmc), host->hw_name, dev_name(mmc_dev(mmc)),
           (host->flags & SDHCI_USE_ADMA)?"A":"",
           (host->flags & SDHCI_USE_DMA)?"DMA":"PIO");
#endif
    return 0;
}
//------------------------------------------------------------------------------

void ka_sdio_remove_host(struct ka_sdio_host *host, int dead)
{
    if (dead) {
        host->flags |= SDHCI_DEVICE_DEAD;

        if (host->mrq) {
            printk(KERN_ERR "%s: Controller removed during "
                   " transfer!\n", mmc_hostname(host->mmc));

            host->mrq->cmd->error = -ENOMEDIUM;            
            sdio_reset_controller(SDIO_SOFT_RESET);                
            host->mrq = NULL;
        }
    }

    mmc_remove_host(host->mmc);
}
//------------------------------------------------------------------------------

static int sdio_init_host_clock_settings(struct ka_sdio_host *host, struct device *dev)
{
    int ret = 0;
    host->clk_io = clk_get(dev, "SDIOCLK");
    if (IS_ERR(host->clk_io))
    {
        dev_err(dev, "failed to get io clock\n");
        ret = PTR_ERR(host->clk_io);
        return ret;
    }

    /* enable the local io clock and keep it running for the moment. */
    clk_enable(host->clk_io);

    host->clock = 12000000;    
    
#if KA2000_MHZ == 12
    host->max_clk = KA2000_OSC_CLOCK;
#else
    host->max_clk = 24000000;  //50000000; //12000000;
#endif
    host->timeout_clk = host->max_clk / 1000; //1000000;
    return ret;
}    
//------------------------------------------------------------------------------
static void sdio_init_host_mmc_settings(struct ka_sdio_host *host)
{
    host->mmc->f_min = KA2000_OSC_CLOCK/2; //host->max_clk / 256;
    host->mmc->f_max = host->max_clk;
    if (host->mmc->f_min > host->mmc->f_max)
        host->mmc->f_min = host->mmc->f_max;

    host->mmc->ops    = &ka_sdio_mmc_ops;
    
    host->mmc->caps =  MMC_CAP_4_BIT_DATA | MMC_CAP_NONREMOVABLE;
    host->mmc->ocr_avail = MMC_VDD_33_34 ;
    host->mmc->ios.bus_width = MMC_BUS_WIDTH_4;

    host->mmc->sdio_type = 1;

    host->mmc->max_hw_segs = 1;
    host->mmc->max_phys_segs    = 1; 
    
    host->mmc->max_req_size     = MAX_BLOCK_COUNT * 512; 
    host->mmc->max_seg_size     = host->mmc->max_req_size;
    host->mmc->max_blk_size     = 512;
    host->mmc->max_blk_count    = MAX_BLOCK_COUNT; 

}
//------------------------------------------------------------------------------

static int sdio_init_irq_settings(struct ka_sdio_host *host, struct device *dev)
{
    int ret = 0;
#if IRQ_ENABLED
    host->mmc->caps |= MMC_CAP_SDIO_IRQ;
    ret = request_irq(IRQ_card_int, &ka_sdio_card_int_irq, IRQF_DISABLED, "ka_sdio", host);
    if (ret < 0)
    {
        dev_err(dev, "request irq error\n");
    }
    printk("irq requested\n");
#endif
    return ret;
}

//------------------------------------------------------------------------------
static int __devinit ka_sdio_probe(struct platform_device *pdev)
{
    struct ka_sdio_platdata *pdata = pdev->dev.platform_data;
    struct device *dev = &pdev->dev;
    struct ka_sdio_host *host;
    int ret = 0;

    if ( pdata == NULL )
    {
        dev_err(dev, "no device data specified\n");
        return -ENOENT;
    }

    host = ka_sdio_alloc_host(dev);
    if (IS_ERR(host))
    {
        dev_err(dev, "ka_sdio_alloc_host() failed\n");
        return PTR_ERR(host);
    }

    platform_set_drvdata(pdev, host);

    host->pdev     = pdev;
    host->pdata = pdata;

    if (sdio_init_host_clock_settings(host, dev))
        goto err_io_clk;
    
    sdio_init_host_mmc_settings(host);
    sdio_init_irq_settings(host, dev);        
    sdio_enable_all_error_flags();
    sdio_enable_gpio0_power_on_sdio_wifi();

    /* add host to mmc stack and do mmc_add_host */
    ret = ka_sdio_add_host(host);
    if (ret)
    {
        dev_err(dev, "ka_sdio_add_host() failed\n");
        goto err_add_host;
    }    

    return 0;

err_add_host:
    if (host->clk_io)
    {
        clk_disable(host->clk_io);
        clk_put(host->clk_io);
    }
    
err_io_clk:
    ka_sdio_free_host(host);
    return ret;
}

static int __devexit ka_sdio_remove(struct platform_device *pdev)
{
    struct ka_sdio_host *host =  platform_get_drvdata(pdev);

    int dead = 0;
    int timeout = 100000;
    while (host->mrq != NULL && timeout--)
    {
        schedule();
    }
    
    ka_sdio_remove_host(host, dead);
    ka_sdio_free_host(host);

    sdio_disable_gpio0_power_off_sdio_wifi();
    platform_set_drvdata(pdev, NULL);

    return 0;
}
//------------------------------------------------------------------------------

static struct platform_driver ka_sdio_driver =
{
    .probe        = ka_sdio_probe,
    .remove        = __devexit_p(ka_sdio_remove),
    .suspend    = NULL,
    .resume     = NULL,
    .driver        = {
        .owner        = THIS_MODULE,
        .name        = "KeyAsic_sdhci", 
    },
};
//------------------------------------------------------------------------------
/* defined at arm/mach-ka2000/clock.c */
extern void ka2000_clk_wakeup(void);
extern void ka2000_sdio_on(void);
extern void ka2000_sdio_off(void);

static int __init ka_sdio_init(void)
{
#ifdef CONFIG_KA2000_POWER_SLEEP_ENABLE
    ka2000_clk_wakeup();
    ka2000_sdio_on();
#endif
    return platform_driver_register(&ka_sdio_driver);
}
//------------------------------------------------------------------------------

static void __exit ka_sdio_exit(void)
{
    sdio_disable_gpio0_power_off_sdio_wifi();
    platform_driver_unregister(&ka_sdio_driver);
    ka2000_sdio_off();
}
//------------------------------------------------------------------------------

module_init(ka_sdio_init);
module_exit(ka_sdio_exit);

MODULE_DESCRIPTION("KeyASIC SDIO");
MODULE_AUTHOR("KeyASIC");
MODULE_LICENSE("GPL v2");
