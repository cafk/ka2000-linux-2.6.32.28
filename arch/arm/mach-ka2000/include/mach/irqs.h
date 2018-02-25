/*
 * KeyASIC Ka2000 interrupt controller definitions
 *
 *  Copyright (C) 2010 KeyASIC.
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
#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

/* Base address */

#define KA2000_ARM_INTC_BASE 0xa0006000

/*add by kathy, this is for Trek2K */
#define IRQ_WDT0                           0
#define IRQ_UARTINT0                       1   //modify by kathy
#define IRQ_SSI                            2
#define IRQ_PWM0                           3
#define IRQ_DMA                            4
#define IRQ_EXT_INT                        5
#define IRQ_IIC                            6
#define IRQ_GPI0_0                         8
#define IRQ_GPI0_1                         9
#define IRQ_GPI0_2                         10
#define IRQ_GPI0_3                         11
#define IRQ_GPI0_4                         12
#define IRQ_GPI0_5                         13
#define IRQ_GPI0_6                         14
#define IRQ_GPI0_7                         15
#define IRQ_GPI1_0                         16
#define IRQ_GPI1_1                         17
#define IRQ_GPI1_2                         18
#define IRQ_GPI1_3                         19
#define IRQ_GPI1_4                         20
#define IRQ_GPI1_5                         21
#define IRQ_PWM_T1                         22
#define IRQ_PWM_T2                         23
#define IRQ_sdm_buf_tran_finish            24
#define IRQ_sdm_data_bound_int             25
#define IRQ_sdm_tran_done_int              26
#define IRQ_sdm_cmd_done_int               27
#define IRQ_sdm_card_error_int             28
#define IRQ_sdm_dma_int                    29


#define IRQ_sdio_buf_tran_finish_int       32
#define IRQ_sdio_data_bound_int            33
#define IRQ_sdio_tran_done_int             34
#define IRQ_sdio_cmd_done_int              35
#define IRQ_sdio_card_error_int            36
#define IRQ_sdio_dma_int                   37
#define IRQ_card_int                       38

#define IRQ_switch_int0                    40
#define IRQ_switch_int1                    41
#define IRQ_switch_int2                    42
#define IRQ_switch_int3                    43
#define IRQ_switch_int4                    44
#define IRQ_switch_int5                    45
#define IRQ_switch_int6                    46
#define IRQ_switch_int7                    47
#define IRQ_switch_int8                    48
#define IRQ_switch_int_b2_normal           50
#define IRQ_switch_int_b2_sleep            51
#define IRQ_switch_int12                   52
#define IRQ_switch_int13                   53

/*-----------------------*/

#define IRQ_VDINT0       0
#define IRQ_VDINT1       40
#define IRQ_VDINT2       2
#define IRQ_HISTINT      3
#define IRQ_H3AINT       4
#define IRQ_PRVUINT      5
#define IRQ_RSZINT       6
#define IRQ_VFOCINT      7
#define IRQ_VENCINT      48
#define IRQ_ASQINT       49
#define IRQ_IMXINT       50
#define IRQ_VLCDINT      51
#define IRQ_USBINT       52
#define IRQ_EMACINT      53



#define IRQ_CCINT0       16
#define IRQ_CCERRINT     17
#define IRQ_TCERRINT0    18
#define IRQ_TCERRINT     19
#define IRQ_PSCIN        20

#define IRQ_IDE          32    //modify by kathy
#define IRQ_HPIINT       23
#define IRQ_MBXINT       24
#define IRQ_MBRINT       25
#define IRQ_MMCINT       26
#define IRQ_SDIOINT      27
#define IRQ_MSINT        28
#define IRQ_DDRINT       29
#define IRQ_AEMIFINT     30
#define IRQ_VLQINT       31
#define IRQ_TINT0_TINT12 22    //modify by kathy
#define IRQ_TINT0_TINT34 33
#define IRQ_TINT1_TINT12 34
#define IRQ_TINT1_TINT34 35
#define IRQ_PWMINT0      36
#define IRQ_PWMINT1      37
#define IRQ_PWMINT2      38
#define IRQ_I2C          39
#define IRQ_UARTINT1     41
#define IRQ_UARTINT2     42
#define IRQ_SPINT0       43
#define IRQ_SPINT1       44

#define IRQ_DSP2ARM0     46
#define IRQ_DSP2ARM1     47



#define IRQ_GPIO0        8
#define IRQ_GPIO1        9
#define IRQ_GPIO2        10
#define IRQ_GPIO3        11
#define IRQ_GPIO4        12
#define IRQ_GPIO5        13
#define IRQ_GPIO6        14
#define IRQ_GPIO7        15




#define IRQ_GPIOBNK0     56
#define IRQ_GPIOBNK1     57
#define IRQ_GPIOBNK2     58
#define IRQ_GPIOBNK3     59
#define IRQ_GPIOBNK4     60
#define IRQ_COMMTX       61
#define IRQ_COMMRX       62
#define IRQ_EMUINT       63

#define KA2000_N_AINTC_IRQ	64
#define KA2000_N_GPIO		1

#define NR_IRQS		        64 //modify by kathy
//#define NR_IRQS			(KA2000_N_AINTC_IRQ + KA2000_N_GPIO)   //mark by kathy

/*
#define KA2000_ARM_INTC_BASE 0x01C48000
#define IRQ_VDINT0       0
#define IRQ_VDINT1       1
#define IRQ_VDINT2       2
#define IRQ_HISTINT      3
#define IRQ_H3AINT       4
#define IRQ_PRVUINT      5
#define IRQ_RSZINT       6
#define IRQ_VFOCINT      7
#define IRQ_VENCINT      8
#define IRQ_ASQINT       9
#define IRQ_IMXINT       10
#define IRQ_VLCDINT      11
#define IRQ_USBINT       12
#define IRQ_EMACINT      13

#define IRQ_CCINT0       16
#define IRQ_CCERRINT     17
#define IRQ_TCERRINT0    18
#define IRQ_TCERRINT     19
#define IRQ_PSCIN        20

#define IRQ_IDE          22
#define IRQ_HPIINT       23
#define IRQ_MBXINT       24
#define IRQ_MBRINT       25
#define IRQ_MMCINT       26
#define IRQ_SDIOINT      27
#define IRQ_MSINT        28
#define IRQ_DDRINT       29
#define IRQ_AEMIFINT     30
#define IRQ_VLQINT       31
#define IRQ_TINT0_TINT12 32
#define IRQ_TINT0_TINT34 33
#define IRQ_TINT1_TINT12 34
#define IRQ_TINT1_TINT34 35
#define IRQ_PWMINT0      36
#define IRQ_PWMINT1      37
#define IRQ_PWMINT2      38
#define IRQ_I2C          39
#define IRQ_UARTINT0     40
#define IRQ_UARTINT1     41
#define IRQ_UARTINT2     42
#define IRQ_SPINT0       43
#define IRQ_SPINT1       44

#define IRQ_DSP2ARM0     46
#define IRQ_DSP2ARM1     47
#define IRQ_GPIO0        48
#define IRQ_GPIO1        49
#define IRQ_GPIO2        50
#define IRQ_GPIO3        51
#define IRQ_GPIO4        52
#define IRQ_GPIO5        53
#define IRQ_GPIO6        54
#define IRQ_GPIO7        55
#define IRQ_GPIOBNK0     56
#define IRQ_GPIOBNK1     57
#define IRQ_GPIOBNK2     58
#define IRQ_GPIOBNK3     59
#define IRQ_GPIOBNK4     60
#define IRQ_COMMTX       61
#define IRQ_COMMRX       62
#define IRQ_EMUINT       63

#define KA2000_N_AINTC_IRQ	64
#define KA2000_N_GPIO		71

#define NR_IRQS			(KA2000_N_AINTC_IRQ + KA2000_N_GPIO)
*/

#define ARCH_TIMER_IRQ IRQ_PWM_T1 //IRQ_PWM0

#endif /* __ASM_ARCH_IRQS_H */
