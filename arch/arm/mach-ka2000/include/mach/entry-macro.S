/*
 * Low-level IRQ helper macros for KeyASIC KA2000-based platforms
 *
 */
#include <mach/io.h>
#include <mach/irqs.h>

		.macro	disable_fiq
		.endm

		.macro  get_irqnr_preamble, base, tmp
		ldr \base, =IO_ADDRESS(KA2000_ARM_INTC_BASE)
		.endm

		.macro  arch_ret_to_user, tmp1, tmp2
		.endm

		.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
		ldr \irqstat, [\base, #0x18]     @ interrupt pending 1
		mov \tmp, #0
		mov \irqnr, #0
		teq \irqstat, #0
		beq 6f

4:              teq \irqstat, #1
                beq 7f

5:		mov \irqstat, \irqstat, lsr #1
		teq \irqstat, #1
		addne \irqnr, #1
		bne 5b

		add \irqnr, #1
                teq \irqstat, #0                  @irqstat=1
		bne 7f

6:		ldr \irqstat, [\base, #0x1c]      @ interrupt pending 2
		cmp \irqstat, #0
		movne \tmp, #32
		bne 4b

7:		add \irqnr, \tmp
		.endm

		.macro	irq_prio_table
		.endm
		

				
