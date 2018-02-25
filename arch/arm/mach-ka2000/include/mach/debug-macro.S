/*
 * Debugging macro for KA2000
 */

/*
		.macro addruart, rx
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx, #0x01000000	@ physical base address
		movne	\rx, #0xfe000000	@ virtual base
		orr	\rx, \rx, #0x00c20000   @ UART 0
		.endm
#define UART_SHIFT	2
*/

		.macro addruart, rx
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx, #0xA0000000	@ physical base address
		movne	\rx, #0xF5000000	@ virtual base
		add	\rx, \rx, #0x00004000   @ UART 0
		.endm

#define UART_SHIFT	0


#include <asm/hardware/debug-8250.S>
