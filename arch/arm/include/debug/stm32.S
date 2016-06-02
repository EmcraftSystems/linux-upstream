/*
 * Copyright (C) 2015
 * Yuri Tikhonov, Emcraft Systems, <yur@emcraft.com>
 *
 * Debugging macro include header
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#if defined(CONFIG_ARCH_STM32F7)
# define STM32_USART_SR		(0x1C)		/* Interrupt&Status Register */
# define STM32_USART_DR		(0x28)		/* Transmit Data Register */
#else
# define STM32_USART_SR		(0x00)		/* Status Register */
# define STM32_USART_DR		(0x04)		/* Data Register */
#endif

#define STM32_USART_SR_TXE	(1 << 7)	/* Transmitter Empty */

	.macro	addruart, rp, rv, tmp
	ldr	\rp, =CONFIG_DEBUG_UART_PHYS	@ phys address
	ldr	\rv, =CONFIG_DEBUG_UART_VIRT	@ virt address
	.endm

	.macro	senduart,rd,rx
	strb	\rd, [\rx, #(STM32_USART_DR)]	@ Write to Data Register
	.endm

	.macro	waituart,rd,rx
1001:	ldr	\rd, [\rx, #(STM32_USART_SR)]	@ Read Status Register
	tst	\rd, #STM32_USART_SR_TXE	@ TXE = 1 when TDR shifted
	beq	1001b				@ branch if TXE = 0
	.endm

	.macro	busyuart,rd,rx
1001:	ldr	\rd, [\rx, #(STM32_USART_SR)]	@ Read Status Register
	tst	\rd, #STM32_USART_SR_TXE	@ TXE = 0 when TDR has data
	beq	1001b				@ branch if TXE = 0
	.endm
