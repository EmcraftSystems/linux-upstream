/*
 * Copyright 2018 EmCraft Systems www.emcraft.com
 * Author: Alexander Dyachenko <sasha_d@emcraft.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define KINETIS_UART0_BASE_ADDR		0x4006A000
#define KINETIS_UART1_BASE_ADDR		0x4006B000
#define KINETIS_UART2_BASE_ADDR		0x4006C000
#define KINETIS_UART3_BASE_ADDR		0x4006D000
#define KINETIS_UART4_BASE_ADDR		0x400EA000
#define KINETIS_UART_BASE_ADDR(n)	KINETIS_UART##n##_BASE_ADDR
#define KINETIS_UART_BASE(n)		KINETIS_UART_BASE_ADDR(n)
#define KINETIS_UART_PHYSICAL_BASE	KINETIS_UART_BASE(CONFIG_DEBUG_KINETIS_UART_PORT)

#define KINETIS_UART_VIRTUAL_BASE	KINETIS_UART_PHYSICAL_BASE

	.macro	addruart, rp, rv, tmp
	ldr	\rp, =KINETIS_UART_PHYSICAL_BASE 	@ physical
	ldr	\rv, =KINETIS_UART_VIRTUAL_BASE 	@ virtual
	.endm

	.macro	senduart, rd, rx
	strb	\rd, [\rx, #0x7]	@ Data Register
	.endm

	.macro	busyuart, rd, rx
1001:	ldrb	\rd, [\rx, #0x4]	@ Status Register 1
	tst	\rd, #1 << 6		@ TC
	beq	1001b			@ wait until transmit done
	.endm

	.macro	waituart,rd,rx
	.endm
