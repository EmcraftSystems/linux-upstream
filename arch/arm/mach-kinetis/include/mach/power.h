/*
 * (C) Copyright 2011, 2012
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */

#ifndef _MACH_KINETIS_POWER_H
#define _MACH_KINETIS_POWER_H

/*
 * Pack the SIM_SCGC[] register index and the bit index in that register into
 * a single word. This is similar to the implementation of `dev_t` in
 * the Linux kernel with its `MAJOR(dev)`, `MINOR(dev)` and
 * `MKDEV(major,minor)` macros.
 *
 * This is useful when you want to have an array of `kinetis_clock_gate_t`s:
 * you do not have to use a 2-dimensional array or a real structure.
 */
typedef u32 kinetis_clock_gate_t;
#define KINETIS_CG_IDX_BITS	16
#define KINETIS_CG_IDX_MASK	((1U << KINETIS_CG_IDX_BITS) - 1)
/*
 * Extract the register number and the bit index from a `kinetis_clock_gate_t`.
 * The register number counts from 0,
 * i.e. the register number for SIM_SCGC7 is 6.
 */
#define KINETIS_CG_REG(gate)	((unsigned int) ((gate) >> KINETIS_CG_IDX_BITS))
#define KINETIS_CG_IDX(gate)	((unsigned int) ((gate) & KINETIS_CG_IDX_MASK))
/*
 * Build a `kinetis_clock_gate_t` from a register number and a bit index
 */
#define KINETIS_MKCG(reg, idx) \
	(((kinetis_clock_gate_t)(reg) << KINETIS_CG_IDX_BITS) | \
	(kinetis_clock_gate_t)(idx))

/*
 * Clock gates for the modules inside the MCU
 */
/* UARTs */
#define KINETIS_CG_UART0	KINETIS_MKCG(3, 10)	/* SIM_SCGC4[10] */
#define KINETIS_CG_UART1	KINETIS_MKCG(3, 11)	/* SIM_SCGC4[11] */
#define KINETIS_CG_UART2	KINETIS_MKCG(3, 12)	/* SIM_SCGC4[12] */
#define KINETIS_CG_UART3	KINETIS_MKCG(3, 13)	/* SIM_SCGC4[13] */
#define KINETIS_CG_UART4	KINETIS_MKCG(0, 10)	/* SIM_SCGC1[10] */
#define KINETIS_CG_UART5	KINETIS_MKCG(0, 11)	/* SIM_SCGC1[11] */
/* Ports */
#define KINETIS_CG_PORTA	KINETIS_MKCG(4, 9)	/* SIM_SCGC5[9]  */
#define KINETIS_CG_PORTB	KINETIS_MKCG(4, 10)	/* SIM_SCGC5[10] */
#define KINETIS_CG_PORTC	KINETIS_MKCG(4, 11)	/* SIM_SCGC5[11] */
#define KINETIS_CG_PORTD	KINETIS_MKCG(4, 12)	/* SIM_SCGC5[12] */
#define KINETIS_CG_PORTE	KINETIS_MKCG(4, 13)	/* SIM_SCGC5[13] */
#define KINETIS_CG_PORTF	KINETIS_MKCG(4, 14)	/* SIM_SCGC5[14] */
/* ENET */
#define KINETIS_CG_ENET		KINETIS_MKCG(1, 0)	/* SIM_SCGC2[0]  */
/* Periodic Interrupt Timer (PIT) */
#define KINETIS_CG_PIT		KINETIS_MKCG(5, 23)	/* SIM_SCGC6[23] */
/* LCD Controller */
#define KINETIS_CG_LCDC		KINETIS_MKCG(2, 22)	/* SIM_SCGC3[22] */
/* DMA controller and DMA request multiplexer */
#define KINETIS_CG_DMA		KINETIS_MKCG(6, 1)	/* SIM_SCGC7[1]  */
#define KINETIS_CG_DMAMUX0	KINETIS_MKCG(5, 1)	/* SIM_SCGC6[1]  */
#define KINETIS_CG_DMAMUX1	KINETIS_MKCG(5, 2)	/* SIM_SCGC6[2]  */
/* USB High Speed */
#define KINETIS_CG_USBHS	KINETIS_MKCG(5, 20)	/* SIM_SCGC6[20] */
/* USB Full Speed */
#define KINETIS_CG_USBFS	KINETIS_MKCG(3, 18)	/* SIM_SCGC4[18] */
/* ADC modules */
#define KINETIS_CG_ADC0		KINETIS_MKCG(5, 27)	/* SIM_SCGC6[27] */
#define KINETIS_CG_ADC1		KINETIS_MKCG(2, 27)	/* SIM_SCGC3[27] */
#define KINETIS_CG_ADC2		KINETIS_MKCG(5, 28)	/* SIM_SCGC6[28] */
#define KINETIS_CG_ADC3		KINETIS_MKCG(2, 28)	/* SIM_SCGC3[28] */
/* ESDHC */
#define KINETIS_CG_ESDHC	KINETIS_MKCG(2, 17)	/* SIM_SCGC3[17] */
/* SPI */
#define KINETIS_CG_SPI0		KINETIS_MKCG(5, 12)	/* SIM_SCGC6[12] */
#define KINETIS_CG_SPI1		KINETIS_MKCG(5, 13)	/* SIM_SCGC6[13] */
#define KINETIS_CG_SPI2		KINETIS_MKCG(2, 12)	/* SIM_SCGC3[12] */
/* I2C */
#define KINETIS_CG_I2C0		KINETIS_MKCG(3, 6)	/* SIM_SCGC4[6]  */
#define KINETIS_CG_I2C1		KINETIS_MKCG(3, 7)	/* SIM_SCGC4[7]  */
/* ADC */
#define KINETIS_CG_ADC0		KINETIS_MKCG(5, 27)	/* SIM_SCGC6[27] */
#define KINETIS_CG_ADC1		KINETIS_MKCG(2, 27)	/* SIM_SCGC3[27] */
#define KINETIS_CG_ADC2		KINETIS_MKCG(5, 28)	/* SIM_SCGC6[28] */
#define KINETIS_CG_ADC3		KINETIS_MKCG(2, 28)	/* SIM_SCGC3[28] */
/* FlexCAN */
#define KINETIS_CG_CAN0		KINETIS_MKCG(5, 4)	/* SIM_SCGC6[4] */
#define KINETIS_CG_CAN1		KINETIS_MKCG(2, 4)	/* SIM_SCGC3[4] */

/* Other stuff, configured from U-Boot */
#define KINETIS_CG_OSC1		KINETIS_MKCG(0, 5)	/* SIM_SCGC1[5]  */
#define KINETIS_CG_DAC0		KINETIS_MKCG(1, 12)	/* SIM_SCGC2[12] */
#define KINETIS_CG_DAC1		KINETIS_MKCG(1, 13)	/* SIM_SCGC2[13] */
#define KINETIS_CG_RNGA		KINETIS_MKCG(2, 0)	/* SIM_SCGC3[0]  */
#define KINETIS_CG_FLEXCAN1	KINETIS_MKCG(2, 4)	/* SIM_SCGC3[4]  */
#define KINETIS_CG_NFC		KINETIS_MKCG(2, 8)	/* SIM_SCGC3[8]  */
#define KINETIS_CG_SAI1		KINETIS_MKCG(2, 15)	/* SIM_SCGC3[15] */
#define KINETIS_CG_FTM2		KINETIS_MKCG(2, 24)	/* SIM_SCGC3[24] */
#define KINETIS_CG_FTM3		KINETIS_MKCG(2, 25)	/* SIM_SCGC3[25] */
#define KINETIS_CG_EWM		KINETIS_MKCG(3, 1)	/* SIM_SCGC4[1]  */
#define KINETIS_CG_CMT		KINETIS_MKCG(3, 2)	/* SIM_SCGC4[2]  */
#define KINETIS_CG_CMP		KINETIS_MKCG(3, 19)	/* SIM_SCGC4[19] */
#define KINETIS_CG_VREF		KINETIS_MKCG(3, 20)	/* SIM_SCGC4[20] */
#define KINETIS_CG_LLWU		KINETIS_MKCG(3, 28)	/* SIM_SCGC4[28] */
#define KINETIS_CG_LPTIMER	KINETIS_MKCG(4, 0)	/* SIM_SCGC5[0]  */
#define KINETIS_CG_DRYICE	KINETIS_MKCG(4, 2)	/* SIM_SCGC5[2]  */
#define KINETIS_CG_DRYICESECREG	KINETIS_MKCG(4, 3)	/* SIM_SCGC5[3]  */
#define KINETIS_CG_TSI		KINETIS_MKCG(4, 5)	/* SIM_SCGC5[5]  */
#define KINETIS_CG_FLEXCAN0	KINETIS_MKCG(5, 4)	/* SIM_SCGC6[4]  */
#define KINETIS_CG_SAI0		KINETIS_MKCG(5, 15)	/* SIM_SCGC6[15] */
#define KINETIS_CG_CRC		KINETIS_MKCG(5, 18)	/* SIM_SCGC6[18] */
#define KINETIS_CG_USBDCD	KINETIS_MKCG(5, 21)	/* SIM_SCGC6[21] */
#define KINETIS_CG_PDB		KINETIS_MKCG(5, 22)	/* SIM_SCGC6[22] */
#define KINETIS_CG_FTM0		KINETIS_MKCG(5, 24)	/* SIM_SCGC6[24] */
#define KINETIS_CG_FTM1		KINETIS_MKCG(5, 25)	/* SIM_SCGC6[25] */
#define KINETIS_CG_RTC		KINETIS_MKCG(5, 29)	/* SIM_SCGC6[29] */
#define KINETIS_CG_FLEXBUS	KINETIS_MKCG(6, 0)	/* SIM_SCGC7[0]  */
#define KINETIS_CG_MPU		KINETIS_MKCG(6, 2)	/* SIM_SCGC7[2]  */

#ifndef __ASSEMBLY__

/*
 * Store the current values of the clock gate registers
 */
void kinetis_periph_push(void);

/*
 * Re-store previously saved clock gate registers
 */
void kinetis_periph_pop(void);

/*
 * Enable or disable the clock on a peripheral device (timers, UARTs, etc)
 */
int kinetis_periph_enable(kinetis_clock_gate_t gate, int enable);

/*
 * Print status of a peripheral clock gate
 */
void kinetis_periph_print(kinetis_clock_gate_t gate, char *name);

/*
 * Print status of all peripheral clock gates
 */
void kinetis_periph_print_all(void);

#endif /* __ASSEMBLY__ */

#endif /*_MACH_KINETIS_POWER_H */
