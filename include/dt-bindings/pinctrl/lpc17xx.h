/*
 * Copyright (C) 2017 Emcraft Systems
 * Sergei Miroshnichenko <sergeimir@emcraft.com>
 *
 * License terms:  GNU General Public License (GPL), version 2
 */
#ifndef _DT_BINDINGS_PINCTRL_LPC17XX_H
#define _DT_BINDINGS_PINCTRL_LPC17XX_H

#define IOCON_NR_PORTS			6
#define IOCON_PINS_PER_PORT		32
#define IOCON_PIN(port, pin)		((port) * IOCON_PINS_PER_PORT + (pin))

#define IOCON_ADW_MODE_INACTIVE		(0 << 3)
#define IOCON_ADW_MODE_PULL_DOWN	(1 << 3)
#define IOCON_ADW_MODE_PULL_UP		(2 << 3)
#define IOCON_ADW_MODE_REPEATER		(3 << 3)

#define IOCON_DW_HYSTERESIS_ENABLE	(1 << 5)

#define IOCON_ADIW_POLARITY_NORMAL	(0 << 6)
#define IOCON_ADIW_POLARITY_INVERTED	(1 << 6)

#define IOCON_A_ADMODE_ANALOG		(0 << 7)
#define IOCON_A_ADMODE_DIGITAL		(1 << 7)

#define IOCON_AW_FILTER_ENABLE		(0 << 8)
#define IOCON_AW_FILTER_DISABLE		(1 << 8)

#define IOCON_I_HS_I2C_FILTER_ENABLE	(0 << 8)
#define IOCON_I_HS_I2C_FILTER_DISABLE	(1 << 8)

#define IOCON_DW_SLEW_MODE_NORMAL	(0 << 9)
#define IOCON_DW_SLEW_MODE_FAST		(1 << 9)

#define IOCON_I_HIDRIVE_SINK_4MA	(0 << 9)
#define IOCON_I_HIDRIVE_SINK_20MA	(1 << 9)

#define IOCON_ADW_PUSH_PULL		(0 << 10)
#define IOCON_ADW_OPEN_DRAIN		(1 << 10)

#define IOCON_A_DAC_ENABLE		(1 << 16)


#define IOCON_UART			(			\
		IOCON_ADW_MODE_INACTIVE				\
		| IOCON_ADIW_POLARITY_NORMAL			\
		| IOCON_DW_SLEW_MODE_NORMAL			\
		)


#define IOCON_FUNC_MASK			0x7
#define IOCON_FUNC_SHIFT		0
#define IOCON_FUNC__GPIO		0

/*	Definition			Pin/Port	Function index */
#define IOCON_P0_0__CAN_RD1		IOCON_PIN(0,0)	1
#define IOCON_P0_0__UART3_TXD		IOCON_PIN(0,0)	2
#define IOCON_P0_0__I2C1_SDA		IOCON_PIN(0,0)	3
#define IOCON_P0_0__UART0_TXD		IOCON_PIN(0,0)	4

#define IOCON_P0_1__CAN_TD1		IOCON_PIN(0,1)	1
#define IOCON_P0_1__UART3_RXD		IOCON_PIN(0,1)	2
#define IOCON_P0_1__I2C1_SCL		IOCON_PIN(0,1)	3
#define IOCON_P0_1__UART0_RXD		IOCON_PIN(0,1)	4

#define IOCON_P0_2__UART0_TXD		IOCON_PIN(0,2)	1
#define IOCON_P0_2__UART3_TXD		IOCON_PIN(0,2)	2

#define IOCON_P0_3__UART0_RXD		IOCON_PIN(0,3)	1
#define IOCON_P0_3__UART3_RXD		IOCON_PIN(0,3)	2

#define IOCON_P0_10__UART2_TXD		IOCON_PIN(0,10)	1
#define IOCON_P0_10__I2C2_SDA		IOCON_PIN(0,10)	2
#define IOCON_P0_10__T3_MAT0		IOCON_PIN(0,10)	3

#define IOCON_P0_11__UART2_RXD		IOCON_PIN(0,11)	1
#define IOCON_P0_11__I2C2_SCL		IOCON_PIN(0,11)	2
#define IOCON_P0_11__T3_MAT1		IOCON_PIN(0,11)	3

#define IOCON_P4_28__EMC_BLS2		IOCON_PIN(4,28)	1
#define IOCON_P4_28__UART3_TXD		IOCON_PIN(4,28)	2
#define IOCON_P4_28__T2_MAT0		IOCON_PIN(4,28)	3
#define IOCON_P4_28__LCD_VD6		IOCON_PIN(4,28)	5
#define IOCON_P4_28__LCD_VD10		IOCON_PIN(4,28)	6
#define IOCON_P4_28__LCD_VD2		IOCON_PIN(4,28)	7

#define IOCON_P4_29__EMC_BLS3		IOCON_PIN(4,29)	1
#define IOCON_P4_29__UART3_RXD		IOCON_PIN(4,29)	2
#define IOCON_P4_29__T2_MAT1		IOCON_PIN(4,29)	3
#define IOCON_P4_29__I2C2_SCL		IOCON_PIN(4,29)	4
#define IOCON_P4_29__LCD_VD7		IOCON_PIN(4,29)	5
#define IOCON_P4_29__LCD_VD11		IOCON_PIN(4,29)	6
#define IOCON_P4_29__LCD_VD3		IOCON_PIN(4,29)	7

/*
 * Examples of pin definition - three arguments:
 * - number,
 * - function (multiplexer),
 * - configuration:
 * nxp,pins = <
 *	IOCON_P0_2__UART0_TXD		IOCON_UART
 *	IOCON_PIN(0,11)		4	IOCON_ADW_MODE_PULL_UP
 * >;
};
*/
#define IOCON_DTS_ARGUMENTS		3

#endif /* _DT_BINDINGS_PINCTRL_LPC17XX_H */
