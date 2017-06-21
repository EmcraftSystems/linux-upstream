/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DTS_IMX6ULL_PINFUNC_H
#define __DTS_IMX6ULL_PINFUNC_H

#include "imx6ul-pinfunc.h"

#define PAD_CTL_SRE_SLOW		(0 << 0)
#define PAD_CTL_SRE_FAST		(1 << 0)

#define PAD_CTL_DSE_DISABLE		(0 << 3)
#define PAD_CTL_DSE_260ohm_3V3		(1 << 3)
#define PAD_CTL_DSE_130ohm_3V3		(2 << 3)
#define PAD_CTL_DSE_87ohm_3V3		(3 << 3)
#define PAD_CTL_DSE_65ohm_3V3		(4 << 3)
#define PAD_CTL_DSE_52ohm_3V3		(5 << 3)
#define PAD_CTL_DSE_43ohm_3V3		(6 << 3)
#define PAD_CTL_DSE_37ohm_3V3		(7 << 3)

#define PAD_CTL_DSE_150ohm_1V8		(1 << 3)
#define PAD_CTL_DSE_75ohm_1V8		(2 << 3)
#define PAD_CTL_DSE_50ohm_1V8		(3 << 3)
#define PAD_CTL_DSE_37ohm_1V8		(4 << 3)
#define PAD_CTL_DSE_30ohm_1V8		(5 << 3)
#define PAD_CTL_DSE_25ohm_1V8		(6 << 3)
#define PAD_CTL_DSE_22ohm_1V8		(7 << 3)

#define PAD_CTL_SPEED_0_LOW_50mhz	(0 << 6)
#define PAD_CTL_SPEED_1_MED_100mhz	(1 << 6)
#define PAD_CTL_SPEED_2_MED_100mhz	(2 << 6)
#define PAD_CTL_SPEED_3_HIGH_200mhz	(3 << 6)

#define PAD_CTL_ODE			(1 << 11)
#define PAD_CTL_PKE			(1 << 12)
#define PAD_CTL_PUE			(1 << 13)

#define PAD_CTL_PUS_100K_DOWN		(0 << 14)
#define PAD_CTL_PUS_47K_UP		(1 << 14)
#define PAD_CTL_PUS_100K_UP		(2 << 14)
#define PAD_CTL_PUS_22K_UP		(3 << 14)

#define PAD_CTL_HYS			(1 << 16)

#define PAD_CTL_SION			(1 << 30)


#define MX6UL_HIGH_DRV			(PAD_CTL_DSE_260ohm_3V3)

#define MX6UL_UART_PAD_CTRL		(PAD_CTL_DSE_43ohm_3V3 |	\
					 PAD_CTL_SPEED_2_MED_100mhz |	\
					 PAD_CTL_SRE_FAST |		\
					 PAD_CTL_HYS)

#define MX6UL_SPI_PAD_CTRL		(PAD_CTL_DSE_87ohm_3V3 |	\
					 PAD_CTL_PUS_100K_UP |		\
					 PAD_CTL_SPEED_2_MED_100mhz |	\
					 PAD_CTL_HYS)

#define MX6UL_ENET_TX_PAD_CTRL		(PAD_CTL_DSE_52ohm_3V3 |	\
					 PAD_CTL_SPEED_3_HIGH_200mhz |	\
					 PAD_CTL_PUE |			\
					 PAD_CTL_PKE |			\
					 PAD_CTL_PUS_22K_UP |		\
					 PAD_CTL_SRE_FAST)

#define MX6UL_ENET_RX_PAD_CTRL		(PAD_CTL_DSE_DISABLE |		\
					 PAD_CTL_SPEED_3_HIGH_200mhz |	\
					 PAD_CTL_PUE |			\
					 PAD_CTL_PKE |			\
					 PAD_CTL_PUS_22K_UP)

#define MX6UL_ENET_MDIO_PAD_CTRL	(PAD_CTL_DSE_52ohm_3V3 |	\
					 PAD_CTL_SPEED_3_HIGH_200mhz |	\
					 PAD_CTL_PUE |			\
					 PAD_CTL_PKE |			\
					 PAD_CTL_PUS_22K_UP |		\
					 PAD_CTL_SRE_FAST)

#define MX6UL_ENET_CLK_OUT_PAD_CTRL	(PAD_CTL_SION |			\
					 PAD_CTL_DSE_65ohm_3V3 |	\
					 PAD_CTL_SPEED_3_HIGH_200mhz |	\
					 PAD_CTL_SRE_FAST)
#define MX6UL_ENET_CLK_IN_PAD_CTRL	(PAD_CTL_DSE_DISABLE |	\
					 PAD_CTL_SPEED_3_HIGH_200mhz |	\
					 PAD_CTL_PUS_22K_UP)

#define MX6UL_I2C_PAD_CTRL		(PAD_CTL_SION |			\
					 PAD_CTL_DSE_43ohm_3V3 |	\
					 PAD_CTL_SPEED_2_MED_100mhz |	\
					 PAD_CTL_ODE |			\
					 PAD_CTL_PKE |			\
					 PAD_CTL_PUE |			\
					 PAD_CTL_PUS_100K_UP |		\
					 PAD_CTL_HYS)

#define MX6UL_SDHC_PAD_CTRL		(PAD_CTL_SRE_FAST |		\
					 PAD_CTL_DSE_87ohm_3V3 |	\
					 PAD_CTL_SPEED_1_MED_100mhz |	\
					 PAD_CTL_PKE |			\
					 PAD_CTL_PUE |			\
					 PAD_CTL_PUS_22K_UP |		\
					 PAD_CTL_HYS)

#define MX6UL_SDHC_CLK_PAD_CTRL		(PAD_CTL_SRE_FAST |		\
					 PAD_CTL_DSE_43ohm_3V3 |	\
					 PAD_CTL_SPEED_1_MED_100mhz |	\
					 PAD_CTL_HYS)

#define MX6UL_LCD_PAD_CTRL		(PAD_CTL_SRE_FAST |		\
					 PAD_CTL_DSE_37ohm_3V3 |	\
					 PAD_CTL_SPEED_1_MED_100mhz)

#define MX6UL_PWM_PAD_CTRL		(PAD_CTL_DSE_43ohm_3V3 |	\
					 PAD_CTL_SPEED_2_MED_100mhz |	\
					 PAD_CTL_PKE |			\
					 PAD_CTL_HYS)

#define MX6UL_NAND_PAD_CTRL		(PAD_CTL_SRE_FAST |		\
					 PAD_CTL_DSE_43ohm_3V3 |	\
					 PAD_CTL_SPEED_2_MED_100mhz |	\
					 PAD_CTL_PKE |			\
					 PAD_CTL_PUE)

#define MX6UL_GPIO_PAD_CTRL		(PAD_CTL_DSE_43ohm_3V3 |	\
					 PAD_CTL_SPEED_0_LOW_50mhz |	\
					 PAD_CTL_PUS_100K_UP)

#define MX6UL_GPIO_KEY_PAD_CTRL		(PAD_CTL_DSE_43ohm_3V3 |	\
					 PAD_CTL_SPEED_0_LOW_50mhz |	\
					 PAD_CTL_PKE |			\
					 PAD_CTL_PUE |			\
					 PAD_CTL_PUS_100K_UP |		\
					 PAD_CTL_HYS)

#define MX6UL_SOUND_PAD_CTRL		(PAD_CTL_DSE_43ohm_3V3 |	\
					 PAD_CTL_SPEED_2_MED_100mhz |	\
					 PAD_CTL_PKE |			\
					 PAD_CTL_PUE |			\
					 PAD_CTL_HYS)

#define MX6UL_WDOG_PAD_CTRL		(PAD_CTL_DSE_43ohm_3V3 |	\
					 PAD_CTL_SPEED_2_MED_100mhz |	\
					 PAD_CTL_PKE |			\
					 PAD_CTL_PUE)

#define MX6UL_OTG_ID_PAD_CTRL		(PAD_CTL_SRE_FAST |		\
					 PAD_CTL_DSE_87ohm_3V3 |	\
					 PAD_CTL_SPEED_1_MED_100mhz |	\
					 PAD_CTL_PKE |			\
					 PAD_CTL_PUE |			\
					 PAD_CTL_PUS_22K_UP |		\
					 PAD_CTL_HYS)
/*
 * The pin function ID is a tuple of
 * <mux_reg conf_reg input_reg mux_mode input_val>
 */
#define MX6ULL_PAD_ENET2_RX_DATA0__EPDC_SDDO08                    0x00E4 0x0370 0x0000 0x9 0x0
#define MX6ULL_PAD_ENET2_RX_DATA1__EPDC_SDDO09                    0x00E8 0x0374 0x0000 0x9 0x0
#define MX6ULL_PAD_ENET2_RX_EN__EPDC_SDDO10                       0x00EC 0x0378 0x0000 0x9 0x0
#define MX6ULL_PAD_ENET2_TX_DATA0__EPDC_SDDO11                    0x00F0 0x037C 0x0000 0x9 0x0
#define MX6ULL_PAD_ENET2_TX_DATA1__EPDC_SDDO12                    0x00F4 0x0380 0x0000 0x9 0x0
#define MX6ULL_PAD_ENET2_TX_EN__EPDC_SDDO13                       0x00F8 0x0384 0x0000 0x9 0x0
#define MX6ULL_PAD_ENET2_TX_CLK__EPDC_SDDO14                      0x00FC 0x0388 0x0000 0x9 0x0
#define MX6ULL_PAD_ENET2_RX_ER__EPDC_SDDO15                       0x0100 0x038C 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_CLK__EPDC_SDCLK                            0x0104 0x0390 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_ENABLE__EPDC_SDLE                          0x0108 0x0394 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_HSYNC__EPDC_SDOE                           0x010C 0x0398 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_VSYNC__EPDC_SDCE0                          0x0110 0x039C 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_RESET__EPDC_GDOE                           0x0114 0x03A0 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_DATA00__EPDC_SDDO00                        0x0118 0x03A4 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_DATA01__EPDC_SDDO01                        0x011C 0x03A8 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_DATA02__EPDC_SDDO02                        0x0120 0x03AC 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_DATA03__EPDC_SDDO03                        0x0124 0x03B0 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_DATA04__EPDC_SDDO04                        0x0128 0x03B4 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_DATA05__EPDC_SDDO05                        0x012C 0x03B8 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_DATA06__EPDC_SDDO06                        0x0130 0x03BC 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_DATA07__EPDC_SDDO07                        0x0134 0x03C0 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_DATA14__EPDC_SDSHR                         0x0150 0x03DC 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_DATA15__EPDC_GDRL                          0x0154 0x03E0 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_DATA16__EPDC_GDCLK                         0x0158 0x03E4 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_DATA17__EPDC_GDSP                          0x015C 0x03E8 0x0000 0x9 0x0
#define MX6ULL_PAD_LCD_DATA21__EPDC_SDCE1                         0x016C 0x03F8 0x0000 0x9 0x0
#define MX6ULL_PAD_CSI_MCLK__ESAI_TX3_RX2                         0x01D4 0x0460 0x0000 0x9 0x0
#define MX6ULL_PAD_CSI_PIXCLK__ESAI_TX2_RX3                       0x01D8 0x0464 0x0000 0x9 0x0
#define MX6ULL_PAD_CSI_VSYNC__ESAI_TX4_RX1                        0x01DC 0x0468 0x0000 0x9 0x0
#define MX6ULL_PAD_CSI_HSYNC__ESAI_TX1                            0x01E0 0x046C 0x0000 0x9 0x0
#define MX6ULL_PAD_CSI_DATA00__ESAI_TX_HF_CLK                     0x01E4 0x0470 0x0000 0x9 0x0
#define MX6ULL_PAD_CSI_DATA01__ESAI_RX_HF_CLK                     0x01E8 0x0474 0x0000 0x9 0x0
#define MX6ULL_PAD_CSI_DATA02__ESAI_RX_FS                         0x01EC 0x0478 0x0000 0x9 0x0
#define MX6ULL_PAD_CSI_DATA03__ESAI_RX_CLK                        0x01F0 0x047C 0x0000 0x9 0x0
#define MX6ULL_PAD_CSI_DATA04__ESAI_TX_FS                         0x01F4 0x0480 0x0000 0x9 0x0
#define MX6ULL_PAD_CSI_DATA05__ESAI_TX_CLK                        0x01F8 0x0484 0x0000 0x9 0x0
#define MX6ULL_PAD_CSI_DATA06__ESAI_TX5_RX0                       0x01FC 0x0488 0x0000 0x9 0x0
#define MX6ULL_PAD_CSI_DATA07__ESAI_T0                            0x0200 0x048C 0x0000 0x9 0x0

#endif /* __DTS_IMX6ULL_PINFUNC_H */
