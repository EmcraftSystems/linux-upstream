/*
 * This file is part of STM32F4 ADC driver
 *
 * Copyright (C) 2016, STMicroelectronics - All Rights Reserved
 * Author: Fabrice Gasnier <fabrice.gasnier@st.com>.
 *
 * License type: GPLv2
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>

#include <mach/pwr.h>
#include <mach/syscfg.h>

#include "stm32-adc.h"

/*
 * STM32F4 ADC accuracy options names
 */
#define STM32F4_OPT1			"stm32f4,adc-accuracy-option1"
#define STM32F4_OPT2			"stm32f4,adc-accuracy-option2"

/*
 * STM32F4 - ADC global register map
 * ________________________________________________________
 * | Offset |                 Register                    |
 * --------------------------------------------------------
 * | 0x000  |                Master ADC1                  |
 * --------------------------------------------------------
 * | 0x100  |                Slave ADC2                   |
 * --------------------------------------------------------
 * | 0x200  |                Slave ADC3                   |
 * --------------------------------------------------------
 * | 0x300  |         Master & Slave common regs          |
 * --------------------------------------------------------
 */

/* STM32F4 - Registers for each ADC instance */
#define STM32F4_ADCX_SR			0x00
#define STM32F4_ADCX_CR1		0x04
#define STM32F4_ADCX_CR2		0x08
#define STM32F4_ADCX_SMPR1		0x0C
#define STM32F4_ADCX_SMPR2		0x10
#define STM32F4_ADCX_HTR		0x24
#define STM32F4_ADCX_LTR		0x28
#define STM32F4_ADCX_SQR1		0x2C
#define STM32F4_ADCX_SQR2		0x30
#define STM32F4_ADCX_SQR3		0x34
#define STM32F4_ADCX_JSQR		0x38
#define STM32F4_ADCX_JDR1		0x3C
#define STM32F4_ADCX_JDR2		0x40
#define STM32F4_ADCX_JDR3		0x44
#define STM32F4_ADCX_JDR4		0x48
#define STM32F4_ADCX_DR			0x4C

/* STM32 - Master & slave registers (common for all instances: 1, 2 & 3) */
#define STM32F4_ADC_CSR			(STM32_ADCX_COMN_OFFSET + 0x00)
#define STM32F4_ADC_CCR			(STM32_ADCX_COMN_OFFSET + 0x04)
#define STM32F4_ADC_CDR			(STM32_ADCX_COMN_OFFSET + 0x08)

/* STM32F4_ADCX_SR - bit fields */
#define STM32F4_OVR			BIT(5)
#define STM32F4_STRT			BIT(4)
#define STM32F4_JSTRT			BIT(3)
#define STM32F4_JEOC			BIT(2)
#define STM32F4_EOC			BIT(1)
#define STM32F4_AWD			BIT(0)

/* STM32F4_ADCX_CR1 - bit fields */
#define STM32F4_OVRIE			BIT(26)
#define STM32F4_RES_SHIFT		24
#define STM32F4_RES_MASK		GENMASK(25, 24)
#define STM32F4_AWDEN			BIT(23)
#define STM32F4_JAWDEN			BIT(22)
#define STM32F4_DISCNUM_SHIFT		13
#define STM32F4_DISCNUM_MASK		GENMASK(15, 13)
#define STM32F4_JDISCEN			BIT(12)
#define STM32F4_DISCEN			BIT(11)
#define STM32F4_JAUTO			BIT(10)
#define STM32F4_AWDSGL			BIT(9)
#define STM32F4_SCAN			BIT(8)
#define STM32F4_JEOCIE			BIT(7)
#define STM32F4_AWDIE			BIT(6)
#define STM32F4_EOCIE			BIT(5)
#define STM32F4_AWDCH_SHIFT		0
#define STM32F4_AWDCH_MASK		GENMASK(4, 0)

/* STM32F4_ADCX_CR2 - bit fields */
#define STM32F4_SWSTART			BIT(30)
#define STM32F4_EXTEN_SHIFT		28
#define STM32F4_EXTEN_MASK		GENMASK(29, 28)
#define STM32F4_EXTSEL_SHIFT		24
#define STM32F4_EXTSEL_MASK		GENMASK(27, 24)
#define STM32F4_JSWSTART		BIT(22)
#define STM32F4_JEXTEN_SHIFT		20
#define STM32F4_JEXTEN_MASK		GENMASK(21, 20)
#define STM32F4_JEXTSEL_SHIFT		16
#define STM32F4_JEXTSEL_MASK		GENMASK(19, 16)
#define STM32F4_ALIGN			BIT(11)
#define STM32F4_EOCS			BIT(10)
#define STM32F4_DDS			BIT(9)
#define STM32F4_DMA			BIT(8)
#define STM32F4_CONT			BIT(1)
#define STM32F4_ADON			BIT(0)

/* STM32F4_ADCX_SMPR1 - bit fields */
#define STM32F4_SMP18_SHIFT		24
#define STM32F4_SMP18_MASK		GENMASK(26, 24)
#define STM32F4_SMP17_SHIFT		21
#define STM32F4_SMP17_MASK		GENMASK(23, 21)
#define STM32F4_SMP16_SHIFT		18
#define STM32F4_SMP16_MASK		GENMASK(20, 18)
#define STM32F4_SMP15_SHIFT		15
#define STM32F4_SMP15_MASK		GENMASK(17, 15)
#define STM32F4_SMP14_SHIFT		12
#define STM32F4_SMP14_MASK		GENMASK(14, 12)
#define STM32F4_SMP13_SHIFT		9
#define STM32F4_SMP13_MASK		GENMASK(11, 9)
#define STM32F4_SMP12_SHIFT		6
#define STM32F4_SMP12_MASK		GENMASK(8, 6)
#define STM32F4_SMP11_SHIFT		3
#define STM32F4_SMP11_MASK		GENMASK(5, 3)
#define STM32F4_SMP10_SHIFT		0
#define STM32F4_SMP10_MASK		GENMASK(2, 0)

/* STM32F4_ADCX_SMPR2 - bit fields */
#define STM32F4_SMP9_SHIFT		27
#define STM32F4_SMP9_MASK		GENMASK(29, 27)
#define STM32F4_SMP8_SHIFT		24
#define STM32F4_SMP8_MASK		GENMASK(26, 24)
#define STM32F4_SMP7_SHIFT		21
#define STM32F4_SMP7_MASK		GENMASK(23, 21)
#define STM32F4_SMP6_SHIFT		18
#define STM32F4_SMP6_MASK		GENMASK(20, 18)
#define STM32F4_SMP5_SHIFT		15
#define STM32F4_SMP5_MASK		GENMASK(17, 15)
#define STM32F4_SMP4_SHIFT		12
#define STM32F4_SMP4_MASK		GENMASK(14, 12)
#define STM32F4_SMP3_SHIFT		9
#define STM32F4_SMP3_MASK		GENMASK(11, 9)
#define STM32F4_SMP2_SHIFT		6
#define STM32F4_SMP2_MASK		GENMASK(8, 6)
#define STM32F4_SMP1_SHIFT		3
#define STM32F4_SMP1_MASK		GENMASK(5, 3)
#define STM32F4_SMP0_SHIFT		0
#define STM32F4_SMP0_MASK		GENMASK(2, 0)
enum stm32f4_adc_smpr {
	STM32F4_SMPR_3_CK_CYCLES,
	STM32F4_SMPR_15_CK_CYCLES,
	STM32F4_SMPR_28_CK_CYCLES,
	STM32F4_SMPR_56_CK_CYCLES,
	STM32F4_SMPR_84_CK_CYCLES,
	STM32F4_SMPR_112_CK_CYCLES,
	STM32F4_SMPR_144_CK_CYCLES,
	STM32F4_SMPR_480_CK_CYCLES,
};

/* STM32F4_ADCX_SQR1 - bit fields */
#define STM32F4_L_SHIFT			20
#define STM32F4_L_MASK			GENMASK(23, 20)
#define STM32F4_SQ16_SHIFT		15
#define STM32F4_SQ16_MASK		GENMASK(19, 15)
#define STM32F4_SQ15_SHIFT		10
#define STM32F4_SQ15_MASK		GENMASK(14, 10)
#define STM32F4_SQ14_SHIFT		5
#define STM32F4_SQ14_MASK		GENMASK(9, 5)
#define STM32F4_SQ13_SHIFT		0
#define STM32F4_SQ13_MASK		GENMASK(4, 0)

/* STM32F4_ADCX_SQR2 - bit fields */
#define STM32F4_SQ12_SHIFT		25
#define STM32F4_SQ12_MASK		GENMASK(29, 25)
#define STM32F4_SQ11_SHIFT		20
#define STM32F4_SQ11_MASK		GENMASK(24, 20)
#define STM32F4_SQ10_SHIFT		15
#define STM32F4_SQ10_MASK		GENMASK(19, 15)
#define STM32F4_SQ9_SHIFT		10
#define STM32F4_SQ9_MASK		GENMASK(14, 10)
#define STM32F4_SQ8_SHIFT		5
#define STM32F4_SQ8_MASK		GENMASK(9, 5)
#define STM32F4_SQ7_SHIFT		0
#define STM32F4_SQ7_MASK		GENMASK(4, 0)

/* STM32F4_ADCX_SQR3 - bit fields */
#define STM32F4_SQ6_SHIFT		25
#define STM32F4_SQ6_MASK		GENMASK(29, 25)
#define STM32F4_SQ5_SHIFT		20
#define STM32F4_SQ5_MASK		GENMASK(24, 20)
#define STM32F4_SQ4_SHIFT		15
#define STM32F4_SQ4_MASK		GENMASK(19, 15)
#define STM32F4_SQ3_SHIFT		10
#define STM32F4_SQ3_MASK		GENMASK(14, 10)
#define STM32F4_SQ2_SHIFT		5
#define STM32F4_SQ2_MASK		GENMASK(9, 5)
#define STM32F4_SQ1_SHIFT		0
#define STM32F4_SQ1_MASK		GENMASK(4, 0)

/* STM32F4_ADCX_JSQR - bit fields */
#define STM32F4_JL_SHIFT		20
#define STM32F4_JL_MASK			GENMASK(21, 20)
#define STM32F4_JSQ4_SHIFT		15
#define STM32F4_JSQ4_MASK		GENMASK(19, 15)
#define STM32F4_JSQ3_SHIFT		10
#define STM32F4_JSQ3_MASK		GENMASK(14, 10)
#define STM32F4_JSQ2_SHIFT		5
#define STM32F4_JSQ2_MASK		GENMASK(9, 5)
#define STM32F4_JSQ1_SHIFT		0
#define STM32F4_JSQ1_MASK		GENMASK(4, 0)

/* STM32F4_ADC_CCR - bit fields */
#define STM32F4_CCR_TSVREFE		BIT(23)
#define STM32F4_CCR_VBATE		BIT(22)
#define STM32F4_CCR_ADCPRE_SHIFT	16
#define STM32F4_CCR_ADCPRE_MASK		GENMASK(17, 16)
#define STM32F4_CCR_DMA_SHIFT		14
#define STM32F4_CCR_DMA_MASK		GENMASK(15, 14)
#define STM32F4_CCR_DDS			BIT(13)
#define STM32F4_CCR_MULTI_SHIFT		0
#define STM32F4_CCR_MULTI_MASK		GENMASK(4, 0)

/*
 * stm32 ADC1, ADC2 & ADC3 are tightly coupled and may be used in multi mode
 * Define here all inputs for all ADC instances
 */
static const struct stm32_adc_chan_spec stm32f4_adc1_channels[] = {
	/* master ADC1 */
	{ IIO_VOLTAGE, 0, "in0" },
	{ IIO_VOLTAGE, 1, "in1" },
	{ IIO_VOLTAGE, 2, "in2" },
	{ IIO_VOLTAGE, 3, "in3" },
	{ IIO_VOLTAGE, 4, "in4" },
	{ IIO_VOLTAGE, 5, "in5" },
	{ IIO_VOLTAGE, 6, "in6" },
	{ IIO_VOLTAGE, 7, "in7" },
	{ IIO_VOLTAGE, 8, "in8" },
	{ IIO_VOLTAGE, 9, "in9" },
	{ IIO_VOLTAGE, 10, "in10" },
	{ IIO_VOLTAGE, 11, "in11" },
	{ IIO_VOLTAGE, 12, "in12" },
	{ IIO_VOLTAGE, 13, "in13" },
	{ IIO_VOLTAGE, 14, "in14" },
	{ IIO_VOLTAGE, 15, "in15" },
	/* internal analog sources */
	{ IIO_TEMP,    16, "ts" },	/* STM32F40x & STM32F41x Temperature */
	{ IIO_VOLTAGE, 17, "vrefint" },
	{ IIO_VOLTAGE, 18, "vbat" },
	{ IIO_TEMP,    18, "ts" },	/* STM32F42x & STM32F43x Temperature */
};

static const struct stm32_adc_chan_spec stm32f4_adc23_channels[] = {
	/* slave ADC2 /	ADC3 */
	{ IIO_VOLTAGE, 0, "in0" },
	{ IIO_VOLTAGE, 1, "in1" },
	{ IIO_VOLTAGE, 2, "in2" },
	{ IIO_VOLTAGE, 3, "in3" },
	{ IIO_VOLTAGE, 4, "in4" },
	{ IIO_VOLTAGE, 5, "in5" },
	{ IIO_VOLTAGE, 6, "in6" },
	{ IIO_VOLTAGE, 7, "in7" },
	{ IIO_VOLTAGE, 8, "in8" },
	{ IIO_VOLTAGE, 9, "in9" },
	{ IIO_VOLTAGE, 10, "in10" },
	{ IIO_VOLTAGE, 11, "in11" },
	{ IIO_VOLTAGE, 12, "in12" },
	{ IIO_VOLTAGE, 13, "in13" },
	{ IIO_VOLTAGE, 14, "in14" },
	{ IIO_VOLTAGE, 15, "in15" },
};

/* Triggers for regular channels */
static const struct stm32_adc_trig_info stm32f4_adc_ext_triggers[] = {
	{ STM32_EXT0, "TIM1_CH1" },
	{ STM32_EXT1, "TIM1_CH2" },
	{ STM32_EXT2, "TIM1_CH3" },
	{ STM32_EXT3, "TIM2_CH2" },
	{ STM32_EXT4, "TIM2_CH3" },
	{ STM32_EXT5, "TIM2_CH4" },
	{ STM32_EXT6, "TIM2_TRGO" },
	{ STM32_EXT7, "TIM3_CH1" },
	{ STM32_EXT8, "TIM3_TRGO" },
	{ STM32_EXT9, "TIM4_CH4" },
	{ STM32_EXT10, "TIM5_CH1" },
	{ STM32_EXT11, "TIM5_CH2" },
	{ STM32_EXT12, "TIM5_CH3" },
	{ STM32_EXT13, "TIM8_CH1" },
	{ STM32_EXT14, "TIM8_TRGO" },
	{ STM32_EXT15, "EXTI_11" },
	{},
};

/* Triggers for injected channels */
static const struct stm32_adc_trig_info  stm32f4_adc_jext_triggers[] = {
	{ STM32_JEXT0, "TIM1_CH4" },
	{ STM32_JEXT1, "TIM1_TRGO" },
	{ STM32_JEXT2, "TIM2_CH1" },
	{ STM32_JEXT3, "TIM2_TRGO" },
	{ STM32_JEXT4, "TIM3_CH2" },
	{ STM32_JEXT5, "TIM3_CH4" },
	{ STM32_JEXT6, "TIM4_CH1" },
	{ STM32_JEXT7, "TIM4_CH2" },
	{ STM32_JEXT8, "TIM4_CH3" },
	{ STM32_JEXT9, "TIM4_TRGO" },
	{ STM32_JEXT10, "TIM5_CH4" },
	{ STM32_JEXT11, "TIM5_TRGO" },
	{ STM32_JEXT12, "TIM8_CH2" },
	{ STM32_JEXT13, "TIM8_CH3" },
	{ STM32_JEXT14, "TIM8_CH4" },
	{ STM32_JEXT15, "EXTI_15" },
	{},
};

static const struct stm32_adc_info stm32f4_adc_info[] = {
	{
		.name = "adc1-master",
		.reg = 0x0,
		.channels = stm32f4_adc1_channels,
		.max_channels = ARRAY_SIZE(stm32f4_adc1_channels),
	},
	{
		.name = "adc2-slave",
		.reg = 0x100,
		.channels = stm32f4_adc23_channels,
		.max_channels = ARRAY_SIZE(stm32f4_adc23_channels),
	},
	{
		.name = "adc3-slave",
		.reg = 0x200,
		.channels = stm32f4_adc23_channels,
		.max_channels = ARRAY_SIZE(stm32f4_adc23_channels),
	},
	{},
};

/**
 * stm32f4_smpr_regs[] - describe sampling time registers & bit fields
 * Sorted so it can be indexed by channel number.
 */
static const struct stm32_adc_regs stm32f4_smpr_regs[] = {
	{ STM32F4_ADCX_SMPR2, STM32F4_SMP0_MASK, STM32F4_SMP0_SHIFT },
	{ STM32F4_ADCX_SMPR2, STM32F4_SMP1_MASK, STM32F4_SMP1_SHIFT },
	{ STM32F4_ADCX_SMPR2, STM32F4_SMP2_MASK, STM32F4_SMP2_SHIFT },
	{ STM32F4_ADCX_SMPR2, STM32F4_SMP3_MASK, STM32F4_SMP3_SHIFT },
	{ STM32F4_ADCX_SMPR2, STM32F4_SMP4_MASK, STM32F4_SMP4_SHIFT },
	{ STM32F4_ADCX_SMPR2, STM32F4_SMP5_MASK, STM32F4_SMP5_SHIFT },
	{ STM32F4_ADCX_SMPR2, STM32F4_SMP6_MASK, STM32F4_SMP6_SHIFT },
	{ STM32F4_ADCX_SMPR2, STM32F4_SMP7_MASK, STM32F4_SMP7_SHIFT },
	{ STM32F4_ADCX_SMPR2, STM32F4_SMP8_MASK, STM32F4_SMP8_SHIFT },
	{ STM32F4_ADCX_SMPR2, STM32F4_SMP9_MASK, STM32F4_SMP9_SHIFT },
	{ STM32F4_ADCX_SMPR1, STM32F4_SMP10_MASK, STM32F4_SMP10_SHIFT },
	{ STM32F4_ADCX_SMPR1, STM32F4_SMP11_MASK, STM32F4_SMP11_SHIFT },
	{ STM32F4_ADCX_SMPR1, STM32F4_SMP12_MASK, STM32F4_SMP12_SHIFT },
	{ STM32F4_ADCX_SMPR1, STM32F4_SMP13_MASK, STM32F4_SMP13_SHIFT },
	{ STM32F4_ADCX_SMPR1, STM32F4_SMP14_MASK, STM32F4_SMP14_SHIFT },
	{ STM32F4_ADCX_SMPR1, STM32F4_SMP15_MASK, STM32F4_SMP15_SHIFT },
	{ STM32F4_ADCX_SMPR1, STM32F4_SMP16_MASK, STM32F4_SMP16_SHIFT },
	{ STM32F4_ADCX_SMPR1, STM32F4_SMP17_MASK, STM32F4_SMP17_SHIFT },
	{ STM32F4_ADCX_SMPR1, STM32F4_SMP18_MASK, STM32F4_SMP18_SHIFT },
};

/**
 * stm32f4_sqr_regs - describe regular sequence registers
 * - L: sequence len (register & bit field)
 * - SQ1..SQ16: sequence entries (register & bit field)
 */
static const struct stm32_adc_regs stm32f4_sqr_regs[STM32_ADC_MAX_SQ + 1] = {
	/* L: len bit field description to be kept as first element */
	{ STM32F4_ADCX_SQR1, STM32F4_L_MASK, STM32F4_L_SHIFT },
	/* SQ1..SQ16 registers & bit fields */
	{ STM32F4_ADCX_SQR3, STM32F4_SQ1_MASK, STM32F4_SQ1_SHIFT },
	{ STM32F4_ADCX_SQR3, STM32F4_SQ2_MASK, STM32F4_SQ2_SHIFT },
	{ STM32F4_ADCX_SQR3, STM32F4_SQ3_MASK, STM32F4_SQ3_SHIFT },
	{ STM32F4_ADCX_SQR3, STM32F4_SQ4_MASK, STM32F4_SQ4_SHIFT },
	{ STM32F4_ADCX_SQR3, STM32F4_SQ5_MASK, STM32F4_SQ5_SHIFT },
	{ STM32F4_ADCX_SQR3, STM32F4_SQ6_MASK, STM32F4_SQ6_SHIFT },
	{ STM32F4_ADCX_SQR2, STM32F4_SQ7_MASK, STM32F4_SQ7_SHIFT },
	{ STM32F4_ADCX_SQR2, STM32F4_SQ8_MASK, STM32F4_SQ8_SHIFT },
	{ STM32F4_ADCX_SQR2, STM32F4_SQ9_MASK, STM32F4_SQ9_SHIFT },
	{ STM32F4_ADCX_SQR2, STM32F4_SQ10_MASK, STM32F4_SQ10_SHIFT },
	{ STM32F4_ADCX_SQR2, STM32F4_SQ11_MASK, STM32F4_SQ11_SHIFT },
	{ STM32F4_ADCX_SQR2, STM32F4_SQ12_MASK, STM32F4_SQ12_SHIFT },
	{ STM32F4_ADCX_SQR1, STM32F4_SQ13_MASK, STM32F4_SQ13_SHIFT },
	{ STM32F4_ADCX_SQR1, STM32F4_SQ14_MASK, STM32F4_SQ14_SHIFT },
	{ STM32F4_ADCX_SQR1, STM32F4_SQ15_MASK, STM32F4_SQ15_SHIFT },
	{ STM32F4_ADCX_SQR1, STM32F4_SQ16_MASK, STM32F4_SQ16_SHIFT },
};

/**
 * stm32f4_jsqr_reg - describe injected sequence register:
 * - JL: injected sequence len
 * - JSQ4..SQ1: sequence entries
 * When JL == 3, ADC converts JSQ1, JSQ2, JSQ3, JSQ4
 * When JL == 2, ADC converts JSQ2, JSQ3, JSQ4
 * When JL == 1, ADC converts JSQ3, JSQ4
 * When JL == 0, ADC converts JSQ4
 */
static const struct stm32_adc_regs stm32f4_jsqr_reg[STM32_ADC_MAX_JSQ + 1] = {
	/* JL: len bit field description to be kept as first element */
	{STM32F4_ADCX_JSQR, STM32F4_JL_MASK, STM32F4_JL_SHIFT},
	/* JSQ4..JSQ1 registers & bit fields */
	{STM32F4_ADCX_JSQR, STM32F4_JSQ4_MASK, STM32F4_JSQ4_SHIFT},
	{STM32F4_ADCX_JSQR, STM32F4_JSQ3_MASK, STM32F4_JSQ3_SHIFT},
	{STM32F4_ADCX_JSQR, STM32F4_JSQ2_MASK, STM32F4_JSQ2_SHIFT},
	{STM32F4_ADCX_JSQR, STM32F4_JSQ1_MASK, STM32F4_JSQ1_SHIFT},
};

static const struct stm32_adc_trig_reginfo stm32f4_adc_trig_reginfo = {
	.reg = STM32F4_ADCX_CR2,
	.exten_mask = STM32F4_EXTEN_MASK,
	.exten_shift = STM32F4_EXTEN_SHIFT,
	.extsel_mask = STM32F4_EXTSEL_MASK,
	.extsel_shift = STM32F4_EXTSEL_SHIFT,
};

static const struct stm32_adc_trig_reginfo stm32f4_adc_jtrig_reginfo = {
	.reg = STM32F4_ADCX_CR2,
	.exten_mask = STM32F4_JEXTEN_MASK,
	.exten_shift = STM32F4_JEXTEN_SHIFT,
	.extsel_mask = STM32F4_JEXTSEL_MASK,
	.extsel_shift = STM32F4_JEXTSEL_SHIFT,
};

static const struct stm32_adc_reginfo stm32f4_adc_reginfo = {
	.isr = STM32F4_ADCX_SR,
	.ovr = STM32F4_OVR,
	.eoc = STM32F4_EOC,
	.jeoc = STM32F4_JEOC,
	.ier = STM32F4_ADCX_CR1,
	.ovrie = STM32F4_OVRIE,
	.eocie = STM32F4_EOCIE,
	.jeocie = STM32F4_JEOCIE,
	.dr = STM32F4_ADCX_DR,
	.cdr = STM32F4_ADC_CDR,
	.jdr = {
		STM32F4_ADCX_JDR1,
		STM32F4_ADCX_JDR2,
		STM32F4_ADCX_JDR3,
		STM32F4_ADCX_JDR4,
	},
	.sqr_regs = stm32f4_sqr_regs,
	.jsqr_reg = stm32f4_jsqr_reg,
	.smpr_regs = stm32f4_smpr_regs,
	.trig_reginfo = &stm32f4_adc_trig_reginfo,
	.jtrig_reginfo = &stm32f4_adc_jtrig_reginfo,
};

static bool stm32f4_adc_is_started(struct stm32_adc *adc)
{
	u32 val = stm32_adc_readl(adc, STM32F4_ADCX_CR2) & STM32F4_ADON;

	return !!val;
}

static bool stm32f4_adc_regular_started(struct stm32_adc *adc)
{
	u32 val = stm32_adc_readl(adc, STM32F4_ADCX_SR) & STM32F4_STRT;

	return !!val;
}

static bool stm32f4_adc_injected_started(struct stm32_adc *adc)
{
	u32 val = stm32_adc_readl(adc, STM32F4_ADCX_SR) & STM32F4_JSTRT;

	return !!val;
}

/**
 * stm32f4_adc_prepare_conv() - Prepare for conversion
 * @adc: stm32 adc instance
 * @chan: ADC channel
 *
 * Internal ADC sources (Temperature, VREF, VBAT) require additional
 * configuring.
 */
static void stm32f4_adc_prepare_conv(struct stm32_adc *adc,
		const struct iio_chan_spec *chan)
{
	u32 val;

	if (adc->id != 0 || chan->channel < 16)
		goto out;

	val = stm32_adc_common_readl(adc->common, STM32F4_ADC_CCR);
	val &= ~(STM32F4_CCR_TSVREFE | STM32F4_CCR_VBATE);

	/* Temperature sensor or VREFINT */
	if (chan->type == IIO_TEMP || chan->channel == 17)
		val |= STM32F4_CCR_TSVREFE;
	/* VBAT */
	if (chan->type == IIO_VOLTAGE && chan->channel == 18)
		val |= STM32F4_CCR_VBATE;

	stm32_adc_common_writel(adc->common, STM32F4_ADC_CCR, val);
out:
	return;
}

/**
 * stm32f4_adc_start_conv() - Start regular or injected conversions
 * @adc: stm32 adc instance
 *
 * Start single conversions for regular or injected channels.
 * Also take care of normal or DMA mode. DMA is used in circular mode for
 * regular conversions, in IIO buffer modes. Rely on rx_buf as raw
 * read doesn't use dma, but direct DR read.
 */
static int stm32f4_adc_start_conv(struct stm32_adc *adc)
{
	u32 val, trig_msk, start_msk;
	unsigned long flags;

	dev_dbg(adc->common->dev, "%s %s\n", __func__,
		adc->injected ? "injected" : "regular");

	stm32_adc_set_bits(adc, STM32F4_ADCX_CR1, STM32F4_SCAN);

	if (!adc->injected) {
		spin_lock_irqsave(&adc->lock, flags);
		val = stm32_adc_readl(adc, STM32F4_ADCX_CR2);
		val &= ~(STM32F4_DMA | STM32F4_DDS);
		if (adc->rx_buf)
			val |= STM32F4_DMA | STM32F4_DDS;
		stm32_adc_writel(adc, STM32F4_ADCX_CR2, val);
		spin_unlock_irqrestore(&adc->lock, flags);
	}

	if (!stm32f4_adc_is_started(adc)) {
		stm32_adc_set_bits(adc, STM32F4_ADCX_CR2,
				   STM32F4_EOCS | STM32F4_ADON);

		/* Wait for Power-up time (tSTAB from datasheet) */
		usleep_range(2, 3);
	}

	if (adc->injected) {
		trig_msk = STM32F4_JEXTEN_MASK;
		start_msk = STM32F4_JSWSTART;
	} else {
		trig_msk = STM32F4_EXTEN_MASK;
		start_msk = STM32F4_SWSTART;
	}

	/* Enable overrun interrupts */
	stm32_adc_set_bits(adc, STM32F4_ADCX_CR1, STM32F4_OVRIE);

	/* Software start ? (e.g. trigger detection disabled ?) */
	if (!adc->multi && !(stm32_adc_readl(adc, STM32F4_ADCX_CR2) & trig_msk)) {
		int i, inum;

		/*
		 * In Triple ADC we support the CRS+ATM mode only. Alternate
		 * trigger mode (ATM) operates as follows: when the 1st trigger
		 * occurs, all injected group channels in ADC1 are converted;
		 * when the 2nd trig happens, all injected group channels in
		 * ADC2 are converted; and so on. So to avoid timeouts we just
		 * repeat sw start multiple times
		 */
		inum = (adc->injected && stm32_avg_is_enabled(adc)) ? 3 : 1;
		for (i = 0; i < inum; i++)
			stm32_adc_set_bits(adc, STM32F4_ADCX_CR2, start_msk);
	}

	return 0;
}

static int stm32f4_adc_stop_conv(struct stm32_adc *adc)
{
	u32 val;

	dev_dbg(adc->common->dev, "%s %s\n", __func__,
		adc->injected ? "injected" : "regular");

	/* First disable trigger for either regular or injected channels */
	if (adc->injected) {
		stm32_adc_clr_bits(adc, STM32F4_ADCX_CR2, STM32F4_JEXTEN_MASK);
		stm32_adc_clr_bits(adc, STM32F4_ADCX_SR, STM32F4_JSTRT);
	} else {
		stm32_adc_clr_bits(adc, STM32F4_ADCX_CR2, STM32F4_EXTEN_MASK);
		stm32_adc_clr_bits(adc, STM32F4_ADCX_SR, STM32F4_STRT);
	}

	/* Disable adc when all triggered conversion have been disabled */
	val = stm32_adc_readl(adc, STM32F4_ADCX_CR2);
	val &= STM32F4_EXTEN_MASK | STM32F4_JEXTEN_MASK;
	if (!val) {
		stm32_adc_clr_bits(adc, STM32F4_ADCX_CR1, STM32F4_SCAN);
		stm32_adc_clr_bits(adc, STM32F4_ADCX_CR2, STM32F4_ADON);
	}

	if (!adc->injected)
		stm32_adc_clr_bits(adc, STM32F4_ADCX_CR2,
				   STM32F4_DMA | STM32F4_DDS);

	return 0;
}

static void stm32f4_adc_recover(struct stm32_adc *adc)
{
	stm32_adc_clr_bits(adc, STM32F4_ADCX_CR2, STM32F4_ADON);
	stm32_adc_clr_bits(adc, STM32F4_ADCX_SR, STM32F4_OVR);
	stm32_adc_set_bits(adc, STM32F4_ADCX_CR2, STM32F4_ADON);
}

/* ADC internal common clock prescaler division ratios */
static int stm32f4_pclk_div[] = {2, 4, 6, 8};

/**
 * stm32f4_parse_dts_params() - Parse dts specific params
 * @dev: device descriptor
 * @np: device node
 */
static int stm32f4_parse_dts_params(struct device *dev, struct device_node *np)
{
	struct device_node *tmp;
	const char *name;
	void *regs;
	u32 opt, val;
	int rv;

	if (of_get_property(np, STM32F4_OPT1, NULL) &&
	    of_get_property(np, STM32F4_OPT2, NULL)) {
		dev_err(dev, "only one (%s or %s) option is allowed\n",
			STM32F4_OPT1, STM32F4_OPT2);
		rv = -EINVAL;
		goto out;
	}

	if (of_get_property(np, STM32F4_OPT1, NULL)) {
		name = "st,stm32-pwr";
		tmp = of_find_compatible_node(NULL, NULL, name);
		if (!tmp) {
			dev_err(dev, "%s: `%s` lookup failed\n",
				STM32F4_OPT1, name);
			rv = -EINVAL;
			goto out;
		}
		regs = of_iomap(tmp, 0);
		of_node_put(tmp);

		writel(STM32_PWR_CR_ADCDC1 | readl(regs + STM32_PWR_CR),
			regs + STM32_PWR_CR);

		dev_info(dev, "%s is used", STM32F4_OPT1);
	}

	rv = of_property_read_u32(np, STM32F4_OPT2, &opt);
	if (!rv && opt) {
		if (STM32_SYSCFG_PMC_ADCxDC2(opt) &
		    ~STM32_SYSCFG_PMC_ADCxDC2_MASK) {
			dev_err(dev, "%s err: bad mask value %x\n",
				STM32F4_OPT2, opt);
			rv = -EINVAL;
			goto out;
		}

		name = "st,stm32-syscfg";
		regs = syscon_regmap_lookup_by_compatible(name);
		if (IS_ERR(regs)) {
			dev_err(dev, "%s: `%s` lookup failed\n",
				STM32F4_OPT2, name);
			rv = -EINVAL;
			goto out;
		}

		val = readl(regs + STM32_SYSCFG_PMC);
		val &= ~STM32_SYSCFG_PMC_ADCxDC2_MASK;
		writel(STM32_SYSCFG_PMC_ADCxDC2(opt) | val,
			regs + STM32_SYSCFG_PMC);

		dev_info(dev, "%s=%x is used", STM32F4_OPT2, opt);
	}

	rv = 0;
out:
	return rv;
}

/**
 * stm32f4_adc_clk_sel() - Select ADC common clock prescaler
 * @adc: stm32 adc instance
 * Select clock prescaler used for analog conversions.
 */
static int stm32f4_adc_clk_sel(struct stm32_adc *adc)
{
	struct stm32_adc_common *common = adc->common;
	unsigned long rate;
	u32 val;
	int i;

	/* Common prescaler is set only once, when 1st ADC instance starts */
	list_for_each_entry(adc, &common->adc_list, adc_list)
		if (stm32f4_adc_is_started(adc))
			return 0;

	rate = clk_get_rate(common->aclk);
	for (i = 0; i < ARRAY_SIZE(stm32f4_pclk_div); i++) {
		if ((rate / stm32f4_pclk_div[i]) <=
		    common->data->max_clock_rate)
			break;
	}
	if (i >= ARRAY_SIZE(stm32f4_pclk_div))
		return -EINVAL;

	val = stm32_adc_common_readl(common, STM32F4_ADC_CCR);
	val &= ~STM32F4_CCR_ADCPRE_MASK;
	val |= i << STM32F4_CCR_ADCPRE_SHIFT;
	stm32_adc_common_writel(common, STM32F4_ADC_CCR, val);

	dev_dbg(common->dev, "Using analog clock source at %ld kHz\n",
		rate / (stm32f4_pclk_div[i] * 1000));

	return 0;
}

/**
 * stm32f4_multi_dma() - configure multi ADC mode
 * @com:	ADC common structure pointer
 * @dm:		DMA mode
 * @mm:		multi ADC mode
 */
static int stm32f4_adc_multi_dma_sel(struct stm32_adc_common *com,
	enum stm32_adc_dma_mode dm, enum stm32_adc_multi_mode mm)
{
	/* 0xIR mask: injected/regular channels marked as `multi`;
	 * array indexes are `enum stm32_adc_multi_mode`
	 */
	static u8 multi_msk[] = {
		0x00,
		0x22, 0x02, 0x00, 0x00, 0x20, 0x02, 0x02, 0x00, 0x20, /* DUAL */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x66, 0x06, 0x00, 0x00, 0x60, 0x06, 0x06, 0x00, 0x60  /* TRPL */
	};
	struct stm32_adc *adc;
	u32 msk, val;

	list_for_each_entry(adc, &com->adc_list, adc_list) {
		msk = 1 << adc->id;
		if (adc->injected)
			msk = msk << 4;
		if (msk & multi_msk[mm])
			adc->multi = true;
	}

	val = stm32_adc_common_readl(com, STM32F4_ADC_CCR);

	val &= ~STM32F4_CCR_DMA_MASK;
	val |= dm << STM32F4_CCR_DMA_SHIFT;

	val &= ~STM32F4_CCR_MULTI_MASK;
	val |= mm << STM32F4_CCR_MULTI_SHIFT;

	val |= STM32F4_CCR_DDS;

	stm32_adc_common_writel(com, STM32F4_ADC_CCR, val);

	return 0;
}

/* stm32f4_smpr_items : Channel-wise programmable sampling time */
static const char * const stm32f4_smpr_items[] = {
	[STM32F4_SMPR_3_CK_CYCLES] = "3_cycles",
	[STM32F4_SMPR_15_CK_CYCLES] = "15_cycles",
	[STM32F4_SMPR_28_CK_CYCLES] = "28_cycles",
	[STM32F4_SMPR_56_CK_CYCLES] = "56_cycles",
	[STM32F4_SMPR_84_CK_CYCLES] = "84_cycles",
	[STM32F4_SMPR_112_CK_CYCLES] = "112_cycles",
	[STM32F4_SMPR_144_CK_CYCLES] = "144_cycles",
	[STM32F4_SMPR_480_CK_CYCLES] = "480_cycles",
};

static const struct iio_enum stm32f4_smpr = {
	.items = stm32f4_smpr_items,
	.num_items = ARRAY_SIZE(stm32f4_smpr_items),
	.get = stm32_adc_get_smpr,
	.set = stm32_adc_set_smpr,
};

static const struct iio_chan_spec_ext_info stm32f4_adc_ext_info[] = {
	IIO_ENUM("smpr", IIO_SEPARATE, &stm32f4_smpr),
	IIO_ENUM_AVAILABLE("smpr", &stm32f4_smpr),
	IIO_ENUM("trigger_pol", IIO_SHARED_BY_ALL, &stm32_adc_trig_pol),
	{
		.name = "trigger_pol_available",
		.shared = IIO_SHARED_BY_ALL,
		.read = iio_enum_available_read,
		.private = (uintptr_t)&stm32_adc_trig_pol,
	},
	{},
};

static const struct stm32_adc_ops stm32f4_adc_ops = {
	.adc_info = stm32f4_adc_info,
	.ext_triggers = stm32f4_adc_ext_triggers,
	.jext_triggers = stm32f4_adc_jext_triggers,
	.adc_reginfo = &stm32f4_adc_reginfo,
	.ext_info = stm32f4_adc_ext_info,
	.highres = 12,
	.max_clock_rate = 36000000,
	.parse_dts_params = stm32f4_parse_dts_params,
	.clk_sel = stm32f4_adc_clk_sel,
	.prepare_conv = stm32f4_adc_prepare_conv,
	.start_conv = stm32f4_adc_start_conv,
	.stop_conv = stm32f4_adc_stop_conv,
	.recover = stm32f4_adc_recover,
	.is_started = stm32f4_adc_is_started,
	.regular_started = stm32f4_adc_regular_started,
	.injected_started = stm32f4_adc_injected_started,
	.multi_dma_sel = stm32f4_adc_multi_dma_sel,
};

static const struct of_device_id stm32f4_adc_of_match[] = {
	{ .compatible = "st,stm32f4-adc", .data = (void *)&stm32f4_adc_ops},
	{},
};
MODULE_DEVICE_TABLE(of, stm32f4_adc_of_match);

static struct platform_driver stm32f4_adc_driver = {
	.probe = stm32_adc_probe,
	.remove = stm32_adc_remove,
	.driver = {
		.name = "stm32f4-adc",
		.of_match_table = stm32f4_adc_of_match,
	},
};

module_platform_driver(stm32f4_adc_driver);

MODULE_AUTHOR("Fabrice Gasnier <fabrice.gasnier@st.com>");
MODULE_DESCRIPTION("STMicroelectronics STM32F4 ADC driver");
MODULE_LICENSE("GPL v2");
