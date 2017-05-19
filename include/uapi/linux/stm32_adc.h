/*
 * Copyright (C) 2017 Emcraft Systems, <www.emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
 *
 * STM32 ADCx definitions shared with user-space apps
 *
 * License terms: GNU General Public License (GPL), version 2
 */
#ifndef _STM32_ADC_H_
#define _STM32_ADC_H_

/*
 * Maximum number of channels in sequence per measurement
 */
#define STM32_ADC_SEQ_MAX	16

/*
 * IOCTLs
 */
#define STM32_ADC_IOC_MAGIC	'A'
#define STM32_ADC_SINGLE_SEQ	_IO(STM32_ADC_IOC_MAGIC, 0)
#define STM32_ADC_LOOP_SEQ	_IO(STM32_ADC_IOC_MAGIC, 1)
#define STM32_ADC_STOP_SEQ	_IO(STM32_ADC_IOC_MAGIC, 2)

/*
 * This structure is available via mmap():
 * - data[STM32_ADCx_MEAS][STM32_ADCx_CHAN]: array of measured values on ADCx
 */
struct stm32_adc_mmap {
	unsigned int		data[1][1];
};

#endif /* _STM32_ADC_H_ */
