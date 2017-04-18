#ifndef _MACH_STM32_PWR_H_
#define _MACH_STM32_PWR_H_

/*
 * PWR registers bits and masks
 */
#define STM32_PWR_CR		0x00
#define STM32_PWR_CSR		0x04

#define STM32_PWR_CR_LPDS	(1 << 0)
#define STM32_PWR_CR_DBP	(1 << 8)
#define STM32_PWR_CR_FPDS	(1 << 9)
#define STM32_PWR_CR_LPUDS	(1 << 10)
#define STM32_PWR_CR_ADCDC1	(1 << 13)
#define STM32_PWR_CR_ODEN	(1 << 16)
#define STM32_PWR_CR_ODSWEN	(1 << 17)
#define STM32_PWR_CR_UDEN	(0x3 << 18)
#define STM32_PWR_CR_MODE	(STM32_PWR_CR_UDEN | STM32_PWR_CR_LPUDS | \
				 STM32_PWR_CR_FPDS | STM32_PWR_CR_LPDS)

#define STM32_PWR_CSR_BRE	(1 << 9)
#define STM32_PWR_CSR_ODRDY	(1 << 16)

#endif
