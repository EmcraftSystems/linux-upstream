#ifndef _MACH_STM32_SRAM_H_
#define _MACH_STM32_SRAM_H_

#define SRAM_DATA	__attribute__((section (".sram.data")))
#define SRAM_TEXT	__attribute__((section (".sram.text"), __long_call__))

int stm32_bkpsram_init(void);
void *stm32_bkpsram_data_alloc(size_t size);

int stm32_sram_init(void);
int stm32_sram_relocate(void);
void *stm32_sram_data_alloc(size_t size);

#endif
