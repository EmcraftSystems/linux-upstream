/*
 * Common defines for v7m cpus
 */
#define V7M_SCS_ICTR			IOMEM(0xe000e004)
#define V7M_SCS_ICTR_INTLINESNUM_MASK		0x0000000f

#define BASEADDR_V7M_SCB		IOMEM(0xe000ed00)

#define V7M_SCB_CPUID			0x00

#define V7M_SCB_ICSR			0x04
#define V7M_SCB_ICSR_PENDSVSET			(1 << 28)
#define V7M_SCB_ICSR_PENDSVCLR			(1 << 27)
#define V7M_SCB_ICSR_PENDSTCLR			(1 << 25)
#define V7M_SCB_ICSR_RETTOBASE			(1 << 11)

#define V7M_SCB_VTOR			0x08

#define V7M_SCB_AIRCR			0x0c
#define V7M_SCB_AIRCR_VECTKEY			(0x05fa << 16)
#define V7M_SCB_AIRCR_SYSRESETREQ		(1 << 2)

#define V7M_SCB_SCR			0x10
#define V7M_SCB_SCR_SLEEPDEEP			(1 << 2)

#define V7M_SCB_CCR			0x14
#define V7M_SCB_CCR_STKALIGN			(1 << 9)

#define V7M_SCB_SHPR2			0x1c
#define V7M_SCB_SHPR3			0x20

#define V7M_SCB_SHCSR			0x24
#define V7M_SCB_SHCSR_USGFAULTENA		(1 << 18)
#define V7M_SCB_SHCSR_BUSFAULTENA		(1 << 17)
#define V7M_SCB_SHCSR_MEMFAULTENA		(1 << 16)

#define V7M_xPSR_FRAMEPTRALIGN			0x00000200
#define V7M_xPSR_EXCEPTIONNO			0x000001ff

/*
 * When branching to an address that has bits [31:28] == 0xf an exception return
 * occurs. Bits [27:5] are reserved (SBOP). If the processor implements the FP
 * extension Bit [4] defines if the exception frame has space allocated for FP
 * state information, SBOP otherwise. Bit [3] defines the mode that is returned
 * to (0 -> handler mode; 1 -> thread mode). Bit [2] defines which sp is used
 * (0 -> msp; 1 -> psp). Bits [1:0] are fixed to 0b01.
 */
#define EXC_RET_STACK_MASK			0x00000004
#define EXC_RET_THREADMODE_PROCESSSTACK		0xfffffffd

#ifndef __ASSEMBLY__

enum reboot_mode;

void armv7m_restart(enum reboot_mode mode, const char *cmd);

/*
 * Run ARM V7M SysTick timer with `usec` timeout, and call `cb` (with `arg`)
 * upon completion. SysTick will run periodically while `cb` returns 0
 */
int armv7m_systick_run(u32 usec, int (*func)(void *arg), void *arg);

/*
 * Stop ARM V7M SysTick timer
 */
int armv7m_systick_stop(void);

/*
 * ARM V7M SysTick Interrupt save/restore macros
 */
void _armv7m_systick_irq_upd(unsigned long *flags, int save);
#define armv7m_systick_irq_save(f)	_armv7m_systick_irq_upd(&(f), 1)
#define armv7m_systick_irq_restore(f)	_armv7m_systick_irq_upd(&(f), 0)

#endif /* __ASSEMBLY__ */
