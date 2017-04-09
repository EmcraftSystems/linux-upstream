/*
 * arch/arm/mach-kinetis/pm.c
 * Kinetis PM code
 *
 * Copyright (C) 2012 Robert Brehm, <brehm@mci.sdu.dk>
 * Copyright (C) 2014-2017 Emcraft Systems
 * Vladimir Khusainov, Emcraft Systems, <vlad@emcraft.com>
 * Alexander Yurtsev, Emcraft Systems, <alex@emcraft.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/suspend.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <asm/irq.h>
#include <asm/atomic.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>

#include <mach/kinetis.h>
#include <mach/power.h>

/*
 * Assembler-level entry point to suspend the system.
 * The second symbol is a dummy entry point to mark
 * the end of the eSRAM-based suspend code.
 */
extern void kinetis_suspend_to_ram(void);
extern void kinetis_suspend_to_ram_end(void);

/*
 * Location in eSRAM where the low-level suspend code will run from
 */
#define KINETIS_SUSPEND_CODE_BASE	0x20002000

#define KINETIS_SUSPEND_ADDR(a)						\
	((void *) (KINETIS_SUSPEND_CODE_BASE +				\
	((unsigned char *) (a) - (unsigned char *) kinetis_suspend_to_ram)))

/*
 * Various bit fields in various hardware registers
 */
/* Auto Trim value  */
#define KINETIS_MCG_ATC_FCRDIV_MSK	(7 << 1)

/*
 * Local to the module globals
 */
static u32 kinetis_pm_sim_fcfg1;

/*
 * Validate suspend state
 * @state		State being entered
 * @returns		1->valid, 0->invalid
 */
static int kinetis_pm_valid(suspend_state_t state)
{
	int ret;

	switch (state) {

	case PM_SUSPEND_ON:
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}

/*
 * Prepare system hardware to suspend
 */
static void kinetis_pm_prepare_to_suspend(void)
{
	/*
	 * Disable internal Flash
	 */
	kinetis_pm_sim_fcfg1 = KINETIS_SIM_RD(fcfg1);
	KINETIS_SIM_WR(fcfg1, KINETIS_SIM_FCFG1_FTFDIS);

	/*
	 * WDOG_STCTLH[STOPEN] = 0.
	 * Disable Watchdog on STOP.
	 */
	writew(readw(&KINETIS_WDOG->stctrlh) & ~KINETIS_WDOG_STCTRLH_STOPEN,
		 &KINETIS_WDOG->stctrlh);

	/*
	 * Put USB voltage regulator in stand-by on STOP
	 */
	KINETIS_SIM_WR(sopt1cfg, KINETIS_SIM_RD(sopt1cfg) |
			KINETIS_SIM_SOPT1CFG_USSWE);
	KINETIS_SIM_WR(sopt1, KINETIS_SIM_RD(sopt1) |
			KINETIS_SIM_SOPT1_USBSSTBY);

	/*
	 * Disable internal reference clock on STOP.
	 */
	KINETIS_MCG_WR_CONST(c1,
			KINETIS_MCG_RD_CONST(c1) & ~KINETIS_MCG_C1_IREFSTEN);

	/*
	 * Disable FLL (or PLL) in bypass modes.
	 */
	KINETIS_MCG_WR_CONST(c2, KINETIS_MCG_RD_CONST(c2) | KINETIS_MCG_C2_LP);

	/*
	 * Disable PLL0 on STOP.
	 */
	KINETIS_MCG_WR_CONST(c5, KINETIS_MCG_RD_CONST(c5) &
			~KINETIS_MCG_C5_PLLSTEN0_MSK);

	/*
	 * Disable PLL1 on STOP.
	 */
	KINETIS_MCG_WR_CONST(c11, KINETIS_MCG_RD_CONST(c11) &
			~KINETIS_MCG_C11_PLLSTEN1_MSK);

	/*
	 * Disable OSC0 on STOP.
	 */
	writeb(readb(&KINETIS_OSC0->cr) & ~KINETIS_OSCx_CR_EREFSTEN,
		&KINETIS_OSC0->cr);

	/*
	 * Disable OSC1 on STOP.
	 */
	kinetis_periph_enable(KINETIS_CG_OSC1, 1);
	writeb(readb(&KINETIS_OSC1->cr) & ~KINETIS_OSCx_CR_EREFSTEN,
		&KINETIS_OSC1->cr);
	kinetis_periph_enable(KINETIS_CG_OSC1, 0);

	/*
	 * Set Fast Clock Reference Divider to Divide Factor 1.
	 * Select Fast Clock as the internal clock reference.
	 */
	KINETIS_MCG_WR_CONST(atc, KINETIS_MCG_RD_CONST(atc) &
			~KINETIS_MCG_ATC_FCRDIV_MSK);
	KINETIS_MCG_WR_CONST(c2, KINETIS_MCG_RD_CONST(c2) |
			KINETIS_MCG_C2_IRCS);

	/*
	 * Disable peripheral clocks, having stored
	 * the current state of the clock registers
	 */
	kinetis_periph_push();
	kinetis_periph_enable(KINETIS_CG_OSC1, 0);
	kinetis_periph_enable(KINETIS_CG_DAC0, 0);
	kinetis_periph_enable(KINETIS_CG_DAC1, 0);
	kinetis_periph_enable(KINETIS_CG_RNGA, 0);
	kinetis_periph_enable(KINETIS_CG_FLEXCAN1, 0);
	kinetis_periph_enable(KINETIS_CG_NFC, 0);
	kinetis_periph_enable(KINETIS_CG_SAI1, 0);
	kinetis_periph_enable(KINETIS_CG_FTM2, 0);
	kinetis_periph_enable(KINETIS_CG_FTM3, 0);
	kinetis_periph_enable(KINETIS_CG_EWM, 0);
	kinetis_periph_enable(KINETIS_CG_CMT, 0);
	kinetis_periph_enable(KINETIS_CG_CMP, 0);
	kinetis_periph_enable(KINETIS_CG_VREF, 0);
	kinetis_periph_enable(KINETIS_CG_LLWU, 0);
	kinetis_periph_enable(KINETIS_CG_LPTIMER, 0);
	kinetis_periph_enable(KINETIS_CG_DRYICE, 0);
	kinetis_periph_enable(KINETIS_CG_DRYICESECREG, 0);
	kinetis_periph_enable(KINETIS_CG_TSI, 0);
	kinetis_periph_enable(KINETIS_CG_FLEXCAN0, 0);
	kinetis_periph_enable(KINETIS_CG_SAI0, 0);
	kinetis_periph_enable(KINETIS_CG_CRC, 0);
	kinetis_periph_enable(KINETIS_CG_USBDCD, 0);
	kinetis_periph_enable(KINETIS_CG_PDB, 0);
	kinetis_periph_enable(KINETIS_CG_PIT, 0);
	kinetis_periph_enable(KINETIS_CG_FTM0, 0);
	kinetis_periph_enable(KINETIS_CG_FTM1, 0);
	kinetis_periph_enable(KINETIS_CG_FLEXBUS, 0);
	kinetis_periph_enable(KINETIS_CG_MPU, 0);
	kinetis_periph_enable(KINETIS_CG_RTC, 0);
	kinetis_periph_enable(KINETIS_CG_ENET, 0);
	kinetis_periph_enable(KINETIS_CG_DMA, 0);
	kinetis_periph_enable(KINETIS_CG_DMAMUX0, 0);
	kinetis_periph_enable(KINETIS_CG_DMAMUX1, 0);

	kinetis_periph_print_all();
}

/*
 * Prepare system hardware to resume
 */
static void kinetis_pm_prepare_to_resume(void)
{
	/*
	 * Restore peripheral clocks
	 */
	kinetis_periph_pop();

	/*
	 * Restore the previous state of the internal Flash
	 */
	KINETIS_SIM_WR(fcfg1, kinetis_pm_sim_fcfg1);
}

/*
 * Enter suspend
 * @state		State being entered
 * @returns		0->success, <0->error code
 */
static int kinetis_pm_enter(suspend_state_t state)
{
	void (* ptr)(void);
	int ret = 0;

	/*
	 * Prepare the system hardware to suspend
	 */
	kinetis_pm_prepare_to_suspend();

	/*
	 * Allow very low power modes.
	 * Switch to Very Low Power Stop mode on WFI.
	 * Enter DEEP SLEEP (vs SLEEP) on WFI.
	 */
	writeb(KINETIS_SMC_PMRPOT_AVLP, &KINETIS_SMC->pmprot);
	writeb(KINETIS_SMC_PMCTRL_STOPM_VLPS, &KINETIS_SMC->pmctrl);
	writel(readl(&KINETIS_SCB->scr) | KINETIS_SCB_SCR_DEEPSLEEP,
		&KINETIS_SCB->scr);

	/*
	 * Jump to suspend code in SRAM
	 */
	ptr = (void *)(KINETIS_SUSPEND_CODE_BASE + 1);
	ptr();

	/*
	 * Switch to Normal Stop mode on WFI.
	 * Disable DEEP SLEEP on WFI.
	 * These are the setting the generic Linux idle() relies upon.
	 */
	writeb(KINETIS_SMC_PMCTRL_STOPM_RUN, &KINETIS_SMC->pmctrl);
	writel(readl(&KINETIS_SCB->scr) & ~KINETIS_SCB_SCR_DEEPSLEEP,
		&KINETIS_SCB->scr);

	/*
	 * Prepare the system hardware to resume
	 */
	kinetis_pm_prepare_to_resume();

	return ret;
}

/*
 * Power Management operations
 */
static struct platform_suspend_ops kinetis_pm_ops = {
	.valid = kinetis_pm_valid,
	.enter = kinetis_pm_enter,
};

/*
 * Device data structure
 */
static struct platform_driver kinetis_pm_driver = {
	.driver = {
		   .name = "kinetis_pm",
	},
};

/*
 * Driver init
 * @returns		0->success, <0->error code
 */
static int __init kinetis_pm_init(void)
{
	int ret;

	/*
	 * Relocate low-level suspend code to SRAM.
	 * It will run with no DDR available.
	 */
	memcpy((unsigned char *) KINETIS_SUSPEND_CODE_BASE,
		(unsigned char *) kinetis_suspend_to_ram,
		(unsigned char *) kinetis_suspend_to_ram_end -
		(unsigned char *) kinetis_suspend_to_ram);

	/*
	 * Register the PM driver
	 */
	if (platform_driver_register(&kinetis_pm_driver) != 0) {
		printk(KERN_ERR "kinetis_pm_driver register failed\n");
		ret = -ENODEV;
		goto Done;
	}

	/*
	 * Register PM operations
	 */
	suspend_set_ops(&kinetis_pm_ops);

	/*
	 * Here, means success
	 */
	printk(KERN_INFO "Power Management for Freescale Kinetis\n");
	ret = 0;
	goto Done;

Done:
	return ret;
}

/*
 * Driver clean-up
 */
static void __exit kinetis_pm_cleanup(void)
{
	platform_driver_unregister(&kinetis_pm_driver);
}

module_init(kinetis_pm_init);
module_exit(kinetis_pm_cleanup);

MODULE_AUTHOR("Robert Brehm");
MODULE_DESCRIPTION("Kinetis PM driver");
MODULE_LICENSE("GPL");


