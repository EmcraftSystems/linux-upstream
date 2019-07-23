/*
 * STM32 On-Chip Real Time Clock Driver
 *
 * (C) Copyright 2012-2015
 * Emcraft Systems, <www.emcraft.com>
 * Alexander Potashev <aspotashev@emcraft.com>
 * Yuri Tikhonov <yur@emcraft.com>
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

#include <linux/bcd.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/clk.h>

/******************************************************************************
 * RTC register definitions
 ******************************************************************************/
/*
 * RTC time register (RTC_TR)
 */
/* Seconds in BCD format */
#define STM32_RTC_TR_SEC_BITS		0
#define STM32_RTC_TR_SEC_MSK		(0x7f << STM32_RTC_TR_SEC_BITS)
/* Minutes in BCD format */
#define STM32_RTC_TR_MIN_BITS		8
#define STM32_RTC_TR_MIN_MSK		(0x7f << STM32_RTC_TR_MIN_BITS)
/* Hours in BCD format */
#define STM32_RTC_TR_HOUR_BITS		16
#define STM32_RTC_TR_HOUR_MSK		(0x3f << STM32_RTC_TR_HOUR_BITS)

/*
 * RTC date register (RTC_DR)
 */
/* Day of month in BCD format (1..31) */
#define STM32_RTC_DR_MDAY_BITS		0
#define STM32_RTC_DR_MDAY_MSK		(0x3f << STM32_RTC_DR_MDAY_BITS)
/* Month in BCD format (1..12) */
#define STM32_RTC_DR_MON_BITS		8
#define STM32_RTC_DR_MON_MSK		(0x1f << STM32_RTC_DR_MON_BITS)
/* Year's last two digits in BCD format */
#define STM32_RTC_DR_YEAR_BITS		16
#define STM32_RTC_DR_YEAR_MSK		(0xff << STM32_RTC_DR_YEAR_BITS)
/* Day of week (1=Monday, 7=Sunday) */
#define STM32_RTC_DR_WDAY_BITS		13
#define STM32_RTC_DR_WDAY_MSK		(7 << STM32_RTC_DR_WDAY_BITS)

/*
 * RTC control register (RTC_CR)
 */
/* Wakeup timer interrupt enable */
#define STM32_RTC_CR_WUTIE_MSK		(1 << 14)
/* Alarm B interrupt enable */
#define STM32_RTC_CR_ALRBIE_MSK		(1 << 13)
/* Alarm A interrupt enable */
#define STM32_RTC_CR_ALRAIE_MSK		(1 << 12)
/* Wakeup timer enable */
#define STM32_RTC_CR_WUTE_MSK		(1 << 10)
/* Alarm B enable */
#define STM32_RTC_CR_ALRBE_MSK		(1 << 9)
/* Alarm A enable */
#define STM32_RTC_CR_ALRAE_MSK		(1 << 8)
/* Hour format (0=24h, 1=AM/PM) */
#define STM32_RTC_CR_FMT_MSK		(1 << 6)
/* Wakeup clock selection */
#define STM32_RTC_CR_WUCKSEL_BITS	0
#define STM32_RTC_CR_WUCKSEL_MSK	(7 << STM32_RTC_CR_WUCKSEL_BITS)
#define STM32_RTC_CR_WUCKSEL_DIV2	(3 << STM32_RTC_CR_WUCKSEL_BITS)

/*
 * RTC initialization and status register (RTC_ISR)
 */
/* Wakeup timer flag */
#define STM32_RTC_ISR_WUTF_MSK		(1 << 10)
/* Alarm B flag */
#define STM32_RTC_ISR_ALRBF_MSK		(1 << 9)
/* Alarm A flag */
#define STM32_RTC_ISR_ALRAF_MSK		(1 << 8)
/* Initialization mode control bit */
#define STM32_RTC_ISR_INIT_MSK		(1 << 7)
/* Initialization mode flag */
#define STM32_RTC_ISR_INITF_MSK		(1 << 6)
/* Registers synchronization flag */
#define STM32_RTC_ISR_RSF_MSK		(1 << 5)
/* Initialization status flag */
#define STM32_RTC_ISR_INITS_MSK		(1 << 4)
/* Wakeup timer write flag */
#define STM32_RTC_ISR_WUTWF_MSK		(1 << 2)
/* Alarm B write flag */
#define STM32_RTC_ISR_ALRBWF_MSK	(1 << 1)
/* Alarm A write flag */
#define STM32_RTC_ISR_ALRAWF_MSK	(1 << 0)

/*
 * RTC alarm register (RTC_ALRMAR, RTC_ALRMBR)
 */
/* Date and day don't care in alarm comparison */
#define STM32_RTC_ALRMR_DATEMASK_MSK	(1 << 31)
/* Hours don't care in alarm comparison */
#define STM32_RTC_ALRMR_HOURMASK_MSK	(1 << 23)
/* Minutes don't care in alarm comparison */
#define STM32_RTC_ALRMR_MINMASK_MSK	(1 << 15)
/* Seconds don't care in alarm comparison */
#define STM32_RTC_ALRMR_SECMASK_MSK	(1 << 7)
/* Week day selection: 0=day in month; 1=day of week */
#define STM32_RTC_ALRMR_WDSEL_MSK	(1 << 30)
/* Date/day of week */
#define STM32_RTC_ALRMR_DAY_BITS	24
#define STM32_RTC_ALRMR_MDAY_MSK	(0x3f << STM32_RTC_ALRMR_DAY_BITS)
#define STM32_RTC_ALRMR_WDAY_MSK	(0x7 << STM32_RTC_ALRMR_DAY_BITS)
/* Hours in BCD format */
#define STM32_RTC_ALRMR_HOUR_BITS	16
#define STM32_RTC_ALRMR_HOUR_MSK	(0x3f << STM32_RTC_ALRMR_HOUR_BITS)
/* Minutes in BCD format */
#define STM32_RTC_ALRMR_MIN_BITS	8
#define STM32_RTC_ALRMR_MIN_MSK		(0x7f << STM32_RTC_ALRMR_MIN_BITS)
/* Seconds in BCD format */
#define STM32_RTC_ALRMR_SEC_BITS	0
#define STM32_RTC_ALRMR_SEC_MSK		(0x7f << STM32_RTC_ALRMR_SEC_BITS)

/******************************************************************************
 * STM32Fx RTC configuration peripherals definitions
 ******************************************************************************/
/*
 * PWR power control register
 */
#define STM32_PWR_CR			0x00
#define STM32_PWR_CR_DBP_MSK		(1 << 8)

/*
 * RCC Backup domain control register
 */
#define STM32_RCC_BDCR			0x70
/* RTC clock enable */
#define STM32_RCC_BDCR_RTCEN_MSK	(1 << 15)
/* RTC clock source selection */
#define STM32_RCC_BDCR_RTCSEL_BITS	8
#define STM32_RCC_BDCR_RTCSEL_MSK	(3 << STM32_RCC_BDCR_RTCSEL_BITS)
#define STM32_RCC_BDCR_RTCSEL_LSE	(1 << STM32_RCC_BDCR_RTCSEL_BITS)
/* LSE Drive bits, STM32F7 only */
#define STM32_RCC_BDCR_LSEDRV_BITS	3
#define STM32_RCC_BDCR_LSEDRV_MSK	(3 << STM32_RCC_BDCR_LSEDRV_BITS)
#define STM32_RCC_BDCR_LSEDRV_LOW	(0 << STM32_RCC_BDCR_LSEDRV_BITS)
#define STM32_RCC_BDCR_LSEDRV_MEDHI	(1 << STM32_RCC_BDCR_LSEDRV_BITS)
#define STM32_RCC_BDCR_LSEDRV_MEDLO	(2 << STM32_RCC_BDCR_LSEDRV_BITS)
#define STM32_RCC_BDCR_LSEDRV_HIGH	STM32_RCC_BDCR_LSEDRV_MSK
/* External low-speed oscillator bypass */
#define STM32_RCC_BDCR_LSEBYP_MSK	(1 << 2)
/* External low-speed oscillator ready */
#define STM32_RCC_BDCR_LSERDY_MSK	(1 << 1)
/*  External low-speed oscillator enable */
#define STM32_RCC_BDCR_LSEON_MSK	(1 << 0)

/*
 * EXTI registers
 */
#define STM32_EXTI_IMR			0x00	/* Interrupt mask reg */
#define STM32_EXTI_RTSR			0x08	/* Rising trig selection reg */
#define STM32_EXTI_PR			0x14	/* Pending reg */

#define STM32_EXTI_LINE_RTC_ALARM	(1 << 17)
#define STM32_EXTI_LINE_RTC_WAKEUP	(1 << 22)

/*
 * STM32 RTC register map
 */
struct stm32_rtc_regs {
	u32 tr;		/* RTC time register */
	u32 dr;		/* RTC date register */
	u32 cr;		/* RTC control register */
	u32 isr;	/* RTC initialization and status register */
	u32 prer;	/* RTC prescaler register */
	u32 wutr;	/* RTC wakeup timer register */
	u32 calibr;	/* RTC calibration register */
	u32 alrmar;	/* RTC alarm A register */
	u32 alrmbr;	/* RTC alarm B register */
	u32 wpr;	/* RTC write protection register */
	u32 rsv0[2];
	u32 tstr;	/* RTC time stamp time register */
	u32 tsdr;	/* RTC time stamp date register */
	u32 rsv1[2];
	u32 tafcr;	/* RTC tamper and alternate function configuration */
	u32 rsv2[3];
	u32 bkpr[20];	/* RTC backup registers */
};

/*
 * STM32 RTC descriptor
 */
struct stm32_rtc {
	struct rtc_device			*rtc_dev;
	volatile struct stm32_rtc_regs __iomem	*regs;
	int					irq_alarm;
	int					irq_wakeup;
	spinlock_t				lock;
	void					*private;

	int	(*init)(struct stm32_rtc *rtc, struct device *dev);
};

/*
 * Device specific OF data
 */
struct stm32_rtc_of_data {
	int	(*init)(struct stm32_rtc *rtc, struct device *dev);
};

/*
 * Disable or enable write-protection of RTC module registers
 */
static void stm32_rtc_write_enable(struct stm32_rtc *rtc, int enable)
{
	if (enable) {
		/* Disable write protection */
		rtc->regs->wpr = 0xca;
		rtc->regs->wpr = 0x53;
	} else {
		/* Enable write protection */
		rtc->regs->wpr = 0xff;
	}
}

/*
 * Enter or exit the initialization mode
 */
static void stm32_rtc_enable_init_mode(struct stm32_rtc *rtc, int enable)
{
	if (enable) {
		/* Enter initialization mode */
		if (!(rtc->regs->isr & STM32_RTC_ISR_INITF_MSK)) {
			/* Switch to the initialization mode */
			rtc->regs->isr = STM32_RTC_ISR_INIT_MSK;
			/* Wait until the initialization mode is entered */
			while (!(rtc->regs->isr & STM32_RTC_ISR_INITF_MSK));
		}
	} else {
		/* Exit initialization mode */
		rtc->regs->isr &= ~STM32_RTC_ISR_INIT_MSK;
	}
}

/*
 * RTC alarm interrupt handler
 */
static irqreturn_t stm32_rtc_alarm_irq(int irq, void *dev_id)
{
	unsigned long events = 0;
	struct stm32_rtc *rtc = (struct stm32_rtc *)dev_id;
	u32 status;
	u32 enabled_irqs;

	spin_lock(&rtc->lock);

	status = rtc->regs->isr;
	enabled_irqs = rtc->regs->cr;

	stm32_rtc_write_enable(rtc, 1);
	/* clear event flags, otherwise new events won't be received */
	rtc->regs->isr &= ~(status & (STM32_RTC_ISR_ALRBF_MSK |
				      STM32_RTC_ISR_ALRAF_MSK));
	stm32_rtc_write_enable(rtc, 0);

	if ((status & STM32_RTC_ISR_ALRAF_MSK) &&
	    (enabled_irqs & STM32_RTC_CR_ALRAIE_MSK)) {
		/* Normal alarm interrupt */
		events |= (RTC_AF | RTC_IRQF);
	}

	if ((status & STM32_RTC_ISR_ALRBF_MSK) &&
	    (enabled_irqs & STM32_RTC_CR_ALRBIE_MSK)) {
		/* Update interrupt (1 Hz) */
		events |= (RTC_UF | RTC_IRQF);
	}

	if (events)
		rtc_update_irq(rtc->rtc_dev, 1, events);

	spin_unlock(&rtc->lock);

	return events ? IRQ_HANDLED : IRQ_NONE;
}

/*
 * RTC wakeup interrupt handler
 */
static irqreturn_t stm32_rtc_wakeup_irq(int irq, void *dev_id)
{
	struct stm32_rtc *rtc = (struct stm32_rtc *)dev_id;
	irqreturn_t rv;

	spin_lock(&rtc->lock);

	if (!(rtc->regs->isr & STM32_RTC_ISR_WUTF_MSK) ||
	    !(rtc->regs->cr & STM32_RTC_CR_WUTIE_MSK)) {
		rv = IRQ_NONE;
		goto out;
	}

	/* Clear event flag, otherwise new events won't be received */
	stm32_rtc_write_enable(rtc, 1);
	rtc->regs->isr &= ~STM32_RTC_ISR_WUTF_MSK;
	stm32_rtc_write_enable(rtc, 0);

	/* Pass periodic interrupt event to the kernel */
	rtc_update_irq(rtc->rtc_dev, 1, RTC_PF | RTC_IRQF);

	rv = IRQ_HANDLED;
out:
	spin_unlock(&rtc->lock);
	return rv;
}

/*
 * Enable or disable the alarm interrupt for a given alarm.
 *
 * alarm = 0 selects Alarm A.
 * alarm = 1 selects Alarm B.
 */
static void stm32_alarm_irq_enable(struct stm32_rtc *rtc, unsigned int alarm,
				   unsigned int enable)
{
	u32 irq_enable_msk =
		alarm ? STM32_RTC_CR_ALRBIE_MSK : STM32_RTC_CR_ALRAIE_MSK;
	u32 irq_flag_msk =
		alarm ? STM32_RTC_ISR_ALRBF_MSK : ~STM32_RTC_ISR_ALRAF_MSK;

	stm32_rtc_write_enable(rtc, 1);

	/* Alarm interrupt enable or disable */
	if (enable)
		rtc->regs->cr |= irq_enable_msk;
	else
		rtc->regs->cr &= ~irq_enable_msk;

	/* Clear alarm event flag, otherwise new events won't be received */
	rtc->regs->isr &= ~irq_flag_msk;

	stm32_rtc_write_enable(rtc, 0);
}

/*
 * Enable or disable alarm interrupts
 */
static int stm32_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct stm32_rtc *rtc = dev_get_drvdata(dev);

	spin_lock_irq(&rtc->lock);

	/* We expose Alarm A to the kernel */
	stm32_alarm_irq_enable(rtc, 0, enabled);

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Configure and enable an RTC alarm
 *
 * alarm = 0 selects Alarm A.
 * alarm = 1 selects Alarm B.
 *
 * "mday", "wday", "hour", "min" and "sec" can be set to (-1) to bypass
 * comparison of the respective component of time and date with the current
 * time.
 */
static void stm32_setup_alarm(struct stm32_rtc *rtc, unsigned int alarm,
	int mday, int wday, int hour, int min, int sec)
{
	u32 enable_msk =
		alarm ? STM32_RTC_CR_ALRBE_MSK : STM32_RTC_CR_ALRAE_MSK;
	u32 write_flag_msk =
		alarm ? STM32_RTC_ISR_ALRBWF_MSK : STM32_RTC_ISR_ALRAWF_MSK;
	u32 tmp;

	stm32_rtc_write_enable(rtc, 1);

	/* Disable alarm */
	rtc->regs->cr &= ~enable_msk;
	/* Poll write flag to make sure the access to registers is allowed */
	while (!(rtc->regs->isr & write_flag_msk));

	/* Prepare the value for the RTC alarm register */
	tmp = 0;

	/*
	 * Set date or day of week
	 */
	if (mday < 0 && wday < 0) {	/* Day is don't care */
		tmp |= STM32_RTC_ALRMR_DATEMASK_MSK;
	} else if (mday > 0) {		/* Date is selected (ignoring "wday") */
		tmp |= (bin2bcd(mday) << STM32_RTC_ALRMR_DAY_BITS) &
			STM32_RTC_ALRMR_MDAY_MSK;
	} else {			/* Day of week is selected */
		tmp |= STM32_RTC_ALRMR_WDSEL_MSK;
		tmp |= ((wday ? wday : 7) << STM32_RTC_ALRMR_DAY_BITS) &
			STM32_RTC_ALRMR_WDAY_MSK;
	}

	/*
	 * Set hours
	 */
	if (hour >= 0) {
		tmp |= (bin2bcd(hour) << STM32_RTC_ALRMR_HOUR_BITS) &
			STM32_RTC_ALRMR_HOUR_MSK;
	} else {
		/* Skip hours comparison */
		tmp |= STM32_RTC_ALRMR_HOURMASK_MSK;
	}

	/*
	 * Set minutes
	 */
	if (min >= 0) {
		tmp |= (bin2bcd(min) << STM32_RTC_ALRMR_MIN_BITS) &
			STM32_RTC_ALRMR_MIN_MSK;
	} else {
		/* Skip minutes comparison */
		tmp |= STM32_RTC_ALRMR_MINMASK_MSK;
	}

	/*
	 * Set seconds
	 */
	if (sec >= 0) {
		tmp |= (bin2bcd(sec) << STM32_RTC_ALRMR_SEC_BITS) &
			STM32_RTC_ALRMR_SEC_MSK;
	} else {
		/* Skip seconds comparison */
		tmp |= STM32_RTC_ALRMR_SECMASK_MSK;
	}

	/* Write to the alarm register */
	*(alarm ? &rtc->regs->alrmbr : &rtc->regs->alrmar) = tmp;

	/* Enable alarm */
	rtc->regs->cr |= enable_msk;

	stm32_rtc_write_enable(rtc, 0);
}

/*
 * Read current time and date from the RTC
 */
static int stm32_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct stm32_rtc *rtc = dev_get_drvdata(dev);

	spin_lock_irq(&rtc->lock);

	/* Time in BCD format */
	tm->tm_sec = bcd2bin((rtc->regs->tr & STM32_RTC_TR_SEC_MSK) >>
		STM32_RTC_TR_SEC_BITS);
	tm->tm_min = bcd2bin((rtc->regs->tr & STM32_RTC_TR_MIN_MSK) >>
		STM32_RTC_TR_MIN_BITS);
	tm->tm_hour = bcd2bin((rtc->regs->tr & STM32_RTC_TR_HOUR_MSK) >>
		STM32_RTC_TR_HOUR_BITS);

	/* Date in BCD format */
	tm->tm_mday = bcd2bin((rtc->regs->dr & STM32_RTC_DR_MDAY_MSK) >>
		STM32_RTC_DR_MDAY_BITS);
	tm->tm_mon = bcd2bin((rtc->regs->dr & STM32_RTC_DR_MON_MSK) >>
		STM32_RTC_DR_MON_BITS) - 1;
	/* Assume the year is between 2001 and 2099 */
	tm->tm_year = bcd2bin((rtc->regs->dr & STM32_RTC_DR_YEAR_MSK) >>
		STM32_RTC_DR_YEAR_BITS) + 100;
	/* Kernel thinks 0=Sunday. RTC thinks 7=Sunday, 0=invalid */
	tm->tm_wday = ((rtc->regs->dr & STM32_RTC_DR_WDAY_MSK) >>
		STM32_RTC_DR_WDAY_BITS) % 7;

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Write time and date to the RTC
 */
static int stm32_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct stm32_rtc *rtc = dev_get_drvdata(dev);

	spin_lock_irq(&rtc->lock);

	/* Disable write-protection; enter initialization mode */
	stm32_rtc_write_enable(rtc, 1);
	stm32_rtc_enable_init_mode(rtc, 1);

	/* Time in BCD format */
	rtc->regs->tr =
		(rtc->regs->tr & ~(STM32_RTC_TR_SEC_MSK |
			STM32_RTC_TR_MIN_MSK | STM32_RTC_TR_HOUR_MSK)) |
		((bin2bcd(tm->tm_sec) << STM32_RTC_TR_SEC_BITS) &
			STM32_RTC_TR_SEC_MSK) |
		((bin2bcd(tm->tm_min) << STM32_RTC_TR_MIN_BITS) &
			STM32_RTC_TR_MIN_MSK) |
		((bin2bcd(tm->tm_hour) << STM32_RTC_TR_HOUR_BITS) &
			STM32_RTC_TR_HOUR_MSK);
	/* Date in BCD format */
	rtc->regs->dr =
		(rtc->regs->dr &
			~(STM32_RTC_DR_MDAY_MSK | STM32_RTC_DR_MON_MSK |
			STM32_RTC_DR_YEAR_MSK | STM32_RTC_DR_WDAY_MSK)) |
		((bin2bcd(tm->tm_mday) << STM32_RTC_DR_MDAY_BITS) &
			STM32_RTC_DR_MDAY_MSK) |
		((bin2bcd(tm->tm_mon + 1) << STM32_RTC_DR_MON_BITS) &
			STM32_RTC_DR_MON_MSK) |
		((bin2bcd(tm->tm_year % 100) << STM32_RTC_DR_YEAR_BITS) &
			STM32_RTC_DR_YEAR_MSK) |
		(((tm->tm_wday ? tm->tm_wday : 7) << STM32_RTC_DR_WDAY_BITS) &
			STM32_RTC_DR_WDAY_MSK);

	/* Exit initialization mode; enable write-protection */
	stm32_rtc_enable_init_mode(rtc, 0);
	stm32_rtc_write_enable(rtc, 0);

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Read the time and date the alarm (Alarm A) is set to
 */
static int stm32_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct stm32_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &alrm->time;
	u32 reg;

	spin_lock_irq(&rtc->lock);

	reg = rtc->regs->alrmar;

	/* Comparison of year and month not supported by STM32 RTC alarm */
	tm->tm_year = -1;
	tm->tm_mon = -1;

	if (reg & STM32_RTC_ALRMR_DATEMASK_MSK) {
		/* Alarm triggers every day */
		tm->tm_mday = -1;
		tm->tm_wday = -1;
	} else if (reg & STM32_RTC_ALRMR_WDSEL_MSK) {
		/* Alarm is set to a day of week */
		tm->tm_mday = -1;
		/* 0 = Sun */
		tm->tm_wday = ((reg & STM32_RTC_ALRMR_WDAY_MSK) >>
			STM32_RTC_ALRMR_DAY_BITS) % 7;
	} else {
		/* Alarm is set to a day of month */
		tm->tm_mday = bcd2bin((reg & STM32_RTC_ALRMR_MDAY_MSK) >>
			STM32_RTC_ALRMR_DAY_BITS);
		tm->tm_wday = -1;
	}

	if (reg & STM32_RTC_ALRMR_HOURMASK_MSK) {
		tm->tm_hour = -1;
	} else {
		tm->tm_hour = bcd2bin((reg & STM32_RTC_ALRMR_HOUR_MSK) >>
			STM32_RTC_ALRMR_HOUR_BITS);
	}

	if (reg & STM32_RTC_ALRMR_MINMASK_MSK) {
		tm->tm_min = -1;
	} else {
		tm->tm_min = bcd2bin((reg & STM32_RTC_ALRMR_MIN_MSK) >>
			STM32_RTC_ALRMR_MIN_BITS);
	}

	if (reg & STM32_RTC_ALRMR_SECMASK_MSK) {
		tm->tm_sec = -1;
	} else {
		tm->tm_sec = bcd2bin((reg & STM32_RTC_ALRMR_SEC_MSK) >>
			STM32_RTC_ALRMR_SEC_BITS);
	}

	alrm->enabled = !!(rtc->regs->cr & STM32_RTC_CR_ALRAE_MSK);
	alrm->pending = !!(rtc->regs->isr & STM32_RTC_ISR_ALRAF_MSK);

	spin_unlock_irq(&rtc->lock);
	return 0;
}

/*
 * Set normal alarm.
 *
 * We use Alarm A, because Alarm B is already used to emulate update interrupt.
 */
static int stm32_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct stm32_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &alrm->time;
	int rv;

	if (rtc_valid_tm(tm)) {
		rv = -EINVAL;
		goto out;
	}

	spin_lock_irq(&rtc->lock);

	/* Set Alarm A */
	stm32_setup_alarm(rtc, 0, tm->tm_mday, tm->tm_wday,
			  tm->tm_hour, tm->tm_min, tm->tm_sec);

	/* Enable Alarm A interrupts if needed */
	if (alrm->enabled)
		stm32_alarm_irq_enable(rtc, 0, 1);

	spin_unlock_irq(&rtc->lock);
	rv = 0;
out:
	return rv;
}

static struct rtc_class_ops stm32_rtc_ops = {
	.read_time		= stm32_rtc_read_time,
	.set_time		= stm32_rtc_set_time,
	.read_alarm		= stm32_rtc_read_alarm,
	.set_alarm		= stm32_rtc_set_alarm,
	.alarm_irq_enable	= stm32_rtc_alarm_irq_enable,
};

/*
 * RTC device initialization function
 */
static int stm32_rtc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node, *parent;
	struct irq_domain *domain;
	struct stm32_rtc *rtc;
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_dev;
	struct resource *r;
	u32 irq;
	int rv;

	rtc = devm_kzalloc(dev, sizeof(*rtc), GFP_KERNEL);
	if (unlikely(!rtc)) {
		dev_err(dev, "Can't alloc rtc dsc.\n");
		rv = -ENOMEM;
		goto out;
	}

	of_dev = of_match_device(dev->driver->of_match_table, dev);
	if (of_dev->data) {
		const struct stm32_rtc_of_data *data = of_dev->data;
		rtc->init = data->init;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rtc->regs = devm_ioremap_resource(&pdev->dev, r);
	if (!rtc->regs) {
		dev_err(dev, "Can't get ioresource.\n");
		rv = -EINVAL;
		goto err;
	}

	if (rtc->init) {
		rv = rtc->init(rtc, dev);
		if (rv) {
			dev_err(dev, "Hw init failed (%d)\n", rv);
			goto err;
		}
	}

	platform_set_drvdata(pdev, rtc);
	device_init_wakeup(dev, 1);
	spin_lock_init(&rtc->lock);

	/* Register our RTC with the RTC framework */
	rtc->rtc_dev = rtc_device_register(pdev->name, dev, &stm32_rtc_ops,
					   THIS_MODULE);
	if (unlikely(IS_ERR(rtc->rtc_dev))) {
		dev_err(dev, "Can't register RTC device.\n");
		rv = PTR_ERR(rtc->rtc_dev);
		goto err;
	}

	/* Map EXTI RTC IRQs */
	parent = of_irq_find_parent(np);
	if (!parent) {
		printk("%s: no parent\n", __func__);
		rv = -ENXIO;
		goto err_reg;
	}
	domain = irq_find_host(parent);
	if (!domain) {
		printk("%s: no domain\n", __func__);
		rv = -ENXIO;
		goto err_reg;
	}

	of_property_read_u32(np, "exti-line-alarm", &irq);
	rtc->irq_alarm = irq_create_mapping(domain, irq);
	if (!(rtc->irq_alarm > 0)) {
		printk("%s: map alarm irq err\n", __func__);
		rv = -ENXIO;
		goto err_reg;
	}
	rv = request_irq(rtc->irq_alarm, stm32_rtc_alarm_irq,
			 0, "RTC.alarm", rtc);
	if (rv < 0) {
		printk("%s: request irq alarm err\n", __func__);
		goto err_reg;
	}

	of_property_read_u32(np, "exti-line-wakeup", &irq);
	rtc->irq_wakeup = irq_create_mapping(domain, irq);
	if (!(rtc->irq_wakeup > 0)) {
		printk("%s: map wakeup irq err\n", __func__);
		rv = -ENXIO;
		goto err_alarm_irq;
	}
	rv = request_irq(rtc->irq_wakeup, stm32_rtc_wakeup_irq,
			 0, "RTC.wakeup", rtc);
	if (rv < 0) {
		printk("%s: request irq alarm err\n", __func__);
		goto err_alarm_irq;
	}

	rv = 0;
	goto out;

err_alarm_irq:
	free_irq(rtc->irq_alarm, dev);
err_reg:
	rtc_device_unregister(rtc->rtc_dev);
err:
	devm_kfree(dev, rtc);
out:
	return rv;
}

static int stm32_rtc_remove(struct platform_device *pdev)
{
	struct stm32_rtc *rtc = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	free_irq(rtc->irq_wakeup, dev);
	free_irq(rtc->irq_alarm, dev);
	rtc_device_unregister(rtc->rtc_dev);
	platform_set_drvdata(pdev, NULL);
	devm_kfree(dev, rtc);

	return 0;
}

static int stm32fx_rtc_init(struct stm32_rtc *rtc, struct device *dev)
{
	struct device_node *np;
	struct regmap *reg;
	struct clk *clk;
	unsigned int v;
	int rv;

	/*
	 * Allow access to RTC
	 */
	np = of_find_compatible_node(NULL, NULL, "st,stm32-pwr");
	if (!np) {
		dev_err(dev, "Failed get stm32-pwr-cr\n");
		rv = -ENODEV;
		goto out;
	}

	clk = of_clk_get(np, 0);
	if (clk)
		clk_prepare_enable(clk);
	reg = syscon_node_to_regmap(np);
	regmap_update_bits(reg, STM32_PWR_CR, STM32_PWR_CR_DBP_MSK,
			   STM32_PWR_CR_DBP_MSK);
	of_node_put(np);

	/*
	 * Set-up RTC oscillator
	 */
	reg = syscon_regmap_lookup_by_compatible("st,stm32f42xx-rcc");
	if (IS_ERR(reg)) {
		dev_err(dev, "Failed get stm32-rcc-bdcr\n");
		rv = -ENODEV;
		goto out;
	}

	/* Disable the low-speed external oscillator */
	regmap_write(reg, STM32_RCC_BDCR, 0);
	/* Wait till LSE is stopped */
	do {
		regmap_read(reg, STM32_RCC_BDCR, &v);
	} while (v & STM32_RCC_BDCR_LSERDY_MSK);

	/* Enable the low-speed external oscillator */
	if (!of_property_read_bool(dev->of_node, "st,lse-bypass")) {
		regmap_write(reg, STM32_RCC_BDCR, STM32_RCC_BDCR_LSEON_MSK);
	} else {
		regmap_write(reg, STM32_RCC_BDCR,
			STM32_RCC_BDCR_LSEBYP_MSK | STM32_RCC_BDCR_LSEON_MSK);
	}
	/* Wait till LSE is ready */
	do {
		regmap_read(reg, STM32_RCC_BDCR, &v);
	} while (!(v & STM32_RCC_BDCR_LSERDY_MSK));
	/* Select low-speed external oscillator (LSE) for RTC */
	regmap_update_bits(reg, STM32_RCC_BDCR, STM32_RCC_BDCR_RTCSEL_MSK,
			   STM32_RCC_BDCR_RTCSEL_LSE);

	if (of_machine_is_compatible("st,stm32f7-som")) {
		regmap_update_bits(reg, STM32_RCC_BDCR,
			STM32_RCC_BDCR_LSEDRV_MSK, STM32_RCC_BDCR_LSEDRV_MEDHI);
	}

	/* Enable the RTC */
	regmap_update_bits(reg, STM32_RCC_BDCR, STM32_RCC_BDCR_RTCEN_MSK,
			   STM32_RCC_BDCR_RTCEN_MSK);

	/*
	 * Configure EXTI
	 */
	reg = syscon_regmap_lookup_by_compatible("st,stm32-exti");
	if (IS_ERR(reg)) {
		dev_err(dev, "Failed get stm32-exti");
		rv = -ENODEV;
		goto out;
	}
	rtc->private = reg;

	/* Enable RTC event lines in EXTI */
	regmap_update_bits(reg, STM32_EXTI_RTSR,
		STM32_EXTI_LINE_RTC_ALARM | STM32_EXTI_LINE_RTC_WAKEUP,
		STM32_EXTI_LINE_RTC_ALARM | STM32_EXTI_LINE_RTC_WAKEUP);
	regmap_write(reg, STM32_EXTI_PR,
		STM32_EXTI_LINE_RTC_ALARM | STM32_EXTI_LINE_RTC_WAKEUP);
	regmap_update_bits(reg, STM32_EXTI_IMR,
		STM32_EXTI_LINE_RTC_ALARM | STM32_EXTI_LINE_RTC_WAKEUP,
		STM32_EXTI_LINE_RTC_ALARM | STM32_EXTI_LINE_RTC_WAKEUP);
	rv = 0;
out:
	return rv;
}

static const struct stm32_rtc_of_data stm32fx_rtc_data = {
	.init		= stm32fx_rtc_init,
};

static const struct of_device_id stm32_rtc_match[] = {
	{ .compatible = "st,stm32f4-rtc", .data = &stm32fx_rtc_data },
	{ }
};
MODULE_DEVICE_TABLE(of, stm32_rtc_match);

static struct platform_driver stm32_rtc_driver = {
	.probe		= stm32_rtc_probe,
	.remove		= stm32_rtc_remove,
	.driver		= {
		.name		= "stm32-rtc",
		.of_match_table	= stm32_rtc_match,
	},
};
module_platform_driver(stm32_rtc_driver);

MODULE_DESCRIPTION("STM32 On-Chip Real Time Clock Driver");
MODULE_AUTHOR("Alexander Potashev <aspotashev@emcraft.com>");
MODULE_LICENSE("GPL");
