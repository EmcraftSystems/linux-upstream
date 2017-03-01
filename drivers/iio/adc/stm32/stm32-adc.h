/*
 * This file is part of STM32 ADC driver
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

#ifndef __STM32_ADC_H
#define __STM32_ADC_H

#include <linux/platform_device.h>
#include <linux/dmaengine.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/irq_work.h>
#include <linux/pwm.h>

/*
 * STM32 - ADC global register map
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
#define STM32_ADCX_COMN_OFFSET		0x300
#define STM32_ADC_ID_MAX		3
#define STM32_ADC_MAX_SQ		16	/* SQ1..SQ16 */
#define STM32_ADC_MAX_JSQ		4	/* JSQ1..JSQ4 */

/* STM32 value masks */
#define STM32_RESULT_MASK	GENMASK(15, 0)
#define STM32_STORAGEBITS	16

/* External trigger enable for regular or injected channels (exten/jexten) */
enum stm32_adc_exten {
	STM32_EXTEN_SWTRIG,
	STM32_EXTEN_HWTRIG_RISING_EDGE,
	STM32_EXTEN_HWTRIG_FALLING_EDGE,
	STM32_EXTEN_HWTRIG_BOTH_EDGES,
};

enum stm32_adc_extsel {
	STM32_EXT0,
	STM32_EXT1,
	STM32_EXT2,
	STM32_EXT3,
	STM32_EXT4,
	STM32_EXT5,
	STM32_EXT6,
	STM32_EXT7,
	STM32_EXT8,
	STM32_EXT9,
	STM32_EXT10,
	STM32_EXT11,
	STM32_EXT12,
	STM32_EXT13,
	STM32_EXT14,
	STM32_EXT15,
	STM32_EXT16,
	STM32_EXT17,
	STM32_EXT18,
	STM32_EXT19,
	STM32_EXT20,
	STM32_EXT21,
	STM32_EXT22,
	STM32_EXT23,
	STM32_EXT24,
	STM32_EXT25,
	STM32_EXT26,
	STM32_EXT27,
	STM32_EXT28,
	STM32_EXT29,
	STM32_EXT30,
	STM32_EXT31,
};

enum stm32_adc_jextsel {
	STM32_JEXT0,
	STM32_JEXT1,
	STM32_JEXT2,
	STM32_JEXT3,
	STM32_JEXT4,
	STM32_JEXT5,
	STM32_JEXT6,
	STM32_JEXT7,
	STM32_JEXT8,
	STM32_JEXT9,
	STM32_JEXT10,
	STM32_JEXT11,
	STM32_JEXT12,
	STM32_JEXT13,
	STM32_JEXT14,
	STM32_JEXT15,
	STM32_JEXT16,
	STM32_JEXT17,
	STM32_JEXT18,
	STM32_JEXT19,
	STM32_JEXT20,
	STM32_JEXT21,
	STM32_JEXT22,
	STM32_JEXT23,
	STM32_JEXT24,
	STM32_JEXT25,
	STM32_JEXT26,
	STM32_JEXT27,
	STM32_JEXT28,
	STM32_JEXT29,
	STM32_JEXT30,
	STM32_JEXT31,
};

enum stm32_adc_dma_mode {
	STM32_DM_DIS		= 0,
	STM32_DM_DMA1,
	STM32_DM_DMA2,
	STM32_DM_DMA3
};

enum stm32_adc_multi_mode {
	STM32_MM_INDEPEND	= 0,

	STM32_MM_DUAL_CRS_ISM	= 1,
	STM32_MM_DUAL_CRS_ATM	= 2,
	STM32_MM_DUAL_ISM	= 5,
	STM32_MM_DUAL_RSM	= 6,
	STM32_MM_DUAL_IM	= 7,
	STM32_MM_DUAL_ATM	= 9,

	STM32_MM_TRIPLE_CRS_ISM	= 17,
	STM32_MM_TRIPLE_CRS_ATM	= 18,
	STM32_MM_TRIPLE_ISM	= 21,
	STM32_MM_TRIPLE_RSM	= 22,
	STM32_MM_TRIPLE_IM	= 23,
	STM32_MM_TRIPLE_ATM	= 25
};

#define	STM32_ADC_TIMEOUT_US	100000
#define	STM32_ADC_TIMEOUT	(msecs_to_jiffies(STM32_ADC_TIMEOUT_US / 1000))

/**
 * struct stm32_adc_chan_spec - specification of stm32 adc channel
 * @type:	IIO channel type
 * @channel:	channel number (single ended)
 * @name:	channel name (single ended)
 */
struct stm32_adc_chan_spec {
	enum iio_chan_type	type;
	int			channel;
	const char		*name;
};

/**
 * struct stm32_adc_chan - Extended specifications of stm32 adc channels
 * @smpr: per channel sampling time selection
 */
struct stm32_adc_chan {
	unsigned smpr:3;
};

/**
 * struct stm32_adc_trig_info - ADC trigger info
 * @extsel:		trigger selection for regular or injected
 * @name:		name of the trigger, corresponding to its source
 */
struct stm32_adc_trig_info {
	u32 extsel;
	const char *name;
};

/**
 * struct stm32_adc_info - stm32 ADC, per instance config data
 * @name:		default name for this instance (like "adc1")
 * @reg:		reg offset for this instance (e.g. 0x0 for adc1...)
 * @channels:		Reference to stm32 channels spec
 * @max_channels:	Number of single ended channels
 */
struct stm32_adc_info {
	const char *name;
	u32 reg;
	const struct stm32_adc_chan_spec *channels;
	int max_channels;
};

/**
 * stm32_adc_regs - stm32 ADC misc registers & bitfield desc
 * @reg:		register offset
 * @mask:		bitfield mask
 * @shift:		left shift
 */
struct stm32_adc_regs {
	int reg;
	int mask;
	int shift;
};

/**
 * stm32_adc_trig_reginfo - stm32 ADC trigger control registers description
 * @reg:		trigger control register offset (exten/jexten)
 * @exten_mask:		external trigger en/polarity mask in @reg
 * @exten_shift:	external trigger en/polarity shift in @reg
 * @extsel_mask:	external trigger source mask in @reg
 * @extsel_shift:	external trigger source shift in @reg
 */
struct stm32_adc_trig_reginfo {
	u32 reg;
	u32 exten_mask;
	u32 exten_shift;
	u32 extsel_mask;
	u32 extsel_shift;
};

/**
 * struct stm32_adc_reginfo - stm32 ADC registers description
 * @isr:		interrupt status register offset
 * @ovr:		onverrun mask in @isr
 * @eoc:		end of conversion mask in @isr
 * @jeoc:		end of injected conversion sequence mask in @isr
 * @ier:		interrupt enable register offset
 * @ovrie:		overrun interrupt enable mask in @ier
 * @eocie:		end of conversion interrupt enable mask in @ier
 * @jeocie:		end of injected conversion sequence interrupt en mask
 * @dr:			data register offset
 * @cdr:		common regular data register for dual and triple modes
 * @jdr:		injected data registers offsets
 * @sqr_regs:		Regular sequence registers description
 * @jsqr_reg:		Injected sequence register description
 * @smpr_regs:		Sampling time registers description
 * @trig_reginfo:	regular trigger control registers description
 * @jtrig_reginfo:	injected trigger control registers description
 */
struct stm32_adc_reginfo {
	u32 isr;
	u32 ovr;
	u32 eoc;
	u32 jeoc;
	u32 ier;
	u32 ovrie;
	u32 eocie;
	u32 jeocie;
	u32 dr;
	u32 cdr;
	u32 jdr[4];
	const struct stm32_adc_regs *sqr_regs;
	const struct stm32_adc_regs *jsqr_reg;
	const struct stm32_adc_regs *smpr_regs;
	const struct stm32_adc_trig_reginfo *trig_reginfo;
	const struct stm32_adc_trig_reginfo *jtrig_reginfo;
};

struct stm32_adc;
struct stm32_adc_common;

/**
 * struct stm32_adc_ops - stm32 ADC, compatible dependent data
 * - stm32 ADC may work as single ADC, or as tightly coupled master/slave ADCs.
 *
 * @adc_info:		Array spec for stm32 adc master/slaves instances
 * @ext_triggers:	Reference to trigger info for regular channels
 * @jext_triggers:	Reference to trigger info for injected channels
 * @adc_reginfo:	stm32 ADC registers description
 * @ext_info:		Extended channel info
 * @highres:		Max resolution
 * @max_clock_rate:	Max input clock rate
 * @parse_dts_params:	Parse specific dts params
 * @clk_sel:		routine to select common clock and prescaler
 * @prepare_conv:	routine to prepare conversions
 * @start_conv:		routine to start conversions
 * @stop_conv:		routine to stop conversions
 * @recover:		routine to recover after overruns
 * @is_started:		routine to get adc 'started' state
 * @regular_started	routine to check regular conversions status
 * @injected_started	routine to check injected conversions status
 * @enable:		optional routine to enable stm32 adc
 * @disable:		optional routine to disable stm32 adc
 * @is_enabled		reports enabled state
 * @multi_dma_sel:	configure multi DMA mode parameters
 */
struct stm32_adc_ops {
	const struct stm32_adc_info *adc_info;
	const struct stm32_adc_trig_info *ext_triggers;
	const struct stm32_adc_trig_info *jext_triggers;
	const struct stm32_adc_reginfo *adc_reginfo;
	const struct iio_chan_spec_ext_info *ext_info;
	int highres;
	unsigned long max_clock_rate;
	int (*parse_dts_params)(struct device *dev, struct device_node *np);
	int (*clk_sel)(struct stm32_adc *adc);
	void (*prepare_conv)(struct stm32_adc *adc,
		const struct iio_chan_spec *chan);
	int (*start_conv)(struct stm32_adc *adc);
	int (*stop_conv)(struct stm32_adc *adc);
	void (*recover)(struct stm32_adc *adc);
	bool (*is_started)(struct stm32_adc *adc);
	bool (*regular_started)(struct stm32_adc *adc);
	bool (*injected_started)(struct stm32_adc *adc);
	int (*enable)(struct stm32_adc *adc);
	void (*disable)(struct stm32_adc *adc);
	bool (*is_enabled)(struct stm32_adc *adc);
	int (*multi_dma_sel)(struct stm32_adc_common *com,
		enum stm32_adc_dma_mode dm, enum stm32_adc_multi_mode mm);
};

struct stm32_adc_common;

/**
 * struct stm32_adc - private data of each ADC IIO instance
 * @common:		reference to ADC block common data
 * @adc_list:		current ADC entry in common ADC list
 * @id:			ADC instance number (e.g. adc 1, 2 or 3)
 * @offset:		ADC instance register offset in ADC block
 * @max_channels:	Max channels number for this ADC.
 * @exten:		External trigger config (enable/polarity)
 * @extrig_list:	External trigger list (for regular channel)
 * @completion:		end of single conversion completion
 * @buffer:		data buffer
 * @bufi:		data buffer index
 * @num_conv:		expected number of scan conversions
 * @injected:		use injected channels on this adc
 * @multi:		adc is used as slave in multi ADC mode
 * @lock:		spinlock
 * @work:		irq work used to call trigger poll routine
 * @dma_chan:		dma channel
 * @rx_buf:		dma rx buffer cpu address
 * @rx_dma_buf:		dma rx buffer bus address
 * @rx_buf_sz:		dma rx buffer size
 * @clk:		optional adc clock, for this adc instance
 * @calib:		optional calibration data
 * @en:			emulates enabled state on some stm32 adc
 */
struct stm32_adc {
	struct stm32_adc_common	*common;
	struct list_head	adc_list;
	int			id;
	int			offset;
	int			max_channels;
	enum stm32_adc_exten	exten;
	struct list_head	extrig_list;
	struct completion	completion;
	u16			*buffer;
	int			bufi;
	int			num_conv;
	bool			injected;
	bool			multi;
	spinlock_t		lock;		/* interrupt lock */
	struct irq_work		work;
	struct dma_chan		*dma_chan;
	u8			*rx_buf;
	dma_addr_t		rx_dma_buf;
	int			rx_buf_sz;
	struct clk		*clk;
	void			*calib;
	bool			en;
};

/**
 * struct stm32_avg - private data of ADC driver used for averaging
 * @num:		Average measurements number
 * @rate:		Average measurements period, ms
 * @chan_num:		Number of channels used per ADCx
 * @tr:			ADC start trigger source
 * @pwm:		Trigger timer PWM
 * @dma:		DMA channel
 * @dma_buf:		Raw DMA buffer CPU address
 * @dma_adr:		Raw DMA buffer DMA address
 * @dma_cookie:		DMA transfer cookie
 * @raw_pos:		Raw buffer position
 * @raw_length:		Raw buffer length
 * @raw_period:		Raw buffer period
 * @lock:		Filtered data buffer lock
 * @task:		Overrun processing tasklet
 * @ovr_msk:		Overrun channels mask
 * @raw_buf:		Raw data buffer pointers (seq indexed [*][s])
 * @flt_buf:		Filtered data buffer (chan indexed [*][c])
 */
struct stm32_avg {
	u32			num;
	u32			rate;
	int			chan_num;
	struct iio_trigger	*tr;
	struct pwm_device	*pwm;
	struct dma_chan		*dma;
	u32			*dma_buf;
	dma_addr_t		dma_adr;
	dma_cookie_t		dma_cookie;
	u32			raw_pos;
	u32			raw_length;
	u32			raw_period;
	spinlock_t		lock;
	struct tasklet_struct	task;
	u8			ovr_msk;
	u32			*raw_buf[STM32_ADC_ID_MAX][STM32_ADC_MAX_SQ];
	u32			flt_buf[STM32_ADC_ID_MAX][STM32_ADC_MAX_SQ];
};

/**
 * struct stm32_adc_common - private data of ADC driver, common to all
 * ADC instances (ADC block)
 * @dev:		device for this controller
 * @phys_base:		control registers base physical addr
 * @base:		control registers base cpu addr
 * @irq:		Common irq line for all adc instances
 * @avg:		Averaging info
 * @data:		STM32 dependent data from compatible
 * @adc_list:		list of all stm32 ADC in this ADC block
 * @aclk:		common clock for the analog circuitry
 * @vref:		regulator reference
 * @vref_mv:		vref voltage (mv)
 * @stm32_chans:	stm32 channels extended specification data
 * @gpio_descs:		gpio descriptor used to configure EXTi triggers
 * @lock:		mutex
 */
struct stm32_adc_common {
	struct device			*dev;
	phys_addr_t			phys_base;
	void __iomem			*base;
	int				irq;
	struct stm32_avg		avg;
	const struct stm32_adc_ops	*data;
	struct list_head		adc_list;
	struct clk			*aclk;
	struct regulator		*vref;
	int				vref_mv;
	struct stm32_adc_chan		*stm32_chans[STM32_ADC_ID_MAX];
	struct gpio_descs		*gpios;
	struct mutex			lock;	/* read_raw lock */
};

/* Helper routines */
static inline struct stm32_adc_chan *to_stm32_chan(struct stm32_adc *adc,
						   const struct iio_chan_spec
						   *chan)
{
	struct stm32_adc_chan *stm32_chans = adc->common->stm32_chans[adc->id];

	return &stm32_chans[chan->channel];
}

static inline int stm32_avg_is_enabled(struct stm32_adc *adc)
{
	return !!adc->common->avg.num;
}

static inline void stm32_adc_prepare_conv(struct stm32_adc *adc,
			const struct iio_chan_spec *chan)
{
	adc->common->data->prepare_conv(adc, chan);
}

static inline int stm32_adc_start_conv(struct stm32_adc *adc)
{
	return adc->common->data->start_conv(adc);
}

static inline int stm32_adc_stop_conv(struct stm32_adc *adc)
{
	return adc->common->data->stop_conv(adc);
}

static inline bool stm32_adc_is_started(struct stm32_adc *adc)
{
	return adc->common->data->is_started(adc);
}

static inline bool stm32_adc_regular_started(struct stm32_adc *adc)
{
	return adc->common->data->regular_started(adc);
}

static inline bool stm32_adc_injected_started(struct stm32_adc *adc)
{
	return adc->common->data->injected_started(adc);
}

static inline bool stm32_adc_clk_sel(struct stm32_adc *adc)
{
	return adc->common->data->clk_sel(adc);
}

static inline int stm32_adc_enable(struct stm32_adc *adc)
{
	if (adc->common->data->enable)
		return adc->common->data->enable(adc);

	adc->en = true;

	return 0;
}

static inline bool stm32_adc_is_enabled(struct stm32_adc *adc)
{
	if (adc->common->data->is_enabled)
		return adc->common->data->is_enabled(adc);
	else
		return adc->en;
}

static inline void stm32_adc_disable(struct stm32_adc *adc)
{
	/* Check there is no regular or injected on-going conversions */
	if (stm32_adc_is_started(adc))
		return;

	if (adc->common->data->disable)
		adc->common->data->disable(adc);
	else
		adc->en = false;
}

static inline void stm32_adc_recover(struct stm32_adc *adc)
{
	if (adc->common->data->recover)
		adc->common->data->recover(adc);
}

static inline int stm32_adc_multi_dma_sel(struct stm32_adc_common *com,
		enum stm32_adc_dma_mode dm, enum stm32_adc_multi_mode mm)
{
	return com->data->multi_dma_sel(com, dm, mm);
}

/* STM32 ADC registers access routines */
static inline u32 stm32_adc_common_readl(struct stm32_adc_common *com, u32 reg)
{
	u32 val = readl_relaxed(com->base + reg);

	return val;
}

static inline void stm32_adc_common_writel(struct stm32_adc_common *com,
					   u32 reg, u32 val)
{
	writel_relaxed(val, com->base + reg);
}

static inline u32 stm32_adc_readl(struct stm32_adc *adc, u32 reg)
{
	u32 val = readl_relaxed(adc->common->base + adc->offset + reg);

	return val;
}

#define stm32_adc_readl_addr(addr)	stm32_adc_readl(adc, addr)

#define stm32_adc_readl_poll_timeout(reg, val, cond, sleep_us, timeout_us) \
	readx_poll_timeout(stm32_adc_readl_addr, reg, val, \
			   cond, sleep_us, timeout_us)

static inline void stm32_adc_writel(struct stm32_adc *adc, u32 reg, u32 val)
{
	writel_relaxed(val, adc->common->base + adc->offset + reg);
}

static inline void stm32_adc_set_bits(struct stm32_adc *adc, u32 reg, u32 bits)
{
	unsigned long flags;

	spin_lock_irqsave(&adc->lock, flags);
	stm32_adc_writel(adc, reg, stm32_adc_readl(adc, reg) | bits);
	spin_unlock_irqrestore(&adc->lock, flags);
}

static inline void stm32_adc_clr_bits(struct stm32_adc *adc, u32 reg, u32 bits)
{
	unsigned long flags;

	spin_lock_irqsave(&adc->lock, flags);
	stm32_adc_writel(adc, reg, stm32_adc_readl(adc, reg) & ~bits);
	spin_unlock_irqrestore(&adc->lock, flags);
}

/* STM32 common extended attributes */
extern const struct iio_enum stm32_adc_trig_pol;
int stm32_adc_set_smpr(struct iio_dev *indio_dev,
		       const struct iio_chan_spec *chan, unsigned int smpr);
int stm32_adc_get_smpr(struct iio_dev *indio_dev,
		       const struct iio_chan_spec *chan);
int stm32_adc_probe(struct platform_device *pdev);
int stm32_adc_remove(struct platform_device *pdev);

/* STM32 average ADC driver API */
int stm32_adc_avg_init(struct stm32_adc_common *common);
int stm32_adc_avg_run(struct stm32_adc_common *common);
int stm32_adc_avg_overrun(struct stm32_adc *adc);
int stm32_adc_avg_get(struct iio_dev *indio_dev,
		      struct iio_chan_spec const *chan, int *val);

#endif
