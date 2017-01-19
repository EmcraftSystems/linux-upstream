/*
 * Header file for the STM32 DMA Controller driver
 *
 * Copyright (C) 2015
 * Yuri Tikhonov, Emcraft Systems, yur@emcraft.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _STM32_DMA_H_
#define _STM32_DMA_H_

/**
 * Maximum number of items which may be transferred with DMA
 */
#define STM32_DMA_NDT_MAX	0xFFFF

/**
 * struct stm32_dma_platform_data - Controller configuration parameters
 * @nr_channels: Number of channels supported by hardware
 * @cap_mask: dma_capability flags supported by the platform
 * @iflags: Channel interrupt flags offsets in DMA_HIST:LISR
 */
struct stm32_dma_platform_data {
	u32		nr_channels;
	dma_cap_mask_t	cap_mask;
	u32		iflags[8];
};

/**
 * struct stm32_dma_slave - Controller-specific information about a slave
 * @dma_dev: required DMA master device
 * @chan_idx: required DMA channel (stream)
 * @cfg: platform specific configuration data
 */
struct stm32_dma_slave {
	struct device	*dma_dev;
	u32		chan_idx;
	u32		cfg;
};

#endif /* _STM32_DMA_H_ */
