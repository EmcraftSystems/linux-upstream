/*
   UIO-DMA kernel back-end
   Copyright (C) 2009 Qualcomm Inc. All rights reserved.
   Written by Max Krasnyansky <maxk@qualcommm.com>

   The UIO-DMA is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The UIO-DMA is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.
*/

#ifndef UIO_DMA_H
#define UIO_DMA_H

#include <linux/types.h>
#include <linux/device.h>

/* kernel interfaces */

int uio_dma_device_open(struct device *dev, uint32_t *devid);
int uio_dma_device_close(uint32_t devid);

/* ioctl defines */

/* Caching modes */
enum {
	UIO_DMA_CACHE_DEFAULT = 0,
	UIO_DMA_CACHE_DISABLE,
	UIO_DMA_CACHE_WRITECOMBINE
};

/* DMA mapping direction */
enum {
	UIO_DMA_BIDIRECTIONAL = 0,
	UIO_DMA_TODEVICE,
	UIO_DMA_FROMDEVICE
};

#define UIO_DMA_ALLOC _IOW('U', 200, int)
struct uio_dma_alloc_req {
	uint64_t dma_mask;
	uint16_t memnode;
	uint16_t cache;
	uint32_t flags;
	uint32_t chunk_size;
	uint32_t chunk_count;
	uint64_t mmap_offset;
	uint64_t phys_start;
	uint64_t phys_end;
};

#define UIO_DMA_FREE  _IOW('U', 201, int)
struct uio_dma_free_req {
	uint64_t mmap_offset;
};

#define UIO_DMA_MAP   _IOW('U', 202, int)
struct uio_dma_map_req {
	uint64_t mmap_offset;
	uint32_t flags;
	uint32_t devid;
	uint8_t  direction;
	uint32_t chunk_count;
	uint32_t chunk_size;
	uint64_t dmaddr[0];
};

#define UIO_DMA_UNMAP _IOW('U', 203, int)
struct uio_dma_unmap_req {
	uint64_t mmap_offset;
	uint32_t devid;
	uint32_t flags;
	uint8_t  direction;
};

#endif /* UIO_DMA_H */
