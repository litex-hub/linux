/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2020 Antmicro <www.antmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _LITEVIDEO_LITEVIDEO_H_
#define _LITEVIDEO_LITEVIDEO_H_

#include <drm/drm.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/wait.h>

/* register offsets */

#define LITEVIDEO_CORE_HRES_OFF          0x1c
#define LITEVIDEO_CORE_HSYNC_START_OFF   0x24
#define LITEVIDEO_CORE_HSYNC_END_OFF     0x2c
#define LITEVIDEO_CORE_HSCAN_OFF         0x34
#define LITEVIDEO_CORE_VRES_OFF          0x3c
#define LITEVIDEO_CORE_VSYNC_START_OFF   0x44
#define LITEVIDEO_CORE_VSYNC_END_OFF     0x4c
#define LITEVIDEO_CORE_VSCAN_OFF         0x54

#define LITEVIDEO_MMCM_WRITE_OFF         0x94
#define LITEVIDEO_MMCM_READY_OFF         0x98
#define LITEVIDEO_MMCM_ADDR_OFF          0x9c
#define LITEVIDEO_MMCM_DATA_OFF          0xa0

#define LITEVIDEO_DMA_ENABLE_OFF         0x18
#define LITEVIDEO_DMA_BASE_ADDR_OFF      0x5c
#define LITEVIDEO_DMA_LENGTH_OFF         0x6c

/* register sizes */

#define LITEVIDEO_CORE_HRES_SIZE         2
#define LITEVIDEO_CORE_HSYNC_START_SIZE  2
#define LITEVIDEO_CORE_HSYNC_END_SIZE    2
#define LITEVIDEO_CORE_HSCAN_SIZE        2
#define LITEVIDEO_CORE_VRES_SIZE         2
#define LITEVIDEO_CORE_VSYNC_START_SIZE  2
#define LITEVIDEO_CORE_VSYNC_END_SIZE    2
#define LITEVIDEO_CORE_VSCAN_SIZE        2

#define LITEVIDEO_MMCM_WRITE_SIZE        1
#define LITEVIDEO_MMCM_READY_SIZE        1
#define LITEVIDEO_MMCM_ADDR_SIZE         1
#define LITEVIDEO_MMCM_DATA_SIZE         2

#define LITEVIDEO_DMA_ENABLE_SIZE        1
#define LITEVIDEO_DMA_BASE_ADDR_SIZE     4
#define LITEVIDEO_DMA_LENGTH_SIZE        4

/* constants */

/* clk100 has to be used for LiteX VideoOut */
#define LITEVIDEO_IDEAL_DIV_VALUE 10000

struct litevideo_prv {
	struct drm_device *drm_dev;
	void __iomem *base;
	struct clk *hdmi_clk;
	wait_queue_head_t wq;
	u32 h_active;
	u32 h_blanking;
	u32 h_front_porch;
	u32 h_sync;
	u32 v_active;
	u32 v_blanking;
	u32 v_front_porch;
	u32 v_sync;
	u32 dma_offset;
	u32 dma_length;
	u32 pixel_clock;
};

#endif /* _LITEVIDEO_LITEVIDEO_H_ */
