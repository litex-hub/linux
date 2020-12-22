// SPDX-License-Identifier: GPL-2.0
/*
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

#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_prime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/litex.h>
#include <linux/errno.h>

#include "litevideo.h"
#include "mmcm.h"

#define DRIVER_NAME     "litevideo"

#define DMA_DISABLE     0
#define DMA_ENABLE      1

static void litevideo_get_md(u32 pixel_clock, u32 *best_m, u32 *best_d)
{
	u32 ideal_m, ideal_d, curr_m, curr_d, m, d;
	u32 curr_diff, test_diff;

	ideal_m = pixel_clock;
	ideal_d = LITEVIDEO_IDEAL_DIV_VALUE;

	/* Start searching from minimum values */

	curr_m = MMCM_MIN_M;
	curr_d = MMCM_MIN_D;

	for (d = MMCM_MIN_D; d < MMCM_MAX_D; d++) {
		for (m = MMCM_MIN_M; m < MMCM_MAX_M; m++) {

			/* Clocks cannot be set perfectly, therefore all
			 * combinations for multiplier (m) and divisor (d)
			 * are checked to find the closest possible clock value
			 */

			curr_diff = abs((d * ideal_d * curr_m) -
					(d * curr_d * ideal_m));
			test_diff = abs((curr_d * ideal_d * m) -
					(d * curr_d * ideal_m));

			if (test_diff < curr_diff) {
				curr_m = m;
				curr_d = d;
			}
		}
	}

	*best_m = curr_m;
	*best_d = curr_d;
}

static int litevideo_mmcm_write(struct litevideo_prv *prv, u32 addr, u32 data)
{
	/* write MMCM register address */

	litex_set_reg(prv->base + LITEVIDEO_MMCM_ADDR_OFF,
		      LITEVIDEO_MMCM_ADDR_SIZE, addr);

	/* write data to send to MMCM register */

	litex_set_reg(prv->base + LITEVIDEO_MMCM_DATA_OFF,
		      LITEVIDEO_MMCM_DATA_SIZE, data);

	/* send the data */

	litex_set_reg(prv->base + LITEVIDEO_MMCM_WRITE_OFF,
		      LITEVIDEO_MMCM_WRITE_SIZE, MMCM_WRITE);

	/* wait for transfer finish */

	if (!wait_event_timeout(prv->wq,
			     litex_get_reg(prv->base + LITEVIDEO_MMCM_READY_OFF,
			     LITEVIDEO_MMCM_READY_SIZE), HZ))
		return -ETIMEDOUT;

	return 0;
}

static int litevideo_clkgen_write(struct litevideo_prv *prv, u32 m, u32 d)
{
	/* write M */

	int ret;

	ret = litevideo_mmcm_write(prv, MMCM_CLKFBOUT1,
				   MMCM_HT_FALLING_EDGE |
				   (m / 2) << MMCM_HT_SHIFT |
				   (m / 2 + (m % 2)) << MMCM_LT_SHIFT);

	if (ret < 0)
		return ret;

	/* write D */

	if (d == 1)
		ret = litevideo_mmcm_write(prv, MMCM_DIVCLK,
					   MMCM_HT_FALLING_EDGE);
	else
		ret = litevideo_mmcm_write(prv, MMCM_DIVCLK,
					   (d / 2) << MMCM_HT_SHIFT |
					   (d / 2 + (d % 2)) << MMCM_LT_SHIFT);

	if (ret < 0)
		return ret;

	/* clkout0_divide = 10 */

	ret = litevideo_mmcm_write(prv, MMCM_CLKOUT0,
				   MMCM_HT_FALLING_EDGE |
				   MMCM_CLKOUT_DIV10 << MMCM_HT_SHIFT |
				   MMCM_CLKOUT_DIV10 << MMCM_LT_SHIFT);

	if (ret < 0)
		return ret;

	/* clkout1_divide = 2 */

	ret = litevideo_mmcm_write(prv, MMCM_CLKOUT1,
				   MMCM_HT_FALLING_EDGE |
				   MMCM_CLKOUT_DIV2 << MMCM_HT_SHIFT |
				   MMCM_CLKOUT_DIV2 << MMCM_LT_SHIFT);

	return ret;
}

static int litevideo_drv_init(struct litevideo_prv *prv, u32 m, u32 d)
{
	int ret;

	/* initialize waitqueue for timeouts in litex_mmcm_write() */

	init_waitqueue_head(&prv->wq);

	/* generate clock */

	ret = litevideo_clkgen_write(prv, m, d);
	if (ret < 0)
		return ret;

	/* timings - horizontal */

	litex_set_reg(prv->base + LITEVIDEO_CORE_HRES_OFF,
		      LITEVIDEO_CORE_HRES_SIZE, prv->h_active);
	litex_set_reg(prv->base + LITEVIDEO_CORE_HSYNC_START_OFF,
		      LITEVIDEO_CORE_HSYNC_START_SIZE,
		      prv->h_active + prv->h_front_porch);
	litex_set_reg(prv->base + LITEVIDEO_CORE_HSYNC_END_OFF,
		      LITEVIDEO_CORE_HSYNC_END_SIZE,
		      prv->h_active + prv->h_front_porch + prv->v_sync);
	litex_set_reg(prv->base + LITEVIDEO_CORE_HSCAN_OFF,
		      LITEVIDEO_CORE_HSCAN_SIZE,
		      prv->h_active + prv->h_blanking);

	/* timings - vertical */

	litex_set_reg(prv->base + LITEVIDEO_CORE_VRES_OFF,
		      LITEVIDEO_CORE_VRES_SIZE, prv->v_active);
	litex_set_reg(prv->base + LITEVIDEO_CORE_VSYNC_START_OFF,
		      LITEVIDEO_CORE_VSYNC_START_SIZE,
		      prv->v_active + prv->v_front_porch);
	litex_set_reg(prv->base + LITEVIDEO_CORE_VSYNC_END_OFF,
		      LITEVIDEO_CORE_VSYNC_END_SIZE,
		      prv->v_active + prv->v_front_porch + prv->v_sync);
	litex_set_reg(prv->base + LITEVIDEO_CORE_VSCAN_OFF,
		      LITEVIDEO_CORE_VSCAN_SIZE,
		      prv->v_active + prv->v_blanking);

	/* configure DMA */

	litex_set_reg(prv->base + LITEVIDEO_DMA_ENABLE_OFF,
		      LITEVIDEO_DMA_ENABLE_SIZE, DMA_DISABLE);

	litex_set_reg(prv->base + LITEVIDEO_DMA_BASE_ADDR_OFF,
		      LITEVIDEO_DMA_BASE_ADDR_SIZE, prv->dma_offset);

	litex_set_reg(prv->base + LITEVIDEO_DMA_LENGTH_OFF,
		      LITEVIDEO_DMA_LENGTH_SIZE, prv->dma_length);

	litex_set_reg(prv->base + LITEVIDEO_DMA_ENABLE_OFF,
		      LITEVIDEO_DMA_ENABLE_SIZE, DMA_ENABLE);

	return 0;
}

static int litevideo_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct litevideo_prv *prv;
	struct resource *res;
	int ret;
	u32 val;
	u32 m, d;

	/* no device tree */

	if (!np)
		return -ENODEV;

	prv = devm_kzalloc(&pdev->dev, sizeof(*prv), GFP_KERNEL);
	if (!prv)
		return -ENOMEM;

	/* pixel clock */

	ret = of_property_read_u32(np, "litevideo,pixel-clock", &val);
	if (ret)
		return -EINVAL;
	prv->pixel_clock = val;

	/* timings - vertical */

	ret = of_property_read_u32(np, "litevideo,v-active", &val);
	if (ret)
		return -EINVAL;
	prv->v_active = val;

	ret = of_property_read_u32(np, "litevideo,v-blanking", &val);
	if (ret)
		return -EINVAL;
	prv->v_blanking = val;

	ret = of_property_read_u32(np, "litevideo,v-front-porch", &val);
	if (ret)
		return -EINVAL;
	prv->v_front_porch = val;

	ret = of_property_read_u32(np, "litevideo,v-sync", &val);
	if (ret)
		return -EINVAL;
	prv->v_sync = val;

	/* timings - horizontal */

	ret = of_property_read_u32(np, "litevideo,h-active", &val);
	if (ret)
		return -EINVAL;
	prv->h_active = val;

	ret = of_property_read_u32(np, "litevideo,h-blanking", &val);
	if (ret)
		return -EINVAL;
	prv->h_blanking = val;

	ret = of_property_read_u32(np, "litevideo,h-front-porch", &val);
	if (ret)
		return -EINVAL;
	prv->h_front_porch = val;

	ret = of_property_read_u32(np, "litevideo,h-sync", &val);
	if (ret)
		return -EINVAL;
	prv->h_sync = val;

	/* DMA */

	ret = of_property_read_u32(np, "litevideo,dma-offset", &val);
	if (ret)
		return -EINVAL;
	prv->dma_offset = val;

	ret = of_property_read_u32(np, "litevideo,dma-length", &val);
	if (ret)
		return -EINVAL;
	prv->dma_length = val;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	prv->base = devm_ioremap_resource(&pdev->dev, res);
	if (!prv->base)
		return -ENXIO;

	litevideo_get_md(prv->pixel_clock, &m, &d);
	return litevideo_drv_init(prv, m, d);
}

static const struct of_device_id litevideo_of_match[] = {
	{ .compatible = "litex,litevideo" },
	{}
};
MODULE_DEVICE_TABLE(of, litevideo_of_match);

static struct platform_driver litevideo_platform_driver = {
	.probe = litevideo_probe,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = litevideo_of_match,
	},
};

module_platform_driver(litevideo_platform_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LiteVideo driver");
MODULE_AUTHOR("Antmicro <www.antmicro.com>");
MODULE_ALIAS("platform:" DRIVER_NAME);
