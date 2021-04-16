// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019-2020 Antmicro <www.antmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/fs.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/genhd.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/genhd.h>
#include <linux/types.h>
#include <linux/blkdev.h>
#include <linux/pm_runtime.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/sched/clock.h>
#include <linux/interrupt.h>

#include "litex_mmc.h"

#define SDCARD_CTRL_DATA_TRANSFER_NONE  0
#define SDCARD_CTRL_DATA_TRANSFER_READ  1
#define SDCARD_CTRL_DATA_TRANSFER_WRITE 2

#define SDCARD_CTRL_RESPONSE_NONE	0
#define SDCARD_CTRL_RESPONSE_SHORT	1
#define SDCARD_CTRL_RESPONSE_LONG	2
#define SDCARD_CTRL_RESPONSE_SHORT_BUSY	3

#define SD_OK         0
#define SD_WRITEERROR 1
#define SD_TIMEOUT    2
#define SD_CRCERROR   3
#define SD_ERR_OTHER  4

#define SDIRQ_CARD_DETECT	1
#define SDIRQ_SD_TO_MEM_DONE	2
#define SDIRQ_MEM_TO_SD_DONE	4
#define SDIRQ_CMD_DONE		8

struct litex_mmc_host {
	struct mmc_host *mmc;
	struct platform_device *dev;

	void __iomem *sdphy;
	void __iomem *sdcore;
	void __iomem *sdreader;
	void __iomem *sdwriter;
	void __iomem *sdirq;

	u32 resp[4];
	u16 rca;

	void *buffer;
	size_t buffer_size;
	dma_addr_t dma_handle;

	unsigned int freq;
	unsigned int clock;
	bool is_bus_width_set;
	bool app_cmd;

	int irq;
	struct completion cmd_done;
};


void sdclk_set_clk(struct litex_mmc_host *host, unsigned int clk_freq) {
	u32 div = clk_freq ? host->freq / clk_freq : 256;
	div = roundup_pow_of_two(div);
	div = min(max(div, (u32)2), (u32)256);
	dev_info(&host->dev->dev,
		"Requested clk_freq=%d: set to %d via div=%d\n",
		clk_freq, host->freq / div, div);
	litex_write16(host->sdphy + LITEX_MMC_SDPHY_CLOCKERDIV_OFF, div);
}


static int sdcard_wait_done(void __iomem *reg) {
	u8 evt;
	for (;;) {
		evt = litex_read8(reg);
		if (evt & 0x1)
			break;
		udelay(5);
	}
	if (evt == 0x1)
		return SD_OK;
	if (evt & 0x2)
		return SD_WRITEERROR;
	if (evt & 0x4)
		return SD_TIMEOUT;
	if (evt & 0x8)
		return SD_CRCERROR;
	pr_err("sdcard_wait_done: unknown error evt=%x\n", evt);
	return SD_ERR_OTHER;
}

static int send_cmd(struct litex_mmc_host *host, u8 cmd, u32 arg,
		    u8 response_len, u8 transfer) {
	void __iomem *reg;
	ulong n;
	u8 i;
	int status;

	litex_write32(host->sdcore + LITEX_MMC_SDCORE_CMDARG_OFF, arg);
	litex_write32(host->sdcore + LITEX_MMC_SDCORE_CMDCMD_OFF,
			 cmd << 8 | transfer << 5 | response_len);
	litex_write8(host->sdcore + LITEX_MMC_SDCORE_CMDSND_OFF, 1);

	/*
	 * Wait for an interrupt if we have an interrupt and either there is
	 * data to be transferred, or if the card can report busy via DAT0.
	 */
	if (host->irq > 0 &&
	    (transfer != SDCARD_CTRL_DATA_TRANSFER_NONE ||
	     response_len == SDCARD_CTRL_RESPONSE_SHORT_BUSY)) {
		reinit_completion(&host->cmd_done);
		litex_write32(host->sdirq + LITEX_MMC_SDIRQ_ENABLE_OFF,
			      SDIRQ_CMD_DONE | SDIRQ_CARD_DETECT);
		wait_for_completion(&host->cmd_done);
	}

	status = sdcard_wait_done(host->sdcore + LITEX_MMC_SDCORE_CMDEVT_OFF);

	if (status != SD_OK) {
		pr_err("Command (cmd %d) failed, status %d\n", cmd, status);
		return status;
	}

	if (response_len != SDCARD_CTRL_RESPONSE_NONE) {
		reg = host->sdcore + LITEX_MMC_SDCORE_CMDRSP_OFF;
		for (i = 0; i < 4; i++) {
			host->resp[i] = litex_read32(reg);
			reg += _next_reg_off(0, sizeof(u32));
		}
	}

	if (!host->app_cmd && cmd == SD_SEND_RELATIVE_ADDR) {
		host->rca = (host->resp[3] >> 16) & 0xffff;
	}

	host->app_cmd = (cmd == MMC_APP_CMD);

	if (transfer == SDCARD_CTRL_DATA_TRANSFER_NONE)
		return status; /* SD_OK from prior sdcard_wait_done(cmd_evt) */

	status = sdcard_wait_done(host->sdcore + LITEX_MMC_SDCORE_DATAEVT_OFF);
	if (status != SD_OK){
		pr_err("Data xfer (cmd %d) failed, status %d\n", cmd, status);
		return status;
	}

	/* wait for completion of (read or write) DMA transfer */
	reg = (transfer == SDCARD_CTRL_DATA_TRANSFER_READ) ?
		host->sdreader + LITEX_MMC_SDBLK2MEM_DONE_OFF :
		host->sdwriter + LITEX_MMC_SDMEM2BLK_DONE_OFF;
	n = jiffies + (HZ << 1);
	while ((litex_read8(reg) & 0x01) == 0)
		if (time_after(jiffies, n)) {
			pr_err("DMA timeout (cmd %d)\n", cmd);
			return SD_TIMEOUT;
		}

	return status;
}

// CMD55
static inline int send_app_cmd(struct litex_mmc_host *host) {
	return send_cmd(host, MMC_APP_CMD, host->rca << 16,
			SDCARD_CTRL_RESPONSE_SHORT,
			SDCARD_CTRL_DATA_TRANSFER_NONE);
}

// ACMD6
static inline int send_app_set_bus_width_cmd(
		struct litex_mmc_host *host, u32 width) {
	return send_cmd(host, SD_APP_SET_BUS_WIDTH, width,
			SDCARD_CTRL_RESPONSE_SHORT,
			SDCARD_CTRL_DATA_TRANSFER_NONE);
}

static int litex_set_bus_width(struct litex_mmc_host *host) {
	bool app_cmd_sent = host->app_cmd; /* was preceding command app_cmd? */
	int status;

	/* ensure 'app_cmd' precedes 'app_set_bus_width_cmd' */
	if (!app_cmd_sent)
		send_app_cmd(host);

	/* litesdcard only supports 4-bit bus width */
	status = send_app_set_bus_width_cmd(host, MMC_BUS_WIDTH_4);

	/* re-send 'app_cmd' if necessary */
	if (app_cmd_sent)
		send_app_cmd(host);

	return status;
}

static int litex_get_cd(struct mmc_host *mmc)
{
	struct litex_mmc_host *host = mmc_priv(mmc);
	int gpio_cd = mmc_gpio_get_cd(mmc);
	int ret;

	if (!mmc_card_is_removable(mmc))
		return 1;

	if (gpio_cd >= 0)
		/* GPIO based card-detect explicitly specified in DTS */
		ret = !!gpio_cd;
	else
		/* use gateware card-detect bit by default */
		ret = !litex_read8(host->sdphy +
				       LITEX_MMC_SDPHY_CARDDETECT_OFF);

	/* ensure bus width will be set (again) upon card (re)insertion */
	if (ret == 0)
		host->is_bus_width_set = false;

	return ret;
}

static irqreturn_t litex_mmc_interrupt(int irq, void *arg)
{
	struct mmc_host *mmc = arg;
	struct litex_mmc_host *host = mmc_priv(mmc);
	u32 pending = litex_read32(host->sdirq + LITEX_MMC_SDIRQ_PENDING_OFF);

	/* Check for card change interrupt */
	if (pending & SDIRQ_CARD_DETECT) {
		litex_write32(host->sdirq + LITEX_MMC_SDIRQ_PENDING_OFF,
			      SDIRQ_CARD_DETECT);
		mmc_detect_change(mmc, msecs_to_jiffies(10));
	}

	/* Check for command completed */
	if (pending & SDIRQ_CMD_DONE) {
		/* Disable it so it doesn't keep interrupting */
		litex_write32(host->sdirq + LITEX_MMC_SDIRQ_ENABLE_OFF,
			      SDIRQ_CARD_DETECT);
		complete(&host->cmd_done);
	}

	return IRQ_HANDLED;
}

static u32 litex_response_len(struct mmc_command *cmd)
{
	u32 response_len = SDCARD_CTRL_RESPONSE_NONE;

	if (cmd->flags & MMC_RSP_136) {
		response_len = SDCARD_CTRL_RESPONSE_LONG;
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_BUSY)
			response_len = SDCARD_CTRL_RESPONSE_SHORT_BUSY;
		else
			response_len = SDCARD_CTRL_RESPONSE_SHORT;
	}

	return response_len;
}

static int litex_map_status(int status)
{
	int error;

	switch (status) {
	case SD_OK:
		error = 0;
		break;
	case SD_WRITEERROR:
		error = -EIO;
		break;
	case SD_TIMEOUT:
		error = -ETIMEDOUT;
		break;
	case SD_CRCERROR:
		error = -EILSEQ;
		break;
	default:
		error = -EINVAL;
		break;
	}
	return error;
}

/*
 * Send request to a card. Command, data transfer, things like this.
 * Call mmc_request_done() when finished.
 */
static void litex_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct litex_mmc_host *host = mmc_priv(mmc);
	struct platform_device *pdev = to_platform_device(mmc->parent);
	struct device *dev = &pdev->dev;
	struct mmc_data *data = mrq->data;
	struct mmc_command *sbc = mrq->sbc;
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_command *stop = mrq->stop;
	unsigned int retries = cmd->retries;
	int status;
	int sg_count;
	enum dma_data_direction dir = DMA_TO_DEVICE;
	bool direct = false;
	dma_addr_t dma_handle;
	unsigned int length = 0;

	u32 response_len = litex_response_len(cmd);
	u32 transfer = SDCARD_CTRL_DATA_TRANSFER_NONE;

	/* First check that the card is still there */
	if (!litex_get_cd(mmc)) {
		cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	/*
	 * Send set-block-count command if needed.
	 */
	if (sbc) {
		status = send_cmd(host, sbc->opcode, sbc->arg,
				  litex_response_len(sbc),
				  SDCARD_CTRL_DATA_TRANSFER_NONE);
		sbc->error = litex_map_status(status);
		if (status != SD_OK) {
			host->is_bus_width_set = false;
			mmc_request_done(mmc, mrq);
			return;
		}
	}

	if (data) {
		/* LiteSDCard only supports 4-bit bus width; therefore, we MUST
		 * inject a SET_BUS_WIDTH (acmd6) before the very first data
		 * transfer, earlier than when the mmc subsystem would normally
		 * get around to it!
		 */
		if (!host->is_bus_width_set) {
			ulong n = jiffies + 2 * HZ; // 500ms timeout
			while (litex_set_bus_width(host) != SD_OK) {
				if (time_after(jiffies, n)) {
					dev_warn(dev, "Can't set bus width!\n");
					cmd->error = -ETIMEDOUT;
					mmc_request_done(mmc, mrq);
					return;
				}
			}
			host->is_bus_width_set = true;
		}

		/*
		 * Try to DMA directly to/from the data buffer.
		 * We can do that if the buffer can be mapped for DMA
		 * in one contiguous chunk.
		 */
		dma_handle = host->dma_handle;
		length = data->blksz * data->blocks;
		if (data->flags & MMC_DATA_READ)
			dir = DMA_FROM_DEVICE;
		sg_count = dma_map_sg(&host->dev->dev, data->sg, data->sg_len,
				      dir);
		if (sg_count == 1) {
			dma_handle = sg_dma_address(data->sg);
			length = sg_dma_len(data->sg);
			direct = true;
		} else if (length > host->buffer_size)
			length = host->buffer_size;

		if (data->flags & MMC_DATA_READ) {
			litex_write8(host->sdreader +
					 LITEX_MMC_SDBLK2MEM_ENA_OFF, 0);
			litex_write64(host->sdreader +
					 LITEX_MMC_SDBLK2MEM_BASE_OFF,
					 dma_handle);
			litex_write32(host->sdreader +
					 LITEX_MMC_SDBLK2MEM_LEN_OFF,
					 length);
			litex_write8(host->sdreader +
					 LITEX_MMC_SDBLK2MEM_ENA_OFF, 1);

			transfer = SDCARD_CTRL_DATA_TRANSFER_READ;

		} else if (data->flags & MMC_DATA_WRITE) {
			if (!direct)
				sg_copy_to_buffer(data->sg, data->sg_len,
					host->buffer, length);

			litex_write8(host->sdwriter +
					 LITEX_MMC_SDMEM2BLK_ENA_OFF, 0);
			litex_write64(host->sdwriter +
					 LITEX_MMC_SDMEM2BLK_BASE_OFF,
					 dma_handle);
			litex_write32(host->sdwriter +
					 LITEX_MMC_SDMEM2BLK_LEN_OFF,
					 length);
			litex_write8(host->sdwriter +
					 LITEX_MMC_SDMEM2BLK_ENA_OFF, 1);

			transfer = SDCARD_CTRL_DATA_TRANSFER_WRITE;
		} else {
			dev_warn(dev, "Data present w/o read or write flag.\n");
			// Intentionally continue: set cmd status, mark req done
		}

		litex_write16(host->sdcore + LITEX_MMC_SDCORE_BLKLEN_OFF,
				 data->blksz);
		litex_write32(host->sdcore + LITEX_MMC_SDCORE_BLKCNT_OFF,
				 data->blocks);
	}

	do {
		status = send_cmd(host, cmd->opcode, cmd->arg,
				  response_len, transfer);
	} while (status != SD_OK && retries-- > 0);

	cmd->error = litex_map_status(status);
	if (status != SD_OK)
		/* card may be gone; don't assume bus width is still set */
		host->is_bus_width_set = false;

	// It looks strange I know, but it's as it should be
	if (response_len == SDCARD_CTRL_RESPONSE_SHORT) {
		cmd->resp[0] = host->resp[3];
		cmd->resp[1] = host->resp[2] & 0xFF;
	} else if (response_len == SDCARD_CTRL_RESPONSE_LONG) {
		cmd->resp[0] = host->resp[0];
		cmd->resp[1] = host->resp[1];
		cmd->resp[2] = host->resp[2];
		cmd->resp[3] = host->resp[3];
	}

	/*
	 * Send stop-transmission command if required.
	 */
	if (stop && (cmd->error || !sbc)) {
		int stop_stat;
		stop_stat = send_cmd(host, stop->opcode, stop->arg,
				     litex_response_len(stop),
				     SDCARD_CTRL_DATA_TRANSFER_NONE);
		stop->error = litex_map_status(stop_stat);
		if (stop_stat != SD_OK)
			host->is_bus_width_set = false;
	}

	if (data)
		dma_unmap_sg(&host->dev->dev, data->sg, data->sg_len, dir);

	if (status == SD_OK && transfer != SDCARD_CTRL_DATA_TRANSFER_NONE) {
		data->bytes_xfered = min(length, mmc->max_req_size);
		if (transfer == SDCARD_CTRL_DATA_TRANSFER_READ && !direct) {
			sg_copy_from_buffer(data->sg, sg_nents(data->sg),
				host->buffer, data->bytes_xfered);
		}
	}

	mmc_request_done(mmc, mrq);
}

static void litex_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct litex_mmc_host *host = mmc_priv(mmc);

	/* updated ios->bus_width -- do nothing;
	 * This happens right after the mmc core subsystem has sent its
	 * own acmd6 to notify the card of the bus-width change, and it's
	 * effectively a no-op given that we already forced bus-width to 4
	 * by snooping on the command flow, and inserting an acmd6 before
	 * the first data xfer comand!
	 */

	if (ios->clock != host->clock) {
		sdclk_set_clk(host, ios->clock);
		host->clock = ios->clock;
	}
}

static const struct mmc_host_ops litex_mmc_ops = {
	.get_cd = litex_get_cd,
	.request = litex_request,
	.set_ios = litex_set_ios,
};

#define MAP_RESOURCE(res_name, idx) \
{ \
	res = platform_get_resource(pdev, IORESOURCE_MEM, idx); \
	host->res_name = devm_ioremap_resource(&pdev->dev, res); \
	if (IS_ERR(host->res_name)) { \
		ret = PTR_ERR(host->res_name); \
		pr_err("MAP_RESOURCE %d failed\n", idx); \
		goto err_exit; \
	} \
}

static int litex_mmc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct litex_mmc_host *host;
	struct device_node *node, *cpu;
	struct mmc_host *mmc;
	int ret;

	node = pdev->dev.of_node;
	if (!node)
		return -ENODEV;

	host = devm_kzalloc(&pdev->dev, sizeof(struct litex_mmc_host),
			    GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	mmc = mmc_alloc_host(sizeof(struct litex_mmc_host), &pdev->dev);
	/* NOTE: mmc_alloc_host() defaults to max_[req,seg]_size=PAGE_SIZE,
	 * max_blk_size=512, and sets max_blk_count accordingly (to 8); If,
	 * for some reason, we want to modify max_blk_count, we must also
	 * re-calculate max_[req,seg]_size=max_blk_size*max_blk_count!
	 */
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->dev = pdev;
	// Initial state
	host->clock = 0;

	cpu = of_get_next_cpu_node(NULL);
	ret = of_property_read_u32(cpu, "clock-frequency", &host->freq);
	of_node_put(cpu);
	if (ret) {
		pr_err("Couldn't find \"clock-frequency\" property in DT\n");
		goto err_exit;
	}

	init_completion(&host->cmd_done);
	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0)
		dev_err(&pdev->dev, "Failed to get IRQ, using polling\n");

	/* LiteSDCard only supports 4-bit bus width; therefore, we MUST inject
	 * a SET_BUS_WIDTH (acmd6) before the very first data transfer, earlier
	 * than when the mmc subsystem would normally get around to it!
	 */
	host->is_bus_width_set = false;
	host->app_cmd = false;

	if (dma_set_mask(&pdev->dev, DMA_BIT_MASK(32))){
		pr_err("Unable to set DMA driver failed\n");
		ret = -EINVAL;
		goto err_exit;
	}

	host->buffer_size = mmc->max_req_size * 2;
	host->buffer = dma_alloc_coherent(&pdev->dev, host->buffer_size,
					  &host->dma_handle, GFP_DMA);
	if (host->buffer == NULL) {
		pr_err("could not allocate dma buffer\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	MAP_RESOURCE(sdphy, 0);
	MAP_RESOURCE(sdcore, 1);
	MAP_RESOURCE(sdreader, 2);
	MAP_RESOURCE(sdwriter, 3);

	if (host->irq > 0)
		MAP_RESOURCE(sdirq, 4);

	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->ops = &litex_mmc_ops;

	mmc->f_min = 12.5e6;
	mmc->f_max = 50e6; // 50Mhz is max frequency sd card can support

	ret = mmc_of_parse(mmc);
	if (ret) {
		pr_err("couldn't parse DT node\n");
		goto err_exit;
	}

	/* add set-by-default capabilities */
	mmc->caps |= MMC_CAP_WAIT_WHILE_BUSY | MMC_CAP_DRIVER_TYPE_D |
		MMC_CAP_CMD23;
	/* default to "disable-wp", "full-pwr-cycle", "no-sdio" */
	mmc->caps2 |= MMC_CAP2_NO_WRITE_PROTECT |
		      MMC_CAP2_FULL_PWR_CYCLE |
		      MMC_CAP2_NO_SDIO;

	platform_set_drvdata(pdev, host);

	ret = mmc_add_host(mmc);
	if (ret < 0) {
		pr_err("mmc_add_host() failed\n");
		goto err_exit;
	}

	/* ensure DMA bus masters are disabled */
	litex_write8(host->sdreader + LITEX_MMC_SDBLK2MEM_ENA_OFF, 0);
	litex_write8(host->sdwriter + LITEX_MMC_SDMEM2BLK_ENA_OFF, 0);

	/* set up interrupt handler */
	if (host->irq > 0) {
		ret = request_irq(host->irq, litex_mmc_interrupt, 0,
				  "litex-mmc", mmc);
		if (ret < 0) {
			dev_err(&pdev->dev, "error %d requesting irq, using polling\n",
				ret);
			host->irq = 0;
		}
	}

	/* enable card-change interrupts, or else ask for polling */
	if (host->irq > 0) {
		litex_write32(host->sdirq + LITEX_MMC_SDIRQ_PENDING_OFF,
			      SDIRQ_CARD_DETECT);	/* clears it */
		litex_write32(host->sdirq + LITEX_MMC_SDIRQ_ENABLE_OFF,
			      SDIRQ_CARD_DETECT);
	} else {
		mmc->caps |= MMC_CAP_NEEDS_POLL;
	}

	return 0;

err_exit:
	kfree(host->buffer);
	mmc_free_host(mmc);
	return ret;
}

static int litex_mmc_remove(struct platform_device *pdev)
{
	struct litex_mmc_host *host = dev_get_drvdata(&pdev->dev);

	if (host->irq > 0)
		free_irq(host->irq, host->mmc);
	mmc_remove_host(host->mmc);
	mmc_free_host(host->mmc);

	return 0;
}

static const struct of_device_id litex_match[] = {
	{ .compatible = "litex,mmc" },
	{},
};

MODULE_DEVICE_TABLE(of, litex_match);

static struct platform_driver litex_mmc_driver = {
	.driver = {
			.name = "litex-mmc",
			.of_match_table = of_match_ptr(litex_match),
		  },
	.probe = litex_mmc_probe,
	.remove = litex_mmc_remove,
};

module_platform_driver(litex_mmc_driver);

MODULE_DESCRIPTION("LiteX SDCard driver");
MODULE_AUTHOR("Antmicro <www.antmicro.com>");
MODULE_LICENSE("GPL v2");

