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
#include <linux/litex.h>
#include <linux/dma-mapping.h>
#include <linux/sched/clock.h>

struct sdphy_regs {
	u32 volatile carddetect[1];
	u32 volatile clockerdivider[1];
	u32 volatile initinitialize[1];
	u32 volatile write_status[1];
};

struct sdcore_regs {
	u32 volatile cmdargument[1];
	u32 volatile cmdcommand[1];
	u32 volatile cmdsend[1];
	u32 volatile cmdresponse[4];
	u32 volatile cmdevent[1];
	u32 volatile dataevent[1];
	u32 volatile blocklength[1];
	u32 volatile blockcount[1];
};

struct sdblock2mem_regs {
	u32 volatile base[2];
	u32 volatile length[1];
	u32 volatile enable[1];
	u32 volatile done[1];
	u32 volatile loop[1];
};

struct sdmem2block_regs{
	u32 volatile base[2];
	u32 volatile length[1];
	u32 volatile enable[1];
	u32 volatile done[1];
	u32 volatile loop[1];
	u32 volatile offset[1];
};


#define MAX_NR_SG 1

#define SDCARD_CTRL_DATA_TRANSFER_NONE  0
#define SDCARD_CTRL_DATA_TRANSFER_READ  1
#define SDCARD_CTRL_DATA_TRANSFER_WRITE 2

#define SDCARD_CTRL_RESPONSE_NONE  0
#define SDCARD_CTRL_RESPONSE_SHORT 1
#define SDCARD_CTRL_RESPONSE_LONG  2

#define SD_OK 0
#define SD_CRCERROR 1
#define SD_TIMEOUT 2
#define SD_WRITEERROR 3

#define write_reg(reg, data) litex_set_reg(reg, sizeof(reg), data)

#define read_reg(reg) litex_get_reg(reg, sizeof(reg))

#define MAX_NR_BLOCKS 1
#define DATA_BLOCK_SIZE 512

struct litex_mmc_host {
	struct mmc_host *mmc;
	struct platform_device *dev;

	struct {
		__iomem struct sdphy_regs *sdphy;
		__iomem struct sdcore_regs *sdcore;
		__iomem struct sdblock2mem_regs *sdreader;
		__iomem struct sdmem2block_regs *sdwriter;
	} regs;

	u32 resp[4];
	u16 rca;

	u8 volatile *buffer;
	size_t buffer_size;
	dma_addr_t dma_handle;

	unsigned int freq;
	unsigned int clock;
	unsigned char bus_width;
	bool is_bus_width_set;
	bool app_cmd;
};

struct litex_mmc_config {
	int (*get_cd)(int module);
	int (*get_ro)(int module);
	void (*set_power)(int module, bool on);
	u32 max_freq;
	u32 caps;
	u8 nr_sg;
};


void sdclk_set_clk(struct litex_mmc_host *host, unsigned int clk_freq) {
	dev_info(&host->dev->dev, "Setting clk freq to: %d\n", clk_freq);
	unsigned int ratio;
	if (clk_freq == 0)
		ratio = 256;
	else
		ratio = host->freq / clk_freq;
	int r = 1;
	while((r<<1) <= ratio) r<<=1;
	int new_val = max(r, 2);
	new_val = min(new_val, 256);
	pr_info("Frequency set to %d with divider %d\n", host->freq/r, new_val);
	write_reg(host->regs.sdphy->clockerdivider, r);
}


static void litex_mmc_read_rsp(struct litex_mmc_host *host, u32 resp[4]) {
	static const u8 reg_size = sizeof(host->regs.sdcore->cmdresponse) / 4;
	unsigned offset;
	unsigned i;
	u8 value;

	resp[0] = resp[1] = resp[2] = resp[3] = 0;

	for (i = 0; i < reg_size; i++) {
		offset = reg_size - 1 - i;
		value = ioread32(&host->regs.sdcore->cmdresponse[offset]) & 0xFF;
		resp[(sizeof(*resp) - 1) - i / 4] |= value << (8 * (i % 4));
	}
}

static void litex_issue_cmd(struct litex_mmc_host *host) {
	write_reg(host->regs.sdcore->cmdsend, 1);
}

static int sdcard_wait_done(__iomem u32 volatile  *reg) {
	u32 evt;
	while (1) {
		evt = read_reg(reg);
		udelay(5);
		if (evt & 0x1) {
			if (evt & 0x4) {
				return SD_TIMEOUT;
			}
			if (evt & 0x8) {
				return SD_CRCERROR;
			}
			return SD_OK;
		}
	}
}

static int send_cmd(struct litex_mmc_host *host, u8 cmd, u32 arg,
		    u8 response_len, u8 transfer) {
	unsigned long n;
	int status = SD_OK;
	unsigned done = 0;
	__iomem u32 volatile *read_mem;
	__iomem u32 volatile *reg2;

	write_reg(host->regs.sdcore->cmdargument, arg);
	write_reg(host->regs.sdcore->cmdcommand,
		cmd << 8 | transfer << 5 | response_len);

	litex_issue_cmd(host);

	if (transfer != SDCARD_CTRL_DATA_TRANSFER_NONE)
		reg2 = (transfer == SDCARD_CTRL_DATA_TRANSFER_READ) ?
			host->regs.sdreader->done : host->regs.sdwriter->done;

	status = sdcard_wait_done(host->regs.sdcore->cmdevent);
	if(status != SD_OK) {
		pr_err("Command failed with status %d\n", status);
		return status;
	}

	if (response_len != SDCARD_CTRL_RESPONSE_NONE) {
		litex_mmc_read_rsp(host, host->resp);
	}

	if (!host->app_cmd && cmd == SD_SEND_RELATIVE_ADDR) {
		host->rca = (host->resp[3] >> 16) & 0xffff;
	}

	if(transfer == SDCARD_CTRL_DATA_TRANSFER_NONE)
		return SD_OK; // we are returning here SD_OK as we already checked status of sdcard_wait_done
	read_mem = transfer == SDCARD_CTRL_DATA_TRANSFER_READ ?
	    host->regs.sdreader->done : host->regs.sdwriter->done;

	status = sdcard_wait_done(host->regs.sdcore->dataevent);
	if (status != SD_OK){
		pr_err("Data transfer failed with status %d\n", status);
		return status;
	}

	n = jiffies + 2 * HZ;
	while (read_reg(read_mem) != 0x01)
		if(time_after(jiffies, n)){
			pr_err("DMA timeout\n");
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

/*
 Return values for the get_ro callback should be:
*   0 for a read/write card
*   1 for a read-only card
*   -ENOSYS when not supported (equal to NULL callback)
*   or a negative errno value when something bad happened
*/
static int litex_get_ro(struct mmc_host *mmc)
{
	struct platform_device *pdev = to_platform_device(mmc->parent);
	struct litex_mmc_config *config = pdev->dev.platform_data;
	int ro;

	if (config && config->get_ro) {
		ro = config->get_ro(pdev->id);
	} else {
		ro = mmc_gpio_get_ro(mmc);
	}

	return ro;
}

static int litex_get_cd(struct mmc_host *mmc)
{
	struct litex_mmc_host *host = mmc_priv(mmc);
	int gpio_cd = mmc_gpio_get_cd(mmc);

	if (!mmc_card_is_removable(host->mmc))
		return 1;

	if(gpio_cd >= 0)
		return !!gpio_cd;

	return !read_reg(host->regs.sdphy->carddetect);
}

static int litex_set_bus_width(struct litex_mmc_host *host) {
	int status;
	// Check if last cmd was app cmd, if not, send app cmd
	if (!host->app_cmd) {
		send_app_cmd(host);
	}
	status = send_app_set_bus_width_cmd(host, host->bus_width);
	if (status == SD_OK) {
		host->is_bus_width_set = true;
	}
	// If last command was app cmd, redo it for this command
	if (host->app_cmd) {
		send_app_cmd(host);
	}
	return status;
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
	int status;

	struct mmc_data *data = mrq->data;
	struct mmc_command *cmd = mrq->cmd;
	int retries = cmd->retries;

	u32 response_len = SDCARD_CTRL_RESPONSE_NONE;
	u32 transfer = SDCARD_CTRL_DATA_TRANSFER_NONE;


	int i;

	if (cmd->flags & MMC_RSP_136) {
		response_len = SDCARD_CTRL_RESPONSE_LONG;
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		response_len = SDCARD_CTRL_RESPONSE_SHORT;
	}

	if (data) {
		// This is a good time (i.e. last moment) to finally set bus
		// width. Ofc if not set yet.
		if (!host->is_bus_width_set) {
			unsigned long n;
			n = jiffies + 2 * HZ; //500ms timeout
			while(litex_set_bus_width(host) != SD_OK) {
				if(time_after(jiffies, n)) {
					dev_warn(dev, "Could not set bus width!\n");
					cmd->error = -ETIMEDOUT;
					mmc_request_done(mmc, mrq);
					return;
				}
			}
		}


		if (mrq->data->flags & MMC_DATA_READ) {
			write_reg(host->regs.sdreader->enable,0);
			write_reg(host->regs.sdreader->base,host->dma_handle);
			write_reg(host->regs.sdreader->length,
					data->blksz*data->blocks);
			write_reg(host->regs.sdreader->enable,1);

			transfer = SDCARD_CTRL_DATA_TRANSFER_READ;

		} else if (mrq->data->flags & MMC_DATA_WRITE) {
			int write_length = min(data->blocks*data->blksz,
					       host->buffer_size);

			sg_copy_to_buffer(data->sg, data->sg_len,
					host->buffer, write_length);

			write_reg(host->regs.sdwriter->enable,0);
			write_reg(host->regs.sdwriter->base,host->dma_handle);
			write_reg(host->regs.sdwriter->length,
					write_length);
			write_reg(host->regs.sdwriter->enable,1);

			transfer = SDCARD_CTRL_DATA_TRANSFER_WRITE;
		} else {
			dev_warn(dev,
		"Data present without read or write flag.\n");
			// Intentionally continue here to set cmd status and mark request done
		}

		write_reg(host->regs.sdcore->blocklength,data->blksz);
		write_reg(host->regs.sdcore->blockcount,data->blocks);

	}

	do {
		status = send_cmd(host, cmd->opcode, cmd->arg,
				response_len, transfer);
	} while(retries-- > 0 && status != SD_OK);

	switch(status) {
		case SD_OK:		cmd->error = 0; break;
		case SD_TIMEOUT:        cmd->error = -ETIMEDOUT; break;
		case SD_CRCERROR:       cmd->error = -EILSEQ; break;
		case SD_WRITEERROR:     cmd->error = -EINVAL; break;
		default:                cmd->error = -EINVAL; break;
	}

	// It looks strange I know, but it's as it should be
	if (response_len == SDCARD_CTRL_RESPONSE_SHORT) {
		cmd->resp[0] = host->resp[3];
		cmd->resp[1] = host->resp[2]&0xFF;
	} else if (SDCARD_CTRL_RESPONSE_LONG) {
		cmd->resp[0] = host->resp[0];
		cmd->resp[1] = host->resp[1];
		cmd->resp[2] = host->resp[2];
		cmd->resp[3] = host->resp[3];
	}


	if (status == SD_OK && transfer != SDCARD_CTRL_DATA_TRANSFER_NONE) {
		data->bytes_xfered = min(data->blksz*data->blocks,
				(u32)MAX_NR_BLOCKS*DATA_BLOCK_SIZE);
		if (transfer == SDCARD_CTRL_DATA_TRANSFER_READ) {
			sg_copy_from_buffer(data->sg, sg_nents(data->sg),
				host->buffer, data->bytes_xfered);
		}
	}

	host->app_cmd = (cmd->opcode == MMC_APP_CMD && status == SD_OK);

	mmc_request_done(mmc, mrq);
}

static void litex_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct litex_mmc_host *host;

	host = mmc_priv(mmc);

	if (ios->clock != host->clock) {
		sdclk_set_clk(host, ios->clock);
		host->clock = ios->clock;
	}

	// Technically there should be some code forsetting requested bus width.
	// We can't send "send_bus_width" command to the card at this moment
	// because the card is not in the right state (the command is only accepted
	// in "trans" state). Requested width should be stored and send when it is
	// possible (see litex_request), but litesdcard supports only 4 bit bus
	// width, so it is known what must be set.
}

static const struct mmc_host_ops litex_mmc_ops = {
	.get_cd = litex_get_cd,
	.get_ro = litex_get_ro,
	.request = litex_request,
	.set_ios = litex_set_ios,
};

#define RESOURCE_OFFSET(host, resource_ptr_offsets, i) *(void **)((char *)host + resource_ptr_offsets[i])

#define MAP_RESOURCE(host, resource_ptr_offsets, pdev, i) \
{ \
	res = platform_get_resource(pdev, IORESOURCE_MEM, i); \
	if (!res) { \
		dev_err(&pdev->dev, "Resource %d missing\n", i); \
		ret = -ENODEV; \
		goto err_exit; \
	} \
	RESOURCE_OFFSET(host, resource_ptr_offsets, i) = \
		devm_ioremap_resource(&pdev->dev, res); \
	if (!RESOURCE_OFFSET(host,resource_ptr_offsets, i)) { \
		dev_err(&pdev->dev, "Couldn't map resource %d\n", i); \
		ret = -ENOMEM; \
		goto err_exit; \
	} \
}

static int litex_mmc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct litex_mmc_host *host;
	struct device_node *node,*cpus;
	struct mmc_host *mmc;
	int ret;
	int i;

	static const size_t resource_ptr_offsets[] = {
		offsetof(struct litex_mmc_host, regs.sdphy),
		offsetof(struct litex_mmc_host, regs.sdcore),
		offsetof(struct litex_mmc_host, regs.sdreader),
		offsetof(struct litex_mmc_host, regs.sdwriter)
	};

	node = pdev->dev.of_node;
	if (!node) {
		return -ENODEV;
	}

	host = devm_kzalloc(&pdev->dev, sizeof(struct litex_mmc_host),
			    GFP_KERNEL);
	if (!host) {
		return -ENOMEM;
	}

	mmc = mmc_alloc_host(sizeof(struct litex_mmc_host), &pdev->dev);
	if (!mmc) {
		return -ENOMEM;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->dev = pdev;
	// Initial state
	host->clock = 0;

	cpus = of_find_node_by_name(NULL, "cpus");
	ret = of_property_read_u32(cpus,"timebase-frequency",&host->freq);
	of_node_put(cpus);
	if(ret){
		goto err_exit;
	}

	// litesdcard only supports 4-bit bus width
	host->bus_width = MMC_BUS_WIDTH_4;
	host->is_bus_width_set = false;
	host->app_cmd = false;

	if (dma_set_mask(&pdev->dev, DMA_BIT_MASK(32))){
		pr_err("Unable to set DMA driver failed\n");
		goto err_exit;
	}

	host->buffer_size = MAX_NR_BLOCKS*DATA_BLOCK_SIZE*2;
	host->buffer = dma_alloc_coherent(&pdev->dev, host->buffer_size,
					  &host->dma_handle, GFP_DMA);
	if(host->buffer == NULL){
		goto err_exit;
	}

	for (i = 0; i < ARRAY_SIZE(resource_ptr_offsets); ++i) {
		MAP_RESOURCE(host, resource_ptr_offsets, pdev, i);
	}

	ret = mmc_of_parse(mmc);
	if (ret) {
		goto err_exit;
	}

	mmc->caps = MMC_CAP_WAIT_WHILE_BUSY | MMC_CAP_DRIVER_TYPE_D;
	mmc->caps2 = MMC_CAP2_NO_SDIO | MMC_CAP2_FULL_PWR_CYCLE | MMC_CAP2_NO_WRITE_PROTECT;
	mmc->ops = &litex_mmc_ops;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

	mmc->max_segs = MAX_NR_SG;
	mmc->max_seg_size = 512;
	mmc->max_blk_size = DATA_BLOCK_SIZE;
	mmc->max_blk_count = MAX_NR_BLOCKS;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;

	mmc->f_min = 125 * 1e5; // sys_clk/256 is minimal frequency mmcm can produce, set minimal to 12.5Mhz on lower frequencies, sdcard sometimes do not initialize properly
	mmc->f_max = 50 * 1e6; // 50Mhz is max frequency sd card can support

	platform_set_drvdata(pdev, host);

	ret = mmc_add_host(mmc);
	if (ret < 0) {
		goto err_exit;
	}

	write_reg(host->regs.sdreader->enable,0);
	write_reg(host->regs.sdreader->base,0);
	write_reg(host->regs.sdreader->length,0);

	write_reg(host->regs.sdwriter->enable,0);
	write_reg(host->regs.sdwriter->base,0);
	write_reg(host->regs.sdwriter->length,0);
	return 0;

err_exit:
	kfree(host->buffer);
	mmc_free_host(mmc);
	return ret;
}

static int litex_mmc_remove(struct platform_device *pdev)
{
	struct litex_mmc_host *host = dev_get_drvdata(&pdev->dev);

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

