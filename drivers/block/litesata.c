// SPDX-License-Identifier: GPL-2.0
/*
 * LiteX LiteSATA block driver
 *
 * Copyright (c) 2022 Gabriel Somlo <gsomlo@gmail.com>
 */

/*

NOTE: This is an alpha/experimental driver for the LiteSATA block device.
	- still needs IRQ support and some cleaning up!

Example DTS node (adjust with your own addresses from csr.csv):
	...
	soc {
		...
                litesata0: litesata@12003000 {
                        compatible = "litex,litesata";
                        reg = <0x12003000 0x100>,
                                <0x12004800 0x100>,
                                <0x12005000 0x100>,
                                <0x12004000 0x100>,
                                <0x12003800 0x100>;
                        reg-names = "ident", "phy", "reader", "writer", "irq";
                        interrupt-parent = <&L1>;
                        interrupts = <4>;
                };
		...

	}
	...
*/

#include <linux/bits.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/litex.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define LITESATA_ID_STRT   0x00 // 1bit, w
#define LITESATA_ID_DONE   0x04 // 1bit, ro
#define LITESATA_ID_DWIDTH 0x08 // 16bit, ro
#define LITESATA_ID_SRCVLD 0x0c // 1bit, ro
#define LITESATA_ID_SRCRDY 0x10 // 1bit, w
#define LITESATA_ID_SRCDAT 0x14 // 32bit, ro

#define LITESATA_PHY_ENA   0x00
#define LITESATA_PHY_STS   0x04

#define PHY_STS_RDY BIT(0)
#define PHY_STS_TX  BIT(1)
#define PHY_STS_RX  BIT(2)
#define PHY_STS_CTL BIT(3)

#define LITESATA_DMA_SECT  0x00
#define LITESATA_DMA_ADDR  0x08
#define LITESATA_DMA_STRT  0x10
#define LITESATA_DMA_DONE  0x14
#define LITESATA_DMA_ERR   0x18

struct litesata_dev {
	struct device *dev;

	void __iomem *lsident;
	void __iomem *lsphy;
	void __iomem *lsreader;
	void __iomem *lswriter;
};

static int litesata_do_dma_sector(void __iomem *regs,
				  sector_t sector, dma_addr_t dma)
{
	unsigned int retry = 16;

	while (retry--) {
		/* NOTE: do *not* start by writing 0 to LITESATA_DMA_STRT!!! */
		litex_write64(regs + LITESATA_DMA_SECT, sector);
		litex_write64(regs + LITESATA_DMA_ADDR, dma);
		litex_write8(regs + LITESATA_DMA_STRT, 1);

		/* wait for dma transfer completion */
		while ((litex_read8(regs + LITESATA_DMA_DONE) & 0x01) == 0);

		if ((litex_read8(regs + LITESATA_DMA_ERR) & 0x01) == 0)
			return 0;

		msleep(10);
	}

	return -EIO;
}

static int litesata_do_dma(void __iomem *regs, dma_addr_t dma,
			   sector_t sector, unsigned int count)
{
	int err;

	while (count--) {
		err = litesata_do_dma_sector(regs, sector, dma);
		if (err)
			return err;

		sector++;
		dma += SECTOR_SIZE;
	}

	return 0;
}

/* Process a single bvec of a bio. */
static int litesata_do_bvec(struct litesata_dev *lbd, struct bio_vec *bv,
			    unsigned int op, sector_t sector)
{
	struct device *dev = lbd->dev;
	dma_addr_t dma;
	void __iomem *regs;
	int err;
	enum dma_data_direction dir;

	if (op_is_write(op)) {
		dir = DMA_TO_DEVICE;
		regs = lbd->lswriter;
	} else {
		dir = DMA_FROM_DEVICE;
		regs = lbd->lsreader;
	}

	dma = dma_map_bvec(dev, bv, dir, 0);
	err = dma_mapping_error(dev, dma);
	if (err)
		return err;

	err = litesata_do_dma(regs, dma, sector, (bv->bv_len >> SECTOR_SHIFT));
	if (err)
		return err;

	dma_unmap_page(dev, dma, bv->bv_len, dir);
	return 0;
}

static void litesata_submit_bio(struct bio *bio)
{
	struct litesata_dev *lbd = bio->bi_bdev->bd_disk->private_data;
	unsigned int op = bio_op(bio);
	sector_t sector = bio->bi_iter.bi_sector;
	struct bio_vec bvec;
	struct bvec_iter iter;

	bio_for_each_segment(bvec, bio, iter) {
		int err;

		/* Don't support un-aligned buffer */
		WARN_ON_ONCE((bvec.bv_offset & (SECTOR_SIZE - 1)) ||
				(bvec.bv_len & (SECTOR_SIZE - 1)));

		err = litesata_do_bvec(lbd, &bvec, op, sector);
		if (err) {
			dev_err(lbd->dev, "error %s sectors %Ld..%Ld\n",
				op_is_write(op) ? "writing" : "reading",
				sector, sector + (bvec.bv_len >> SECTOR_SHIFT));
			bio_io_error(bio);
			return;
		}
		sector += (bvec.bv_len >> SECTOR_SHIFT);
	}

	bio_endio(bio);
}

static const struct block_device_operations litesata_fops = {
	.owner		= THIS_MODULE,
	.submit_bio	= litesata_submit_bio,
};

static int litesata_init_ident(struct litesata_dev *lbd, sector_t *size)
{
	int i;
	u32 data;
	u16 buf[128];
	u8 model[38];

	/* reset phy */
	litex_write8(lbd->lsphy + LITESATA_PHY_ENA, 0);
	msleep(1);
	litex_write8(lbd->lsphy + LITESATA_PHY_ENA, 1);
	msleep(100);
	/* check phy status */
	if ((litex_read8(lbd->lsphy + LITESATA_PHY_STS) & 0x01) == 0)
		return -ENODEV;

	/* initiate `identify` sequence */
	litex_write8(lbd->lsident + LITESATA_ID_STRT, 1);
	msleep(100);
	/* check `identify` status */
	if ((litex_read8(lbd->lsident + LITESATA_ID_DONE) & 0x01) == 0)
		return -ENODEV;

	/* read `identify` response into buf */
	// FIXME: make buf be u32[64], read in-place, and use le/be/2cpu
	for(i = 0;
	    i < 128 && litex_read8(lbd->lsident + LITESATA_ID_SRCVLD);
	    i += 2) {
		data = litex_read32(lbd->lsident + LITESATA_ID_SRCDAT);
		litex_write8(lbd->lsident + LITESATA_ID_SRCRDY, 1);
		buf[i + 0] = ((data >>  0) & 0xffff);
		buf[i + 1] = ((data >> 16) & 0xffff);
	}
	/* get disk model */
	// FIXME: there's gotta be a better way to do this :)
	for (i = 0; i < 18; i++) {
		model[2*i + 0] = (buf[27+i] >> 8) & 0xff;
		model[2*i + 1] = (buf[27+i] >> 0) & 0xff;
	}
	model[36] = model[37] = 0;
	/* get disk capacity */
	*size = 0;
	*size += (((u64) buf[100]) <<  0);
	*size += (((u64) buf[101]) << 16);
	*size += (((u64) buf[102]) << 32);
	*size += (((u64) buf[103]) << 48);

	/* success */
	dev_info(lbd->dev, "%Ld bytes; %s\n", *size << SECTOR_SHIFT, model);
	return 0;
}

static void litesata_cleanup_disk(void *disk)
{
	blk_cleanup_disk(disk);
}

static int litesata_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct litesata_dev *lbd;
	struct gendisk *disk;
	sector_t size;
	int t;
	int err = -ENOMEM;

	lbd = devm_kzalloc(dev, sizeof(*lbd), GFP_KERNEL);
	if (!lbd)
		return -ENOMEM;

	lbd->dev = dev;

	lbd->lsident = devm_platform_ioremap_resource_byname(pdev, "ident");
	if (IS_ERR(lbd->lsident))
		return PTR_ERR(lbd->lsident);

	lbd->lsphy = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(lbd->lsphy))
		return PTR_ERR(lbd->lsphy);

	lbd->lsreader = devm_platform_ioremap_resource_byname(pdev, "reader");
	if (IS_ERR(lbd->lsreader))
		return PTR_ERR(lbd->lsreader);

	lbd->lswriter = devm_platform_ioremap_resource_byname(pdev, "writer");
	if (IS_ERR(lbd->lswriter))
		return PTR_ERR(lbd->lswriter);

	// FIXME: use some sort of macro/wrapper for the retry/timeout count
	t = 16; do
		err = litesata_init_ident(lbd, &size);
	while (err && t--);
	if (err)
		return err;

	disk = blk_alloc_disk(NUMA_NO_NODE);
	if (!disk)
		return -ENOMEM;

	err = devm_add_action_or_reset(dev, litesata_cleanup_disk, disk);
	if (err)
		return dev_err_probe(dev, err,
				     "Can't register cleanup_disk action\n");

	disk->private_data = lbd;
	disk->fops = &litesata_fops;
	strcpy(disk->disk_name, "litesata");
	set_capacity(disk, size);
	blk_queue_flag_set(QUEUE_FLAG_NONROT, disk->queue);
	blk_queue_physical_block_size(disk->queue, SECTOR_SIZE);
	blk_queue_logical_block_size(disk->queue, SECTOR_SIZE);

	err = add_disk(disk);
	if (err)
		return err;

	dev_info(dev, "probe success; sector size = %d\n", SECTOR_SIZE);
	return 0;
}

static const struct of_device_id litesata_match[] = {
	{ .compatible = "litex,litesata" },
	{ }
};
MODULE_DEVICE_TABLE(of, litesata_match);

static struct platform_driver litesata_driver = {
	.probe = litesata_probe,
	.driver = {
		.name = "litesata",
		.of_match_table = litesata_match,
	},
};
module_platform_driver(litesata_driver);

MODULE_DESCRIPTION("LiteX LiteSATA block driver");
MODULE_AUTHOR("Gabriel Somlo <gsomlo@gmail.com>");
MODULE_LICENSE("GPL v2");
