// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Antmicro <www.antmicro.com>
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
#include <linux/platform_device.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/of.h>
#include <linux/bits.h>
#include <linux/types.h>
#include <linux/kconfig.h>
#include <linux/litex.h>
#include <asm/byteorder.h>

#define OFFSET_REG_SINK_DATA     0x0
#define OFFSET_REG_SINK_READY    0x10

#define REG_SINK_DATA_SIZE       0x4
#define REG_SINK_READY_SIZE      0x1

#define INITIAL_HEADER_SIZE      -1 /* Set to maximum value */
#define ALLOWED_FPGA_MGR_FLAGS   (FPGA_MGR_PARTIAL_RECONFIG | \
				 FPGA_MGR_COMPRESSED_BITSTREAM)
#define BITSTREAM_INSTR_SIZE     sizeof(uint32_t)

/* Macros for accessing ICAP registers */

#define WRITE_SINK_DATA(mem, val)  litex_set_reg(mem + OFFSET_REG_SINK_DATA,  \
						 REG_SINK_DATA_SIZE, val)
#define READ_SINK_READY(mem)       litex_get_reg(mem + OFFSET_REG_SINK_READY, \
						 REG_SINK_READY_SIZE)

struct litex_fpga {
	void __iomem *membase;
};

/* Helper functions */

static inline bool bit_has_sync(uint8_t *buf, size_t count)
{
	int i;

	for (i = 0; i < count - BITSTREAM_INSTR_SIZE; i += BITSTREAM_INSTR_SIZE)
	/* Sync word is 0xAA995566 */
		if (buf[i] == 0xAA &&
		    buf[i + 1] == 0x99 &&
		    buf[i + 2] == 0x55 &&
		    buf[i + 3] == 0x66)
			return true;
	return false;
}

static inline bool bit_is_aligned(uint8_t *buf, size_t count)
{
	return !(count % BITSTREAM_INSTR_SIZE);
}

/* API functions */

static enum fpga_mgr_states litex_fpga_state(struct fpga_manager *mgr)
{
	return FPGA_MGR_STATE_UNKNOWN;
}

static int litex_fpga_write_init(struct fpga_manager *mgr,
				 struct fpga_image_info *info,
				 const char *buf, size_t count)
{
	/* Check if driver supports given operations */
	if (info->flags & ~ALLOWED_FPGA_MGR_FLAGS) {
		dev_err(&mgr->dev, "Unsupported bitstream flags occurred\n");
		return -EINVAL;
	}

	return 0;
}

static int litex_fpga_write(struct fpga_manager *mgr,
			    const char *buf, size_t count)
{
	const struct litex_fpga *fpga_s = (const struct litex_fpga *) mgr->priv;
	uint32_t *buf32;
	int i, count32;

	/* Bitstream should consist of 32bit words*/
	if (!bit_is_aligned((uint8_t *) buf, count)) {
		dev_err(&mgr->dev, "Invalid bitstream alignment\n");
		return -EINVAL;
	}

	/* Correct bitstream contains sync word */
	if (!bit_has_sync((uint8_t *) buf, count)) {
		dev_err(&mgr->dev, "Bitstream has no sync word\n");
		return -EINVAL;
	}

	buf32 = (uint32_t *) buf;
	count32 = count / BITSTREAM_INSTR_SIZE;
	for (i = 0; i < count32; ++i) {
		while (!READ_SINK_READY(fpga_s->membase))
			;
		WRITE_SINK_DATA(fpga_s->membase, be32_to_cpu(buf32[i]));
	}

	return 0;
}

static int litex_fpga_write_complete(struct fpga_manager *mgr,
				     struct fpga_image_info *info)
{
	return 0;
}

static const struct fpga_manager_ops litex_fpga_manager_ops = {
	.initial_header_size   = INITIAL_HEADER_SIZE,
	.state                 = litex_fpga_state,
	.write_init            = litex_fpga_write_init,
	.write                 = litex_fpga_write,
	.write_complete        = litex_fpga_write_complete,
};

/* Driver functions */

static int litex_fpga_remove(struct platform_device *pdev)
{
	struct fpga_manager *mgr;

	mgr = platform_get_drvdata(pdev);
	fpga_mgr_unregister(mgr);

	return 0;
}

static int litex_fpga_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct litex_fpga *fpga_s;
	struct fpga_manager *mgr;
	struct resource *res;

	if (!node)
		return -ENODEV;

	fpga_s = devm_kzalloc(&pdev->dev, sizeof(*fpga_s), GFP_KERNEL);
	if (!fpga_s)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EBUSY;

	fpga_s->membase = devm_of_iomap(&pdev->dev, node, 0, &res->end);
	if (IS_ERR_OR_NULL(fpga_s->membase))
		return -EIO;

	mgr = devm_fpga_mgr_create(&pdev->dev,
				   "LiteX ICAPBitstream FPGA Manager",
				   &litex_fpga_manager_ops, fpga_s);
	if (!mgr)
		return -ENOMEM;

	platform_set_drvdata(pdev, mgr);
	return fpga_mgr_register(mgr);
}

static const struct of_device_id litex_of_match[] = {
	{.compatible = "litex,fpga-icap"},
	{},
};

MODULE_DEVICE_TABLE(of, litex_of_match);

static struct platform_driver litex_fpga_driver = {
	.driver		= {
		.name		= "litex-icap-fpga-mgr",
		.of_match_table = of_match_ptr(litex_of_match)
	},
	.probe		= litex_fpga_probe,
	.remove	= litex_fpga_remove
};

module_platform_driver(litex_fpga_driver);

MODULE_DESCRIPTION("LiteX ICAPBitstream FPGA Manager driver");
MODULE_AUTHOR("Antmicro <www.antmicro.com>");
MODULE_LICENSE("GPL v2");
