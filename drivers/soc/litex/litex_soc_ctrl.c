// SPDX-License-Identifier: GPL-2.0
/*
 * LiteX SoC Controller Driver
 *
 * Copyright (C) 2020 Antmicro
 *
 */

#include <linux/litex.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/io.h>

/* scratch register located one CPU x-len past the base address */
#ifdef CONFIG_64BIT
#define SCRATCH_REG_OFF     8
#else
#define SCRATCH_REG_OFF     4
#endif

#define SCRATCH_REG_VALUE   0x12345678
#define SCRATCH_TEST_VALUE  0xdeadbeef

static int accessors_ok = 0;

/*
 * Check if accessors are safe to be used by other drivers
 * returns true if yes - false if not
 */
int litex_check_accessors(void)
{
	return accessors_ok;
}

struct litex_soc_ctrl_device {
	void __iomem *base;
};

/* Check LiteX CSR read/write access */
static int litex_check_csr_access(void __iomem *reg_addr)
{
	u32 reg;

	reg = litex_reg_readl(reg_addr + SCRATCH_REG_OFF);

	if (reg != SCRATCH_REG_VALUE) {
		panic("Scratch register read error! Expected: 0x%x, got: 0x%x",
		      SCRATCH_REG_VALUE, reg);
		return -EINVAL;
	}

	litex_reg_writel(reg_addr + SCRATCH_REG_OFF, SCRATCH_TEST_VALUE);
	reg = litex_reg_readl(reg_addr + SCRATCH_REG_OFF);

	if (reg != SCRATCH_TEST_VALUE) {
		panic("Scratch register write error! Expected: 0x%x, got: 0x%x",
		      SCRATCH_TEST_VALUE, reg);
		return -EINVAL;
	}

	/* restore original value of the SCRATCH register */
	litex_reg_writel(reg_addr + SCRATCH_REG_OFF, SCRATCH_REG_VALUE);

	/* Set flag for other drivers */
	accessors_ok = 1;
	pr_info("LiteX SoC Controller driver initialized: subreg=%d, align=%d",
		LITEX_SUBREG_SIZE, LITEX_SUBREG_ALIGN);

	return 0;
}

static const struct of_device_id litex_soc_ctrl_of_match[] = {
	{.compatible = "litex,soc_controller"},
	{},
};

MODULE_DEVICE_TABLE(of, litex_soc_ctrl_of_match);

static int litex_soc_ctrl_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct device_node *node;
	const struct of_device_id *id;
	struct litex_soc_ctrl_device *soc_ctrl_dev;
	struct resource *res;

	dev = &pdev->dev;
	node = dev->of_node;
	if (!node)
		return -ENODEV;

	id = of_match_node(litex_soc_ctrl_of_match, node);
	if (!id)
		return -ENODEV;

	soc_ctrl_dev = devm_kzalloc(dev, sizeof(*soc_ctrl_dev), GFP_KERNEL);
	if (IS_ERR_OR_NULL(soc_ctrl_dev))
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(res))
		return -EBUSY;

	soc_ctrl_dev->base = devm_of_iomap(dev, node, 0, &res->end);
	if (IS_ERR_OR_NULL(soc_ctrl_dev->base))
		return -EIO;

	return litex_check_csr_access(soc_ctrl_dev->base);
}

static int litex_soc_ctrl_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver litex_soc_ctrl_driver = {
	.driver = {
		.name = "litex-soc-controller",
		.of_match_table = of_match_ptr(litex_soc_ctrl_of_match)
	},
	.probe = litex_soc_ctrl_probe,
	.remove = litex_soc_ctrl_remove
};

module_platform_driver(litex_soc_ctrl_driver);
MODULE_DESCRIPTION("LiteX SoC Controller driver");
MODULE_AUTHOR("Antmicro <www.antmicro.com>");
