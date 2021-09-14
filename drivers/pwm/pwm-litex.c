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

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/stmp_device.h>
#include <linux/litex.h>

#define REG_EN_ENABLE           0x1
#define REG_EN_DISABLE          0x0

#define ENABLE_REG_OFFSET       0x0
#define WIDTH_REG_OFFSET        0x4
#define PERIOD_REG_OFFSET       0x14

struct litex_pwm_chip {
	struct pwm_chip chip;
	unsigned int clock;
	void __iomem *base;
	void __iomem *width;
	void __iomem *period;
	void __iomem *enable;
};

#define to_litex_pwm_chip(_chip) container_of(_chip, struct litex_pwm_chip, chip)

static int litex_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			  int duty_ns, int period_ns)
{
	struct litex_pwm_chip *litex = to_litex_pwm_chip(chip);
	unsigned long period_cycles, duty_cycles;
	unsigned long long c;

	/* Calculate period cycles */
	c = (unsigned long long)litex->clock * (unsigned long long)period_ns;
	do_div(c, NSEC_PER_SEC);
	period_cycles = c;

	/* Calculate duty cycles */
	c *= duty_ns;
	do_div(c, period_ns);
	duty_cycles = c;

    /* Apply values to registers */
	litex_write32(litex->width, duty_cycles);
	litex_write32(litex->period, period_cycles);

	return 0;
}

static int litex_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct litex_pwm_chip *litex = to_litex_pwm_chip(chip);

	litex_write8(litex->enable, REG_EN_ENABLE);
	return 0;
}

static void litex_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct litex_pwm_chip *litex = to_litex_pwm_chip(chip);

	litex_write8(litex->enable, REG_EN_DISABLE);
}

static const struct pwm_ops litex_pwm_ops = {
	.config = litex_pwm_config,
	.enable = litex_pwm_enable,
	.disable = litex_pwm_disable,
	.owner = THIS_MODULE,
};

static int litex_pwm_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct litex_pwm_chip *litex;
	struct resource *res;
	int ret;

	if (!node) {
		dev_err(&pdev->dev, "Fail on obtaining device node\n");
		return -ENOMEM;
	}

	litex = devm_kzalloc(&pdev->dev, sizeof(*litex), GFP_KERNEL);

	if (!litex) {
		dev_err(&pdev->dev, "Fail on memory allocation\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Device is busy\n");
		return -EBUSY;
	}

	litex->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(litex->base)) {
		dev_err(&pdev->dev, "Fail to get base address\n");
		return -EIO;
	}

	ret = of_property_read_u32(node, "clock", &(litex->clock));
	if (ret < 0) {
		dev_err(&pdev->dev, "No clock record in the dts file\n");
		return -ENODEV;
	}

	litex->width = litex->base + WIDTH_REG_OFFSET;
	litex->period = litex->base + PERIOD_REG_OFFSET;
	litex->enable = litex->base + ENABLE_REG_OFFSET;

	litex->chip.dev = &pdev->dev;
	litex->chip.ops = &litex_pwm_ops;
	litex->chip.base = -1;
	litex->chip.npwm = 1;

	ret = pwmchip_add(&litex->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to add pwm chip %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, litex);

	return 0;
}

static int litex_pwm_remove(struct platform_device *pdev)
{
	struct litex_pwm_chip *litex = platform_get_drvdata(pdev);

	pwmchip_remove(&litex->chip);
	return 0;
}

static const struct of_device_id litex_of_match[] = {
	{ .compatible = "litex,pwm" },
	{}
};
MODULE_DEVICE_TABLE(of, litex_of_match);

static struct platform_driver litex_pwm_driver = {
	.driver = {
		.name = "litex-pwm",
		.of_match_table   = of_match_ptr(litex_of_match)
	},
	.probe = litex_pwm_probe,
	.remove = litex_pwm_remove,
};
module_platform_driver(litex_pwm_driver);

MODULE_DESCRIPTION("LiteX PWM driver");
MODULE_AUTHOR("Antmicro <www.antmicro.com>");
MODULE_LICENSE("GPL v2");
