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
#include <linux/of.h>
#include <linux/hwmon.h>
#include <linux/litex.h>

#define TEMP_REG_OFFSET               0x0
#define TEMP_REG_SIZE                 2
#define VCCINT_REG_OFFSET             0x8
#define VCCINT_REG_SIZE               2
#define VCCAUX_REG_OFFSET             0x10
#define VCCAUX_REG_SIZE               2
#define VCCBRAM_REG_OFFSET            0x18
#define VCCBRAM_REG_SIZE              2

#define CHANNEL_TEMP                  0
#define CHANNEL_VCCINT                0
#define CHANNEL_VCCAUX                1
#define CHANNEL_VCCBRAM               2

struct litex_hwmon {
	void __iomem     *membase;
	struct device    *hdev;
};

/* Transfer functions taken from XILINX UG480 (v1.10.1)
 * www.xilinx.com/support/documentation/user_guides/ug480_7Series_XADC.pdf
 */

static inline long litex_temp_transfer_fun(long val)
{
	return ((val * 503975ULL) / 4096ULL) - 273150ULL;
}

static inline long litex_supp_transfer_fun(long val)
{
	return ((val * 3000) / 4096);
}

static inline int litex_read_temp(struct litex_hwmon *hwmon_s, u32 attr,
				  int channel, long *val)
{
	unsigned long raw_data;

	if (attr != hwmon_temp_input)
		return -ENOTSUPP;

	if (channel != CHANNEL_TEMP)
		return -EINVAL;

	raw_data = litex_get_reg(hwmon_s->membase + TEMP_REG_OFFSET, TEMP_REG_SIZE);
	*val = litex_temp_transfer_fun(raw_data);
	return 0;
}

static inline int litex_read_in(struct litex_hwmon *hwmon_s, u32 attr,
				int channel, long *val)
{
	int offset;
	int size;
	unsigned long raw_data;

	if (attr != hwmon_in_input)
		return -ENOTSUPP;

	switch (channel) {
	case CHANNEL_VCCINT:
		offset = VCCINT_REG_OFFSET;
		size = VCCINT_REG_SIZE;
		break;
	case CHANNEL_VCCAUX:
		offset = VCCAUX_REG_OFFSET;
		size = VCCAUX_REG_SIZE;
		break;
	case CHANNEL_VCCBRAM:
		offset = VCCBRAM_REG_OFFSET;
		size = VCCBRAM_REG_SIZE;
		break;
	default:
		return -EINVAL;
	}

	raw_data = litex_get_reg(hwmon_s->membase + offset, size);
	*val = litex_supp_transfer_fun(raw_data);
	return 0;
}

/* API functions */

umode_t litex_hwmon_is_visible(const void *drvdata,
			       enum hwmon_sensor_types type,
			       u32 attr, int channel)
{
	return 0444;
}

int litex_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
		     u32 attr, int channel, long *val)
{
	struct litex_hwmon *hwmon_s = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		return litex_read_temp(hwmon_s, attr, channel, val);
	case hwmon_in:
		return litex_read_in(hwmon_s, attr, channel, val);
	default:
		return -ENOTSUPP;
	}

	return 0;
}

static const struct hwmon_ops litex_hwmon_ops = {
	.is_visible   = litex_hwmon_is_visible,
	.read         = litex_hwmon_read,
};

/* Attribute management */

static const unsigned int litex_temp_config[] = {
	HWMON_T_INPUT,
	0
};

static const struct hwmon_channel_info litex_hwmon_temp = {
	.type = hwmon_temp,
	.config = litex_temp_config
};

static const unsigned int litex_vcc_config[] = {
	HWMON_I_INPUT,
	HWMON_I_INPUT,
	HWMON_I_INPUT,
	0
};

static const struct hwmon_channel_info litex_hwmon_vcc = {
	.type = hwmon_in,
	.config = litex_vcc_config
};

static const struct hwmon_channel_info *litex_hwmon_channel_info[] = {
	&litex_hwmon_temp,
	&litex_hwmon_vcc,
	NULL
};

static const struct hwmon_chip_info litex_chip_info = {
	.ops = &litex_hwmon_ops,
	.info = litex_hwmon_channel_info
};

/* Driver functions */

static int litex_hwmon_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct litex_hwmon *hwmon_s;
	struct resource *res;

	if (!node)
		return -ENODEV;

	hwmon_s = devm_kzalloc(&pdev->dev, sizeof(*hwmon_s), GFP_KERNEL);
	if (!hwmon_s)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EBUSY;

	hwmon_s->membase = devm_of_iomap(&pdev->dev, node, 0, &res->end);
	if (IS_ERR_OR_NULL(hwmon_s->membase))
		return -EIO;

	hwmon_s->hdev = devm_hwmon_device_register_with_info(&pdev->dev,
							     "litex_xadc",
							     hwmon_s,
							     &litex_chip_info,
							     NULL);
	platform_set_drvdata(pdev, hwmon_s);
	return PTR_ERR_OR_ZERO(hwmon_s->hdev);
}

static const struct of_device_id litex_of_match[] = {
	{.compatible = "litex,hwmon-xadc"},
	{},
};

MODULE_DEVICE_TABLE(of, litex_of_match);

static struct platform_driver litex_hwmon_driver = {
	.driver		= {
		.name		= "litex-hwmon",
		.of_match_table = of_match_ptr(litex_of_match)
	},
	.probe		= litex_hwmon_probe,
};

module_platform_driver(litex_hwmon_driver);
