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
#include <linux/gpio/driver.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/litex.h>

#define GPIO_PINS_MAX   32

struct litex_gpio {
	void __iomem *membase;
	int port_direction;
	int reg_span;
	struct gpio_chip chip;
};

/* API functions */

static int litex_gpio_get_value(struct gpio_chip *chip, unsigned int offset)
{
	struct litex_gpio *gpio_s = gpiochip_get_data(chip);
	u32 regv;

	if (offset >= chip->ngpio)
		return -EINVAL;

	regv = litex_get_reg(gpio_s->membase, gpio_s->reg_span);
	return !!(regv & BIT(offset));
}

static int litex_gpio_get_multiple(struct gpio_chip *chip, unsigned long *mask,
				   unsigned long *bits)
{
	struct litex_gpio *gpio_s = gpiochip_get_data(chip);
	u32 regv;

	if (*mask >= (1 << chip->ngpio))
		return -EINVAL;

	regv = litex_get_reg(gpio_s->membase, gpio_s->reg_span);
	*bits = (regv & *mask);
	return 0;
}

static void litex_gpio_set_value(struct gpio_chip *chip, unsigned int offset,
				 int val)
{
	struct litex_gpio *gpio_s = gpiochip_get_data(chip);
	u32 regv, new_regv;

	if (offset >= chip->ngpio)
		return;

	regv = litex_get_reg(gpio_s->membase, gpio_s->reg_span);
	new_regv = (regv & ~BIT(offset)) | (!!val << offset);
	litex_set_reg(gpio_s->membase, gpio_s->reg_span, new_regv);
}

static void litex_gpio_set_multiple(struct gpio_chip *chip, unsigned long *mask,
				    unsigned long *bits)
{
	struct litex_gpio *gpio_s = gpiochip_get_data(chip);
	u32 regv, new_regv;

	if (*mask >= (1 << chip->ngpio))
		return;

	regv = litex_get_reg(gpio_s->membase, gpio_s->reg_span);
	new_regv = (regv & ~(*mask)) | (*bits);
	litex_set_reg(gpio_s->membase, gpio_s->reg_span, new_regv);
}

static int litex_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct litex_gpio *gpio_s = gpiochip_get_data(chip);

	return gpio_s->port_direction;
}

static int litex_gpio_direction_input(struct gpio_chip *chip,
				      unsigned int offset)
{
	struct litex_gpio *gpio_s = gpiochip_get_data(chip);

	if (gpio_s->port_direction != GPIOF_DIR_IN)
		return -ENOTSUPP;
	else
		return 0;
}

static int litex_gpio_direction_output(struct gpio_chip *chip,
				       unsigned int offset, int value)
{
	struct litex_gpio *gpio_s = gpiochip_get_data(chip);

	if (gpio_s->port_direction != GPIOF_DIR_OUT)
		return -ENOTSUPP;
	else
		return 0;
}

/* Driver functions */

static int litex_gpio_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct litex_gpio *gpio_s;
	struct resource *res;
	int ret_i;

	int dt_ngpio;
	const char *dt_direction;

	if (!node)
		return -ENODEV;

	gpio_s = devm_kzalloc(&pdev->dev, sizeof(*gpio_s), GFP_KERNEL);
	if (!gpio_s)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EBUSY;

	gpio_s->membase = devm_of_iomap(&pdev->dev, node, 0, &res->end);
	if (IS_ERR_OR_NULL(gpio_s->membase))
		return -EIO;

	ret_i = of_property_read_u32(node, "litex,ngpio", &dt_ngpio);
	if (ret_i < 0) {
		dev_err(&pdev->dev, "No litex,ngpio entry in the dts file\n");
		return -ENODEV;
	}
	if (dt_ngpio >= GPIO_PINS_MAX) {
		dev_err(&pdev->dev,
			"LiteX GPIO driver cannot use more than %d pins\n",
			GPIO_PINS_MAX);
		return -EINVAL;
	}

	ret_i = of_property_read_string(node, "litex,direction",
					      &dt_direction);
	if (ret_i < 0) {
		dev_err(&pdev->dev, "No litex,direction entry in the dts file\n");
		return -ENODEV;
	}

	if (!strcmp(dt_direction, "in"))
		gpio_s->port_direction = GPIOF_DIR_IN;
	else if (!strcmp(dt_direction, "out"))
		gpio_s->port_direction = GPIOF_DIR_OUT;
	else
		return -ENODEV;

	/* Assign API functions */

	gpio_s->chip.label             = "litex_gpio";
	gpio_s->chip.owner             = THIS_MODULE;
	gpio_s->chip.get               = litex_gpio_get_value;
	gpio_s->chip.get_multiple      = litex_gpio_get_multiple;
	gpio_s->chip.set               = litex_gpio_set_value;
	gpio_s->chip.set_multiple      = litex_gpio_set_multiple;
	gpio_s->chip.get_direction     = litex_gpio_get_direction;
	gpio_s->chip.direction_input   = litex_gpio_direction_input;
	gpio_s->chip.direction_output  = litex_gpio_direction_output;
	gpio_s->chip.parent            = &pdev->dev;
	gpio_s->chip.base              = -1;
	gpio_s->chip.ngpio             = dt_ngpio;
	gpio_s->chip.can_sleep         = false;

	gpio_s->reg_span = (dt_ngpio + LITEX_SUBREG_SIZE_BIT - 1) /
			   LITEX_SUBREG_SIZE_BIT;

	platform_set_drvdata(pdev, gpio_s);
	return devm_gpiochip_add_data(&pdev->dev, &gpio_s->chip, gpio_s);
}

static const struct of_device_id litex_of_match[] = {
	{.compatible = "litex,gpio"},
	{},
};

MODULE_DEVICE_TABLE(of, litex_of_match);

static struct platform_driver litex_gpio_driver = {
	.driver = {
		.name             = "litex-gpio",
		.of_match_table   = of_match_ptr(litex_of_match)
	},
	.probe  = litex_gpio_probe,
};

module_platform_driver(litex_gpio_driver);

MODULE_DESCRIPTION("LiteX gpio driver");
MODULE_AUTHOR("Antmicro <www.antmicro.com>");
MODULE_LICENSE("GPL v2");
