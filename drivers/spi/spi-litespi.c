// SPDX-License-Identifier: GPL-2.0
/*
 * LiteSPI controller (LiteX) Driver
 *
 * Copyright (C) 2019 Antmicro Ltd. <www.antmicro.com>
 */

#include <linux/device.h>
#include <linux/litex.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#define DRIVER_NAME "litespi"

/* register sizes */
#define LITESPI_SZ_CTRL		2
#define LITESPI_SZ_STAT		1
#define LITESPI_SZ_MOSI		LITEX_SUBREG_SIZE
#define LITESPI_SZ_MISO		LITEX_SUBREG_SIZE
#define LITESPI_SZ_CS		1

/* register offsets */
#define LITESPI_OFF_CTRL	0x00
#define LITESPI_OFF_STAT	\
	_next_reg_off(LITESPI_OFF_CTRL, LITESPI_SZ_CTRL)
#define LITESPI_OFF_MOSI	\
	_next_reg_off(LITESPI_OFF_STAT, LITESPI_SZ_STAT)
#define LITESPI_OFF_MISO	\
	_next_reg_off(LITESPI_OFF_MOSI, LITESPI_SZ_MOSI)
#define LITESPI_OFF_CS		\
	_next_reg_off(LITESPI_OFF_MISO, LITESPI_SZ_MISO)

#define LITESPI_CTRL_SHIFT_BPW	8
#define LITESPI_CTRL_START_BIT	0

struct litespi_hw {
	struct spi_master *master;
	void __iomem *base;
};

static inline void litespi_wait_xfer_end(struct litespi_hw *hw)
{
	while (!litex_read8(hw->base + LITESPI_OFF_STAT))
		cpu_relax();
}

static void litespi_cs(struct spi_device *spi, bool enable)
{
	struct litespi_hw *hw = spi_master_get_devdata(spi->master);
	u8 cs = litex_read8(hw->base + LITESPI_OFF_CS);
	if (!enable) {// inversed logic
		cs |= BIT(spi->chip_select);
	}
	else {
		cs &= ~BIT(spi->chip_select);
	}

	litex_write8(hw->base + LITESPI_OFF_CS, cs);
}

static int litespi_rxtx(struct spi_master *master, struct spi_device *spi,
			struct spi_transfer *t)
{
	struct litespi_hw *hw = spi_master_get_devdata(master);
	u16 ctl_word = t->bits_per_word << LITESPI_CTRL_SHIFT_BPW;
	int i;

	/* set word size */
	litex_write16(hw->base + LITESPI_OFF_CTRL, ctl_word);

	/* add start bit to ctl_word */
	ctl_word |= BIT(LITESPI_CTRL_START_BIT);

	/*
	 * Validated SPI transfer length is multiple of SPI word size, which
	 * is itself a power-of-two multiple, and fits within LITEX_SUBREG_SIZE
	 */
	if (t->bits_per_word <= 8) {
		const u8 *tx = t->tx_buf;
		u8 *rx = t->rx_buf;

		/* word size is 1 byte */
		for (i = 0; i < t->len; i++) {
			if (tx)
				litex_write8(hw->base +
						 LITESPI_OFF_MOSI, *tx++);

			litex_write16(hw->base + LITESPI_OFF_CTRL, ctl_word);
			litespi_wait_xfer_end(hw);

			if (rx)
				*rx++ = litex_read8(hw->base +
							LITESPI_OFF_MISO);
		}
	} else if (t->bits_per_word <= 16) {
		const u16 *tx = t->tx_buf;
		u16 *rx = t->rx_buf;

		/* word size is 2 bytes */
		for (i = 0; i < t->len / 2; i++) {
			if (tx)
				litex_write16(hw->base + LITESPI_OFF_MOSI,
						 be16_to_cpu(*tx++));

			litex_write16(hw->base + LITESPI_OFF_CTRL, ctl_word);
			litespi_wait_xfer_end(hw);

			if (rx)
				*rx++ = cpu_to_be16(litex_read16(
						hw->base + LITESPI_OFF_MISO));
		}
	} else {
		const u32 *tx = t->tx_buf;
		u32 *rx = t->rx_buf;

		/* word size is 4 bytes */
		for (i = 0; i < t->len / 4; i++) {
			if (tx)
				litex_write32(hw->base + LITESPI_OFF_MOSI,
						 be32_to_cpu(*tx++));

			litex_write16(hw->base + LITESPI_OFF_CTRL, ctl_word);
			litespi_wait_xfer_end(hw);

			if (rx)
				*rx++ = cpu_to_be32(litex_read32(
						hw->base + LITESPI_OFF_MISO));
		}
	}

	return 0;
}

static int litespi_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct litespi_hw *hw;
	struct spi_master *master;
	struct resource *res;
	int ret;
	u32 val;

	master = spi_alloc_master(&pdev->dev, sizeof(*hw));
	if (!master)
		return -ENOMEM;

	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = pdev->id;
	master->set_cs = litespi_cs;
	master->transfer_one = litespi_rxtx;
	master->mode_bits = SPI_MODE_0 | SPI_CS_HIGH;
	master->flags = SPI_CONTROLLER_MUST_RX | SPI_CONTROLLER_MUST_TX;

	/* get bits per word property */
	ret = of_property_read_u32(node, "litespi,max-bpw", &val);
	if (ret)
		goto err;
	if (val > LITEX_SUBREG_SIZE * 8) {
		ret = -EINVAL;
		goto err;
	}
	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, val);

	/* get sck frequency */
	ret = of_property_read_u32(node, "litespi,sck-frequency", &val);
	if (ret)
		goto err;
	master->max_speed_hz = val;

	/* get num cs */
	ret = of_property_read_u32(node, "litespi,num-cs", &val);
	if (ret)
		goto err;
	master->num_chipselect = val;

	hw = spi_master_get_devdata(master);
	hw->master = master;

	/* get base address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hw->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hw->base)) {
		ret = PTR_ERR(hw->base);
		goto err;
	}

	/* register controller */
	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret)
		goto err;

	return 0;

err:
	spi_master_put(master);
	return ret;
}

static const struct of_device_id litespi_match[] = {
	{ .compatible = "litex,litespi" },
	{}
};
MODULE_DEVICE_TABLE(of, litespi_match);

static struct platform_driver litespi_driver = {
	.probe = litespi_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(litespi_match)
	}
};
module_platform_driver(litespi_driver)

MODULE_AUTHOR("Antmicro Ltd <www.antmicro.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
