// SPDX-License-Identifier: GPL-2.0
/*
 * LiteSPI controller (LiteX) Driver
 *
 * Copyright (C) 2019 Antmicro Ltd. <www.antmicro.com>
 */

#include <linux/device.h>
#include <linux/litex.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
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
	struct mutex bus_mutex;
};

static inline void litespi_wait_xfer_end(struct litespi_hw *hw)
{
	while (!litex_reg_readb(hw->base + LITESPI_OFF_STAT))
		cpu_relax();
}

static void litespi_rxtx(struct litespi_hw *hw, struct spi_transfer *t)
{
	int i;
	u16 val;
	const u8 *tx = t->tx_buf;
	u8 *rx = t->rx_buf;

	for (i = 0; i < t->len; i++) {
		if (tx) {
			litex_reg_writeb(hw->base + LITESPI_OFF_MOSI, *tx++);
		}

		val = litex_reg_readw(hw->base + LITESPI_OFF_CTRL);
		litex_reg_writew(hw->base + LITESPI_OFF_CTRL,
				 val | BIT(LITESPI_CTRL_START_BIT));
		litespi_wait_xfer_end(hw);

		if (rx) {
			*rx++ = litex_reg_readb(hw->base + LITESPI_OFF_MISO);
		}
	}
}

static int litespi_xfer_one(struct spi_master *master, struct spi_message *m)
{
	struct litespi_hw *hw = spi_master_get_devdata(master);
	struct spi_transfer *t;

	mutex_lock(&hw->bus_mutex);

	/* setup chip select */
	litex_reg_writeb(hw->base + LITESPI_OFF_CS,
			 BIT(m->spi->chip_select));

	list_for_each_entry(t, &m->transfers, transfer_list) {
		litespi_rxtx(hw, t);
		m->actual_length += t->len;
	}

	m->status = 0;
	spi_finalize_current_message(master);

	mutex_unlock(&hw->bus_mutex);

	return 0;
}

static int litespi_setup(struct spi_device *spi)
{
	struct litespi_hw *hw = spi_master_get_devdata(spi->master);

	/* lock and wait for transfer finish */
	mutex_lock(&hw->bus_mutex);
	litespi_wait_xfer_end(hw);

	/* set word size and clear CS bits */
	litex_reg_writew(hw->base + LITESPI_OFF_CTRL,
			 spi->bits_per_word << LITESPI_CTRL_SHIFT_BPW);

	mutex_unlock(&hw->bus_mutex);

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

	if (!litex_check_accessors())
		return -EPROBE_DEFER;

	master = spi_alloc_master(&pdev->dev, sizeof(*hw));
	if (!master)
		return -ENOMEM;

	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = pdev->id;
	master->setup = litespi_setup;
	master->transfer_one_message = litespi_xfer_one;
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
	mutex_init(&hw->bus_mutex);

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
