 /*
 * Copyright (c) 2020 Antmicro <www.antmicro.com>
 */

#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/module.h>

#include <linux/litex.h>
#include <linux/io.h>

#include <linux/types.h>
#include <linux/signal.h>

#define SPIFLASH4X0_LABEL "spiflash"

#define WIP 0x1
#define WEN_SET 0x2

#define SPI_CSR_BASE spi->csr_base

#define SPI_DATA_IN ((SPI_CSR_BASE) + 0x18)
#define SPI_DATA_OUT ((SPI_CSR_BASE) + 0x38)
#define SPI_IN_LEN ((SPI_CSR_BASE) + 0x10)
#define SPI_OUT_LEN ((SPI_CSR_BASE) + 0x14)
#define COMMAND_QUEUED ((SPI_CSR_BASE) + 0x0c)

#define READ_STATUS_REGISTER 0x05
#define WRITE_ENABLE 0x06
#define READ_EVC_REG 0x65
#define WRITE_EVC_REG 0x61
#define READ_ID 0x9F
#define SUBSECTOR_ERASE 0x20

struct litex_qspi_flash {
	struct spi_nor nor;
	struct device *dev;
	void __iomem *mem_base;
	void __iomem *csr_base;
	int mem_size;
	int spi_timeout;
	int flash_timeout;
};

static int wait_while_spi_busy(struct spi_nor *nor)
{
	struct litex_qspi_flash *spi = nor->priv;
	long timeout = spi->spi_timeout;

	while (litex_get_reg((u8 *)COMMAND_QUEUED, 1)) {
		if (timeout < 0)
			return -ETIMEDOUT;

		timeout--;
	}
	return 0;
}

static int spi_tranceive(struct spi_nor *nor,
			 u8 *write_buf, int write_len,
			 u8 *read_buf, int read_len)
{
	int i;
	struct litex_qspi_flash *spi = nor->priv;

	if (write_len > 8 || write_len < 1 || read_len > 8 || read_len < 0)
		return -EPERM;
	if (wait_while_spi_busy(nor))
		return -EBUSY;

	for (i = 0; i < write_len; i++)
		litex_set_reg(SPI_DATA_IN + i*4, 1, write_buf[i]);

	litex_set_reg(SPI_OUT_LEN, 1, read_len);
	litex_set_reg(SPI_IN_LEN, 1, write_len);

	if (wait_while_spi_busy(nor))
		return -EBUSY;

	for (i = 0; i < read_len; i++)
		read_buf[i] = litex_get_reg(SPI_DATA_OUT + i*4, 1);

	return 0;
}

static int read_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	return spi_tranceive(nor, &opcode, 1, buf, len);
}

static int write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	u8 spi_buf[8] = {opcode};
	int i;

	if (len > 7 || len < 0)
		return -EPERM;

	for (i = 0; i < len; i++)
		spi_buf[i + 1] = buf[i];

	return spi_tranceive(nor, spi_buf, len, NULL, 0);
}

static int wait_while_flash_busy(struct spi_nor *nor)
{
	struct litex_qspi_flash *spi = nor->priv;
	u8 command = READ_STATUS_REGISTER;
	u8 receive = 0x00;
	unsigned long timeout = spi->flash_timeout;
	int ret;

	do {
		if (!timeout)
			return -EBUSY;

		timeout--;
		ret = spi_tranceive(nor, &command, 1, &receive, 1);
		if (ret)
			return ret;

	} while (receive & WIP);

	return 0;
}

static int write_enable(struct spi_nor *nor)
{
	u8 spi_buf;
	u8 receive;
	int ret;

	ret = wait_while_flash_busy(nor);
	if (ret)
		return ret;

	spi_buf = WRITE_ENABLE;
	ret = spi_tranceive(nor, &spi_buf, 1, NULL, 0);

	if (ret)
		return ret;

	spi_buf = READ_STATUS_REGISTER;

	ret = spi_tranceive(nor, &spi_buf, 1, &receive, 1);
	if (ret)
		return ret;

	if (!(receive & WEN_SET))
		return -EAGAIN;

	return 0;
}

static ssize_t flash_write(struct spi_nor *nor, loff_t addr,
			size_t data_size,
			const u_char *data)
{
	u32 buf;
	u8 flash_offset;
	u8 miss;
	u8 min;
	int data_left = data_size;
	u8 *buf8 = (u8 *)&buf;
	u8 *data8 = (u8 *)data;
	struct litex_qspi_flash *spi = nor->priv;
	int ret;

	if ((addr + data_size) > spi->mem_size || addr < 0)
		return -EFAULT;

	while (data_left > 0) {
		buf = 0xFFFFFFFF;
		flash_offset = addr & 0x3;
		miss = 4 - flash_offset;

		ret = wait_while_spi_busy(nor);
		if (ret)
			return ret;

		if (write_enable(nor))
			break;

		min = (data_left < miss ? data_left : miss);
		memcpy(buf8 + flash_offset, data8, min);

		iowrite32(buf, spi->mem_base + addr - flash_offset);
		ret = wait_while_flash_busy(nor);
		if (ret)
			return ret;

		data8 += min;
		addr += min;
		data_left -= min;
	}

	return data_size - data_left;
}

static ssize_t flash_read(struct spi_nor *nor, loff_t addr,
				 size_t data_size, u_char *data)
{
	u32 buf;
	u32 data_left = data_size;
	u8 flash_offset;
	u8 miss;
	u8 min;
	u8 *buf8 = (u8 *)&buf;
	u8 *data8 = (u8 *)data;
	struct litex_qspi_flash *spi = nor->priv;

	if ((addr + data_size) > spi->mem_size || addr < 0)
		return -EFAULT;

	while (data_left > 0) {
		flash_offset = addr & 0x3;
		miss = 4 - flash_offset;
		min = (data_left < miss) ? data_left : miss;

		if (wait_while_flash_busy(nor))
			break;

		buf = ioread32(spi->mem_base + addr - flash_offset);

		memcpy(data8, (buf8 + flash_offset), min);
		addr += min;
		data8 += min;
		data_left -= min;
	}

	return data_size - data_left;
}

static ssize_t flash_erase(struct spi_nor *nor, loff_t addr)
{
	struct litex_qspi_flash *spi = nor->priv;
	u8 send[4] = {SUBSECTOR_ERASE, (addr) >> 16, (addr) >> 8, (addr) >> 0};
	int ret;

	if (addr >= spi->mem_size || addr < 0)
		return -EFAULT;

	if (wait_while_flash_busy(nor))
		return -EBUSY;

	if (write_enable(nor))
		return -EIO;

	if (wait_while_flash_busy(nor))
		return -EBUSY;

	ret = spi_tranceive(nor, send, 4, NULL, 0);
	if (ret)
		return ret;

	if (wait_while_spi_busy(nor))
		return -EBUSY;

	return 0;
}

int spi_flash_litex_init(struct platform_device *pdev)
{
	struct litex_qspi_flash *spi;
	struct spi_nor *nor;
	struct resource *res;
	struct device_node *flash_node;
	resource_size_t size = 0;
	int ret;
	int i;
	const struct spi_nor_hwcaps hwcaps = {
		.mask = SNOR_HWCAPS_READ |
			SNOR_HWCAPS_READ_FAST |
			SNOR_HWCAPS_PP,
	};

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No DT found\n");
		return -EINVAL;
	}

	spi = devm_kzalloc(&pdev->dev, sizeof(*spi), GFP_KERNEL);
	flash_node = of_get_next_available_child(pdev->dev.of_node, NULL);
	if (IS_ERR_OR_NULL(flash_node)) {
		dev_err(&pdev->dev, "no SPI flash device to configure\n");
		return -ENODEV;
	}

	spi->mem_size = be32_to_cpup(of_get_property(flash_node, "memory_size", NULL));
	spi->spi_timeout = be32_to_cpup(of_get_property(flash_node, "spi_timeout", NULL));
	spi->flash_timeout = be32_to_cpup(of_get_property(flash_node, "flash_timeout", NULL));

	if (IS_ERR_OR_NULL(spi->mem_size)) {
		dev_err(&pdev->dev, "Cannot determine flash memory_size\n");
		return -ENODEV;
	}
	if (IS_ERR_OR_NULL(spi->spi_timeout)) {
		dev_err(&pdev->dev, "Cannot determine spi timeout\n");
		return -ENODEV;
	}
	if (IS_ERR_OR_NULL(spi->flash_timeout)) {
		dev_err(&pdev->dev, "Cannot determine flash timeout\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, spi);
	spi->dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	spi->mem_base = devm_of_iomap(&pdev->dev, flash_node, 0, &size);

	if (IS_ERR_OR_NULL(spi->mem_base)) {
		dev_err(&pdev->dev, "Cannot map flash to memory\n");
		return -ENODEV;
	}

	spi->csr_base = devm_of_iomap(&pdev->dev, flash_node, 1, &size);

	if (IS_ERR_OR_NULL(spi->csr_base)) {
		dev_err(&pdev->dev, "Cannot map CSR to memory\n");
		return -ENODEV;
	}

	nor = &spi->nor;
	nor->dev = spi->dev;
	nor->priv = spi;
	spi_nor_set_flash_node(nor, flash_node);
	nor->read_reg = read_reg;
	nor->write_reg = write_reg;
	nor->read = flash_read;
	nor->write = flash_write;
	nor->erase = flash_erase;
	nor->read_proto = SNOR_HWCAPS_READ_4_4_4;
	nor->write_proto = SNOR_HWCAPS_PP_4_4_4;
	nor->mtd.name = "spiflash";
	nor->addr_width = 3;
	nor->erase_opcode = SPINOR_OP_BE_4K;

	ret = spi_nor_scan(nor, NULL, &hwcaps);

	if (ret) {
		dev_err(&pdev->dev, "Fail to verify flash memory\n");
		return ret;
	}

	ret = mtd_device_register(&nor->mtd, NULL, 0);

	if (ret) {
		dev_err(&pdev->dev, "Fail to register device\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id litex_of_match[] = {
	{ .compatible = "litex,spiflash,quad_read/write" }
};

MODULE_DEVICE_TABLE(of, litex_of_match);

static struct platform_driver litex_spi_flash_driver = {
	.probe	= spi_flash_litex_init,
	.driver = {
		.name = "litex-spiflash_quad_read_write",
		.of_match_table = of_match_ptr(litex_of_match)
	},
};
module_platform_driver(litex_spi_flash_driver);

MODULE_DESCRIPTION("LiteX SPI Flash driver");
MODULE_AUTHOR("Antmicro <www.antmicro.com>");
MODULE_LICENSE("GPL v2");
