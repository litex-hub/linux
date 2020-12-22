/*
 * LiteX Liteeth Ethernet
 *
 * Copyright 2017 Joel Stanley <joel@jms.id.au>
 *
 */

#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/iopoll.h>

#include "litex_liteeth.h"

#define DRV_NAME	"liteeth"
#define DRV_VERSION	"0.1"

#define LITEETH_BUFFER_SIZE		0x800
#define MAX_PKT_SIZE			LITEETH_BUFFER_SIZE

struct liteeth {
	void __iomem *base;
	void __iomem *mdio_base;
	struct net_device *netdev;
	int use_polling;
	struct timer_list poll_timer;
	struct device *dev;
	struct mii_bus *mii_bus;

	/* Link management */
	int cur_duplex;
	int cur_speed;

	/* Tx */
	int tx_slot;
	int num_tx_slots;
	void __iomem *tx_base;

	/* Rx */
	int rx_slot;
	int num_rx_slots;
	void __iomem *rx_base;
};


static int liteeth_rx(struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);
	struct sk_buff *skb;
	unsigned char *data;
	u8 rx_slot;
	int len;

	rx_slot = litex_read8(priv->base + LITEETH_WRITER_SLOT_OFF);
	len = litex_read32(priv->base + LITEETH_WRITER_LENGTH_OFF);

	skb = netdev_alloc_skb(netdev, len + NET_IP_ALIGN);
	if (!skb) {
		netdev_err(netdev, "couldn't get memory");
		netdev->stats.rx_dropped++;
		return NET_RX_DROP;
	}

	/* Ensure alignemnt of the ip header within the skb */
	skb_reserve(skb, NET_IP_ALIGN);
	if (len == 0 || len > 2048)
		return NET_RX_DROP;
	data = skb_put(skb, len);
	memcpy_fromio(data, priv->rx_base + rx_slot * LITEETH_BUFFER_SIZE, len);
	skb->protocol = eth_type_trans(skb, netdev);

	netdev->stats.rx_packets++;
	netdev->stats.rx_bytes += len;

	return netif_rx(skb);
}

static irqreturn_t liteeth_interrupt(int irq, void *dev_id)
{
	struct net_device *netdev = dev_id;
	struct liteeth *priv = netdev_priv(netdev);
	u8 reg;

	reg = litex_read8(priv->base + LITEETH_READER_EV_PENDING_OFF);
	if (reg) {
		netdev->stats.tx_packets++;
		litex_write8(priv->base + LITEETH_READER_EV_PENDING_OFF, reg);
	}

	reg = litex_read8(priv->base + LITEETH_WRITER_EV_PENDING_OFF);
	if (reg) {
		liteeth_rx(netdev);
		litex_write8(priv->base + LITEETH_WRITER_EV_PENDING_OFF, reg);
	}

	return IRQ_HANDLED;
}

static void liteeh_timeout(struct timer_list *t)
{
	struct liteeth *priv = from_timer(priv, t, poll_timer);

	liteeth_interrupt(0, priv->netdev);
	mod_timer(&priv->poll_timer, jiffies + msecs_to_jiffies(10));
}

static int liteeth_open(struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);
	int err;

	/* TODO: Remove these once we have working mdio support */
	priv->cur_duplex = DUPLEX_FULL;
	priv->cur_speed = SPEED_100;
	netif_carrier_on(netdev);

	if (!priv->use_polling) {
		err = request_irq(netdev->irq, liteeth_interrupt, 0, netdev->name, netdev);
		if (err) {
			netdev_err(netdev, "failed to request irq %d\n", netdev->irq);
			goto err_irq;
		}
	}

	/* Clear pending events? */
	litex_write8(priv->base + LITEETH_WRITER_EV_PENDING_OFF, 1);
	litex_write8(priv->base + LITEETH_READER_EV_PENDING_OFF, 1);

	if (!priv->use_polling) {
		/* Enable IRQs? */
		litex_write8(priv->base + LITEETH_WRITER_EV_ENABLE_OFF, 1);
		litex_write8(priv->base + LITEETH_READER_EV_ENABLE_OFF, 1);
	}

	netif_start_queue(netdev);

	if (priv->use_polling) {
		timer_setup(&priv->poll_timer, liteeh_timeout, 0);
		mod_timer(&priv->poll_timer, jiffies + msecs_to_jiffies(50));
	}

	return 0;

err_irq:
	netif_carrier_off(netdev);
	return err;
}

static int liteeth_stop(struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);

	netif_stop_queue(netdev);

	del_timer_sync(&priv->poll_timer);

	litex_write8(priv->base + LITEETH_WRITER_EV_ENABLE_OFF, 0);
	litex_write8(priv->base + LITEETH_READER_EV_ENABLE_OFF, 0);

	if (!priv->use_polling) {
		free_irq(netdev->irq, netdev);
	}

	return 0;
}

static int liteeth_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct liteeth *priv = netdev_priv(netdev);
	void *txbuffer;
	int ret;
	u8 val;

	/* Reject oversize packets */
	if (unlikely(skb->len > MAX_PKT_SIZE)) {
		if (net_ratelimit())
			netdev_dbg(netdev, "tx packet too big\n");
		goto drop;
	}

	txbuffer = priv->tx_base + priv->tx_slot * LITEETH_BUFFER_SIZE;
	memcpy_fromio(txbuffer, skb->data, skb->len);
	litex_write8(priv->base + LITEETH_READER_SLOT_OFF, priv->tx_slot);
	litex_write16(priv->base + LITEETH_READER_LENGTH_OFF, skb->len);

	ret = readx_poll_timeout_atomic(litex_read8,
			priv->base + LITEETH_READER_READY_OFF,
			val, val, 5, 1000);
	if (ret == -ETIMEDOUT) {
		netdev_err(netdev, "LITEETH_READER_READY timed out\n");
		goto drop;
	}

	litex_write8(priv->base + LITEETH_READER_START_OFF, 1);

	priv->tx_slot = (priv->tx_slot + 1) % priv->num_tx_slots;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
drop:
	/* Drop the packet */
	dev_kfree_skb_any(skb);
	netdev->stats.tx_dropped++;

	return NETDEV_TX_OK;
}

static void liteeth_get_drvinfo(struct net_device *netdev,
				struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	strlcpy(info->bus_info, dev_name(&netdev->dev), sizeof(info->bus_info));
}

static const struct net_device_ops liteeth_netdev_ops = {
	.ndo_open		= liteeth_open,
	.ndo_stop		= liteeth_stop,
	.ndo_start_xmit         = liteeth_start_xmit,
};

static const struct ethtool_ops liteeth_ethtool_ops = {
	.get_drvinfo		= liteeth_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.set_link_ksettings	= phy_ethtool_set_link_ksettings,
	.nway_reset		= phy_ethtool_nway_reset,
};

static void liteeth_reset_hw(struct liteeth *priv)
{
	/* Reset, twice */
	litex_write8(priv->base + LITEETH_PHY_CRG_RESET_OFF, 0);
	udelay(10);
	litex_write8(priv->base + LITEETH_PHY_CRG_RESET_OFF, 1);
	udelay(10);
	litex_write8(priv->base + LITEETH_PHY_CRG_RESET_OFF, 0);
	udelay(10);
}

static int liteeth_probe(struct platform_device *pdev)
{
	struct net_device *netdev;
	void __iomem *buf_base;
	struct resource *res;
	struct liteeth *priv;
	int irq, err;

	netdev = alloc_etherdev(sizeof(*priv));
	if (!netdev)
		return -ENOMEM;

	priv = netdev_priv(netdev);
	priv->netdev = netdev;
	priv->dev = &pdev->dev;

	priv->use_polling = 0;
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get IRQ, using polling\n");
		priv->use_polling = 1;
		irq = 0;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base)) {
		err = PTR_ERR(priv->base);
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	priv->mdio_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->mdio_base)) {
		err = PTR_ERR(priv->mdio_base);
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	buf_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(buf_base)) {
		err = PTR_ERR(buf_base);
		goto err;
	}

	err = of_property_read_u32(pdev->dev.of_node, "rx-fifo-depth",
			&priv->num_rx_slots);
	if (err) {
		dev_err(&pdev->dev, "unable to get rx-fifo-depth\n");
		goto err;
	}

	err = of_property_read_u32(pdev->dev.of_node, "tx-fifo-depth",
			&priv->num_tx_slots);
	if (err) {
		dev_err(&pdev->dev, "unable to get tx-fifo-depth\n");
		goto err;
	}

	/* Rx slots */
	priv->rx_base = buf_base;
	priv->rx_slot = 0;

	/* Tx slots come after Rx slots */
	priv->tx_base = buf_base + priv->num_rx_slots * LITEETH_BUFFER_SIZE;
	priv->tx_slot = 0;

	err = of_get_mac_address(pdev->dev.of_node, netdev->dev_addr);
	if (err)
		eth_hw_addr_random(netdev);

	SET_NETDEV_DEV(netdev, &pdev->dev);
	platform_set_drvdata(pdev, netdev);

	netdev->netdev_ops = &liteeth_netdev_ops;
	netdev->ethtool_ops = &liteeth_ethtool_ops;
	netdev->irq = irq;

	liteeth_reset_hw(priv);

	err = register_netdev(netdev);
	if (err) {
		dev_err(&pdev->dev, "Failed to register netdev\n");
		goto err;
	}

	netdev_info(netdev, "irq %d, mapped at %px\n", netdev->irq, priv->base);

	return 0;
err:
	free_netdev(netdev);
	return err;
}

static int liteeth_remove(struct platform_device *pdev)
{
	struct net_device *netdev;

	netdev = platform_get_drvdata(pdev);
	unregister_netdev(netdev);
	free_netdev(netdev);
	return 0;
}

static const struct of_device_id liteeth_of_match[] = {
	{ .compatible = "litex,liteeth" },
	{ }
};
MODULE_DEVICE_TABLE(of, liteeth_of_match);

static struct platform_driver liteeth_driver = {
	.probe = liteeth_probe,
	.remove = liteeth_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = liteeth_of_match,
	},
};
module_platform_driver(liteeth_driver);

MODULE_AUTHOR("Joel Stanley <joel@jms.id.au>");
MODULE_LICENSE("GPL");
