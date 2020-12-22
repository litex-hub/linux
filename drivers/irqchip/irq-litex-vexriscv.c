// SPDX-License-Identifier: GPL-2.0
/*
 * VexRiscv interrupt controller driver
 *
 * Copyright (C) 2019 Antmicro Ltd. <www.antmicro.com>
 */

#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdesc.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#define IRQ_MASK	CONFIG_LITEX_VEXRISCV_CSR_MASK
#define IRQ_PENDING	CONFIG_LITEX_VEXRISCV_CSR_PENDING
#define IRQ_VEC_SZ	32

static struct irq_domain *root_domain;

struct litex_vexriscv_intc {
	struct irq_chip chip;
	irq_flow_handler_t handle;
	unsigned long flags;
};

static unsigned int litex_vexriscv_irq_getie(void)
{
	return (csr_read(sstatus) & IE_SIE) != 0;
}

static void litex_vexriscv_irq_setie(unsigned int ie)
{
	if (ie)
		csr_write(sstatus, IE_SIE);
	else
		csr_clear(sstatus, IE_SIE);
}

static unsigned int litex_vexriscv_irq_getmask(void)
{
	u32 mask;

	__asm__ __volatile__ ("csrr %0, %1" : "=r"(mask) : "i"(IRQ_MASK));
	return mask;
}

static void litex_vexriscv_irq_setmask(unsigned int mask)
{
	__asm__ volatile ("csrw %0, %1" :: "i"(IRQ_MASK), "r"(mask));
}

static unsigned int litex_vexriscv_irq_pending(void)
{
	u32 pending;

	__asm__ __volatile__ ("csrr %0, %1" : "=r"(pending) : "i"(IRQ_PENDING));
	return pending;
}

static int litex_vexriscv_intc_map(struct irq_domain *d, unsigned int irq,
				   irq_hw_number_t hw)
{
	struct litex_vexriscv_intc *intc = d->host_data;

	pr_debug("irqchip: LiteX Vexriscv irqmap: %d\n", irq);
	irq_set_chip_and_handler(irq, &intc->chip, intc->handle);
	irq_set_status_flags(irq, intc->flags);

	pr_debug("irqchip: mapcount: %d\n", d->mapcount);

	return 0;
}

static void litex_vexriscv_intc_mask(struct irq_data *data)
{
	unsigned int mask = litex_vexriscv_irq_getmask();
	unsigned int irqbit = BIT(data->hwirq);

	pr_debug("irqchip: LiteX VexRiscv irqchip mask: 0x%08x -> 0x%08x",
		mask, mask & ~irqbit);
	mask &= ~irqbit;
	litex_vexriscv_irq_setmask(mask);

	if (!mask)
		litex_vexriscv_irq_setie(0);
}

static void litex_vexriscv_intc_unmask(struct irq_data *data)
{
	unsigned int mask = litex_vexriscv_irq_getmask();
	unsigned int irqbit = BIT(data->hwirq);

	pr_debug("irqchip: LiteX VexRiscv irqchip mask: 0x%08x -> 0x%08x",
		mask, mask | irqbit);
	mask |= irqbit;
	litex_vexriscv_irq_setmask(mask);

	if (!litex_vexriscv_irq_getie())
		litex_vexriscv_irq_setie(1);
}

static void litex_vexriscv_intc_ack(struct irq_data *data)
{
	/* no need to ack IRQs */
}

static void litex_vexriscv_intc_handle_irq(struct pt_regs *regs)
{
	unsigned int irq;
	int i;

	irq = litex_vexriscv_irq_pending() & litex_vexriscv_irq_getmask();
	pr_debug("irqchip: Litex Vexriscv irqchip mask: 0x%08x pending: 0x%08x",
		litex_vexriscv_irq_getmask(),
		litex_vexriscv_irq_pending());

	for (i = 0; i < IRQ_VEC_SZ; i++) {
		pr_debug("irqchip: i %d, irq 0x%08x, trying 0x%08lx\n", i, irq, BIT(i));

		if (irq & BIT(i)) {
			pr_debug("irqchip: handling at i = %d\n", i);
			handle_domain_irq(root_domain, i, regs);
		}
	}
}

static const struct irq_domain_ops litex_vexriscv_domain_ops = {
	.xlate	= irq_domain_xlate_onecell,
	.map	= litex_vexriscv_intc_map,
};

static struct litex_vexriscv_intc litex_vexriscv_intc0 = {
	.handle	= handle_edge_irq,
	.flags	= IRQ_TYPE_DEFAULT | IRQ_TYPE_PROBE,
	.chip	= {
		.name		= "litex-vexriscv-intc0",
		.irq_mask	= litex_vexriscv_intc_mask,
		.irq_unmask	= litex_vexriscv_intc_unmask,
		.irq_ack	= litex_vexriscv_intc_ack,
	},
};

static int __init litex_vexriscv_intc_init(struct device_node *node, struct device_node *parent)
{
	/* clear interrupts for init */
	litex_vexriscv_irq_setie(0);
	litex_vexriscv_irq_setmask(0);

	root_domain = irq_domain_add_linear(node, IRQ_VEC_SZ,
					    &litex_vexriscv_domain_ops,
					    &litex_vexriscv_intc0);
	irq_set_default_host(root_domain);
	set_handle_irq(litex_vexriscv_intc_handle_irq);

	/* print device info */
	pr_info("irqchip: LiteX VexRiscv irqchip driver initialized. "
		"IE: %d, mask: 0x%08x, pending: 0x%08x\n",
		litex_vexriscv_irq_getie(),
		litex_vexriscv_irq_getmask(),
		litex_vexriscv_irq_pending());
	pr_info("irqchip: LiteX VexRiscv irqchip settings: "
		"mask CSR 0x%03x, pending CSR 0x%03x\n", IRQ_MASK,
		IRQ_PENDING);

	return 0;
}

IRQCHIP_DECLARE(litex_vexriscv_intc0, "vexriscv,intc0", litex_vexriscv_intc_init);
