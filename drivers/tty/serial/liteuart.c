// SPDX-License-Identifier: GPL-2.0
/*
 * LiteUART serial controller (LiteX) Driver
 *
 * Copyright (C) 2019 Antmicro Ltd <www.antmicro.com>
 */

#include <linux/console.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/timer.h>
#include <linux/tty_flip.h>

/* module-related defines */
#define DRIVER_NAME	"liteuart"
#define DRIVER_MAJOR	0
#define DRIVER_MINOR	0
#define DEV_NAME	"ttyLXU"

/* base address offsets */
#define OFF_RXTX	0x00
#define OFF_TXFULL	0x04
#define OFF_RXEMPTY	0x08
#define OFF_EV_STATUS	0x0c
#define OFF_EV_PENDING	0x10
#define OFF_EV_ENABLE	0x14

/* events */
#define EV_TX	0x1
#define EV_RX	0x2

/* setting/getting registers */
#ifdef __LITTLE_ENDIAN
# define PORT_REG_SET(port, off, val)	iowrite32(val, port->membase + \
		OFF_##off)
# define PORT_REG_GET(port, off)	ioread32(port->membase + OFF_##off)
#else
# define PORT_REG_SET(port, off, val)	iowrite32be(val, port->membase + \
		OFF_##off)
# define PORT_REG_GET(port, off)	ioread32be(port->membase + OFF_##off)
#endif

struct liteuart_port {
	struct uart_port port;
	struct timer_list timer;
};

#define to_liteuart_port(port)	container_of(port, struct liteuart_port, port)

static struct liteuart_port liteuart_ports[CONFIG_SERIAL_LITEUART_NR_PORTS];

#ifdef CONFIG_SERIAL_LITEUART_CONSOLE
static struct console liteuart_console;
#endif

static struct uart_driver liteuart_driver = {
	.owner = THIS_MODULE,
	.driver_name = DRIVER_NAME,
	.dev_name = DEV_NAME,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.nr = CONFIG_SERIAL_LITEUART_NR_PORTS,
#ifdef CONFIG_SERIAL_LITEUART_CONSOLE
	.cons = &liteuart_console,
#endif
};

static void liteuart_timer(struct timer_list *t)
{
	struct liteuart_port *uart = from_timer(uart, t, timer);
	struct uart_port *port = &uart->port;
	unsigned int flg = TTY_NORMAL;
	int ch;
	unsigned int status;

	while ((status = !PORT_REG_GET(port, RXEMPTY)) == 1) {
		ch = PORT_REG_GET(port, RXTX);
		port->icount.rx++;

		/* necessary for RXEMPTY to refresh its value */
		PORT_REG_SET(port, EV_PENDING, EV_TX | EV_RX);

		/* no overflow bits in status */
		if (!(uart_handle_sysrq_char(port, ch)))
			uart_insert_char(port, status, 0, ch, flg);

		tty_flip_buffer_push(&port->state->port);
	}

	mod_timer(&uart->timer, jiffies + uart_poll_timeout(port));
}

static void liteuart_putchar(struct uart_port *port, int ch)
{
	while (PORT_REG_GET(port, TXFULL))
		cpu_relax();

	PORT_REG_SET(port, RXTX, ch);
}

static unsigned int liteuart_tx_empty(struct uart_port *port)
{
	/* not really tx empty, just checking if tx is not full */
	if (!PORT_REG_GET(port, TXFULL))
		return TIOCSER_TEMT;

	return 0;
}

static void liteuart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* modem control register is not present in LiteUART */
}

static unsigned int liteuart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void liteuart_stop_tx(struct uart_port *port)
{
}

static void liteuart_start_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	unsigned char ch;

	if (unlikely(port->x_char)) {
		PORT_REG_SET(port, RXTX, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
	} else if (!uart_circ_empty(xmit)) {
		while (xmit->head != xmit->tail) {
			ch = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			port->icount.tx++;
			liteuart_putchar(port, ch);
		}
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static void liteuart_stop_rx(struct uart_port *port)
{
	struct liteuart_port *uart = to_liteuart_port(port);

	/* just delete timer */
	del_timer(&uart->timer);
}

static void liteuart_break_ctl(struct uart_port *port, int break_state)
{
	/* LiteUART doesn't support sending break signal */
}

static int liteuart_startup(struct uart_port *port)
{
	struct liteuart_port *uart = to_liteuart_port(port);

	/* disable events */
	PORT_REG_SET(port, EV_ENABLE, 0);

	/* prepare timer for polling */
	timer_setup(&uart->timer, liteuart_timer, 0);
	mod_timer(&uart->timer, jiffies + uart_poll_timeout(port));

	return 0;
}

static void liteuart_shutdown(struct uart_port *port)
{
}

static void liteuart_set_termios(struct uart_port *port, struct ktermios *new,
				 struct ktermios *old)
{
	unsigned int baud;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* update baudrate */
	baud = uart_get_baud_rate(port, new, old, 0, 460800);
	uart_update_timeout(port, new->c_cflag, baud);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *liteuart_type(struct uart_port *port)
{
	return (port->type == PORT_LITEUART) ? DRIVER_NAME : NULL;
}

static void liteuart_release_port(struct uart_port *port)
{
}

static int liteuart_request_port(struct uart_port *port)
{
	return 0;
}

static void liteuart_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_LITEUART;
}

static int liteuart_verify_port(struct uart_port *port,
				struct serial_struct *ser)
{
	if (port->type != PORT_UNKNOWN && ser->type != PORT_LITEUART)
		return -EINVAL;

	return 0;
}

static const struct uart_ops liteuart_ops = {
	.tx_empty	= liteuart_tx_empty,
	.set_mctrl	= liteuart_set_mctrl,
	.get_mctrl	= liteuart_get_mctrl,
	.stop_tx	= liteuart_stop_tx,
	.start_tx	= liteuart_start_tx,
	.stop_rx	= liteuart_stop_rx,
	.break_ctl	= liteuart_break_ctl,
	.startup	= liteuart_startup,
	.shutdown	= liteuart_shutdown,
	.set_termios	= liteuart_set_termios,
	.type		= liteuart_type,
	.release_port	= liteuart_release_port,
	.request_port	= liteuart_request_port,
	.config_port	= liteuart_config_port,
	.verify_port	= liteuart_verify_port,
};

static int liteuart_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct liteuart_port *uart;
	struct uart_port *port;
	int dev_id;

	/* no device tree */
	if (!np)
		return -ENODEV;

	dev_id = of_alias_get_id(np, "serial");
	if (dev_id < 0 || dev_id >= CONFIG_SERIAL_LITEUART_NR_PORTS)
		return -EINVAL;

	uart = &liteuart_ports[dev_id];
	port = &uart->port;

	/* get {map,mem}base */
	port->mapbase = platform_get_resource(pdev, IORESOURCE_MEM, 0)->start;
	port->membase = of_iomap(np, 0);
	if (!port->membase)
		return -ENXIO;

	/* values not from device tree */
	port->dev = &pdev->dev;
	port->iotype = UPIO_MEM;
	port->flags = UPF_BOOT_AUTOCONF;
	port->ops = &liteuart_ops;
	port->regshift = 2;
	port->fifosize = 16;
	port->iobase = 1;
	port->type = PORT_UNKNOWN;
	port->line = dev_id;

	return uart_add_one_port(&liteuart_driver,
				 &liteuart_ports[dev_id].port);
}

static int liteuart_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id liteuart_of_match[] = {
	{ .compatible = "litex,liteuart" },
	{}
};
MODULE_DEVICE_TABLE(of, liteuart_of_match);

static struct platform_driver liteuart_platform_driver = {
	.probe = liteuart_probe,
	.remove = liteuart_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(liteuart_of_match),
	},
};

#ifdef CONFIG_SERIAL_LITEUART_CONSOLE

static void liteuart_console_write(struct console *co, const char *s,
	unsigned int count)
{
	struct uart_port *port = &liteuart_ports[co->index].port;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	uart_console_write(port, s, count, liteuart_putchar);
	spin_unlock_irqrestore(&port->lock, flags);
}

static int liteuart_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	port = &liteuart_ports[co->index].port;
	if (!port->membase)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console liteuart_console = {
	.name = DRIVER_NAME,
	.write = liteuart_console_write,
	.device = uart_console_device,
	.setup = liteuart_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &liteuart_driver,
};

static int __init liteuart_console_init(void)
{
	register_console(&liteuart_console);

	return 0;
}

console_initcall(liteuart_console_init);
#endif /* CONFIG_SERIAL_LITEUART_CONSOLE */

static int __init liteuart_init(void)
{
	int res;

	res = uart_register_driver(&liteuart_driver);
	if (res)
		return res;

	res = platform_driver_register(&liteuart_platform_driver);
	if (res) {
		uart_unregister_driver(&liteuart_driver);
		return res;
	}

	return 0;
}

static void __exit liteuart_exit(void)
{
	platform_driver_unregister(&liteuart_platform_driver);
	uart_unregister_driver(&liteuart_driver);
}

module_init(liteuart_init);
module_exit(liteuart_exit);

MODULE_AUTHOR("Filip Kokosinski");
MODULE_DESCRIPTION("LiteUART serial driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
