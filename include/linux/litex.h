/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_LITEX_H
#define _LINUX_LITEX_H

#include <asm/io.h>

#if defined(CONFIG_LITEX_SUBREG_SIZE) && \
    (CONFIG_LITEX_SUBREG_SIZE == 1 || CONFIG_LITEX_SUBREG_SIZE == 4)
#define LITEX_SUBREG_SIZE      CONFIG_LITEX_SUBREG_SIZE
#else
#error LiteX subregister size (LITEX_SUBREG_SIZE) must be 4 or 1!
#endif

#ifdef CONFIG_64BIT
#define LITEX_SUBREG_ALIGN     8
#else
#define LITEX_SUBREG_ALIGN     4
#endif

#define LITEX_SUBREG_SIZE_BIT  (LITEX_SUBREG_SIZE * 8)

// function implemented in
// drivers/soc/litex/litex_soc_controller.c
// to check if accessors are safe to be used
// returns true if yes - false if not
//
// Important: all drivers that use functions from this header
// must check at the beginning of their probe()
// if LiteX SoC Controller driver has checked read and write to CSRs
// and then return -EPROBE_DEFER when false
//
// example:
// if (!litex_check_accessors())
//     return -EPROBE_DEFER;
int litex_check_accessors(void);

static inline ulong _rd_ptr_w_barrier(const volatile void __iomem *addr)
{
	ulong val;

	__io_br();
	val = *(const volatile ulong __force *)addr;
	__io_ar();
	return val;
}

static inline void _wr_ptr_w_barrier(volatile void __iomem *addr, ulong value)
{
	__io_br();
	*(volatile ulong __force *)addr = value;
	__io_ar();
}

/* number of LiteX subregisters needed to store a register of given reg_size */
#define _litex_num_subregs(reg_size) \
	(((reg_size) - 1) / LITEX_SUBREG_SIZE + 1)

/* offset of a LiteX register based on offset and size of preceding register */
#define _next_reg_off(off, size) \
	((off) + _litex_num_subregs(size) * LITEX_SUBREG_ALIGN)

/* read a LiteX register of a given reg_size, located at address a */
static inline u64 _litex_rd_reg(void __iomem *a, u32 reg_size)
{
	u32 i;
	u64 r;

	r = _rd_ptr_w_barrier(a);
	for (i = 1; i < _litex_num_subregs(reg_size); i++) {
		r <<= LITEX_SUBREG_SIZE_BIT;
		r |= _rd_ptr_w_barrier(a + i * LITEX_SUBREG_ALIGN);
	}
	return r;
}

/* write value v to a LiteX register of given reg_size, located at address a */
static inline void _litex_wr_reg(void __iomem *a, u32 reg_size, u64 v)
{
	u32 i, ns;

	ns = _litex_num_subregs(reg_size);
	for (i = 0; i < ns; i++) {
		_wr_ptr_w_barrier(a + i * LITEX_SUBREG_ALIGN,
				  v >> (LITEX_SUBREG_SIZE_BIT * (ns - 1 - i)));
	}
}

/* helper accessors for standard unsigned integer widths (b/w/l/q) */
static inline u8 litex_reg_readb(void __iomem *a)
{
	return _litex_rd_reg(a, sizeof(u8));
}

static inline u16 litex_reg_readw(void __iomem *a)
{
	return _litex_rd_reg(a, sizeof(u16));
}

static inline u32 litex_reg_readl(void __iomem *a)
{
	return _litex_rd_reg(a, sizeof(u32));
}

static inline u64 litex_reg_readq(void __iomem *a)
{
	return _litex_rd_reg(a, sizeof(u64));
}

static inline void litex_reg_writeb(void __iomem *a, u8 v)
{
	_litex_wr_reg(a, sizeof(u8), v);
}

static inline void litex_reg_writew(void __iomem *a, u16 v)
{
	_litex_wr_reg(a, sizeof(u16), v);
}

static inline void litex_reg_writel(void __iomem *a, u32 v)
{
	_litex_wr_reg(a, sizeof(u32), v);
}

static inline void litex_reg_writeq(void __iomem *a, u64 v)
{
	_litex_wr_reg(a, sizeof(u64), v);
}

// for backward compatibility with linux-on-litex-vexriscv existing modules
static inline void litex_set_reg(void __iomem *reg, u32 reg_size, ulong val)
{
	_litex_wr_reg(reg, reg_size, val);
}

static inline ulong litex_get_reg(void __iomem *reg, u32 reg_size)
{
	return _litex_rd_reg(reg, reg_size);
}

#endif /* _LINUX_LITEX_H */
