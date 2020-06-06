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

#define LITEX_SUBREG_ALIGN     4

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

static inline ulong _readu_cpu(const volatile void __iomem *addr)
{
	return cpu_to_le32(readl(addr));
}

static inline void _writeu_cpu(volatile void __iomem *addr, ulong value)
{
	writel(le32_to_cpu(value), addr);
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

	r = _readu_cpu(a);
	for (i = 1; i < _litex_num_subregs(reg_size); i++) {
		r <<= LITEX_SUBREG_SIZE_BIT;
		a += LITEX_SUBREG_ALIGN;
		r |= _readu_cpu(a);
	}
	return r;
}

/* write value v to a LiteX register of given reg_size, located at address a */
static inline void _litex_wr_reg(void __iomem *a, u32 reg_size, u64 v)
{
	u32 i, ns;

	ns = _litex_num_subregs(reg_size);
	for (i = 0; i < ns; i++) {
		_writeu_cpu(a, v >> (LITEX_SUBREG_SIZE_BIT * (ns - 1 - i)));
		a += LITEX_SUBREG_ALIGN;
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

/* NOTE: Large LiteX CSRs (typically those larger than 64 bits) may represent
 * arrays of smaller (unsigned) integers. The following set of accessors will
 * transfer such an array of standard unsigned integers between a LiteX CSR
 * and a memory buffer. In the case of an array of u8 values, this is the
 * equivalent of memcpy between a LiteX CSR and RAM.
 */

/* read a LiteX register located at address a into a buffer of cnt elements */
#define _litex_rd_reg_buf(a, buf, cnt) \
{ \
	u32 i, j, ns, ns_elem; \
	u64 r; \
	if (sizeof(buf[0]) >= LITEX_SUBREG_SIZE) { \
		/* one or more subregisters per element */ \
		for (i = 0; i < cnt; i++) { \
			buf[i] = _litex_rd_reg(a, sizeof(buf[0])); \
			a += LITEX_SUBREG_ALIGN * \
			     _litex_num_subregs(sizeof(buf[0])); \
		} \
	} else { \
		/* multiple elements per subregister (2, 4, or 8) */ \
		ns = _litex_num_subregs(sizeof(buf[0]) * cnt); \
		ns_elem = LITEX_SUBREG_SIZE / sizeof(buf[0]); \
		for (i = 0; i < ns; i++) { \
			r = _readu_cpu(a); \
			for (j = ns_elem - 1; j >= 0; j--) { \
				if (i * ns_elem + j < cnt) \
					buf[i * ns_elem + j] = r; \
				r >>= sizeof(buf[0]) * 8; \
			} \
			a += LITEX_SUBREG_ALIGN;  \
		} \
	} \
}

/* write a LiteX register located at addres a from a buffer of cnt elements */
#define _litex_wr_reg_buf(a, buf, cnt) \
{ \
        u32 i, j, ns, ns_elem; \
        u64 v; \
        if (sizeof(buf[0]) >= LITEX_SUBREG_SIZE) { \
                /* one or more subregisters per element */ \
                for (i = 0; i < cnt; i++) { \
                        _litex_wr_reg(a, buf[i], sizeof(buf[0])); \
                        a += LITEX_SUBREG_ALIGN * \
			     _litex_num_subregs(sizeof(buf[0])); \
                } \
        } else { \
                /* multiple elements per subregister (2, 4, or 8) */ \
                ns = _litex_num_subregs(sizeof(buf[0]) * cnt); \
                ns_elem = LITEX_SUBREG_SIZE / sizeof(buf[0]); \
                for (i = 0; i < ns; i++) { \
                        v = buf[i * ns_elem + 0]; \
                        for (j = 1; j < ns_elem; j++) { \
                                if (i * ns_elem + j == cnt) \
                                        break; \
                                v <<= sizeof(buf[0]) * 8; \
                                v |= buf[i * ns_elem + j]; \
                        } \
                        _writeu_cpu(a, v); \
                        a += LITEX_SUBREG_ALIGN;  \
                } \
        } \
}

static inline void litex_reg_rdbuf_b(void __iomem *a, uint8_t *buf, int cnt)
{
	_litex_rd_reg_buf(a, buf, cnt);
}

static inline void litex_reg_wrbuf_b(void __iomem *a,
				     const uint8_t *buf, int cnt)
{
	_litex_wr_reg_buf(a, buf, cnt);
}

static inline void litex_reg_rdbuf_w(void __iomem *a, uint16_t *buf, int cnt)
{
	_litex_rd_reg_buf(a, buf, cnt);
}

static inline void litex_reg_wrbuf_w(void __iomem *a,
				     const uint16_t *buf, int cnt)
{
	_litex_wr_reg_buf(a, buf, cnt);
}

static inline void litex_reg_rdbuf_l(void __iomem *a, uint32_t *buf, int cnt)
{
	_litex_rd_reg_buf(a, buf, cnt);
}

static inline void litex_reg_wrbuf_l(void __iomem *a,
				     const uint32_t *buf, int cnt)
{
	_litex_wr_reg_buf(a, buf, cnt);
}

/* NOTE: the macros' "else" branch is never reached, so no need for warning
 * re. >= 64bit left shift */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshift-count-overflow"
static inline void litex_reg_rdbuf_q(void __iomem *a, uint64_t *buf, int cnt)
{
	_litex_rd_reg_buf(a, buf, cnt);
}

static inline void litex_reg_wrbuf_q(void __iomem *a,
				     const uint64_t *buf, int cnt)
{
	_litex_wr_reg_buf(a, buf, cnt);
}
#pragma GCC diagnostic pop

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
