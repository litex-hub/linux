/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_LITEX_H
#define _LINUX_LITEX_H

#include <linux/io.h>
#include <linux/types.h>
#include <linux/compiler_types.h>

#define LITEX_REG_SIZE             0x4
#define LITEX_SUBREG_SIZE          0x1
#define LITEX_SUBREG_SIZE_BIT      (LITEX_SUBREG_SIZE * 8)

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

static inline u32 read_pointer_with_barrier(const volatile void __iomem *addr)
{
    u32 val;
    __io_br();
    val = *(const volatile u32 __force *)addr;
    __io_ar();
    return val;
}

static inline void write_pointer_with_barrier(volatile void __iomem *addr, u32 val)
{
    __io_br();
    *(volatile u32 __force *)addr = val;
    __io_ar();
}

// Helper functions for manipulating LiteX registers
static inline void litex_set_reg(void __iomem *reg, u32 reg_size, u32 val)
{
	u32 shifted_data, shift, i;

	for (i = 0; i < reg_size; ++i) {
		shift = ((reg_size - i - 1) * LITEX_SUBREG_SIZE_BIT);
		shifted_data = val >> shift;
		write_pointer_with_barrier(reg + (LITEX_REG_SIZE * i), shifted_data);
	}
}

static inline u32 litex_get_reg(void __iomem *reg, u32 reg_size)
{
	u32 shifted_data, shift, i;
	u32 result = 0;

	for (i = 0; i < reg_size; ++i) {
		shifted_data = read_pointer_with_barrier(reg + (LITEX_REG_SIZE * i));
		shift = ((reg_size - i - 1) * LITEX_SUBREG_SIZE_BIT);
		result |= (shifted_data << shift);
	}

	return result;
}

#endif /* _LINUX_LITEX_H */
