/* SPDX-License-Identifier: GPL-2.0+ */
#define LITEX_UNUSED(var)         (void)(var)

/* Common values */
#define NS_IN_SEC		1000000000
#define FULL_BYTE		0xFF
#define KHZ			1000

/* MMCM specific numbers */
#define CLKOUT_MAX		7
#define DELAY_TIME_MAX		63
#define PHASE_MUX_MAX		7
#define HIGH_LOW_TIME_SAFE_MAX	62
#define HIGH_LOW_TIME_REG_MAX	63
#define MAX_DIVIDER		126
#define MAX_DIVIDER_F		126000
#define MIN_DIVIDER		2
#define MIN_DIVIDER_F		2000
#define PHASE_MUX_RES_FACTOR	8
/* Fractional divider resolution multiplied by 1000 to keep precision */
#define FRACT_DIV_RES		125
/* Minimal value when div is odd and fract is on */
#define ODD_AND_FRAC		9
/* Odd div and no frac or even div and enabled fract max value */
#define EVEN_AND_FRAC		8

/* DRP registers index */
/* Additional register */
#define MMCM			0
/* DRP registers */
#define DRP_RESET		0
#define DRP_READ		1
#define DRP_WRITE		2
#define DRP_DRDY		3
#define DRP_ADR			4
#define DRP_DAT_W		5
#define DRP_DAT_R		6
#define DRP_LOCKED		7

/* Register space offsets */
#define DRP_OF_RESET		0x0
#define DRP_OF_LOCKED		0x4
#define DRP_OF_READ		0x8
#define DRP_OF_WRITE		0xc
#define DRP_OF_DRDY		0x10
#define DRP_OF_ADR		0x14
#define DRP_OF_DAT_W		0x18
#define DRP_OF_DAT_R		0x20

/* Register sizes */
#define DRP_SIZE_RESET		0x1
#define DRP_SIZE_READ		0x1
#define DRP_SIZE_WRITE		0x1
#define DRP_SIZE_DRDY		0x1
#define DRP_SIZE_ADR		0x1
#define DRP_SIZE_DAT_W		0x2
#define DRP_SIZE_DAT_R		0x2
#define DRP_SIZE_LOCKED		0x1

/* Register values */
#define GLOB_DIV_VAL		0x41
#define GLOB_MUL_VAL		0x82
#define FULL_REG_16		0xFFFF
#define KEEP_REG_16		FULL_REG_16
#define KEEP_IN_MUL		0xF000
#define KEEP_IN_DIV		0xE000
#define ZERO_REG		0x0
#define REG1_MASK		0x1000
#define REG2_MASK		0x8000
#define REG1_BITSET		0x41
#define REG2_BITSET		0x40
#define CLKOUT5_FRAC_MASK	0xC3FF
#define CLKOUT5_FRAC_MASK_F	0xFBFF
#define CLKOUT5_FRAC_MASK_P	0xC7FF
#define REG1_FREQ_MASK		0xF000
#define REG2_FREQ_MASK		0x803F
#define REG1_DUTY_MASK		0xF000
#define REG2_DUTY_MASK		0xFF7F
#define REG1_PHASE_MASK		0x1FFF
#define REG2_PHASE_MASK		0xFCC0
#define REG2_PHASE_F_MASK	0x80C0
#define FILT_MASK		0x9900
#define LOCK1_MASK		0x03FF
#define LOCK23_MASK		0x7FFF
/* Control bits extraction masks */
#define HL_TIME_MASK		0x3F
#define FRAC_MASK		0x7
#define EDGE_MASK		0x1
#define NO_CNT_MASK		0x1
#define FRAC_EN_MASK		0x1
#define PHASE_MUX_MASK		0x7
#define PHASE_MUX_F_MASK	0x7
#define F_FRAC_MASK		0xF8
#define TWO_LSBITS		0x3

/* Bit groups start position in DRP registers */
#define HIGH_TIME_POS		6
#define LOW_TIME_POS		0
#define PHASE_MUX_POS		13
#define PHASE_MUX_F_POS		11
#define FRAC_POS		12
#define FRAC_EN_POS		11
#define FRAC_WF_R_POS		10
#define FRAC_WF_F_POS		10
#define EDGE_POS		7
#define NO_CNT_POS		6
#define DELAY_TIME_POS		0

/* MMCM Register addresses */
#define POWER_REG		0x28
#define DIV_REG			0x16
#define LOCK_REG1		0x18
#define LOCK_REG2		0x19
#define LOCK_REG3		0x1A
#define FILT_REG1		0x4E
#define FILT_REG2		0x4F
#define CLKOUT0_REG1		0x08
#define CLKOUT0_REG2		0x09
#define CLKOUT1_REG1		0x0A
#define CLKOUT1_REG2		0x0B
#define CLKOUT2_REG1		0x0C
#define CLKOUT2_REG2		0x0D
#define CLKOUT3_REG1		0x0E
#define CLKOUT3_REG2		0x0F
#define CLKOUT4_REG1		0x10
#define CLKOUT4_REG2		0x11
#define CLKOUT5_REG1		0x06
#define CLKOUT5_REG2		0x07
#define CLKOUT6_REG1		0x12
#define CLKOUT6_REG2		0x13
#define CLKFBOUT_REG1		0x14
#define CLKFBOUT_REG2		0x15
