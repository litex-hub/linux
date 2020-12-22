/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2020 Antmicro <www.antmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _LITEVIDEO_MMCM_H_
#define _LITEVIDEO_MMCM_H_

/* mmcm addresses */

#define MMCM_CLKFBOUT1     0x14
#define MMCM_DIVCLK        0x16
#define MMCM_CLKOUT0       0x08
#define MMCM_CLKOUT1       0x0a

/* predefined values and shifts */

#define MMCM_HT_FALLING_EDGE   0x1000
#define MMCM_HT_SHIFT          6
#define MMCM_LT_SHIFT          0

/* This values should be set both to HIGH_TIME and LOW_TIME in CLKOUTx
 * - clk division time = 2*VALUE   // (i.e. MMCM_CLKOUT_DIV10)
 * - duty              = 50%       // (HIGH_TIME / (HIGH_TIME + LOW_TIME))
 */

#define MMCM_CLKOUT_DIV10  5
#define MMCM_CLKOUT_DIV2   1

#define MMCM_WRITE         1

#define MMCM_MIN_M         2
#define MMCM_MAX_M         128
#define MMCM_MIN_D         1
#define MMCM_MAX_D         128

#endif /* _LITEVIDEO_MMCM_H_ */
