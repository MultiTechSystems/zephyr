/*
 * Copyright (c) 2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2019 Foundries.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include "rtc_regs.h"
#include "gcr_regs.h"
#include "fcr_regs.h"


static inline void calibrate_mcu()
{
	MXC_RTC->ctrl |= 1; // set enable
	while (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_ERTCO_RDY)) {}

	MXC_FCR->autocal2 = (MXC_FCR->autocal2 & ~MXC_F_FCR_AUTOCAL2_DIV) | (3051 << MXC_F_FCR_AUTOCAL2_DIV_POS);   // div field to 3,051
	MXC_FCR->autocal2 = (MXC_FCR->autocal2 & ~MXC_F_FCR_AUTOCAL2_RUNTIME) | (10 << MXC_F_FCR_AUTOCAL2_RUNTIME_POS);  // runtime field to 10
	MXC_FCR->autocal1 = 0x00000100;   // initial field to 0x100
	// gain field to 4
	MXC_FCR->autocal0 |= 0x407;  // FCR_AUTOCAL0.sel, FCR_AUTOCAL0.en, and FCR_AUTOCAL0.load fields to 1

	k_sleep(K_MSEC(10));

	MXC_FCR->autocal0 &= 0xFFFFFFFD; // FCR_AUTOCAL0.en field to 0

}

static int board_multitech_xdot_es_init(void)
{
	calibrate_mcu();

	return 0;
}

/* needs to be done after GPIO driver init, which is at
 * POST_KERNEL:KERNEL_INIT_PRIORITY_DEFAULT.
 */
SYS_INIT(board_multitech_xdot_es_init, POST_KERNEL, 99);
