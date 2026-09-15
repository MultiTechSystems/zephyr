/*
 * Copyright (c) 2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2019 Foundries.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <stdbool.h>

#include "rtc_regs.h"
#include "gcr_regs.h"
#include "fcr_regs.h"


/* How long to wait for the 32.768 kHz ERTCO to report ready.
 *
 * A crystal of this kind starts in a few hundred milliseconds when it starts at
 * all, so a second is generous and anything past it is not a slow start. */
#define ERTCO_READY_TIMEOUT_MS 1000

static inline bool ertco_wait_ready(void)
{
	/* BOUNDED, BECAUSE THE UNBOUNDED FORM WEDGES THE BOARD.
	 *
	 * This ran `while (!(MXC_GCR->clkctrl & ERTCO_RDY)) {}` with no way out,
	 * at POST_KERNEL priority 99 -- which is after every driver and after the
	 * last application init hook, and before the kernel switches to the main
	 * thread. A boot that spins here therefore produces NO console output at
	 * all, takes no fault, and never reaches main: the board is simply silent
	 * on every UART, which reads as dead hardware rather than as a busy wait.
	 *
	 * It survives a warm reset. A core reset does not reset the RTC block, so
	 * an ERTCO left un-ready by the reset that interrupted its start-up is
	 * still un-ready on the next boot -- and on the one after that. Only
	 * removing power restarts the oscillator, which is exactly the signature
	 * that had this blamed on the SX1262's analog latch-up, a real failure of
	 * that part and a different one: this is on the other side of the board
	 * and no radio has been touched when it happens.
	 *
	 * Measured on an xDot ES over LCTT certification runs: a `DutResetReq`
	 * reboot left the board silent, and every subsequent boot stopped at the
	 * same point until power was cycled.
	 */
	const int64_t deadline = k_uptime_get() + ERTCO_READY_TIMEOUT_MS;

	while (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_ERTCO_RDY)) {
		if (k_uptime_get() > deadline) {
			return false;
		}
		k_busy_wait(100);
	}
	return true;
}

static inline void calibrate_mcu()
{
	MXC_RTC->ctrl |= 1; // set enable

	if (!ertco_wait_ready()) {
		/* CARRY ON WITHOUT THE CALIBRATION, and say so.
		 *
		 * The trim below tunes the IPO against the ERTCO; with no ERTCO
		 * there is nothing to tune against, and applying it anyway would
		 * load a gain computed from a reference that never arrived. The
		 * cost of skipping is an IPO left at its factory trim -- which
		 * shifts the UART divisor and is a console at the wrong baud, a
		 * legible failure. The cost of not returning is a board that never
		 * reaches main, which is not.
		 */
		printk("[board] ERTCO not ready after %d ms -- skipping IPO "
		       "calibration; the console may be off-baud\n",
		       ERTCO_READY_TIMEOUT_MS);
		return;
	}

	MXC_FCR->autocal2 = (MXC_FCR->autocal2 & ~MXC_F_FCR_AUTOCAL2_DIV) | (3051 << MXC_F_FCR_AUTOCAL2_DIV_POS);   // div field to 3,051
	MXC_FCR->autocal2 = (MXC_FCR->autocal2 & ~MXC_F_FCR_AUTOCAL2_RUNTIME) | (10 << MXC_F_FCR_AUTOCAL2_RUNTIME_POS);  // runtime field to 10
	MXC_FCR->autocal1 = 0x00000100;   // initial field to 0x100
	// gain field to 4
	MXC_FCR->autocal0 |= 0x407;  // FCR_AUTOCAL0.sel, FCR_AUTOCAL0.en, and FCR_AUTOCAL0.load fields to 1

	/* BUSY-WAIT, NOT k_sleep. The calibration window has to be waited out
	 * with the scheduler untouched, because this function has just enabled
	 * the RTC and armed an autocal that trims the IPO -- and on this board
	 * the kernel's own tick is derived from that same ERTCO/RTC. A `k_sleep`
	 * here asks the scheduler to wake this thread on a tick that the lines
	 * above may have just stopped, and a tick that never arrives is a sleep
	 * that never returns.
	 *
	 * It runs at POST_KERNEL priority 99, after every driver and every other
	 * init hook and before the switch to the main thread, so a boot that
	 * blocks here prints NOTHING, takes no fault, and never reaches main --
	 * and because a core reset does not reset the RTC block, the next boot
	 * blocks in the same place. Only removing power clears it, which is what
	 * made it look like the SX1262's analog latch-up rather than a sleep.
	 *
	 * `k_busy_wait` spins on the cycle counter instead and owes the scheduler
	 * nothing. The 10 ms is unchanged; only the way of spending it is.
	 */
	k_busy_wait(10 * USEC_PER_MSEC);

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
