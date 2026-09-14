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

		/*
	1. Enable the ERTCO by setting GCR_CLKCTRL.ertco_en to 1.
	2. Wait until GCR_CLKCTRL.ertco_rdy reads 1. The ERTCO is now operating.
	3. Set the FCR_AUTOCAL2.div field to 3,051. See the FCR_AUTOCAL2.div field for additional information.
	4. Set the FCR_AUTOCAL2.runtime field to 10.
	5. Set the FCR_AUTOCAL1.initial field to 0x100.
	6. Set the FCR_AUTOCAL0.gain field to 4.
	7. Set the FCR_AUTOCAL0.sel, FCR_AUTOCAL0.en, and FCR_AUTOCAL0.load fields to 1 by performing a bitwise OR of
	the FCR_AUTOCAL0 register with 0x7.
	8. Wait 10ms for the trim to complete.
	a. The calculated trim is loaded to the FCR_AUTOCAL0.gain field and is used by the hardware as long as the
	FCR_AUTOCAL0.sel field is set to 1.
	9. Set the FCR_AUTOCAL0.en field to 0 to stop the calibration.
	*/

	MXC_RTC->ctrl |= 1; // set enable
	// printk("GCR: %08x\n", MXC_GCR->clkctrl);
	while (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_ERTCO_RDY)) {}

	MXC_FCR->autocal2 = (MXC_FCR->autocal2 & ~MXC_F_FCR_AUTOCAL2_DIV) | (3051 << MXC_F_FCR_AUTOCAL2_DIV_POS);   // div field to 3,051
	MXC_FCR->autocal2 = (MXC_FCR->autocal2 & ~MXC_F_FCR_AUTOCAL2_RUNTIME) | (10 << MXC_F_FCR_AUTOCAL2_RUNTIME_POS);  // runtime field to 10
	MXC_FCR->autocal1 = 0x00000100;   // initial field to 0x100
	// gain field to 4
	MXC_FCR->autocal0 |= 0x407;  // FCR_AUTOCAL0.sel, FCR_AUTOCAL0.en, and FCR_AUTOCAL0.load fields to 1

	k_sleep(K_MSEC(25));

	// printk("AUTOCAL0: %08x\n", MXC_FCR->autocal0);

	MXC_FCR->autocal0 &= 0xFFFFFFFD; // FCR_AUTOCAL0.en field to 0

}

// #define MTS_NVM_ENABLE_GPIO_SPEC	GPIO_DT_SPEC_GET(DT_NODELABEL(nvm_enable), gpios)
// #define MTS_FLASH_CS_EXT_SPEC	GPIO_DT_SPEC_GET(DT_NODELABEL(flash_cs_ext), gpios)
// #define MTS_LED_TEST_SPEC	GPIO_DT_SPEC_GET(DT_NODELABEL(ledtest), gpios)

// static inline void nvm_power_enable(bool on) {
// 	struct gpio_dt_spec nvm_gpio = MTS_NVM_ENABLE_GPIO_SPEC;
// 	struct gpio_dt_spec ext_cs_gpio = MTS_FLASH_CS_EXT_SPEC;
// 	struct gpio_dt_spec led_test = MTS_LED_TEST_SPEC;

// 	if (!gpio_is_ready_dt(&nvm_gpio)) {
// 		return;
// 	}

// 	if (!gpio_is_ready_dt(&ext_cs_gpio)) {
// 		return;
// 	}

// 	if (!gpio_is_ready_dt(&led_test)) {
// 		return;
// 	}

// 	gpio_pin_configure_dt(&led_test, (on ? GPIO_OUTPUT_INACTIVE : GPIO_OUTPUT_ACTIVE));
// 	gpio_pin_configure_dt(&nvm_gpio, (on ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE));
// 	gpio_pin_configure_dt(&ext_cs_gpio, (on ? GPIO_OUTPUT_INACTIVE : GPIO_OUTPUT_ACTIVE));
// }

static int board_multitech_xdot_ad_init(void)
{
	// nvm_power_enable(true);
	calibrate_mcu();

	return 0;
}

/* needs to be done after GPIO driver init, which is at
 * POST_KERNEL:KERNEL_INIT_PRIORITY_DEFAULT.
 */
SYS_INIT(board_multitech_xdot_ad_init, POST_KERNEL, 99);
