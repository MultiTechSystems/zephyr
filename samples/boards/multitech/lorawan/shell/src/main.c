/*
 * Class A LoRaWAN sample application
 *
 * Copyright (c) 2020 Manivannan Sadhasivam <mani@kernel.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/drivers/eeprom.h>

#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include "config.h"
#include "lorawan_app.h"


#define DELAY K_MSEC(1000)

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lorawan_shell_main);



int main(void)
{
	init_config();
	lorawan_init();

	return 0;
}
