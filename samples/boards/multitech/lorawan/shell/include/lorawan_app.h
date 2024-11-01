/*
 * Class A LoRaWAN sample application
 *
 * Copyright (c) 2020 Manivannan Sadhasivam <mani@kernel.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MULTITECH_LORAWAN_APP_H_
#define MULTITECH_LORAWAN_APP_H_

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



#define DELAY K_MSEC(1000)


#define AT_APPLICATION_VERSION "0.0.0"
#define LW_VERSION "1.0.4"
#define RP_VERSION "1.0.3"


int lorawan_init(void);
void lorawan_join_network(struct lorawan_join_config *join_cfg);
void lorwan_datarate_changed(enum lorawan_datarate dr);


#endif