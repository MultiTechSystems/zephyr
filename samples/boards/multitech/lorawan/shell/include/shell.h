/*
 * Class A LoRaWAN sample application
 *
 * Copyright (c) 2020 Manivannan Sadhasivam <mani@kernel.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MULTITECH_LORAWAN_SHELL_H_
#define MULTITECH_LORAWAN_SHELL_H_



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




uint8_t uplinkPort = 2;
uint8_t uplinkPacket[255] = { 0xFF };
uint8_t uplinkSize = 1;
enum lorawan_message_type uplinkType = LORAWAN_MSG_UNCONFIRMED;
bool network_joined = false;



  
static int cmd_deveui(const struct shell *sh, size_t argc, char **argv);
static int cmd_joineui(const struct shell *sh, size_t argc, char **argv);
static int cmd_appkey(const struct shell *sh, size_t argc, char **argv);
static int cmd_save(const struct shell *sh, size_t argc, char **argv);
static int cmd_send(const struct shell *sh, size_t argc, char **argv);
static int cmd_join(const struct shell *sh, size_t argc, char **argv);


#endif