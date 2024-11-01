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
#include "shell.h"
#include "lorawan_app.h"

#define DELAY K_MSEC(1000)

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lorawan_shell);




extern struct perisistent_config device_config;


static void convert_bin2hex(uint8_t* setting, size_t len, char* buffer) {
	for (int i = 0; i < len; i++) {
		sprintf(buffer+(i*2), "%02x", setting[i]);
	}
	buffer[len*2] = '\0';
}

static void convert_hex2bin(uint8_t* setting, size_t len, char* buffer) {
	for (int i = 0; i < len; i++) {
		sscanf(buffer+(i*2), "%2hhx", &setting[i]);
	}
}

  
static int cmd_deveui(const struct shell *sh, size_t argc, char **argv)
{	
	char buffer[17] = {0};
	if (argc == 1) {
		convert_bin2hex(device_config.deveui, 8, buffer);
		shell_print(sh, "deveui: [%s]", buffer);
	} else {
		if (strlen(argv[1]) != 16) {
			shell_print(sh, "Expects 8 bytes");
			return 1;
		}
		convert_hex2bin(device_config.deveui, 8, argv[1]);
	}
	return 0;
}

static int cmd_joineui(const struct shell *sh, size_t argc, char **argv)
{
	char buffer[17] = {0};
	if (argc == 1) {
		convert_bin2hex(device_config.joineui, 8, buffer);
		shell_print(sh, "joineui: [%s]", buffer);
	} else {
		if (strlen(argv[1]) != 16) {
			shell_print(sh, "Expects 8 bytes");
			return 1;
		}
		convert_hex2bin(device_config.joineui, 8, argv[1]);
	}
	return 0;
}

static int cmd_appkey(const struct shell *sh, size_t argc, char **argv)
{
	char buffer[33] = {0};
	if (argc == 1) {
		convert_bin2hex(device_config.appkey, 16, buffer);
		shell_print(sh, "appkey: [%s]", buffer);
	} else {
		if (strlen(argv[1]) != 32) {
			shell_print(sh, "Expects 16 bytes");
			return 1;
		}
		convert_hex2bin(device_config.appkey, 16, argv[1]);
	}

	return 0;
}

static int cmd_save(const struct shell *sh, size_t argc, char **argv)
{
	int rc = save_config();
	if (rc == 0) {
		shell_print(sh, "OK");
	} else {
		shell_print(sh, "Failed to save %d", rc);
	}
	return 0;
}

static int cmd_send(const struct shell *sh, size_t argc, char **argv)
{
	sscanf(argv[1], "%hhd", &uplinkPort);
	sscanf(argv[2], "%hhd", (uint8_t*)&uplinkType);

	if (uplinkType > 1) {
		shell_print(sh, "Invalid uplink type %d, expects (0:UNC,1:CNF)", uplinkType);
		return 2;
	}

	uplinkSize = strlen(argv[3])/2;
	convert_hex2bin(uplinkPacket, uplinkSize, argv[3]);
	int ret = lorawan_send(uplinkPort, uplinkPacket, uplinkSize, uplinkType);

	if (ret < 0) {
		shell_print(sh, "Send failed %d", ret);
		return 1;
	}

	return 0;
}

static int cmd_join(const struct shell *sh, size_t argc, char **argv)
{
    struct lorawan_join_config join_cfg;

	join_cfg.mode = LORAWAN_ACT_OTAA;
	join_cfg.dev_eui = device_config.deveui;
	join_cfg.otaa.join_eui = device_config.joineui;
	join_cfg.otaa.app_key = device_config.appkey;
	join_cfg.otaa.nwk_key = device_config.appkey;

    device_config.devnonce++;
	// need to save/restore nonce device_config
	join_cfg.otaa.dev_nonce = device_config.devnonce; 
	// join_cfg.otaa.join_nonce = 0u;

	lorawan_join_network(&join_cfg);
    
    save_config();
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_lorawan,
	SHELL_CMD_ARG(deveui, NULL, "Set/get [deveui:HEX8]", cmd_deveui, 1, 1),
	SHELL_CMD_ARG(joineui, NULL, "Set/get [joineui:HEX8]", cmd_joineui, 1, 1),
	SHELL_CMD_ARG(appkey, NULL, "Set/get [appkey:HEX16]", cmd_appkey, 1, 1),
	SHELL_CMD(save, NULL, "Save Config", cmd_save),
	SHELL_CMD(join, NULL, "Join Network", cmd_join),
	SHELL_CMD_ARG(send, NULL, "<port> <0:UNC,1:CNF> <payload:HEX>", cmd_send, 4, 0)
);

SHELL_CMD_REGISTER(lw, &sub_lorawan, "LoRaWAN Commands", NULL);


