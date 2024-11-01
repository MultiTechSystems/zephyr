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



#include "config.h"
#include "lorawan_app.h"



#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lorawan_shell_config);





struct perisistent_config device_config;


#if defined(CONFIG_BOARD_MULTITECH_XDOT_AD)
static const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
const struct device *const mem_pwr_en = DEVICE_DT_GET(DT_NODELABEL(mem_pwr_en));
#endif


#if defined(CONFIG_BOARD_MULTITECH_XDOT_AD) || defined(CONFIG_BOARD_MULTITECH_XDOT)
static const struct device *nvm_dev = DEVICE_DT_GET(DT_ALIAS(eeprom_0));
#define USE_EEPROM 1
#else
static const struct device *nvm_dev = DEVICE_DT_GET(DT_ALIAS(flash1));
#define USE_EEPROM 0
#endif



#define LORAWAN_NVM_OFFSET 0
#define LORAWAN_NVM_MAGIC  0xEE9704
#define SPI_FLASH_SECTOR_SIZE        0x10000

static int nvm_write(off_t offset, const void *data, size_t len) {
	if (USE_EEPROM)
		return eeprom_write(nvm_dev, offset, data, len);
	else {
		LOG_INF("USE FLASH write");
		flash_erase(nvm_dev, offset, SPI_FLASH_SECTOR_SIZE);
		return flash_write(nvm_dev, offset, data, len);
	}
}

static int nvm_read(off_t offset, void *data, size_t len) {
	if (USE_EEPROM)
		return eeprom_read(nvm_dev, offset, data, len);
	else {
		LOG_INF("USE FLASH read");
		return flash_read(nvm_dev, offset, data, len);
	}
}

int init_config() {
    
#if defined(CONFIG_BOARD_MULTITECH_XDOT_AD)
	pm_device_runtime_get(mem_pwr_en);

	if (pm_device_is_powered(mem_pwr_en)) {
		printk("Internal EEPROM/FLASH is ON\n");
	} else {
		printk("Internal EEPROM/FLASH is OFF\n");
	}

	device_init(i2c);
	device_init(nvm_dev);
#endif

	if (!device_is_ready(nvm_dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       nvm_dev->name);
		return 1;
	}

	int rc = nvm_read(LORAWAN_NVM_OFFSET, &device_config, sizeof(device_config));
	if (rc < 0) {
		printk("Error: Couldn't read eeprom: err: %d.\n", rc);
		return 1;
	}

	if (device_config.magic != LORAWAN_NVM_MAGIC) {
		printk("NO MAGIC\n");
		device_config.magic = LORAWAN_NVM_MAGIC;
		memset(device_config.deveui, 0, 8);
		memset(device_config.joineui, 0, 8);
		memset(device_config.appkey, 0, 16);
		device_config.devnonce = 0;
	}

    return 0;
}

int save_config() {
	return nvm_write(LORAWAN_NVM_OFFSET, &device_config, sizeof(device_config));
}
