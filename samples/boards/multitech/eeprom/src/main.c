/*
 * Copyright (c) 2021 Thomas Stranger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/drivers/eeprom.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#define EEPROM_SAMPLE_OFFSET 0
#define EEPROM_SAMPLE_MAGIC  0xEE9703

struct perisistant_values {
	uint32_t magic;
	uint32_t boot_count;
};


#if defined(CONFIG_BOARD_XDOT_AD)
static const struct device *i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
const struct device *const mem_pwr_en = DEVICE_DT_GET(DT_NODELABEL(mem_pwr_en));
#endif

static const struct device *eeprom = DEVICE_DT_GET(DT_ALIAS(eeprom_0));
/*
 * Get a device structure from a devicetree node with alias eeprom-0
 */
static const struct device *get_eeprom_device(void)
{
#if defined(CONFIG_BOARD_XDOT_AD)
	pm_device_runtime_get(mem_pwr_en);

	if (pm_device_is_powered(mem_pwr_en)) {
		printk("Internal EEPROM/FLASH is ON\n");
	} else {
		printk("Internal EEPROM/FLASH is OFF\n");
	}

	device_init(i2c);
	device_init(eeprom);
#endif

	if (!device_is_ready(eeprom)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       eeprom->name);
		return NULL;
	}

	printk("Found EEPROM device \"%s\"\n", eeprom->name);
	return eeprom;
}


int main(void)
{

	const struct device *eeprom = get_eeprom_device();
	size_t eeprom_size;
	struct perisistant_values values;
	int rc;

	if (eeprom == NULL) {
		return 0;
	}

	eeprom_size = eeprom_get_size(eeprom);
	printk("Using eeprom with size of: %zu.\n", eeprom_size);

	rc = eeprom_read(eeprom, EEPROM_SAMPLE_OFFSET, &values, sizeof(values));
	if (rc < 0) {
		printk("Error: Couldn't read eeprom: err: %d.\n", rc);
		return 0;
	}

	if (values.magic != EEPROM_SAMPLE_MAGIC) {
		values.magic = EEPROM_SAMPLE_MAGIC;
		values.boot_count = 0;
	}

	values.boot_count++;
	printk("Device booted %d times.\n", values.boot_count);

	rc = eeprom_write(eeprom, EEPROM_SAMPLE_OFFSET, &values, sizeof(values));
	if (rc < 0) {
		printk("Error: Couldn't write eeprom: err:%d.\n", rc);
		return 0;
	}

	printk("Reset the MCU to see the increasing boot counter.\n\n");
	return 0;
}
