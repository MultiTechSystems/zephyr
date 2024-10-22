/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <stdio.h>
#include <string.h>

#if defined(CONFIG_BOARD_ADAFRUIT_FEATHER_STM32F405)
#define SPI_FLASH_TEST_REGION_OFFSET 0xf000
#elif defined(CONFIG_BOARD_ARTY_A7_DESIGNSTART_FPGA_CORTEX_M1) || \
	defined(CONFIG_BOARD_ARTY_A7_DESIGNSTART_FPGA_CORTEX_M3)
/* The FPGA bitstream is stored in the lower 536 sectors of the flash. */
#define SPI_FLASH_TEST_REGION_OFFSET \
	DT_REG_SIZE(DT_NODE_BY_FIXED_PARTITION_LABEL(fpga_bitstream))
#elif defined(CONFIG_BOARD_NPCX9M6F_EVB) || \
	defined(CONFIG_BOARD_NPCX7M6FB_EVB)
#define SPI_FLASH_TEST_REGION_OFFSET 0x7F000
#else
#define SPI_FLASH_TEST_REGION_OFFSET 0x00000
#endif

#define SPI_FLASH_SECTOR_SIZE        0x10000

#define SPI_FLASH_MULTI_SECTOR_TEST

#if defined(CONFIG_FLASH_STM32_OSPI) || \
	defined(CONFIG_FLASH_STM32_QSPI) || \
	defined(CONFIG_FLASH_STM32_XSPI)
#define SPI_FLASH_MULTI_SECTOR_TEST
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(jedec_spi_nor)
#define SPI_FLASH_COMPAT jedec_spi_nor
#else
#define SPI_FLASH_COMPAT invalid
#endif

const uint8_t erased[] = { 0xff, 0xff, 0xff, 0xff };

void single_sector_test(const struct device *flash_dev)
{
	const uint8_t expected[] = { 0x55, 0xaa, 0x66, 0x99 };
	const size_t len = sizeof(expected);
	uint8_t buf[sizeof(expected)];
	int rc;

	k_sleep(K_MSEC(1000));

	printf("\nTEST OFFSET: %d SECTOR SIZE: %d\r\n", SPI_FLASH_TEST_REGION_OFFSET, SPI_FLASH_SECTOR_SIZE);

	printf("\nPerform test on single sector");
	/* Write protection needs to be disabled before each write or
	 * erase, since the flash component turns on write protection
	 * automatically after completion of write and erase
	 * operations.
	 */
	printf("\nTest 1: Flash erase\n");

	/* Full flash erase if SPI_FLASH_TEST_REGION_OFFSET = 0 and
	 * SPI_FLASH_SECTOR_SIZE = flash size
	 */
	rc = flash_erase(flash_dev, SPI_FLASH_TEST_REGION_OFFSET,
			 SPI_FLASH_SECTOR_SIZE);
	if (rc != 0) {
		printf("Flash erase failed! %d\n", rc);
	} else {
		/* Check erased pattern */
		memset(buf, 0, len);
		rc = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, buf, len);
		if (rc != 0) {
			printf("Flash read failed! %d\n", rc);
			return;
		}
		if (memcmp(erased, buf, len) != 0) {
			printf("Flash erase failed at offset 0x%x got 0x%x\n",
				SPI_FLASH_TEST_REGION_OFFSET, *(uint32_t *)buf);
			return;
		}
		printf("Flash erase succeeded!\n");
	}
	printf("\nTest 2: Flash write\n");

	printf("Attempting to write %zu bytes\n", len);
	rc = flash_write(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, expected, len);
	if (rc != 0) {
		printf("Flash write failed! %d\n", rc);
		return;
	}

	memset(buf, 0, len);
	rc = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, buf, len);
	if (rc != 0) {
		printf("Flash read failed! %d\n", rc);
		return;
	}

	if (memcmp(expected, buf, len) == 0) {
		printf("Data read matches data written. Good!!\n");
	} else {
		const uint8_t *wp = expected;
		const uint8_t *rp = buf;
		const uint8_t *rpe = rp + len;

		printf("Data read does not match data written!!\n");
		while (rp < rpe) {
			printf("%08x wrote %02x read %02x %s\n",
			       (uint32_t)(SPI_FLASH_TEST_REGION_OFFSET + (rp - buf)),
			       *wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
			++rp;
			++wp;
		}
	}
}

#if defined SPI_FLASH_MULTI_SECTOR_TEST
void multi_sector_test(const struct device *flash_dev)
{
	const uint8_t expected[] = { 0x55, 0xaa, 0x66, 0x99 };
	const size_t len = sizeof(expected);
	uint8_t buf[sizeof(expected)];
	int rc;

	printf("\nPerform test on multiple consecutive sectors");

	/* Write protection needs to be disabled before each write or
	 * erase, since the flash component turns on write protection
	 * automatically after completion of write and erase
	 * operations.
	 */
	printf("\nTest 1: Flash erase\n");

	/* Full flash erase if SPI_FLASH_TEST_REGION_OFFSET = 0 and
	 * SPI_FLASH_SECTOR_SIZE = flash size
	 * Erase 2 sectors for check for erase of consequtive sectors
	 */
	rc = flash_erase(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, SPI_FLASH_SECTOR_SIZE * 2);
	if (rc != 0) {
		printf("Flash erase failed! %d\n", rc);
	} else {
		/* Read the content and check for erased */
		memset(buf, 0, len);
		size_t offs = SPI_FLASH_TEST_REGION_OFFSET;

		while (offs < SPI_FLASH_TEST_REGION_OFFSET + 2 * SPI_FLASH_SECTOR_SIZE) {
			rc = flash_read(flash_dev, offs, buf, len);
			if (rc != 0) {
				printf("Flash read failed! %d\n", rc);
				return;
			}
			if (memcmp(erased, buf, len) != 0) {
				printf("Flash erase failed at offset 0x%x got 0x%x\n",
				offs, *(uint32_t *)buf);
				return;
			}
			offs += SPI_FLASH_SECTOR_SIZE;
		}
		printf("Flash erase succeeded!\n");
	}

	printf("\nTest 2: Flash write\n");

	size_t offs = SPI_FLASH_TEST_REGION_OFFSET;

	while (offs < SPI_FLASH_TEST_REGION_OFFSET + 2 * SPI_FLASH_SECTOR_SIZE) {
		printf("Attempting to write %zu bytes at offset 0x%x\n", len, offs);
		rc = flash_write(flash_dev, offs, expected, len);
		if (rc != 0) {
			printf("Flash write failed! %d\n", rc);
			return;
		}

		memset(buf, 0, len);
		rc = flash_read(flash_dev, offs, buf, len);
		if (rc != 0) {
			printf("Flash read failed! %d\n", rc);
			return;
		}

		if (memcmp(expected, buf, len) == 0) {
			printf("Data read matches data written. Good!!\n");
		} else {
			const uint8_t *wp = expected;
			const uint8_t *rp = buf;
			const uint8_t *rpe = rp + len;

			printf("Data read does not match data written!!\n");
			while (rp < rpe) {
				printf("%08x wrote %02x read %02x %s\n",
					(uint32_t)(offs + (rp - buf)),
					*wp, *rp, (*rp == *wp) ? "match" : "MISMATCH");
				++rp;
				++wp;
			}
		}
		offs += SPI_FLASH_SECTOR_SIZE;
	}
}
#endif

#if defined(CONFIG_BOARD_MULTITECH_XDOT_AD)

#define LED0_NODE DT_ALIAS(eeprom_switch)
static const struct gpio_dt_spec eeprom_switch = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define FLASH_CS_EXT_NODE DT_ALIAS(flash_cs_ext)
static const struct gpio_dt_spec flash_cs_switch = GPIO_DT_SPEC_GET(FLASH_CS_EXT_NODE, gpios);

#endif

int main(void)
{
	
#if defined(CONFIG_BOARD_MULTITECH_XDOT_AD)

	int err = 0;
	
	if (!gpio_is_ready_dt(&eeprom_switch)) {
		printf("The load switch pin GPIO port is not ready.\n");
	}

	err = gpio_pin_configure_dt(&eeprom_switch, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		printf("Configuring GPIO pin failed: %d\n", err);
	}

	err = gpio_pin_set_dt(&eeprom_switch, 1);
	if (err != 0) {
		printf("Setting GPIO pin level failed: %d\n", err);
	}

	if (!gpio_is_ready_dt(&flash_cs_switch)) {
		printf("The load switch pin GPIO port is not ready.\n");
	}

	err = gpio_pin_configure_dt(&flash_cs_switch, GPIO_OUTPUT_LOW);
	if (err != 0) {
		printf("Configuring GPIO pin failed: %d\n", err);
	}

	err = gpio_pin_set_dt(&flash_cs_switch, 0);
	if (err != 0) {
		printf("Setting GPIO pin level failed: %d\n", err);
	}

	k_msleep(50);
#endif

#if 0
	bool led_state = true;
	int switch_value = 1;
	int ret = 0;
	int cnt = 0;

	while (false) {
		ret = gpio_pin_set_dt(&eeprom_switch, switch_value);
		
		if (ret < 0) {
			return 0;
		}
		printf("LED state: %s %d\n", led_state ? "ON" : "OFF", switch_value);

		led_state = !led_state;
		switch_value = switch_value == 1 ? 0 : 1;
		k_msleep(3000);

		if (cnt++ > 5)
			break;
	}


	gpio_pin_set_dt(&eeprom_switch, 1);

	k_msleep(50);
#endif

	// gpio_pin_set_dt(&eeprom_switch, 0);

	k_msleep(50);

	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(flash1));

	if (!device_is_ready(flash_dev)) {
		printk("%s: device not ready.\n", flash_dev->name);

#if 0
		k_msleep(250);

		while (true) {
			ret = gpio_pin_set_dt(&eeprom_switch, switch_value);
			
			if (ret < 0) {
				return 0;
			}
			printf("LED state: %s %d\n", led_state ? "ON" : "OFF", switch_value);

			led_state = !led_state;
			switch_value = switch_value == 1 ? 0 : 1;
			k_msleep(3000);

			if (cnt++ > 5)
				break;
		}
#endif
		return 0;
	}

	printf("\n%s SPI flash testing\n", flash_dev->name);
	printf("==========================\n");

	single_sector_test(flash_dev);
#if defined SPI_FLASH_MULTI_SECTOR_TEST
	multi_sector_test(flash_dev);
#endif

	while (1) {
		k_msleep(200);
	}

	return 0;
}
