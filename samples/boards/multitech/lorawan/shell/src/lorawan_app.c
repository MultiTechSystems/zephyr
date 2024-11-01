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
LOG_MODULE_REGISTER(lorawan_shell_app);



#define AT_APPLICATION_VERSION "0.0.0"
#define LW_VERSION "1.0.4"
#define RP_VERSION "1.0.3"


const struct device *lora_dev;

void lorawan_join_network(struct lorawan_join_config *join_cfg) {
	int ret = -1;

	uint16_t ch_mask[LORAWAN_CHANNELS_MASK_SIZE_US915] = { 0x00ff, 0, 0, 0, 0x0001 };


	LOG_INF("Joining network over OTAA");

	lorawan_set_channels_mask(ch_mask, LORAWAN_CHANNELS_MASK_SIZE_US915);

	ret = lorawan_join(join_cfg);
	
	if (ret != 0) {
		LOG_ERR("lorawan_join_network failed: %d", ret);
		k_sleep(DELAY);
	}
 
}


static void dl_callback(uint8_t port, bool data_pending,
			int16_t rssi, int8_t snr,
			uint8_t len, const uint8_t *hex_data)
{
	LOG_INF("Port %d, Pending %d, RSSI %ddB, SNR %ddBm", port, data_pending, rssi, snr);
	if (hex_data) {
		LOG_HEXDUMP_INF(hex_data, len, "Payload: ");
	}
}

void lorwan_datarate_changed(enum lorawan_datarate dr)
{
	uint8_t unused, max_size;

	lorawan_get_payload_sizes(&unused, &max_size);
	LOG_INF("New Datarate: DR_%d, Max Payload %d", dr, max_size);
}



int lorawan_init(void)
{
	printk("LoRaWAN Shell Demo\n");
    
    int ret = 0;

#if defined(CONFIG_LORAMAC_REGION_EU868)
	/* If more than one region Kconfig is selected, app should set region
	 * before calling lorawan_start()
	 */
	LOG_INF("SET CHPLAN: EU868");
	ret = lorawan_set_region(LORAWAN_REGION_EU868);
	if (ret < 0) {
		LOG_ERR("lorawan_set_region failed: %d", ret);
		return 0;
	}
#elif defined(CONFIG_LORAMAC_REGION_AU915)
	/* If more than one region Kconfig is selected, app should set region
	 * before calling lorawan_start()
	 */
	
	LOG_INF("SET CHPLAN: AU915");
	ret = lorawan_set_region(LORAWAN_REGION_AU915);
	if (ret < 0) {
		LOG_ERR("lorawan_set_region failed: %d", ret);
		return 0;
	}
#elif defined(CONFIG_LORAMAC_REGION_US915)
	/* If more than one region Kconfig is selected, app should set region
	 * before calling lorawan_start()
	 */
	
	LOG_INF("SET CHPLAN: US915");
	ret = lorawan_set_region(LORAWAN_REGION_US915);
	if (ret < 0) {
		LOG_ERR("lorawan_set_region failed: %d", ret);
		return 0;
	}
#endif


	struct lorawan_downlink_cb downlink_cb = {
		.port = LW_RECV_PORT_ANY,
		.cb = dl_callback
	};

	lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

	if (!device_is_ready(lora_dev)) {
		LOG_ERR("%s: device not ready.", lora_dev->name);
		return 0;
	}


	ret = lorawan_start();

	if (ret < 0) {
		LOG_ERR("lorawan_start failed: %d", ret);
		return 0;
	}

	lorawan_register_downlink_callback(&downlink_cb);
	lorawan_register_dr_changed_callback(lorwan_datarate_changed);

	lorawan_enable_adr(true);


	return 0;
}

SYS_INIT(lorawan_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

