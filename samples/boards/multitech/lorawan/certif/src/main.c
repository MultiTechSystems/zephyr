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


/* Customize based on network configuration */
#define LORAWAN_DEV_EUI			{ 0x00, 0x80, 0x00, 0x00, 0x00, 0x00,\
					  0x00, 0x01 }
#define LORAWAN_JOIN_EUI		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					  0x00, 0x00 }
#define LORAWAN_APP_KEY			{ 0x38, 0xaf, 0x28, 0xce, 0xfd, 0x49, 0xed, 0xaf, 0x18, 0xed, 0x46, 0x03, 0xbe, 0x9b, 0x04, 0xc2 }

#define DELAY K_MSEC(1000)

#define LOG_LEVEL LORAWAN_LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lorawan_class_a);




#define AT_APPLICATION_VERSION "0.0.0"
#define LW_VERSION "1.0.4"
#define RP_VERSION "1.0.3"


struct lorawan_join_config join_cfg;
uint8_t uplinkPort = 2;
uint8_t uplinkPacket[255] = { 0xFF };
uint8_t uplinkSize = 1;
enum lorawan_message_type uplinkType = LORAWAN_MSG_UNCONFIRMED;
k_timeout_t uplinkPeriod = K_MSEC(5000);
uint32_t downlinkCounter = 0;
bool downlinkReceived = false;
bool cert_test_joined = false;
bool cert_test_enabled = true;

static void cert_test_join_network() {
	int ret = -1;

	uint16_t ch_mask[LORAWAN_CHANNELS_MASK_SIZE_US915] = { 0x00ff, 0, 0, 0, 0x0001 };


	LOG_INF("Joining network over OTAA");
	 do {
		join_cfg.otaa.dev_nonce++;
		lorawan_set_channels_mask(ch_mask, LORAWAN_CHANNELS_MASK_SIZE_US915);

		ret = lorawan_join(&join_cfg);
		
	    if (ret != 0) {
			LOG_ERR("lorawan_join_network failed: %d", ret);
			k_sleep(DELAY);
		}
	} while (ret < 0);

	cert_test_joined = true;
}


static void dl_callback(uint8_t port, bool data_pending,
			int16_t rssi, int8_t snr,
			uint8_t len, const uint8_t *hex_data)
{
	LOG_INF("Port %d, Pending %d, RSSI %ddB, SNR %ddBm", port, data_pending, rssi, snr);
	if (hex_data) {
		LOG_HEXDUMP_INF(hex_data, len, "Payload: ");
	}

	if (!cert_test_enabled) {
		return;
	}

	downlinkReceived = true;
	uplinkPort = 2;
	uplinkPacket[0] = 0xFF;
	uplinkSize = 1;
	downlinkCounter++;

	if (port == 224) {
		switch (hex_data[0]) {
			case 0x00: { // PackageVersionReq
				LOG_INF("PackageVersionReq");
				uplinkPort = 224;
				uplinkPacket[0] = 0x00;
				uplinkPacket[1] = 0x05;
				uplinkPacket[2] = 0x01;
				uplinkSize = 3;
				break;
			}
			case 0x01: { // DutResetReq
				if (len == 1) {
					LOG_INF("DutResetReq");
					k_sleep(K_MSEC(2000));
					sys_reboot(SYS_REBOOT_WARM);
				}
				break;
			}   
			case 0x02: { // DutJoinReq
				if (len == 1) {
					k_sleep(K_MSEC(5000));
					cert_test_joined = false;
					return;
				}
				break;
			}
			case 0x03: { // SwitchClassReq
				if (hex_data[1] < 3) {
					switch (hex_data[1]) {
						case 0x00:
							lorawan_set_class(LORAWAN_CLASS_A);
							break;
						case 0x01:
							lorawan_set_class(LORAWAN_CLASS_B);
							break;
						case 0x02:
							lorawan_set_class(LORAWAN_CLASS_C);
							break;
					}
				}
				break;
			}
			case 0x04: { // ADR Enabled/Disable
				LOG_INF("ADREnableReq");
				if (hex_data[1] == 1)
					lorawan_enable_adr(true);
				else
					lorawan_enable_adr(false);
				break;
			}
			case 0x05: { // RegionalDutyCycleCtrlReq
				LOG_INF("RegionalDutyCycleCtrlReq");
				if (hex_data[1] == 0) {
					lorawan_cert_test_disable_dutycycle();
				} else { 
					// lorawan_enable_dutycycle(true);
				}
				break;
			}
			case 0x06: { // TxPeriodicityChangeReq
				LOG_INF("TxPeriodicityChangeReq");
				if (hex_data[1] < 2)
					// 0, 1 => 5s
					if (hex_data[1] == 1)
						uplinkPeriod = K_MSEC(1000);
					else
						uplinkPeriod = K_MSEC(0);
					
				else if (hex_data[1] < 8)
					// 2 - 7 => 10s - 60s
					uplinkPeriod = K_MSEC((hex_data[1] - 1) * 10000U);
				else if (hex_data[1] < 11) {
					// 8, 9, 10 => 120s, 240s, 480s
					uplinkPeriod = K_MSEC(120 * (1 << (hex_data[1] - 8)) * 1000U);
				}
				break;
			}
			case 0x07: { // TxFramesCtrl
				LOG_INF("TxFramesCtrl");
				if (hex_data[1] == 0) {
					// NO-OP
				} else if (hex_data[1] == 1) {
					uplinkType = LORAWAN_MSG_UNCONFIRMED;
				} else if (hex_data[1] == 2) {
					uplinkType = LORAWAN_MSG_CONFIRMED;
					lorawan_set_conf_msg_tries(1);
				}
				break;
			}
			case 0x08: { // EchoPayloadReq
				LOG_INF("EchoPayloadReq");
				uplinkPacket[0] = 0x08;
				for (size_t i = 1; i < len; i++) {
					uplinkPacket[i] = hex_data[i] + 1;
				}
				uplinkSize = len;
				uplinkPort = 224;
				break;
			}
			case 0x09: { // RxAppCntReq
				LOG_INF("RxAppCntReq");
				uplinkPacket[0] = 0x09;

				uplinkPacket[1] = downlinkCounter & 0xFF;
				uplinkPacket[2] = downlinkCounter >> 8;
				uplinkSize = 3;
				uplinkPort = 224;
				break;
			}
			case 0x0A: { // RxAppCntResetReq
				LOG_INF("RxAppCntResetReq");
				downlinkCounter = 0;
				break;
			}
			case 0x20: { // LinkCheckReq
				LOG_INF("LinkCheckReq");
				lorawan_req_link_check();
				break;
			}
			case 0x21: { // DeviceTimeReq
				LOG_INF("DeviceTimeReq");
				lorawan_req_device_time();
				break;
			}
			case 0x22: { // PingSlotInfo
				LOG_INF("PingSlotInfo not implemented");
				// not implemented
				break;
			}
			case 0x7E: { // DutFPort224DisableReq
				cert_test_enabled = false;
				cert_test_joined = false;
				break;
			}
			case 0x7F: { // DutVersionReq
				LOG_INF("DutVersionReq");
				char version[10] = AT_APPLICATION_VERSION;
				int temp = 0;
				uint8_t index = 0;
				uplinkPacket[index++] = 0x7F;
				sscanf(&version[0], "%d", &temp);
				uplinkPacket[index++] = temp; // AT_APP_VERSION_MAJOR; // MAJOR
				sscanf(&version[2], "%d", &temp);
				uplinkPacket[index++] = temp; // AT_APP_VERSION_MINOR; // MINOR
				sscanf(&version[4], "%d", &temp);
				uplinkPacket[index++] = temp; // AT_APP_VERSION_PATCH; // PATCH
				if (strlen(version) > 7) {
					sscanf(&version[6], "%d", &temp);
					uplinkPacket[index++] = temp; // AT_APP_VERSION_PATCH; // PATCH
				} else {
					uplinkPacket[index++] = 0; // AT_APP_VERSION_PATCH; // PATCH
				}
				strncpy(version, LW_VERSION, strlen(LW_VERSION)+1);
				sscanf(&version[0], "%d", &temp);
				uplinkPacket[index++] = temp; // LW_VERSION; // MAJOR
				sscanf(&version[2], "%d", &temp);
				uplinkPacket[index++] = temp; // LW_VERSION; // MINOR
				sscanf(&version[4], "%d", &temp);
				uplinkPacket[index++] = temp; // LW_VERSION; // PATCH
				if (strlen(version) > 7) {
					sscanf(&version[6], "%d", &temp);
					uplinkPacket[index++] = temp; // LW_VERSION; // PATCH
				} else {
					uplinkPacket[index++] = 0; // LW_VERSION; // PATCH
				}
				strncpy(version, RP_VERSION, strlen(RP_VERSION)+1);
				sscanf(&version[0], "%d", &temp);
				uplinkPacket[index++] = temp; // RP_VERSION; // MAJOR
				sscanf(&version[2], "%d", &temp);
				uplinkPacket[index++] = temp; // RP_VERSION; // MINOR
				sscanf(&version[4], "%d", &temp);
				uplinkPacket[index++] = temp; // RP_VERSION; // PATCH
				if (strlen(version) > 7) {
					sscanf(&version[6], "%d", &temp);
					uplinkPacket[index++] = temp; // RP_VERSION; // PATCH
				} else {
					uplinkPacket[index++] = 0; // RP_VERSION; // PATCH
				}
				uplinkSize = index;
				uplinkPort = 224;
				break;
			}
		} 
	}

}

static void lorwan_datarate_changed(enum lorawan_datarate dr)
{
	uint8_t unused, max_size;

	lorawan_get_payload_sizes(&unused, &max_size);
	LOG_INF("New Datarate: DR_%d, Max Payload %d", dr, max_size);
}


// static const struct gpio_dt_spec eeprom_switch =
// 	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(eeprom_0), gpios, {0});

int main(void)
{

	// if (!gpio_is_ready_dt(&eeprom_switch)) {
	// 	printf("The load switch pin GPIO port is not ready.\n");
	// 	return 0;
	// }

	const struct device *lora_dev;

	int ret;

	struct lorawan_downlink_cb downlink_cb = {
		.port = LW_RECV_PORT_ANY,
		.cb = dl_callback
	};

	lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));



/*
	This is working for xDot 1.5 with 24MHz xtal

	MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_ERFO_EN;
	while (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_ERFO_RDY)) {}

	MXC_GCR->clkctrl = (MXC_GCR->clkctrl & ~MXC_F_GCR_CLKCTRL_SYSCLK_SEL) | (MXC_S_GCR_CLKCTRL_SYSCLK_SEL_ERFO);
	while (!(MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_RDY)) {}

	LOG_RAW("\r\nGCR CLK: %08x\r\n", MXC_GCR->clkctrl);
	LOG_RAW("\r\nGCR ERF: %08x\r\n", MXC_GCR->clkctrl | MXC_F_GCR_CLKCTRL_ERFO_EN);
	LOG_RAW("\r\nGCR SEL: %08x\r\n", (MXC_GCR->clkctrl & ~MXC_F_GCR_CLKCTRL_SYSCLK_SEL) | (MXC_S_GCR_CLKCTRL_SYSCLK_SEL_ERFO));
	
*/




	uint32_t last_time = arch_k_cycle_get_32();
	// uint32_t elapsed_time = 0;
	// uint32_t last_ms = (MXC_RTC->sec * 1000 + (MXC_RTC->ssec * 1000 / 0xFFF));
	// uint32_t ssec = 0;


	/* autocalibration did not change accuracy very much */

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


	while (false) {

		LOG_RAW("TIME: %u\r\n", arch_k_cycle_get_32() - last_time);
		// while (!(MXC_RTC->ctrl & MXC_F_RTC_CTRL_RDY)) {}
		// ssec = MXC_RTC->ssec;
		// LOG_RAW("SSEC %04x %d\r\n", ssec, (ssec*1000)/0xFFF);
		LOG_RAW("SLEEP 15s\r\n");
		log_thread_trigger();
		// k_sleep(K_MSEC(10));

		last_time = arch_k_cycle_get_32();
		// while (!(MXC_RTC->ctrl & MXC_F_RTC_CTRL_RDY)) {}
		// last_ms = (MXC_RTC->sec * 1000 + ((MXC_RTC->ssec * 1000) / 0xFFF));
		k_sleep(K_MSEC(15000));

		// LOG_RAW("ELAPSED: %u\r\n", (MXC_RTC->sec * 1000 + (MXC_RTC->ssec * 1000 / 0xFFF))  - last_ms);
		// LOG_RAW("RTC %u %u\r\n", MXC_RTC->sec, MXC_RTC->ssec);
		// LOG_RAW("\r\nTIME: %u\r\n", arch_k_cycle_get_32() - last_time);
		// LOG_RAW("SLEEP 5s\r\n");		
		// last_time = arch_k_cycle_get_32();
		// last_ms = (MXC_RTC->sec * 1000 + (MXC_RTC->ssec * 1000 / 0xFFF));
		// k_sleep(K_MSEC(5000));
		
		// LOG_RAW("ELAPSED: %u\r\n", (MXC_RTC->sec * 1000 + (MXC_RTC->ssec * 1000 / 0xFFF))  - last_ms);
		// LOG_RAW("RTC %u %u\r\n", MXC_RTC->sec, MXC_RTC->ssec);
		// LOG_RAW("\r\nTIME: %u\r\n", arch_k_cycle_get_32() - last_time);
		// LOG_RAW("SLEEP 10s\r\n");		
		// last_time = arch_k_cycle_get_32();
		// last_ms = (MXC_RTC->sec * 1000 + (MXC_RTC->ssec * 1000 / 0xFFF));
		// k_sleep(K_MSEC(10000));

		// LOG_RAW("ELAPSED: %u\r\n", (MXC_RTC->sec * 1000 + (MXC_RTC->ssec * 1000 / 0xFFF))  - last_ms);
		// LOG_RAW("RTC %u %u\r\n", MXC_RTC->sec, MXC_RTC->ssec);
		// LOG_RAW("\r\nTIME: %u\r\n", arch_k_cycle_get_32() - last_time);
		// LOG_RAW("SLEEP 15s\r\n");
		// last_time = arch_k_cycle_get_32();
		// last_ms = (MXC_RTC->sec * 1000 + (MXC_RTC->ssec * 1000 / 0xFFF));
		// k_sleep(K_MSEC(15000));
		// while (!(MXC_RTC->ctrl & MXC_F_RTC_CTRL_RDY)) {}
		// elapsed_time = (MXC_RTC->sec * 1000 + ((MXC_RTC->ssec * 1000) / 0xFFF))  - last_ms;
		// LOG_RAW("ELAPSED: %u\r\n", elapsed_time);
	}



	if (!device_is_ready(lora_dev)) {
		LOG_ERR("%s: device not ready.", lora_dev->name);
		return 0;
	}

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

	ret = lorawan_start();
	lorawan_cert_test_disable_dutycycle();
	if (ret < 0) {
		LOG_ERR("lorawan_start failed: %d", ret);
		return 0;
	}

	lorawan_register_downlink_callback(&downlink_cb);
	lorawan_register_dr_changed_callback(lorwan_datarate_changed);

	lorawan_enable_adr(true);

	uint8_t dev_eui[] = LORAWAN_DEV_EUI;
	uint8_t join_eui[] = LORAWAN_JOIN_EUI;
	uint8_t app_key[] = LORAWAN_APP_KEY;

	join_cfg.mode = LORAWAN_ACT_OTAA;
	join_cfg.dev_eui = dev_eui;
	join_cfg.otaa.join_eui = join_eui;
	join_cfg.otaa.app_key = app_key;
	join_cfg.otaa.nwk_key = app_key;

	// need to save/restore nonce values
	join_cfg.otaa.dev_nonce = 0u;
	// join_cfg.otaa.join_nonce = 0u;

	bool duty_cycle_limited = false;


	while (1) {

		if (!cert_test_joined) {		
			cert_test_join_network();
		} else {
			downlinkReceived = false;

			ret = lorawan_send(uplinkPort, uplinkPacket, uplinkSize, uplinkType);

			/*
			* Note: The stack may return -EAGAIN if the provided data
			* length exceeds the maximum possible one for the region and
			* datarate. But since we are just sending the same data here,
			* we'll just continue.
			*/
			if (ret == -EAGAIN) {
				LOG_ERR("lorawan_send failed: %d. Continuing...", ret);
				k_sleep(DELAY);
				continue;
			}

			if (ret < 0) {
				LOG_ERR("lorawan_send failed: %d", ret);

				if (ret == -111) {
					duty_cycle_limited = true;
				}

				k_sleep(uplinkPeriod);		
			} else {
				duty_cycle_limited = false;
				LOG_INF("Data sent!");
				k_sleep(uplinkPeriod);
			}

			if (!downlinkReceived && !duty_cycle_limited) {
				// reset uplink data unless tx was blocked by duty-cycle
				uplinkPort = 2;
				uplinkPacket[0] = 0xFF;
				uplinkSize = 1;
			}
		}
	}
}
