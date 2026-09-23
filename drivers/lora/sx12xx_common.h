/*
 * Copyright (c) 2020 Andreas Sandberg
 * Copyright (c) 2020 Grinn
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SX12XX_COMMON_H_
#define ZEPHYR_DRIVERS_SX12XX_COMMON_H_

#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/device.h>

int __sx12xx_configure_pin(const struct gpio_dt_spec *gpio, gpio_flags_t flags);

#define sx12xx_configure_pin(_name, _flags)				\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(0, _name##_gpios),		\
		    (__sx12xx_configure_pin(&dev_config._name, _flags)),\
		    (0))

int sx12xx_lora_send(const struct device *dev, uint8_t *data,
		     uint32_t data_len);

int sx12xx_lora_send_async(const struct device *dev, uint8_t *data,
			   uint32_t data_len, struct k_poll_signal *async);

int sx12xx_lora_recv(const struct device *dev, uint8_t *data, uint8_t size,
		     k_timeout_t timeout, int16_t *rssi, int8_t *snr);

int sx12xx_lora_recv_async(const struct device *dev, lora_recv_cb cb, void *user_data);

#if defined(CONFIG_LORA_SX126X)
/*
 * A receive whose EMPTY end is decided by the radio, not by a kernel timer.
 *
 * The radio is woken and configured first; then `arm(arm_ctx)` is called, which
 * waits for the instant the window must open and returns how long the radio
 * should listen, and SetRx is issued at once. That time is programmed into
 * the SX126x (SetRx, 15.625 us RTC steps) with the RX timer stopped on
 * preamble detection, so an empty
 * window ends at the radio's timeout (-ETIMEDOUT) and a detected frame is
 * received to its end. `backstop` bounds the whole call for a frame whose
 * preamble was detected (-EAGAIN), and should be sized for the longest frame.
 *
 * Declared here and not in <zephyr/drivers/lora.h> deliberately: the public
 * API has no receive-window concept, and this is the one caller's need.
 */
typedef uint32_t (*sx12xx_rx_arm_t)(void *ctx);

int sx12xx_lora_recv_timed(const struct device *dev, uint8_t *data, uint8_t size,
			   sx12xx_rx_arm_t arm, void *arm_ctx, k_timeout_t backstop,
			   int16_t *rssi, int8_t *snr);
#endif

#if defined(CONFIG_LORA_SX126X)
/*
 * LoRaWAN GFSK (e.g. AS923/EU868 DR7). Selects the FSK modem for one
 * direction; lora_config() selects LoRa again. `bandwidth` and
 * `bandwidth_afc` are single-sided, in Hz; `preamble_len` is in bytes.
 */
struct sx12xx_fsk_config {
	uint32_t frequency;
	uint32_t bitrate;
	uint32_t fdev;
	uint32_t bandwidth;
	uint32_t bandwidth_afc;
	uint16_t preamble_len;
	int8_t tx_power;
	bool tx;
};

int sx12xx_fsk_config(const struct device *dev,
		      const struct sx12xx_fsk_config *config);
#endif

int sx12xx_lora_config(const struct device *dev,
		       struct lora_modem_config *config);

int sx12xx_lora_test_cw(const struct device *dev, uint32_t frequency,
			int8_t tx_power,
			uint16_t duration);

int sx12xx_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_SX12XX_COMMON_H_ */
