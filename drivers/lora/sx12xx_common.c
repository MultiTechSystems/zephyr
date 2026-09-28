/*
 * Copyright (c) 2019 Manivannan Sadhasivam
 * Copyright (c) 2020 Grinn
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/kernel.h>

/* LoRaMac-node specific includes */
#include <radio.h>
#if defined(CONFIG_LORA_SX126X)
#include <sx126x/sx126x.h>
#endif

#include "sx12xx_common.h"

#define STATE_FREE      0
#define STATE_BUSY      1
#define STATE_CLEANUP   2

LOG_MODULE_REGISTER(sx12xx_common, CONFIG_LORA_LOG_LEVEL);

struct sx12xx_rx_params {
	uint8_t *buf;
	uint8_t *size;
	int16_t *rssi;
	int8_t *snr;
};

static struct sx12xx_data {
	const struct device *dev;
	struct k_poll_signal *operation_done;
	lora_recv_cb async_rx_cb;
	void *async_user_data;
	RadioEvents_t events;
	struct lora_modem_config tx_cfg;
	/* The last LoRa receive configuration, so a timed receive can re-apply
	 * it in single mode and restore it continuous afterwards (SX127x).
	 */
	struct lora_modem_config rx_cfg;
	/* Which modem the last configuration selected, per direction. Every
	 * path below that re-arms the radio names a modem, and naming LoRa
	 * while the part is in GFSK rewrites its packet parameters as LoRa.
	 */
	RadioModems_t rx_modem;
	RadioModems_t tx_modem;
	uint32_t fsk_tx_bitrate;
	uint16_t fsk_tx_preamble;
	atomic_t modem_usage;
	struct sx12xx_rx_params rx_params;
} dev_data;

int __sx12xx_configure_pin(const struct gpio_dt_spec *gpio, gpio_flags_t flags)
{
	int err;

	if (!device_is_ready(gpio->port)) {
		LOG_ERR("GPIO device not ready %s", gpio->port->name);
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(gpio, flags);
	if (err) {
		LOG_ERR("Cannot configure gpio %s %d: %d", gpio->port->name,
			gpio->pin, err);
		return err;
	}

	return 0;
}

/**
 * @brief Attempt to acquire the modem for operations
 *
 * @param data common sx12xx data struct
 *
 * @retval true if modem was acquired
 * @retval false otherwise
 */
static inline bool modem_acquire(struct sx12xx_data *data)
{
	return atomic_cas(&data->modem_usage, STATE_FREE, STATE_BUSY);
}

/**
 * @brief Safely release the modem from any context
 *
 * This function can be called from any context and guarantees that the
 * release operations will only be run once.
 *
 * @param data common sx12xx data struct
 *
 * @retval true if modem was released by this function
 * @retval false otherwise
 */
static bool modem_release(struct sx12xx_data *data)
{
	/* Increment atomic so both acquire and release will fail */
	if (!atomic_cas(&data->modem_usage, STATE_BUSY, STATE_CLEANUP)) {
		return false;
	}
	/* Put radio back into sleep mode */
	Radio.Sleep();
	/* Completely release modem */
	data->operation_done = NULL;
	atomic_clear(&data->modem_usage);
	return true;
}

static void sx12xx_ev_rx_done(uint8_t *payload, uint16_t size, int16_t rssi,
			      int8_t snr)
{
	struct k_poll_signal *sig = dev_data.operation_done;

	/* Receiving in asynchronous mode */
	if (dev_data.async_rx_cb) {
		/* Start receiving again */
		Radio.Rx(0);
		/* Run the callback */
		dev_data.async_rx_cb(dev_data.dev, payload, size, rssi, snr,
				   dev_data.async_user_data);
		/* Don't run the synchronous code */
		return;
	}

	/* Manually release the modem instead of just calling modem_release
	 * as we need to perform cleanup operations while still ensuring
	 * others can't use the modem.
	 */
	if (!atomic_cas(&dev_data.modem_usage, STATE_BUSY, STATE_CLEANUP)) {
		return;
	}
	/* We can make two observations here:
	 *  1. lora_recv hasn't already exited due to a timeout.
	 *         (modem_release would have been successfully called)
	 *  2. If the k_poll in lora_recv times out before we raise the signal,
	 *     but while this code is running, it will block on the
	 *     signal again.
	 * This lets us guarantee that the operation_done signal and pointers
	 * in rx_params are always valid in this function.
	 */

	/* Store actual size */
	if (size < *dev_data.rx_params.size) {
		*dev_data.rx_params.size = size;
	}
	/* Copy received data to output buffer */
	memcpy(dev_data.rx_params.buf, payload,
	       *dev_data.rx_params.size);
	/* Output RSSI and SNR */
	if (dev_data.rx_params.rssi) {
		*dev_data.rx_params.rssi = rssi;
	}
	if (dev_data.rx_params.snr) {
		*dev_data.rx_params.snr = snr;
	}
	/* Put radio back into sleep mode */
	Radio.Sleep();
	/* Completely release modem */
	dev_data.operation_done = NULL;
	atomic_clear(&dev_data.modem_usage);
	/* Notify caller RX is complete */
	k_poll_signal_raise(sig, 0);
}

static void sx12xx_ev_tx_done(void)
{
	struct k_poll_signal *sig = dev_data.operation_done;

	if (modem_release(&dev_data)) {
		/* Raise signal if provided */
		if (sig) {
			k_poll_signal_raise(sig, 0);
		}
	}
}

static void sx12xx_ev_tx_timed_out(void)
{
	/* Just release the modem */
	modem_release(&dev_data);
}

static void sx12xx_ev_rx_error(void)
{
	struct k_poll_signal *sig = dev_data.operation_done;

	/* Receiving in asynchronous mode */
	if (dev_data.async_rx_cb) {
		/* Start receiving again */
		Radio.Rx(0);
		/* Don't run the synchronous code */
		return;
	}

	/* Finish synchronous receive with error */
	if (modem_release(&dev_data)) {
		/* Raise signal if provided */
		if (sig) {
			k_poll_signal_raise(sig, -EIO);
		}
	}
}

/*
 * The radio's own RX timeout, and a header error, both arrive here. Neither
 * happens on a continuous receive (SetRx 0xFFFFFF has no timer), so only the
 * timed synchronous receive can see one.
 */
static void sx12xx_ev_rx_timeout(void)
{
	struct k_poll_signal *sig = dev_data.operation_done;

	if (dev_data.async_rx_cb) {
		return;
	}

	if (modem_release(&dev_data)) {
		if (sig) {
			k_poll_signal_raise(sig, -ETIMEDOUT);
		}
	}
}

int sx12xx_lora_send(const struct device *dev, uint8_t *data,
		     uint32_t data_len)
{
	struct k_poll_signal done = K_POLL_SIGNAL_INITIALIZER(done);
	struct k_poll_event evt = K_POLL_EVENT_INITIALIZER(
		K_POLL_TYPE_SIGNAL,
		K_POLL_MODE_NOTIFY_ONLY,
		&done);
	uint32_t air_time;
	int ret;

	/* Validate that we have a TX configuration */
	if (!dev_data.tx_cfg.frequency) {
		return -EINVAL;
	}

	ret = sx12xx_lora_send_async(dev, data, data_len, &done);
	if (ret < 0) {
		return ret;
	}

	/* Calculate expected airtime of the packet */
	if (dev_data.tx_modem == MODEM_FSK) {
		air_time = Radio.TimeOnAir(MODEM_FSK, 0, dev_data.fsk_tx_bitrate,
					   0, dev_data.fsk_tx_preamble,
					   false, data_len, true);
	} else {
		air_time = Radio.TimeOnAir(MODEM_LORA,
					   dev_data.tx_cfg.bandwidth,
					   dev_data.tx_cfg.datarate,
					   dev_data.tx_cfg.coding_rate,
					   dev_data.tx_cfg.preamble_len,
					   0, data_len, true);
	}
	LOG_DBG("Expected air time of %d bytes = %dms", data_len, air_time);

	/* Wait for the packet to finish transmitting.
	 * Use twice the tx duration to ensure that we are actually detecting
	 * a failed transmission, and not some minor timing variation between
	 * modem and driver.
	 */
	ret = k_poll(&evt, 1, K_MSEC(2 * air_time));
	if (ret < 0) {
		LOG_ERR("Packet transmission failed!");
		if (!modem_release(&dev_data)) {
			/* TX done interrupt is currently running */
			k_poll(&evt, 1, K_FOREVER);
		}
	}
	return ret;
}

int sx12xx_lora_send_async(const struct device *dev, uint8_t *data,
			   uint32_t data_len, struct k_poll_signal *async)
{
	/* Ensure available, freed by sx12xx_ev_tx_done */
	if (!modem_acquire(&dev_data)) {
		return -EBUSY;
	}

	/* Store signal */
	dev_data.operation_done = async;

	Radio.SetMaxPayloadLength(dev_data.tx_modem, data_len);

	Radio.Send(data, data_len);

	return 0;
}

int sx12xx_lora_recv(const struct device *dev, uint8_t *data, uint8_t size,
		     k_timeout_t timeout, int16_t *rssi, int8_t *snr)
{
	struct k_poll_signal done = K_POLL_SIGNAL_INITIALIZER(done);
	struct k_poll_event evt = K_POLL_EVENT_INITIALIZER(
		K_POLL_TYPE_SIGNAL,
		K_POLL_MODE_NOTIFY_ONLY,
		&done);
	int ret;

	/* Ensure available, decremented by sx12xx_ev_rx_done or on timeout */
	if (!modem_acquire(&dev_data)) {
		return -EBUSY;
	}

	dev_data.async_rx_cb = NULL;
	/* Store operation signal */
	dev_data.operation_done = &done;
	/* Set data output location */
	dev_data.rx_params.buf = data;
	dev_data.rx_params.size = &size;
	dev_data.rx_params.rssi = rssi;
	dev_data.rx_params.snr = snr;

	Radio.SetMaxPayloadLength(dev_data.rx_modem, 255);
	Radio.Rx(0);

	ret = k_poll(&evt, 1, timeout);
	if (ret < 0) {
		if (!modem_release(&dev_data)) {
			/* Releasing the modem failed, which means that
			 * the RX callback is currently running. Wait until
			 * the RX callback finishes and we get our packet.
			 */
			k_poll(&evt, 1, K_FOREVER);

			/* We did receive a packet */
			return size;
		}
		LOG_INF("Receive timeout");
		return ret;
	}

	if (done.result < 0) {
		LOG_ERR("Receive error");
		return done.result;
	}

	return size;
}

#if defined(CONFIG_LORA_SX126X)
int sx12xx_lora_recv_timed(const struct device *dev, uint8_t *data, uint8_t size,
			   sx12xx_rx_arm_t arm, void *arm_ctx, k_timeout_t backstop,
			   int16_t *rssi, int8_t *snr)
{
	struct k_poll_signal done = K_POLL_SIGNAL_INITIALIZER(done);
	struct k_poll_event evt = K_POLL_EVENT_INITIALIZER(
		K_POLL_TYPE_SIGNAL,
		K_POLL_MODE_NOTIFY_ONLY,
		&done);
	uint64_t steps;
	int ret;

	if (!modem_acquire(&dev_data)) {
		return -EBUSY;
	}

	dev_data.async_rx_cb = NULL;
	dev_data.operation_done = &done;
	dev_data.rx_params.buf = data;
	dev_data.rx_params.size = &size;
	dev_data.rx_params.rssi = rssi;
	dev_data.rx_params.snr = snr;

	Radio.SetMaxPayloadLength(dev_data.rx_modem, 255);
	/* What RadioRx() does, except the timeout: RadioRx() can only program
	 * 0xFFFFFF (continuous) or a whole-millisecond software timer.
	 */
	SX126xSetDioIrqParams(IRQ_RADIO_ALL, IRQ_RADIO_ALL,
			      IRQ_RADIO_NONE, IRQ_RADIO_NONE);
	SX126xSetStopRxTimerOnPreambleDetect(true);
	/* Everything above may wake the part from sleep and waits on BUSY with
	 * 1 ms sleeps, so it costs milliseconds -- measured as a window opening
	 * ~5-10 ms late on an xDot ES. Only SetRx is left to issue: the caller
	 * waits for its instant here and says how long the radio should listen,
	 * so nothing sleeps between that instant and the command.
	 * 15.625 us per step = 1000/64 us; 0 would mean single-shot with no
	 * timeout and 0xFFFFFF continuous, so neither may be produced.
	 */
	steps = DIV_ROUND_UP((uint64_t)arm(arm_ctx) * 64U, 1000U);
	steps = CLAMP(steps, 1U, 0xFFFFFEU);
	SX126xSetRx((uint32_t)steps);

	ret = k_poll(&evt, 1, backstop);
	if (ret < 0) {
		if (!modem_release(&dev_data)) {
			/* An RX event is being handled; wait for its result. */
			k_poll(&evt, 1, K_FOREVER);
			return (done.result < 0) ? done.result : size;
		}
		return -EAGAIN;
	}

	if (done.result < 0) {
		return done.result;
	}

	return size;
}
#endif

#if defined(CONFIG_LORA_SX127X)
__weak void sx12xx_rx_armed_hook(void)
{
}

/* The SX127x symbol timeout: RegModemConfig2 bits 1:0 (MSB) and
 * RegSymbTimeoutLsb, 10 bits of symbols. The same registers on SX1272 and
 * SX1276 (loramac-node sx127{2,6}Regs-LoRa.h).
 */
#define SX127X_REG_LR_MODEMCONFIG2   0x1E
#define SX127X_REG_LR_SYMBTIMEOUTLSB 0x1F
#define SX127X_REG_LR_IRQFLAGS       0x12
#define SX127X_REG_DIOMAPPING1       0x40
#define SX127X_IRQ_VALIDHEADER       0x10
#define SX127X_DIO3_MASK             0x30
#define SX127X_DIO3_VALIDHEADER      0x10

static const uint16_t sx127x_bw_khz[] = { [BW_125_KHZ] = 125, [BW_250_KHZ] = 250,
					  [BW_500_KHZ] = 500 };

static void sx127x_apply_rx_cfg(uint16_t symb_timeout, bool continuous)
{
	const struct lora_modem_config *c = &dev_data.rx_cfg;

	Radio.SetRxConfig(MODEM_LORA, c->bandwidth, c->datarate, c->coding_rate,
			  0, c->preamble_len, symb_timeout,
			  c->implicit_len != 0, c->implicit_len,
			  false, 0, 0, c->iq_inverted, continuous);
}

int sx12xx_lora_recv_timed(const struct device *dev, uint8_t *data, uint8_t size,
			   sx12xx_rx_arm_t arm, void *arm_ctx, k_timeout_t backstop,
			   int16_t *rssi, int8_t *snr)
{
	struct k_poll_signal done = K_POLL_SIGNAL_INITIALIZER(done);
	struct k_poll_event evt = K_POLL_EVENT_INITIALIZER(
		K_POLL_TYPE_SIGNAL,
		K_POLL_MODE_NOTIFY_ONLY,
		&done);
	uint32_t listen_us;
	uint32_t sym_us;
	uint32_t symbols;
	int ret;

	if (dev_data.rx_modem != MODEM_LORA || dev_data.rx_cfg.frequency == 0 ||
	    dev_data.rx_cfg.bandwidth > BW_500_KHZ) {
		return -EINVAL;
	}
	if (!modem_acquire(&dev_data)) {
		return -EBUSY;
	}

	dev_data.async_rx_cb = NULL;
	dev_data.operation_done = &done;
	dev_data.rx_params.buf = data;
	dev_data.rx_params.size = &size;
	dev_data.rx_params.rssi = rssi;
	dev_data.rx_params.snr = snr;

	/* SINGLE mode, so the radio ends an empty window itself with RxTimeout
	 * (DIO1) and a detected preamble is received to its end. Everything that
	 * costs SPI time happens here, before the caller's instant; the symbol
	 * timeout is written after it, from the duration arm() returns.
	 */
	Radio.SetMaxPayloadLength(dev_data.rx_modem, 255);
	sx127x_apply_rx_cfg(1023, false);

	listen_us = arm(arm_ctx);
	sym_us = (uint32_t)(((1U << dev_data.rx_cfg.datarate) * 1000U) /
			    sx127x_bw_khz[dev_data.rx_cfg.bandwidth]);
	symbols = CLAMP(DIV_ROUND_UP(listen_us, MAX(sym_us, 1U)), 4U, 1023U);
	Radio.Write(SX127X_REG_LR_MODEMCONFIG2,
		    (Radio.Read(SX127X_REG_LR_MODEMCONFIG2) & ~0x03U) |
		    ((symbols >> 8) & 0x03U));
	Radio.Write(SX127X_REG_LR_SYMBTIMEOUTLSB, (uint8_t)(symbols & 0xFFU));
	/* DIO3 AS VALIDHEADER, for sx127x_dio_isr_hook(3): the instant the
	 * explicit header is in, a fixed 8 + 4.25 + 8 symbols after the frame
	 * began and independent of the payload -- where RxDone trails the last
	 * symbol by an amount that is not documented. loramac-node's SetRx
	 * rewrites only DIO0/DIO2, and its DIO3 handler (CAD) clears only
	 * CADDONE, so the flag is cleared here or the line stays high and the
	 * next header gives no edge. No CadDone callback is registered, so the
	 * handler's CadDone(false) is a no-op. Implicit-header frames (beacons)
	 * raise no ValidHeader. */
	Radio.Write(SX127X_REG_LR_IRQFLAGS, SX127X_IRQ_VALIDHEADER);
	Radio.Write(SX127X_REG_DIOMAPPING1,
		    (Radio.Read(SX127X_REG_DIOMAPPING1) & ~SX127X_DIO3_MASK) |
		    SX127X_DIO3_VALIDHEADER);
	Radio.Rx(0);
	sx12xx_rx_armed_hook();

	ret = k_poll(&evt, 1, backstop);
	if (ret < 0) {
		if (!modem_release(&dev_data)) {
			/* An RX event is being handled; wait for its result. */
			k_poll(&evt, 1, K_FOREVER);
			ret = (done.result < 0) ? done.result : size;
		} else {
			ret = -EAGAIN;
		}
	} else {
		ret = (done.result < 0) ? done.result : size;
	}

	/* Leave the part as lora_config left it: continuous, symbol timeout 0,
	 * so a following lora_recv/lora_recv_async is not a single receive.
	 */
	sx127x_apply_rx_cfg(0, true);
	return ret;
}
#endif

int sx12xx_lora_recv_async(const struct device *dev, lora_recv_cb cb, void *user_data)
{
	/* Cancel ongoing reception */
	if (cb == NULL) {
		if (!modem_release(&dev_data)) {
			/* Not receiving or already being stopped */
			return -EINVAL;
		}
		return 0;
	}

	/* Ensure available */
	if (!modem_acquire(&dev_data)) {
		return -EBUSY;
	}

	/* Store parameters */
	dev_data.async_rx_cb = cb;
	dev_data.async_user_data = user_data;

	/* Start reception */
	Radio.SetMaxPayloadLength(dev_data.rx_modem, 255);
	Radio.Rx(0);

	return 0;
}

int sx12xx_lora_config(const struct device *dev,
		       struct lora_modem_config *config)
{
	/* Ensure available, decremented after configuration */
	if (!modem_acquire(&dev_data)) {
		return -EBUSY;
	}

	Radio.SetChannel(config->frequency);

	if (config->tx) {
		dev_data.tx_modem = MODEM_LORA;
		/* Store TX config locally for airtime calculations */
		memcpy(&dev_data.tx_cfg, config, sizeof(dev_data.tx_cfg));
		/* Configure radio driver */
		Radio.SetTxConfig(MODEM_LORA, config->tx_power, 0,
				  config->bandwidth, config->datarate,
				  config->coding_rate, config->preamble_len,
				  false, true, 0, 0, config->iq_inverted, 4000);
	} else {
		dev_data.rx_modem = MODEM_LORA;
		memcpy(&dev_data.rx_cfg, config, sizeof(dev_data.rx_cfg));
		/* SYMBOL TIMEOUT 0 FOR A CONTINUOUS WINDOW, not 10.
		 *
		 * `Radio.Rx(0)` below is a CONTINUOUS receive, and
		 * SX126xSetLoRaSymbNumTimeout(10) asks the part to validate a
		 * preamble over ten symbols when LoRaWAN transmits eight. MTS's
		 * driver zeroes it explicitly for exactly this case --
		 * SxRadio1262::SetRxConfig: `if (rx_continuous) { symb_timeout
		 * = 0; }` -- and that driver demodulates SF10BW500 on this
		 * board where this one does not (12/13 against 0/8, same
		 * gateway, same hour).
		 *
		 * The old value arrived with a `TODO: Get symbol timeout value
		 * from config parameters` and the Zephyr LoRa API still has no
		 * field for it, so 0 is the only value that is correct for
		 * every caller of a continuous receive. */
		/* fixLen/payloadLen from implicit_len: 0 keeps the explicit
		 * header; N receives an implicit-header frame of N octets (a
		 * Class B beacon). crcOn stays false either way -- LoRaWAN
		 * downlinks and beacons carry no PHY payload CRC. */
		Radio.SetRxConfig(MODEM_LORA, config->bandwidth,
				  config->datarate, config->coding_rate,
				  0, config->preamble_len, 0,
				  config->implicit_len != 0, config->implicit_len,
				  false, 0, 0, config->iq_inverted, true);
	}

	Radio.SetPublicNetwork(config->public_network);

	modem_release(&dev_data);
	return 0;
}

#if defined(CONFIG_LORA_SX126X)
int sx12xx_fsk_config(const struct device *dev,
		      const struct sx12xx_fsk_config *config)
{
	/* LoRaWAN GFSK is loramac-node's own MODEM_FSK branch, unchanged: sync
	 * word C1 94 C1, whitening seed 0x01FF, CRC-16 CCITT, variable length,
	 * Gaussian BT 1.0, preamble in bytes. It matches what the SX1302 HAL
	 * transmits. The Zephyr LoRa API only ever reached the LoRa branch, so
	 * this is the way in; after it the ordinary send/recv calls work,
	 * because they now name the modem configured here.
	 */
	if (!modem_acquire(&dev_data)) {
		return -EBUSY;
	}

	Radio.SetChannel(config->frequency);

	if (config->tx) {
		dev_data.tx_modem = MODEM_FSK;
		dev_data.fsk_tx_bitrate = config->bitrate;
		dev_data.fsk_tx_preamble = config->preamble_len;
		/* tx_cfg.frequency is what lora_send checks for "configured" */
		dev_data.tx_cfg.frequency = config->frequency;
		Radio.SetTxConfig(MODEM_FSK, config->tx_power, config->fdev, 0,
				  config->bitrate, 0, config->preamble_len,
				  false, true, false, 0, false, 4000);
	} else {
		dev_data.rx_modem = MODEM_FSK;
		/* `bandwidth` is single-sided; radio.c doubles it for the part.
		 * Continuous and symbol timeout 0, as the LoRa path: a timed
		 * receive bounds the window through SetRx instead.
		 */
		Radio.SetRxConfig(MODEM_FSK, config->bandwidth, config->bitrate,
				  0, config->bandwidth_afc, config->preamble_len,
				  0, false, 0, true, false, 0, false, true);
	}

	modem_release(&dev_data);
	return 0;
}
#endif

int sx12xx_lora_test_cw(const struct device *dev, uint32_t frequency,
			int8_t tx_power,
			uint16_t duration)
{
	/* Ensure available, freed in sx12xx_ev_tx_done */
	if (!modem_acquire(&dev_data)) {
		return -EBUSY;
	}

	Radio.SetTxContinuousWave(frequency, tx_power, duration);
	return 0;
}

int sx12xx_init(const struct device *dev)
{
	atomic_set(&dev_data.modem_usage, 0);
	/* MODEM_FSK is 0, so a zeroed dev_data would name FSK */
	dev_data.rx_modem = MODEM_LORA;
	dev_data.tx_modem = MODEM_LORA;

	dev_data.dev = dev;
	dev_data.events.TxDone = sx12xx_ev_tx_done;
	dev_data.events.RxDone = sx12xx_ev_rx_done;
	dev_data.events.RxError = sx12xx_ev_rx_error;
	dev_data.events.RxTimeout = sx12xx_ev_rx_timeout;
	/* TX timeout event raises at the end of the test CW transmission */
	dev_data.events.TxTimeout = sx12xx_ev_tx_timed_out;
	Radio.Init(&dev_data.events);

	/*
	 * Automatically place the radio into sleep mode upon boot.
	 * The required `lora_config` call before transmission or reception
	 * will bring the radio out of sleep mode before it is used. The radio
	 * is automatically placed back into sleep mode upon TX or RX
	 * completion.
	 */
	Radio.Sleep();

	return 0;
}
