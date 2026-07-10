/*
	Copyright 2026 Lukas Hrazky

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	*/

#include "transport_spi_hw.h"
#include <string.h>

// SPI read sets bit 7 of the register address, write clears it.
#define SPI_READ_BIT 0x80

#define SPI_DATASIZE_8BIT	0
#define SPI_MODE_0			0
#define SPI_MODE_1			SPI_CR1_CPHA
#define SPI_MODE_2			SPI_CR1_CPOL
#define SPI_MODE_3			(SPI_CR1_CPOL | SPI_CR1_CPHA)

// Default IMU SPI clock used when bus_hz is 0.
#define SPI_HW_DEFAULT_HZ	10500000

// A maximum register transaction is 17 bytes (about 13 us at 10.5 MHz).
// Two milliseconds leaves ample scheduling margin without allowing a lost
// DMA completion to hang IMU configuration or recovery indefinitely.
#define SPI_SYNC_TIMEOUT	MS2ST(2)

static SPIDriver *dev_of(transport_t *t) {
	return t->bus.spi_hw.spid;
}

static const transport_interface_t spi_hw_interface;

static void async_notify(transport_t *t, bool error) {
	if (t->bus.spi_hw.async_done_cb) {
		t->bus.spi_hw.async_done_cb(t->bus.spi_hw.async_done_arg, error);
	}
}

static void spi_end_cb(SPIDriver *spip) {
	transport_t *t = (transport_t *)spip->app_arg;
	if (t == NULL) {
		return;
	}

	palSetPad(t->bus.spi_hw.cfg.ssport, t->bus.spi_hw.cfg.sspad);
	if (t->bus.spi_hw.async_active) {
		t->bus.spi_hw.async_complete = true;
		// EXTI has a higher priority than the SPI DMA ISR. Publish complete
		// before clearing active so a nested DRDY can never observe false/false.
		t->bus.spi_hw.async_active = false;
		async_notify(t, t->bus.spi_hw.async_error);
	} else if (t->bus.spi_hw.sync_active) {
		t->bus.spi_hw.sync_active = false;
		chSysLockFromISR();
		chBSemSignalI(&t->bus.spi_hw.sync_sem);
		chSysUnlockFromISR();
	}
}

static void spi_err_cb(SPIDriver *spip) {
	transport_t *t = (transport_t *)spip->app_arg;
	if (t == NULL) {
		return;
	}

	if (t->bus.spi_hw.async_active) {
		t->bus.spi_hw.async_error = true;
		t->bus.spi_hw.async_dma_errors++;
	} else if (t->bus.spi_hw.sync_active) {
		t->bus.spi_hw.sync_error = true;
		t->bus.spi_hw.sync_dma_errors++;
	}
}

// The SPI mutex serializes thread callers, while thread_owned extends that
// ownership to the DRDY ISR, which intentionally bypasses the mutex.
static bool claim_thread_bus(transport_t *t) {
	SPIDriver *spid = dev_of(t);
	spiAcquireBus(spid);

	chSysLock();
	bool claimed = !t->bus.spi_hw.thread_owned && !t->bus.spi_hw.async_active &&
			!t->bus.spi_hw.async_complete && spid->state == SPI_READY;
	if (claimed) {
		t->bus.spi_hw.thread_owned = true;
	}
	chSysUnlock();

	if (!claimed) {
		spiReleaseBus(spid);
	}
	return claimed;
}

static void release_thread_bus(transport_t *t) {
	chSysLock();
	t->bus.spi_hw.thread_owned = false;
	chSysUnlock();
	spiReleaseBus(dev_of(t));
}

// Reset the peripheral while thread_owned and the SPI mutex are held. The
// ownership barrier remains asserted across spiStop/spiStart, and spiStart's
// DMA allocation clears any pending NVIC state before another launch.
static void reset_spi_while_owned(transport_t *t) {
	SPIDriver *spid = dev_of(t);
	t->bus.spi_hw.spi_resets++;

	chSysLock();
	if (spid->state != SPI_READY && spid->state != SPI_STOP) {
		dmaStreamDisable(spid->dmatx);
		dmaStreamDisable(spid->dmarx);
		spid->spi->CR1 = 0;
		spid->spi->CR2 = 0;
		spid->state = SPI_READY;
	}
	t->bus.spi_hw.async_active = false;
	t->bus.spi_hw.async_complete = false;
	t->bus.spi_hw.async_error = false;
	t->bus.spi_hw.sync_active = false;
	t->bus.spi_hw.sync_error = false;
	chSysUnlock();

	palSetPad(t->bus.spi_hw.cfg.ssport, t->bus.spi_hw.cfg.sspad);
	if (spid->state != SPI_STOP) {
		spiStop(spid);
	}
	spiStart(spid, &t->bus.spi_hw.cfg);
	spid->err_cb = spi_err_cb;
	spid->app_arg = t;
}

static bool start_sync_transfer(transport_t *t, size_t len, bool exchange) {
	SPIDriver *spid = dev_of(t);
	chBSemReset(&t->bus.spi_hw.sync_sem, true);
	t->bus.spi_hw.sync_error = false;

	// Clear a stale RXNE state before arming DMA, matching the proven G18 path.
	volatile uint32_t rxne_clear = spid->spi->DR;
	(void)rxne_clear;
	palClearPad(t->bus.spi_hw.cfg.ssport, t->bus.spi_hw.cfg.sspad);

	chSysLock();
	bool started = t->bus.spi_hw.thread_owned && !t->bus.spi_hw.async_active &&
			!t->bus.spi_hw.async_complete && spid->state == SPI_READY;
	if (started) {
		t->bus.spi_hw.sync_active = true;
		if (exchange) {
			spiStartExchangeI(spid, len, t->bus.spi_hw.txd, t->bus.spi_hw.rxd);
		} else {
			spiStartSendI(spid, len, t->bus.spi_hw.txd);
		}
	}
	chSysUnlock();

	if (!started) {
		palSetPad(t->bus.spi_hw.cfg.ssport, t->bus.spi_hw.cfg.sspad);
		return false;
	}

	msg_t msg = chBSemWaitTimeout(&t->bus.spi_hw.sync_sem, SPI_SYNC_TIMEOUT);
	if (msg != MSG_OK) {
		t->bus.spi_hw.sync_timeouts++;
		reset_spi_while_owned(t);
		return false;
	}
	return !t->bus.spi_hw.sync_error;
}

static bool read_reg(transport_t *t, uint8_t dev_addr, uint8_t reg, uint8_t *rx, size_t len) {
	(void)dev_addr;
	if (rx == NULL || len == 0 || len > IMU_MAX_BURST || !claim_thread_bus(t)) {
		return false;
	}
	uint8_t *txd = t->bus.spi_hw.txd;
	uint8_t *rxd = t->bus.spi_hw.rxd;

	txd[0] = reg | SPI_READ_BIT;
	memset(txd + 1, 0, len);

	// One full-duplex exchange (address byte + len bytes read back), not a separate
	// send-then-receive. Use the same callback-driven DMA path as asynchronous
	// sampling because ChibiOS forbids spiExchange() when end_cb is configured.
	bool ok = start_sync_transfer(t, 1 + len, true);
	if (ok) {
		memcpy(rx, rxd + 1, len);
	}
	release_thread_bus(t);
	return ok;
}

static bool write_reg(transport_t *t, uint8_t dev_addr, uint8_t reg, const uint8_t *tx, size_t len) {
	(void)dev_addr;
	if (tx == NULL || len == 0 || len > IMU_MAX_BURST || !claim_thread_bus(t)) {
		return false;
	}
	uint8_t *txd = t->bus.spi_hw.txd;

	txd[0] = reg & ~SPI_READ_BIT;
	memcpy(txd + 1, tx, len);

	bool ok = start_sync_transfer(t, 1 + len, false);
	release_thread_bus(t);
	return ok;
}

static uint16_t max_sample_rate(transport_t *t) {
	(void)t;
	return 10000;
}

static void recover(transport_t *t) {
	transport_spi_hw_async_abort(t);
}

static void deinit(transport_t *t) {
	if (!transport_spi_hw_async_supported(t)) {
		return;
	}

	transport_spi_hw_async_abort(t);
	SPIDriver *spid = dev_of(t);
	spid->err_cb = NULL;
	spid->app_arg = NULL;
	if (spid->state != SPI_STOP) {
		spiStop(spid);
	}
}

static const transport_interface_t spi_hw_interface = {
	.name = "spi-hw",
	.max_sample_rate = max_sample_rate,
	.read_reg = read_reg,
	.write_reg = write_reg,
	.recover = recover,
	.deinit = deinit,
};

// CR1.BR field value (0-7) for the fastest prescaler whose SPI clock does not
// exceed bus_hz (0 = SPI_HW_DEFAULT_HZ). SPI clock = pclk / 2^(br + 1).
static uint8_t hz_to_cr1br(SPIDriver *spid, uint32_t bus_hz) {
	// SPI1 is clocked from PCLK2, every other SPI from PCLK1.
	uint32_t pclk = STM32_PCLK1;
#if STM32_SPI_USE_SPI1
	if (spid == &SPID1) {
		pclk = STM32_PCLK2;
	}
#else
	(void)spid;
#endif

	if (bus_hz == 0) {
		bus_hz = SPI_HW_DEFAULT_HZ;
	}

	for (uint8_t br = 0; br < 7; br++) {
		if ((pclk >> (br + 1)) <= bus_hz) {
			return br;
		}
	}

	return 7; // equals prescaler /256
}

void transport_spi_hw_init(transport_t *t, SPIDriver *spid, uint32_t af,
		stm32_gpio_t *nss_gpio, uint8_t nss_pin, stm32_gpio_t *sck_gpio, uint8_t sck_pin,
		stm32_gpio_t *mosi_gpio, uint8_t mosi_pin, stm32_gpio_t *miso_gpio, uint8_t miso_pin,
		uint32_t bus_hz) {
	memset(&t->bus.spi_hw, 0, sizeof(t->bus.spi_hw));
	t->interface = &spi_hw_interface;
	t->bus.spi_hw.spid = spid;
	chBSemObjectInit(&t->bus.spi_hw.sync_sem, true);

	// Preload the inactive level before changing the pin to an output. This
	// prevents a low NSS pulse while the sensor and SPI peripheral initialize.
	palSetPad(nss_gpio, nss_pin);
	palSetPadMode(nss_gpio, nss_pin,
			PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(sck_gpio, sck_pin,
			PAL_MODE_ALTERNATE(af) | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(mosi_gpio, mosi_pin,
			PAL_MODE_ALTERNATE(af) | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(miso_gpio, miso_pin,
			PAL_MODE_ALTERNATE(af) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);

	t->bus.spi_hw.cfg = (SPIConfig){
		spi_end_cb, nss_gpio, nss_pin,
		(uint16_t)((hz_to_cr1br(spid, bus_hz) << 3) | SPI_MODE_3 | SPI_DATASIZE_8BIT)
	};
	spid->err_cb = spi_err_cb;
	spiStart(spid, &t->bus.spi_hw.cfg);
	spid->app_arg = t;
}

bool transport_spi_hw_async_supported(transport_t *t) {
	return t != NULL && t->interface == &spi_hw_interface;
}

void transport_spi_hw_async_set_callback(transport_t *t,
		void (*callback)(void *arg, bool error), void *arg) {
	if (!transport_spi_hw_async_supported(t)) {
		return;
	}

	t->bus.spi_hw.async_done_cb = callback;
	t->bus.spi_hw.async_done_arg = arg;
}

bool transport_spi_hw_async_busy(transport_t *t) {
	return transport_spi_hw_async_supported(t) &&
			(t->bus.spi_hw.async_active || t->bus.spi_hw.async_complete);
}

bool transport_spi_hw_async_active(transport_t *t) {
	return transport_spi_hw_async_supported(t) && t->bus.spi_hw.async_active;
}

bool transport_spi_hw_async_start_read(transport_t *t, uint8_t reg, size_t len, bool from_isr) {
	if (!transport_spi_hw_async_supported(t) || len == 0 || len > IMU_MAX_BURST) {
		return false;
	}

	SPIDriver *spid = dev_of(t);
	if (from_isr) {
		chSysLockFromISR();
	} else {
		chSysLock();
	}

	// async_txd[1..] was zeroed once at initialization. Dedicated async buffers
	// let this critical section contain only ownership, one address-byte store,
	// and the DMA launch; no high-rate memset can delay FOC.
	bool started = !t->bus.spi_hw.thread_owned && !t->bus.spi_hw.async_active &&
			!t->bus.spi_hw.async_complete && spid->state == SPI_READY;
	if (started) {
		t->bus.spi_hw.async_txd[0] = reg | SPI_READ_BIT;
		t->bus.spi_hw.async_error = false;
		t->bus.spi_hw.async_complete = false;
		t->bus.spi_hw.async_active = true;

		// Clear a stale RXNE state before arming DMA, matching the proven G18 path.
		volatile uint32_t rxne_clear = spid->spi->DR;
		(void)rxne_clear;
		palClearPad(t->bus.spi_hw.cfg.ssport, t->bus.spi_hw.cfg.sspad);
		spiStartExchangeI(spid, len + 1, t->bus.spi_hw.async_txd,
				t->bus.spi_hw.async_rxd);
	}
	if (from_isr) {
		chSysUnlockFromISR();
	} else {
		chSysUnlock();
	}

	return started;
}

bool transport_spi_hw_async_copy_read(transport_t *t, uint8_t *rx, size_t len) {
	if (!transport_spi_hw_async_supported(t) || rx == NULL || len == 0 ||
			len > IMU_MAX_BURST || !t->bus.spi_hw.async_complete ||
			t->bus.spi_hw.async_error) {
		return false;
	}

	memcpy(rx, t->bus.spi_hw.async_rxd + 1, len);
	t->bus.spi_hw.async_complete = false;
	return true;
}

void transport_spi_hw_async_abort(transport_t *t) {
	if (!transport_spi_hw_async_supported(t)) {
		return;
	}

	SPIDriver *spid = dev_of(t);
	spiAcquireBus(spid);
	chSysLock();
	// Hold an ownership barrier across the stop/start sequence so DRDY cannot
	// arm a new DMA transfer while the peripheral is being recovered.
	t->bus.spi_hw.thread_owned = true;
	chSysUnlock();

	reset_spi_while_owned(t);

	chSysLock();
	t->bus.spi_hw.thread_owned = false;
	chSysUnlock();
	spiReleaseBus(spid);
}

bool transport_spi_hw_get_status(transport_t *t, transport_spi_hw_status_t *status) {
	if (!transport_spi_hw_async_supported(t) || status == NULL) {
		return false;
	}

	chSysLock();
	status->thread_owned = t->bus.spi_hw.thread_owned;
	status->sync_active = t->bus.spi_hw.sync_active;
	status->async_active = t->bus.spi_hw.async_active;
	status->async_complete = t->bus.spi_hw.async_complete;
	status->async_error = t->bus.spi_hw.async_error;
	status->sync_timeouts = t->bus.spi_hw.sync_timeouts;
	status->sync_dma_errors = t->bus.spi_hw.sync_dma_errors;
	status->async_dma_errors = t->bus.spi_hw.async_dma_errors;
	status->spi_resets = t->bus.spi_hw.spi_resets;
	status->spi_state = dev_of(t)->state;
	chSysUnlock();
	return true;
}
