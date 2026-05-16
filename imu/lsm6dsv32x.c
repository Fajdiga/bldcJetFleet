/*
	Copyright 2024 Contributors

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "lsm6dsv32x.h"
#include "terminal.h"
#include "i2c_bb.h"
#include "spi_bb.h"
#include "commands.h"
#include "utils_math.h"

#include <stdio.h>
#include <string.h>

#define LSM6DSV32X_BURST_READ_LEN	13

static thread_t *lsm6dsv32x_thread_ref = NULL;
static binary_semaphore_t m_drdy_sem;
static binary_semaphore_t m_spi_sem;
static bool m_drdy_sem_init = false;
static bool m_spi_sem_init = false;
static bool m_use_int1 = false;
static i2c_bb_state *m_i2c_bb;
static SPIDriver *m_spi_dev = NULL;
static stm32_gpio_t *m_nss_gpio;
static int m_nss_pin;
static bool m_use_spi = false;
static volatile uint16_t lsm6dsv32x_addr;
static uint8_t m_stream_txb[LSM6DSV32X_BURST_READ_LEN];
static uint8_t m_stream_rxb[LSM6DSV32X_BURST_READ_LEN];
static volatile bool m_spi_stream_enabled = false;
static volatile bool m_spi_stream_active = false;
static volatile bool m_spi_stream_complete = false;
static volatile bool m_spi_stream_pending = false;
static volatile bool m_spi_sync_active = false;
static volatile bool m_spi_dma_error = false;
static volatile uint32_t m_spi_stream_overruns = 0;
static volatile uint32_t m_stat_drdy = 0;
static volatile uint32_t m_stat_drdy_ignored_disabled = 0;
static volatile uint32_t m_stat_drdy_ignored_active = 0;
static volatile uint32_t m_stat_drdy_ignored_complete = 0;
static volatile uint32_t m_stat_pending_latched = 0;
static volatile uint32_t m_stat_pending_restarted = 0;
static volatile uint32_t m_stat_pending_restart_failed = 0;
static volatile uint32_t m_stat_drdy_ignored_sync = 0;
static volatile uint32_t m_stat_drdy_ignored_not_ready = 0;
static volatile uint32_t m_stat_stream_started = 0;
static volatile uint32_t m_stat_stream_completed = 0;
static volatile uint32_t m_stat_stream_copied = 0;
static volatile uint32_t m_stat_copy_failed = 0;
static volatile uint32_t m_stat_wait_timeout = 0;
static volatile uint32_t m_stat_recover = 0;
static volatile uint32_t m_stat_reset_ok = 0;
static volatile uint32_t m_stat_reset_fail = 0;
static volatile uint32_t m_stat_sync_transfer_failed = 0;
static volatile uint32_t m_stat_spi_dma_errors = 0;
static volatile uint32_t m_stat_samples = 0;
static volatile uint32_t m_stat_last_sample_time = 0;
static volatile uint32_t m_stat_min_sample_dt = 0xFFFFFFFF;
static volatile uint32_t m_stat_max_sample_dt = 0;
static volatile int16_t m_stat_last_raw[6];
static volatile int16_t m_stat_max_raw_delta[6];

// Default rate. Can be changed before init with lsm6dsv32x_set_rate_hz().
static int rate_hz = 1000;

// Explicit default. If your enum has IMU_FILTER_LOW, this can be changed to:
// static IMU_FILTER filter = IMU_FILTER_LOW;
static IMU_FILTER filter = (IMU_FILTER)0;

static void spi_end_cb(SPIDriver *spi_dev);
static void spi_error_cb(SPIDriver *spi_dev);
static bool spi_transfer(const uint8_t *txb, uint8_t *rxb, size_t len);
static void prepare_stream_read(void);
static bool copy_stream_read(uint8_t *data, int len);
static void recover_spi_stream(void);
static bool start_stream_read(bool from_isr);

// SPI mode 3: CPOL = 1, CPHA = 1.
// Prescaler /8 gives ~5.25 MHz on SPI3 @ 42 MHz APB1.
// LSM6DSV32X SPI max is 10 MHz, so /8 is safe.
static const SPIConfig m_spi_cfg = {
	.end_cb = spi_end_cb,
	.ssport = NULL,
	.sspad = 0,
	.cr1 = SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA,
};

static bool reset_init_lsm6dsv32x(void);
static void terminal_read_reg(int argc, const char **argv);
static void terminal_write_reg(int argc, const char **argv);
static void terminal_stats(int argc, const char **argv);
static void reset_stats(void);
static uint8_t read_single_reg(uint8_t reg);
static bool write_single_reg(uint8_t reg, uint8_t value);
static bool write_config_reg(uint8_t reg, uint8_t value, const char *name);
static uint8_t odr_from_rate(int hz, int *odr_hz);
static uint8_t accel_lpf2_bw_for_filter(IMU_FILTER f, int odr_hz);
static uint8_t gyro_lpf1_bw_for_filter(IMU_FILTER f);
static bool read_regs(uint8_t reg, uint8_t *data, int len);
static THD_FUNCTION(lsm6dsv32x_thread, arg);

// Function pointer
static void(*read_callback)(float *accel, float *gyro, float *mag) = 0;

void lsm6dsv32x_set_rate_hz(int hz) {
	if (hz < 1) {
		hz = 1;
	}

	rate_hz = hz;
}

void lsm6dsv32x_set_filter(IMU_FILTER f) {
	filter = f;
}

void lsm6dsv32x_int1_isr(void) {
	if (m_use_spi) {
		m_stat_drdy++;

		if (!m_spi_stream_enabled || m_spi_dev == NULL || m_nss_gpio == NULL) {
			m_stat_drdy_ignored_disabled++;
			return;
		}

		bool busy_active = m_spi_stream_active;
		bool busy_complete = m_spi_stream_complete;
		bool busy_sync = m_spi_sync_active;
		bool busy_not_ready = m_spi_dev->state != SPI_READY;

		if (busy_active || busy_complete || busy_sync || busy_not_ready) {
			bool can_latch_pending = busy_active || busy_complete || busy_not_ready;

			if (busy_active) {
				m_stat_drdy_ignored_active++;
			}

			if (busy_complete) {
				m_stat_drdy_ignored_complete++;
			}

			if (busy_sync) {
				m_stat_drdy_ignored_sync++;
			}

			if (busy_not_ready) {
				m_stat_drdy_ignored_not_ready++;
			}

			if (can_latch_pending && !m_spi_stream_pending) {
				m_spi_stream_pending = true;
				m_stat_pending_latched++;
			} else {
				m_spi_stream_overruns++;
			}

			return;
		}

		start_stream_read(true);
		return;
	}

	if (m_drdy_sem_init) {
		chSysLockFromISR();
		chBSemSignalI(&m_drdy_sem);
		chSysUnlockFromISR();
	}
}

void lsm6dsv32x_init_spi(SPIDriver *spi_dev, stm32_gpio_t *nss_gpio, int nss_pin,
		stkalign_t *work_area, size_t work_area_size) {

	read_callback = 0;
	m_use_spi = true;
	m_use_int1 = true;
	m_spi_dev = spi_dev;
	m_nss_gpio = nss_gpio;
	m_nss_pin = nss_pin;
	m_spi_stream_enabled = false;
	m_spi_stream_active = false;
	m_spi_stream_complete = false;
	m_spi_stream_pending = false;
	m_spi_dma_error = false;

	if (!m_drdy_sem_init) {
		// true = taken, so the thread waits for the first real interrupt.
		chBSemObjectInit(&m_drdy_sem, true);
		m_drdy_sem_init = true;
	}

	if (!m_spi_sem_init) {
		chBSemObjectInit(&m_spi_sem, true);
		m_spi_sem_init = true;
	}

	palSetPad(m_nss_gpio, m_nss_pin);
	m_spi_dev->err_cb = spi_error_cb;
	spiStart(m_spi_dev, &m_spi_cfg);

	// Verify WHO_AM_I.
	uint8_t who = read_single_reg(LSM6DSV32X_WHO_AM_I);
	if (who != LSM6DSV32X_WHO_AM_I_VAL) {
		commands_printf("LSM6DSV32X SPI WHO_AM_I mismatch: 0x%02X", who);
		return;
	}

	if (!reset_init_lsm6dsv32x()) {
		commands_printf("LSM6DSV32X SPI Init FAILED");
		return;
	}

	prepare_stream_read();
	m_spi_stream_enabled = true;

	terminal_register_command_callback(
			"lsm6dsv32x_read_reg",
			"Read register of the LSM6DSV32X",
			"[reg]",
			terminal_read_reg);

	terminal_register_command_callback(
			"lsm6dsv32x_write_reg",
			"Write register of the LSM6DSV32X",
			"[reg] [value]",
			terminal_write_reg);

	terminal_register_command_callback(
			"lsm6dsv32x_stats",
			"Print or reset LSM6DSV32X stream statistics",
			"[reset]",
			terminal_stats);

	lsm6dsv32x_thread_ref = chThdCreateStatic(work_area, work_area_size,
			NORMALPRIO, lsm6dsv32x_thread, NULL);
}

void lsm6dsv32x_init(i2c_bb_state *i2c_state,
		stkalign_t *work_area, size_t work_area_size) {

	read_callback = 0;
	m_use_spi = false;
	m_use_int1 = false;
	m_i2c_bb = i2c_state;

	// Recover I2C bus in case it is stuck.
	i2c_bb_restore_bus(m_i2c_bb);
	chThdSleepMilliseconds(1);

	// Detect I2C address.
	uint8_t txb[1];
	uint8_t rxb[1];

	txb[0] = LSM6DSV32X_WHO_AM_I;
	lsm6dsv32x_addr = LSM6DSV32X_ADDR_A;

	bool res = i2c_bb_tx_rx(m_i2c_bb, lsm6dsv32x_addr, txb, 1, rxb, 1);
	if (!res || rxb[0] != LSM6DSV32X_WHO_AM_I_VAL) {
		commands_printf("LSM6DSV32X Address A failed, trying B (rx: 0x%02X)", rxb[0]);

		lsm6dsv32x_addr = LSM6DSV32X_ADDR_B;
		res = i2c_bb_tx_rx(m_i2c_bb, lsm6dsv32x_addr, txb, 1, rxb, 1);

		if (!res || rxb[0] != LSM6DSV32X_WHO_AM_I_VAL) {
			commands_printf("LSM6DSV32X Address B failed (rx: 0x%02X)", rxb[0]);
			return;
		}
	}

	if (!reset_init_lsm6dsv32x()) {
		commands_printf("LSM6DSV32X Init FAILED");
		return;
	}

	terminal_register_command_callback(
			"lsm6dsv32x_read_reg",
			"Read register of the LSM6DSV32X",
			"[reg]",
			terminal_read_reg);

	terminal_register_command_callback(
			"lsm6dsv32x_write_reg",
			"Write register of the LSM6DSV32X",
			"[reg] [value]",
			terminal_write_reg);

	terminal_register_command_callback(
			"lsm6dsv32x_stats",
			"Print or reset LSM6DSV32X stream statistics",
			"[reset]",
			terminal_stats);

	lsm6dsv32x_thread_ref = chThdCreateStatic(work_area, work_area_size,
			NORMALPRIO, lsm6dsv32x_thread, NULL);
}

static bool reset_init_lsm6dsv32x(void) {
	// Software reset.
	if (!write_config_reg(LSM6DSV32X_CTRL3, LSM6DSV32X_SW_RESET, "Software Reset")) {
		return false;
	}

	chThdSleepMilliseconds(10);

	if (m_use_spi && !write_config_reg(LSM6DSV32X_IF_CFG,
			LSM6DSV32X_I2C_I3C_DISABLE,
			"SPI Interface Config")) {
		return false;
	}

	/*
	 * BDU:
	 *   Prevents low/high byte tearing while output registers are read.
	 *
	 * IF_INC:
	 *   Required for burst reads. We read 12 bytes starting at OUTX_L_G.
	 */
	if (!write_config_reg(LSM6DSV32X_CTRL3,
			LSM6DSV32X_BDU | LSM6DSV32X_IF_INC,
			"BDU/Auto-Increment Config")) {
		return false;
	}

	int odr_hz = 0;
	uint8_t odr = odr_from_rate(rate_hz, &odr_hz);

	/*
	 * Accelerometer full-scale: +-32 g.
	 * Sensitivity: 0.976 mg/LSB.
	 *
	 * LSM6DS3-TRC uses +-16 g, but that was a real application bottleneck.
	 * Keep the LSM6DSV32X at +-32 g and compare noise in physical units.
	 *
	 * LSM6DS3-TRC reference bandwidths at 6.66 kHz internal ODR:
	 * LOW ~= 400 Hz, MEDIUM ~= 133 Hz, HIGH ~= 67 Hz.
	 *
	 * LSM6DSV32X accelerometer LPF1 is the default first-stage output
	 * (ODR/2 in high-performance mode). LPF2 is the configurable stage.
	 * For the intended 1920/3840 Hz rates this maps to:
	 * 1920 Hz: LOW 480 Hz, MEDIUM 96 Hz, HIGH 43 Hz
	 * 3840 Hz: LOW 384 Hz, MEDIUM 192 Hz, HIGH 85 Hz
	 */
	uint8_t ctrl8_val = LSM6DSV32X_XL_FS_MODE | LSM6DSV32X_FS_XL_32g;
	ctrl8_val |= accel_lpf2_bw_for_filter(filter, odr_hz);
	uint8_t ctrl9_val = LSM6DSV32X_XL_LPF2_EN;

	if (!write_config_reg(LSM6DSV32X_CTRL8, ctrl8_val, "Accel FS/Filter Config")) {
		return false;
	}

	if (!write_config_reg(LSM6DSV32X_CTRL9, ctrl9_val, "Accel LPF2 Enable Config")) {
		return false;
	}

	/*
	 * Gyro full-scale: +-2000 dps.
	 *
	 * LSM6DS3-TRC reference bandwidths: LOW ~= 351 Hz,
	 * MEDIUM ~= 237 Hz, HIGH ~= 173 Hz.
	 *
	 * For 1920/3840 Hz on LSM6DSV32X this maps to:
	 * LOW BW_3 ~= 387/403 Hz, MEDIUM BW_1 ~= 210/213 Hz,
	 * HIGH BW_2 ~= 155/156 Hz.
	 */
	uint8_t ctrl6_val = LSM6DSV32X_FS_G_2000dps;
	ctrl6_val |= gyro_lpf1_bw_for_filter(filter);

	if (!write_config_reg(LSM6DSV32X_CTRL6, ctrl6_val, "Gyro FS/Filter Config")) {
		return false;
	}

	// Always use gyro LPF1; the preset selects how close we are to LSM6DS3-TRC.
	uint8_t ctrl7_val = LSM6DSV32X_G_LPF1_EN;

	if (!write_config_reg(LSM6DSV32X_CTRL7, ctrl7_val, "Gyro LPF1 Enable Config")) {
		return false;
	}

	/*
	 * Determine accelerometer ODR from requested sample rate.
	 * Filter settings must not change ODR.
	 */
	if (!write_config_reg(LSM6DSV32X_CTRL1,
			LSM6DSV32X_XL_MODE_HIGH_PERF | odr,
			"Accel ODR Config")) {
		return false;
	}

	/*
	 * Determine gyro ODR from requested sample rate.
	 * INT1 is routed from gyro DRDY, so this controls interrupt frequency.
	 */
	if (!write_config_reg(LSM6DSV32X_CTRL2,
			LSM6DSV32X_G_MODE_HIGH_PERF | odr,
			"Gyro ODR Config")) {
		return false;
	}

	/*
	 * Production low-latency mode:
	 *
	 * Route only gyro data-ready to INT1.
	 * Accel and gyro are configured to the same ODR.
	 * Your scope test showed accel and gyro DRDY are aligned.
	 *
	 * Do not route both XL and G DRDY in production. Multiple interrupt
	 * sources on one pin are ORed by hardware, not ANDed.
	 */
	if (m_use_int1) {
		if (!write_config_reg(LSM6DSV32X_INT1_CTRL,
				LSM6DSV32X_INT1_DRDY_G,
				"INT1 Data-Ready Config")) {
			return false;
		}
	}

	return true;
}

void lsm6dsv32x_stop(void) {
	if (lsm6dsv32x_thread_ref != NULL) {
		chThdTerminate(lsm6dsv32x_thread_ref);
		chThdWait(lsm6dsv32x_thread_ref);
	}

	if (m_use_spi && m_spi_dev != NULL) {
		m_spi_stream_enabled = false;
		m_spi_stream_active = false;
		m_spi_stream_complete = false;
		m_spi_stream_pending = false;
		m_spi_dma_error = false;
		spiStop(m_spi_dev);
		m_spi_dev->err_cb = NULL;
		m_spi_dev = NULL;
		m_use_spi = false;
	}

	lsm6dsv32x_thread_ref = NULL;

	terminal_unregister_callback(terminal_read_reg);
	terminal_unregister_callback(terminal_write_reg);
	terminal_unregister_callback(terminal_stats);
}

void lsm6dsv32x_set_read_callback(void(*func)(float *accel, float *gyro, float *mag)) {
	read_callback = func;
}

static void spi_end_cb(SPIDriver *spi_dev) {
	(void)spi_dev;

	if (m_nss_gpio != NULL) {
		palSetPad(m_nss_gpio, m_nss_pin);
	}

	if (m_spi_stream_active) {
		m_spi_stream_active = false;
		m_spi_stream_complete = true;
		m_stat_stream_completed++;

		if (m_drdy_sem_init) {
			chSysLockFromISR();
			chBSemSignalI(&m_drdy_sem);
			chSysUnlockFromISR();
		}

		return;
	}

	if (m_spi_sem_init) {
		chSysLockFromISR();
		chBSemSignalI(&m_spi_sem);
		chSysUnlockFromISR();
	}
}

static void spi_error_cb(SPIDriver *spi_dev) {
	(void)spi_dev;

	m_spi_dma_error = true;
	m_stat_spi_dma_errors++;
}

static bool spi_transfer(const uint8_t *txb, uint8_t *rxb, size_t len) {
	if (m_spi_dev == NULL || txb == NULL || rxb == NULL || len == 0 || !m_spi_sem_init) {
		return false;
	}

	m_spi_sync_active = true;

	if (m_spi_stream_active || m_spi_stream_complete) {
		m_spi_sync_active = false;
		return false;
	}

	chBSemReset(&m_spi_sem, true);
	spiAcquireBus(m_spi_dev);
	m_spi_dma_error = false;

	// Same DMA/RXNE guard used by the async encoder SPI drivers.
	volatile uint32_t rxne_clear = m_spi_dev->spi->DR;
	(void)rxne_clear;

	palClearPad(m_nss_gpio, m_nss_pin);

	chSysLock();
	spiStartExchangeI(m_spi_dev, len, txb, rxb);
	chSysUnlock();

	msg_t msg = chBSemWait(&m_spi_sem);
	bool dma_error = m_spi_dma_error;
	m_spi_dma_error = false;
	spiReleaseBus(m_spi_dev);
	m_spi_sync_active = false;

	bool ok = msg == MSG_OK && !dma_error;

	if (!ok) {
		m_stat_sync_transfer_failed++;
	}

	return ok;
}

static void prepare_stream_read(void) {
	m_stream_txb[0] = LSM6DSV32X_OUTX_L_G | LSM6DSV32X_SPI_RD_MASK;

	for (int i = 1; i < LSM6DSV32X_BURST_READ_LEN; i++) {
		m_stream_txb[i] = 0;
		m_stream_rxb[i] = 0;
	}

	m_stream_rxb[0] = 0;
}

static bool copy_stream_read(uint8_t *data, int len) {
	if (data == NULL || len != (LSM6DSV32X_BURST_READ_LEN - 1) ||
			!m_spi_stream_complete || m_spi_dma_error) {
		m_spi_dma_error = false;
		m_stat_copy_failed++;
		return false;
	}

	for (int i = 0; i < len; i++) {
		data[i] = m_stream_rxb[i + 1];
	}

	m_spi_stream_complete = false;
	m_spi_dma_error = false;
	m_stat_stream_copied++;
	return true;
}

static void recover_spi_stream(void) {
	m_stat_recover++;
	m_spi_stream_enabled = false;
	m_spi_stream_active = false;
	m_spi_stream_complete = false;
	m_spi_stream_pending = false;
	m_spi_dma_error = false;

	if (m_drdy_sem_init) {
		chBSemReset(&m_drdy_sem, true);
	}

	if (m_spi_dev != NULL) {
		palSetPad(m_nss_gpio, m_nss_pin);

		if (m_spi_dev->state != SPI_READY) {
			spiStop(m_spi_dev);
			spiStart(m_spi_dev, &m_spi_cfg);
		}
	}
}

static bool start_stream_read(bool from_isr) {
	if (!m_spi_stream_enabled || m_spi_dev == NULL || m_nss_gpio == NULL ||
			m_spi_stream_active || m_spi_stream_complete || m_spi_sync_active ||
			m_spi_dev->state != SPI_READY) {
		return false;
	}

	m_spi_stream_complete = false;
	m_spi_dma_error = false;
	m_spi_stream_active = true;
	m_stat_stream_started++;

	// Same DMA/RXNE guard used by the async encoder SPI drivers.
	volatile uint32_t rxne_clear = m_spi_dev->spi->DR;
	(void)rxne_clear;

	palClearPad(m_nss_gpio, m_nss_pin);

	if (from_isr) {
		chSysLockFromISR();
		spiStartExchangeI(m_spi_dev, LSM6DSV32X_BURST_READ_LEN, m_stream_txb, m_stream_rxb);
		chSysUnlockFromISR();
	} else {
		chSysLock();
		spiStartExchangeI(m_spi_dev, LSM6DSV32X_BURST_READ_LEN, m_stream_txb, m_stream_rxb);
		chSysUnlock();
	}

	return true;
}

static bool read_regs(uint8_t reg, uint8_t *data, int len) {
	if (m_use_spi) {
		uint8_t txb[len + 1];
		uint8_t rxb[len + 1];

		txb[0] = reg | LSM6DSV32X_SPI_RD_MASK;
		for (int i = 1; i < (len + 1); i++) {
			txb[i] = 0;
		}

		if (!spi_transfer(txb, rxb, len + 1)) {
			return false;
		}

		for (int i = 0; i < len; i++) {
			data[i] = rxb[i + 1];
		}

		return true;
	}

	uint8_t txb[1] = { reg };
	return i2c_bb_tx_rx(m_i2c_bb, lsm6dsv32x_addr, txb, 1, data, len);
}

static bool write_single_reg(uint8_t reg, uint8_t value) {
	if (m_use_spi) {
		uint8_t txb[2] = {
				reg & LSM6DSV32X_SPI_WR_MASK,
				value,
		};
		uint8_t rxb[sizeof(txb)];

		return spi_transfer(txb, rxb, sizeof(txb));
	}

	uint8_t txb[2] = { reg, value };
	return i2c_bb_tx_rx(m_i2c_bb, lsm6dsv32x_addr, txb, 2, 0, 0);
}

static bool write_config_reg(uint8_t reg, uint8_t value, const char *name) {
	bool res = write_single_reg(reg, value);

	if (!res) {
		commands_printf("LSM6DSV32X %s FAILED (reg 0x%02X, val 0x%02X)",
				name, reg, value);
	}

	return res;
}

static uint8_t odr_from_rate(int hz, int *odr_hz) {
	uint8_t odr = LSM6DSV32X_ODR_XL_960Hz;
	int actual_hz = 960;

	if (hz <= 8) {
		odr = LSM6DSV32X_ODR_XL_7_5Hz;
		actual_hz = 8;
	} else if (hz <= 15) {
		odr = LSM6DSV32X_ODR_XL_15Hz;
		actual_hz = 15;
	} else if (hz <= 30) {
		odr = LSM6DSV32X_ODR_XL_30Hz;
		actual_hz = 30;
	} else if (hz <= 60) {
		odr = LSM6DSV32X_ODR_XL_60Hz;
		actual_hz = 60;
	} else if (hz <= 120) {
		odr = LSM6DSV32X_ODR_XL_120Hz;
		actual_hz = 120;
	} else if (hz <= 240) {
		odr = LSM6DSV32X_ODR_XL_240Hz;
		actual_hz = 240;
	} else if (hz <= 480) {
		odr = LSM6DSV32X_ODR_XL_480Hz;
		actual_hz = 480;
	} else if (hz <= 960) {
		odr = LSM6DSV32X_ODR_XL_960Hz;
		actual_hz = 960;
	} else if (hz <= 1920) {
		odr = LSM6DSV32X_ODR_XL_1920Hz;
		actual_hz = 1920;
	} else if (hz <= 3840) {
		odr = LSM6DSV32X_ODR_XL_3840Hz;
		actual_hz = 3840;
	} else {
		odr = LSM6DSV32X_ODR_XL_7680Hz;
		actual_hz = 7680;
	}

	if (odr_hz) {
		*odr_hz = actual_hz;
	}

	return odr;
}

static uint8_t accel_lpf2_bw_for_filter(IMU_FILTER f, int odr_hz) {
	switch (f) {
	case IMU_FILTER_LOW:
		return odr_hz >= 3840 ? LSM6DSV32X_XL_HP_BW_ODR_10 : LSM6DSV32X_XL_HP_BW_ODR_4;
	case IMU_FILTER_MEDIUM:
		return LSM6DSV32X_XL_HP_BW_ODR_20;
	case IMU_FILTER_HIGH:
	default:
		return LSM6DSV32X_XL_HP_BW_ODR_45;
	}
}

static uint8_t gyro_lpf1_bw_for_filter(IMU_FILTER f) {
	switch (f) {
	case IMU_FILTER_LOW:
		return LSM6DSV32X_G_LPF1_BW_3;
	case IMU_FILTER_MEDIUM:
		return LSM6DSV32X_G_LPF1_BW_1;
	case IMU_FILTER_HIGH:
	default:
		return LSM6DSV32X_G_LPF1_BW_2;
	}
}

static uint8_t read_single_reg(uint8_t reg) {
	uint8_t rxb[1] = { 0 };

	if (read_regs(reg, rxb, 1)) {
		return rxb[0];
	}

	return 0;
}

static void terminal_read_reg(int argc, const char **argv) {
	if (argc == 2) {
		int reg = -1;
		sscanf(argv[1], "%d", &reg);

		if (reg >= 0) {
			unsigned int res = read_single_reg(reg);

			char bl[9];
			utils_byte_to_binary(res & 0xFF, bl);

			commands_printf("Reg 0x%02x: %s (0x%02x)\n", reg, bl, res);
		} else {
			commands_printf("Invalid argument(s).\n");
		}
	} else {
		commands_printf("This command requires one argument.\n");
	}
}

static void terminal_write_reg(int argc, const char **argv) {
	if (argc == 3) {
		int reg = -1;
		int val = -1;

		sscanf(argv[1], "%d", &reg);
		sscanf(argv[2], "%d", &val);

		if (reg >= 0 && val >= 0 && val <= 255) {
			bool res = write_single_reg(reg, val);
			unsigned int readback = read_single_reg(reg);

			char bl[9];
			utils_byte_to_binary(readback & 0xFF, bl);

			commands_printf("Write reg 0x%02x = 0x%02x: %s. Readback: %s (0x%02x)\n",
					reg, val, res ? "OK" : "FAIL", bl, readback);
		} else {
			commands_printf("Invalid argument(s).\n");
		}
	} else {
		commands_printf("This command requires two arguments: [reg] [value]\n");
	}
}

static void terminal_stats(int argc, const char **argv) {
	if (argc == 2 && strcmp(argv[1], "reset") == 0) {
		reset_stats();
		commands_printf("LSM6DSV32X stats reset\n");
		return;
	}

	if (argc != 1) {
		commands_printf("Usage: lsm6dsv32x_stats [reset]\n");
		return;
	}

	uint32_t drdy;
	uint32_t ignored_disabled;
	uint32_t ignored_active;
	uint32_t ignored_complete;
	uint32_t pending_latched;
	uint32_t pending_restarted;
	uint32_t pending_restart_failed;
	uint32_t ignored_sync;
	uint32_t ignored_not_ready;
	uint32_t stream_started;
	uint32_t stream_completed;
	uint32_t stream_copied;
	uint32_t copy_failed;
	uint32_t wait_timeout;
	uint32_t recover;
	uint32_t reset_ok;
	uint32_t reset_fail;
	uint32_t sync_failed;
	uint32_t dma_errors;
	uint32_t samples;
	uint32_t min_sample_dt;
	uint32_t max_sample_dt;
	uint32_t last_sample_time;
	uint32_t overruns;
	int16_t raw[6];
	int16_t max_raw_delta[6];
	bool stream_enabled;
	bool stream_active;
	bool stream_complete;
	bool stream_pending;
	bool sync_active;
	spistate_t spi_state = SPI_STOP;

	chSysLock();
	drdy = m_stat_drdy;
	ignored_disabled = m_stat_drdy_ignored_disabled;
	ignored_active = m_stat_drdy_ignored_active;
	ignored_complete = m_stat_drdy_ignored_complete;
	pending_latched = m_stat_pending_latched;
	pending_restarted = m_stat_pending_restarted;
	pending_restart_failed = m_stat_pending_restart_failed;
	ignored_sync = m_stat_drdy_ignored_sync;
	ignored_not_ready = m_stat_drdy_ignored_not_ready;
	stream_started = m_stat_stream_started;
	stream_completed = m_stat_stream_completed;
	stream_copied = m_stat_stream_copied;
	copy_failed = m_stat_copy_failed;
	wait_timeout = m_stat_wait_timeout;
	recover = m_stat_recover;
	reset_ok = m_stat_reset_ok;
	reset_fail = m_stat_reset_fail;
	sync_failed = m_stat_sync_transfer_failed;
	dma_errors = m_stat_spi_dma_errors;
	samples = m_stat_samples;
	min_sample_dt = m_stat_min_sample_dt;
	max_sample_dt = m_stat_max_sample_dt;
	last_sample_time = m_stat_last_sample_time;
	overruns = m_spi_stream_overruns;
	for (int i = 0; i < 6; i++) {
		raw[i] = m_stat_last_raw[i];
		max_raw_delta[i] = m_stat_max_raw_delta[i];
	}
	stream_enabled = m_spi_stream_enabled;
	stream_active = m_spi_stream_active;
	stream_complete = m_spi_stream_complete;
	stream_pending = m_spi_stream_pending;
	sync_active = m_spi_sync_active;
	if (m_spi_dev != NULL) {
		spi_state = m_spi_dev->state;
	}
	chSysUnlock();

	uint32_t lost_est = drdy > stream_copied ? drdy - stream_copied : 0;
	uint32_t pending_done = stream_completed > stream_copied ? stream_completed - stream_copied : 0;
	uint32_t pending_started = stream_started > stream_completed ? stream_started - stream_completed : 0;
	uint32_t age_ms = last_sample_time != 0 ? ST2MS(chVTGetSystemTimeX() - last_sample_time) : 0;
	uint32_t min_dt_us = min_sample_dt == 0xFFFFFFFF ? 0 : ST2US(min_sample_dt);
	uint32_t max_dt_us = ST2US(max_sample_dt);

	commands_printf("LSM6DSV32X stats:");
	commands_printf("  mode spi=%d int1=%d rate=%dHz spi_state=%d", m_use_spi, m_use_int1, rate_hz, spi_state);
	commands_printf("  flags en=%d active=%d complete=%d pending=%d sync=%d",
			stream_enabled, stream_active, stream_complete, stream_pending, sync_active);
	commands_printf("  drdy=%u started=%u completed=%u copied=%u samples=%u", drdy, stream_started, stream_completed, stream_copied, samples);
	commands_printf("  lost_est=%u pending_started=%u pending_done=%u overruns=%u", lost_est, pending_started, pending_done, overruns);
	commands_printf("  overrun reasons active=%u complete=%u sync=%u not_ready=%u disabled=%u",
			ignored_active, ignored_complete, ignored_sync, ignored_not_ready, ignored_disabled);
	commands_printf("  pending latched=%u restart ok=%u fail=%u",
			pending_latched, pending_restarted, pending_restart_failed);
	commands_printf("  failures copy=%u timeout=%u recover=%u reset_ok=%u reset_fail=%u sync_fail=%u dma=%u",
			copy_failed, wait_timeout, recover, reset_ok, reset_fail, sync_failed, dma_errors);
	commands_printf("  sample_dt_us min=%u max=%u last_age_ms=%u", min_dt_us, max_dt_us, age_ms);
	commands_printf("  last_raw g=[%d %d %d] a=[%d %d %d]",
			raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
	commands_printf("  max_raw_delta g=[%d %d %d] a=[%d %d %d]",
			max_raw_delta[0], max_raw_delta[1], max_raw_delta[2],
			max_raw_delta[3], max_raw_delta[4], max_raw_delta[5]);
}

static void reset_stats(void) {
	chSysLock();
	m_spi_stream_overruns = 0;
	m_stat_drdy = 0;
	m_stat_drdy_ignored_disabled = 0;
	m_stat_drdy_ignored_active = 0;
	m_stat_drdy_ignored_complete = 0;
	m_stat_pending_latched = 0;
	m_stat_pending_restarted = 0;
	m_stat_pending_restart_failed = 0;
	m_stat_drdy_ignored_sync = 0;
	m_stat_drdy_ignored_not_ready = 0;
	m_stat_stream_started = 0;
	m_stat_stream_completed = 0;
	m_stat_stream_copied = 0;
	m_stat_copy_failed = 0;
	m_stat_wait_timeout = 0;
	m_stat_recover = 0;
	m_stat_reset_ok = 0;
	m_stat_reset_fail = 0;
	m_stat_sync_transfer_failed = 0;
	m_stat_spi_dma_errors = 0;
	m_stat_samples = 0;
	m_stat_last_sample_time = 0;
	m_stat_min_sample_dt = 0xFFFFFFFF;
	m_stat_max_sample_dt = 0;
	for (int i = 0; i < 6; i++) {
		m_stat_last_raw[i] = 0;
		m_stat_max_raw_delta[i] = 0;
	}
	chSysUnlock();
}

static THD_FUNCTION(lsm6dsv32x_thread, arg) {
	(void)arg;

	chRegSetThreadName("LSM6DSV32X");

	systime_t iteration_timer = chVTGetSystemTimeX();

	const int local_rate_hz = rate_hz > 0 ? rate_hz : 1;
	const systime_t desired_interval = US2ST(1000000 / local_rate_hz);

	// Watchdog timeout for INT1 wait: 4x expected sample period, minimum 5 ms.
	const systime_t int1_timeout =
			MS2ST(5) > (desired_interval * 4) ? MS2ST(5) : (desired_interval * 4);

	while (!chThdShouldTerminateX()) {
		uint8_t rxb[12];
		bool res = false;

		if (m_use_int1) {
			msg_t wait_res = chBSemWaitTimeout(&m_drdy_sem, int1_timeout);

			if (wait_res == MSG_OK) {
				if (m_use_spi) {
					res = copy_stream_read(rxb, 12);
					bool restart_pending = false;

					if (res) {
						chSysLock();
						restart_pending = m_spi_stream_pending;
						m_spi_stream_pending = false;
						chSysUnlock();
					}

					if (restart_pending) {
						if (start_stream_read(false)) {
							m_stat_pending_restarted++;
						} else {
							m_stat_pending_restart_failed++;
						}
					}
				} else {
					res = read_regs(LSM6DSV32X_OUTX_L_G, rxb, 12);
				}
			} else {
				m_stat_wait_timeout++;
			}
		} else {
			// Read gyro and accel output registers: 12 bytes starting at OUTX_L_G.
			res = read_regs(LSM6DSV32X_OUTX_L_G, rxb, 12);
		}

		if (!res) {
			if (m_use_spi) {
				recover_spi_stream();
			} else {
				i2c_bb_restore_bus(m_i2c_bb);
			}

			if (reset_init_lsm6dsv32x()) {
				m_stat_reset_ok++;
				if (m_use_spi && m_use_int1) {
					prepare_stream_read();
					m_spi_stream_enabled = true;
				}
			} else {
				m_stat_reset_fail++;
			}

			chThdSleepMilliseconds(10);

			iteration_timer = chVTGetSystemTimeX();
			continue;
		}

		// Parse gyro raw values. Output is low byte first, high byte second.
		int16_t raw_gx = (int16_t)(((uint16_t)rxb[1] << 8) | rxb[0]);
		int16_t raw_gy = (int16_t)(((uint16_t)rxb[3] << 8) | rxb[2]);
		int16_t raw_gz = (int16_t)(((uint16_t)rxb[5] << 8) | rxb[4]);

		// Parse accel raw values. Output is low byte first, high byte second.
		int16_t raw_ax = (int16_t)(((uint16_t)rxb[7] << 8) | rxb[6]);
		int16_t raw_ay = (int16_t)(((uint16_t)rxb[9] << 8) | rxb[8]);
		int16_t raw_az = (int16_t)(((uint16_t)rxb[11] << 8) | rxb[10]);

		systime_t sample_time = chVTGetSystemTimeX();
		chSysLock();
		if (m_stat_last_sample_time != 0) {
			uint32_t sample_dt = sample_time - m_stat_last_sample_time;
			if (sample_dt < m_stat_min_sample_dt) {
				m_stat_min_sample_dt = sample_dt;
			}

			if (sample_dt > m_stat_max_sample_dt) {
				m_stat_max_sample_dt = sample_dt;
			}

			int16_t raw_now[6] = {
					raw_gx, raw_gy, raw_gz, raw_ax, raw_ay, raw_az,
			};

			for (int i = 0; i < 6; i++) {
				int32_t delta = (int32_t)raw_now[i] - (int32_t)m_stat_last_raw[i];
				if (delta < 0) {
					delta = -delta;
				}

				if (delta > m_stat_max_raw_delta[i]) {
					m_stat_max_raw_delta[i] = delta > 32767 ? 32767 : (int16_t)delta;
				}
			}
		}
		m_stat_last_sample_time = sample_time;
		m_stat_samples++;
		m_stat_last_raw[0] = raw_gx;
		m_stat_last_raw[1] = raw_gy;
		m_stat_last_raw[2] = raw_gz;
		m_stat_last_raw[3] = raw_ax;
		m_stat_last_raw[4] = raw_ay;
		m_stat_last_raw[5] = raw_az;
		chSysUnlock();

		// Gyro: +-2000 dps, sensitivity = 70.0 mdps/LSB.
		float gx = (float)raw_gx * 70.0f / 1000.0f;
		float gy = (float)raw_gy * 70.0f / 1000.0f;
		float gz = (float)raw_gz * 70.0f / 1000.0f;

		// Accel: +-32 g, sensitivity = 0.976 mg/LSB.
		float ax = (float)raw_ax * 0.976f / 1000.0f;
		float ay = (float)raw_ay * 0.976f / 1000.0f;
		float az = (float)raw_az * 0.976f / 1000.0f;

		if (read_callback) {
			float tmp_accel[3] = { ax, ay, az };
			float tmp_gyro[3] = { gx, gy, gz };

			// LSM6DSV32X has no magnetometer.
			float tmp_mag[3] = { 0.0f, 0.0f, 0.0f };

			read_callback(tmp_accel, tmp_gyro, tmp_mag);
		}

		if (m_use_int1) {
			continue;
		}

		// Polling-mode delay between loops.
		iteration_timer += desired_interval;

		systime_t current_time = chVTGetSystemTimeX();
		systime_t remaining_sleep_time = iteration_timer - current_time;

		if (remaining_sleep_time > 0 && remaining_sleep_time < desired_interval) {
			chThdSleep(remaining_sleep_time);
		} else {
			iteration_timer = current_time;
			chThdSleep(desired_interval);
		}
	}
}
