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

// Default rate. Can be changed before init with lsm6dsv32x_set_rate_hz().
static int rate_hz = 1000;

// Explicit default. If your enum has IMU_FILTER_LOW, this can be changed to:
// static IMU_FILTER filter = IMU_FILTER_LOW;
static IMU_FILTER filter = (IMU_FILTER)0;

static void spi_end_cb(SPIDriver *spi_dev);
static bool spi_transfer(const uint8_t *txb, uint8_t *rxb, size_t len);

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

// Hardware SPI init. Call this from imu_init_lsm6dsv32x_spi which configures pin AFs.
void lsm6dsv32x_int1_isr(void) {
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

	lsm6dsv32x_thread_ref = chThdCreateStatic(work_area, work_area_size,
			NORMALPRIO, lsm6dsv32x_thread, NULL);
}

static bool reset_init_lsm6dsv32x(void) {
	// Software reset.
	if (!write_config_reg(LSM6DSV32X_CTRL3, LSM6DSV32X_SW_RESET, "Software Reset")) {
		return false;
	}

	chThdSleepMilliseconds(10);

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
		spiStop(m_spi_dev);
		m_spi_dev = NULL;
		m_use_spi = false;
	}

	lsm6dsv32x_thread_ref = NULL;

	terminal_unregister_callback(terminal_read_reg);
	terminal_unregister_callback(terminal_write_reg);
}

void lsm6dsv32x_set_read_callback(void(*func)(float *accel, float *gyro, float *mag)) {
	read_callback = func;
}

static void spi_end_cb(SPIDriver *spi_dev) {
	(void)spi_dev;

	if (m_nss_gpio != NULL) {
		palSetPad(m_nss_gpio, m_nss_pin);
	}

	if (m_spi_sem_init) {
		chSysLockFromISR();
		chBSemSignalI(&m_spi_sem);
		chSysUnlockFromISR();
	}
}

static bool spi_transfer(const uint8_t *txb, uint8_t *rxb, size_t len) {
	if (m_spi_dev == NULL || txb == NULL || rxb == NULL || len == 0 || !m_spi_sem_init) {
		return false;
	}

	chBSemReset(&m_spi_sem, true);
	spiAcquireBus(m_spi_dev);

	// Same DMA/RXNE guard used by the async encoder SPI drivers.
	volatile uint32_t rxne_clear = m_spi_dev->spi->DR;
	(void)rxne_clear;

	palClearPad(m_nss_gpio, m_nss_pin);

	chSysLock();
	spiStartExchangeI(m_spi_dev, len, txb, rxb);
	chSysUnlock();

	msg_t msg = chBSemWait(&m_spi_sem);
	spiReleaseBus(m_spi_dev);

	return msg == MSG_OK;
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

		if (m_use_int1) {
			/*
			 * Wait for gyro DRDY interrupt.
			 * Then immediately read gyro + accel output registers in one SPI burst.
			 *
			 * No STATUS_REG polling and no FIFO here to keep latency and CPU load low.
			 */
			chBSemWaitTimeout(&m_drdy_sem, int1_timeout);
		}

		// Read gyro and accel output registers: 12 bytes starting at OUTX_L_G.
		bool res = read_regs(LSM6DSV32X_OUTX_L_G, rxb, 12);

		if (!res) {
			if (!m_use_spi) {
				i2c_bb_restore_bus(m_i2c_bb);
			}

			reset_init_lsm6dsv32x();
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
