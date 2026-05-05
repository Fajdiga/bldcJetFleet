/*
	Copyright 2024 Contributors

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

#include "lsm6dsv32x.h"
#include "terminal.h"
#include "i2c_bb.h"
#include "spi_bb.h"
#include "commands.h"
#include "utils_math.h"

#include <stdio.h>


static thread_t *lsm6dsv32x_thread_ref = NULL;
static binary_semaphore_t m_drdy_sem;
static bool m_drdy_sem_init = false;
static bool m_use_int1 = false;
static i2c_bb_state *m_i2c_bb;
static SPIDriver *m_spi_dev = NULL;
static stm32_gpio_t *m_nss_gpio;
static int m_nss_pin;
static bool m_use_spi = false;
static volatile uint16_t lsm6dsv32x_addr;
static int rate_hz = 1000;
static IMU_FILTER filter;

// SPI mode 3 (CPOL=1, CPHA=1), baud prescaler /8 (~5.25 MHz on SPI3 @ 42 MHz APB1)
static const SPIConfig m_spi_cfg = {
	.end_cb = NULL,
	.ssport = NULL,
	.sspad = 0,
	.cr1 = SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA,

};

static bool reset_init_lsm6dsv32x(void);
static void terminal_read_reg(int argc, const char **argv);
static void terminal_write_reg(int argc, const char **argv);
static uint8_t read_single_reg(uint8_t reg);
static bool write_single_reg(uint8_t reg, uint8_t value);
static bool read_regs(uint8_t reg, uint8_t *data, int len);
static THD_FUNCTION(lsm6dsv32x_thread, arg);

// Function pointers
static void(*read_callback)(float *accel, float *gyro, float *mag) = 0;


void lsm6dsv32x_set_rate_hz(int hz) {
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
		chBSemObjectInit(&m_drdy_sem, true);
		m_drdy_sem_init = true;
	}

	palSetPad(m_nss_gpio, m_nss_pin);
	spiStart(m_spi_dev, &m_spi_cfg);

	// Verify WHO_AM_I
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

	lsm6dsv32x_thread_ref = chThdCreateStatic(work_area, work_area_size, NORMALPRIO, lsm6dsv32x_thread, NULL);
}

void lsm6dsv32x_init(i2c_bb_state *i2c_state,
		stkalign_t *work_area, size_t work_area_size) {

	read_callback = 0;
	m_use_spi = false;
	m_use_int1 = false;
	m_i2c_bb = i2c_state;

	// Recover I2C bus in case it is stuck
	i2c_bb_restore_bus(m_i2c_bb);
	chThdSleepMilliseconds(1);

	// Detect I2C address
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

	lsm6dsv32x_thread_ref = chThdCreateStatic(work_area, work_area_size, NORMALPRIO, lsm6dsv32x_thread, NULL);
}

static bool reset_init_lsm6dsv32x(void) {
	// Software reset
	if (!write_single_reg(LSM6DSV32X_CTRL3, LSM6DSV32X_SW_RESET)) {
		return false;
	}
	chThdSleepMilliseconds(10);

	// Enable BDU and IF_INC (auto-increment for multi-byte reads)
	if (!write_single_reg(LSM6DSV32X_CTRL3, LSM6DSV32X_BDU | LSM6DSV32X_IF_INC)) {
		return false;
	}

	// Set accel full-scale to +-32g (FS_XL=11 on 32X, sensitivity = 0.976 mg/LSB)
	// Bit 2 (XL_FS_MODE) must be set to 1 for correct operation of LSM6DSV32X
	uint8_t ctrl8_val = LSM6DSV32X_XL_FS_MODE | LSM6DSV32X_FS_XL_32g;
	if (filter == IMU_FILTER_HIGH) {
		ctrl8_val |= LSM6DSV32X_XL_HP_BW_ODR_4;
	}
	if (!write_single_reg(LSM6DSV32X_CTRL8, ctrl8_val)) {
		return false;
	}

	// Enable accel LPF2 only in high filter mode. The enable bit is in CTRL9;
	// CTRL8 only selects the cutoff ratio.
	uint8_t ctrl9_val = 0;
	if (filter == IMU_FILTER_HIGH) {
		ctrl9_val |= LSM6DSV32X_XL_LPF2_EN;
	}
	if (!write_single_reg(LSM6DSV32X_CTRL9, ctrl9_val)) {
		return false;
	}

	// Set gyro full-scale to +-2000dps (CTRL6)
	uint8_t ctrl6_val = LSM6DSV32X_FS_G_2000dps;
	if (filter >= IMU_FILTER_MEDIUM) {
		ctrl6_val |= LSM6DSV32X_G_LPF1_BW_3;
	}
	if (!write_single_reg(LSM6DSV32X_CTRL6, ctrl6_val)) {
		return false;
	}

	// Enable gyro LPF1 if filter is medium or higher (CTRL7)
	uint8_t ctrl7_val = 0;
	if (filter >= IMU_FILTER_MEDIUM) {
		ctrl7_val |= LSM6DSV32X_G_LPF1_EN;
	}
	if (!write_single_reg(LSM6DSV32X_CTRL7, ctrl7_val)) {
		return false;
	}

	// Determine accel ODR from the requested sample rate. Filter settings must
	// not change ODR, otherwise DRDY/callback frequency changes unexpectedly.
	uint8_t odr_xl = LSM6DSV32X_ODR_XL_960Hz;
	if (rate_hz <= 8) {
		odr_xl = LSM6DSV32X_ODR_XL_7_5Hz;
	} else if (rate_hz <= 15) {
		odr_xl = LSM6DSV32X_ODR_XL_15Hz;
	} else if (rate_hz <= 30) {
		odr_xl = LSM6DSV32X_ODR_XL_30Hz;
	} else if (rate_hz <= 60) {
		odr_xl = LSM6DSV32X_ODR_XL_60Hz;
	} else if (rate_hz <= 120) {
		odr_xl = LSM6DSV32X_ODR_XL_120Hz;
	} else if (rate_hz <= 240) {
		odr_xl = LSM6DSV32X_ODR_XL_240Hz;
	} else if (rate_hz <= 480) {
		odr_xl = LSM6DSV32X_ODR_XL_480Hz;
	} else if (rate_hz <= 960) {
		odr_xl = LSM6DSV32X_ODR_XL_960Hz;
	} else if (rate_hz <= 1920) {
		odr_xl = LSM6DSV32X_ODR_XL_1920Hz;
	} else if (rate_hz <= 3840) {
		odr_xl = LSM6DSV32X_ODR_XL_3840Hz;
	} else {
		odr_xl = LSM6DSV32X_ODR_XL_7680Hz;
	}
	if (!write_single_reg(LSM6DSV32X_CTRL1, LSM6DSV32X_XL_MODE_HIGH_PERF | odr_xl)) {
		return false;
	}

	// Determine gyro ODR from the requested sample rate. INT1 is routed from
	// gyro DRDY, so this directly controls the interrupt frequency.
	uint8_t odr_g = LSM6DSV32X_ODR_G_960Hz;
	if (rate_hz <= 8) {
		odr_g = LSM6DSV32X_ODR_G_7_5Hz;
	} else if (rate_hz <= 15) {
		odr_g = LSM6DSV32X_ODR_G_15Hz;
	} else if (rate_hz <= 30) {
		odr_g = LSM6DSV32X_ODR_G_30Hz;
	} else if (rate_hz <= 60) {
		odr_g = LSM6DSV32X_ODR_G_60Hz;
	} else if (rate_hz <= 120) {
		odr_g = LSM6DSV32X_ODR_G_120Hz;
	} else if (rate_hz <= 240) {
		odr_g = LSM6DSV32X_ODR_G_240Hz;
	} else if (rate_hz <= 480) {
		odr_g = LSM6DSV32X_ODR_G_480Hz;
	} else if (rate_hz <= 960) {
		odr_g = LSM6DSV32X_ODR_G_960Hz;
	} else if (rate_hz <= 1920) {
		odr_g = LSM6DSV32X_ODR_G_1920Hz;
	} else if (rate_hz <= 3840) {
		odr_g = LSM6DSV32X_ODR_G_3840Hz;
	} else {
		odr_g = LSM6DSV32X_ODR_G_7680Hz;
	}
	if (!write_single_reg(LSM6DSV32X_CTRL2, LSM6DSV32X_G_MODE_HIGH_PERF | odr_g)) {
		return false;
	}

	// Route gyro data-ready to INT1 when interrupt-driven mode is in use
	if (m_use_int1) {
		if (!write_single_reg(LSM6DSV32X_INT1_CTRL, LSM6DSV32X_INT1_DRDY_G)) {
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

static bool read_regs(uint8_t reg, uint8_t *data, int len) {
	if (m_use_spi) {
		palClearPad(m_nss_gpio, m_nss_pin);
		spiPolledExchange(m_spi_dev, reg | LSM6DSV32X_SPI_RD_MASK);
		for (int i = 0; i < len; i++) {
			data[i] = spiPolledExchange(m_spi_dev, 0);
		}
		palSetPad(m_nss_gpio, m_nss_pin);
		return true;
	}

	uint8_t txb[1] = { reg };
	return i2c_bb_tx_rx(m_i2c_bb, lsm6dsv32x_addr, txb, 1, data, len);
}

static bool write_single_reg(uint8_t reg, uint8_t value) {
	if (m_use_spi) {
		palClearPad(m_nss_gpio, m_nss_pin);
		spiPolledExchange(m_spi_dev, reg & LSM6DSV32X_SPI_WR_MASK);
		spiPolledExchange(m_spi_dev, value);
		palSetPad(m_nss_gpio, m_nss_pin);
		return true;
	}

	uint8_t txb[2] = { reg, value };
	return i2c_bb_tx_rx(m_i2c_bb, lsm6dsv32x_addr, txb, 2, 0, 0);
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
	const systime_t desired_interval = US2ST(1000000 / rate_hz);
	// Watchdog timeout for INT1 wait: 4x the expected sample period, min 5ms
	const systime_t int1_timeout = MS2ST(5) > (desired_interval * 4) ? MS2ST(5) : (desired_interval * 4);

	while (!chThdShouldTerminateX()) {
		uint8_t rxb[12];

		if (m_use_int1) {
			// Block until DRDY EXTI fires (or timeout to recover from missed pulses)
			chBSemWaitTimeout(&m_drdy_sem, int1_timeout);
		}

		// Read gyro and accel output registers (12 bytes starting at OUTX_L_G)
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

		// Parse gyro: +-2000dps, sensitivity = 70.0 mdps/LSB
		float gx = (float)((int16_t)((uint16_t)rxb[1] << 8) + rxb[0]) * 70.0f / 1000.0f;
		float gy = (float)((int16_t)((uint16_t)rxb[3] << 8) + rxb[2]) * 70.0f / 1000.0f;
		float gz = (float)((int16_t)((uint16_t)rxb[5] << 8) + rxb[4]) * 70.0f / 1000.0f;

		// Parse accel: +-32g, sensitivity = 0.976 mg/LSB
		float ax = (float)((int16_t)((uint16_t)rxb[7] << 8) + rxb[6]) * 0.976f / 1000.0f;
		float ay = (float)((int16_t)((uint16_t)rxb[9] << 8) + rxb[8]) * 0.976f / 1000.0f;
		float az = (float)((int16_t)((uint16_t)rxb[11] << 8) + rxb[10]) * 0.976f / 1000.0f;

		if (read_callback) {
			float tmp_accel[3] = {ax, ay, az};
			float tmp_gyro[3] = {gx, gy, gz};
			float tmp_mag[3] = {1, 2, 3};
			read_callback(tmp_accel, tmp_gyro, tmp_mag);
		}

		if (m_use_int1) {
			continue;
		}

		// Polling-mode delay between loops
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
