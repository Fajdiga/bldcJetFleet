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

#include "lsm6dsv32x.h"
#include "imu_config.h"
#include "commands.h"
#include "terminal.h"
#include "transport_spi_hw.h"

#include <stdio.h>
#include <string.h>

#define LSM6DSV32X_ADDR_A		0x6A
#define LSM6DSV32X_ADDR_B		0x6B

#define REG_IF_CFG				0x03
#define REG_INT1_CTRL			0x0D
#define REG_WHO_AM_I			0x0F
#define REG_CTRL1				0x10
#define REG_CTRL2				0x11
#define REG_CTRL3				0x12
#define REG_CTRL4				0x13
#define REG_CTRL6				0x15
#define REG_CTRL7				0x16
#define REG_CTRL8				0x17
#define REG_CTRL9				0x18
#define REG_OUTX_L_G			0x22

#define WHO_AM_I_VAL			0x70
#define IF_CFG_I2C_I3C_DISABLE	(1 << 0)
#define CTRL3_SW_RESET			(1 << 0)
#define CTRL3_IF_INC				(1 << 2)
#define CTRL3_BDU				(1 << 6)
#define CTRL4_DRDY_MASK			(1 << 3)
#define CTRL7_LPF1_G_EN		(1 << 0)
#define CTRL8_MUST_SET			(1 << 2)
#define CTRL9_LPF2_XL_EN		(1 << 3)
#define INT1_DRDY_G				(1 << 1)

#define FS_G_4000DPS			0x0C
#define FS_G_2000DPS			0x04
#define FS_XL_32G				0x03
#define GYRO_LSB_4000DPS	(140.0f / 1000.0f)
#define GYRO_LSB_2000DPS	(70.0f / 1000.0f)
#define ACCEL_LSB_32G		(0.976f / 1000.0f)

#define ASYNC_BURST_LEN		12
#define ASYNC_TIMEOUTS_BEFORE_RESET 3

static const struct { uint16_t hz; uint8_t code; } odr_ladder[] = {
	{8, 0x2}, {15, 0x3}, {30, 0x4}, {60, 0x5}, {120, 0x6}, {240, 0x7},
	{480, 0x8}, {960, 0x9}, {1920, 0xA}, {3840, 0xB}, {7680, 0xC},
};
#define ODR_LADDER_N (sizeof(odr_ladder) / sizeof(odr_ladder[0]))

typedef struct {
	IMU_FILTER filter;
	volatile bool stream_enabled;
	volatile bool pending;
	volatile bool fallback_active;
	volatile bool fallback_ready;
	uint8_t timeout_streak;
	volatile uint32_t drdy;
	volatile uint32_t ignored_active;
	volatile uint32_t ignored_complete;
	volatile uint32_t ignored_sync;
	volatile uint32_t pending_latched;
	volatile uint32_t pending_restarted;
	volatile uint32_t pending_restart_failed;
	volatile uint32_t overruns;
	volatile uint32_t stream_started;
	volatile uint32_t stream_completed;
	volatile uint32_t stream_copied;
	volatile uint32_t copy_failed;
	volatile uint32_t timeouts;
	volatile uint32_t timeout_fallbacks;
	volatile uint32_t fallback_completed;
	volatile uint32_t fallback_failed;
	volatile uint32_t recoveries;
	volatile uint32_t reset_ok;
	volatile uint32_t reset_fail;
	volatile uint32_t samples;
	volatile uint32_t last_sample_time;
	volatile uint32_t min_sample_dt;
	volatile uint32_t max_sample_dt;
	volatile int16_t last_raw[6];
	volatile int16_t max_raw_delta[6];
} lsm6dsv32x_async_state_t;

static lsm6dsv32x_async_state_t m_async = {
	.min_sample_dt = 0xFFFFFFFF,
};
static bool m_stats_registered = false;

static bool read_reg(imu_device_t *dev, uint8_t reg, uint8_t *res) {
	return transport_read_reg(dev->transport, dev->dev_addr, reg, res, 1);
}

static bool write_reg(imu_device_t *dev, uint8_t reg, uint8_t value) {
	return transport_write_reg(dev->transport, dev->dev_addr, reg, &value, 1);
}

static bool probe(imu_device_t *dev) {
	uint8_t id = 0;
	dev->dev_addr = LSM6DSV32X_ADDR_A;
	bool ok = read_reg(dev, REG_WHO_AM_I, &id);
	if (!ok || id != WHO_AM_I_VAL) {
		commands_printf("LSM6DSV32X address A failed, trying B (rx: %d)", id);
		dev->dev_addr = LSM6DSV32X_ADDR_B;
		ok = read_reg(dev, REG_WHO_AM_I, &id);
	}
	if (!ok || id != WHO_AM_I_VAL) {
		commands_printf("LSM6DSV32X probe failed (rx: %d)", id);
	}
	return ok && id == WHO_AM_I_VAL;
}

static uint8_t odr_from_rate(uint16_t requested, uint16_t *actual_hz) {
	uint8_t i = 0;
	while (i < ODR_LADDER_N - 1 && odr_ladder[i].hz < requested) {
		i++;
	}
	if (actual_hz) {
		*actual_hz = odr_ladder[i].hz;
	}
	return odr_ladder[i].code;
}

static uint8_t g18_accel_lpf2_bw(IMU_FILTER filter, uint16_t odr_hz) {
	switch (filter) {
	case IMU_FILTER_LOW:
		return odr_hz >= 3840 ? 0x20 : 0x00; // ODR/10 or ODR/4
	case IMU_FILTER_MEDIUM:
		return 0x40; // ODR/20
	case IMU_FILTER_HIGH:
	default:
		return 0x60; // ODR/45
	}
}

static uint8_t g18_gyro_lpf1_bw(IMU_FILTER filter) {
	switch (filter) {
	case IMU_FILTER_LOW:
		return 0x30;
	case IMU_FILTER_MEDIUM:
		return 0x10;
	case IMU_FILTER_HIGH:
	default:
		return 0x20;
	}
}

static void reset_stats(void) {
	m_async.drdy = 0;
	m_async.ignored_active = 0;
	m_async.ignored_complete = 0;
	m_async.ignored_sync = 0;
	m_async.pending_latched = 0;
	m_async.pending_restarted = 0;
	m_async.pending_restart_failed = 0;
	m_async.overruns = 0;
	m_async.stream_started = 0;
	m_async.stream_completed = 0;
	m_async.stream_copied = 0;
	m_async.copy_failed = 0;
	m_async.timeouts = 0;
	m_async.timeout_fallbacks = 0;
	m_async.fallback_completed = 0;
	m_async.fallback_failed = 0;
	m_async.recoveries = 0;
	m_async.reset_ok = 0;
	m_async.reset_fail = 0;
	m_async.samples = 0;
	m_async.last_sample_time = 0;
	m_async.min_sample_dt = 0xFFFFFFFF;
	m_async.max_sample_dt = 0;
	for (int i = 0; i < 6; i++) {
		m_async.last_raw[i] = 0;
		m_async.max_raw_delta[i] = 0;
	}
}

static void note_sample(const int16_t raw[6]) {
	uint32_t now = chVTGetSystemTimeX();
	bool had_previous = m_async.last_sample_time != 0;
	if (had_previous) {
		uint32_t dt = now - m_async.last_sample_time;
		if (dt < m_async.min_sample_dt) {
			m_async.min_sample_dt = dt;
		}
		if (dt > m_async.max_sample_dt) {
			m_async.max_sample_dt = dt;
		}
	}
	m_async.last_sample_time = now;
	m_async.samples++;

	for (int i = 0; i < 6; i++) {
		if (had_previous) {
			int32_t delta = (int32_t)raw[i] - (int32_t)m_async.last_raw[i];
			if (delta < 0) {
				delta = -delta;
			}
			if (delta > m_async.max_raw_delta[i]) {
				m_async.max_raw_delta[i] = delta > 32767 ? 32767 : (int16_t)delta;
			}
		}
		m_async.last_raw[i] = raw[i];
	}
}

static bool decode_sample(imu_device_t *dev, const uint8_t raw_bytes[ASYNC_BURST_LEN],
		float accel[3], float gyro[3], float mag[3]) {
	int16_t raw[6] = {
		(int16_t)((raw_bytes[1] << 8) | raw_bytes[0]),
		(int16_t)((raw_bytes[3] << 8) | raw_bytes[2]),
		(int16_t)((raw_bytes[5] << 8) | raw_bytes[4]),
		(int16_t)((raw_bytes[7] << 8) | raw_bytes[6]),
		(int16_t)((raw_bytes[9] << 8) | raw_bytes[8]),
		(int16_t)((raw_bytes[11] << 8) | raw_bytes[10]),
	};

	const float gyro_scale = dev->use_async ? GYRO_LSB_2000DPS : GYRO_LSB_4000DPS;
	gyro[0] = raw[0] * gyro_scale;
	gyro[1] = raw[1] * gyro_scale;
	gyro[2] = raw[2] * gyro_scale;
	accel[0] = raw[3] * ACCEL_LSB_32G;
	accel[1] = raw[4] * ACCEL_LSB_32G;
	accel[2] = raw[5] * ACCEL_LSB_32G;
	mag[0] = 0.0f;
	mag[1] = 0.0f;
	mag[2] = 0.0f;

	if (dev->use_async) {
		note_sample(raw);
	}
	return true;
}

static bool configure_standard(imu_device_t *dev, IMU_FILTER filter) {
	if (!probe(dev)) {
		return false;
	}

	uint16_t odr_hz = dev->sample_rate_hz;
	uint8_t odr = odr_from_rate(dev->sample_rate_hz, &odr_hz);
	if (dev->use_drdy) {
		dev->sample_rate_hz = odr_hz;
	} else {
		// Polling uses the maximum ODR to provide a fresh sample at each timed read.
		odr = odr_ladder[ODR_LADDER_N - 1].code;
		odr_hz = odr_ladder[ODR_LADDER_N - 1].hz;
	}

	uint8_t div = filter == IMU_FILTER_HIGH ? 8 : filter == IMU_FILTER_MEDIUM ? 4 : 2;
	uint16_t cutoff = dev->sample_rate_hz / div;
	bool lpf2_en = odr_hz / 2 > cutoff;
	uint8_t lpf2_bw = 0;
	if (lpf2_en) {
		static const uint16_t lpf2_n[8] = {4, 10, 20, 45, 100, 200, 400, 800};
		lpf2_bw = 7;
		for (uint8_t code = 0; code < 8; code++) {
			if (odr_hz / lpf2_n[code] <= cutoff) {
				lpf2_bw = code;
				break;
			}
		}
	}

	bool lpf1_en = filter != IMU_FILTER_LOW;
	uint8_t lpf1_bw = filter == IMU_FILTER_HIGH ? 0x2 : 0x0;
	bool ok = write_reg(dev, REG_CTRL3, CTRL3_BDU | CTRL3_IF_INC);
	ok = ok && write_reg(dev, REG_CTRL8, (lpf2_bw << 5) | CTRL8_MUST_SET | FS_XL_32G);
	ok = ok && write_reg(dev, REG_CTRL9, lpf2_en ? CTRL9_LPF2_XL_EN : 0);
	ok = ok && write_reg(dev, REG_CTRL6, (lpf1_bw << 4) | FS_G_4000DPS);
	ok = ok && write_reg(dev, REG_CTRL7, lpf1_en ? CTRL7_LPF1_G_EN : 0);
	ok = ok && write_reg(dev, REG_CTRL4, dev->use_drdy ? CTRL4_DRDY_MASK : 0);
	ok = ok && write_reg(dev, REG_CTRL1, odr);
	ok = ok && write_reg(dev, REG_CTRL2, odr);
	return ok;
}

static bool configure_async(imu_device_t *dev, IMU_FILTER filter) {
	if (!probe(dev)) {
		return false;
	}

	bool ok = write_reg(dev, REG_CTRL3, CTRL3_SW_RESET);
	if (!ok) {
		return false;
	}
	chThdSleepMilliseconds(10);

	uint16_t odr_hz = dev->sample_rate_hz;
	uint8_t odr = odr_from_rate(dev->sample_rate_hz, &odr_hz);
	dev->sample_rate_hz = odr_hz;

	ok = write_reg(dev, REG_IF_CFG, IF_CFG_I2C_I3C_DISABLE);
	ok = ok && write_reg(dev, REG_CTRL3, CTRL3_BDU | CTRL3_IF_INC);
	ok = ok && write_reg(dev, REG_CTRL8,
			g18_accel_lpf2_bw(filter, odr_hz) | CTRL8_MUST_SET | FS_XL_32G);
	ok = ok && write_reg(dev, REG_CTRL9, CTRL9_LPF2_XL_EN);
	ok = ok && write_reg(dev, REG_CTRL6, FS_G_2000DPS | g18_gyro_lpf1_bw(filter));
	ok = ok && write_reg(dev, REG_CTRL7, CTRL7_LPF1_G_EN);
	// Do not mask DRDY during filter settling on the interrupt-driven G18 path.
	// The worker's short missed-edge timeout can otherwise reset the sensor before
	// settling completes, permanently preventing the first INT1 edge.
	ok = ok && write_reg(dev, REG_CTRL4, 0);
	ok = ok && write_reg(dev, REG_CTRL1, odr);
	ok = ok && write_reg(dev, REG_CTRL2, odr);
	if (!ok) {
		commands_printf("LSM6DSV32X G18 configuration failed");
		return false;
	}

	m_async.filter = filter;
	m_async.stream_enabled = false;
	m_async.pending = false;
	m_async.fallback_active = false;
	m_async.fallback_ready = false;
	m_async.timeout_streak = 0;
	return true;
}

static bool configure(imu_device_t *dev, IMU_FILTER filter, bool use_mag) {
	(void)use_mag; // LSM6DSV32X is a 6-axis device.
	return dev->use_async ? configure_async(dev, filter) : configure_standard(dev, filter);
}

static bool read_sample(imu_device_t *dev, float accel[3], float gyro[3], float mag[3]) {
	uint8_t raw[ASYNC_BURST_LEN];
	if (!transport_read_reg(dev->transport, dev->dev_addr, REG_OUTX_L_G, raw, sizeof(raw))) {
		return false;
	}
	return decode_sample(dev, raw, accel, gyro, mag);
}

static bool async_supported(imu_device_t *dev) {
#ifdef IMU_ASYNC_DMA
	return IMU_ASYNC_DMA && transport_spi_hw_async_supported(dev->transport);
#else
	(void)dev;
	return false;
#endif
}

static bool async_start_sample(imu_device_t *dev, bool from_isr) {
	if (!from_isr) {
		// Timeout recovery deliberately leaves streaming disabled until the
		// worker has drained its semaphore and successfully launched this DMA.
		if (!m_async.fallback_ready || m_async.stream_enabled || m_async.fallback_active ||
				transport_spi_hw_async_busy(dev->transport) ||
				!transport_spi_hw_async_start_read(dev->transport, REG_OUTX_L_G,
						ASYNC_BURST_LEN, false)) {
			return false;
		}
		m_async.fallback_ready = false;
		m_async.fallback_active = true;
		m_async.timeout_fallbacks++;
		m_async.stream_started++;
		m_async.stream_enabled = true;
		return true;
	}

	if (!m_async.stream_enabled) {
		return false;
	}

	m_async.drdy++;
	if (transport_spi_hw_async_busy(dev->transport)) {
		if (transport_spi_hw_async_active(dev->transport)) {
			m_async.ignored_active++;
		} else {
			m_async.ignored_complete++;
		}
		if (!m_async.pending) {
			m_async.pending = true;
			m_async.pending_latched++;
		} else {
			m_async.overruns++;
		}
		return false;
	}

	if (!transport_spi_hw_async_start_read(dev->transport, REG_OUTX_L_G,
			ASYNC_BURST_LEN, true)) {
		// Preserve a single edge while a synchronous command/configuration transfer
		// owns SPI. The next DRDY starts the normal read, whose completion consumes
		// this latch immediately afterwards.
		m_async.ignored_sync++;
		if (!m_async.pending) {
			m_async.pending = true;
			m_async.pending_latched++;
		} else {
			m_async.overruns++;
		}
		return false;
	}

	m_async.stream_started++;
	return true;
}

static bool async_take_sample(imu_device_t *dev, float accel[3], float gyro[3], float mag[3]) {
	uint8_t raw[ASYNC_BURST_LEN];
	bool was_fallback = m_async.fallback_active;
	if (!transport_spi_hw_async_copy_read(dev->transport, raw, sizeof(raw))) {
		m_async.copy_failed++;
		if (was_fallback) {
			m_async.fallback_failed++;
			m_async.fallback_active = false;
		}
		return false;
	}
	m_async.fallback_active = false;

	m_async.stream_completed++;
	m_async.stream_copied++;
	chSysLock();
	bool restart_pending = m_async.pending;
	m_async.pending = false;
	chSysUnlock();
	if (restart_pending) {
		if (transport_spi_hw_async_start_read(dev->transport, REG_OUTX_L_G, ASYNC_BURST_LEN, false)) {
			m_async.pending_restarted++;
			m_async.stream_started++;
		} else {
			m_async.pending_restart_failed++;
		}
	}

	bool ok = decode_sample(dev, raw, accel, gyro, mag);
	if (ok) {
		if (was_fallback) {
			m_async.fallback_completed++;
		} else {
			// Only a DRDY-originated completion proves that the interrupt is alive.
			m_async.timeout_streak = 0;
		}
	} else if (was_fallback) {
		m_async.fallback_failed++;
	}
	return ok;
}

static bool async_timeout(imu_device_t *dev) {
	m_async.timeouts++;
	m_async.timeout_streak++;
	m_async.stream_enabled = false;
	m_async.pending = false;
	m_async.fallback_active = false;
	m_async.fallback_ready = false;
	// Quiesce the old generation before the worker drains its completion
	// semaphore and starts the replacement DMA transaction. active/complete
	// publication has no false/false window, so an idle transport needs no
	// disruptive SPI stop/start merely because the DRDY edge was absent.
	if (transport_spi_hw_async_busy(dev->transport)) {
		transport_spi_hw_async_abort(dev->transport);
	}

	if (m_async.timeout_streak >= ASYNC_TIMEOUTS_BEFORE_RESET) {
		m_async.recoveries++;
		m_async.timeout_streak = 0;
		if (configure_async(dev, m_async.filter) &&
				write_reg(dev, REG_INT1_CTRL, INT1_DRDY_G)) {
			m_async.reset_ok++;
			m_async.fallback_ready = true;
			return true;
		} else {
			m_async.reset_fail++;
			return false;
		}
	}

	// The sensor interrupt remains configured, but streaming stays disabled
	// until the worker has reset the semaphore and launched its fallback DMA.
	m_async.fallback_ready = true;
	return true;
}

static void on_read_fail(imu_device_t *dev) {
	if (!dev->use_async) {
		transport_recover(dev->transport);
		return;
	}

	m_async.recoveries++;
	m_async.pending = false;
	m_async.stream_enabled = false;
	m_async.fallback_ready = false;
	if (m_async.fallback_active) {
		m_async.fallback_failed++;
		m_async.fallback_active = false;
	}
	transport_spi_hw_async_abort(dev->transport);
	if (configure_async(dev, m_async.filter) &&
			write_reg(dev, REG_INT1_CTRL, INT1_DRDY_G)) {
		m_async.reset_ok++;
		m_async.fallback_ready = true;
	} else {
		m_async.reset_fail++;
	}
}

static void enable_drdy_output(imu_device_t *dev, bool enable) {
	if (!write_reg(dev, REG_INT1_CTRL, enable ? INT1_DRDY_G : 0)) {
		commands_printf("LSM6DSV32X DRDY register write failed");
		return;
	}

	if (dev->use_async) {
		m_async.stream_enabled = enable;
		m_async.pending = false;
		m_async.fallback_active = false;
		m_async.fallback_ready = false;
	}
}

static void async_stop(imu_device_t *dev) {
	(void)dev;
	m_async.stream_enabled = false;
	m_async.pending = false;
	m_async.fallback_active = false;
	m_async.fallback_ready = false;
}

static void terminal_stats(int argc, const char **argv) {
	if (argc == 2 && strcmp(argv[1], "reset") == 0) {
		reset_stats();
		commands_printf("LSM6DSV32X async statistics reset");
		return;
	}
	if (argc != 1) {
		commands_printf("Usage: lsm6dsv32x_stats [reset]");
		return;
	}

	commands_printf("LSM6DSV32X async: drdy=%u started=%u completed=%u copied=%u samples=%u",
			m_async.drdy, m_async.stream_started, m_async.stream_completed,
			m_async.stream_copied, m_async.samples);
	commands_printf("  state stream=%u pending=%u fallback=%u ready=%u timeout_streak=%u age_ms=%.3f",
			m_async.stream_enabled, m_async.pending, m_async.fallback_active,
			m_async.fallback_ready, m_async.timeout_streak,
			(double)(m_async.last_sample_time == 0 ? 0.0f :
					(float)(chVTGetSystemTimeX() - m_async.last_sample_time) *
					1000.0f / (float)CH_CFG_ST_FREQUENCY));
	commands_printf("  pending=%u restart_ok=%u restart_fail=%u overruns=%u busy=%u/%u sync=%u",
			m_async.pending_latched, m_async.pending_restarted,
			m_async.pending_restart_failed, m_async.overruns, m_async.ignored_active,
			m_async.ignored_complete, m_async.ignored_sync);
	commands_printf("  copy_fail=%u timeout=%u fallback_start/ok/fail=%u/%u/%u recover=%u reset_ok=%u reset_fail=%u",
			m_async.copy_failed, m_async.timeouts, m_async.timeout_fallbacks,
			m_async.fallback_completed, m_async.fallback_failed,
			m_async.recoveries, m_async.reset_ok, m_async.reset_fail);
	commands_printf("  sample_dt_ticks min=%u max=%u raw_delta_max=%d,%d,%d,%d,%d,%d",
			m_async.min_sample_dt == 0xFFFFFFFF ? 0 : m_async.min_sample_dt,
			m_async.max_sample_dt,
			m_async.max_raw_delta[0], m_async.max_raw_delta[1], m_async.max_raw_delta[2],
			m_async.max_raw_delta[3], m_async.max_raw_delta[4], m_async.max_raw_delta[5]);
}

static const imu_device_interface_t lsm6dsv32x_interface = {
	.name = "LSM6DSV32X",
	.configure = configure,
	.read_sample = read_sample,
	.on_read_fail = on_read_fail,
	.enable_drdy_output = enable_drdy_output,
	.async_supported = async_supported,
	.async_start_sample = async_start_sample,
	.async_take_sample = async_take_sample,
	.async_timeout = async_timeout,
	.async_stop = async_stop,
};

imu_device_t lsm6dsv32x_device(transport_t *transport) {
	if (!m_stats_registered) {
		terminal_register_command_callback(
				"lsm6dsv32x_stats",
				"Print or reset LSM6DSV32X asynchronous sampling statistics",
				"[reset]",
				terminal_stats);
		m_stats_registered = true;
	}

	return (imu_device_t){ .interface = &lsm6dsv32x_interface, .transport = transport };
}
