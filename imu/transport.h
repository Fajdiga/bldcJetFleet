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

#ifndef IMU_TRANSPORT_H_
#define IMU_TRANSPORT_H_

#include "ch.h"
#include "hal.h"
#include "i2c_bb.h"
#include "spi_bb.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Generic register-access transport for IMU drivers.
// dev_addr is the 7-bit I2C slave address; it is ignored by the SPI transports.
// The SPI read/write direction bit (0x80) is owned by the SPI transports, so
// drivers always pass a clean register number.

// Size of the fixed stack buffer used to assemble a register transaction on paths whose
// underlying transfer API wants one contiguous buffer (hardware SPI reads/writes, and
// bit-bang I2C writes); those paths reject requests larger than this.
#define IMU_MAX_BURST 16

typedef struct transport transport_t;

typedef struct {
	// Human-readable transport name for diagnostics, e.g. "i2c-bb".
	const char *name;
	// Highest sample rate (Hz) this bus may be driven at. Setting too high
	// sample rate has caused rare unexplained MCU resets, probably due to CPU
	// saturation. Each bus should set a safe maximum.
	uint16_t (*max_sample_rate)(transport_t *t);
	bool (*read_reg)(transport_t *t, uint8_t dev_addr, uint8_t reg, uint8_t *rx, size_t len);
	bool (*write_reg)(transport_t *t, uint8_t dev_addr, uint8_t reg, const uint8_t *tx, size_t len);
	// Recover a stuck bus (e.g. an I2C slave still holding the data line low). NULL if unsupported.
	void (*recover)(transport_t *t);
	void (*deinit)(transport_t *t);
} transport_interface_t;

struct transport {
	const transport_interface_t *interface;
	union {
		i2c_bb_state i2c_bb;
		spi_bb_state spi_bb;
		struct {
			SPIDriver *spid;
			SPIConfig cfg;
			// DMA buffers, to guarantee their RAM placement (different callers
			// can have their stack in CCM ram, which is not supported by DMA).
			// The DRDY path has dedicated buffers so it never writes a register
			// transaction's buffers while ownership is changing during recovery.
			uint8_t txd[1 + IMU_MAX_BURST];
			uint8_t rxd[1 + IMU_MAX_BURST];
			uint8_t async_txd[1 + IMU_MAX_BURST];
			uint8_t async_rxd[1 + IMU_MAX_BURST];
			// Both register IO and the DRDY path use the configured SPI callback.
			// thread_owned prevents the ISR-started transfer from racing a register
			// transaction (the SPI mutex alone cannot protect against an ISR).
			binary_semaphore_t sync_sem;
			volatile bool thread_owned;
			volatile bool sync_active;
			// State for the optional ISR-started DMA read path. Register and async
			// transactions use their dedicated DMA buffer pairs above.
			volatile bool async_active;
			volatile bool async_complete;
			volatile bool async_error;
			volatile bool sync_error;
			volatile uint32_t sync_timeouts;
			volatile uint32_t sync_dma_errors;
			volatile uint32_t async_dma_errors;
			volatile uint32_t spi_resets;
			void (*async_done_cb)(void *arg, bool error);
			void *async_done_arg;
		} spi_hw;
	} bus;
};

static inline bool transport_read_reg(transport_t *t, uint8_t dev_addr, uint8_t reg, uint8_t *rx, size_t len) {
	return t->interface->read_reg(t, dev_addr, reg, rx, len);
}

static inline bool transport_write_reg(transport_t *t, uint8_t dev_addr, uint8_t reg, const uint8_t *tx, size_t len) {
	return t->interface->write_reg(t, dev_addr, reg, tx, len);
}

static inline void transport_recover(transport_t *t) {
	if (t->interface->recover) {
		t->interface->recover(t);
	}
}

static inline void transport_deinit(transport_t *t) {
	if (t && t->interface && t->interface->deinit) {
		t->interface->deinit(t);
	}
}

static inline uint16_t transport_max_sample_rate(transport_t *t) {
	return t->interface->max_sample_rate(t);
}

#endif /* IMU_TRANSPORT_H_ */
