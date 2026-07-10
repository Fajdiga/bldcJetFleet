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

#ifndef IMU_DEVICE_H_
#define IMU_DEVICE_H_

#include "transport.h"
#include "datatypes.h"   // IMU_FILTER

#include <stdint.h>
#include <stdbool.h>

// An IMU device: holds the operations vtable and the transport it talks over.
typedef struct imu_device imu_device_t;

typedef struct {
	// Human-readable model name for diagnostics.
	const char *name;

	// Probe the device and write its register init sequence. false on failure.
	bool (*configure)(imu_device_t *dev, IMU_FILTER filter, bool use_mag);

	// Read one sample into accel/gyro/mag (engineering units). false if the read failed.
	bool (*read_sample)(imu_device_t *dev, float accel[3], float gyro[3], float mag[3]);

	// Optional: failure policy, e.g. re-init. (NULL = just retry after a short sleep)
	// An asynchronous driver must return with its DRDY stream disabled; the shared
	// worker drains its completion semaphore before launching the replacement DMA.
	void (*on_read_fail)(imu_device_t *dev);

	// Optional: enable the IMU data-ready signal on its INT pin. (NULL = timed read)
	void (*enable_drdy_output)(imu_device_t *dev, bool enable);

	// Optional asynchronous sampling contract. It is intended for high-rate devices
	// whose DMA transaction must begin directly in the data-ready ISR. Drivers that
	// leave these hooks NULL continue to use the shared thread's synchronous path.
	bool (*async_supported)(imu_device_t *dev);
	// from_isr is true for a DRDY launch and false for the worker's asynchronous
	// timeout fallback. The latter must use the same DMA state machine.
	bool (*async_start_sample)(imu_device_t *dev, bool from_isr);
	bool (*async_take_sample)(imu_device_t *dev, float accel[3], float gyro[3], float mag[3]);
	// Quiesce/repair an expired transfer and return with DRDY streaming disabled.
	// true means the worker may reset its semaphore and launch a fallback DMA read.
	bool (*async_timeout)(imu_device_t *dev);
	void (*async_stop)(imu_device_t *dev);
} imu_device_interface_t;

struct imu_device {
	const imu_device_interface_t *interface;
	transport_t *transport;
	// Runtime-detected model variant (e.g. "TR-C", "9250"), NULL if none.
	const char *variant;
	uint8_t dev_addr; // I2C slave address; ignored by the SPI transports
	// Sample rate (Hz) the read loop runs at. Set by imu_thread_set_device(); a device's
	// configure() may override it with the effective rate it actually programmed.
	uint16_t sample_rate_hz;
	// Resolved in imu_thread_set_device(): true when the read loop will be DRDY-driven (the
	// board wires a DRDY pin and this device routes its data-ready to it). Drivers consult
	// it in configure() to match their ODR/filter setup to the access mode. false = timed poll.
	bool use_drdy;
	// True when the generic IMU worker delegates DRDY sampling to the optional
	// asynchronous device contract above.
	bool use_async;
	void *priv;
};

#endif /* IMU_DEVICE_H_ */
