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

#ifndef IMU_TRANSPORT_SPI_HW_H_
#define IMU_TRANSPORT_SPI_HW_H_

#include "transport.h"

typedef struct {
	bool thread_owned;
	bool sync_active;
	bool async_active;
	bool async_complete;
	bool async_error;
	uint32_t sync_timeouts;
	uint32_t sync_dma_errors;
	uint32_t async_dma_errors;
	uint32_t spi_resets;
	uint32_t spi_state;
} transport_spi_hw_status_t;

// Initialise t as a hardware SPI transport (SPI mode 3): configure the pins for alternate
// function af, start spid at the fastest prescaler not exceeding bus_hz (0 = default), and run
// over spid.
void transport_spi_hw_init(transport_t *t, SPIDriver *spid, uint32_t af,
		stm32_gpio_t *nss_gpio, uint8_t nss_pin, stm32_gpio_t *sck_gpio, uint8_t sck_pin,
		stm32_gpio_t *mosi_gpio, uint8_t mosi_pin, stm32_gpio_t *miso_gpio, uint8_t miso_pin,
		uint32_t bus_hz);

// Optional ISR-started DMA burst support. It is deliberately exposed only by
// the hardware-SPI transport: bit-banged and I2C transports keep their normal
// synchronous register API.
bool transport_spi_hw_async_supported(transport_t *t);
void transport_spi_hw_async_set_callback(transport_t *t,
		void (*callback)(void *arg, bool error), void *arg);
bool transport_spi_hw_async_start_read(transport_t *t, uint8_t reg, size_t len, bool from_isr);
bool transport_spi_hw_async_copy_read(transport_t *t, uint8_t *rx, size_t len);
bool transport_spi_hw_async_busy(transport_t *t);
bool transport_spi_hw_async_active(transport_t *t);
void transport_spi_hw_async_abort(transport_t *t);
bool transport_spi_hw_get_status(transport_t *t, transport_spi_hw_status_t *status);

#endif /* IMU_TRANSPORT_SPI_HW_H_ */
