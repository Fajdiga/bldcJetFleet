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

// Initialise t as a hardware SPI transport (SPI mode 3): configure the pins for alternate
// function af, start spid at the fastest prescaler not exceeding bus_hz (0 = default), and run
// over spid.
void transport_spi_hw_init(transport_t *t, SPIDriver *spid, uint32_t af,
		stm32_gpio_t *nss_gpio, uint8_t nss_pin, stm32_gpio_t *sck_gpio, uint8_t sck_pin,
		stm32_gpio_t *mosi_gpio, uint8_t mosi_pin, stm32_gpio_t *miso_gpio, uint8_t miso_pin,
		uint32_t bus_hz);

// Start a register read using the hardware SPI DMA engine. The ISR variant is
// safe to call from a DRDY EXTI handler; the thread variant is for restarting a
// latched transfer after the previous sample has been copied.
bool transport_spi_hw_async_start_isr(transport_t *t, uint8_t reg, size_t len);
bool transport_spi_hw_async_start(transport_t *t, uint8_t reg, size_t len);

// Wait for the active DMA transfer, copy its payload, and release the async
// slot. This is called from the IMU thread, never from an ISR.
bool transport_spi_hw_async_read(transport_t *t, uint8_t *rx, size_t len, systime_t timeout);

bool transport_spi_hw_async_busy(transport_t *t);
bool transport_spi_hw_async_active(transport_t *t);
void transport_spi_hw_async_abort(transport_t *t);

#endif /* IMU_TRANSPORT_SPI_HW_H_ */
