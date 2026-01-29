/*
	Copyright 2024 JetFleet

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

#ifndef I2C_HW_H_
#define I2C_HW_H_

#include "ch.h"
#include "hal.h"
#include "stdint.h"
#include "stdbool.h"

typedef struct {
	I2CDriver *i2c_dev;          // ChibiOS I2C driver (I2CD1, I2CD2, etc.)
	stm32_gpio_t *sda_gpio;
	int sda_pin;
	stm32_gpio_t *scl_gpio;
	int scl_pin;
	uint8_t gpio_af;             // GPIO alternate function
	uint32_t speed;              // I2C speed in Hz (100000, 400000, etc.)
	bool is_running;
	bool has_error;
	mutex_t mutex;
} i2c_hw_state;

void i2c_hw_init(i2c_hw_state *s);
void i2c_hw_deinit(i2c_hw_state *s);
void i2c_hw_restore_bus(i2c_hw_state *s);
bool i2c_hw_tx_rx(i2c_hw_state *s, uint16_t addr, uint8_t *txbuf, size_t txbytes, uint8_t *rxbuf, size_t rxbytes);

#endif /* I2C_HW_H_ */
