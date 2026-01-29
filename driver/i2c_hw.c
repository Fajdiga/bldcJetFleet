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

#include "i2c_hw.h"

// I2C timeout in milliseconds
#define I2C_TIMEOUT_MS		100

void i2c_hw_init(i2c_hw_state *s) {
	chMtxObjectInit(&s->mutex);

	// Configure I2C speed
	I2CConfig i2ccfg;
	i2ccfg.op_mode = OPMODE_I2C;

	if (s->speed >= 400000) {
		i2ccfg.clock_speed = s->speed;
		i2ccfg.duty_cycle = FAST_DUTY_CYCLE_2;
	} else {
		i2ccfg.clock_speed = s->speed > 0 ? s->speed : 100000;
		i2ccfg.duty_cycle = STD_DUTY_CYCLE;
	}

	chMtxLock(&s->mutex);

	// Configure GPIO pins for I2C alternate function
	palSetPadMode(s->scl_gpio, s->scl_pin,
			PAL_MODE_ALTERNATE(s->gpio_af) |
			PAL_STM32_OTYPE_OPENDRAIN |
			PAL_STM32_OSPEED_MID1 |
			PAL_STM32_PUDR_PULLUP);

	palSetPadMode(s->sda_gpio, s->sda_pin,
			PAL_MODE_ALTERNATE(s->gpio_af) |
			PAL_STM32_OTYPE_OPENDRAIN |
			PAL_STM32_OSPEED_MID1 |
			PAL_STM32_PUDR_PULLUP);

	// Start I2C peripheral
	i2cStart(s->i2c_dev, &i2ccfg);

	s->is_running = true;
	s->has_error = false;

	chMtxUnlock(&s->mutex);
}

void i2c_hw_deinit(i2c_hw_state *s) {
	chMtxLock(&s->mutex);

	if (s->is_running) {
		i2cStop(s->i2c_dev);

		// Reset pins to input
		palSetPadMode(s->scl_gpio, s->scl_pin, PAL_MODE_INPUT);
		palSetPadMode(s->sda_gpio, s->sda_pin, PAL_MODE_INPUT);

		s->is_running = false;
	}

	chMtxUnlock(&s->mutex);
}

void i2c_hw_restore_bus(i2c_hw_state *s) {
	chMtxLock(&s->mutex);

	if (!s->is_running) {
		chMtxUnlock(&s->mutex);
		return;
	}

	// Stop the I2C peripheral first
	i2cStop(s->i2c_dev);

	// Configure pins as open-drain outputs for bus recovery
	palSetPadMode(s->scl_gpio, s->scl_pin,
			PAL_STM32_OTYPE_OPENDRAIN |
			PAL_STM32_OSPEED_MID1 |
			PAL_STM32_PUDR_PULLUP);

	palSetPadMode(s->sda_gpio, s->sda_pin,
			PAL_STM32_OTYPE_OPENDRAIN |
			PAL_STM32_OSPEED_MID1 |
			PAL_STM32_PUDR_PULLUP);

	// Set both lines high
	palSetPad(s->scl_gpio, s->scl_pin);
	palSetPad(s->sda_gpio, s->sda_pin);

	chThdSleep(1);

	// Generate 16 clock pulses to release any stuck slave
	for (int i = 0; i < 16; i++) {
		palClearPad(s->scl_gpio, s->scl_pin);
		chThdSleep(1);
		palSetPad(s->scl_gpio, s->scl_pin);
		chThdSleep(1);
	}

	// Generate start then stop condition
	palClearPad(s->sda_gpio, s->sda_pin);  // SDA low while SCL high = START
	chThdSleep(1);
	palClearPad(s->scl_gpio, s->scl_pin);
	chThdSleep(1);
	palSetPad(s->scl_gpio, s->scl_pin);
	chThdSleep(1);
	palSetPad(s->sda_gpio, s->sda_pin);    // SDA high while SCL high = STOP

	// Reconfigure pins for I2C alternate function
	palSetPadMode(s->scl_gpio, s->scl_pin,
			PAL_MODE_ALTERNATE(s->gpio_af) |
			PAL_STM32_OTYPE_OPENDRAIN |
			PAL_STM32_OSPEED_MID1 |
			PAL_STM32_PUDR_PULLUP);

	palSetPadMode(s->sda_gpio, s->sda_pin,
			PAL_MODE_ALTERNATE(s->gpio_af) |
			PAL_STM32_OTYPE_OPENDRAIN |
			PAL_STM32_OSPEED_MID1 |
			PAL_STM32_PUDR_PULLUP);

	// Configure I2C speed
	I2CConfig i2ccfg;
	i2ccfg.op_mode = OPMODE_I2C;

	if (s->speed >= 400000) {
		i2ccfg.clock_speed = s->speed;
		i2ccfg.duty_cycle = FAST_DUTY_CYCLE_2;
	} else {
		i2ccfg.clock_speed = s->speed > 0 ? s->speed : 100000;
		i2ccfg.duty_cycle = STD_DUTY_CYCLE;
	}

	// Restart I2C peripheral
	s->i2c_dev->state = I2C_STOP;
	i2cStart(s->i2c_dev, &i2ccfg);

	s->has_error = false;

	chMtxUnlock(&s->mutex);
}

bool i2c_hw_tx_rx(i2c_hw_state *s, uint16_t addr, uint8_t *txbuf, size_t txbytes, uint8_t *rxbuf, size_t rxbytes) {
	msg_t status = MSG_OK;

	chMtxLock(&s->mutex);

	if (!s->is_running) {
		s->has_error = true;
		chMtxUnlock(&s->mutex);
		return false;
	}

	i2cAcquireBus(s->i2c_dev);

	if (txbytes > 0 && rxbytes > 0) {
		// Combined write-read transaction
		status = i2cMasterTransmitTimeout(s->i2c_dev, addr,
				txbuf, txbytes,
				rxbuf, rxbytes,
				MS2ST(I2C_TIMEOUT_MS));
	} else if (txbytes > 0) {
		// Write only
		status = i2cMasterTransmitTimeout(s->i2c_dev, addr,
				txbuf, txbytes,
				NULL, 0,
				MS2ST(I2C_TIMEOUT_MS));
	} else if (rxbytes > 0) {
		// Read only
		status = i2cMasterReceiveTimeout(s->i2c_dev, addr,
				rxbuf, rxbytes,
				MS2ST(I2C_TIMEOUT_MS));
	}

	i2cReleaseBus(s->i2c_dev);

	if (status != MSG_OK) {
		s->has_error = true;
		chMtxUnlock(&s->mutex);
		return false;
	}

	s->has_error = false;
	chMtxUnlock(&s->mutex);
	return true;
}
