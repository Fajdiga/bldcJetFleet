/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */


#include "hw.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_math.h"
#include <math.h>
#include "terminal.h"
#include "commands.h"
#include "mc_interface.h"

// Variables
static volatile bool i2c_running = false;
static mutex_t shutdown_mutex;
static float bt_lastval = 0.0;
static float bt_baseline = -1.0;
static bool will_poweroff = false;
static bool button_was_pressed = false;
static unsigned int bt_hold_counter = 0;
static unsigned int shutdown_request_counter = 0;
static unsigned int bt_press_debounce_counter = 0;
static unsigned int bt_release_debounce_counter = 0;

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

// Private functions
static void terminal_button_test(int argc, const char **argv);

void hw_init_gpio(void) {

	chMtxObjectInit(&shutdown_mutex);

	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// AUX
	AUX_OFF();
	palSetPadMode(AUX_GPIO, AUX_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// LEDs disabled - ESC is potted

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// Phase filters
	palSetPadMode(GPIOC, 15, PAL_MODE_OUTPUT_OPENDRAIN);
	palSetPadMode(GPIOC, 14, PAL_MODE_OUTPUT_OPENDRAIN);
	palSetPadMode(GPIOC, 13, PAL_MODE_OUTPUT_OPENDRAIN);
	PHASE_FILTER_OFF();

	// ShutDown
	palSetPadMode(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN, PAL_MODE_OUTPUT_OPENDRAIN);
	// Release open-drain hold line; external pullup keeps the regulator latched.
	HW_SHUTDOWN_HOLD_ON();
	palSetPadMode(HW_SHUTDOWN_SENSE_GPIO, HW_SHUTDOWN_SENSE_PIN, PAL_MODE_INPUT_ANALOG);

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);

	// I2C2 pins (PB10/PB11) are unused on G18 (IMU is on SPI3). Leave them floating
	// high-Z so they do not interfere with the LSM6DSV32X on SPI3.
	palSetPadMode(GPIOB, 10, PAL_MODE_INPUT);
	palSetPadMode(GPIOB, 11, PAL_MODE_INPUT);

	// LSM6DSV32X INT1 (data-ready) on PD2 — rising-edge EXTI
	palSetPadMode(LSM6DSV32X_INT_GPIO, LSM6DSV32X_INT_PIN, PAL_MODE_INPUT_PULLDOWN);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(LSM6DSV32X_INT_EXTI_PORTSRC, LSM6DSV32X_INT_EXTI_PINSRC);
	{
		EXTI_InitTypeDef EXTI_InitStructure;
		EXTI_InitStructure.EXTI_Line = LSM6DSV32X_INT_EXTI_LINE;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
	}
	nvicEnableVector(LSM6DSV32X_INT_EXTI_CH, 6);

	terminal_register_command_callback(
			"test_button",
			"Try sampling the shutdown button",
			0,
			terminal_button_test);
}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);   // CURR1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, ADC_SampleTime_15Cycles);    // SENS1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_15Cycles);    // EXT (PA5)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_15Cycles);   // TEMP_MOTOR (PC4)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_15Cycles);    // TEMP_MOS_2 (PB0)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 6, ADC_SampleTime_15Cycles);    // dummy

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);   // CURR2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2, ADC_SampleTime_15Cycles);    // SENS2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, ADC_SampleTime_15Cycles);    // EXT2 (PA6)
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);   // SHUTDOWN (PC5)
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 5, ADC_SampleTime_15Cycles);    // TEMP_MOS_3 (PB1)
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 6, ADC_SampleTime_15Cycles);    // dummy

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);   // CURR3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 2, ADC_SampleTime_15Cycles);    // SENS3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, ADC_SampleTime_15Cycles);    // TEMP_MOS (PA3)
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);   // VIN (PC3)
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 5, ADC_SampleTime_15Cycles);   // VIN (repeated)
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 6, ADC_SampleTime_15Cycles);   // VIN (repeated)

	// Injected channels (unchanged)
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles);
}

void hw_start_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void) {
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running) {
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;

	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void) {
	if (i2c_running) {
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for(int i = 0;i < 16;i++) {
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
				PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
				PAL_STM32_OTYPE_OPENDRAIN |
				PAL_STM32_OSPEED_MID1 |
				PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}

#define SHUTDOWN_PRESS_DELTA_V 0.30
#define SHUTDOWN_RELEASE_DELTA_V 0.20
#define TIME_100MS 10
#define TIME_300MS 30
#define TIME_1S 100
#define ERPM_THRESHOLD 100

/**
 * hw_sample_shutdown_button - return false if shutdown is requested, true otherwise.
 *
 * Behavior: edge-based sampling tuned for hardware where the pressed/unpressed
 * difference is small and the absolute ADC voltage drifts with temperature. A
 * slow baseline follows the unpressed voltage until a debounced rising edge is
 * detected. The hold time is counted while pressed, and shutdown is only
 * requested after a debounced falling edge if the hold time was valid.
 *
 * Shutdown happens on release after a valid hold time.
 *
 * Valid shutdown hold time: at least 1.0s
 */

bool hw_sample_shutdown_button(void) {
	chMtxLock(&shutdown_mutex);
	float newval = ADC_VOLTS(ADC_IND_SHUTDOWN);
	chMtxUnlock(&shutdown_mutex);
	bt_lastval = newval;

	if (bt_baseline < 0.0) {
		bt_baseline = newval;
	}

	float bt_delta = newval - bt_baseline;
	bool press_level = bt_delta > SHUTDOWN_PRESS_DELTA_V;
	bool release_level = bt_delta < SHUTDOWN_RELEASE_DELTA_V;

	if (shutdown_request_counter > 0) {
		shutdown_request_counter--;
		return false;
	}

	if (!button_was_pressed && !press_level) {
		bt_baseline += (newval - bt_baseline) * 0.02;
	}

	if (!button_was_pressed) {
		if (press_level) {
			bt_press_debounce_counter++;
		} else {
			bt_press_debounce_counter = 0;
		}

		if (bt_press_debounce_counter >= TIME_100MS) {
			button_was_pressed = true;
			bt_hold_counter = bt_press_debounce_counter;
			bt_press_debounce_counter = 0;
			bt_release_debounce_counter = 0;
			will_poweroff = false;
		}
	} else if (!release_level) {
		bt_hold_counter++;
		bt_release_debounce_counter = 0;
	} else {
		bt_release_debounce_counter++;

		if (bt_release_debounce_counter < TIME_300MS) {
			return true;
		}

		if (bt_hold_counter > TIME_1S &&
				fabsf(mc_interface_get_rpm()) < ERPM_THRESHOLD) {
			shutdown_request_counter = 80;
			will_poweroff = true;
		} else {
			will_poweroff = false;
		}

		button_was_pressed = false;
		bt_hold_counter = 0;
		bt_release_debounce_counter = 0;
		bt_baseline += (newval - bt_baseline) * 0.02;

		if (shutdown_request_counter > 0) {
			shutdown_request_counter--;
			return false;
		}
	}

	return true;
}


float hw_JetFleetG18_get_temp(void) {
	float t1 = (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15);
	float t2 = (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_2]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15);
	float t3 = (1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_3]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15);
	float res = 0.0;

	if (t1 > t2 && t1 > t3) {
		res = t1;
	} else if (t2 > t1 && t2 > t3) {
		res = t2;
	} else {
		res = t3;
	}

	return res;
}

static void terminal_button_test(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	for (int i = 0;i < 40;i++) {
		commands_printf("BT: %d:%d [%.2fV], BASE=%.2f, DELTA=%.2f, TH=%.2f/%.2f, PR=%d, OFF=%d",
				HW_SAMPLE_SHUTDOWN(), bt_hold_counter, (double)bt_lastval,
				(double)bt_baseline, (double)(bt_lastval - bt_baseline),
				(double)SHUTDOWN_PRESS_DELTA_V, (double)SHUTDOWN_RELEASE_DELTA_V,
				(int)button_was_pressed, (int)will_poweroff);
		chThdSleepMilliseconds(100);
	}
}
