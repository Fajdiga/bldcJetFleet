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
#include "utils_sys.h"
#include <math.h>
#include "terminal.h"
#include "commands.h"
#include "mc_interface.h"
#include "shutdown.h"
#include "app.h"

#define EXT_BUZZER_ON()    palSetPad(HW_ICU_GPIO, HW_ICU_PIN)
#define EXT_BUZZER_OFF()   palClearPad(HW_ICU_GPIO, HW_ICU_PIN)

void buzzer_init(void) {
    // External Buzzer (using servo pin!)
    palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN,
                  PAL_MODE_OUTPUT_PUSHPULL |
                  PAL_STM32_OSPEED_HIGHEST);
	EXT_BUZZER_ON();
    chThdSleepMilliseconds(30);
    EXT_BUZZER_OFF();
}

// Private functions
static void terminal_button_test(int argc, const char **argv);

/**
 * Enable the brown-out reset at level 3 (about 2.8 V). Without it the MCU is
 * only reset below about 1.8 V, so it keeps running, and can hang, when the
 * 3.3 V supply dips but does not go away completely. That happens when the
 * button is pressed while the board is powering down.
 *
 * The level is stored in the option bytes, so this only writes once.
 */
static void brown_out_reset_init(void) {
	if (FLASH_OB_GetBOR() != OB_BOR_LEVEL3) {
		FLASH_OB_Unlock();
		FLASH_OB_BORConfig(OB_BOR_LEVEL3);
		FLASH_OB_Launch();
		FLASH_OB_Lock();
	}
}

void hw_init_gpio(void) {
	brown_out_reset_init();
	
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	#ifdef HW_USE_BRK
	// BRK Fault pin
	palSetPadMode(BRK_GPIO, BRK_PIN, PAL_MODE_ALTERNATE(GPIO_AF_TIM1));
	#else	
	// Soft Lockout
	palSetPadMode(BRK_GPIO, BRK_PIN, PAL_MODE_INPUT);
	#endif

	
	// AUX
	AUX_OFF();
	palSetPadMode(AUX_GPIO, AUX_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);	

	// LEDs
	palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// On-board Buzzer (using servo pin!)
	palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	EXT_BUZZER_OFF();

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
	palSetPadMode(HW_SHUTDOWN_SENSE_GPIO, HW_SHUTDOWN_SENSE_PIN, PAL_MODE_INPUT);

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);

	//palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);

	terminal_register_command_callback(
			"test_button",
			"Try sampling the shutdown button",
			0,
			terminal_button_test);
}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 6, ADC_SampleTime_15Cycles);

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 5, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 6, ADC_SampleTime_15Cycles);

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 5, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 6, ADC_SampleTime_15Cycles);

	// Injected channels
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

// Generic hardware I2C is not routed on JetFleetF6: HW_I2C_SDA shares PC5 with
// shutdown sense, and PC5 has no valid I2C alternate function on STM32F4. The
// IMU has its own bit-banged path via LSM6DS3_SDA_*, so these stay no-ops.
void hw_start_i2c(void) {
}

void hw_stop_i2c(void) {
}

void hw_try_restore_i2c(void) {
}

#define BT_HOLD_TIME_S			0.5		// Hold time to power off when stopped
#define BT_HOLD_TIME_SPINNING_S	3.0		// Hold time to power off when spinning
#define BT_ERPM_THRESHOLD		100.0
#define BT_REQUEST_TIME_S		3.0		// Must outlast the 0.7 s vote in shutdown.c
#define BT_BEEP_TIME_S			0.3
#define BT_DEBOUNCE_SAMPLES		3		// 30 ms at the 10 ms sample rate

/*
 * Power button
 *
 * The button is a digital input: high when pressed, low when released. The
 * hold pin overrides the button: pulling it low cuts the power even when the
 * button is pressed, and the power then comes straight back when the MCU
 * resets and lets go of the pin. The hold pin must therefore only be pulled
 * low while the button is released.
 */

static bool button_pressed(void) {
	return palReadPad(HW_SHUTDOWN_SENSE_GPIO, HW_SHUTDOWN_SENSE_PIN) == PAL_HIGH;
}

static bool hold_is_off(void) {
	return (palReadLatch(HW_SHUTDOWN_GPIO) & (1U << HW_SHUTDOWN_PIN)) == 0;
}

// The shutdown thread wants the power cut, but the button was pressed
static volatile bool hold_off_pending = false;

/**
 * Cut the power. If the button is pressed, wait until it is released. That
 * is done from hw_sample_shutdown_button.
 */
void hw_shutdown_hold_off(void) {
	if (button_pressed()) {
		hold_off_pending = true;
	} else {
		palClearPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN);
	}
}

/**
 * hw_sample_shutdown_button - sampled by the shutdown thread in shutdown.c
 *
 * Always off mode is for a latching switch: return the switch state. The
 * shutdown thread keeps the board on while it is closed and shuts down when
 * it opens.
 *
 * All other modes are for a push button: return false when shutdown is
 * requested, true otherwise.
 * - The press that powered the board on is ignored until it is released.
 * - Holding the button for 0.5 s (3 s when the motor is spinning) arms the
 *   shutdown and sounds the buzzer. Shutdown is requested on release.
 * - Pressing the button again does not cancel the request. The power is cut
 *   when the button is released.
 */
bool hw_sample_shutdown_button(void) {
	static bool boot_released = false;
	static bool was_pressed = false;
	static bool armed = false;
	static bool off_request = false;
	static bool beep_active = false;
	static systime_t press_time = 0;
	static systime_t request_time = 0;
	static systime_t beep_start = 0;
	static bool pressed = false;
	static bool pressed_init = false;
	static int debounce_cnt = 0;

	bool pressed_raw = button_pressed();

	// The power was cut and the button was pressed before it went away. Take
	// the hold back so that the board does not brown out, and cut the power
	// again when the button is released.
	if (pressed_raw && hold_is_off()) {
		HW_SHUTDOWN_HOLD_ON();
		hold_off_pending = true;
	}

	// Debounce, so that contact bounce does not look like a new press. Start
	// from the actual state, so that a button held at boot is seen as held.
	if (!pressed_init) {
		pressed_init = true;
		pressed = pressed_raw;
	}

	if (pressed_raw == pressed) {
		debounce_cnt = 0;
	} else if (++debounce_cnt >= BT_DEBOUNCE_SAMPLES) {
		pressed = pressed_raw;
		debounce_cnt = 0;
	}

	if (app_get_configuration()->shutdown_mode == SHUTDOWN_MODE_ALWAYS_OFF) {
		hold_off_pending = false;
		return pressed;
	}

	if (hold_off_pending) {
		if (!pressed && !pressed_raw) {
			hold_off_pending = false;
			palClearPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN);
		}
		was_pressed = pressed;
		return false;
	}

	if (!boot_released) {
		boot_released = !pressed;
		return true;
	}

	if (pressed) {
		if (!was_pressed) {
			press_time = chVTGetSystemTimeX();
			armed = false;
		}

		bool spinning = fabsf(mc_interface_get_rpm()) > BT_ERPM_THRESHOLD;
		if (!armed && !off_request && UTILS_AGE_S(press_time) >
				(spinning ? BT_HOLD_TIME_SPINNING_S : BT_HOLD_TIME_S)) {
			// Let the user know that the button can be released now
			armed = true;
			beep_active = true;
			beep_start = chVTGetSystemTimeX();
			EXT_BUZZER_ON();
		}
	} else {
		if (was_pressed && armed) {
			off_request = true;
			request_time = chVTGetSystemTimeX();
		}
		armed = false;
	}

	// The shutdown thread did not cut the power (always on mode, or the
	// motor could not be stopped). Drop the request.
	if (off_request && UTILS_AGE_S(request_time) > BT_REQUEST_TIME_S) {
		off_request = false;
	}

	if (beep_active && (!pressed || UTILS_AGE_S(beep_start) > BT_BEEP_TIME_S)) {
		beep_active = false;
		EXT_BUZZER_OFF();
	}

	was_pressed = pressed;

	return !off_request;
}

static void terminal_button_test(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	for (int i = 0;i < 40;i++) {
		commands_printf("BT: pressed=%d filtered=%d hold_off=%d",
				(int)button_pressed(), (int)shutdown_button_pressed(), (int)hold_is_off());
		chThdSleepMilliseconds(100);
	}
}
