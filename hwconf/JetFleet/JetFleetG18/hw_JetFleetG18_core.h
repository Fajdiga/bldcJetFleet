/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

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

#ifndef HW_JETFLEETG18_CORE_H_
#define HW_JETFLEETG18_CORE_H_

// ====================================================================================
// Hardware Properties
// ====================================================================================

#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS
#define HW_HAS_PHASE_FILTERS
#define HW_BOOT_VESC_CAN

// ====================================================================================
// Phase Filter
// ====================================================================================

#define PHASE_FILTER_OFF()		palSetPad(GPIOC, 15); palSetPad(GPIOC, 14); palSetPad(GPIOC, 13)
#define PHASE_FILTER_ON()		palClearPad(GPIOC, 15); palClearPad(GPIOC, 14); palClearPad(GPIOC, 13)

// ====================================================================================
// AUX Output
// ====================================================================================

#define AUX_GPIO				GPIOC
#define AUX_PIN					9
#define AUX_ON()				palSetPad(AUX_GPIO, AUX_PIN)
#define AUX_OFF()				palClearPad(AUX_GPIO, AUX_PIN)

// ====================================================================================
// LEDs - Disabled (ESC is potted, no CPU overhead)
// ====================================================================================

#define LED_GREEN_ON()			((void)0)
#define LED_GREEN_OFF()			((void)0)
#define LED_RED_ON()			((void)0)
#define LED_RED_OFF()			((void)0)

// ====================================================================================
// Shutdown Control
// ====================================================================================

#define HW_SHUTDOWN_GPIO			GPIOA
#define HW_SHUTDOWN_PIN				4
#define HW_SHUTDOWN_SENSE_GPIO		GPIOC
#define HW_SHUTDOWN_SENSE_PIN		5
#define HW_SHUTDOWN_HOLD_ON()		palSetPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN)
#define HW_SHUTDOWN_HOLD_OFF()		palClearPad(HW_SHUTDOWN_GPIO, HW_SHUTDOWN_PIN)
#define HW_SAMPLE_SHUTDOWN()		hw_sample_shutdown_button()

// ====================================================================================
// ADC Configuration
// ====================================================================================

#define HW_ADC_CHANNELS			21
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			7

// ADC Indexes
// Position 1: Current samples (first)
#define ADC_IND_CURR1			0
#define ADC_IND_CURR2			1
#define ADC_IND_CURR3			2
// Position 2: Current samples (second - back-to-back oversample)
#define ADC_IND_CURR1_2			3
#define ADC_IND_CURR2_2			4
#define ADC_IND_CURR3_2			5
// Position 3: Sense voltages
#define ADC_IND_SENS1			6
#define ADC_IND_SENS2			7
#define ADC_IND_SENS3			8
// Position 4: External inputs
#define ADC_IND_EXT				9
#define ADC_IND_EXT2			10
#define ADC_IND_TEMP_MOS		11
// Position 5: System voltages
#define ADC_IND_TEMP_MOTOR		12
#define ADC_IND_SHUTDOWN		13
#define ADC_IND_VIN_SENS		14
// Position 6: Additional temps
#define ADC_IND_TEMP_MOS_2		15
#define ADC_IND_TEMP_MOS_3		16
// Position 7: Unused

// ====================================================================================
// Current/Voltage Sensing
// ====================================================================================

#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					300000.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		4.06	// CT4022 65A (20.3mV/A)
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.001
#endif

#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

#define GET_CURRENT1()	(((float)ADC_Value[ADC_IND_CURR1] + (float)ADC_Value[ADC_IND_CURR1_2]) / 2.0)
#define GET_CURRENT2()	(((float)ADC_Value[ADC_IND_CURR2] + (float)ADC_Value[ADC_IND_CURR2_2]) / 2.0)
#define GET_CURRENT3()	(((float)ADC_Value[ADC_IND_CURR3] + (float)ADC_Value[ADC_IND_CURR3_2]) / 2.0)

// ====================================================================================
// Temperature Sensing
// ====================================================================================

#define NTC_RES(adc_val)		(10000.0 / ((4095.0 / (float)adc_val) - 1.0))
#define NTC_TEMP(adc_ind)		hw_JetFleetG18_get_temp()
#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0))
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS1()			(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS2()			(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_2]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS3()			(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_3]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// ====================================================================================
// COMM-Port ADC
// ====================================================================================

#define HW_ADC_EXT_GPIO			GPIOA
#define HW_ADC_EXT_PIN			5
#define HW_ADC_EXT2_GPIO		GPIOA
#define HW_ADC_EXT2_PIN			6

// ====================================================================================
// ICU (Servo Input)
// ====================================================================================

#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				6

// ====================================================================================
// I2C Peripheral
// ====================================================================================

#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// ====================================================================================
// LSM6DS3 IMU (Hardware I2C - faster than bitbanging)
// ====================================================================================

#define LSM6DS3_HW_I2C
#define LSM6DS3_I2C_DEV			I2CD2
#define LSM6DS3_I2C_AF			GPIO_AF_I2C2
#define LSM6DS3_I2C_SPEED		700000		// Hz (max ~636kHz actual due to STM32F4 I2C overhead)
#define LSM6DS3_SCL_GPIO		GPIOB
#define LSM6DS3_SCL_PIN			10
#define LSM6DS3_SDA_GPIO		GPIOB
#define LSM6DS3_SDA_PIN			11

// IMU Interrupt (Data Ready)
#define LSM6DS3_INT_GPIO			GPIOB
#define LSM6DS3_INT_PIN				12
#define LSM6DS3_INT_EXTI_PORTSRC	EXTI_PortSourceGPIOB
#define LSM6DS3_INT_EXTI_PINSRC		EXTI_PinSource12
#define LSM6DS3_INT_EXTI_CH			EXTI15_10_IRQn
#define LSM6DS3_INT_EXTI_LINE		EXTI_Line12
#define LSM6DS3_INT_EXTI_ISR_VEC	EXTI15_10_IRQHandler

// IMU Orientation
#define IMU_FLIP				// Chip mounted upside down

// ====================================================================================
// Hall/Encoder
// ====================================================================================

#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// ====================================================================================
// SPI Stubs (no SPI hardware on G18 - PC12 not routed, PA4 is shutdown)
// Required by NRF/encoder defaults in hw.h and encoder_cfg.c
// ====================================================================================

#define HW_SPI_PORT_NSS			0
#define HW_SPI_PIN_NSS			0
#define HW_SPI_PORT_SCK			0
#define HW_SPI_PIN_SCK			0
#define HW_SPI_PORT_MOSI		0
#define HW_SPI_PIN_MOSI			0
#define HW_SPI_PORT_MISO		0
#define HW_SPI_PIN_MISO			0

// ====================================================================================
// UART Peripheral
// ====================================================================================

#define HW_UART_DEV				SD4
#define HW_UART_GPIO_AF			GPIO_AF_UART4
#define HW_UART_TX_PORT			GPIOC
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOC
#define HW_UART_RX_PIN			11

// ====================================================================================
// Measurement Macros
// ====================================================================================

#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// ====================================================================================
// Timing
// ====================================================================================

#define HW_DEAD_TIME_NSEC		24.0

// ====================================================================================
// Voltage Sensing
// ====================================================================================

#ifndef VIN_R2
#define VIN_R2					5000.0
#endif

// ====================================================================================
// Default Configuration Overrides
// ====================================================================================

#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			60.0
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			180.0	// 40s safe max
#endif
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV					40000.0
#endif
#ifndef MCCONF_FOC_CONTROL_SAMPLE_MODE
#define MCCONF_FOC_CONTROL_SAMPLE_MODE	FOC_CONTROL_SAMPLE_MODE_V0
#endif
#ifndef MCCONF_FOC_CURRENT_SAMPLE_MODE
#define MCCONF_FOC_CURRENT_SAMPLE_MODE	FOC_CURRENT_SAMPLE_MODE_ALL_SENSORS
#endif
#ifndef MCCONF_FOC_DT_US
#define MCCONF_FOC_DT_US				0.0		// No deadtime compensation (fast 35ns gate drivers)
#endif
#ifndef MCCONF_L_MIN_DUTY
#define MCCONF_L_MIN_DUTY				0.0		// No minimum duty limit
#endif
#ifndef MCCONF_L_CURRENT_MAX
#define MCCONF_L_CURRENT_MAX			50.0
#endif
#ifndef MCCONF_L_CURRENT_MIN
#define MCCONF_L_CURRENT_MIN			-50.0
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			50.0
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-50.0
#endif
// Battery current soft rolloff - prevents hard cutoff when hitting limit
#ifndef MCCONF_L_IN_CURRENT_MAP_START
#define MCCONF_L_IN_CURRENT_MAP_START	0.90	// Start limiting at 90% of battery limit
#endif
#ifndef MCCONF_L_IN_CURRENT_MAP_FILTER
#define MCCONF_L_IN_CURRENT_MAP_FILTER	0.003	// Moderate filtering
#endif
#ifndef MCCONF_L_LIM_TEMP_FET_START
#define MCCONF_L_LIM_TEMP_FET_START		80.0
#endif
#ifndef MCCONF_L_LIM_TEMP_FET_END
#define MCCONF_L_LIM_TEMP_FET_END		90.0
#endif
// Use raw (unfiltered) current for overcurrent fault detection - faster response
#ifndef MCCONF_L_SLOW_ABS_OVERCURRENT
#define MCCONF_L_SLOW_ABS_OVERCURRENT	false
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		300.0	// CT4022 linear range ±325A, fault above 300A
#endif
#ifndef APPCONF_APP_TO_USE
#define APPCONF_APP_TO_USE				APP_NONE
#endif

// ====================================================================================
// Hardware Limits
// ====================================================================================

// CT4022 current transducer: 10-90% linear range
// Full scale ±406A, linear range ±325A (80%), ABS above linear for fault margin
#ifndef HW_LIM_CURRENT
#define HW_LIM_CURRENT			-300.0, 300.0
#endif
#ifndef HW_LIM_CURRENT_IN
#define HW_LIM_CURRENT_IN		-200.0, 200.0
#endif
#ifndef HW_LIM_CURRENT_ABS
#define HW_LIM_CURRENT_ABS		0.0, 350.0
#endif

#ifndef HW_LIM_ERPM
#define HW_LIM_ERPM				-200e3, 200e3
#endif
#ifndef HW_LIM_DUTY_MIN
#define HW_LIM_DUTY_MIN			0.0, 0.1
#endif
#ifndef HW_LIM_DUTY_MAX
#define HW_LIM_DUTY_MAX			0.0, 0.95
#endif
#ifndef HW_LIM_TEMP_FET
#define HW_LIM_TEMP_FET			-40.0, 90.0
#endif
#ifndef HW_LIM_VIN
#define HW_LIM_VIN				50.0, 190.0		// HW headroom for safety
#endif

// GaN hardware supports high switching frequencies
#ifndef HW_LIM_FOC_CTRL_LOOP_FREQ
#define HW_LIM_FOC_CTRL_LOOP_FREQ	3000.0, 100000.0
#endif

// ====================================================================================
// Function Declarations
// ====================================================================================

float hw_JetFleetG18_get_temp(void);
bool hw_sample_shutdown_button(void);

#endif /* HW_JETFLEETG18_CORE_H_ */
