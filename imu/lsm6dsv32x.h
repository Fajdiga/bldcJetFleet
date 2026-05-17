/*
	Copyright 2024 Contributors

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

#ifndef LSM6DSV32X_H_
#define LSM6DSV32X_H_

#include "ch.h"
#include "hal.h"

#include "i2c_bb.h"
#include "spi_bb.h"

void lsm6dsv32x_set_rate_hz(int hz);
void lsm6dsv32x_set_filter(IMU_FILTER f);
void lsm6dsv32x_init(i2c_bb_state *i2c_state, stkalign_t *work_area, size_t work_area_size);
void lsm6dsv32x_init_spi_bb(spi_bb_state *spi_state, stkalign_t *work_area, size_t work_area_size);
void lsm6dsv32x_init_spi(SPIDriver *spi_dev, stm32_gpio_t *nss_gpio, int nss_pin, stkalign_t *work_area, size_t work_area_size);
void lsm6dsv32x_set_read_callback(void(*func)(float *accel, float *gyro, float *mag));
void lsm6dsv32x_stop(void);
void lsm6dsv32x_int1_isr(void);

// SPI read/write masks
#define LSM6DSV32X_SPI_RD_MASK					0x80
#define LSM6DSV32X_SPI_WR_MASK					0x7F


// I2C addresses (7-bit)
#define LSM6DSV32X_ADDR_A					0x6A
#define LSM6DSV32X_ADDR_B					0x6B

/************** Device Register  *******************/
#define LSM6DSV32X_FUNC_CFG_ACCESS				0x01
#define LSM6DSV32X_PIN_CTRL					0x02
#define LSM6DSV32X_IF_CFG					0x03
#define LSM6DSV32X_ODR_TRIG_CFG					0x06
#define LSM6DSV32X_FIFO_CTRL1					0x07
#define LSM6DSV32X_FIFO_CTRL2					0x08
#define LSM6DSV32X_FIFO_CTRL3					0x09
#define LSM6DSV32X_FIFO_CTRL4					0x0A
#define LSM6DSV32X_COUNTER_BDR_REG1				0x0B
#define LSM6DSV32X_COUNTER_BDR_REG2				0x0C
#define LSM6DSV32X_INT1_CTRL					0x0D
#define LSM6DSV32X_INT2_CTRL					0x0E
#define LSM6DSV32X_WHO_AM_I					0x0F
#define LSM6DSV32X_CTRL1					0x10
#define LSM6DSV32X_CTRL2					0x11
#define LSM6DSV32X_CTRL3					0x12
#define LSM6DSV32X_CTRL4					0x13
#define LSM6DSV32X_CTRL5					0x14
#define LSM6DSV32X_CTRL6					0x15
#define LSM6DSV32X_CTRL7					0x16
#define LSM6DSV32X_CTRL8					0x17
#define LSM6DSV32X_CTRL9					0x18
#define LSM6DSV32X_CTRL10					0x19
#define LSM6DSV32X_CTRL_STATUS					0x1A
#define LSM6DSV32X_FIFO_STATUS1					0x1B
#define LSM6DSV32X_FIFO_STATUS2					0x1C
#define LSM6DSV32X_ALL_INT_SRC					0x1D
#define LSM6DSV32X_STATUS_REG					0x1E
#define LSM6DSV32X_OUT_TEMP_L					0x20
#define LSM6DSV32X_OUT_TEMP_H					0x21
#define LSM6DSV32X_OUTX_L_G					0x22
#define LSM6DSV32X_OUTX_H_G					0x23
#define LSM6DSV32X_OUTY_L_G					0x24
#define LSM6DSV32X_OUTY_H_G					0x25
#define LSM6DSV32X_OUTZ_L_G					0x26
#define LSM6DSV32X_OUTZ_H_G					0x27
#define LSM6DSV32X_OUTX_L_A					0x28
#define LSM6DSV32X_OUTX_H_A					0x29
#define LSM6DSV32X_OUTY_L_A					0x2A
#define LSM6DSV32X_OUTY_H_A					0x2B
#define LSM6DSV32X_OUTZ_L_A					0x2C
#define LSM6DSV32X_OUTZ_H_A					0x2D
#define LSM6DSV32X_TIMESTAMP0					0x40
#define LSM6DSV32X_TIMESTAMP1					0x41
#define LSM6DSV32X_TIMESTAMP2					0x42
#define LSM6DSV32X_TIMESTAMP3					0x43
#define LSM6DSV32X_WAKE_UP_SRC					0x45
#define LSM6DSV32X_TAP_SRC					0x46
#define LSM6DSV32X_D6D_SRC					0x47
#define LSM6DSV32X_MD1_CFG					0x5E
#define LSM6DSV32X_MD2_CFG					0x5F

// WHO_AM_I expected value
#define LSM6DSV32X_WHO_AM_I_VAL					0x70

/************** IF_CFG (0x03) - Interface configuration *******************/
#define LSM6DSV32X_I2C_I3C_DISABLE			0x01

/************** INT1_CTRL (0x0D) - INT1 pin control *******************/
#define LSM6DSV32X_INT1_DRDY_XL				 0x01
#define LSM6DSV32X_INT1_DRDY_G				 0x02

/************** CTRL1 (0x10) - Accelerometer ODR and mode *******************/
// ODR bits [3:0]
typedef enum {
	LSM6DSV32X_ODR_XL_OFF				 = 0x00,
	LSM6DSV32X_ODR_XL_1_875Hz			 = 0x01,
	LSM6DSV32X_ODR_XL_7_5Hz			 = 0x02,
	LSM6DSV32X_ODR_XL_15Hz				 = 0x03,
	LSM6DSV32X_ODR_XL_30Hz				 = 0x04,
	LSM6DSV32X_ODR_XL_60Hz				 = 0x05,
	LSM6DSV32X_ODR_XL_120Hz			 = 0x06,
	LSM6DSV32X_ODR_XL_240Hz			 = 0x07,
	LSM6DSV32X_ODR_XL_480Hz			 = 0x08,
	LSM6DSV32X_ODR_XL_960Hz			 = 0x09,
	LSM6DSV32X_ODR_XL_1920Hz			 = 0x0A,
	LSM6DSV32X_ODR_XL_3840Hz			 = 0x0B,
	LSM6DSV32X_ODR_XL_7680Hz			 = 0x0C,
} LSM6DSV32X_ODR_XL_t;

// Accel operating mode bits [6:4]
typedef enum {
	LSM6DSV32X_XL_MODE_HIGH_PERF		 = 0x00,
	LSM6DSV32X_XL_MODE_HIGH_ACC_ODR		 = 0x10,
	LSM6DSV32X_XL_MODE_LP2_AVG			 = 0x40,
	LSM6DSV32X_XL_MODE_LP4_AVG			 = 0x50,
	LSM6DSV32X_XL_MODE_LP8_AVG			 = 0x60,
	LSM6DSV32X_XL_MODE_NORMAL			 = 0x70,
} LSM6DSV32X_XL_MODE_t;

/************** CTRL2 (0x11) - Gyroscope ODR and mode *******************/
// ODR bits [3:0]
typedef enum {
	LSM6DSV32X_ODR_G_OFF				 = 0x00,
	LSM6DSV32X_ODR_G_7_5Hz				 = 0x02,
	LSM6DSV32X_ODR_G_15Hz				 = 0x03,
	LSM6DSV32X_ODR_G_30Hz				 = 0x04,
	LSM6DSV32X_ODR_G_60Hz				 = 0x05,
	LSM6DSV32X_ODR_G_120Hz				 = 0x06,
	LSM6DSV32X_ODR_G_240Hz				 = 0x07,
	LSM6DSV32X_ODR_G_480Hz				 = 0x08,
	LSM6DSV32X_ODR_G_960Hz				 = 0x09,
	LSM6DSV32X_ODR_G_1920Hz			 = 0x0A,
	LSM6DSV32X_ODR_G_3840Hz			 = 0x0B,
	LSM6DSV32X_ODR_G_7680Hz			 = 0x0C,
} LSM6DSV32X_ODR_G_t;

// Gyro operating mode bits [6:4]
typedef enum {
	LSM6DSV32X_G_MODE_HIGH_PERF			 = 0x00,
	LSM6DSV32X_G_MODE_HIGH_ACC_ODR		 = 0x10,
	LSM6DSV32X_G_MODE_SLEEP			 = 0x40,
	LSM6DSV32X_G_MODE_LOW_POWER			 = 0x50,
} LSM6DSV32X_G_MODE_t;

/************** CTRL3 (0x12) - Control register 3 *******************/
#define LSM6DSV32X_SW_RESET				 0x01
#define LSM6DSV32X_IF_INC				 0x04
#define LSM6DSV32X_BDU					 0x40
#define LSM6DSV32X_BOOT					 0x80

/************** CTRL6 (0x15) - Gyro full-scale and LPF1 BW *******************/
// Gyro full-scale bits [3:0]
typedef enum {
	LSM6DSV32X_FS_G_125dps				 = 0x00,
	LSM6DSV32X_FS_G_250dps				 = 0x01,
	LSM6DSV32X_FS_G_500dps				 = 0x02,
	LSM6DSV32X_FS_G_1000dps			 = 0x03,
	LSM6DSV32X_FS_G_2000dps			 = 0x04,
	LSM6DSV32X_FS_G_4000dps			 = 0x0C,
} LSM6DSV32X_FS_G_t;

// Gyro LPF1 bandwidth bits [6:4]
typedef enum {
	LSM6DSV32X_G_LPF1_BW_0				 = 0x00,
	LSM6DSV32X_G_LPF1_BW_1				 = 0x10,
	LSM6DSV32X_G_LPF1_BW_2				 = 0x20,
	LSM6DSV32X_G_LPF1_BW_3				 = 0x30,
	LSM6DSV32X_G_LPF1_BW_4				 = 0x40,
	LSM6DSV32X_G_LPF1_BW_5				 = 0x50,
	LSM6DSV32X_G_LPF1_BW_6				 = 0x60,
	LSM6DSV32X_G_LPF1_BW_7				 = 0x70,
} LSM6DSV32X_G_LPF1_BW_t;

/************** CTRL7 (0x16) - Gyro filter settings *******************/
#define LSM6DSV32X_G_LPF1_EN				 0x01

/************** CTRL8 (0x17) - Accel full-scale and filter bandwidth *******************/
// Bit 2 must be set to 1 for correct operation of LSM6DSV32X
#define LSM6DSV32X_XL_FS_MODE				 0x04

// Accel full-scale bits [1:0] for LSM6DSV32X
typedef enum {
	LSM6DSV32X_FS_XL_4g				 = 0x00,
	LSM6DSV32X_FS_XL_8g				 = 0x01,
	LSM6DSV32X_FS_XL_16g				 = 0x02,
	LSM6DSV32X_FS_XL_32g				 = 0x03,
} LSM6DSV32X_FS_XL_t;

// Accel HP / LPF2 bandwidth bits [7:5]
typedef enum {
	LSM6DSV32X_XL_HP_BW_ODR_4			 = 0x00,
	LSM6DSV32X_XL_HP_BW_ODR_10			 = 0x20,
	LSM6DSV32X_XL_HP_BW_ODR_20			 = 0x40,
	LSM6DSV32X_XL_HP_BW_ODR_45			 = 0x60,
	LSM6DSV32X_XL_HP_BW_ODR_100		 = 0x80,
	LSM6DSV32X_XL_HP_BW_ODR_200		 = 0xA0,
	LSM6DSV32X_XL_HP_BW_ODR_400		 = 0xC0,
	LSM6DSV32X_XL_HP_BW_ODR_800		 = 0xE0,
} LSM6DSV32X_XL_HP_BW_t;

/************** CTRL9 (0x18) - Accel filter enables *******************/
#define LSM6DSV32X_XL_LPF2_EN				 0x08

/************** STATUS_REG (0x1E) bits *******************/
#define LSM6DSV32X_STATUS_XLDA				 0x01
#define LSM6DSV32X_STATUS_GDA				 0x02
#define LSM6DSV32X_STATUS_TDA				 0x04

#endif /* LSM6DSV32X_H_ */
