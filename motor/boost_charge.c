/*
	Boost charging through the motor windings. See boost_charge.h.

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

#include "boost_charge.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "mc_interface.h"
#include "mcconf_default.h"
#include "utils_math.h"
#include "commands.h"
#include "terminal.h"
#include <math.h>

// A charger is assumed to be connected when all three phases, with all FETs
// off, sit above this voltage and within BOOST_DETECT_V_DIFF of each other.
// The threshold is the larger of a fixed voltage and a fraction of the battery
// voltage, so that FET leakage does not look like a charger. With nothing
// connected the phases float to around 12 V through gate/FET leakage, so a
// charger below BOOST_DETECT_V_MIN is never detected on its own and has to be
// locked from the app (boost_charge_set_lock).
#ifndef BOOST_DETECT_V_MIN
#define BOOST_DETECT_V_MIN		15.0
#endif
#ifndef BOOST_DETECT_V_FRAC
#define BOOST_DETECT_V_FRAC		0.25
#endif
#ifndef BOOST_DETECT_V_DIFF
#define BOOST_DETECT_V_DIFF		2.0		// Plus 3 % of the phase voltage
#endif
#define BOOST_DETECT_TIME		0.2		// Seconds of steady voltage to lock at runtime
#define BOOST_UNLOCK_TIME		2.0		// Seconds without supply voltage to allow unlocking
#define BOOST_SETPOINT_TIMEOUT	0.5		// Seconds
#define BOOST_RATIO_MAX			0.93	// Highest supply to battery voltage ratio, keep below BOOST_DUTY_MAX
#define BOOST_DUTY_MAX			0.95	// Highest high side duty, leaves the low side room for dead time
#define BOOST_DUTY_MIN			0.15	// Lowest high side duty, limits the step-up ratio to 6.7
// The legs never average less than BOOST_DUTY_MIN * v_bat. A supply below that
// would get current pushed back into it from the battery, so starting needs this
// much margin above it and running trips just above it.
#define BOOST_RATIO_START		(1.15 * BOOST_DUTY_MIN)
#define BOOST_RATIO_RUN			(1.03 * BOOST_DUTY_MIN)
// Worst case error of each voltage measurement (phase and battery). The supply
// window checks assume both readings are off by this much in the unsafe direction.
#ifndef BOOST_V_ERR
#define BOOST_V_ERR				2.0
#endif
// Boost starts with the legs this far below the supply voltage, as a fraction
// of it plus BOOST_START_MARGIN_V, so that the first current flows into the
// battery and not back into the supply. Only the ratio of the phase and battery
// dividers matters here. It is measured by the offset calibration; without that
// measurement the margin has to cover 1 % resistors (2 % per divider).
#define BOOST_START_MARGIN_CAL	0.01
#define BOOST_START_MARGIN_UNCAL	0.04
#define BOOST_START_MARGIN_V	0.1
// Fast reverse current trip on the unfiltered current. Short reverse spikes end
// up in the output capacitors of the supply; this stops anything that lasts.
#define BOOST_REV_TRIP_A		2.0
#define BOOST_REV_TRIP_CNT		4
#define BOOST_REV_GAIN			0.5		// Fraction of the one period (deadbeat) gain used against reverse current
#define BOOST_NO_CURRENT_TIME	0.5		// Seconds at BOOST_DUTY_MIN with less than half the current

#if defined(HW_HAS_DUAL_MOTORS) || defined(HW_HAS_DUAL_PARALLEL)
// ponytail: single motor hardware only. Dual hardware never locks, so the
// rest of this file is inert there. Add TIM8 handling if it is ever needed.
#define BOOST_UNSUPPORTED
#endif

#define LP(value, sample, k)		((value) += (k) * ((sample) - (value)))
#define PH_V(adc_volts)			((adc_volts) * ((VIN_R1 + VIN_R2) / VIN_R2) * ADC_VOLTS_PH_FACTOR)

static volatile boost_config_t m_conf = {0};
static volatile bool m_lock = false;
static volatile bool m_boot_lock = false;
// Locked from the app with the supply below the detect threshold. Leakage and a
// low voltage charger then look the same, so unplugging can not be seen and only
// the app unlocks it.
static volatile bool m_manual_lock = false;
static volatile bool m_active = false;
static volatile boost_trip_t m_trip = BOOST_TRIP_NONE;
static volatile float m_i_set = 0.0;
static volatile float m_i_ref = 0.0;
static volatile float m_integ = 0.0;
static volatile float m_duty = 0.0;
static volatile float m_v_sup = 0.0;
static volatile float m_v_bat = 0.0;
static volatile float m_i_in = 0.0;
static volatile float m_set_age = 0.0;
static volatile float m_v_low_time = 0.0;
static volatile float m_detect_time = 0.0;
static volatile float m_no_current_time = 0.0;
static volatile float m_v_phase[3] = {0.0, 0.0, 0.0};
static volatile int m_ov_cnt = 0;
static volatile int m_rev_cnt = 0;
static volatile float m_ph_gain = 1.0;	// Phase dividers / battery divider
static volatile bool m_gain_cal = false;

static void terminal_status(int argc, const char **argv);

// Phase voltages in motor phase order, the same mapping as update_valpha_vbeta
static void read_phase_volts(float *v) {
	const float k = 1.0 / m_ph_gain;
	v[0] = k * PH_V(ADC_V_L1_VOLTS);
#ifdef HW_HAS_3_SHUNTS
	v[1] = k * PH_V(ADC_V_L2_VOLTS);
	v[2] = k * PH_V(ADC_V_L3_VOLTS);
#else
	v[1] = k * PH_V(ADC_V_L3_VOLTS);
	v[2] = k * PH_V(ADC_V_L2_VOLTS);
#endif
}

void boost_charge_calibrate(float v_ph_adc_half, float v_bat) {
	if (v_bat < 8.0) {
		return;
	}

	const float gain = PH_V(v_ph_adc_half) / (0.5 * v_bat);

	// Anything further off than the resistors allow is a failed measurement
	if (gain > 0.95 && gain < 1.05) {
		m_ph_gain = gain;
		m_gain_cal = true;
	}
}

static float start_margin(float v_sup) {
	return BOOST_START_MARGIN_V +
			(m_gain_cal ? BOOST_START_MARGIN_CAL : BOOST_START_MARGIN_UNCAL) * v_sup;
}

// Supply too low: even the lowest duty would push current back into it
static bool supply_below_ratio(float v_sup, float v_bat, float ratio) {
	return (v_sup - BOOST_V_ERR) < ratio * (v_bat + BOOST_V_ERR);
}

// Supply too high: the body diodes would conduct uncontrolled into the battery
static bool supply_above_ratio(float v_sup, float v_bat) {
	return (v_sup + BOOST_V_ERR) > BOOST_RATIO_MAX * (v_bat - BOOST_V_ERR);
}

static float detect_threshold(float v_bat) {
	return fmaxf(BOOST_DETECT_V_MIN, BOOST_DETECT_V_FRAC * v_bat);
}

static bool charger_seen(const float *v, float v_bat) {
	float v_min = fminf(v[0], fminf(v[1], v[2]));
	float v_max = fmaxf(v[0], fmaxf(v[1], v[2]));
	return v_min > detect_threshold(v_bat) &&
			(v_max - v_min) < (BOOST_DETECT_V_DIFF + 0.03 * v_max);
}

static void trip(boost_trip_t reason) {
	m_active = false;
	m_i_set = 0.0;
	m_i_ref = 0.0;
	if (m_trip == BOOST_TRIP_NONE) {
		m_trip = reason;
	}
}

bool boost_charge_boot_check(void) {
	static bool registered = false;
	if (!registered) {
		registered = true;
		terminal_register_command_callback(
				"boost_status",
				"Print the boost charge status and phase voltages",
				0, terminal_status);
	}

#ifdef BOOST_UNSUPPORTED
	return false;
#else
	// Let the ADC run a few cycles after it was started
	chThdSleepMilliseconds(10);

	int seen = 0;
	for (int i = 0;i < 50;i++) {
		float v[3];
		read_phase_volts(v);
		if (charger_seen(v, GET_INPUT_VOLTAGE())) {
			seen++;
		}
		chThdSleepMilliseconds(1);
	}

	// A real supply gives 50 of 50. Lock on far less than that, as a false
	// lock only blocks the motor until the voltage is gone, while a missed
	// charger gets shorted by the offset calibration.
	if (seen >= 10) {
		m_lock = true;
		m_boot_lock = true;
	}

	return m_lock;
#endif
}

bool boost_charge_locked(void) {
	return m_lock;
}

bool boost_charge_set_lock(bool lock) {
#ifdef BOOST_UNSUPPORTED
	(void)lock;
	return false;
#else
	if (lock) {
		if (m_lock) {
			return true;
		}

		if (mc_interface_get_configuration()->motor_type != MOTOR_TYPE_FOC ||
				mc_interface_get_state() != MC_STATE_OFF ||
				fabsf(mc_interface_get_rpm()) > 300.0) {
			return false;
		}

		m_v_low_time = 0.0;
		m_manual_lock = true;
		m_lock = true;
		return true;
	} else {
		if (!m_lock) {
			return true;
		}

		if (m_active || (!m_manual_lock && m_v_low_time < BOOST_UNLOCK_TIME)) {
			return false;
		}

		m_trip = BOOST_TRIP_NONE;
		m_boot_lock = false;
		m_manual_lock = false;
		m_lock = false;
		return true;
	}
#endif
}

void boost_charge_configure(const boost_config_t *conf) {
	// Stop first so that the interrupt never runs on a half written config
	boost_charge_stop();
	m_conf = *conf;
}

bool boost_charge_set_current(float current) {
	if (!m_lock || m_trip != BOOST_TRIP_NONE) {
		return false;
	}

	// Written so that NaN ends up as 0
	if (!(current > 0.0)) {
		current = 0.0;
	} else if (!(current < m_conf.i_max)) {
		current = m_conf.i_max;
	}

	if (!m_active && current > 0.0) {
		const volatile mc_configuration *mcconf = mc_interface_get_configuration();

		bool conf_ok = m_conf.phase >= 0 && m_conf.phase <= 2 && m_conf.i_max > 0.0 &&
				m_conf.i_slew > 0.0 && m_conf.i_trip > 0.0 && m_conf.v_bat_max > 0.0 &&
				m_conf.v_sup_max > m_conf.v_sup_min;

#ifndef HW_HAS_3_SHUNTS
		// Without a third shunt both boost legs are only measured when the
		// supply is on the phase without a shunt.
		conf_ok = conf_ok && m_conf.phase == 2;
#endif

		if (!conf_ok) {
			trip(BOOST_TRIP_BAD_CONFIG);
		} else if (m_boot_lock &&
				mcconf->foc_offsets_current[0] == MCCONF_FOC_OFFSETS_CURRENT_0 &&
				mcconf->foc_offsets_current[1] == MCCONF_FOC_OFFSETS_CURRENT_1 &&
				mcconf->foc_offsets_current[2] == MCCONF_FOC_OFFSETS_CURRENT_2) {
			// The offset calibration was skipped at boot and the stored
			// offsets are the untouched defaults.
			trip(BOOST_TRIP_NO_OFFSETS);
		} else if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			trip(BOOST_TRIP_FAULT);
		} else if (m_v_sup < m_conf.v_sup_min) {
			trip(BOOST_TRIP_SUPPLY_LOW);
		} else if (m_v_sup > m_conf.v_sup_max) {
			trip(BOOST_TRIP_SUPPLY_OV);
		} else if (supply_above_ratio(m_v_sup, m_v_bat)) {
			trip(BOOST_TRIP_SUPPLY_HIGH);
		} else if (supply_below_ratio(m_v_sup, m_v_bat, BOOST_RATIO_START)) {
			trip(BOOST_TRIP_SUPPLY_RATIO);
		}

		if (m_trip != BOOST_TRIP_NONE) {
			return false;
		}

		m_integ = start_margin(m_v_sup);
		m_i_ref = 0.0;
		m_ov_cnt = 0;
		m_rev_cnt = 0;
		m_no_current_time = 0.0;
		m_set_age = 0.0;
		m_i_set = current;
		m_active = true;
	} else {
		m_i_set = current;
		m_set_age = 0.0;
	}

	return true;
}

void boost_charge_stop(void) {
	m_active = false;
	m_i_set = 0.0;
	m_i_ref = 0.0;
}

void boost_charge_clear_trip(void) {
	if (!m_active) {
		m_trip = BOOST_TRIP_NONE;
	}
}

int boost_charge_phase(void) {
	return m_conf.phase;
}

void boost_charge_get_status(boost_status_t *status) {
	status->lock = m_lock;
	status->active = m_active;
	status->trip = m_trip;
	status->v_sup = m_v_sup;
	status->i_in = m_i_in;
	status->v_bat = m_v_bat;
	status->duty = m_duty;
	status->i_ref = m_i_ref;
	status->v_low_time = m_v_low_time;
	status->v_phase[0] = m_v_phase[0];
	status->v_phase[1] = m_v_phase[1];
	status->v_phase[2] = m_v_phase[2];
}

const char* boost_charge_trip_str(boost_trip_t reason) {
	switch (reason) {
	case BOOST_TRIP_NONE: return "none";
	case BOOST_TRIP_PACK_OV: return "pack over-voltage";
	case BOOST_TRIP_OVERCURRENT: return "over-current";
	case BOOST_TRIP_REVERSE_CURRENT: return "reverse current";
	case BOOST_TRIP_SUPPLY_HIGH: return "supply too close to battery voltage";
	case BOOST_TRIP_SUPPLY_LOW: return "supply voltage too low";
	case BOOST_TRIP_SUPPLY_OV: return "supply over-voltage";
	case BOOST_TRIP_TIMEOUT: return "setpoint timeout";
	case BOOST_TRIP_FAULT: return "VESC fault";
	case BOOST_TRIP_NO_OFFSETS: return "no stored current offsets";
	case BOOST_TRIP_BAD_CONFIG: return "bad configuration";
	case BOOST_TRIP_NO_CURRENT: return "no current at the lowest duty";
	case BOOST_TRIP_SUPPLY_RATIO: return "supply too low for the battery voltage";
	}
	return "unknown";
}

boost_out_t boost_charge_isr(float ia, float ib, float ic, bool motor_off, float dt, float *duty_hs) {
#ifdef BOOST_UNSUPPORTED
	(void)ia; (void)ib; (void)ic; (void)motor_off; (void)dt; (void)duty_hs;
	return BOOST_OUT_UNLOCKED;
#else
	float v[3];
	read_phase_volts(v);
	m_v_phase[0] = v[0];
	m_v_phase[1] = v[1];
	m_v_phase[2] = v[2];

	const float v_bat_raw = GET_INPUT_VOLTAGE();
	LP(m_v_bat, v_bat_raw, 0.05);
	const float v_bat = fmaxf(m_v_bat, 1.0);

	if (!m_lock) {
		// Charger plugged in while the outputs are off (case 3 with luck)
		if (motor_off && charger_seen(v, v_bat)) {
			m_detect_time += dt;
			if (m_detect_time > BOOST_DETECT_TIME) {
				m_v_low_time = 0.0;
				m_manual_lock = false;
				m_lock = true;
			}
		} else {
			m_detect_time = 0.0;
		}

		if (!m_lock) {
			return BOOST_OUT_UNLOCKED;
		}
	}

	const int phase = m_conf.phase;
	const float i_leg_1 = phase == 0 ? ib : ia;
	const float i_leg_2 = phase == 2 ? ib : ic;

	// The input current returns through the two boost legs, so this holds no
	// matter on which side of the phase current sensor the supply is connected.
	const float i_in = -(i_leg_1 + i_leg_2);

	LP(m_v_sup, v[phase >= 0 && phase <= 2 ? phase : 0], 0.05);
	LP(m_i_in, i_in, 0.01);
	const float v_sup = m_v_sup;

	if (v_sup >= detect_threshold(v_bat)) {
		// A real charger is seen, so unplugging can be detected from here on
		m_manual_lock = false;
		m_v_low_time = 0.0;
	} else if (m_manual_lock) {
		// Keeps the script from unlocking a low voltage charger on its own
		m_v_low_time = 0.0;
	} else if (m_v_low_time < 1000.0) {
		m_v_low_time += dt;
	}

	// With this duty the legs average just below the supply voltage, so the
	// first current is small and flows into the battery. It is always written
	// to the timer so that enabling the outputs never starts from a stale
	// compare value.
	float duty = (v_sup - start_margin(v_sup)) / v_bat;
	utils_truncate_number(&duty, BOOST_DUTY_MIN, BOOST_DUTY_MAX);
	*duty_hs = duty;
	m_duty = duty;

	if (!m_active) {
		return BOOST_OUT_OFF;
	}

	m_set_age += dt;

	if (v_bat_raw > m_conf.v_bat_max) {
		m_ov_cnt++;
	} else {
		m_ov_cnt = 0;
	}

	if (i_in < -BOOST_REV_TRIP_A) {
		m_rev_cnt++;
	} else {
		m_rev_cnt = 0;
	}

	if (m_ov_cnt >= 2) {
		trip(BOOST_TRIP_PACK_OV);
	} else if (m_rev_cnt >= BOOST_REV_TRIP_CNT) {
		trip(BOOST_TRIP_REVERSE_CURRENT);
	} else if (fabsf(i_leg_1) > m_conf.i_trip || fabsf(i_leg_2) > m_conf.i_trip ||
			fabsf(i_in) > m_conf.i_trip) {
		trip(BOOST_TRIP_OVERCURRENT);
	} else if (m_i_in < -fmaxf(3.0, 0.25 * m_conf.i_max)) {
		trip(BOOST_TRIP_REVERSE_CURRENT);
	} else if (v_sup > m_conf.v_sup_max) {
		trip(BOOST_TRIP_SUPPLY_OV);
	} else if (supply_above_ratio(v_sup, v_bat)) {
		trip(BOOST_TRIP_SUPPLY_HIGH);
	} else if (v_sup < m_conf.v_sup_min) {
		trip(BOOST_TRIP_SUPPLY_LOW);
	} else if (supply_below_ratio(v_sup, v_bat, BOOST_RATIO_RUN)) {
		trip(BOOST_TRIP_SUPPLY_RATIO);
	} else if (m_set_age > BOOST_SETPOINT_TIMEOUT) {
		trip(BOOST_TRIP_TIMEOUT);
	} else if (mc_interface_get_fault() != FAULT_CODE_NONE) {
		trip(BOOST_TRIP_FAULT);
	}

	if (!m_active) {
		return BOOST_OUT_OFF;
	}

	float i_ref = m_i_ref;
	utils_step_towards(&i_ref, m_i_set, m_conf.i_slew * dt);
	m_i_ref = i_ref;

	// Setpoint 0 and ramped down: clean stop
	if (m_i_set <= 0.0 && i_ref <= 0.0) {
		m_active = false;
		return BOOST_OUT_OFF;
	}

	// The current path is the supply winding in series with the two other
	// windings in parallel: 1.5 times the phase resistance and inductance.
	const volatile mc_configuration *mcconf = mc_interface_get_configuration();
	const float kp = m_conf.kp > 0.0 ? m_conf.kp : 1.5 * mcconf->foc_current_kp;
	const float ki = m_conf.ki > 0.0 ? m_conf.ki : 1.5 * mcconf->foc_current_ki;

	// u is the voltage across the windings. A lower high side duty gives a
	// lower average leg voltage and therefore more current.
	const float err = i_ref - i_in;
	float u = kp * err + m_integ;

	// Current flowing back into the supply: answer it with half the gain that
	// would remove it in one period. Proportional only and one sided, so it
	// does not overshoot into forward current. Mostly matters at the start,
	// where a voltage reading error gives the first current its direction.
	const float l_tot = 1.5 * mcconf->foc_motor_l;
	if (i_in < 0.0 && l_tot > 0.0) {
		u -= BOOST_REV_GAIN * l_tot / dt * i_in;
	}
	duty = (v_sup - u) / v_bat;

	const bool sat_low = duty < BOOST_DUTY_MIN;
	const bool sat_high = duty > BOOST_DUTY_MAX;
	utils_truncate_number(&duty, BOOST_DUTY_MIN, BOOST_DUTY_MAX);

	// The loop asks for more current than the lowest duty can give. Nothing is
	// connected (leakage only) or the supply is too low for this step-up ratio.
	if (sat_low && i_ref > 0.5 && m_i_in < 0.5 * i_ref) {
		m_no_current_time += dt;
		if (m_no_current_time > BOOST_NO_CURRENT_TIME) {
			trip(BOOST_TRIP_NO_CURRENT);
			return BOOST_OUT_OFF;
		}
	} else {
		m_no_current_time = 0.0;
	}

	// Integrate unless that pushes further into saturation
	if (!(sat_low && err > 0.0) && !(sat_high && err < 0.0)) {
		m_integ += ki * err * dt;
	}

	*duty_hs = duty;
	m_duty = duty;

	return BOOST_OUT_SWITCHING;
#endif
}

static void terminal_status(int argc, const char **argv) {
	(void)argc; (void)argv;

	commands_printf("Lock      : %d (boot: %d, manual: %d)", m_lock, m_boot_lock, m_manual_lock);
	commands_printf("Active    : %d", m_active);
	commands_printf("Trip      : %s", boost_charge_trip_str(m_trip));
	commands_printf("Phase V   : %.1f %.1f %.1f V", (double)m_v_phase[0], (double)m_v_phase[1], (double)m_v_phase[2]);
	commands_printf("Detect at : %.1f V", (double)detect_threshold(m_v_bat));
	commands_printf("Supply    : %.1f V", (double)m_v_sup);
	commands_printf("Battery   : %.1f V", (double)m_v_bat);
	commands_printf("Input I   : %.2f A (setpoint %.2f A)", (double)m_i_in, (double)m_i_ref);
	commands_printf("Duty      : %.3f", (double)m_duty);
	// At low current this is the phase and battery voltage reading error
	commands_printf("Loop offs : %.2f V", (double)m_integ);
	commands_printf("Ph/bat cal: %.4f (%s)\n", (double)m_ph_gain, m_gain_cal ? "measured" : "not measured");
}
