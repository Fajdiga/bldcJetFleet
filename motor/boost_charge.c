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
#include "flash_helper.h"
#include <math.h>
#include <stdio.h>

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
// Boot check threshold, as a fraction of the battery voltage: anything clearly
// above the leakage float (about 9 % of the battery) skips the calibration.
#ifndef BOOST_BOOT_V_FRAC
#define BOOST_BOOT_V_FRAC		0.15
#endif
#define BOOST_UNLOCK_TIME		2.0		// Seconds without supply voltage to allow unlocking
#define BOOST_SETPOINT_TIMEOUT	0.5		// Seconds
#define BOOST_RATIO_MAX			0.93	// Highest supply to battery voltage ratio, keep below BOOST_DUTY_MAX
#define BOOST_DUTY_MAX			0.95	// Highest high side duty, leaves the low side room for dead time
#define BOOST_DUTY_MIN			0.10	// Lowest high side duty, limits the step-up ratio to 10
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
// Boost starts with the legs at the supply voltage as it was read at that
// moment, and that voltage is held for the duty until this much forward current
// flows. A reading error then balances itself: if the legs are a little above
// the supply, the current that flows back lifts the output capacitor of the
// supply by that error and stops. If the duty followed the supply voltage from
// the start, the legs would rise with it and the current would keep flowing.
// The error is the supply phase divider against the battery divider, which the
// offset calibration measures for each phase (boost_charge_calibrate).
#define BOOST_FF_TRACK_A		0.3
// Fast reverse current trip on the unfiltered current. Short reverse spikes end
// up in the output capacitors of the supply; this stops anything that lasts.
#define BOOST_REV_TRIP_A		2.0
#define BOOST_REV_TRIP_CNT		4
#define BOOST_REV_BLANK			0.002	// s from the start without the fast reverse trip
#define BOOST_REV_GAIN			0.5		// Fraction of the one period (deadbeat) gain used against reverse current
#define BOOST_NO_CURRENT_TIME	0.5		// Seconds at BOOST_DUTY_MIN with less than half the current

// The current sensors are zeroed while the lock is set and all FETs have been
// off for this long, so that the winding current is gone.
#define BOOST_ZERO_DELAY		0.05	// Seconds
// A lock from the boot check is dropped when no script has armed the charger
// detection after this long: the package was removed, or its script does not run.
#define BOOST_ARM_TIMEOUT		5.0		// Seconds

#if defined(HW_HAS_DUAL_MOTORS) || defined(HW_HAS_DUAL_PARALLEL) || defined(HW_USE_LINE_TO_LINE)
// ponytail: single motor hardware with phase to ground voltage sensing only.
// Other hardware never locks, so the rest of this file is inert there. Add
// TIM8 handling if it is ever needed.
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
// Battery divider / phase divider for ADC_V_L1 to ADC_V_L3
static volatile float m_ph_k[3] = {1.0, 1.0, 1.0};
static volatile int m_gain_cal = 0;		// One bit for each measured phase
static volatile bool m_armed = false;
static volatile float m_unarmed_time = 0.0;
static volatile float m_i_zero = 0.0;	// Input current reading with all FETs off
static volatile float m_off_time = 0.0;
static volatile float m_v_ff = 0.0;		// Supply voltage used for the duty
static volatile bool m_ff_track = false;

// How late the current sensors read on a current ramp [s]. Low side shunts read
// R * i + L * di/dt, which is the current from L / R later. 0 for sensors without
// a noticeable lag. Set per hardware, and adjustable with boost_lag for testing.
#ifndef BOOST_I_SENSE_LAG
#define BOOST_I_SENSE_LAG		0.0
#endif
static volatile float m_sense_lag = BOOST_I_SENSE_LAG;

// Interrupt samples for boost_trace: the first BOOST_TRACE_START after each
// start, then every BOOST_TRACE_DIV-th sample in a ring, frozen at a trip.
#define BOOST_TRACE_LEN			64
#define BOOST_TRACE_START		32
#define BOOST_TRACE_DIV			64
typedef struct {
	float t_active;
	float i_leg_1;
	float i_leg_2;
	float i_in;
	float i_ref;
	float integ;
	float duty;
	float v_sup;
} boost_trace_t;
static boost_trace_t m_trace[BOOST_TRACE_LEN];
static volatile int m_trace_cnt = 0;	// Samples since the start
static volatile bool m_trace_frozen = false;
static volatile float m_active_time = 0.0;

// Sensor zero while switching, measured with no supply connected when charge
// mode is switched on: both legs switch at the same duty, so no current can
// flow. It differs from the zero with the FETs off (low side shunts read
// differently with the low side on). Used at the duty in use while charging.
#define BOOST_ZTAB_N			8
#define BOOST_ZTAB_DUTY(i)		(0.15 + 0.1 * (float)(i))
#define BOOST_ZTAB_SAMPLES		200
static volatile float m_ztab[BOOST_ZTAB_N];
static volatile bool m_ztab_ok = false;
static volatile int m_zsweep = -1;		// Duty index being measured, -1 = idle
static volatile bool m_zsweep_req = false;	// Sweep asked for by the script start
static volatile int m_zsweep_n = 0;
static volatile float m_zsweep_sum = 0.0;

// A charger that can not take current back only rises above the voltage it had
// before the start when current flows back into it. Above this rise the current
// reference is raised by BOOST_RISE_GAIN per volt, so that sensor errors at low
// current can not pump it up.
#define BOOST_V_RISE			1.0		// V
#define BOOST_RISE_GAIN			2.0		// A/V
static volatile float m_v_open = 0.0;	// Supply voltage before the start

// While the legs switch, the phase and battery voltage readings can be off by
// a few percent (coupling from the switching phases into the sense lines),
// while they are right with the outputs off. These factors correct the
// readings while switching, for the trips, the supply guard and the status.
// The duty uses the raw readings: its integrator takes the offset anyway.
// Measured per hardware, and adjustable per board from the package.
#ifndef BOOST_V_SUP_SW_K
#define BOOST_V_SUP_SW_K		1.0
#endif
#ifndef BOOST_V_BAT_SW_K
#define BOOST_V_BAT_SW_K		1.0
#endif
static volatile float m_k_sup = BOOST_V_SUP_SW_K;
static volatile float m_k_bat = BOOST_V_BAT_SW_K;

// The supply voltage in the duty follows the measured one through this filter.
// Following it in every period, a charger without capacitors (whose voltage
// drops as soon as it delivers current) and the loop oscillate.
#define BOOST_FF_TAU			0.017	// s

// Zero current test: with no supply on the phase, both boost legs switch at the
// same duty, so no current can flow and the sensors show their offset while
// switching. Run from the terminal with boost_zero.
#define BOOST_ZERO_SKIP			20		// Samples left out while the outputs come on
static volatile int m_zero_n = 0;		// Samples left, -1 = aborted
static volatile float m_zero_duty = 0.0;
static volatile float m_zero_sum_1 = 0.0;
static volatile float m_zero_sum_2 = 0.0;
static volatile int m_zero_cnt = 0;

static void terminal_status(int argc, const char **argv);
static void terminal_trace(int argc, const char **argv);
static void terminal_zero(int argc, const char **argv);
static void terminal_lag(int argc, const char **argv);

// Phase voltages in motor phase order, the same mapping as update_valpha_vbeta
static void read_phase_volts(float *v) {
	v[0] = m_ph_k[0] * PH_V(ADC_V_L1_VOLTS);
#ifdef HW_HAS_3_SHUNTS
	v[1] = m_ph_k[1] * PH_V(ADC_V_L2_VOLTS);
	v[2] = m_ph_k[2] * PH_V(ADC_V_L3_VOLTS);
#else
	v[1] = m_ph_k[2] * PH_V(ADC_V_L3_VOLTS);
	v[2] = m_ph_k[1] * PH_V(ADC_V_L2_VOLTS);
#endif
}

void boost_charge_calibrate(int adc_ch, float v_ph_adc_1, float v_ph_adc_2, float v_bat) {
	if (adc_ch < 0 || adc_ch > 2) {
		return;
	}

	const float gain = PH_V(v_ph_adc_2) / fmaxf(v_bat, 1.0);

	// A reading that still moves has not settled, and anything further off than
	// the resistors allow is a failed measurement. The wider margin is used then.
	if (v_bat < 8.0 || fabsf(v_ph_adc_2 - v_ph_adc_1) > 0.003 * v_ph_adc_2 ||
			gain < 0.95 || gain > 1.05) {
		m_gain_cal &= ~(1 << adc_ch);
		m_ph_k[adc_ch] = 1.0;
	} else {
		m_ph_k[adc_ch] = 1.0 / gain;
		m_gain_cal |= 1 << adc_ch;
	}
}

// The charger detection only runs for the boost charge package. Its script
// stores a marker in the lisp eeprom, which is what the boot check can see
// before any script runs, and arms the detection when it starts.
#ifndef BOOST_UNSUPPORTED
static bool package_installed(void) {
	eeprom_var v;
	return flash_helper_code_size(CODE_IND_LISP) > 0 &&
			conf_general_read_eeprom_var_custom(&v, BOOST_EEPROM_ARM_ADDR) &&
			v.as_i32 == BOOST_EEPROM_ARM_MAGIC;
}
#endif

bool boost_charge_armed(void) {
#ifdef BOOST_UNSUPPORTED
	return false;
#else
	return m_armed || package_installed();
#endif
}

void boost_charge_arm(void) {
	m_armed = true;
	// Measure the sensor zero while switching once, if nothing is connected
	if (!m_ztab_ok) {
		m_zsweep_req = true;
	}
}

bool boost_charge_zero_sweeping(void) {
	return m_zsweep >= 0;
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

static bool phases_above(const float *v, float threshold) {
	float v_min = fminf(v[0], fminf(v[1], v[2]));
	float v_max = fmaxf(v[0], fmaxf(v[1], v[2]));
	return v_min > threshold && (v_max - v_min) < (BOOST_DETECT_V_DIFF + 0.03 * v_max);
}

static bool charger_seen(const float *v, float v_bat) {
	return phases_above(v, detect_threshold(v_bat));
}

static void trip(boost_trip_t reason) {
	m_active = false;
	m_i_set = 0.0;
	m_i_ref = 0.0;
	if (m_trip == BOOST_TRIP_NONE) {
		m_trip = reason;
		m_trace_frozen = true;
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
		terminal_register_command_callback(
				"boost_trace",
				"Print the last boost charge samples before the trip",
				0, terminal_trace);
		terminal_register_command_callback(
				"boost_zero",
				"Measure the current sensor offsets while switching. Charge mode on, NO supply connected.",
				0, terminal_zero);
		terminal_register_command_callback(
				"boost_lag",
				"Print or set the current sensor lag in us (not stored)",
				"[us]", terminal_lag);
	}

#ifdef BOOST_UNSUPPORTED
	return false;
#else
	if (!package_installed()) {
		return false;
	}

	// Let the ADC run a few cycles after it was started
	chThdSleepMilliseconds(10);

	int seen = 0;
	for (int i = 0;i < 50;i++) {
		float v[3];
		read_phase_volts(v);
		// Lower than the runtime threshold: a charger that is switched off
		// but still has charged capacitors on the phase must not get the
		// battery switched onto it by the calibration either.
		if (phases_above(v, fmaxf(BOOST_DETECT_V_MIN, BOOST_BOOT_V_FRAC * GET_INPUT_VOLTAGE()))) {
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

		// Nothing on the phases (they float to around 12 V): measure the
		// sensor zero while switching, see m_ztab.
		const float v_max = fmaxf(m_v_phase[0], fmaxf(m_v_phase[1], m_v_phase[2]));
		if (v_max < 0.5 * detect_threshold(m_v_bat) && m_zero_n == 0) {
			m_ztab_ok = false;
			m_zsweep_n = 0;
			m_zsweep_sum = 0.0;
			m_zsweep = 0;
		}
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
	m_k_sup = conf->k_sup_sw > 0.0 ? conf->k_sup_sw : BOOST_V_SUP_SW_K;
	m_k_bat = conf->k_bat_sw > 0.0 ? conf->k_bat_sw : BOOST_V_BAT_SW_K;
}

bool boost_charge_set_current(float current) {
	if (!m_lock || m_trip != BOOST_TRIP_NONE || m_zsweep >= 0) {
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

		m_integ = 0.0;
		m_v_ff = m_v_sup;
		m_v_open = m_v_sup;
		m_ff_track = false;
		m_i_ref = 0.0;
		m_ov_cnt = 0;
		m_rev_cnt = 0;
		m_no_current_time = 0.0;
		m_set_age = 0.0;
		m_active_time = 0.0;
		m_trace_cnt = 0;
		m_trace_frozen = false;
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
	status->v_sup = m_active ? m_v_sup * m_k_sup : m_v_sup;
	status->i_in = m_i_in;
	status->v_bat = m_active ? m_v_bat * m_k_bat : m_v_bat;
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

	if (!m_armed) {
		if (m_lock && m_unarmed_time < BOOST_ARM_TIMEOUT) {
			m_unarmed_time += dt;
		} else {
			m_lock = false;
		}
	}

	const int phase = m_conf.phase;
	float i_leg_1 = phase == 0 ? ib : ia;
	float i_leg_2 = phase == 2 ? ib : ic;
	const float duty_applied = m_duty;	// Set in the previous interrupt

	// The script asked for a zero sweep when it started (boot): do it now if
	// the motor is off and nothing is on the phases, otherwise not at all.
	if (m_zsweep_req) {
		m_zsweep_req = false;
		const float v_max = fmaxf(v[0], fmaxf(v[1], v[2]));
		if (!m_active && m_zsweep < 0 && (m_lock || motor_off) &&
				v_max < 0.5 * detect_threshold(v_bat)) {
			m_ztab_ok = false;
			m_zsweep_n = 0;
			m_zsweep_sum = 0.0;
			m_zsweep = 0;
		}
	}

	// Sensor zero sweep, see m_ztab. The supply reading and the detection are
	// left alone meanwhile, as the floating supply phase follows the legs.
	if (m_zsweep >= 0) {
		const bool outputs_ours = m_lock || motor_off || m_zsweep_n > 0 || m_zsweep > 0;
		if (m_active || !outputs_ours || fabsf(i_leg_1) > 5.0 || fabsf(i_leg_2) > 5.0) {
			// Current flows: a supply was connected after all
			m_zsweep = -1;
			m_ztab_ok = false;
			return m_lock ? BOOST_OUT_OFF : BOOST_OUT_UNLOCKED;
		}

		if (++m_zsweep_n > BOOST_ZERO_SKIP) {
			m_zsweep_sum += -(i_leg_1 + i_leg_2);
		}

		if (m_zsweep_n >= BOOST_ZERO_SKIP + BOOST_ZTAB_SAMPLES) {
			m_ztab[m_zsweep] = m_zsweep_sum / (float)BOOST_ZTAB_SAMPLES;
			m_zsweep_n = 0;
			m_zsweep_sum = 0.0;
			if (++m_zsweep >= BOOST_ZTAB_N) {
				m_zsweep = -1;
				m_ztab_ok = true;
				// Outputs off. Unlocked, the next interrupt hands the
				// motor back (BOOST_OUT_UNLOCKED).
				return BOOST_OUT_OFF;
			}
		}

		*duty_hs = BOOST_ZTAB_DUTY(m_zsweep);
		m_duty = *duty_hs;
		return BOOST_OUT_SWITCHING;
	}

	if (!m_lock) {
		// Charger plugged in while the outputs are off (case 3 with luck)
		if (m_armed && motor_off && charger_seen(v, v_bat)) {
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

	LP(m_v_sup, v[phase >= 0 && phase <= 2 ? phase : 0], 0.05);
	const float v_sup = m_v_sup;

	// The current is sampled in the middle of the low side on time, where it
	// rises at v_sup / l_tot, half of that in each leg. A sensor that lags,
	// such as a shunt with its own inductance (it reads R * i + L * di/dt), then
	// reads the current from m_sense_lag later, which is corrected here.
	// The current along the supply phase pulls the rotor in line with it, so
	// the windings show their d axis inductance (Ld = L - (Lq - Ld) / 2).
	const volatile mc_configuration *mcc = mc_interface_get_configuration();
	const float l_tot_sense = 1.5 * fmaxf(mcc->foc_motor_l - 0.5 * mcc->foc_motor_ld_lq_diff,
			0.5 * mcc->foc_motor_l);
	if (m_active && l_tot_sense > 0.0) {
		const float leg_ramp = 0.5 * m_sense_lag * v_sup / l_tot_sense;
		i_leg_1 += leg_ramp;	// Legs read the current into the motor, it falls here
		i_leg_2 += leg_ramp;
	}

	// The input current returns through the two boost legs, so this holds no
	// matter on which side of the phase current sensor the supply is connected.
	const float i_raw = -(i_leg_1 + i_leg_2);

	// With all FETs off and the supply below the battery no current can flow,
	// so what the sensors show then is their offset. This is more accurate than
	// the offsets from boot, which matters for small and for reverse currents.
	if (m_active || m_zero_n != 0) {
		m_off_time = 0.0;
	} else if (m_off_time < BOOST_ZERO_DELAY) {
		m_off_time += dt;
	} else if (v_sup < v_bat) {
		LP(m_i_zero, i_raw, 0.01);
	}

	float zero = m_i_zero;
	if (m_active && m_ztab_ok) {
		float x = (duty_applied - BOOST_ZTAB_DUTY(0)) / (BOOST_ZTAB_DUTY(1) - BOOST_ZTAB_DUTY(0));
		utils_truncate_number(&x, 0.0, (float)(BOOST_ZTAB_N - 1) - 0.001);
		const int i = (int)x;
		zero = m_ztab[i] + (m_ztab[i + 1] - m_ztab[i]) * (x - (float)i);
	}

	const float i_in = i_raw - zero;
	LP(m_i_in, i_in, 0.01);

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

	// With this duty the legs average the supply voltage, so no current flows.
	// It is always written to the timer so that enabling the outputs never
	// starts from a stale compare value.
	float duty = v_sup / v_bat;
	utils_truncate_number(&duty, BOOST_DUTY_MIN, BOOST_DUTY_MAX);
	*duty_hs = duty;
	m_duty = duty;

	if (!m_active && m_zero_n > 0) {
		if (fabsf(i_leg_1) > 5.0 || fabsf(i_leg_2) > 5.0) {
			// Current flows: a supply is connected after all
			m_zero_n = -1;
			return BOOST_OUT_OFF;
		}

		if (m_zero_n-- <= m_zero_cnt) {
			m_zero_sum_1 += i_leg_1;
			m_zero_sum_2 += i_leg_2;
		}

		*duty_hs = m_zero_duty;
		m_duty = m_zero_duty;
		return m_zero_n > 0 ? BOOST_OUT_SWITCHING : BOOST_OUT_OFF;
	}

	if (!m_active) {
		return BOOST_OUT_OFF;
	}

	m_set_age += dt;
	m_active_time += dt;

	// Corrected readings for the trips and the supply guard
	const float v_sup_c = v_sup * m_k_sup;
	const float v_bat_c = v_bat * m_k_bat;
	const float v_bat_raw_c = v_bat_raw * m_k_bat;

	if (!m_trace_frozen) {
		const int n = m_trace_cnt++;
		int ind = -1;
		if (n < BOOST_TRACE_START) {
			ind = n;
		} else if ((n - BOOST_TRACE_START) % BOOST_TRACE_DIV == 0) {
			ind = BOOST_TRACE_START + ((n - BOOST_TRACE_START) / BOOST_TRACE_DIV) % (BOOST_TRACE_LEN - BOOST_TRACE_START);
		}

		if (ind >= 0) {
			boost_trace_t *t = &m_trace[ind];
			t->t_active = m_active_time;
			t->i_leg_1 = i_leg_1;
			t->i_leg_2 = i_leg_2;
			t->i_in = i_in;
			t->i_ref = m_i_ref;
			t->integ = m_integ;
			t->duty = duty_applied;
			t->v_sup = v_sup;
		}
	}

	if (v_bat_raw_c > m_conf.v_bat_max) {
		m_ov_cnt++;
	} else {
		m_ov_cnt = 0;
	}

	// Left out at the start: the switching does not start in the middle of a
	// ripple period, so the current rings through zero for up to a ms or so
	// (windings against the supply capacitors) before the loop has settled it.
	if (i_in < -BOOST_REV_TRIP_A && m_active_time > BOOST_REV_BLANK) {
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
	} else if (v_sup_c > m_conf.v_sup_max) {
		trip(BOOST_TRIP_SUPPLY_OV);
	} else if (supply_above_ratio(v_sup_c, v_bat_c)) {
		trip(BOOST_TRIP_SUPPLY_HIGH);
	} else if (v_sup_c < m_conf.v_sup_min) {
		trip(BOOST_TRIP_SUPPLY_LOW);
	} else if (supply_below_ratio(v_sup_c, v_bat_c, BOOST_RATIO_RUN)) {
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
	float err = i_ref - i_in;
	const float rise = v_sup_c - m_v_open - BOOST_V_RISE;
	if (rise > 0.0) {
		err += BOOST_RISE_GAIN * rise;
	}
	float u = kp * err + m_integ;

	// Current flowing back into the supply: answer it with half the gain that
	// would remove it in one period. Proportional only and one sided, so it
	// does not overshoot into forward current.
	const float l_tot = 1.5 * mcconf->foc_motor_l;
	if (i_in < 0.0 && l_tot > 0.0) {
		u -= BOOST_REV_GAIN * l_tot / dt * i_in;
	}

	// See BOOST_FF_TRACK_A. The integrator takes over what the supply voltage
	// has moved in the meantime, so that the duty does not jump.
	if (!m_ff_track && m_i_in > BOOST_FF_TRACK_A) {
		m_integ += v_sup - m_v_ff;
		u += v_sup - m_v_ff;
		m_ff_track = true;
	}
	if (m_ff_track) {
		LP(m_v_ff, v_sup, dt / BOOST_FF_TAU);
	}
	duty = (m_v_ff - u) / v_bat;

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

	commands_printf("Detection : %s", m_armed ? "armed by the script" : "off, no boost charge script running");
	commands_printf("Lock      : %d (boot: %d, manual: %d)", m_lock, m_boot_lock, m_manual_lock);
	commands_printf("Active    : %d", m_active);
	commands_printf("Trip      : %s", boost_charge_trip_str(m_trip));
	commands_printf("Phase V   : %.1f %.1f %.1f V", (double)m_v_phase[0], (double)m_v_phase[1], (double)m_v_phase[2]);
	commands_printf("Detect at : %.1f V", (double)detect_threshold(m_v_bat));
	commands_printf("Supply    : %.1f V (raw %.1f)", (double)(m_active ? m_v_sup * m_k_sup : m_v_sup), (double)m_v_sup);
	commands_printf("Battery   : %.1f V (raw %.1f)", (double)(m_active ? m_v_bat * m_k_bat : m_v_bat), (double)m_v_bat);
	commands_printf("Sw. V cal : supply x%.4f, battery x%.4f (while switching)", (double)m_k_sup, (double)m_k_bat);
	commands_printf("Input I   : %.2f A (setpoint %.2f A)", (double)m_i_in, (double)m_i_ref);
	commands_printf("Duty      : %.3f", (double)m_duty);
	// At low current this is the phase and battery voltage reading error
	commands_printf("Loop offs : %.2f V", (double)m_integ);
	commands_printf("I zero    : %.2f A", (double)m_i_zero);
	commands_printf("Bat/ph cal: %.4f %.4f %.4f (%s)\n", (double)m_ph_k[0], (double)m_ph_k[1], (double)m_ph_k[2],
			m_gain_cal == 7 ? "measured" : "not measured");
}

static void terminal_trace(int argc, const char **argv) {
	(void)argc; (void)argv;

	const int cnt = m_trace_cnt;
	commands_printf("Trip: %s, %d samples since the start, zero table %s, I zero off %.2f A",
			boost_charge_trip_str(m_trip), cnt, m_ztab_ok ? "measured" : "not measured", (double)m_i_zero);
	if (m_ztab_ok) {
		for (int i = 0;i < BOOST_ZTAB_N;i++) {
			commands_printf("  zero at duty %.2f: %.2f A", (double)BOOST_ZTAB_DUTY(i), (double)m_ztab[i]);
		}
	}
	commands_printf("   t [ms]  leg1 [A] leg2 [A]  in [A]  ref [A]  integ [V]  duty    sup [V]");

	// The first samples in order, then the ring from its oldest entry
	const int ring_len = BOOST_TRACE_LEN - BOOST_TRACE_START;
	const int ring_cnt = cnt > BOOST_TRACE_START ? (cnt - BOOST_TRACE_START - 1) / BOOST_TRACE_DIV + 1 : 0;
	const int n_first = cnt < BOOST_TRACE_START ? cnt : BOOST_TRACE_START;
	const int n_ring = ring_cnt < ring_len ? ring_cnt : ring_len;
	for (int k = 0;k < n_first + n_ring;k++) {
		int i = k;
		if (k >= n_first) {
			const int oldest = ring_cnt > ring_len ? ring_cnt % ring_len : 0;
			i = BOOST_TRACE_START + (oldest + k - n_first) % ring_len;
		}
		const boost_trace_t *t = &m_trace[i];
		commands_printf("%9.3f %8.2f %8.2f %7.2f %8.2f %9.2f  %.4f %7.1f",
				(double)(t->t_active * 1000.0), (double)t->i_leg_1, (double)t->i_leg_2,
				(double)t->i_in, (double)t->i_ref, (double)t->integ, (double)t->duty, (double)t->v_sup);
	}
	commands_printf(" ");
}

static void terminal_zero(int argc, const char **argv) {
	(void)argc; (void)argv;

	if (!m_lock || m_active) {
		commands_printf("Turn charge mode on first, and do not start charging.\n");
		return;
	}

	// The phases float to around 12 V without a supply
	if (m_v_sup > 0.5 * detect_threshold(m_v_bat)) {
		commands_printf("A supply is connected (%.1f V). Disconnect it first.\n", (double)m_v_sup);
		return;
	}

	commands_printf("PWM %.1f kHz, battery %.1f V", (double)(mc_interface_get_configuration()->foc_f_zv / 2000.0),
			(double)m_v_bat);
	commands_printf(" duty   leg1 [A]  leg2 [A]  in [A]");

	const int samples = 1000;
	for (int i = 2;i <= 19;i++) {
		const float duty = 0.05 * (float)i;

		m_zero_sum_1 = 0.0;
		m_zero_sum_2 = 0.0;
		m_zero_cnt = samples;
		m_zero_duty = duty;
		m_zero_n = samples + BOOST_ZERO_SKIP;

		for (int t = 0;t < 500 && m_zero_n > 0;t++) {
			chThdSleepMilliseconds(1);
		}

		if (m_zero_n != 0) {
			m_zero_n = 0;
			commands_printf("Aborted at duty %.2f: current flowed, is a supply connected?\n", (double)duty);
			return;
		}

		const float i1 = m_zero_sum_1 / (float)samples;
		const float i2 = m_zero_sum_2 / (float)samples;
		commands_printf(" %.2f   %7.2f   %7.2f   %6.2f", (double)duty, (double)i1, (double)i2, (double)-(i1 + i2));
		chThdSleepMilliseconds(20);
	}

	commands_printf("I zero with the outputs off: %.2f A\n", (double)m_i_zero);
}

static void terminal_lag(int argc, const char **argv) {
	if (argc == 2) {
		float us = -1.0;
		sscanf(argv[1], "%f", &us);
		if (us >= 0.0 && us <= 20.0 && !m_active) {
			m_sense_lag = us * 1e-6;
		} else {
			commands_printf("Use 0 to 20 us, not while charging.");
		}
	}
	commands_printf("Current sensor lag: %.2f us\n", (double)(m_sense_lag * 1e6));
}
