/*
	Host test for motor/boost_charge.c: runs the charge lock and the boost
	current loop against a simulated supply, motor and battery.

	Build and run: sh run.sh
 */

#include "boost_charge.h"
#include "conf_general.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

// 12s example from the plan: 34 V supply, 60 mOhm and 150 uH per phase
#define R_TOT		(1.5 * 0.060)
#define L_TOT		(1.5 * 150e-6)
#define FS			17500.0
#define DT			(1.0 / FS)

float sim_v_phase[3] = {0.0, 0.0, 0.0};
float sim_ph_gain[3] = {1.0, 1.0, 1.0};
float sim_v_bat = 44.0;
int sim_marker = 0;

static mc_configuration conf = {MOTOR_TYPE_FOC, {2040.0, 2050.0, 2047.0}, 150e-6 * 1000.0, 0.060 * 1000.0, 150e-6};
static mc_fault_code fault = FAULT_CODE_NONE;
static float v_supply = 0.0;	// 0 = unplugged
static int floating = 0;		// Phase voltage from leakage only, no current can flow
static float i_l = 0.0;			// Winding current, supply to battery
static float i_sens_ofs = 0.0;	// Offset error of one leg current sensor
static boost_out_t out = BOOST_OUT_UNLOCKED;

const volatile mc_configuration* mc_interface_get_configuration(void) { return &conf; }
mc_state mc_interface_get_state(void) { return MC_STATE_OFF; }
mc_fault_code mc_interface_get_fault(void) { return fault; }
float mc_interface_get_rpm(void) { return 0.0; }

// One PWM period. refresh >= 0 keeps the setpoint alive like the script does.
static void step(float refresh) {
	static int cnt = 0;
	if (refresh >= 0.0 && (cnt++ % 1750) == 0) {
		boost_charge_set_current(refresh);
	}

	// The supply holds all three phases at its voltage while the FETs are off
	sim_v_phase[0] = v_supply;
	sim_v_phase[1] = out == BOOST_OUT_SWITCHING ? 0.0 : v_supply;
	sim_v_phase[2] = sim_v_phase[1];

	float duty = 0.0;
	out = boost_charge_isr(0.0, -i_l / 2.0 + i_sens_ofs, -i_l / 2.0, true, DT, &duty);

	if (floating) {
		i_l = 0.0;
	} else if (out == BOOST_OUT_SWITCHING) {
		i_l += (v_supply - duty * sim_v_bat - i_l * R_TOT) / L_TOT * DT;
	} else {
		// FETs off: the body diodes only let current flow into the battery
		i_l += (v_supply - sim_v_bat - i_l * R_TOT) / L_TOT * DT;
		if (i_l < 0.0) {
			i_l = 0.0;
		}
	}
}

static void run(float seconds, float refresh) {
	for (int i = 0;i < (int)(seconds * FS);i++) {
		step(refresh);
	}
}

static boost_status_t status(void) {
	boost_status_t st;
	boost_charge_get_status(&st);
	return st;
}

// The offset calibration: each phase is held at the battery voltage. With
// settle != 1 the two readings differ, as if the phase filter was still moving.
static void calibrate(float settle) {
	for (int ph = 0;ph < 3;ph++) {
		sim_v_phase[ph] = 44.0;
		const float v = ph == 0 ? ADC_V_L1_VOLTS : (ph == 1 ? ADC_V_L2_VOLTS : ADC_V_L3_VOLTS);
		boost_charge_calibrate(ph, settle * v, v, 44.0);
	}

	float duty = 0.0;
	sim_v_phase[0] = sim_v_phase[1] = sim_v_phase[2] = 30.0;
	boost_charge_isr(0.0, 0.0, 0.0, true, DT, &duty);
	sim_v_phase[0] = sim_v_phase[1] = sim_v_phase[2] = 0.0;
}

int main(void) {
	const boost_config_t bc = {
			.i_max = 10.0, .i_slew = 2.0, .i_trip = 20.0, .v_bat_max = 51.0,
			.v_sup_min = 30.0, .v_sup_max = 38.0, .kp = 0.0, .ki = 0.0, .phase = 0};

	// Divider calibration. A reading that is still settling is thrown away.
	sim_ph_gain[0] = 1.03;
	sim_ph_gain[1] = 0.97;
	sim_ph_gain[2] = 1.01;
	calibrate(0.99);
	assert(fabsf(status().v_phase[0] - 30.0 * 1.03) < 0.01);

	// Settled: every phase gets its own correction
	calibrate(1.0);
	for (int ph = 0;ph < 3;ph++) {
		assert(fabsf(status().v_phase[ph] - 30.0) < 0.01);
	}

	// Package not installed: a charger is not looked for, at boot or later
	sim_v_phase[0] = sim_v_phase[1] = sim_v_phase[2] = 34.0;
	assert(!boost_charge_armed() && !boost_charge_boot_check());
	v_supply = 34.0;
	run(1.0, -1.0);
	assert(out == BOOST_OUT_UNLOCKED && !boost_charge_locked());

	// Marker left behind by a removed package: the boot lock is given up
	sim_marker = BOOST_EEPROM_ARM_MAGIC;
	assert(boost_charge_armed() && boost_charge_boot_check());
	run(4.0, -1.0);
	assert(boost_charge_locked());
	run(1.5, -1.0);
	assert(out == BOOST_OUT_UNLOCKED && !boost_charge_locked());

	// The script of the package arms the detection
	v_supply = 0.0;
	sim_v_phase[0] = sim_v_phase[1] = sim_v_phase[2] = 0.0;
	boost_charge_arm();

	// No charger: boot check does not lock and the motor stays available
	assert(!boost_charge_boot_check());
	run(1.0, -1.0);
	assert(out == BOOST_OUT_UNLOCKED && !boost_charge_locked());

	// Charger plugged in with the outputs off: locks by itself
	v_supply = 34.0;
	run(0.1, -1.0);
	assert(!boost_charge_locked());
	run(0.2, -1.0);
	assert(boost_charge_locked() && out == BOOST_OUT_OFF);

	// Unconfigured: refuses to start
	assert(!boost_charge_set_current(5.0));
	assert(status().trip == BOOST_TRIP_BAD_CONFIG);
	boost_charge_clear_trip();
	boost_charge_configure(&bc);

	// Regulates to the setpoint without overshooting it much on the way
	float i_peak = 0.0;
	for (int i = 0;i < (int)(5.0 * FS);i++) {
		step(5.0);
		i_peak = fmaxf(i_peak, i_l);
	}
	printf("5 A setpoint: i = %.3f A, peak %.3f A, duty %.4f\n", i_l, i_peak, status().duty);
	assert(out == BOOST_OUT_SWITCHING);
	assert(fabsf(i_l - 5.0) < 0.05 && i_peak < 5.5);
	assert(fabsf(status().duty - (34.0 - 5.0 * R_TOT) / 44.0) < 0.002);

	// A current sensor that reads 0.8 A off is zeroed while the FETs are off
	run(6.0, 0.0);
	assert(out == BOOST_OUT_OFF);
	i_sens_ofs = 0.8;
	run(1.0, -1.0);
	assert(fabsf(status().i_in) < 0.02);
	run(5.0, 1.0);
	printf("1 A setpoint, sensor 0.8 A off: i = %.3f A
", i_l);
	assert(out == BOOST_OUT_SWITCHING && fabsf(i_l - 1.0) < 0.05);

	// The setpoint is clamped to the configured maximum
	run(5.0, 50.0);
	assert(fabsf(i_l - 10.0) < 0.05);

	// Setpoint 0 ramps down and switches off without a trip
	run(6.0, 0.0);
	assert(out == BOOST_OUT_OFF && !status().active && status().trip == BOOST_TRIP_NONE);

	// The script hangs: outputs off within the timeout
	run(3.0, 5.0);
	assert(out == BOOST_OUT_SWITCHING);
	run(0.6, -1.0);
	assert(out == BOOST_OUT_OFF && status().trip == BOOST_TRIP_TIMEOUT);
	assert(!boost_charge_set_current(5.0));
	boost_charge_clear_trip();

	// Battery disconnected while charging: over-voltage trip
	run(3.0, 5.0);
	sim_v_bat = 52.0;
	run(0.001, 5.0);
	assert(out == BOOST_OUT_OFF && status().trip == BOOST_TRIP_PACK_OV);
	sim_v_bat = 44.0;
	boost_charge_clear_trip();

	// VESC fault while charging
	run(3.0, 5.0);
	fault = FAULT_CODE_OVER_VOLTAGE;
	run(0.001, 5.0);
	assert(out == BOOST_OUT_OFF && status().trip == BOOST_TRIP_FAULT);
	fault = FAULT_CODE_NONE;
	boost_charge_clear_trip();

	// Unplugged while charging: stops, and the current is gone
	run(3.0, 5.0);
	v_supply = 0.0;
	run(0.05, 5.0);
	assert(out == BOOST_OUT_OFF && status().trip == BOOST_TRIP_SUPPLY_LOW && i_l == 0.0);

	// The lock only opens after the supply has been gone for 2 s
	run(1.0, -1.0);
	assert(!boost_charge_set_lock(false));
	run(1.5, -1.0);
	assert(boost_charge_set_lock(false) && !boost_charge_locked());
	run(0.1, -1.0);
	assert(out == BOOST_OUT_UNLOCKED);

	// 12 V from FET leakage (or a 12 V charger): never locks on its own
	v_supply = 12.0;
	run(1.0, -1.0);
	assert(!boost_charge_locked());

	// Locked from the app: the script must not see it as unplugged
	assert(boost_charge_set_lock(true));
	run(3.0, -1.0);
	assert(boost_charge_locked() && status().v_low_time == 0.0);

	// Started with only leakage on the phase: trips instead of charging forever
	const boost_config_t bc_low = {
			.i_max = 10.0, .i_slew = 2.0, .i_trip = 20.0, .v_bat_max = 51.0,
			.v_sup_min = 10.0, .v_sup_max = 15.0, .kp = 0.0, .ki = 0.0, .phase = 0};
	boost_charge_configure(&bc_low);
	floating = 1;
	run(2.0, 5.0);
	assert(out == BOOST_OUT_OFF && status().trip == BOOST_TRIP_NO_CURRENT);
	floating = 0;
	boost_charge_clear_trip();

	// 12 V charger on a 160 V pack: refused before any switching
	sim_v_bat = 160.0;
	const boost_config_t bc_hv = {
			.i_max = 10.0, .i_slew = 2.0, .i_trip = 20.0, .v_bat_max = 170.0,
			.v_sup_min = 10.0, .v_sup_max = 15.0, .kp = 0.0, .ki = 0.0, .phase = 0};
	boost_charge_configure(&bc_hv);
	run(0.5, -1.0);
	assert(!boost_charge_set_current(5.0) && status().trip == BOOST_TRIP_SUPPLY_RATIO);
	assert(out == BOOST_OUT_OFF && i_l == 0.0);
	boost_charge_clear_trip();
	sim_v_bat = 44.0;
	run(0.5, -1.0);
	boost_charge_configure(&bc);

	// The app can always unlock its own manual lock
	assert(boost_charge_set_lock(false) && !boost_charge_locked());
	v_supply = 0.0;
	run(0.1, -1.0);

	// Supply too close to the battery voltage: refused
	assert(boost_charge_set_lock(true));
	v_supply = 34.0;
	sim_v_bat = 35.0;
	run(0.5, -1.0);
	assert(!boost_charge_set_current(5.0) && status().trip == BOOST_TRIP_SUPPLY_HIGH);
	assert(!boost_charge_set_lock(false));

	// Charger present at power on
	assert(boost_charge_boot_check());

	printf("boost_charge: all tests passed\n");
	return 0;
}
