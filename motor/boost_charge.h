/*
	Boost charging through the motor windings.

	Supply + on one motor phase, supply - on battery -. The FETs of the supply
	phase stay off and the two other legs switch in phase as a synchronous
	boost converter, using the motor inductance as the boost inductor.

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

#ifndef MOTOR_BOOST_CHARGE_H_
#define MOTOR_BOOST_CHARGE_H_

#include <stdbool.h>

// What the PWM interrupt has to do with the outputs
typedef enum {
	BOOST_OUT_UNLOCKED = 0,	// Charge lock not set, normal motor control
	BOOST_OUT_OFF,			// Charge lock set, all FETs off
	BOOST_OUT_SWITCHING		// Charge lock set, boost legs switching
} boost_out_t;

typedef enum {
	BOOST_TRIP_NONE = 0,
	BOOST_TRIP_PACK_OV,			// Battery voltage above v_bat_max
	BOOST_TRIP_OVERCURRENT,		// Leg or input current above i_trip
	BOOST_TRIP_REVERSE_CURRENT,	// Current flowing back into the supply
	BOOST_TRIP_SUPPLY_HIGH,		// Supply too close to or above battery voltage
	BOOST_TRIP_SUPPLY_LOW,		// Supply below v_sup_min (unplugged or collapsed)
	BOOST_TRIP_SUPPLY_OV,		// Supply above v_sup_max
	BOOST_TRIP_TIMEOUT,			// Setpoint not refreshed in time
	BOOST_TRIP_FAULT,			// VESC fault code active
	BOOST_TRIP_NO_OFFSETS,		// Booted with charger and no stored current offsets
	BOOST_TRIP_BAD_CONFIG,		// Not configured or not possible on this hardware
	BOOST_TRIP_NO_CURRENT,		// Duty at its limit and still no current: no supply or supply too low
	BOOST_TRIP_SUPPLY_RATIO		// Supply too low for the battery voltage, the lowest duty would push current back into it
} boost_trip_t;

typedef struct {
	float i_max;		// Maximum input current setpoint [A]
	float i_slew;		// Setpoint slew rate [A/s]
	float i_trip;		// Overcurrent trip for input and leg currents [A]
	float v_bat_max;	// Battery over-voltage trip [V]
	float v_sup_min;	// Supply under-voltage trip [V]
	float v_sup_max;	// Supply over-voltage trip [V]
	float kp;			// Current loop P gain [V/A], 0 = derive from motor config
	float ki;			// Current loop I gain [V/(A*s)], 0 = derive from motor config
	int phase;			// Supply phase, 0 to 2
} boost_config_t;

typedef struct {
	bool lock;
	bool active;
	boost_trip_t trip;
	float v_sup;		// Filtered voltage on the supply phase [V]
	float i_in;			// Filtered input current [A]
	float v_bat;		// Filtered battery voltage [V]
	float duty;			// High side duty of the boost legs
	float i_ref;		// Slew limited setpoint [A]
	float v_low_time;	// Time the supply phase has been below the detect threshold [s]
	float v_phase[3];	// Raw phase voltages [V]
} boost_status_t;

// Called once from mcpwm_foc_init before the gate driver is enabled and before
// the offset calibration. Sets the charge lock if a charger is connected.
bool boost_charge_boot_check(void);

// Called from the offset calibration with the average phase reading [ADC volts]
// while the phases were driven at 50 % duty, and the battery voltage [V]. Gives
// the gain of the phase dividers against the battery divider, which is what
// decides the direction of the first current when boost starts.
void boost_charge_calibrate(float v_ph_adc_half, float v_bat);

// True while the charge lock is set. All motor commands must be refused then.
bool boost_charge_locked(void);

// Set or clear the charge lock. Setting needs the motor at standstill with
// outputs off. Clearing needs boost stopped and the supply unplugged for 2 s.
bool boost_charge_set_lock(bool lock);

void boost_charge_configure(const boost_config_t *conf);

// Refresh the input current setpoint. Must be called at least every 500 ms
// while charging. Starts the converter if it is stopped and all checks pass.
// A setpoint of 0 ramps the current down and then switches the outputs off.
bool boost_charge_set_current(float current);

// Switch the outputs off now. The winding current decays into the battery.
void boost_charge_stop(void);
void boost_charge_clear_trip(void);
void boost_charge_get_status(boost_status_t *status);
const char* boost_charge_trip_str(boost_trip_t trip);

// Runs in the PWM interrupt, every cycle. The phase currents are in ampere,
// positive out of the inverter. duty_hs is the high side duty for all legs.
boost_out_t boost_charge_isr(float ia, float ib, float ic, bool motor_off, float dt, float *duty_hs);

// Supply phase, 0 to 2
int boost_charge_phase(void);

#endif /* MOTOR_BOOST_CHARGE_H_ */
