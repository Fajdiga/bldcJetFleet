# Boost Charge

Charges the battery through the motor windings. The supply + goes to one motor phase and the supply − to battery −. The ESC keeps that phase off and switches the two other legs as a synchronous boost converter.

This package only works on firmware built with `motor/boost_charge.c`. On other firmware the script stops with an error at the first `boost-status` call.

**Bench-tested on one JetFleet F6 only (lab supply, 0.8 to 3 A into 124 V). Start every new setup with a lab supply limited to 1 to 2 A.**

## Charger connection hardware

The winding current has a ripple of 10 to 40 A peak to peak at the switching frequency. It has to come from a capacitor bank at the phase connection, not from the charger and its leads:

- **Capacitor bank between the charger + and − at the phase connection**: at least 1000 µF, low ESR, rated well above the charger voltage, with short thick leads. Without it the phase voltage swings several volts each period, the current reading is wrong by an amp or more (it read 3 A where 1.9 A flowed), and the charger and the loop can oscillate.
- **Bleeder resistor across the bank** (for example 10 kΩ, 2 W), so that it is empty a few seconds after the charger is unplugged or switched off.
- **Put the bank in the charger cable or adapter**, not on the motor phase. A capacitor on a motor phase is shorted by every PWM edge while driving.
- A fast fuse in the supply + lead.

Never let the ESC boot while a charged capacitor bank is on the phase and the charger is off: at boot the offset calibration switches each phase to the battery. The firmware now skips the calibration when any phase is above 15 % of the battery voltage (at least 15 V), but an older firmware switched the battery straight onto the bank, which can destroy the high side FET.

## Rules

- Plug in with the ESC off, **or** enable charge mode first. Never plug the charger into an ESC that is driving the motor: the supply is shorted and only the fuse helps.
- A supply that is switched off but still plugged in counts as plugged in.
- The supply voltage must be below the empty-pack voltage (90 to 95 % of it), and above 12 % of the battery voltage plus 2 V for measurement error (at least 7.1 V for 42 V, 21.6 V for 168 V). Below that, charging is refused.
- There is no diode or switch on the supply. A fast fuse in the supply + lead is mandatory.

## How to charge

1. Open the **Charge** tab and tap **Enable charge mode**. The motor is now locked.
2. Plug in the charger. The page shows the charger voltage once it is steady.
3. Tap **Start charging**.
4. When the page says **Charged**, unplug. The motor is released 2 s after the charger voltage is gone.

If the charger was already plugged in when the ESC was switched on, steps 1 and 2 are done for you.

**Setup** has three things to set: the charge current, the voltage to charge each cell to, and the motor phase the charger + is wired to. Turn on "Start charging as soon as the charger is plugged in" to skip step 3. Everything else is under "Show advanced settings" and can stay as it is.

## What the states mean

- **Ready to drive**: normal motor operation.
- **Plug in the charger**: motor locked, waiting for a charger voltage that is steady for 1 s.
- **Charger connected**: waiting for the start button.
- **Checking the battery**: all cells and temperatures must be in range.
- **Starting**: low current while the rotor settles. The wheel can move up to half an electrical turn.
- **Ramping up**: current rises at the ramp rate.
- **Charging**: holds the charge current, less when the motor or FETs get hot or the charger voltage sags.
- **Topping off**: lowers the current to hold the highest cell at the target.
- **Charged**: current fell below the stop level for 5 s. Stays locked until the charger is unplugged.
- **Stopped by a fault**: stays until you clear it, or until the charger is unplugged.

## Without a BMS

If no BMS has been heard on the CAN bus for 30 s, the pack voltage divided by the cell count is used in place of the cell voltages, and there are no cell temperature checks. The cell count comes from Motor Settings > Additional Info > Setup. Set it correctly first: with a wrong count the charge is refused, or stops at the wrong voltage.

## Before the first charge

1. After installing the package, restart the ESC once without the charger. The firmware only looks for a charger, and only measures its voltage dividers at boot, once this package has been installed and has run. That boot also stores the current offsets (FOC offset calibration, then write the motor configuration).
2. With no charger connected, open the terminal and run `boost_status`. The three phase voltages must be well below "Detect at". If they are not, the ESC will lock itself at every boot. "Bat/ph cal" should say "measured": the phase voltage dividers were then measured against the battery divider at boot. If it says "not measured", the first current at each start is larger: the charger voltage can rise by up to 4 % for a moment, or a few amps can flow into the battery.
3. Set the charge current to 1 A for the first run.
4. For a run without cell checks, set the pack voltage limit under advanced settings and use **Bench test** at 1 A. Compare the charger current with an external meter.

## Settings worth knowing

- **Charger phase**: on hardware with two shunts this must be phase 3, the phase without a shunt.
- **Lowest and highest charger voltage**: 0 follows the charger, so it can be turned up or down while charging. Charging backs off when the charger sags to 80 % of its voltage (which it follows down within about 10 s) and stops when it rises 10 % above the highest voltage it held while delivering current.
- **Current loop Kp/Ki**: 0 uses 1.5 times the FOC current controller gains, which matches the winding path of one phase in series with two in parallel.
- **Switching frequency**: applied while charge mode is on, restored afterwards. 0 keeps the motor setting. The value is the PWM frequency, half of the FOC zero vector frequency. Up to 200 kHz: above about 17 kHz the current loop runs at about 35 kHz and only every few PWM periods. Hardware with phase current sensors (G18) then samples alternately in V0 and V7, which cancels sensor and sample delays.
- **Pack voltage limit**: must be set by hand for the bench test, as no cell data is used there.

## What the firmware does on its own

- Nothing at all until this package is installed: the charger detection is switched on by the script.
- Boot check: looks for a charger on the phases before the gate driver is enabled and skips the offset calibration if it finds one.
- Zeroes the charger current reading while charge mode is on and all FETs are off, so that small currents and reverse current are measured correctly.
- Charge lock: every motor command, the brake and the timeout action become "all FETs off".
- Trips inside the PWM interrupt: pack over-voltage, over-current, reverse current, supply too high, too low or too close to the battery voltage, and a setpoint that was not refreshed for 500 ms.
