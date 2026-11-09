# AMK inverter, motor, and battery plant {#inverter_model}

@tableofcontents

## Device behavior versus firmware behavior

The real main-module firmware already contains an AMK control state machine.
Powertrain models the **other side of that interface**, not a second copy of
its firmware logic. The feedback messages are derived from device state and
wheel motion. Constant “ready” status cannot substitute for the enable sequence.

For each inverter:

1. SystemReady requires an available healthy HV bus and no modeled inverter fault.
2. A fresh `bDcOn` request must persist for 20 ms before QuitDcOn.
3. `bInverterOn` then persists for 10 ms before inverter-ready acknowledgment.
4. `bEnable` is required for delivered torque.
5. A SET older than **37 ms** clears enable and immediately zeros delivered torque.
6. Controller error-reset clears the modeled thermal fault, which relatches if
   the underlying temperature remains beyond its threshold.

The shared HV bus approaches pack voltage with an estimated **60 ms RC time
constant**. PC4 indicates precharge incomplete until 95% is reached. This is
a precharge-voltage/timing proxy; capacitor energy, contactor coils, weld
detection, insulation monitoring, switching transients, and the accumulator
board's full battery-monitor chain are not represented.

## AMK units and wheel mapping

AMK A/B/C/D map to **FR/FL/RL/RR**. CRIT rotor rpm comes from physical wheel
spin times the configured 12.51 reduction.

The firmware's setpoint and limit words are **parts per thousand of nominal
motor torque**. SuperDBC scales these fields by 0.1, producing percent-like
values. Its torque-limit signal unit labels say “Nm”, but the firmware's
`AMK_DEFAULT_POS_LIMIT=2140`, `AMK_DEFAULT_NEG_LIMIT=-500`, and VCU conversion
use the percentage/PPT interpretation. The device model applies

\f[
T_{shaft}=9.8\,\mathrm{Nm}\;\frac{raw}{1000}
         =9.8\,\mathrm{Nm}\;\frac{decoded}{100}.
\f]

Thus raw setpoint 2100 requests 20.58 Nm; the default positive limit 2140 is
20.972 Nm; the negative limit -500 is -4.9 Nm. Do **not** interpret decoded
214 as 214 physical newton-metres.

## Torque, limits, and mechanical coupling

Enabled torque follows a first-order **12 ms** response:

\f[
T_{next}=T+(T_{target}-T)(1-e^{-\Delta t/\tau}).
\f]

The target is limited by both received torque-limit words and the actual
source GearedMotor envelope: a 21 Nm AMK cap and inverse-rpm constant-power
behavior above 16,500 rpm. An additional
estimated 20 kW motoring / 10 kW regenerative limit applies per motor. Thermal
derating spans 115..125 °C motor temperature. Motor and inverter thermal faults
latch above 135 °C and 125 °C respectively.

Regeneration is disabled at 500 V or full modeled SOC. Torque is also gated
when modeled pack energy is exhausted or its thermal condition is unhealthy.
Loss of a command/enable does not use the ordinary torque lag to keep applying
old torque: it zeros output.

Motoring wheel torque is shaft torque × reduction × gearbox efficiency.
Regenerative wheel torque divides by efficiency, so recovered shaft energy
cannot exceed extracted wheel energy. Both the inverter limit and wheel-torque
conversion call the source GearedMotor; the regenerative correction is explicit
in the adapter. The vehicle mass matrix handles body/spin coupling and tire
contact torque.
There is no motor-speed target servo forcing a wheel to the reported rpm.

## Electrical and thermal estimates

The equivalent pack starts at 80% SOC with
\f$V_{oc}=380+100\,SOC\f$ V, 6 kWh usable energy, and 0.08 Ω series resistance.
Electrical power includes mechanical shaft power with a 95% conversion
efficiency, estimated copper loss \f$0.45T^2\f$, speed-dependent loss
\f$10^{-6}rpm^2\f$, and 20 W per active inverter. Pack current is power divided
by terminal voltage; terminal voltage includes the resistive drop. SOC integrates
electrical energy and is bounded to [0,1].

Motor and inverter temperatures use one lumped heat capacity/cooling path
each (6,000 and 2,500 J/K). Pack heating uses current-squared series loss and a
50,000 J/K thermal mass. Ambient is 25 °C. These values are **uncalibrated
engineering estimates**, not AMK datasheet-identified thermal models.

Feedback includes actual torque percentage, actual speed, status bits, DC-link
voltage, motor temperature, and inverter/IGBT temperature. Overload percentages
currently remain zero. No switching-level inverter, FOC/current-loop dynamics,
phase electrical model, detailed AMK error-code catalog, coolant circuit,
cell balancing, or hardware safety certification is claimed.
