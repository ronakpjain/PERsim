# Powertrain {#inverter_model}

@tableofcontents

`Powertrain` models the **other side** of the main module's AMK interface: four
inverter responders, torque delivery, the HV bus, and an equivalent pack. It
does not duplicate firmware state-machine logic.

## Enable sequence

For each inverter:

1. SystemReady requires a healthy HV bus and no modeled fault.
2. `bDcOn` must persist 20 ms before QuitDcOn.
3. `bInverterOn` persists 10 ms before inverter-ready.
4. `bEnable` is required for delivered torque.
5. A SET older than **37 ms** clears enable and zeros torque immediately.
6. Error reset clears a latched thermal fault; it relatches if still too hot.

The HV bus approaches pack voltage with an estimated **60 ms RC** time constant.
PC4 reports precharge incomplete until 95% is reached. Capacitor energy,
contactors, weld detection, insulation monitoring, and the accumulator board's
full battery-monitor chain are not modeled.

## Units and mapping

AMK A/B/C/D map to **FR/FL/RL/RR**. CRIT rotor rpm is physical wheel spin times
the 12.51 reduction.

Firmware setpoint and limit words are **parts per thousand of nominal motor
torque** (9.8 Nm). SuperDBC scales fields by 0.1, so decoded values look like
percent. The device applies

\f[ T_{shaft}=9.8\,\mathrm{Nm}\;\frac{raw}{1000}. \f]

Raw 2100 requests 20.58 Nm; default limits 2140/-500 are 20.972/-4.9 Nm. Do not
read decoded 214 as 214 Nm.

## Torque and limits

Enabled torque follows a first-order **12 ms** response. The target is limited
by received limit words and the source `GearedMotor` envelope: 21 Nm cap and
inverse-rpm constant power above 16,500 rpm. An estimated 20 kW motoring / 10 kW
regenerative limit applies per motor. Thermal derating spans 115..125 °C; motor
and inverter faults latch at 135 °C and 125 °C. Regeneration is disabled at
500 V or full SOC, and torque is gated when pack energy is exhausted or the pack
is thermally unhealthy.

Motoring wheel torque is shaft torque × reduction × efficiency. Regenerative
wheel torque divides by efficiency so recovered energy cannot exceed extracted
energy. Both the inverter limit and wheel-torque conversion call the source
`GearedMotor`. Loss of command/enable zeros torque without the ordinary lag;
there is no motor-speed servo forcing a wheel speed.

## Electrical and thermal estimates

The pack starts at 80% SOC with \f$V_{oc}=380+100\,SOC\f$ V, 6 kWh usable, and
0.08 Ω series resistance. Electrical power includes shaft power at 95%
efficiency, estimated copper loss \f$0.45T^2\f$, speed loss \f$10^{-6}rpm^2\f$,
and 20 W per active inverter. Terminal voltage includes the resistive drop; SOC
integrates electrical energy and is bounded to [0,1].

Motor and inverter temperatures use lumped heat capacities (6,000 and
2,500 J/K); pack heating uses current-squared loss and 50,000 J/K. Ambient is
25 °C. These are uncalibrated engineering estimates, not databook thermal
models. Overload percentages remain zero.

No switching-level inverter, FOC current loop, phase electrical model, detailed
AMK error catalog, coolant circuit, cell balancing, or hardware safety
certification is claimed.
