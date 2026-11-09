# Scope and limitations

PERsim is an uncalibrated engineering prototype, not a validated digital twin.

`--verify` checks that firmware reached readiness and the selected scenario
produced the expected physical response. It does not prove calibration, exact
MCU timing, stability for arbitrary parameters, or correctness of an external
TV algorithm. The CM5 client requires Linux; end-to-end guest/TV validation has
not been performed here, and historical PER26 captures are not PER27
identification ([captures](data.md)).

## Known boundaries

- Tire data, full inertia, brake hydraulics, steering ratio, and electrical or
  thermal constants are not calibrated against the car.
- Wheel paths, toe/camber, and shock motion ratios use source linkage splines.
  Full upright inertia, upright twist, anti-roll bars, bushing compliance, and
  runtime hardpoint changes are not modeled.
- Tires have no relaxation length, aligning moment, temperature/wear model, or
  measured load sensitivity.
- Terrain is a height field; arbitrary obstacles and body-to-body impacts are
  absent.
- IMU/GPS use ideal truth by default. `--sensor-noise` adds estimated Gaussian
  sample noise, not measured bias or correlated errors.
- Main-module SDC inputs are held closed. Fault-chain electrical behavior and
  the a_box battery-monitor SPI devices are not simulated.
- Inverters, battery, and thermals are lumped approximations, not full
  electrical transients.
- FIL models MCU execution and peripherals but is not cycle-accurate hardware;
  parallel board lanes have bounded skew. A QEMU guest uses wall time and TCP.
- The procedural body is a display proxy, not CAD or crash geometry.

Hardware correlation and parameter identification are required before using lap
times, grip limits, fault timing, or thermal predictions as design evidence.
