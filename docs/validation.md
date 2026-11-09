# Scope and limitations {#validation}

PERsim is an uncalibrated engineering prototype, not a validated digital twin.

The optional `--verify` runtime check reports whether firmware readiness and
control/physical response occurred during a scenario. It does not prove
vehicle-model calibration, exact MCU timing, stability for arbitrary parameters,
or correctness of the external TV algorithm.

The CM5 C client requires a Linux target environment. This checkout's macOS
host cannot directly bind SocketCAN. End-to-end CM5 support requires booting
the Yoctocoupler image, exercising vcan0, and running the actual CM5 application.
No CM5 TV application was present in the referenced Yoctocoupler image sources,
so an end-to-end VM/TV-algorithm validation is not claimed.

Historical PER26 captures have been reviewed for interface conventions and
operating ranges; see @ref captured_data for files, observations, data-quality
limits, and reproduction. This is not PER27 parameter identification.

## Known boundaries

- PER27 geometry and component constants use live source presets, but tire data,
  full inertia, suspension kinematics, brake hydraulics, steering ratio, and
  electrical/thermal parameters are not calibrated against the car.
- Wheel paths, toe/camber, and shock motion ratios use source linkage splines.
  Full upright rotational inertia, upright axial twist, anti-roll bars, bushing
  compliance, and arbitrary hardpoint changes at runtime are not modeled.
- Tires use the selected source reduced-MF/tanh/linear force model, its
  longitudinal-first friction circle, and source vertical spring/damper contact.
- Ground is a height field; arbitrary obstacles and body-to-body impacts are absent.
- Source IMU/GPS models use ideal truth by default; `--sensor-noise` adds
  explicitly estimated Gaussian sample noise, not measured bias or correlated
  errors. Latency mainly comes from report periods and 5 ms feedback coupling.
- Main-module SDC multiplexed inputs are held closed; fault-chain electrical
  behavior and the a_box battery-monitor firmware/SPI devices are not simulated.
- Inverters, battery, and thermals are lumped device approximations, not a full
  electrical transient simulation.
- FIL models MCU execution and peripherals but is not cycle-accurate hardware;
  its parallel board lanes have bounded skew.
- A QEMU Linux guest uses wall time and TCP, not the deterministic FIL clock.
- Procedural body geometry is a display proxy. It is not CAD or crash geometry.

Hardware correlation and parameter identification remain necessary before
using lap times, grip limits, fault timing, or thermal predictions as design evidence.
