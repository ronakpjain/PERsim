# Reused modeling-and-simulation components {#reused_models}

@tableofcontents

## Live source integration

PERsim compiles components from the checkout selected by
`PERSIM_MODELING_DIR` (default `../modeling-and-simulation`). It does not copy
their force laws into this repository. `cmake/Modeling.cmake` builds only the
required C++ sources, with the same pinned mp-units and gsl-lite dependencies
as the sibling project. Python bindings, standalone demos, and its whole-car
planar solver are not built.

ModelComponents is the SI-double boundary around the sibling's unit-safe
types. Parameters::Parameters reads the live presets rather than maintaining a
second list of PER27 geometry, wheel, spring, tire, motor, and aero constants.
Firmware-facing AMK behavior remains in Powertrain.

## Component inventory

| Source | Integration |
|---|---|
| `models/vehicle/vehicle.hpp` | PER27 mass, geometry, weight fraction, and yaw inertia initialize Parameters directly. |
| `models/utils/quantities.hpp`, `eigen_quantities.hpp` | Actual mp-units quantities and Eigen conversions at the adapter boundary. |
| `models/tires/reduced_mf.hpp` | Default tangential tire forces; source DEFAULT_TIRE stiffness and shape. |
| `models/tires/tanh.hpp` | Selectable source tire with `--tire tanh`. |
| `models/tires/linear.hpp` | Selectable source tire with `--tire linear`. |
| `models/tires/tire.hpp` | Source longitudinal-first combined friction-circle saturation, not a separately implemented radial clamp. |
| `models/tires/vertical_tire.hpp` | Actual unilateral spring/damper contact evaluation, with the explicit vertical-parameter override below. |
| `models/dynamics/wheel.hpp` | PER27_WHEEL radius/inertia and Wheel::surfaceSpeed. Spin accelerations remain in the coupled mass matrix. |
| `models/dynamics/spring_damper.hpp` | Actual source spring/damper evaluation for each shock. |
| `models/suspension/coilover.hpp` | Front/rear shock stiffness, damping, reference length, maximum length, and stroke. |
| `models/suspension/suspension_kinematics.py` | Its actual interpolation objects are exported at build time and evaluated natively at every physics substep. |
| `models/suspension/data/kinematics_table.npz` | Wheel paths, toe/camber, shock lengths, and first/second geometry derivatives. |
| `models/powertrain/geared_motor.cpp` | AMK constant-torque/constant-power envelope, software caps, and gearbox torque conversions. |
| `models/powertrain/throttle_map.hpp` | Actual source ThrottleMap in explicit standalone mode only. FIL uses real firmware torque requests. |
| `models/aero/aero.hpp` | Actual Aero::getForce and COP, using AE27 defaults. Source aero clamps reverse forward-speed input to zero. |
| `models/dynamics/rigid_body.cpp` | RigidBodyMotion point velocities, RigidBodyLoads force/moment accumulation, RigidBodyState transforms, and RigidMount3D sensor geometry. |
| `models/sensors/imu.hpp` | Actual mounted specific-force/gyro evaluation and optional seeded sample noise. |
| `models/sensors/gps.hpp` | Actual rover position/velocity/baseline evaluation and optional seeded sample noise. |
| `models/utils/noise.hpp` | Used by the source IMU/GPS sample methods; deterministic per-plant PRNG. |

## Suspension export, without a Python physics loop

`tools/generate_suspension.py` imports the sibling's SuspensionKinematics class
directly and calls `from_file`. That class validates metres and source wheel
order and converts quaternions to the rotation-vector splines used by its own
`evaluate` method.

The exporter writes the **actual cubic B-spline knots and coefficients** into
a build-generated C++ header. It does not refit a lower-resolution grid or
substitute linear interpolation. Native tensor-product evaluation produces
positions, rotations, shock lengths, gradients, and Hessians.

The source wheel order **FR/FL/RR/RL** is explicitly reordered to PERsim/AMK
**FR/FL/RL/RR**. As in the Python evaluator, both rear tie-rod offsets remain
zero even though the NPZ contains additional rear rack samples.
Rack-to-steering inversion uses the nominal mean front-wheel
yaw samples. Rack is bounded to the supplied table domain; steering lock is
derived from that domain rather than invented beyond the available geometry.
The source table currently covers ±40 mm bump and ±10 mm rack, about ±7.9°
mean front-wheel yaw at nominal bump.

Small dynamic travel excursions beyond the tabulated bump endpoints use their
endpoint tangents, with soft travel stops. This extension is a numerical guard,
not additional solved linkage data. Shock extension/compression stops also use
the source maximum length and stroke.

The generated header records a SHA-256 digest of the table and source
interpolator. Table reference geometry must match the live PER27 presets;
a mismatch fails explicitly rather than silently mixing model revisions. Changes to either regenerate it. The table was originally
generated from the sibling's hardpoints by its linkage preprocessing; PERsim
uses that checked-in result rather than launching a nonlinear linkage solve
inside the real-time loop. To change hardpoints, regenerate the source NPZ in
the modeling repository first.

## Explicit adaptations and overrides

- **Full 3D coupling:** source wheel paths replace the body-Z slider model.
  The 14×14 mass solve includes path Jacobians and centripetal, Coriolis,
  path-curvature, and prescribed-rack acceleration terms. Shock forces map
  through actual `dL/dq`; contact moments also map through axle-direction
  derivatives. The axial wheel-inertia approximation does not model a full
  upright inertia tensor or axial twist of the upright.
- **Vertical tire parameters:** reuse VerticalTire with 180 kN/m and
  650 N·s/m, not its prototype 15 kN/m / zero-damping default. These remain
  estimates; the softer prototype default would consume the linkage's travel
  range under the static corner load.
- **Gear ratio:** use the source GearedMotor with **12.51**, matching current
  firmware, instead of DT27's 13.0. Gearbox efficiency comes from DT27.
- **Regeneration:** the source gearbox uses a motoring torque conversion in both
  directions. The adapter corrects regenerative wheel torque by dividing that
  result by efficiency squared, so recovered shaft energy cannot exceed
  extracted wheel energy.
- **Contact and slip:** terrain-normal/camber contact geometry and reversible,
  low-speed-regularized slip remain 3D adapter responsibilities. Tire forces and
  the combined-slip policy come from the selected source tire.
- **Sensors:** retain the firmware's IMU mounting rotation and source sensor
  evaluation. The IZZE CAN adapter negates conventional accelerometer specific
  force to reproduce the historical device's `g-a` polarity (-1 g stationary
  body Z); gyro and source/CSV truth are unchanged. See @ref captured_data.
  GPS rover is at the body reference with a synthetic base one wheelbase behind
  it. Mount locations and noise levels remain explicit assumptions, not measured
  hardware presets.
- **Noise:** `--sensor-noise` enables independent per-axis sample deviations:
  accelerometer 0.02 g, gyro 0.25°/s, GPS position 0.02 m, velocity 0.05 m/s,
  baseline 0.005 m. Default noise is zero. The source PRNG starts at seed zero.
- **Reset:** relax the source shock/contact system before time zero; do not
  override the source reference lengths with a fabricated 1:1 preload.

## Components not substituted into the closed loop

- `vehicle/planar_7dof.cpp` suppresses heave, roll, and pitch and uses a
  quasi-static load-transfer solve. It cannot replace the full 3D plant.
- `corner/rigid_corner.cpp` is a fixed-mount planar assembly with planar slip
  conventions. Its tire, wheel, and motor components are reused individually;
  calling its complete evaluation would double-integrate wheel accelerations.
- `suspension/suspension_jacobians.py` expresses the linkage using a different
  seven-coordinate ride vector. PERsim differentiates the same source splines
  with respect to its 14-DOF coordinates rather than mixing coordinate systems.
- `vehicle/full_15dof.py`, `corner/suspension_corner.py`, and
  `powertrain/brakes.hpp` are placeholders with no executable physical model.
- `algorithms/control/*` are alternative controllers, not plant components;
  they do not run alongside or replace the real FIL torque-vector firmware.
- No battery, inverter handshake, thermal, terrain, or collision model exists
  in the sibling repository. Those remain explicitly approximate PERsim devices.

See @ref vehicle_model for coupling equations and @ref validation for limits.
