# Reused models and overrides {#reused_models}

@tableofcontents

PERsim compiles components from the sibling
[modeling-and-simulation](https://github.com/PurdueElectricRacing/modeling-and-simulation) checkout selected by
`PERSIM_MODELING_DIR` (default `../modeling-and-simulation`). Force laws are not
copied into this repository. `cmake/SiblingModels.cmake` builds only the needed
C++ sources with the same pinned mp-units and gsl-lite revisions; the sibling's
Python bindings, demos, and whole-car planar solver are not built.

`ModelComponents` is the SI-double boundary around the sibling's unit-safe
types. `Parameters` reads live presets rather than maintaining a second list of
PER27 constants. Firmware-facing AMK behavior lives in `Powertrain`.

## Inventory

| Source | Use |
|---|---|
| [vehicle.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/vehicle/vehicle.hpp) | PER27 mass, geometry, weight fraction, yaw inertia |
| [quantities.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/utils/quantities.hpp) | mp-units quantities and Eigen conversions |
| [tire.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/tires/tire.hpp) | Combined-slip policy, longitudinal-first friction circle |
| [reduced_mf.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/tires/reduced_mf.hpp) | Default tangential tire; source stiffness and shape |
| [tanh.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/tires/tanh.hpp), [linear.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/tires/linear.hpp) | Alternative registered tires |
| [vertical_tire.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/tires/vertical_tire.hpp) | Unilateral vertical contact |
| [wheel.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/dynamics/wheel.hpp) | Wheel radius/inertia and tread speed |
| [spring_damper.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/dynamics/spring_damper.hpp) | Shock force along increasing length |
| [coilover.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/suspension/coilover.hpp) | Front/rear shock presets and stroke limits |
| [suspension_kinematics.py](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/suspension/suspension_kinematics.py) | Source interpolator exported at build time |
| [kinematics_table.npz](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/suspension/data/kinematics_table.npz) | Wheel paths, toe/camber, shock lengths, derivatives |
| [rigid_body.cpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/dynamics/rigid_body.cpp) | Point velocities, load accumulation, sensor mounts |
| [geared_motor.cpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/powertrain/geared_motor.cpp) | AMK envelope, limits, gearbox conversions |
| [throttle_map.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/powertrain/throttle_map.hpp) | Standalone pedal map only |
| [aero.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/aero/aero.hpp) | Aero force and COP using AE27 defaults |
| [imu.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/sensors/imu.hpp) | Mounted specific force/gyro and optional noise |
| [gps.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/sensors/gps.hpp) | Rover position/velocity and baseline |
| [noise.hpp](https://github.com/PurdueElectricRacing/modeling-and-simulation/blob/main/models/utils/noise.hpp) | Deterministic per-plant sample noise |

## Suspension splines without a Python loop

`tools/generate_suspension.py` imports the source `SuspensionKinematics` class
and exports its actual cubic B-spline knots, coefficients, and a SHA-256 digest
into a generated C++ header. The runtime evaluates the tensor product natively,
including positions, rotations, shock lengths, gradients, and Hessians.

The source order **FR/FL/RR/RL** is reordered to PERsim/AMK **FR/FL/RL/RR**.
As in the source evaluator, rear tie-rod offsets stay zero even though the NPZ
contains rear rack samples. Rack is bounded to the table domain; steering lock
is derived from that domain (±10 mm rack, about ±7.9° mean front-wheel yaw).
Excursions beyond the bump table use endpoint tangents and soft travel stops.
Table reference geometry must match the live presets, and a mismatch fails
explicitly instead of mixing model revisions.

## Explicit adaptations

- **3D coupling.** The 14×14 mass solve uses source wheel-path Jacobians and
  Hessians; shock forces map through the actual `dL/dq`. Axial wheel inertia is
  modeled, but not a full upright inertia tensor or upright twist.
- **Vertical contact.** Source `VerticalTire` with 180 kN/m and 650 N·s/m, not
  its 15 kN/m / zero-damping prototype defaults. These remain estimates.
- **Gear ratio.** Source `GearedMotor` with **12.51** for current firmware, not
  DT27's 13.0. Gearbox efficiency comes from DT27.
- **Regeneration.** The source gearbox applies the motoring conversion in both
  directions; the adapter divides regenerative wheel torque by efficiency
  squared so recovered energy cannot exceed extracted energy.
- **Contact and slip.** Terrain-normal/camber contact geometry and reversible,
  low-speed-regularized slip (1 m/s denominator) are adapter responsibilities.
  Tire forces and combined-slip saturation stay in the selected source tire.
- **Sensors.** The firmware IMU mounting rotation is retained. The IZZE CAN
  encoder negates accelerometer specific force to reproduce the device's `g-a`
  polarity; gyro, plant truth, and CSV values are unchanged. GPS rover is at
  the body reference with a synthetic base one wheelbase behind it.
- **Noise.** Off by default. `--sensor-noise` enables 0.02 g accelerometer,
  0.25°/s gyro, 0.02 m position, 0.05 m/s velocity, and 0.005 m baseline noise
  from a seed-zero PRNG.
- **Reset.** A damped relaxation settles the actual shock/contact system before
  time zero; source reference lengths are not replaced by a fabricated preload.

## Not substituted into the closed loop

- `vehicle/planar_7dof.cpp` suppresses heave/roll/pitch and uses quasi-static
  load transfer, so it cannot replace the full 3D plant.
- `corner/rigid_corner.cpp` is a planar fixed-mount assembly; calling its whole
  evaluation would double-integrate wheel accelerations.
- `suspension/suspension_jacobians.py` uses a different seven-coordinate ride
  vector. PERsim differentiates the same source splines in its 14-DOF frame.
- `vehicle/full_15dof.py`, `corner/suspension_corner.py`, and
  `powertrain/brakes.hpp` are placeholders with no executable physical model.
- `algorithms/control/*` are alternative controllers, not plant components.
- No battery, inverter handshake, thermal, terrain, or collision model exists
  in the sibling; these remain explicitly approximate PERsim devices.

To add a tire or terrain model, see [adding models](extending.md). For coupling
equations and limits see [the physical model](model.md) and
[scope and limitations](validation.md).
