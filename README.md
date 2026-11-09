# PERsim

A raylib 3D proving ground for the PER27 car, coupled to real PER STM32
firmware through [FIL](https://github.com/ronakpjain/fil). The default backend runs
dashboard, main module, torque-vector, and both driveline ELF images. The
plant supplies the battery, inverter, steering, IMU, GPS, and pedal devices.

**This is an engineering prototype, not a validated digital twin.** The car
has a dynamic 6-DOF chassis, four suspension coordinates, and four wheel-spin
coordinates. Roll, pitch, heave, wheel lift, and terrain response are physical
states, not display animations. Component models and suspension linkage data
are reused directly from `../modeling-and-simulation`. Several inertia values,
electrical/thermal constants, and tire parameters still require measurement and
calibration. The rendered car is a procedural proxy, not PER CAD.

## Build and drive

Requires CMake 3.25+, a C++20 compiler with `std::format`, Eigen (3.4 or 5),
Python 3 with NumPy/SciPy, the modeling checkout, and PER firmware images.
Raylib 5.5, mp-units, and gsl-lite are fetched at pinned revisions; the first
configure needs GitHub access. On macOS, install build dependencies with
`brew install cmake eigen doxygen`. Install NumPy/SciPy in your Python environment;
select that interpreter with `-DPython3_EXECUTABLE=/path/to/venv/bin/python`.

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 6
./build/persim
```

Defaults match this workspace:

- `../monorepo`: firmware outputs, SuperDBC, and current CAN node wiring
- `~/coding/projects/fil`: FIL C++ source checkout; developed against `main`
- `../modeling-and-simulation`: live component model sources, presets, and suspension splines
- `../yoctocoupler`: optional CM5/Pi 5 QEMU guest, not needed for STM32 TV mode

Override paths with `-DPERSIM_MONOREPO_DIR=...`, `-DPERSIM_FIL_DIR=...`, and
`-DPERSIM_MODELING_DIR=...`.
The simulator does not rebuild the firmware; build it in the monorepo first.
Do not substitute unrelated ELF images for the configuration used to build them.

The default start sequence holds the brake, pulses the actual dashboard start
GPIO at 1.5 simulated seconds, and waits for the firmware's 2.5-second buzzer.
READY TO DRIVE normally occurs around 4 seconds. `--manual-start` instead
requires holding the brake and pressing Enter.

| Control | Action |
|---|---|
| W / up | Throttle |
| S / down / space | Mechanical brake + firmware pedal inputs |
| A / D or left / right | Steer |
| Enter | Dashboard start button |
| T | Request TV enable/disable |
| R | Reset **both** the physical plant and firmware |
| P | Pause |
| C | Centered rear chase / orbit / overhead camera |
| Right mouse, wheel | Orbit, zoom |
| F | Tire force vectors |
| Escape | Close |

Keyboard throttle builds progressively; A/D steering ramps and reduces its
maximum angle with speed to avoid an instant full-lock slide. This is an input
aid, not extra tire grip. Gamepad left stick and analog triggers retain direct
analog control. The default chase camera stays centered behind the car's heading,
including through turns. TV runs in the firmware's autocross mode.

The marked strip at
`y=-14..-6 m` contains a bump, ramp, and ripples. The patch at
`x=25..45, y=6..14 m` has 45% grip. Cones and markings are visual only.

## Headless and telemetry

```sh
./build/persim-headless --scenario acceleration --duration 6 --verify --csv build/run.csv
./build/persim-headless --scenario brake --duration 10 --verify
./build/persim --scenario circle --duration 6 --screenshot build/frame.png
```

`--standalone` explicitly replaces the firmware controller with a simple pedal
map, but retains the inverter handshake, torque lag, and vehicle dynamics.
It is never selected automatically after a firmware error.

For a display-free build use `-DPERSIM_BUILD_VIEWER=OFF`. To build without FIL
use `-DPERSIM_WITH_FIL=OFF` and pass `--standalone` at runtime. SuperDBC and
node configuration generation still use the monorepo. CSV rows contain
vehicle pose, velocity, body-frame IMU specific force, wheel loads/slips/travel,
source shock lengths/steering/camber, inverter torque and enable state, battery state, and firmware counters. CSV acceleration remains conventional specific
force (+g on stationary upright Z), unlike the IZZE device's opposite CAN polarity.
Viewer resets start a new numbered CSV file.

## Reused physical models

Tires, vertical contact, wheel parameters/tread speed, spring/dampers, front/rear
coilovers, aero, motor/gearbox, mounted IMU/GPS, and rigid-body kinematics come
from the sibling checkout. Its actual suspension splines supply wheel paths,
camber, toe, shock motion ratios, and dynamic Jacobians. Python runs only at
build time; the real-time plant is native C++.

`--tire reduced-mf` (default), `--tire tanh`, and `--tire linear` select the
available source tire models. `--sensor-noise` enables seeded source IMU/GPS
sample noise. Real firmware remains the controller; the sibling planar solver
and placeholder full-3D/brake models are not substituted into the closed loop.

See [the component inventory and explicit overrides](docs/models.md).

## Historical capture review

`data/PER26/` contains historical vehicle telemetry, not PER27 calibration.
[The capture review](docs/data.md) records corroborated interfaces, the IZZE
accelerometer sign correction, measured operating ranges, and problematic signals.
`tools/analyze_capture.py` is a read-only, streaming offline analysis utility
(requires pandas in addition to NumPy/SciPy); it does not fit parameters,
replay firmware, or add a test suite.

## Virtualized CM5 TV

```sh
./build/persim --external-tv
```

This removes the STM32 torque-vector board from the network and opens a
**localhost-only** CAN gateway on port 29500. A small Linux userspace client
connects the Yoctocoupler guest's existing SocketCAN interface to it. There is
no new kernel driver, and the VM is **not** instruction-lockstepped with FIL.
No CM5 TV algorithm/application is bundled with this project.

See [the CM5 bridge guide](docs/cm5.md) for the wire protocol, guest build,
QEMU networking, optional Yocto recipe, and limitations.

## Doxygen reference

```sh
cmake --build build --target docs
open build/docs/html/index.html
```

The documentation build fails on Doxygen warnings. The generated reference
includes source browsing and these design pages:

- [Reused source components and explicit adaptations](docs/models.md)
- [Physical model, equations, units, and parameter provenance](docs/model.md)
- [Timing, ownership, firmware devices, and CAN wiring](docs/architecture.md)
- [AMK inverter, motor, and battery model](docs/powertrain.md)
- [CM5 guest and CAN gateway](docs/cm5.md)
- [Scope and known limitations](docs/validation.md)
- [Historical telemetry evidence and data quality](docs/data.md)

Public interfaces are documented in `include/persim/`; implementations live in
`src/`. No changes to the referenced repositories are required.
