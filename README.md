# PERsim

A raylib 3D proving ground for the PER27 car, coupled to real PER STM32 firmware
through [FIL](https://github.com/ronakpjain/fil). The plant owns the battery,
inverters, steering, IMU, GPS, and pedals; the firmware owns the vehicle state
machine and torque vectoring.

**An engineering prototype, not a validated digital twin.** The chassis has
6-DOF dynamics, four suspension coordinates, and four wheel spins. Tire,
inertia, electrical, thermal, and steering constants are estimates.

## Quick start

Requires CMake 3.25+, a C++20 compiler with `std::format`, Eigen, Python 3 with
NumPy/SciPy, and sibling checkouts of
[modeling-and-simulation](https://github.com/PurdueElectricRacing/modeling-and-simulation) and the PER monorepo.
Raylib, mp-units, and gsl-lite are fetched on first configure. On macOS:
`brew install cmake eigen doxygen`.

```sh
cmake --preset default
cmake --build --preset default -j 6
./build/persim
```

Presets: `default` (viewer + FIL), `headless`, `standalone` (no FIL), `debug`.
Build firmware images in the monorepo first. Override sibling paths with
`-DPERSIM_MONOREPO_DIR=... -DPERSIM_FIL_DIR=... -DPERSIM_MODELING_DIR=...`.

See [reused models](docs/models.md) and [adding models](docs/extending.md).

## Drive

| Control | Action |
|---|---|
| W / up | Throttle |
| S / down / space | Mechanical brake + firmware pedal inputs |
| A / D or left / right | Steer |
| Enter | Dashboard start button |
| T | Request TV enable/disable |
| R | Reset plant and firmware |
| P | Pause |
| C | Chase / orbit / overhead camera |
| Right mouse, wheel | Orbit, zoom |
| F | Tire force vectors |
| Escape | Close |

Startup holds the brake, pulses the dashboard start GPIO at 1.5 s, and waits for
the firmware buzzer. READY TO DRIVE normally occurs near 4 s. `--manual-start`
requires holding the brake and pressing Enter. The marked bump/ramp/ripple strip
is at `y=-14..-6 m`; the 45% grip patch is at `x=25..45, y=6..14 m`. Cones and
paint are visual only.

## Headless and options

```sh
./build/persim-headless --scenario acceleration --duration 6 --verify --csv build/run.csv
./build/persim-headless --scenario brake --duration 10 --verify
./build/persim --scenario circle --duration 6 --screenshot build/frame.png
./build/persim --list-models
```

Scenarios: `manual`, `idle`, `acceleration`, `circle`, `brake`, `bumps`.
`--tire` and `--terrain` select registered models; `--flat` is short for
`--terrain flat`. `--standalone` uses a simple pedal map instead of FIL and is
never an automatic fallback. `--sensor-noise` enables seeded IMU/GPS noise.
`--external-tv` replaces the STM32 TV with a CM5/SocketCAN peer. Run
`--help` for the full list.

CSV rows contain pose, IMU specific force, wheel loads/slips/travel, shock
lengths, inverter torque and enable state, battery state, and firmware counters.
`--preset headless` builds without raylib (`PERSIM_BUILD_VIEWER=OFF`).

## Documentation

- [Reused models and overrides](docs/models.md)
- [Adding models](docs/extending.md)
- [Physical model](docs/model.md)
- [Architecture and timing](docs/architecture.md)
- [Powertrain](docs/powertrain.md)
- [CM5 CAN bridge](docs/cm5.md)
- [Historical captures](docs/data.md)
- [Scope and limitations](docs/validation.md)

API reference: `cmake --build build --target docs`, then open
`build/docs/html/index.html`. The documentation build fails on Doxygen warnings.
