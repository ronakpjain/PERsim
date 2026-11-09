# Adding models

Model choices are name-based registries, so a new implementation is one class
plus one table row. The CLI, help text, errors, and `--list-models` all read the
same registry.

## Where code lives

| Concern | Files |
|---|---|
| Tire registry and adapters | `include/persim/tire.hpp`, `src/tire.cpp` |
| Terrain registry and profiles | `include/persim/terrain.hpp`, `src/terrain.cpp` |
| SI parameters and source presets | `include/persim/parameters.hpp`, `src/parameters.cpp` |
| Coupled 14-DOF plant | `include/persim/vehicle.hpp`, `src/vehicle.cpp` |
| Sibling-model boundary | `include/persim/model_components.hpp`, `src/model_components.cpp` |
| Inverter/battery devices | `include/persim/powertrain.hpp`, `src/powertrain.cpp` |
| CAN sensor encoding | `include/persim/sensors.hpp`, `src/sensors.cpp` |
| CLI and scenarios | `include/persim/options.hpp`, `src/options.cpp` |
| Session timing | `include/persim/session.hpp`, `src/session.cpp` |

Components from [modeling-and-simulation](https://github.com/PurdueElectricRacing/modeling-and-simulation) are
included directly. Do not copy their equations into PERsim; wrap them at the
SI boundary instead.

## Add a tire model

1. Include the component (or implement the force law directly) in `src/tire.cpp`.
2. Derive from `persim::TireModel`.
3. Add a factory and one registry row.

```cpp
/** @brief Constant-friction tire for coarse sensitivity studies. */
class SimpleTire final : public TireModel {
public:
    Eigen::Vector2d force(double alpha, double kappa, double load, double grip) override {
        const double fx = -grip * load * std::tanh(kappa / 0.1);
        const double fy = -grip * load * std::tanh(alpha / 0.1);
        return {fx, fy};
    }
};

std::unique_ptr<TireModel> make_simple(const Parameters&) {
    return std::make_unique<SimpleTire>();
}

constexpr std::array<TireModelInfo, 4> registry{{
    {"reduced-mf", "Reduced Magic Formula (source default)", make_reduced_mf},
    {"tanh", "Smooth saturating tangent model", make_tanh},
    {"linear", "Linear stiffness with friction-circle saturation", make_linear},
    {"simple", "Constant-friction sensitivity model", make_simple},
}};
```

`--tire simple` and `--list-models` work immediately. If the model needs new
physics parameters, add them to `Parameters` with an SI unit in the comment and
set a default in `Parameters::Parameters()`; note whether the value is a source
preset or an estimate.

## Add a terrain profile

Derive from `TerrainModel`, implement `name()`, `summary()`, and `sample()`,
then add a registry row. `sample()` must return elevation in metres, an upward
unit normal, and the local grip multiplier. Override `marked_course()` only if
the viewer should paint the fixed proving-ground course.

```cpp
/** @brief Uniform low-grip skidpad. */
class Skidpad final : public TerrainModel {
public:
    std::string_view name() const override { return "skidpad"; }
    std::string_view summary() const override { return "Flat, uniform 0.7 grip"; }
    Surface sample(double, double) const override { return {0.0, {0, 0, 1}, 0.7}; }
};

constexpr std::array<TerrainInfo, 3> registry{{
    {"proving-ground", "...", []() -> std::unique_ptr<TerrainModel> {
        return std::make_unique<ProvingGroundTerrain>(); }},
    {"flat", "...", []() -> std::unique_ptr<TerrainModel> {
        return std::make_unique<FlatTerrain>(); }},
    {"skidpad", "Flat, uniform 0.7 grip", []() -> std::unique_ptr<TerrainModel> {
        return std::make_unique<Skidpad>(); }},
}};
```

Use `--terrain skidpad` or the `Options::terrain` field. Physics and rendering
sample the same object, so they cannot disagree.

## Contracts

- Registry names are the CLI and documentation source of truth.
- The model boundary uses SI doubles; keep mp-units inside the source adapters.
- Models are deterministic and independent of raylib, FIL, and CLI parsing.
- Document every public type and member: the Doxygen build fails on warnings.
- After a change, rebuild and smoke-test:

```sh
cmake --build --preset default -j 6
./build/persim --list-models
./build/persim-headless --standalone --scenario acceleration \
  --duration 2 --tire simple --terrain skidpad --verify
```
