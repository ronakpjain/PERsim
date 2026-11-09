/** @file parameters.hpp
 * @brief Live sibling-model presets and explicitly estimated 3D plant properties.
 * @ingroup physics
 */
#pragma once

#include <Eigen/Core>
#include <array>
#include <string>

namespace persim {
inline constexpr double gravity = 9.81; ///< Standard gravity magnitude, m/s².
inline constexpr double pi = 3.14159265358979323846; ///< Mathematical π.
/** @brief AMK wire order; differs from the modeling repo's FR/FL/RR/RL order. */
enum WheelIndex {
    FR, ///< Front right, inverter A.
    FL, ///< Front left, inverter B.
    RL, ///< Rear left, inverter C.
    RR, ///< Rear right, inverter D.
    wheel_count ///< Number of independent wheel corners.
};
/** @brief Stable human-readable wheel names in AMK order. */
inline constexpr std::array<const char*, 4> wheel_names{"FR", "FL", "RL", "RR"};

/** @brief SI-valued geometry, component constants, and model selection.
 * @details Presets are read directly from modeling-and-simulation. Source
 * components and explicit overrides are distinguished in docs/models.md. ISO 8855: +X forward, +Y left, +Z up.
 */
struct Parameters {
    /** @brief Load source PER27, tire, wheel, coilover, AMK, and aero presets. */
    Parameters();
    std::string tire = "reduced-mf"; ///< Tire model name; see persim/tire.hpp.
    bool sensor_noise = false; ///< Enable explicitly estimated IMU/GPS sample noise.
    double mass; ///< Vehicle plus driver mass, kg; PER27.
    double wheelbase; ///< Axle spacing, m; PER27.
    double front_fraction; ///< Static front weight fraction; PER27.
    double track; ///< Common front/rear track, m; PER27.
    double cg_height; ///< Nominal body reference height, m; PER27.
    Eigen::Vector3d total_inertia; ///< Total reference roll/pitch/yaw inertia, kg·m²; only yaw is sourced.
    double unsprung_mass = 32.0; ///< Estimated total hub/unsprung mass, kg.
    double radius; ///< Unloaded tire radius, m; PER27_WHEEL.
    double wheel_inertia; ///< Axial spin inertia per wheel, kg·m²; PER27_WHEEL.
    double spring; ///< Front shock stiffness, N/m; rear follows the source rear/front ratio.
    double damper; ///< Front shock damping, N·s/m; source linkage sets wheel motion ratio.
    double bump_travel; ///< Positive nominal bump limit from the source table, m.
    double droop_travel; ///< Magnitude of the source nominal droop limit, m.
    double stop_stiffness = 220000.0; ///< Additional unilateral travel-stop stiffness, N/m; estimate.
    double tire_stiffness = 180000.0; ///< Vertical contact stiffness, N/m; estimate, not the sibling prototype's 15 kN/m.
    double tire_damping = 650.0; ///< Vertical contact damping, N·s/m; estimate.
    double cornering_stiffness; ///< Reduced-MF lateral stiffness, N/rad; DEFAULT_TIRE.
    double longitudinal_stiffness; ///< Reduced-MF longitudinal stiffness, N per unit slip; DEFAULT_TIRE.
    double tire_shape; ///< Reduced-MF lateral shape factor; DEFAULT_TIRE.
    double longitudinal_tire_shape; ///< Reduced-MF longitudinal shape factor; DEFAULT_TIRE.
    double friction; ///< Source Tire default coefficient, not measured slick grip.
    double rolling_resistance = 0.015; ///< Rolling-resistance coefficient; estimate.
    double cda; ///< Drag coefficient × area, m²; AE27.
    double cla; ///< Downforce coefficient × area, m²; AE27.
    double cop_x; ///< Aero center X offset from body reference, m; AE27.
    double air_density; ///< Air density, kg/m³; AE27.
    double gear_ratio = 12.51; ///< Rotor-to-wheel speed ratio; current firmware, not sibling DT27=13.
    double gear_efficiency; ///< Gearbox mechanical efficiency; sibling DT27.
    double rated_motor_torque = 9.8; ///< AMK nominal torque used for percent/PPT conversion, Nm; firmware.
    double max_motor_torque; ///< Absolute shaft torque cap, Nm; AMK_DD5.
    double motor_knee_rpm; ///< Constant-torque/constant-power knee, rotor rpm; AMK_DD5.
    double torque_time_constant = 0.012; ///< Estimated first-order torque response time constant, s.
    double max_steering; ///< Mean nominal front-wheel steering lock supported by source rack table, rad.
    double steering_rate = 1.8; ///< Estimated center road-wheel actuator slew limit, rad/s.
    double steering_sensor_ratio = 4.0; ///< Estimated steering-wheel/road-wheel angle ratio for LWS, dimensionless.
    double max_brake_torque = 180.0; ///< Estimated per-wheel mechanical brake torque before axle bias, Nm.
    double battery_capacity_j = 6.0 * 3.6e6; ///< Estimated usable pack energy, J (6 kWh).
    double battery_resistance = 0.08; ///< Estimated equivalent pack resistance, Ω.

    /** @brief Return the total static weight fraction of the wheel's axle. */
    double axle_fraction(int i) const { return i < 2 ? front_fraction : 1.0 - front_fraction; }
    /** @brief Return the corner's statically supported total mass, kg. */
    double corner_mass(int i) const { return mass * axle_fraction(i) / 2.0; }
    /** @brief Return the corner's unsprung point mass, kg. */
    double hub_mass(int i) const { return unsprung_mass * axle_fraction(i) / 2.0; }
    /** @brief Return nominal hub offset from the sprung-body reference, m. */
    Eigen::Vector3d hub_reference(int i) const {
        return {i < 2 ? wheelbase * (1.0 - front_fraction) : -wheelbase * front_fraction,
                (i == FL || i == RL ? 1.0 : -1.0) * track / 2.0,
                radius - cg_height};
    }
};
}
