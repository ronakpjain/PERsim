/**
 * @file parameters.cpp
 * @brief Live sibling presets and explicit 3D plant overrides.
 * @ingroup physics
 */
#include "persim/parameters.hpp"
#include "persim/tire.hpp"
#include "persim/suspension_db.hpp"

#include <aero/aero.hpp>
#include <dynamics/wheel.hpp>
#include <powertrain/geared_motor.hpp>
#include <suspension/coilover.hpp>
#include <tires/reduced_mf.hpp>
#include <vehicle/vehicle.hpp>

#include <cmath>
#include <stdexcept>

namespace persim {
Parameters::Parameters() {
    mass = siValue(PER27.mass);
    wheelbase = siValue(PER27.wheelbase);
    front_fraction = PER27.front_weight_fraction;
    track = siValue(PER27.track_width);
    cg_height = siValue(PER27.cg_height);
    total_inertia = {38.0, 110.0, siValue(PER27.yaw_inertia)};
    radius = siValue(PER27_WHEEL.radius);
    wheel_inertia = siValue(PER27_WHEEL.inertia);
    spring = siValue(FRONT_COILOVER.spring_damper.stiffness);
    damper = siValue(FRONT_COILOVER.spring_damper.damping);
    bump_travel = model_data::travel_knots.back();
    droop_travel = -model_data::travel_knots.front();
    cornering_stiffness = siValue(DEFAULT_TIRE.lateral_stiffness);
    longitudinal_stiffness = siValue(DEFAULT_TIRE.longitudinal_stiffness);
    tire_shape = DEFAULT_TIRE.lateral_shape;
    longitudinal_tire_shape = DEFAULT_TIRE.longitudinal_shape;
    friction = source_tire_friction();
    cda = siValue(AE27.CDA);
    cla = siValue(AE27.CLA);
    cop_x = siValue(AE27.cop_offset_x);
    air_density = siValue(AE27.air_density);
    gear_efficiency = DT27.efficiency;
    max_motor_torque = siValue(AMK_DD5.max_torque);
    motor_knee_rpm = siValue(AMK_DD5.knee_speed) * 60 / (2 * pi);
    max_steering = std::min(-model_data::steering_samples.front(), model_data::steering_samples.back());
    if (std::abs(model_data::reference_chassis_center[0] + siValue(PER27.front_distance())) > 1e-9 ||
        std::abs(model_data::reference_chassis_center[2] - cg_height) > 1e-9 ||
        std::abs(model_data::reference_wheel_centers[1] + track / 2) > 1e-9 ||
        std::abs(model_data::reference_wheel_centers[2] - radius) > 1e-9 ||
        std::abs(model_data::reference_wheel_centers[6] + wheelbase) > 1e-9)
        throw std::runtime_error("Source suspension table does not match PER27 geometry; regenerate it in modeling-and-simulation");
}
}
