/** @file vehicle.cpp
 * @brief Coupled 14-DOF chassis/linkage dynamics using the sibling's components.
 * @ingroup physics
 */
#include "persim/vehicle.hpp"
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace persim {
namespace {
/** @brief Generalized 14×14 positive-definite mass matrix. */
using Matrix = Eigen::Matrix<double, 14, 14>;
/** @brief Generalized velocity/force vector in documented coordinate order. */
using Generalized = Eigen::Matrix<double, 14, 1>;
/** @brief World-translation Jacobian for one unsprung point mass. */
using Jacobian = Eigen::Matrix<double, 3, 14>;
/** @brief Build the matrix satisfying skew(v) * w = v × w. */
Eigen::Matrix3d skew(const Vec3& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
}
/** @brief Smooth half-cosine window with flat support and zero endpoints. */
double window(double v, double begin, double end, double edge) {
    if (v <= begin || v >= end) return 0.0;
    const double t = std::min({(v - begin) / edge, (end - v) / edge, 1.0});
    return 0.5 - 0.5 * std::cos(pi * t);
}
/** @brief Analytic road height in metres for the proving-ground profile. */
double height_at(double x, double y, TerrainKind kind) {
    if (kind == TerrainKind::flat) return 0.0;
    const double strip = window(y, -14.0, -6.0, 1.0);
    const double bump = 0.065 * window(x, 18.0, 22.0, 2.0);
    const double ramp = 0.4 * window(x, 35.0, 53.0, 7.0);
    const double ripple = 0.025 * window(x, 58.0, 80.0, 2.0) * (1.0 + std::sin(x * 3.0));
    return strip * (bump + ramp + ripple);
}
}
Surface surface_at(double x, double y, TerrainKind kind) {
    constexpr double e = 0.001;
    Vec3 normal{-(height_at(x + e, y, kind) - height_at(x - e, y, kind)) / (2 * e),
                -(height_at(x, y + e, kind) - height_at(x, y - e, kind)) / (2 * e), 1.0};
    const double grip = kind == TerrainKind::proving_ground && x > 25 && x < 45 && y > 6 && y < 14 ? 0.45 : 1.0;
    return {height_at(x, y, kind), normal.normalized(), grip};
}
Vehicle::Vehicle(Parameters parameters, TerrainKind terrain)
    : parameters_(std::move(parameters)), models_(parameters_), terrain_(terrain) {
    const auto& p = parameters_;
    sprung_inertia_ = p.total_inertia.asDiagonal();
    for (int i = 0; i < 4; ++i) {
        const auto geometry = models_.suspension(i, 0, 0);
        const Vec3& r = geometry.hub;
        sprung_inertia_ -= p.hub_mass(i) * (r.squaredNorm() * Eigen::Matrix3d::Identity() - r * r.transpose());
        sprung_inertia_ -= p.wheel_inertia * geometry.orientation.col(1) * geometry.orientation.col(1).transpose();
    }
    if (p.mass <= p.unsprung_mass || p.spring <= 0 || p.tire_stiffness <= 0 || p.max_steering <= 0 ||
        p.radius <= 0 || p.wheel_inertia <= 0 || p.gear_efficiency <= 0 || p.gear_efficiency > 1 ||
        sprung_inertia_.llt().info() != Eigen::Success)
        throw std::invalid_argument("Invalid vehicle mass, inertia, stiffness, or drivetrain");
    reset();
}
void Vehicle::reset(const Vec3& ground_position, double heading) {
    const auto& p = parameters_;
    state_ = {};
    state_.position = ground_position + Vec3(0, 0, p.cg_height
        + surface_at(ground_position.x(), ground_position.y(), terrain_).height);
    state_.orientation = Quat(Eigen::AngleAxisd(heading, Vec3::UnitZ()));
    state_.rack_offset = models_.rack_offset(0);
    for (int i = 0; i < 4; ++i) state_.wheels[i].travel = -p.corner_mass(i) * gravity / p.tire_stiffness;
    telemetry_ = {};
    // Settle the actual shock reference lengths, not a separate 1:1 preload model.
    const Actuators parked{{}, 1, 0};
    for (int i = 0; i < 4000; ++i) {
        step(parked, 0.0005);
        double motion = state_.velocity.norm() + state_.angular_velocity.norm();
        for (const auto& wheel : state_.wheels) motion += std::abs(wheel.travel_rate);
        if (i > 1000 && motion < 1e-6) break;
    }
    const Vec3 shift(ground_position.x() - state_.position.x(), ground_position.y() - state_.position.y(), 0);
    state_.position += shift;
    state_.velocity.setZero(); state_.angular_velocity.setZero();
    for (auto& wheel : state_.wheels) { wheel.travel_rate = 0; wheel.omega = 0; wheel.phase = 0; }
    telemetry_.acceleration.setZero(); telemetry_.angular_acceleration.setZero();
    telemetry_.specific_force = state_.orientation.conjugate() * Vec3(0, 0, gravity);
    for (auto& wheel : telemetry_.wheels) { wheel.hub += shift; wheel.contact += shift; }
}
void Vehicle::step(const Actuators& actuators, double dt) {
    if (!std::isfinite(dt) || dt <= 0 || dt > 0.0005 + 1e-12)
        throw std::invalid_argument("Physics substep must be in (0, 0.0005] seconds");
    const auto& p = parameters_;
    const double target = std::clamp(actuators.steering, -1.0, 1.0) * p.max_steering;
    state_.steering += std::clamp(target - state_.steering, -p.steering_rate * dt, p.steering_rate * dt);
    const double rack = models_.rack_offset(state_.steering);
    const double rack_rate = (rack - state_.rack_offset) / dt;
    const double rack_acceleration = (rack_rate - state_.rack_rate) / dt;
    state_.rack_offset = rack; state_.rack_rate = rack_rate;
    const Eigen::Matrix3d rotation = state_.orientation.toRotationMatrix();
    const Vec3 omega = state_.angular_velocity;
    const Vec3 world_omega = rotation * omega;
    const Vec3 body_velocity = rotation.transpose() * state_.velocity;
    const Vec3 g(0, 0, -gravity);
    const double sprung_mass = p.mass - p.unsprung_mass;
    Matrix mass = Matrix::Zero();
    Generalized rhs = Generalized::Zero();
    mass.block<3, 3>(0, 0) = sprung_mass * Eigen::Matrix3d::Identity();
    mass.block<3, 3>(3, 3) = sprung_inertia_;
    rhs.head<3>() = sprung_mass * g;
    rhs.segment<3>(3) = -omega.cross(sprung_inertia_ * omega);
    telemetry_.mechanical_power = 0;
    for (int i = 0; i < 4; ++i) {
        const auto& wheel = state_.wheels[i];
        auto& t = telemetry_.wheels[i];
        const auto geometry = models_.suspension(i, wheel.travel, rack);
        const Vec3& r = geometry.hub;
        const Vec3 relative_velocity = geometry.travel * wheel.travel_rate + geometry.rack * rack_rate;
        Jacobian jac = Jacobian::Zero();
        jac.block<3, 3>(0, 0).setIdentity();
        jac.block<3, 3>(0, 3) = -rotation * skew(r);
        jac.col(6 + i) = rotation * geometry.travel;
        const Vec3 bias = rotation * (omega.cross(omega.cross(r)) + 2 * omega.cross(relative_velocity)
            + geometry.travel2 * wheel.travel_rate * wheel.travel_rate
            + 2 * geometry.travel_rack * wheel.travel_rate * rack_rate
            + geometry.rack2 * rack_rate * rack_rate + geometry.rack * rack_acceleration);
        const double hub_mass = p.hub_mass(i);
        mass.noalias() += hub_mass * jac.transpose() * jac;
        rhs.noalias() += jac.transpose() * (hub_mass * (g - bias));

        const Vec3 axle = geometry.orientation.col(1);
        const Vec3 axle_dot = geometry.axle_travel * wheel.travel_rate + geometry.axle_rack * rack_rate;
        t.steering = std::atan2(-axle.x(), axle.y());
        t.orientation = Quat(geometry.orientation);
        Generalized spin_jac = Generalized::Zero();
        spin_jac.segment<3>(3) = axle;
        spin_jac[10 + i] = 1;
        mass.noalias() += p.wheel_inertia * spin_jac * spin_jac.transpose();
        const double absolute_spin = wheel.omega + axle.dot(omega);
        const Vec3 angular_momentum = p.wheel_inertia * absolute_spin * axle;
        const double spin_bias = axle_dot.dot(omega);
        rhs.segment<3>(3) -= omega.cross(angular_momentum)
            + p.wheel_inertia * (absolute_spin * axle_dot + spin_bias * axle);
        rhs[6 + i] += p.wheel_inertia * absolute_spin * geometry.axle_travel.dot(omega);
        rhs[10 + i] -= p.wheel_inertia * spin_bias;

        t.hub = state_.position + rotation * r;
        const Vec3 hub_velocity = rotation * (models_.point_velocity(body_velocity, omega, r) + relative_velocity);
        const auto ground = surface_at(t.hub.x(), t.hub.y(), terrain_);
        const Vec3 world_axle = rotation * axle;
        const Vec3 radial_normal = ground.normal - world_axle * ground.normal.dot(world_axle);
        const double rolling_factor = radial_normal.norm();
        if (rolling_factor < 1e-5) throw std::runtime_error("Wheel plane is parallel to road normal");
        const Vec3 offset = -p.radius * radial_normal / rolling_factor;
        t.contact = t.hub + offset;
        const double penetration = p.radius * rolling_factor - (t.hub.z() - ground.height) * ground.normal.z();
        const Vec3 forward = world_axle.cross(ground.normal).normalized();
        const Vec3 left = ground.normal.cross(forward).normalized();
        const Vec3 upright_omega = world_omega + rotation * axle.cross(axle_dot);
        const Vec3 base_velocity = hub_velocity + upright_omega.cross(offset);
        t.normal_load = models_.normal_force(penetration, -base_velocity.dot(ground.normal));
        const Vec3 spin_velocity = wheel.omega * world_axle.cross(offset);
        const double vx = base_velocity.dot(forward);
        const double tread = models_.tread_speed(wheel.omega) * rolling_factor;
        t.slip_ratio = (tread - vx) / std::max({std::abs(vx), std::abs(tread), 1.0});
        t.slip_angle = std::atan2(-(base_velocity + spin_velocity).dot(left), std::max(std::abs(vx), 1.0));
        const Eigen::Vector2d tangent = models_.tire_force(t.slip_angle, t.slip_ratio, t.normal_load, ground.friction);
        t.force = t.normal_load * ground.normal + tangent.x() * forward + tangent.y() * left;
        rhs.noalias() += jac.transpose() * t.force;
        const Vec3 contact_moment = rotation.transpose() * offset.cross(t.force);
        rhs.segment<3>(3) += contact_moment;
        rhs[6 + i] += axle.cross(geometry.axle_travel).dot(contact_moment);
        const double tire_torque = axle.dot(contact_moment);

        const double drive_torque = models_.wheel_torque(actuators.motor_torque[i], wheel.omega);
        const double rolling_torque = std::clamp(wheel.omega * 0.5,
            -p.rolling_resistance * t.normal_load * p.radius, p.rolling_resistance * t.normal_load * p.radius);
        const double brake_limit = std::clamp(actuators.brake, 0.0, 1.0) * p.max_brake_torque * (i < 2 ? 1.15 : 0.85);
        const double free_torque = drive_torque + tire_torque - rolling_torque;
        const double brake_torque = std::clamp(p.wheel_inertia * wheel.omega / dt + free_torque, -brake_limit, brake_limit);
        rhs[10 + i] += free_torque - brake_torque;
        telemetry_.mechanical_power += drive_torque * wheel.omega;

        const double shock_rate = geometry.shock_travel * wheel.travel_rate + geometry.shock_rack * rack_rate;
        t.shock_length = geometry.shock_length;
        t.spring_force = models_.shock_force(i, geometry.shock_length, shock_rate);
        rhs[6 + i] += t.spring_force * geometry.shock_travel;
        if (wheel.travel > p.bump_travel) rhs[6 + i] -= p.stop_stiffness * (wheel.travel - p.bump_travel);
        if (wheel.travel < -p.droop_travel) rhs[6 + i] -= p.stop_stiffness * (wheel.travel + p.droop_travel);
    }
    const AeroLoads aero = models_.aero(body_velocity.x());
    telemetry_.downforce = -aero.force.z();
    rhs.head<3>() += rotation * aero.force;
    rhs.segment<3>(3) += aero.moment;
    // Unilateral belly contact, separate from the source tire and suspension models.
    for (double x : {-0.55, 0.55}) for (double y : {-0.25, 0.25}) {
        const Vec3 local(x, y, -p.cg_height + 0.055), offset = rotation * local;
        const Vec3 point = state_.position + offset;
        const auto ground = surface_at(point.x(), point.y(), terrain_);
        const double penetration = ground.height - point.z();
        if (penetration <= 0) continue;
        const Vec3 velocity = state_.velocity + world_omega.cross(offset);
        const double normal = std::max(0.0, 300000 * penetration - 1500 * velocity.dot(ground.normal));
        const Vec3 tangent = velocity - ground.normal * velocity.dot(ground.normal);
        const Vec3 force = normal * ground.normal - tangent * std::min(500.0, 0.5 * normal / std::max(tangent.norm(), 0.1));
        rhs.head<3>() += force;
        rhs.segment<3>(3) += rotation.transpose() * offset.cross(force);
    }
    const Eigen::LDLT<Matrix> solver(mass);
    if (solver.info() != Eigen::Success || !solver.isPositive()) throw std::runtime_error("Non-positive vehicle mass matrix");
    const Generalized acceleration = solver.solve(rhs);
    telemetry_.acceleration = acceleration.head<3>();
    telemetry_.angular_acceleration = acceleration.segment<3>(3);
    telemetry_.specific_force = rotation.transpose() * (telemetry_.acceleration - g);
    state_.velocity += acceleration.head<3>() * dt;
    state_.angular_velocity += acceleration.segment<3>(3) * dt;
    state_.position += state_.velocity * dt;
    const Vec3 angle = state_.angular_velocity * dt;
    if (angle.norm() > 1e-14)
        state_.orientation = (state_.orientation * Quat(Eigen::AngleAxisd(angle.norm(), angle.normalized()))).normalized();
    for (int i = 0; i < 4; ++i) {
        auto& w = state_.wheels[i];
        w.travel_rate += acceleration[6 + i] * dt;
        w.travel += w.travel_rate * dt;
        w.omega += acceleration[10 + i] * dt;
        w.phase = std::remainder(w.phase + w.omega * dt, 2 * pi);
    }
    if (!finite()) throw std::runtime_error("Non-finite vehicle state");
}
bool Vehicle::finite() const {
    if (!state_.position.allFinite() || !state_.velocity.allFinite() || !state_.orientation.coeffs().allFinite() ||
        !state_.angular_velocity.allFinite() || !std::isfinite(state_.steering) ||
        !std::isfinite(state_.rack_offset) || !std::isfinite(state_.rack_rate)) return false;
    for (const auto& w : state_.wheels)
        if (!std::isfinite(w.travel) || !std::isfinite(w.travel_rate) || !std::isfinite(w.omega)) return false;
    return true;
}
double Vehicle::energy() const {
    const auto& p = parameters_;
    const auto rotation = state_.orientation.toRotationMatrix();
    const auto& omega = state_.angular_velocity;
    double energy = 0.5 * (p.mass - p.unsprung_mass) * state_.velocity.squaredNorm()
        + (p.mass - p.unsprung_mass) * gravity * state_.position.z() + 0.5 * omega.dot(sprung_inertia_ * omega);
    for (int i = 0; i < 4; ++i) {
        const auto& w = state_.wheels[i];
        const auto geometry = models_.suspension(i, w.travel, state_.rack_offset);
        const Vec3 hub = state_.position + rotation * geometry.hub;
        const Vec3 velocity = state_.velocity + rotation * (omega.cross(geometry.hub)
            + geometry.travel * w.travel_rate + geometry.rack * state_.rack_rate);
        const Vec3 axle = geometry.orientation.col(1);
        const double spin = w.omega + axle.dot(omega);
        energy += 0.5 * p.hub_mass(i) * velocity.squaredNorm() + p.hub_mass(i) * gravity * hub.z();
        energy += 0.5 * p.wheel_inertia * spin * spin + models_.shock_energy(i, geometry.shock_length);
        const double stop = std::max({w.travel - p.bump_travel, -p.droop_travel - w.travel, 0.0});
        energy += 0.5 * p.stop_stiffness * stop * stop;
        const auto ground = surface_at(hub.x(), hub.y(), terrain_);
        const double factor = std::sqrt(std::max(0.0, 1 - std::pow((rotation * axle).dot(ground.normal), 2)));
        const double penetration = std::max(0.0, p.radius * factor - (hub.z() - ground.height) * ground.normal.z());
        energy += 0.5 * p.tire_stiffness * penetration * penetration;
    }
    return energy;
}
}
