/** @file model_components.cpp
 * @brief Direct adapters to the sibling models and exact exported cubic splines.
 * @ingroup physics
 */
#include "persim/model_components.hpp"
#include "persim/tire.hpp"
#include "persim/vehicle.hpp"
#include "persim/suspension_db.hpp"

#include <aero/aero.hpp>
#include <dynamics/wheel.hpp>
#include <powertrain/geared_motor.hpp>
#include <powertrain/throttle_map.hpp>
#include <sensors/gps.hpp>
#include <sensors/imu.hpp>
#include <suspension/coilover.hpp>
#include <tires/vertical_tire.hpp>
#include <vehicle/vehicle.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <stdexcept>

namespace persim {
namespace {
/** @brief Cross-product matrix for a body-frame vector. */
Eigen::Matrix3d cross_matrix(const Vec3& v) {
    Eigen::Matrix3d s;
    s << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return s;
}
/** @brief Four local cubic basis weights and their first two derivatives. */
struct Basis {
    int first;                       ///< First active coefficient index.
    std::array<double, 4> value{};    ///< Basis values.
    std::array<double, 4> slope{};    ///< First derivative per metre.
    std::array<double, 4> curve{};    ///< Second derivative per square metre.
};
/** @brief Cox-de Boor basis, including repeated endpoint knots. */
template<std::size_t N>
Basis basis(const std::array<double, N>& knots, double x) {
    const int count = static_cast<int>(N) - 4;
    const int span = std::clamp(static_cast<int>(std::upper_bound(knots.begin(), knots.end(), x) - knots.begin()) - 1, 3, count - 1);
    std::array<std::array<double, 4>, 4> weights{};
    for (int degree = 0; degree <= 3; ++degree) {
        auto& w = weights[degree];
        w[0] = 1;
        std::array<double, 4> left{}, right{};
        for (int j = 1; j <= degree; ++j) {
            left[j] = x - knots[span + 1 - j];
            right[j] = knots[span + j] - x;
            double saved = 0;
            for (int r = 0; r < j; ++r) {
                const double denominator = right[r + 1] + left[j - r];
                const double temp = denominator > 0 ? w[r] / denominator : 0;
                w[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            w[j] = saved;
        }
    }
    const auto value = [&](int index, int degree) {
        const int local = index - (span - degree);
        return local >= 0 && local <= degree ? weights[degree][local] : 0.0;
    };
    const auto derivative = [&](int index, int degree) {
        const double a = knots[index + degree] - knots[index];
        const double b = knots[index + degree + 1] - knots[index + 1];
        return (a > 0 ? degree * value(index, degree - 1) / a : 0)
             - (b > 0 ? degree * value(index + 1, degree - 1) / b : 0);
    };
    Basis out{span - 3};
    for (int j = 0; j < 4; ++j) {
        const int i = out.first + j;
        out.value[j] = weights[3][j];
        out.slope[j] = derivative(i, 3);
        const double a = knots[i + 3] - knots[i];
        const double b = knots[i + 4] - knots[i + 1];
        out.curve[j] = (a > 0 ? 3 * derivative(i, 2) / a : 0)
                     - (b > 0 ? 3 * derivative(i + 1, 2) / b : 0);
    }
    return out;
}
/** @brief One scalar spline value, gradient, and Hessian in travel/rack coordinates. */
struct SplineValue {
    double f = 0;    ///< Value in the channel's SI unit.
    double q = 0;    ///< Travel derivative.
    double u = 0;    ///< Rack derivative.
    double qq = 0;   ///< Travel Hessian.
    double qu = 0;   ///< Mixed Hessian.
    double uu = 0;   ///< Rack Hessian.
};
/** @brief Evaluate one source tensor-product spline without resampling it. */
SplineValue spline(int wheel, int channel, const Basis& q, const Basis& u) {
    constexpr int nq = model_data::travel_coefficients, nu = model_data::rack_coefficients;
    const std::size_t start = static_cast<std::size_t>(wheel * 7 + channel) * nq * nu;
    SplineValue v;
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) {
        const double c = model_data::coefficients[start + (q.first + i) * nu + u.first + j];
        v.f += c * q.value[i] * u.value[j];
        v.q += c * q.slope[i] * u.value[j];
        v.u += c * q.value[i] * u.slope[j];
        v.qq += c * q.curve[i] * u.value[j];
        v.qu += c * q.slope[i] * u.slope[j];
        v.uu += c * q.value[i] * u.curve[j];
    }
    return v;
}
}
/** @brief Unit-safe source objects, stable sensor mounts, and per-plant PRNG. */
struct ModelComponents::Impl {
    Parameters parameters;       ///< SI adapter configuration.
    Wheel wheel;                 ///< Source wheel surface-speed model.
    GearedMotor motor;           ///< Source AMK envelope and gearbox model.
    SpringDamper front_shock;     ///< Source front shock spring/damper.
    SpringDamper rear_shock;      ///< Source rear shock spring/damper.
    VerticalTire vertical;        ///< Source unilateral contact model.
    std::unique_ptr<TireModel> tire; ///< Selected source tangential model.
    Aero aerodynamic;            ///< Source aero force and COP.
    ThrottleMap throttle;        ///< Explicit standalone pedal map.
    std::mt19937 generator{0};   ///< Deterministic sensor-noise stream.
    RigidMount3D imu_mount;       ///< Actual firmware mounting rotation, at CG.
    RigidMount3D rover_mount;     ///< Synthetic GPS rover at the body reference.
    RigidMount3D base_mount;      ///< Synthetic base one wheelbase behind the rover.
    IMU imu_sensor;              ///< Source mounted accelerometer and gyro.
    GPS gps_sensor;              ///< Source mounted dual-antenna GPS.

    /** @brief Construct stable mount references before source sensor objects. */
    explicit Impl(const Parameters& p)
        : parameters(p), wheel({length(p.radius), inertia(p.wheel_inertia)}),
          motor({moment(p.max_motor_torque), angularSpeed(p.motor_knee_rpm * 2 * pi / 60),
                 AMK_DD5.software_forward_limit, AMK_DD5.software_reverse_limit}, {p.gear_ratio, p.gear_efficiency}),
          front_shock({stiffness(p.spring), damping(p.damper)}),
          rear_shock({REAR_COILOVER.spring_damper.stiffness * (p.spring / siValue(FRONT_COILOVER.spring_damper.stiffness)),
                      REAR_COILOVER.spring_damper.damping * (p.damper / siValue(FRONT_COILOVER.spring_damper.damping))}),
          vertical({{stiffness(p.tire_stiffness), damping(p.tire_damping)}}), tire(make_tire_model(p.tire, p)),
          aerodynamic({area(p.cda), area(p.cla), density(p.air_density), length(p.cop_x)}),
          throttle(motor.maxWheelTorque()),
          imu_mount{displacement3d(0, 0, 0), Quat(Eigen::AngleAxisd(-pi / 2, Vec3::UnitZ()) * Eigen::AngleAxisd(2.2689280276, Vec3::UnitX()))},
          rover_mount{displacement3d(0, 0, 0), Quat::Identity()},
          base_mount{displacement3d(-p.wheelbase, 0, 0), Quat::Identity()},
          imu_sensor(imu_mount, acceleration(p.sensor_noise ? 0.02 * gravity : 0),
                     angularSpeed(p.sensor_noise ? 0.25 * pi / 180 : 0), generator),
          gps_sensor(rover_mount, base_mount, length(p.sensor_noise ? 0.02 : 0),
                     speed(p.sensor_noise ? 0.05 : 0), length(p.sensor_noise ? 0.005 : 0), generator) {}
};
ModelComponents::ModelComponents(const Parameters& parameters) : impl_(std::make_unique<Impl>(parameters)) {}
ModelComponents::~ModelComponents() = default;

SuspensionGeometry ModelComponents::suspension(int wheel, double travel, double rack) const {
    if (wheel < 0 || wheel >= wheel_count || !std::isfinite(travel) || !std::isfinite(rack))
        throw std::invalid_argument("Invalid suspension lookup coordinates");
    if (rack < model_data::rack_knots.front() - 1e-12 || rack > model_data::rack_knots.back() + 1e-12)
        throw std::invalid_argument("Rack displacement exceeds source suspension table");
    const double bounded_q = std::clamp(travel, model_data::travel_knots.front(), model_data::travel_knots.back());
    // SuspensionKinematics.evaluate fixes rear tie-rod offsets to zero.
    const double bounded_u = wheel < 2 ? std::clamp(rack, model_data::rack_knots.front(), model_data::rack_knots.back()) : 0.0;
    const Basis q = basis(model_data::travel_knots, bounded_q), u = basis(model_data::rack_knots, bounded_u);
    std::array<SplineValue, 7> v;
    for (int channel = 0; channel < 7; ++channel) {
        v[channel] = spline(wheel, channel, q, u);
        if (travel != bounded_q) {
            v[channel].f += v[channel].q * (travel - bounded_q);
            v[channel].u += v[channel].qu * (travel - bounded_q);
            v[channel].qq = 0;
        }
    }
    SuspensionGeometry out;
    for (int k = 0; k < 3; ++k) {
        out.hub[k] = v[k].f; out.travel[k] = v[k].q; out.rack[k] = v[k].u;
        out.travel2[k] = v[k].qq; out.travel_rack[k] = v[k].qu; out.rack2[k] = v[k].uu;
    }
    const Vec3 rotation(v[3].f, v[4].f, v[5].f);
    const Vec3 rq(v[3].q, v[4].q, v[5].q), ru(v[3].u, v[4].u, v[5].u);
    const double theta = rotation.norm();
    const Eigen::Matrix3d skew = cross_matrix(rotation);
    Eigen::Matrix3d left_jacobian;
    if (theta < 1e-6) left_jacobian = Eigen::Matrix3d::Identity() + 0.5 * skew + skew * skew / 6;
    else left_jacobian = Eigen::Matrix3d::Identity() + (1 - std::cos(theta)) / (theta * theta) * skew
                        + (theta - std::sin(theta)) / (theta * theta * theta) * skew * skew;
    out.orientation = theta < 1e-14 ? Eigen::Matrix3d::Identity() : Eigen::AngleAxisd(theta, rotation / theta).toRotationMatrix();
    const Vec3 axle = out.orientation.col(1);
    out.axle_travel = (left_jacobian * rq).cross(axle);
    out.axle_rack = (left_jacobian * ru).cross(axle);
    out.shock_length = v[6].f; out.shock_travel = v[6].q; out.shock_rack = v[6].u;
    if (wheel >= 2) {
        out.rack.setZero(); out.travel_rack.setZero(); out.rack2.setZero();
        out.axle_rack.setZero(); out.shock_rack = 0;
    }
    return out;
}

double ModelComponents::rack_offset(double steering) const {
    const auto& angles = model_data::steering_samples;
    const auto& rack = model_data::rack_samples;
    const auto next = std::lower_bound(angles.begin(), angles.end(), steering);
    if (next == angles.begin()) return rack.front();
    if (next == angles.end()) return rack.back();
    const auto i = static_cast<std::size_t>(next - angles.begin());
    return std::lerp(rack[i - 1], rack[i], (steering - angles[i - 1]) / (angles[i] - angles[i - 1]));
}

double ModelComponents::shock_force(int wheel, double value, double rate) const {
    const auto& source = wheel < 2 ? FRONT_COILOVER : REAR_COILOVER;
    const SpringDamperState state{length(value) - source.reference_length, speed(rate)};
    const double minimum = siValue(source.max_length - source.stroke), maximum = siValue(source.max_length);
    const double stop = impl_->parameters.stop_stiffness * (std::max(0.0, minimum - value) - std::max(0.0, value - maximum));
    return siValue(wheel < 2 ? impl_->front_shock.evaluate(state) : impl_->rear_shock.evaluate(state)) + stop;
}
double ModelComponents::shock_energy(int wheel, double value) const {
    const auto& source = wheel < 2 ? FRONT_COILOVER : REAR_COILOVER;
    const double k = wheel < 2 ? impl_->parameters.spring : impl_->parameters.spring
        * siValue(REAR_COILOVER.spring_damper.stiffness) / siValue(FRONT_COILOVER.spring_damper.stiffness);
    const double stop = std::max({siValue(source.max_length - source.stroke) - value, value - siValue(source.max_length), 0.0});
    return 0.5 * k * std::pow(value - siValue(source.reference_length), 2)
         + 0.5 * impl_->parameters.stop_stiffness * stop * stop;
}
double ModelComponents::normal_force(double compression, double rate) const {
    return siValue(impl_->vertical.evaluate({length(compression), speed(rate)}));
}
Eigen::Vector2d ModelComponents::tire_force(double alpha, double kappa, double load, double road_grip) {
    return impl_->tire->force(alpha, kappa, load, impl_->parameters.friction * road_grip);
}
double ModelComponents::tread_speed(double omega) const {
    return siValue(impl_->wheel.surfaceSpeed({angularSpeed(omega)}));
}
double ModelComponents::shaft_limit(double wheel_speed) const {
    const auto maximum = impl_->motor.evaluate(impl_->motor.maxWheelTorque(), angularSpeed(wheel_speed));
    return siValue(impl_->motor.wheelToMotorTorque(maximum));
}
double ModelComponents::wheel_torque(double shaft, double wheel_speed) const {
    const auto& s = *impl_;
    const double result = siValue(s.motor.evaluate(s.motor.motorToWheelTorque(moment(shaft)), angularSpeed(wheel_speed)));
    return shaft * wheel_speed >= 0 ? result : result / std::pow(s.parameters.gear_efficiency, 2);
}
double ModelComponents::throttle_torque(double pedal) const {
    return siValue(impl_->motor.wheelToMotorTorque(impl_->throttle.getTorqueRequest(std::clamp(pedal, 0.0, 1.0))));
}
AeroLoads ModelComponents::aero(double forward_speed) const {
    RigidBodyLoads loads;
    loads.addForce(impl_->aerodynamic.getForce(speed(forward_speed)), impl_->aerodynamic.getCopOffset());
    return {siValue(loads.force_local), siValue(loads.moment_local)};
}
Vec3 ModelComponents::point_velocity(const Vec3& velocity, const Vec3& omega, const Vec3& offset) const {
    return siValue(RigidBodyMotion{vel3d(velocity), angularVel3d(omega)}.velAt(displacement3d(offset)));
}
ImuMeasurement ModelComponents::imu(const VehicleState& state, const VehicleTelemetry& telemetry) const {
    const auto rotation = state.orientation.toRotationMatrix();
    const RigidBodyMotion motion{vel3d(rotation.transpose() * state.velocity), angularVel3d(state.angular_velocity)};
    const RigidBodyAcceleration accel{accel3d(rotation.transpose() * telemetry.acceleration), angularAccel3d(telemetry.angular_acceleration)};
    const auto measurement = impl_->imu_sensor.sample(motion, accel, accel3d(rotation.transpose() * Vec3(0, 0, -gravity)));
    return {siValue(measurement.accelerometer), siValue(measurement.gyro)};
}
GpsMeasurement ModelComponents::gps(const VehicleState& state) const {
    RigidBodyState chassis;
    chassis.pos_world = pos3d(state.position);
    chassis.vel_world = vel3d(state.velocity);
    chassis.setOrientation(state.orientation);
    chassis.angular_vel_world = angularVel3d(state.orientation * state.angular_velocity);
    const auto measurement = impl_->gps_sensor.sample(chassis);
    return {siValue(measurement.position), siValue(measurement.velocity), siValue(measurement.baseline)};
}
}
