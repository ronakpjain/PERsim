/** @file model_components.hpp
 * @brief SI-double boundary to the sibling's unit-safe component models.
 * @ingroup physics
 */
#pragma once
#include "persim/parameters.hpp"
#include <Eigen/Geometry>
#include <memory>

namespace persim {
struct VehicleState;
struct VehicleTelemetry;

/** @brief One corner's source suspension spline evaluation in body coordinates. */
struct SuspensionGeometry {
    Eigen::Vector3d hub;          ///< Wheel centre relative to body reference, m.
    Eigen::Matrix3d orientation;  ///< Wheel-local to body rotation, including toe/camber.
    Eigen::Vector3d travel;       ///< Hub derivative with respect to bump travel, m/m.
    Eigen::Vector3d rack;         ///< Hub derivative with respect to rack travel, m/m.
    Eigen::Vector3d travel2;      ///< Second hub derivative with respect to bump, 1/m.
    Eigen::Vector3d travel_rack;   ///< Mixed hub derivative, 1/m.
    Eigen::Vector3d rack2;        ///< Second hub derivative with respect to rack, 1/m.
    Eigen::Vector3d axle_travel;  ///< Axle unit-vector derivative with respect to bump, 1/m.
    Eigen::Vector3d axle_rack;    ///< Axle unit-vector derivative with respect to rack, 1/m.
    double shock_length;         ///< Source linkage shock length, m.
    double shock_travel;         ///< Shock motion ratio dL/dq, m/m.
    double shock_rack;           ///< Shock rack derivative dL/du, m/m.
};
/** @brief Source Aero force and moment about the body reference, N and Nm. */
struct AeroLoads {
    Eigen::Vector3d force;        ///< Force in body coordinates, N.
    Eigen::Vector3d moment;       ///< COP moment in body coordinates, Nm.
};
/** @brief Source mounted IMU output before CAN quantization. */
struct ImuMeasurement {
    Eigen::Vector3d acceleration; ///< Specific force in sensor axes, m/s².
    Eigen::Vector3d gyro;         ///< Angular rate in sensor axes, rad/s.
};
/** @brief Source dual-antenna GPS output before UBX quantization. */
struct GpsMeasurement {
    Eigen::Vector3d position;     ///< Rover position in world coordinates, m.
    Eigen::Vector3d velocity;     ///< Rover velocity in world coordinates, m/s.
    Eigen::Vector3d baseline;     ///< Base-to-rover vector in world coordinates, m.
};
/** @brief Direct source components plus exported SuspensionKinematics splines.
 * @details No copied tire/aero/spring/motor equations. Keeps mp-units out of
 * firmware/rendering interfaces; SI quantities are constructed at this boundary.
 * Sensor sampling owns a deterministic PRNG, independent of firmware execution.
 * See docs/models.md.
 */
class ModelComponents {
public:
    /** @brief Build components from source presets and explicit parameter overrides. */
    explicit ModelComponents(const Parameters& parameters);
    /** @brief Release private unit-safe component instances. */
    ~ModelComponents();
    /** @brief Evaluate source cubic splines; endpoint tangents extend beyond bump limits.
     * @param wheel AMK-order wheel index, FR/FL/RL/RR.
     * @param travel Signed bump from source nominal geometry, m.
     * @param rack Prescribed lateral rack offset within the source domain, m.
     */
    SuspensionGeometry suspension(int wheel, double travel, double rack) const;
    /** @brief Map mean nominal front-wheel steering angle, rad, to rack offset, m. */
    double rack_offset(double steering) const;
    /** @brief Return source coilover force along increasing shock length, N. */
    double shock_force(int wheel, double length, double velocity) const;
    /** @brief Return source spring potential at a shock length, J. */
    double shock_energy(int wheel, double length) const;
    /** @brief Evaluate source unilateral vertical tire force, N. */
    double normal_force(double compression, double compression_velocity) const;
    /** @brief Evaluate the selected source tire with its longitudinal-first friction circle.
     * @return Tire-plane Fx,Fy in N, with the road multiplier applied to source mu.
     */
    Eigen::Vector2d tire_force(double slip_angle, double slip_ratio, double load, double road_grip);
    /** @brief Evaluate source wheel tread speed from relative wheel spin, m/s. */
    double tread_speed(double angular_speed) const;
    /** @brief Evaluate source motor speed envelope, returning available shaft torque, Nm. */
    double shaft_limit(double wheel_speed) const;
    /** @brief Evaluate source gearbox/motor output, with energy-consistent regen correction, Nm. */
    double wheel_torque(double shaft_torque, double wheel_speed) const;
    /** @brief Evaluate the source ThrottleMap for the explicit standalone controller, shaft Nm. */
    double throttle_torque(double pedal) const;
    /** @brief Evaluate source Aero force and COP moment at body-forward speed, m/s. */
    AeroLoads aero(double forward_speed) const;
    /** @brief Evaluate source rigid-body point velocity in body axes, m/s. */
    Eigen::Vector3d point_velocity(const Eigen::Vector3d& velocity, const Eigen::Vector3d& omega,
                                 const Eigen::Vector3d& offset) const;
    /** @brief Sample source IMU; configured noise is zero unless explicitly enabled. */
    ImuMeasurement imu(const VehicleState& state, const VehicleTelemetry& telemetry) const;
    /** @brief Sample source GPS, including its rigid mount/baseline geometry. */
    GpsMeasurement gps(const VehicleState& state) const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
