/**
 * @file vehicle.hpp
 * @brief SI-valued multibody state, contact telemetry, and the 14-DOF plant.
 * @ingroup physics
 */
#pragma once
#include "persim/model_components.hpp"
#include <Eigen/Geometry>
#include <array>

/** @brief PERsim models and orchestration; all dynamics use ISO 8855 Z-up. */
namespace persim {
/** @brief Double-precision vector; its physical unit is specified at each use. */
using Vec3 = Eigen::Vector3d;
/** @brief Unit quaternion rotating body-frame vectors into world coordinates. */
using Quat = Eigen::Quaterniond;

/** @brief Select a flat surface or the deterministic bump/low-grip test pad. */
enum class TerrainKind {
    flat,           ///< Zero-elevation, unit-grip road.
    proving_ground  ///< Analytic bump, ramp, ripple, and reduced-grip regions.
};
/** @brief Local tangent plane and friction multiplier for a road sample. */
struct Surface {
    double height;    ///< World Z at the sampled X,Y, in m.
    Vec3 normal;      ///< Upward unit normal in world coordinates.
    double friction;  ///< Dimensionless multiplier on Parameters::friction.
};
/** @brief Sample the same road surface used by physics and rendering.
 * @param x World X coordinate, m.
 * @param y World Y coordinate, m.
 * @param kind Selected terrain profile.
 * @return Elevation, normal, and grip; the road has no lateral boundaries.
 */
Surface surface_at(double x, double y, TerrainKind kind);

/** @brief Driver controls, applied through firmware sensors and mechanical actuators. */
struct DriverInput {
    double throttle = 0.0;  ///< Pedal travel, normalized [0,1].
    double brake = 0.0;     ///< Pedal travel [0,1]; physical brakes do not require firmware.
    double steering = 0.0;  ///< Normalized [-1,1], positive left.
    bool start = false;    ///< Dashboard start button pressed, active-low GPIO.
    bool tv_enabled = true;///< Requested TV setting, not a claim about the firmware's state.
};
/** @brief Physical actuator values after the inverter and controller boundaries. */
struct Actuators {
    std::array<double, 4> motor_torque{}; ///< Motor-shaft Nm, FR/FL/RL/RR.
    double brake = 0.0;                  ///< Mechanical brake demand [0,1].
    double steering = 0.0;               ///< Steering actuator demand [-1,1].
};
/** @brief One source-linkage bump coordinate and relative wheel spin. */
struct WheelState {
    double travel = 0.0;      ///< Source signed bump travel from nominal hardpoint geometry, m.
    double travel_rate = 0.0; ///< Suspension coordinate velocity, m/s.
    double omega = 0.0;       ///< Spin relative to the upright, rad/s; positive drives forward.
    double phase = 0.0;       ///< Wrapped spin angle, rad, for display only.
};
/** @brief Integrated rigid-body and wheel state; see @ref vehicle_model. */
struct VehicleState {
    Vec3 position{0.0, 0.0, 0.271};       ///< Sprung-body reference position in world, m.
    Vec3 velocity = Vec3::Zero();         ///< Reference velocity in world, m/s.
    Quat orientation = Quat::Identity();  ///< Body-to-world unit quaternion.
    Vec3 angular_velocity = Vec3::Zero(); ///< Angular velocity expressed in body, rad/s.
    std::array<WheelState, 4> wheels{};    ///< FR/FL/RL/RR suspension and spin states.
    double steering = 0.0;               ///< Prescribed mean nominal front-wheel steering angle, rad.
    double rack_offset = 0.0;             ///< Source lateral steering rack position, m.
    double rack_rate = 0.0;               ///< Rack velocity over the previous substep, m/s.
};
/** @brief Quantities evaluated at the beginning of the latest physics substep. */
struct WheelTelemetry {
    Vec3 hub = Vec3::Zero();     ///< Hub position in world, m.
    Vec3 contact = Vec3::Zero(); ///< Effective tire contact position in world, m.
    Vec3 force = Vec3::Zero();   ///< Total ground force on the wheel in world, N.
    double normal_load = 0.0;    ///< Unilateral tire normal force, N.
    double slip_ratio = 0.0;     ///< Regularized dimensionless longitudinal slip.
    double slip_angle = 0.0;     ///< Signed lateral tire slip angle, rad.
    double steering = 0.0;       ///< Source linkage toe/steering angle in body XY, rad.
    double spring_force = 0.0;   ///< Source shock axial force along increasing length, N.
    double shock_length = 0.0;   ///< Source linkage shock length, m.
    Quat orientation = Quat::Identity(); ///< Wheel-local to body rotation, including camber/toe.
};
/** @brief Latest force evaluation, including physically generated IMU truth. */
struct VehicleTelemetry {
    Vec3 acceleration = Vec3::Zero();   ///< Reference inertial acceleration in world, m/s².
    Vec3 angular_acceleration = Vec3::Zero(); ///< Angular acceleration in body coordinates, rad/s².
    Vec3 specific_force = Vec3::Zero(); ///< Body accelerometer truth Rᵀ(a-g), m/s².
    std::array<WheelTelemetry, 4> wheels{}; ///< Contact and suspension evaluation.
    double downforce = 0.0;             ///< Positive aerodynamic downforce magnitude, N.
    double mechanical_power = 0.0;      ///< Sum of delivered wheel torque × relative spin, W.
};
/**
 * @brief Dynamic 6-DOF chassis, four suspension coordinates, and four wheel spins.
 * @details A coupled 14×14 mass matrix uses source suspension wheel paths and
 * axial spin inertia. Linkage Jacobians/Hessians and shock motion ratios are
 * evaluated every substep. Tire/aero/spring/motor forces use sibling components.
 * @see vehicle_model
 */
class Vehicle {
public:
    /** @brief Construct a statically preloaded plant; reject invalid mass/inertia. */
    explicit Vehicle(Parameters parameters = {}, TerrainKind terrain = TerrainKind::proving_ground);
    /** @brief Relax the source spring/contact system to a stationary initial ride state.
     * @param ground_position World X,Y and an optional additional vertical offset, m.
     * @param heading Initial yaw about world +Z, rad.
     */
    void reset(const Vec3& ground_position = Vec3::Zero(), double heading = 0.0);
    /** @brief Integrate one semi-implicit substep.
     * @param actuators Motor-shaft torques, brake, and steering demand.
     * @param dt Positive duration, at most 0.0005 s.
     * @throws std::invalid_argument If dt violates the stability contract.
     * @throws std::runtime_error If the mass matrix or resulting state is invalid.
     */
    void step(const Actuators& actuators, double dt);
    /** @brief Return the immutable parameter profile. */
    const Parameters& parameters() const { return parameters_; }
    /** @brief Return the integrated state at the end of the latest substep. */
    const VehicleState& state() const { return state_; }
    /** @brief Return the latest, beginning-of-substep force evaluation. */
    const VehicleTelemetry& telemetry() const { return telemetry_; }
    /** @brief Mutable setup hook for headless initial-condition experiments.
     * @warning Mutate only before stepping, on the simulation-owner thread.
     */
    VehicleState& initial_state() { return state_; }
    /** @brief Return the selected road profile. */
    TerrainKind terrain() const { return terrain_; }
    /** @brief Access source sensor/drivetrain adapters without advancing the plant. */
    const ModelComponents& components() const { return models_; }
    /** @brief Return mechanical kinetic + gravitational + spring/tire energy, J.
     * @note Belly-contact potential is excluded; use away from belly contact.
     */
    double energy() const;
    /** @brief Check the integrated pose, velocities, travel, and spin for finiteness. */
    bool finite() const;
private:
    Parameters parameters_;
    ModelComponents models_;
    TerrainKind terrain_;
    VehicleState state_;
    VehicleTelemetry telemetry_;
    Eigen::Matrix3d sprung_inertia_;
};
}
