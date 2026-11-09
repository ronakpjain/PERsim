/** @file session.hpp
 * @brief Causal orchestration of firmware, devices, and substepped physical dynamics.
 * @ingroup firmware
 */
#pragma once
#include "persim/options.hpp"
#include "persim/fil_bridge.hpp"
#include <chrono>
#include <memory>
namespace persim {
/** @brief Self-contained observation suitable for cross-thread publication and CSV. */
struct Snapshot {
    VehicleState car;                  ///< Integrated physical vehicle state.
    VehicleTelemetry telemetry;        ///< Latest physics force evaluation.
    std::array<InverterState, 4> inverters; ///< Device state in FR/FL/RL/RR order.
    BatteryState battery;              ///< Equivalent-pack state.
    FirmwareStatus firmware;           ///< Latest observed firmware/network outputs.
    DriverInput driver;                ///< Effective controls after startup overrides.
    double time = 0;                   ///< Shared physical/FIL time, s.
    double realtime_factor = 0;        ///< Total simulated / elapsed wall seconds.
    double peak_torque = 0;            ///< Peak absolute delivered shaft torque, Nm.
    double peak_speed = 0;             ///< Peak inertial reference speed, m/s.
    std::uint64_t motor_commands = 0;   ///< Valid delivered AMK SET frames.
    bool ever_ready = false;           ///< Whether main_hb has reported READY TO DRIVE.
    bool standalone = false;           ///< Explicit non-firmware controller selection.
    std::string error;                 ///< Frontend-owned fatal diagnostic, empty on success.
};
/**
 * @brief Single-owner deterministic plant/firmware control interval.
 * @details Sensors are sampled at the beginning of each 5 ms slice. Delivered
 * AMK commands split physical integration at their actual CAN delivery times;
 * no future command is applied retroactively.
 * See docs/architecture.md.
 */
class Session {
public:
    /** @brief Create the physical plant and requested backend; never silently fall back. */
    explicit Session(Options options);
    /** @brief Advance one 5 ms control slice, with at most 0.5 ms physics substeps.
     * @throws std::runtime_error On a firmware/device/clock failure.
     */
    void step(DriverInput input);
    /** @brief Copy the current observation; caller must synchronize cross-thread access. */
    Snapshot snapshot() const;
    /** @brief Return this session's immutable runtime configuration. */
    const Options& options() const { return options_; }
    /** @brief Require readiness, AMK traffic, TV output, and scenario-specific physical response.
     * @throws std::runtime_error With a failed validation invariant.
     */
    void verify() const;
private:
    void evolve_until(std::uint64_t end, const DriverInput& input);
    Options options_;
    Vehicle car_;
    Powertrain powertrain_;
#if PERSIM_WITH_FIL
    std::unique_ptr<FilBridge> firmware_;
#endif
    DriverInput driver_;
    FirmwareStatus status_;
    std::uint64_t now_ = 0;
    std::uint64_t next_standalone_command_ = 0;
    bool ever_ready_ = false;
    double peak_torque_ = 0;
    double peak_speed_ = 0;
    std::chrono::steady_clock::time_point started_;
};
/** @brief Map firmware car_state_t values to display text; tolerate unknown values. */
const char* car_state_name(int state);
}
