/**
 * @file powertrain.hpp
 * @brief Closed-loop AMK inverter devices, motor response, and battery state.
 * @ingroup powertrain
 */
#pragma once
#include "persim/protocol.hpp"
#include "persim/vehicle.hpp"
#include <array>
#include <cstdint>
#include <vector>
namespace persim {
/** @brief State of one simulated inverter, independent of the firmware's AMK FSM. */
struct InverterState {
    bool dc_on = false;        ///< DC-on acknowledgment after the modeled 20 ms delay.
    bool enabled = false;      ///< Inverter ready and controller enable asserted.
    bool fault = false;        ///< Latched modeled thermal fault.
    bool command_seen = false; ///< Whether any valid SET frame has arrived.
    std::uint64_t command_time = 0; ///< Latest SET delivery timestamp in simulated ns.
    double torque = 0.0;       ///< Delivered motor-shaft torque, Nm, after lag/limits.
    double motor_temperature = 30.0; ///< Lumped motor temperature, °C.
    double inverter_temperature = 30.0; ///< Lumped inverter/IGBT temperature, °C.
    double dc_voltage = 0.0;   ///< Modeled inverter DC-link voltage, V.
    double dc_ready_time = 0.0;///< Time continuously requesting a healthy DC link, s.
    double enable_time = 0.0;  ///< Time continuously requesting inverter-on after DC-on, s.
    CanFrame command;          ///< Latest validated AMK SET frame.
};
/** @brief Single equivalent pack, not a cell-level electrochemical model. */
struct BatteryState {
    double voltage = 460.0;       ///< Loaded terminal voltage, V.
    double hv_bus_voltage = 0.0;   ///< Equivalent precharged HV bus, V; 60 ms RC proxy.
    double current = 0.0;         ///< Pack current, A; negative during net regeneration.
    double state_of_charge = 0.8; ///< Remaining energy fraction [0,1].
    double temperature = 30.0;    ///< Lumped pack temperature, °C.
    double power = 0.0;           ///< DC electrical power drawn by the drive, W.
};
/**
 * @brief Four AMK responders with enable handshakes, watchdogs, and torque dynamics.
 * @details Receives firmware-produced SET frames at their CAN delivery times.
 * Outputs actual speed/torque, status acknowledgments, DC voltage, and temperatures.
 * No driver-to-torque shortcut exists in this model.
 * See docs/powertrain.md.
 */
class Powertrain {
public:
    /** @brief Reset all inverter devices and the pack to their initial state. */
    explicit Powertrain(Parameters parameters = {});
    /** @brief Accept a matching SET frame and update that inverter's watchdog.
     * @param frame Delivered MCAN frame.
     * @param time_ns Delivery time, not CAN queue admission time.
     * @return True for an accepted AMK SET frame, false for unrelated traffic.
     */
    bool receive(const CanFrame& frame, std::uint64_t time_ns);
    /** @brief Advance electrical/thermal state and torque response.
     * @param car Current physical wheel spin state.
     * @param now_ns Beginning of this plant substep in simulated ns.
     * @param dt Positive substep duration, s; Session supplies at most 0.5 ms.
     */
    void step(const VehicleState& car, std::uint64_t now_ns, double dt);
    /** @brief Construct CRIT frames and optionally INFO/TEMPS device reports.
     * @param car Physical wheel state, used for rotor speed.
     * @param slow Include status and temperature messages when true.
     */
    std::vector<CanFrame> feedback(const VehicleState& car, bool slow) const;
    /** @brief Return actual motor-shaft torques in FR/FL/RL/RR order, Nm. */
    std::array<double, 4> motor_torques() const;
    /** @brief Return all four inverter states in FR/FL/RL/RR order. */
    const std::array<InverterState, 4>& inverters() const { return inverters_; }
    /** @brief Return the current equivalent-pack state. */
    const BatteryState& battery() const { return battery_; }
    /** @brief Count valid firmware/device SET deliveries, including zero-torque commands. */
    std::uint64_t command_count() const { return command_count_; }
private:
    Parameters parameters_;
    ModelComponents models_;
    std::array<InverterState, 4> inverters_{};
    BatteryState battery_;
    std::uint64_t command_count_ = 0;
};
}
