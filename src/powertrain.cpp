/** @file powertrain.cpp
 * @brief AMK device handshake, watchdog, motor response, and pack integration.
 * @ingroup powertrain
 */
#include "persim/powertrain.hpp"
#include "persim/protocol_db.hpp"
#include <algorithm>
#include <cmath>

namespace persim {
namespace {
/** @brief AMK command schemas in FR/FL/RL/RR order. */
constexpr std::array<const Message*, 4> sets{&db::INVA_SET, &db::INVB_SET, &db::INVC_SET, &db::INVD_SET};
/** @brief Actual rotor-speed/torque schemas in AMK order. */
constexpr std::array<const Message*, 4> crits{&db::INVA_CRIT, &db::INVB_CRIT, &db::INVC_CRIT, &db::INVD_CRIT};
/** @brief Device handshake/status schemas in AMK order. */
constexpr std::array<const Message*, 4> infos{&db::INVA_INFO, &db::INVB_INFO, &db::INVC_INFO, &db::INVD_INFO};
/** @brief Temperature feedback schemas in AMK order. */
constexpr std::array<const Message*, 4> temps{&db::INVA_TEMPS, &db::INVB_TEMPS, &db::INVC_TEMPS, &db::INVD_TEMPS};
}
Powertrain::Powertrain(Parameters parameters) : parameters_(std::move(parameters)), models_(parameters_) {}
bool Powertrain::receive(const CanFrame& frame, std::uint64_t time_ns) {
    for (int i = 0; i < 4; ++i) if (matches(*sets[i], frame)) {
        auto& inv = inverters_[i];
        inv.command = frame;
        inv.command_time = time_ns;
        inv.command_seen = true;
        ++command_count_;
        if (get_signal(*sets[i], frame, "AMK_Control_bErrorReset") != 0) inv.fault = false;
        return true;
    }
    return false;
}
void Powertrain::step(const VehicleState& car, std::uint64_t now_ns, double dt) {
    const auto& p = parameters_;
    double power = 0.0;
    const bool pack_healthy = battery_.state_of_charge > 0.001 && battery_.temperature < 65;
    battery_.hv_bus_voltage += ((pack_healthy ? battery_.voltage : 0) - battery_.hv_bus_voltage) * (1 - std::exp(-dt / 0.060));
    for (int i = 0; i < 4; ++i) {
        auto& inv = inverters_[i];
        const bool fresh = inv.command_seen && now_ns >= inv.command_time && now_ns - inv.command_time <= 37'000'000;
        const auto signal = [&](const char* name) { return fresh ? get_signal(*sets[i], inv.command, name) : 0.0; };
        const bool dc_requested = signal("AMK_Control_bDcOn") != 0 && battery_.hv_bus_voltage > 300 && !inv.fault && pack_healthy;
        inv.dc_ready_time = dc_requested ? inv.dc_ready_time + dt : 0.0;
        inv.dc_on = inv.dc_ready_time >= 0.020;
        inv.dc_voltage += ((inv.dc_on ? battery_.hv_bus_voltage : 0) - inv.dc_voltage) * (1 - std::exp(-dt / 0.01));
        const bool enable_requested = inv.dc_on && signal("AMK_Control_bInverterOn") != 0;
        inv.enable_time = enable_requested ? inv.enable_time + dt : 0.0;
        inv.enabled = inv.enable_time >= 0.010 && signal("AMK_Control_bEnable") != 0;
        const double motor_speed = car.wheels[i].omega * p.gear_ratio;
        const double rpm = std::abs(motor_speed) * 60 / (2 * pi);
        const double thermal = std::clamp((125.0 - inv.motor_temperature) / 10.0, 0.0, 1.0);
        const double limit = thermal * models_.shaft_limit(car.wheels[i].omega);
        // Decoded AMK words are nominal-torque percent (raw words use 0.1% scaling).
        // SuperDBC's limit-word unit label is not the inverter's physical unit.
        const double requested = p.rated_motor_torque * signal("AMK_TorqueSetpoint") / 100.0;
        const double positive = std::clamp(p.rated_motor_torque * signal("AMK_PositiveTorqueLimit") / 100.0, 0.0, limit);
        const double negative = std::clamp(p.rated_motor_torque * signal("AMK_NegativeTorqueLimit") / 100.0, -limit, 0.0);
        double target = inv.enabled ? std::clamp(requested, negative, positive) : 0.0;
        const double motor_power_limit = target * motor_speed >= 0 ? 20000.0 : 10000.0;
        if (std::abs(target * motor_speed) > motor_power_limit) target = std::copysign(motor_power_limit / std::abs(motor_speed), target);
        if (target * motor_speed < 0 && (battery_.voltage >= 500 || battery_.state_of_charge >= 1)) target = 0;
        if (!fresh || !inv.enabled) inv.torque = 0;
        else inv.torque += (target - inv.torque) * (1 - std::exp(-dt / p.torque_time_constant));
        const double copper_loss = 0.45 * inv.torque * inv.torque;
        const double iron_loss = 1e-6 * rpm * rpm;
        const double mechanical = inv.torque * motor_speed;
        const double electrical = mechanical >= 0 ? mechanical / 0.95 : mechanical * 0.95;
        power += electrical + copper_loss + iron_loss + (inv.dc_on ? 20.0 : 0.0);
        inv.motor_temperature += (copper_loss + iron_loss - 6 * (inv.motor_temperature - 25)) * dt / 6000;
        inv.inverter_temperature += (std::abs(electrical - mechanical) + 10 - 10 * (inv.inverter_temperature - 25)) * dt / 2500;
        inv.fault = inv.fault || inv.motor_temperature > 135 || inv.inverter_temperature > 125;
    }
    battery_.power = power;
    battery_.current = power / std::max(battery_.voltage, 300.0);
    battery_.state_of_charge = std::clamp(battery_.state_of_charge - power * dt / p.battery_capacity_j, 0.0, 1.0);
    const double ocv = 380.0 + 100.0 * battery_.state_of_charge;
    battery_.voltage = std::max(250.0, ocv - p.battery_resistance * battery_.current);
    battery_.temperature += (battery_.current * battery_.current * p.battery_resistance - 20 * (battery_.temperature - 25)) * dt / 50000;
}
std::vector<CanFrame> Powertrain::feedback(const VehicleState& car, bool slow) const {
    std::vector<CanFrame> frames;
    for (int i = 0; i < 4; ++i) {
        const auto& inv = inverters_[i];
        const auto& crit = *crits[i];
        auto frame = make_frame(crit);
        set_signal(crit, frame, "AMK_ActualSpeed", car.wheels[i].omega * parameters_.gear_ratio * 60 / (2 * pi));
        set_signal(crit, frame, "AMK_ActualTorque", inv.torque / parameters_.rated_motor_torque * 100);
        frames.push_back(frame);
        if (!slow) continue;
        const auto& info = *infos[i];
        frame = make_frame(info);
        set_signal(info, frame, "AMK_Status_bSystemReady", battery_.hv_bus_voltage > 300 && battery_.state_of_charge > 0.001 && !inv.fault);
        set_signal(info, frame, "AMK_Status_bError", inv.fault);
        set_signal(info, frame, "AMK_Status_bQuitDcOn", inv.dc_on);
        set_signal(info, frame, "AMK_Status_bDcOn", inv.dc_on);
        set_signal(info, frame, "AMK_Status_bQuitInverterOn", inv.enabled);
        set_signal(info, frame, "AMK_Status_bInverterOn", inv.enabled);
        set_signal(info, frame, "AMK_DCBusVoltage", inv.dc_voltage);
        frames.push_back(frame);
        const auto& temperature = *temps[i];
        frame = make_frame(temperature);
        for (const auto& s : temperature.signals) {
            if (s.name.find("Motor") != std::string_view::npos) set_signal(temperature, frame, s.name, inv.motor_temperature);
            else if (s.name.find("Temp") != std::string_view::npos) set_signal(temperature, frame, s.name, inv.inverter_temperature);
        }
        frames.push_back(frame);
    }
    return frames;
}
std::array<double, 4> Powertrain::motor_torques() const {
    return {inverters_[0].torque, inverters_[1].torque, inverters_[2].torque, inverters_[3].torque};
}
}
