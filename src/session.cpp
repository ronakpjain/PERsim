/** @file session.cpp
 * @brief Causal actuator delivery and physical/firmware time coordination.
 * @ingroup firmware
 */
#include "persim/session.hpp"
#include "persim/protocol_db.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace persim {
const char* car_state_name(int state) {
    constexpr std::array names{"FATAL", "IDLE", "PRECHARGING", "ENERGIZED", "BUZZING", "READY TO DRIVE"};
    return state >= 0 && state < static_cast<int>(names.size()) ? names[state] : "UNKNOWN";
}
Session::Session(Options options)
    : options_(std::move(options)), car_(options_.parameters, options_.terrain), powertrain_(car_.parameters()),
      started_(std::chrono::steady_clock::now()) {
    if (options_.scenario == "bumps") car_.reset(Vec3(10, -10, 0));
#if PERSIM_WITH_FIL
    if (!options_.standalone)
        firmware_ = std::make_unique<FilBridge>(options_.network, options_.external_tv, options_.tv_port);
#else
    if (!options_.standalone) throw std::runtime_error("FIL support was not built");
#endif
}
void Session::evolve_until(std::uint64_t end, const DriverInput& input) {
    if (end < now_) throw std::runtime_error("Non-monotonic co-simulation event");
    while (now_ < end) {
        const auto step = std::min<std::uint64_t>(500'000, end - now_);
        const double dt = static_cast<double>(step) * 1e-9;
        powertrain_.step(car_.state(), now_, dt);
        Actuators actuators{powertrain_.motor_torques(), input.brake, input.steering};
        car_.step(actuators, dt);
        now_ += step;
        for (const auto torque : actuators.motor_torque) peak_torque_ = std::max(peak_torque_, std::abs(torque));
        peak_speed_ = std::max(peak_speed_, car_.state().velocity.norm());
    }
}
void Session::step(DriverInput input) {
    if (!std::isfinite(input.throttle) || !std::isfinite(input.brake) || !std::isfinite(input.steering))
        throw std::invalid_argument("Driver inputs must be finite");
    input.throttle = std::clamp(input.throttle, 0.0, 1.0);
    input.brake = std::clamp(input.brake, 0.0, 1.0);
    input.steering = std::clamp(input.steering, -1.0, 1.0);
    if (!options_.standalone && status_.car_state != 5) {
        input.throttle = 0;
        if (options_.auto_start) input.brake = std::max(input.brake, 0.3);
    }
    if (!options_.standalone && options_.auto_start && now_ >= 1'500'000'000 && now_ < 1'650'000'000)
        input.start = true;
    driver_ = input;
    std::uint64_t end = now_ + 5'000'000;
    if (options_.standalone) {
        status_.car_state = 5;
        if (now_ >= next_standalone_command_) {
            constexpr std::array<const Message*, 4> messages{&db::INVA_SET, &db::INVB_SET, &db::INVC_SET, &db::INVD_SET};
            for (int i = 0; i < 4; ++i) {
                const auto& m = *messages[i];
                auto f = make_frame(m);
                set_signal(m, f, "AMK_Control_bInverterOn", 1);
                set_signal(m, f, "AMK_Control_bDcOn", 1);
                set_signal(m, f, "AMK_Control_bEnable", 1);
                const double shaft = car_.components().throttle_torque(input.throttle);
                set_signal(m, f, "AMK_TorqueSetpoint", shaft / car_.parameters().rated_motor_torque * 100 * (i < 2 ? 2.0 / 3.0 : 1.0));
                set_signal(m, f, "AMK_PositiveTorqueLimit", 214);
                set_signal(m, f, "AMK_NegativeTorqueLimit", -50);
                powertrain_.receive(f, now_);
            }
            next_standalone_command_ = now_ + 15'000'000;
        }
    } else {
#if PERSIM_WITH_FIL
        const auto commands = firmware_->advance(car_, powertrain_, input, end - now_);
        status_ = firmware_->status();
        end = status_.time_ns;
        for (const auto& event : commands) {
            if (event.time_ns < now_ || event.time_ns > end)
                throw std::runtime_error("Actuator delivery lies outside its co-simulation slice");
            evolve_until(event.time_ns, input);
            powertrain_.receive(event.frame, event.time_ns);
        }
#endif
    }
    evolve_until(end, input);
    ever_ready_ = ever_ready_ || status_.car_state == 5;
}
Snapshot Session::snapshot() const {
    const double seconds = static_cast<double>(now_) * 1e-9;
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
    return {car_.state(), car_.telemetry(), powertrain_.inverters(), powertrain_.battery(),
            status_, driver_, seconds, seconds / std::max(wall, 1e-9),
            peak_torque_, peak_speed_, powertrain_.command_count(), ever_ready_, options_.standalone, {}};
}
void Session::verify() const {
    if (!car_.finite()) throw std::runtime_error("Verification: non-finite vehicle state");
    if (!ever_ready_) throw std::runtime_error("Verification: firmware never reached READY TO DRIVE");
    if (powertrain_.command_count() < 4) throw std::runtime_error("Verification: missing AMK command feedback");
    if (!options_.standalone && !options_.external_tv && status_.tv_requests == 0)
        throw std::runtime_error("Verification: torque-vector firmware never published a request");
    if (options_.external_tv && !status_.external_connected)
        throw std::runtime_error("Verification: external TV was not connected");
    if (options_.scenario != "idle" && options_.scenario != "manual" && (peak_torque_ < 1 || peak_speed_ < 1))
        throw std::runtime_error("Verification: controller commands did not accelerate the physical car");
    if (options_.scenario == "brake" && now_ > 8'000'000'000 && car_.state().velocity.norm() > 1)
        throw std::runtime_error("Verification: mechanical brakes did not stop the car");
}
}
