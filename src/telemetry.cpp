/** @file telemetry.cpp
 * @brief Checked CSV serialization of value snapshots.
 * @ingroup frontend
 */
#include "persim/telemetry.hpp"
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace persim {
TelemetryWriter::TelemetryWriter(const std::filesystem::path& path) {
    if (path.empty()) return;
    stream_.open(path);
    if (!stream_) throw std::runtime_error("Cannot write telemetry: " + path.string());
    stream_ << std::setprecision(12)
        << "time,x,y,z,vx,vy,vz,qw,qx,qy,qz,roll_rate,pitch_rate,yaw_rate,ax,ay,az,throttle,brake,steering,rack_offset,rack_rate,car_state,tv_enabled,can_frames,tv_requests,motor_commands,battery_v,battery_a,soc,downforce";
    for (const auto* name : wheel_names)
        for (const auto* field : {"normal_load", "slip_ratio", "slip_angle", "travel", "omega", "torque", "enabled", "motor_c", "shock_length", "steering", "camber"})
            stream_ << ',' << name << '_' << field;
    stream_ << '\n';
}
void TelemetryWriter::write(const Snapshot& s) {
    if (!stream_.is_open()) return;
    stream_ << s.time;
    for (const auto* v : {&s.car.position, &s.car.velocity})
        for (int i = 0; i < 3; ++i) stream_ << ',' << (*v)[i];
    stream_ << ',' << s.car.orientation.w() << ',' << s.car.orientation.x() << ',' << s.car.orientation.y() << ',' << s.car.orientation.z();
    for (const auto* v : {&s.car.angular_velocity, &s.telemetry.specific_force})
        for (int i = 0; i < 3; ++i) stream_ << ',' << (*v)[i];
    stream_ << ',' << s.driver.throttle << ',' << s.driver.brake << ',' << s.car.steering
        << ',' << s.car.rack_offset << ',' << s.car.rack_rate
        << ',' << s.firmware.car_state << ',' << s.firmware.tv_enabled << ',' << s.firmware.transmitted_frames
        << ',' << s.firmware.tv_requests << ',' << s.motor_commands
        << ',' << s.battery.voltage << ',' << s.battery.current << ',' << s.battery.state_of_charge << ',' << s.telemetry.downforce;
    for (int i = 0; i < 4; ++i)
        stream_ << ',' << s.telemetry.wheels[i].normal_load << ',' << s.telemetry.wheels[i].slip_ratio
            << ',' << s.telemetry.wheels[i].slip_angle << ',' << s.car.wheels[i].travel
            << ',' << s.car.wheels[i].omega << ',' << s.inverters[i].torque
            << ',' << s.inverters[i].enabled << ',' << s.inverters[i].motor_temperature
            << ',' << s.telemetry.wheels[i].shock_length << ',' << s.telemetry.wheels[i].steering
            << ',' << std::asin(std::clamp((s.telemetry.wheels[i].orientation * Vec3::UnitY()).z(), -1.0, 1.0));
    stream_ << '\n';
    if (!stream_) throw std::runtime_error("Telemetry write failed");
}
void TelemetryWriter::finish() {
    if (!stream_.is_open()) return;
    stream_.close();
    if (!stream_) throw std::runtime_error("Telemetry close failed");
}
}
