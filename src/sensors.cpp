/** @file sensors.cpp
 * @brief Pedal, steering, mounted IMU, pack, and UBX sensor encoders.
 * @ingroup firmware
 */
#include "persim/sensors.hpp"
#include "persim/protocol_db.hpp"
#include <algorithm>
#include <cmath>

namespace persim {
std::array<std::uint16_t, 4> pedal_adc(const DriverInput& input) {
    const double throttle = std::clamp(input.throttle, 0.0, 1.0);
    const double brake = std::clamp(input.brake, 0.0, 1.0);
    return {static_cast<std::uint16_t>(std::lround(200 + throttle * 510)),
            static_cast<std::uint16_t>(4095 - std::lround(3320 + throttle * 150)),
            static_cast<std::uint16_t>(std::lround(2550 + brake * 450)),
            static_cast<std::uint16_t>(std::lround(450 + brake * 850))};
}
CanFrame tv_request(bool enabled) {
    auto f = make_frame(db::vcu_driver_request);
    set_signal(db::vcu_driver_request, f, "vcu_mode", 2); // AUTOCROSS: closed-loop yaw control.
    set_signal(db::vcu_driver_request, f, "lateral_gain", 50);
    set_signal(db::vcu_driver_request, f, "longitudinal_gain", 50);
    set_signal(db::vcu_driver_request, f, "electronic_brake_bias", 50);
    set_signal(db::vcu_driver_request, f, "is_regen_enabled", 1);
    set_signal(db::vcu_driver_request, f, "is_tv_enabled", enabled);
    return f;
}
std::vector<CanFrame> sensor_frames(const Vehicle& car, const Powertrain& powertrain,
                                  bool imu, bool battery) {
    std::vector<CanFrame> frames;
    auto frame = make_frame(db::LWS_Standard);
    // control_loop.c negates LWS; vcu_interface.md requires right-positive ST_RAW.
    set_signal(db::LWS_Standard, frame, "LWS_ANGLE",
               car.state().steering * 180 / pi * car.parameters().steering_sensor_ratio);
    set_signal(db::LWS_Standard, frame, "OK", 1);
    set_signal(db::LWS_Standard, frame, "CAL", 1);
    frames.push_back(frame);
    if (imu) {
        const auto measurement = car.components().imu(car.state(), car.telemetry());
        const Vec3 gyro = measurement.gyro * (180 / pi);
        // Source IMU returns a-g. Historical IZZE captures report g-a:
        // stationary body Z is -1 g after firmware's mounting transform.
        // Adapt only the device wire convention; plant/CSV truth remains a-g.
        const Vec3 acceleration = -measurement.acceleration / gravity;
        frame = make_frame(db::IZZE_angular_rate);
        for (int i = 0; i < 3; ++i)
            set_signal(db::IZZE_angular_rate, frame, std::array{"X_axis", "Y_axis", "Z_axis"}[i], gyro[i]);
        frames.push_back(frame);
        frame = make_frame(db::IZZE_acceleration);
        for (int i = 0; i < 3; ++i)
            set_signal(db::IZZE_acceleration, frame, std::array{"X_axis", "Y_axis", "Z_axis"}[i], acceleration[i]);
        set_signal(db::IZZE_acceleration, frame, "temperature", 30);
        frames.push_back(frame);
    }
    if (battery) {
        const auto& b = powertrain.battery();
        frame = make_frame(db::pack_bms);
        set_signal(db::pack_bms, frame, "pack_voltage", b.voltage);
        set_signal(db::pack_bms, frame, "min_cell_voltage", b.voltage / 120);
        set_signal(db::pack_bms, frame, "max_cell_voltage", b.voltage / 120);
        set_signal(db::pack_bms, frame, "max_temp", b.temperature);
        set_signal(db::pack_bms, frame, "avg_temp", b.temperature);
        frames.push_back(frame);
    }
    frame = make_frame(db::pack_analog);
    set_signal(db::pack_analog, frame, "pack_voltage", powertrain.battery().voltage);
    set_signal(db::pack_analog, frame, "pack_current", powertrain.battery().current);
    frames.push_back(frame);
    return frames;
}
namespace {
/** @brief Store the low 32 bits of a signed or unsigned value as little-endian UBX data. */
void put32(std::vector<std::uint8_t>& b, int at, std::int64_t value) {
    for (int n = 0; n < 4; ++n) b.at(at + n) = static_cast<std::uint8_t>(static_cast<std::uint64_t>(value) >> (8 * n));
}
/** @brief Wrap a NAV-class payload with a UBX header and Fletcher checksum. */
std::vector<std::uint8_t> ubx(std::uint8_t id, std::vector<std::uint8_t> payload) {
    std::vector<std::uint8_t> b{0xb5, 0x62, 0x01, id, static_cast<std::uint8_t>(payload.size()), 0};
    b.insert(b.end(), payload.begin(), payload.end());
    std::uint8_t a = 0, c = 0;
    for (std::size_t i = 2; i < b.size(); ++i) { a += b[i]; c += a; }
    b.push_back(a); b.push_back(c);
    return b;
}
}
std::vector<std::uint8_t> gps_packet(const Vehicle& car, std::uint64_t time_ns) {
    const auto measurement = car.components().gps(car.state());
    const auto& position = measurement.position;
    const auto& velocity = measurement.velocity;
    std::vector<std::uint8_t> pvt(92);
    put32(pvt, 0, static_cast<std::int64_t>(time_ns / 1'000'000));
    pvt[4] = 0xea; pvt[5] = 7; // synthetic 2026-01-01 epoch
    pvt[6] = 1; pvt[7] = 1; pvt[11] = 7;
    pvt[20] = 3; pvt[21] = 1; pvt[23] = 20;
    put32(pvt, 24, std::llround((-86.9 - position.y() / 84700.0) * 1e7));
    put32(pvt, 28, std::llround((40.4 + position.x() / 111111.0) * 1e7));
    put32(pvt, 32, std::llround(position.z() * 1000));
    put32(pvt, 36, std::llround(position.z() * 1000));
    put32(pvt, 40, 100); put32(pvt, 44, 150);
    put32(pvt, 48, std::llround(velocity.x() * 1000));
    put32(pvt, 52, std::llround(-velocity.y() * 1000));
    put32(pvt, 56, std::llround(-velocity.z() * 1000));
    put32(pvt, 60, std::llround(velocity.head<2>().norm() * 1000));
    put32(pvt, 64, std::llround(std::atan2(-velocity.y(), velocity.x()) * 180 / pi * 1e5));
    auto bytes = ubx(0x07, pvt);
    std::vector<std::uint8_t> relative(64);
    relative[0] = 1;
    put32(relative, 4, static_cast<std::int64_t>(time_ns / 1'000'000));
    const Vec3& baseline = measurement.baseline;
    put32(relative, 8, std::llround(baseline.x() * 100));
    put32(relative, 12, std::llround(-baseline.y() * 100));
    put32(relative, 16, std::llround(-baseline.z() * 100));
    put32(relative, 20, std::llround(baseline.norm() * 100));
    put32(relative, 24, std::llround(std::atan2(-baseline.y(), baseline.x()) * 180 / pi * 1e5));
    put32(relative, 60, 0x137);
    const auto rel = ubx(0x3c, relative);
    bytes.insert(bytes.end(), rel.begin(), rel.end());
    return bytes;
}
}
