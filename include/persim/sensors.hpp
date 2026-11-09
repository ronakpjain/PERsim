/**
 * @file sensors.hpp
 * @brief Sibling-model sensor sampling and FIL peripheral/CAN encoding.
 * @ingroup firmware
 */
#pragma once
#include "persim/powertrain.hpp"
#include <array>
#include <cstdint>
#include <vector>
namespace persim {
/** @brief Encode calibrated dashboard pedals as ADC1 channels 8,7,2,9.
 * @details Includes the second APPS inversion and regen/brake harness swap.
 * @return Four saturated 12-bit ADC values in channel order.
 */
std::array<std::uint16_t, 4> pedal_adc(const DriverInput& input);
/** @brief Encode steering, source-model IMU samples, BMS, and battery current.
 * @details LWS angle is positive for physical left steering; the firmware
 * negates it to obtain the VCU's right-positive steering convention.
 * IZZE acceleration reports the negative of source specific force (g-a),
 * matching historical captures: stationary body Z is -1 g after firmware
 * mounting compensation. Gyro polarity and plant/CSV specific force (a-g)
 * are unchanged. See @ref captured_data for the supporting observations.
 * @param car Pose, angular rate, actual steering angle, and IMU specific force.
 * @param powertrain Terminal voltage, current, and equivalent pack temperature.
 * @param imu Include IZZE gyro/accelerometer reports.
 * @param battery Include the slower pack_bms report.
 * @return External-device CAN reports; firmware remains the owner of motor_speeds.
 */
std::vector<CanFrame> sensor_frames(const Vehicle& car, const Powertrain& powertrain,
                                  bool imu, bool battery);
/** @brief Build UBX NAV-PVT + NAV-RELPOSNED packets for the TV's cyclic USART3 DMA.
 * @param car Source GPS sample and mounts; world +X is north, +Y is west in this synthetic georeference.
 * @param time_ns Monotonic simulated time, encoded as GPS iTOW.
 * @return 172 bytes with UBX headers/checksums and a synthetic fixed UTC epoch.
 */
std::vector<std::uint8_t> gps_packet(const Vehicle& car, std::uint64_t time_ns);
/** @brief Encode an AUTOCROSS-mode driver request with 50% gains and regen enabled. */
CanFrame tv_request(bool enabled);
}
