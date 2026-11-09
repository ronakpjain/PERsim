/**
 * @file fil_bridge.hpp
 * @brief Bounded-time C++ FIL adapter and delivered actuator events.
 * @ingroup firmware
 */
#pragma once
#include "persim/powertrain.hpp"
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
namespace persim {
/** @brief Actuator frame at its simulated CAN EOF/delivery time. */
struct TimedFrame {
    std::uint64_t time_ns; ///< Simulated delivery timestamp, ns.
    CanFrame frame;       ///< Valid standard classic-CAN frame.
};
/** @brief Observed firmware outputs; never inferred from physical vehicle motion. */
struct FirmwareStatus {
    int car_state = 0;          ///< Latest main_hb state; 5 is READY TO DRIVE.
    bool tv_enabled = false;   ///< Latest vcu_settings acknowledgment.
    std::uint64_t transmitted_frames = 0; ///< Frames delivered to the plant observer.
    std::uint64_t tv_requests = 0; ///< Delivered vcu_torque_request count.
    std::uint64_t time_ns = 0;  ///< FIL shared clock after the latest slice.
    bool external_connected = false; ///< Live CM5 TCP connection, not proof of a healthy TV app.
    std::string link_status;    ///< Gateway diagnostic for UI display.
};
/**
 * @brief Own a real firmware network and inject plant-owned peripheral inputs.
 * @details Only the simulation thread may call this class. Scripted stimuli
 * and a_box are rejected to avoid competing sensor/battery publishers.
 * See docs/architecture.md.
 */
class FilBridge {
public:
    /** @brief Load strict-MMIO firmware and attach plant/device CAN nodes.
     * @param network Stimulus-free network with buses 0/1/3 and matching firmware wiring.
     * @param external_tv Remove torque_vector and enable the CM5 TCP gateway.
     * @param port Localhost TCP port, used only in external TV mode.
     * @throws std::runtime_error For missing images, invalid wiring, or bind/load failures.
     */
    FilBridge(const std::filesystem::path& network, bool external_tv, std::uint16_t port);
    /** @brief Detach all callbacks before destroying their world. */
    ~FilBridge();
    FilBridge(const FilBridge&) = delete; ///< World ownership cannot be copied.
    FilBridge& operator=(const FilBridge&) = delete; ///< World ownership cannot be copied.
    /** @brief Sample sensors and advance firmware, returning ordered motor commands.
     * @param car Physical state at the beginning of the control interval.
     * @param powertrain Inverter/battery state at that same time.
     * @param input Effective driver input, including startup overrides.
     * @param duration_ns Positive relative shared-time budget, normally 5,000,000 ns.
     * @return Commands ordered by delivery time; no physical state is advanced here.
     * @throws std::runtime_error If firmware stops or exceeds its bounded time budget.
     */
    std::vector<TimedFrame> advance(const Vehicle& car, const Powertrain& powertrain,
                                    const DriverInput& input, std::uint64_t duration_ns);
    /** @brief Return the latest observed state and shared firmware clock. */
    const FirmwareStatus& status() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
