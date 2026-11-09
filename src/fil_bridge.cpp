/** @file fil_bridge.cpp
 * @brief Strict-MMIO FIL world, peripheral injection, and timed CAN observation.
 * @ingroup firmware
 */
#include "persim/fil_bridge.hpp"
#include "persim/can_link.hpp"
#include "persim/protocol_db.hpp"
#include "persim/sensors.hpp"

#include <fil/common/error.hpp>
#include <fil/config/config.hpp>
#include <fil/devices/can_bus.hpp>
#include <fil/sim/board.hpp>
#include <fil/sim/event_loop.hpp>
#include <fil/sim/world.hpp>
#include <fil/stm32g4/stm32g4.hpp>
#include <fil/stm32g4/peripheral.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace persim {
namespace {
/** @brief Convert a FIL value result into a value or source-aware exception. */
template<class T> T checked(fil::Result<T> result) {
    if (!result) throw std::runtime_error(fil::formatError(result.error()));
    return std::move(result).value();
}
/** @brief Convert a FIL success/error result into a source-aware exception. */
void checked(fil::Result<void> result) {
    if (!result) throw std::runtime_error(fil::formatError(result.error()));
}
/** @brief Convert a validated standard classic frame into FIL's normalized representation. */
fil::devices::CanFrame native(const CanFrame& f) {
    fil::devices::CanFrame result;
    result.id = f.id; result.dlc = f.length;
    std::copy(f.data.begin(), f.data.end(), result.data.begin());
    return result;
}
/** @brief Copy a standard classic FIL frame and its known virtual-bus identity. */
CanFrame frame_from(CanBus bus, const fil::devices::CanFrame& f) {
    CanFrame result{bus, f.id, f.dlc, {}};
    std::copy_n(f.data.begin(), std::min<int>(f.dlc, 8), result.data.begin());
    return result;
}
}
/** @brief FIL-only ownership, CAN attachments, and sensor report deadlines. */
struct FilBridge::Impl {
    std::unique_ptr<fil::sim::World> world; ///< Owns boards, event loop, and timed buses.
    std::unique_ptr<CanLink> link; ///< Optional localhost CM5 connection.
    std::array<fil::devices::VirtualCanBus*, 4> buses{}; ///< Borrowed buses indexed by PER ID; CCAN slot unused.
    std::array<fil::devices::VirtualCanBus::NodeId, 4> plant_nodes{}; ///< Plant transmit/observe attachments.
    std::array<fil::devices::VirtualCanBus::NodeId, 4> external_nodes{}; ///< CM5 transmit/observe attachments.
    std::vector<TimedFrame> commands; ///< Delivered AMK commands collected for causal plant replay.
    FirmwareStatus status; ///< Latest observed controller outputs.
    std::uint64_t next_fast = 0; ///< Next LWS/pack-analog report time, ns.
    std::uint64_t next_crit = 0; ///< Next AMK CRIT report time, ns.
    std::uint64_t next_imu = 0; ///< Next IZZE report time, ns.
    std::uint64_t next_battery = 0; ///< Next BMS summary report time, ns.
    std::uint64_t next_info = 0; ///< Next AMK INFO/TEMPS report time, ns.
    std::uint64_t next_gps = 200'000'000; ///< Next TV UART GPS report time, ns.
    std::uint64_t next_settings = 200'000'000; ///< Next driver-settings request time, ns.
    bool external = false; ///< Exclusive CM5 rather than STM32 TV mode.

    /** @brief Validate publishers, load firmware, and attach device/transport nodes. */
    Impl(const std::filesystem::path& network, bool external_tv, std::uint16_t port) : external(external_tv) {
        auto config = checked(fil::config::loadNetworkConfig(network));
        if (!config.stimulus_paths.empty())
            throw std::runtime_error("PERsim requires a network without scripted stimuli (the plant owns sensor inputs)");
        for (auto it = config.board_paths.begin(); it != config.board_paths.end();) {
            const auto board = checked(fil::config::loadBoardConfig(*it));
            if (board.name == "a_box")
                throw std::runtime_error("Remove a_box from the plant network: PERsim owns the battery messages");
            if (external && board.name == "torque_vector") it = config.board_paths.erase(it);
            else ++it;
        }
        world = checked(fil::sim::World::load(config, true));
        world->setDiagnosticsEnabled(false);
        for (const auto* required : {"dashboard", "main_module"})
            if (!world->board(required)) throw std::runtime_error(std::string("Required firmware board missing: ") + required);
        if (!external && !world->board("torque_vector"))
            throw std::runtime_error("STM32 TV mode requires torque_vector firmware");
        if (external) link = std::make_unique<CanLink>(port);
        for (const int b : {0, 1, 3}) {
            buses[b] = world->canBus(std::to_string(b));
            if (!buses[b] || buses[b]->deliveryMode() != fil::devices::VirtualCanBus::DeliveryMode::timed)
                throw std::runtime_error("Expected timed CAN buses 0 (VCAN), 1 (MCAN), 3 (SCAN)");
            const auto bus = static_cast<CanBus>(b);
            plant_nodes[b] = checked(buses[b]->attach("persim-plant", false,
                [this, bus](const fil::devices::CanFrame& f, std::uint64_t at) {
                    if (f.extended || f.fd || f.dlc > 8) return;
                    const auto frame = frame_from(bus, f);
                    ++status.transmitted_frames;
                    if (matches(db::main_hb, frame)) status.car_state = static_cast<int>(get_signal(db::main_hb, frame, "car_state"));
                    if (matches(db::vcu_settings, frame)) status.tv_enabled = get_signal(db::vcu_settings, frame, "is_tv_enabled") != 0;
                    if (matches(db::vcu_torque_request, frame)) ++status.tv_requests;
                    if (bus == CanBus::motor && frame.id >= db::INVA_SET.id && frame.id <= db::INVD_SET.id)
                        commands.push_back({at, frame});
                }));
            if (link) external_nodes[b] = checked(buses[b]->attach("persim-cm5", false,
                [this, bus](const fil::devices::CanFrame& f, std::uint64_t) {
                    if (!f.extended && !f.fd && f.dlc <= 8) link->send(frame_from(bus, f));
                }));
        }
        gpio("main_module", "GPIOB", 9, true);
        gpio("main_module", "GPIOC", 4, true);
        gpio("dashboard", "GPIOB", 14, true);
    }
    /** @brief Detach callbacks while their target world and gateway are still alive. */
    ~Impl() {
        for (const int b : {0, 1, 3}) if (buses[b]) {
            buses[b]->detach(plant_nodes[b]);
            if (external_nodes[b]) buses[b]->detach(external_nodes[b]);
        }
    }
    /** @brief Drive a required firmware GPIO endpoint between world slices. */
    void gpio(const char* name, const char* port, int pin, bool high) {
        auto* board = world->board(name);
        auto* gpio = board ? board->peripherals().gpio(port) : nullptr;
        if (!gpio) throw std::runtime_error(std::string("GPIO endpoint missing: ") + name + "/" + port);
        gpio->setInput(pin, high);
    }
    /** @brief Update an ADC sample; optional driveline boards may be absent. */
    void adc(const char* name, const char* instance, int channel, std::uint16_t value, bool required = true) {
        auto* board = world->board(name);
        if (!board && !required) return;
        auto* adc = board ? board->peripherals().adc(instance) : nullptr;
        if (!adc) throw std::runtime_error(std::string("ADC endpoint missing: ") + name + "/" + instance);
        adc->setChannelValue(channel, value);
    }
    /** @brief Submit a plant or CM5 frame and report queue-admission errors. */
    void send(const CanFrame& frame, std::uint64_t at, bool from_external = false) {
        const auto b = static_cast<unsigned>(frame.bus);
        checked(buses.at(b)->send(from_external ? external_nodes[b] : plant_nodes[b], native(frame), at));
    }
};
FilBridge::FilBridge(const std::filesystem::path& network, bool external_tv, std::uint16_t port)
    : impl_(std::make_unique<Impl>(network, external_tv, port)) {}
FilBridge::~FilBridge() = default;
const FirmwareStatus& FilBridge::status() const { return impl_->status; }

std::vector<TimedFrame> FilBridge::advance(const Vehicle& car, const Powertrain& powertrain,
                                         const DriverInput& input, std::uint64_t duration_ns) {
    auto& s = *impl_;
    const auto now = s.world->eventLoop().now();
    const auto& battery = powertrain.battery();
    s.gpio("main_module", "GPIOC", 4, battery.hv_bus_voltage < 0.95 * battery.voltage);
    const auto values = pedal_adc(input);
    for (int i = 0; i < 4; ++i) s.adc("dashboard", "ADC1", std::array{8, 7, 2, 9}[i], values[i]);
    s.gpio("dashboard", "GPIOB", 14, !input.start);
    const auto shock = [&](int index) {
        const double length = car.telemetry().wheels[index].shock_length;
        return static_cast<std::uint16_t>(std::lround(std::clamp(1 - (length - 0.145) / 0.075, 0.0, 1.0) * 4095));
    };
    s.adc("front_driveline", "ADC3", 5, shock(FR), false);
    s.adc("front_driveline", "ADC4", 3, shock(FL), false);
    s.adc("rear_driveline", "ADC3", 5, shock(RL), false);
    s.adc("rear_driveline", "ADC4", 3, shock(RR), false);
    if (s.link) {
        for (const auto& frame : s.link->poll()) s.send(frame, now, true);
        s.status.external_connected = s.link->connected();
        s.status.link_status = s.link->status();
    }
    if (now >= s.next_fast) {
        const bool imu = now >= s.next_imu;
        const bool battery = now >= s.next_battery;
        for (const auto& frame : sensor_frames(car, powertrain, imu, battery)) s.send(frame, now);
        s.next_fast = now + 10'000'000;
        if (imu) s.next_imu = now + 50'000'000;
        if (battery) s.next_battery = now + 200'000'000;
    }
    if (now >= s.next_crit) {
        const bool slow = now >= s.next_info;
        for (const auto& frame : powertrain.feedback(car.state(), slow)) s.send(frame, now);
        s.next_crit = now + 15'000'000;
        if (slow) s.next_info = now + 500'000'000;
    }
    if (now >= s.next_settings) {
        s.send(tv_request(input.tv_enabled), now);
        s.next_settings = now + 200'000'000;
    }
    if (!s.external && now >= s.next_gps) {
        auto* usart = s.world->board("torque_vector")->peripherals().usart("USART3");
        if (!usart) throw std::runtime_error("TV GPS USART3 is missing");
        const auto bytes = gps_packet(car, now);
        usart->injectRx(bytes);
        s.next_gps = now + 200'000'000;
    }
    fil::sim::WorldRunOptions options;
    options.duration_ns = duration_ns;
    options.max_instructions_per_board = 50'000'000;
    options.adc_decimation = 1;
    options.enable_ram_capsules = false; // No cycle-exact claim for real-firmware capsule replay.
    const auto result = checked(s.world->run(options));
    if (result.reason != fil::sim::WorldStopReason::time_budget)
        throw std::runtime_error("FIL stopped: " + std::string(fil::sim::worldStopReasonName(result.reason)) + " " + result.message);
    const auto end = s.world->eventLoop().now();
    if (end < now + duration_ns || end > now + duration_ns + 100'000)
        throw std::runtime_error("FIL exceeded the bounded co-simulation time window");
    s.status.time_ns = end;
    auto frames = std::move(s.commands);
    s.commands.clear();
    std::stable_sort(frames.begin(), frames.end(), [](const auto& a, const auto& b) { return a.time_ns < b.time_ns; });
    return frames;
}
}
