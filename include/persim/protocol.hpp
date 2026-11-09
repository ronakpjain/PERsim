/**
 * @file protocol.hpp
 * @brief SuperDBC-driven signal codec and portable classic-CAN TCP framing.
 * @ingroup can
 */
#pragma once
#include <array>
#include <cstdint>
#include <span>
#include <string_view>
namespace persim {
/** @brief Actual PER bus IDs; CCAN (2) is intentionally not modeled. */
enum class CanBus : std::uint8_t {
    vehicle = 0, ///< VCAN, driver/control network.
    motor = 1,   ///< MCAN, AMK inverter network.
    sensor = 3   ///< SCAN, driveline sensors and diagnostic telemetry.
};
/** @brief Standard 11-bit classic-CAN data frame; no RTR, error, or FD frames. */
struct CanFrame {
    CanBus bus = CanBus::vehicle;        ///< Logical PER network.
    std::uint32_t id = 0;                ///< Standard arbitration ID, at most 0x7ff.
    std::uint8_t length = 0;             ///< Payload length in bytes, 0..8.
    std::array<std::uint8_t, 8> data{};   ///< Payload; bytes beyond length are padding.
};
/** @brief Generated physical-to-raw signal definition from authoritative SuperDBC. */
struct Signal {
    std::string_view name; ///< Signal name, preserving SuperDBC spelling.
    int start;            ///< DBC start bit; Motorola uses sawtooth numbering.
    int width;            ///< Raw field width in bits; generated selections are at most 32.
    bool big_endian;      ///< Motorola MSB-first field if true; Intel otherwise.
    bool is_signed;       ///< Two's-complement raw field if true.
    double scale;         ///< Physical units per raw count.
    double offset;        ///< Physical additive offset.
};
/** @brief Generated, immutable standard-CAN message schema. */
struct Message {
    std::string_view name; ///< Message name in SuperDBC.
    std::string_view bus;  ///< VCAN, MCAN, or SCAN.
    std::uint32_t id;      ///< Standard arbitration ID.
    std::uint8_t length;  ///< Payload length in bytes.
    int period_ms;        ///< Nominal period in ms, zero for event-driven messages.
    std::span<const Signal> signals; ///< Fields with static lifetime.
};
/** @brief Create a zero-payload frame with a schema's bus, ID, and length. */
CanFrame make_frame(const Message& message);
/** @brief Encode a physical signal, rounded and saturated to its raw representation.
 * @throws std::invalid_argument For unknown signals, mismatched frames, or NaN/Inf.
 */
void set_signal(const Message& message, CanFrame& frame, std::string_view name, double value);
/** @brief Decode a signed or unsigned signal into physical units.
 * @throws std::invalid_argument For an unknown signal or mismatched frame.
 */
double get_signal(const Message& message, const CanFrame& frame, std::string_view name);
/** @brief Match bus, identifier, and exact payload length. */
bool matches(const Message& message, const CanFrame& frame);
/** @brief Version-1 PCAN record length, independent of host struct layout. */
inline constexpr std::size_t wire_frame_size = 20;
/** @brief Serialize one record; validate standard ID, bus, and classic payload length.
 * @see cm5_bridge
 */
std::array<std::uint8_t, wire_frame_size> encode_wire(const CanFrame& frame);
/** @brief Parse a complete record; reject unknown magic, version, flags, bus, or size.
 * @note TCP clients must accumulate partial records before calling this function.
 */
CanFrame decode_wire(std::span<const std::uint8_t, wire_frame_size> bytes);
}
