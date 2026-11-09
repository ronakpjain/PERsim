/** @file protocol.cpp
 * @brief Intel/Motorola physical signal codec and validated PCAN framing.
 * @ingroup can
 */
#include "persim/protocol.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace persim {
namespace {
/** @brief Find an exact signal name or reject an unknown schema field. */
const Signal& signal_named(const Message& m, std::string_view name) {
    for (const auto& s : m.signals) if (s.name == name) return s;
    throw std::invalid_argument("Unknown signal " + std::string(m.name) + "/" + std::string(name));
}
/** @brief Reject unsupported classic-CAN identifiers, payloads, and bus IDs. */
void validate(const CanFrame& f) {
    if (f.id > 0x7ff || f.length > 8 || (f.bus != CanBus::vehicle && f.bus != CanBus::motor && f.bus != CanBus::sensor))
        throw std::invalid_argument("Only standard classic CAN on buses 0/1/3 is supported");
}
/** @brief Advance a DBC Intel or Motorola sawtooth bit index. */
int next_bit(int bit, bool big) { return big ? (bit % 8 == 0 ? bit + 15 : bit - 1) : bit + 1; }
}
CanFrame make_frame(const Message& m) {
    const auto bus = m.bus == "MCAN" ? CanBus::motor : m.bus == "SCAN" ? CanBus::sensor : CanBus::vehicle;
    return {bus, m.id, m.length, {}};
}
bool matches(const Message& m, const CanFrame& f) {
    return f.bus == make_frame(m).bus && f.id == m.id && f.length == m.length;
}
void set_signal(const Message& m, CanFrame& f, std::string_view name, double value) {
    if (!matches(m, f) || !std::isfinite(value)) throw std::invalid_argument("Invalid CAN signal/frame");
    const auto& s = signal_named(m, name);
    const double low = s.is_signed ? -std::ldexp(1.0, s.width - 1) : 0.0;
    const double high = std::ldexp(1.0, s.width - (s.is_signed ? 1 : 0)) - 1;
    const auto raw = static_cast<std::uint64_t>(static_cast<std::int64_t>(
        std::round(std::clamp((value - s.offset) / s.scale, low, high))));
    int bit = s.start;
    for (int n = 0; n < s.width; ++n, bit = next_bit(bit, s.big_endian)) {
        const int source = s.big_endian ? s.width - n - 1 : n;
        const auto mask = static_cast<std::uint8_t>(1u << (bit % 8));
        f.data.at(static_cast<std::size_t>(bit / 8)) &= static_cast<std::uint8_t>(~mask);
        if ((raw >> source) & 1u) f.data.at(static_cast<std::size_t>(bit / 8)) |= mask;
    }
}
double get_signal(const Message& m, const CanFrame& f, std::string_view name) {
    if (!matches(m, f)) throw std::invalid_argument("CAN frame does not match schema");
    const auto& s = signal_named(m, name);
    std::uint64_t raw = 0;
    int bit = s.start;
    for (int n = 0; n < s.width; ++n, bit = next_bit(bit, s.big_endian)) {
        const int destination = s.big_endian ? s.width - n - 1 : n;
        raw |= static_cast<std::uint64_t>((f.data.at(static_cast<std::size_t>(bit / 8)) >> (bit % 8)) & 1u) << destination;
    }
    const auto signed_raw = s.is_signed && (raw & (1ull << (s.width - 1)))
        ? static_cast<std::int64_t>(raw) - static_cast<std::int64_t>(1ull << s.width)
        : static_cast<std::int64_t>(raw);
    return static_cast<double>(signed_raw) * s.scale + s.offset;
}
std::array<std::uint8_t, wire_frame_size> encode_wire(const CanFrame& f) {
    validate(f);
    std::array<std::uint8_t, wire_frame_size> b{'P', 'C', 'A', 'N', 1, static_cast<std::uint8_t>(f.bus), f.length, 0};
    for (int i = 0; i < 4; ++i) b[8 + i] = static_cast<std::uint8_t>(f.id >> (8 * i));
    std::copy(f.data.begin(), f.data.end(), b.begin() + 12);
    return b;
}
CanFrame decode_wire(std::span<const std::uint8_t, wire_frame_size> b) {
    if (b[0] != 'P' || b[1] != 'C' || b[2] != 'A' || b[3] != 'N' || b[4] != 1 || b[7] != 0)
        throw std::invalid_argument("Invalid PCAN protocol header/version/flags");
    CanFrame f;
    f.bus = static_cast<CanBus>(b[5]);
    f.length = b[6];
    for (int i = 0; i < 4; ++i) f.id |= static_cast<std::uint32_t>(b[8 + i]) << (8 * i);
    std::copy(b.begin() + 12, b.end(), f.data.begin());
    validate(f);
    return f;
}
}
