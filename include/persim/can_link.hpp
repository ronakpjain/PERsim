/**
 * @file can_link.hpp
 * @brief Localhost nonblocking TCP endpoint for a virtualized SocketCAN peer.
 * @ingroup can
 */
#pragma once
#include "persim/protocol.hpp"
#include <cstdint>
#include <string>
#include <vector>
namespace persim {
/**
 * @brief Single-owner, single-peer classic-CAN gateway with bounded buffering.
 * @details No background thread mutates FIL. Unknown protocol records or a full
 * output buffer disconnect the peer and expose a diagnostic, rather than
 * silently dropping queued traffic. Absent peers do not accumulate history.
 * See docs/cm5.md.
 */
class CanLink {
public:
    /** @brief Listen on IPv4 loopback; throw if socket/bind/listen setup fails. */
    explicit CanLink(std::uint16_t port);
    /** @brief Close the peer and listener without waiting for network traffic. */
    ~CanLink();
    CanLink(const CanLink&) = delete; ///< Socket ownership cannot be copied.
    CanLink& operator=(const CanLink&) = delete; ///< Socket ownership cannot be copied.
    /** @brief Accept at most one peer, flush output, and parse bounded incoming batches.
     * @return Complete records received in stream order; partial records are retained.
     */
    std::vector<CanFrame> poll();
    /** @brief Queue a validated frame for a connected peer; no-op when disconnected. */
    void send(const CanFrame& frame);
    /** @brief Return TCP connection state, not TV application health. */
    bool connected() const { return peer_ >= 0; }
    /** @brief Return the latest connection/protocol/backpressure diagnostic. */
    const std::string& status() const { return status_; }
private:
    void disconnect(const std::string& reason);
    void flush();
    int listener_ = -1;
    int peer_ = -1;
    std::vector<std::uint8_t> rx_;
    std::vector<std::uint8_t> tx_;
    std::size_t sent_ = 0;
    std::string status_ = "waiting for CM5";
};
}
