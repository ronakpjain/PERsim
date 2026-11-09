/** @file can_link.cpp
 * @brief Bounded nonblocking localhost TCP CAN connection.
 * @ingroup can
 */
#include "persim/can_link.hpp"
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>

namespace persim {
namespace {
/** @brief Enable nonblocking I/O or report setup failure. */
void nonblocking(int fd) {
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0)
        throw std::runtime_error("Could not set CAN gateway socket nonblocking");
}
}
CanLink::CanLink(std::uint16_t port) {
    listener_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listener_ < 0) throw std::runtime_error("Could not create CAN gateway socket");
    try {
        const int yes = 1;
        setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0 || listen(listener_, 1) < 0)
            throw std::runtime_error("Could not listen on localhost:" + std::to_string(port) + ": " + std::strerror(errno));
        nonblocking(listener_);
    } catch (...) { close(listener_); throw; }
}
CanLink::~CanLink() {
    if (peer_ >= 0) close(peer_);
    if (listener_ >= 0) close(listener_);
}
void CanLink::disconnect(const std::string& reason) {
    if (peer_ >= 0) close(peer_);
    peer_ = -1;
    rx_.clear(); tx_.clear(); sent_ = 0;
    status_ = reason;
}
void CanLink::flush() {
    while (peer_ >= 0 && sent_ < tx_.size()) {
#ifdef MSG_NOSIGNAL
        constexpr int flags = MSG_NOSIGNAL;
#else
        constexpr int flags = 0;
#endif
        const auto n = ::send(peer_, tx_.data() + sent_, tx_.size() - sent_, flags);
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
        if (n <= 0) { disconnect("CM5 disconnected (write)"); return; }
        sent_ += static_cast<std::size_t>(n);
    }
    if (sent_ == tx_.size()) { tx_.clear(); sent_ = 0; }
}
void CanLink::send(const CanFrame& frame) {
    if (peer_ < 0) return;
    flush();
    if (peer_ < 0) return;
    if (sent_ > 0) {
        tx_.erase(tx_.begin(), tx_.begin() + static_cast<std::ptrdiff_t>(sent_));
        sent_ = 0;
    }
    if (tx_.size() >= 1024 * 1024) { disconnect("CM5 disconnected: CAN backpressure"); return; }
    const auto bytes = encode_wire(frame);
    tx_.insert(tx_.end(), bytes.begin(), bytes.end());
}
std::vector<CanFrame> CanLink::poll() {
    std::vector<CanFrame> frames;
    if (peer_ < 0) {
        peer_ = accept(listener_, nullptr, nullptr);
        if (peer_ < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                throw std::runtime_error("CAN gateway accept failed");
            return frames;
        }
        try {
            nonblocking(peer_);
            const int yes = 1;
            if (setsockopt(peer_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes)) < 0)
                throw std::runtime_error("Could not disable CAN gateway TCP buffering");
#ifdef SO_NOSIGPIPE
            if (setsockopt(peer_, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof(yes)) < 0)
                throw std::runtime_error("Could not protect the CAN gateway from SIGPIPE");
#endif
        } catch (...) { disconnect("CM5 socket setup failed"); throw; }
        status_ = "CM5 connected (wall-clock CAN)";
    }
    flush();
    if (peer_ < 0) return frames;
    std::uint8_t bytes[4096];
    for (int batch = 0; batch < 16; ++batch) {
        const auto n = recv(peer_, bytes, sizeof(bytes), 0);
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
        if (n <= 0) { disconnect(rx_.empty() ? "CM5 disconnected (read)" : "CM5 disconnected: truncated PCAN record"); break; }
        rx_.insert(rx_.end(), bytes, bytes + n);
        while (rx_.size() >= wire_frame_size) {
            try { frames.push_back(decode_wire(std::span<const std::uint8_t, wire_frame_size>(rx_.data(), wire_frame_size))); }
            catch (const std::exception& e) { disconnect(e.what()); return {}; }
            rx_.erase(rx_.begin(), rx_.begin() + wire_frame_size);
        }
    }
    return frames;
}
}
