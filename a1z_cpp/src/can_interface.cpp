#include "a1z/can_interface.hpp"

#include <cstring>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

#ifdef __linux__
// Linux SocketCAN headers
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#else
// macOS stub - CAN not supported
#warning "CAN interface is Linux-only, using stub implementation"
#endif

namespace a1z {

CanInterface::CanInterface(const std::string& channel, int bitrate, const std::string& name)
    : channel_(channel), bitrate_(bitrate), name_(name) {
#ifdef __linux__
    if (!setup_socket()) {
        throw std::runtime_error("Failed to open CAN interface: " + channel);
    }
    std::cout << "[CanInterface] " << name_ << " opened on " << channel_ << std::endl;
#else
    std::cerr << "[CanInterface] " << name_ << " CAN not supported on this platform (Linux only)"
              << std::endl;
#endif
}

CanInterface::~CanInterface() {
    close();
}

#ifdef __linux__
bool CanInterface::setup_socket() {
    // Create socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "[CanInterface] " << name_ << " socket creation failed: "
                  << strerror(errno) << std::endl;
        return false;
    }

    // Get interface index
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, channel_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "[CanInterface] " << name_ << " ioctl SIOCGIFINDEX failed: "
                  << strerror(errno) << std::endl;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Bind socket
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "[CanInterface] " << name_ << " bind failed: "
                  << strerror(errno) << std::endl;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Set receive timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10000;  // 10ms default
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return true;
}
#endif

bool CanInterface::send(const CanFrame& frame) {
#ifdef __linux__
    if (socket_fd_ < 0) return false;

    struct can_frame cf;
    std::memset(&cf, 0, sizeof(cf));
    cf.can_id = frame.id;
    cf.can_dlc = frame.dlc;
    std::memcpy(cf.data, frame.data.data(), frame.dlc);

    if (frame.is_extended) {
        cf.can_id |= CAN_EFF_FLAG;
    }
    if (frame.is_rtr) {
        cf.can_id |= CAN_RTR_FLAG;
    }

    ssize_t nbytes = write(socket_fd_, &cf, sizeof(cf));
    return nbytes == sizeof(cf);
#else
    (void)frame;
    return false;
#endif
}

std::optional<CanFrame> CanInterface::receive(int timeout_ms) {
#ifdef __linux__
    if (socket_fd_ < 0) return std::nullopt;

    // Set timeout
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct can_frame cf;
    ssize_t nbytes = read(socket_fd_, &cf, sizeof(cf));

    if (nbytes != sizeof(cf)) {
        return std::nullopt;
    }

    CanFrame frame;
    frame.id = cf.can_id & CAN_EFF_MASK;
    frame.dlc = cf.can_dlc;
    frame.is_extended = (cf.can_id & CAN_EFF_FLAG) != 0;
    frame.is_error = (cf.can_id & CAN_ERR_FLAG) != 0;
    frame.is_rtr = (cf.can_id & CAN_RTR_FLAG) != 0;
    std::memcpy(frame.data.data(), cf.data, cf.can_dlc);

    return frame;
#else
    (void)timeout_ms;
    return std::nullopt;
#endif
}

std::optional<CanFrame> CanInterface::try_receive() {
    return receive(0);
}

void CanInterface::start_buffered_reader(std::function<void(const CanFrame&)> callback) {
    if (reader_running_.exchange(true)) {
        return;  // Already running
    }

    {
        std::lock_guard<std::mutex> lock(reader_mutex_);
        reader_callback_ = std::move(callback);
    }

    reader_thread_ = std::thread([this]() {
        while (reader_running_) {
            auto frame = receive(10);
            if (frame) {
                std::lock_guard<std::mutex> lock(reader_mutex_);
                if (reader_callback_) {
                    reader_callback_(*frame);
                }
            }
        }
    });
}

void CanInterface::stop_buffered_reader() {
    reader_running_ = false;
    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
}

void CanInterface::close() {
    stop_buffered_reader();
#ifdef __linux__
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
#endif
}

} // namespace a1z
