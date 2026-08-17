#pragma once

#include "a1z/types.hpp"
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace a1z {

/**
 * @brief Linux SocketCAN interface for CAN bus communication.
 *
 * Provides similar functionality to python-can's BusABC but using native
 * Linux SocketCAN APIs for lower latency and better real-time performance.
 */
class CanInterface {
public:
    /**
     * @brief Construct a CAN interface.
     * @param channel CAN channel name (e.g., "can0", "can1")
     * @param bitrate CAN bus bitrate (default 1Mbps)
     * @param name Interface name for logging
     */
    explicit CanInterface(const std::string& channel = "can0",
                          int bitrate = 1000000,
                          const std::string& name = "default");

    ~CanInterface();

    // Delete copy
    CanInterface(const CanInterface&) = delete;
    CanInterface& operator=(const CanInterface&) = delete;

    /**
     * @brief Send a CAN frame.
     * @param frame CAN frame to send
     * @return true if sent successfully
     */
    bool send(const CanFrame& frame);

    /**
     * @brief Receive a CAN frame with timeout.
     * @param timeout_ms Timeout in milliseconds
     * @return Received frame or nullopt on timeout
     */
    std::optional<CanFrame> receive(int timeout_ms = 10);

    /**
     * @brief Try to receive without blocking.
     * @return Received frame or nullopt if no data
     */
    std::optional<CanFrame> try_receive();

    /**
     * @brief Start buffered receive in background thread.
     * @param callback Function called for each received frame
     */
    void start_buffered_reader(std::function<void(const CanFrame&)> callback);

    /**
     * @brief Stop buffered reader.
     */
    void stop_buffered_reader();

    /**
     * @brief Close the CAN interface.
     */
    void close();

    /**
     * @brief Check if interface is open.
     */
    bool is_open() const { return socket_fd_ >= 0; }

    /**
     * @brief Get channel name.
     */
    const std::string& channel() const { return channel_; }

private:
    bool setup_socket();

    std::string channel_;
    int bitrate_;
    std::string name_;
    int socket_fd_ = -1;

    // Buffered reader
    std::atomic<bool> reader_running_{false};
    std::thread reader_thread_;
    std::function<void(const CanFrame&)> reader_callback_;
    mutable std::mutex reader_mutex_;
};

} // namespace a1z
