#pragma once

#include "a1z/types.hpp"
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace a1z {

/**
 * @brief Transport backend type.
 */
enum class TransportType {
    SocketCAN,  ///< Direct CAN bus (Linux SocketCAN)
    G4Ros       ///< ROS2 topic via g4spi_node (lemo main board)
};

/**
 * @brief Abstract transport interface for CAN frame send/receive.
 *
 * This abstracts the underlying transport (SocketCAN or ROS2 topic) so the
 * motor drivers and control loop can work with either backend.
 */
class Transport {
public:
    virtual ~Transport() = default;

    /**
     * @brief Send a CAN frame.
     * @param frame CAN frame to send
     * @return true if sent successfully
     */
    virtual bool send(const CanFrame& frame) = 0;

    /**
     * @brief Receive a CAN frame with timeout.
     * @param timeout_ms Timeout in milliseconds (0 = non-blocking)
     * @return Received frame or nullopt on timeout
     */
    virtual std::optional<CanFrame> receive(int timeout_ms = 0) = 0;

    /**
     * @brief Start buffered receive in background thread.
     * @param callback Function called for each received frame
     */
    virtual void start_buffered_reader(std::function<void(const CanFrame&)> callback) = 0;

    /**
     * @brief Stop buffered reader.
     */
    virtual void stop_buffered_reader() = 0;

    /**
     * @brief Close the transport.
     */
    virtual void close() = 0;

    /**
     * @brief Check if transport is open.
     */
    virtual bool is_open() const = 0;

    /**
     * @brief Get transport type.
     */
    virtual TransportType type() const = 0;
};

/**
 * @brief Create a SocketCAN transport.
 * @param channel CAN channel name (e.g., "can0")
 * @param bitrate CAN bus bitrate
 * @param name Interface name for logging
 */
std::shared_ptr<Transport> create_socketcan_transport(
    const std::string& channel = "can0",
    int bitrate = 1000000,
    const std::string& name = "default");

/**
 * @brief Create a G4Ros transport (ROS2 topic).
 * @param arm_side "left" or "right" arm
 * @param node_name ROS2 node name (optional)
 */
std::shared_ptr<Transport> create_g4ros_transport(
    const std::string& arm_side = "left",
    const std::string& node_name = "");

} // namespace a1z
