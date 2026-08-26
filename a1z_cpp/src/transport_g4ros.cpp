#include "a1z/transport.hpp"

#ifdef A1Z_HAS_ROS2

#include <rclcpp/rclcpp.hpp>
#include <lemo_main_board/msg/motor_data.hpp>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>

namespace a1z {

class G4RosTransport : public Transport {
public:
    G4RosTransport(const std::string& arm_side, const std::string& node_name)
        : arm_side_(arm_side)
        , arm_id_(arm_side == "left" ? 1 : 2) {

        if (arm_side != "left" && arm_side != "right") {
            throw std::invalid_argument("arm_side must be 'left' or 'right'");
        }

        // Initialize ROS2 if needed
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
            owns_context_ = true;
        }

        // Create node
        std::string name = node_name.empty() ?
            ("a1z_g4ros_" + arm_side) : node_name;
        node_ = rclcpp::Node::make_shared(name);

        // Queue depth 100: /motor_send and /motor_data each carry both arms at
        // ~2700 msg/s; the old depth of 10 covered only ~3.7 ms of traffic, and
        // KEEP_LAST overflow drops the OLDEST frame first — which is always the
        // earliest-replying motor (can_id 1/2) in each control cycle.
        // 100 gives ~37 ms of slack; do NOT raise much further — with FastDDS
        // SHM transport each queued sample holds a shared-memory buffer, and
        // very deep reader queues can exhaust the writer-side SHM pool and
        // throttle g4spi_node's publisher for every subscriber (2026-08
        // measured severe receive degradation with depth 1000).
        pub_ = node_->create_publisher<lemo_main_board::msg::MotorData>(
            "motor_send", 100);
        // Explicit RELIABLE (matches the rclcpp default and g4spi_node's
        // publisher); written out so nobody "optimizes" it to best-effort,
        // which would silently drop motor feedback under load.
        sub_ = node_->create_subscription<lemo_main_board::msg::MotorData>(
            "motor_data", rclcpp::QoS(rclcpp::KeepLast(100)).reliable(),
            [this](lemo_main_board::msg::MotorData::ConstSharedPtr msg) {
                on_motor_frame(msg);
            });

        // Start spinning in background
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        spin_thread_ = std::thread([this]() { executor_->spin(); });
    }

    ~G4RosTransport() override {
        close();
    }

    bool send(const CanFrame& frame) override {
        if (!node_) return false;

        auto msg = lemo_main_board::msg::MotorData();
        msg.header.stamp = node_->now();
        msg.arm_id = arm_id_;
        msg.can_id = frame.id;
        msg.data.assign(frame.data.begin(), frame.data.begin() + frame.dlc);

        try {
            pub_->publish(msg);
            return true;
        } catch (...) {
            return false;
        }
    }

    std::optional<CanFrame> receive(int timeout_ms) override {
        std::unique_lock<std::mutex> lock(rx_mutex_);

        if (timeout_ms <= 0) {
            if (rx_queue_.empty()) return std::nullopt;
            auto frame = rx_queue_.front();
            rx_queue_.pop_front();
            return frame;
        }

        auto deadline = std::chrono::steady_clock::now() +
                       std::chrono::milliseconds(timeout_ms);

        while (rx_queue_.empty()) {
            if (rx_cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
                return std::nullopt;
            }
        }

        auto frame = rx_queue_.front();
        rx_queue_.pop_front();
        return frame;
    }

    void start_buffered_reader(std::function<void(const CanFrame&)> callback) override {
        // For ROS2, callbacks are already handled by the executor
        // This is a no-op since we use the subscription callback
        (void)callback;
    }

    void stop_buffered_reader() override {
        // No-op for ROS2
    }

    void close() override {
        if (executor_) {
            executor_->cancel();
        }
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
        if (node_) {
            node_.reset();
        }
        if (owns_context_ && rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    bool is_open() const override {
        return node_ != nullptr;
    }

    TransportType type() const override {
        return TransportType::G4Ros;
    }

private:
    void on_motor_frame(lemo_main_board::msg::MotorData::ConstSharedPtr msg) {
        // Filter frames for the other arm
        if (msg->arm_id != arm_id_) {
            return;
        }

        // Inter-arrival gap monitor: gaps >100 ms mean the DDS delivery or
        // this executor's spin thread stalled (2026-08-26 right-arm 305 ms
        // feedback outage — root cause still unknown, this catches it live).
        const auto now = std::chrono::steady_clock::now();
        if (last_cb_time_.time_since_epoch().count() != 0) {
            const double gap_ms =
                std::chrono::duration<double, std::milli>(now - last_cb_time_).count();
            if (gap_ms > 100.0) {
                std::cerr << "[G4Ros:" << arm_side_ << "] motor_data gap "
                          << gap_ms << " ms" << std::endl;
            }
        }
        last_cb_time_ = now;

        CanFrame frame;
        frame.id = msg->can_id;
        frame.dlc = std::min(static_cast<size_t>(msg->data.size()),
                            static_cast<size_t>(8));
        std::copy(msg->data.begin(), msg->data.begin() + frame.dlc,
                 frame.data.begin());

        std::lock_guard<std::mutex> lock(rx_mutex_);
        rx_queue_.push_back(frame);
        rx_cv_.notify_one();
    }

    std::string arm_side_;
    uint8_t arm_id_;
    bool owns_context_ = false;
    std::chrono::steady_clock::time_point last_cb_time_{};

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<lemo_main_board::msg::MotorData>::SharedPtr pub_;
    rclcpp::Subscription<lemo_main_board::msg::MotorData>::SharedPtr sub_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread spin_thread_;

    std::mutex rx_mutex_;
    std::condition_variable rx_cv_;
    std::deque<CanFrame> rx_queue_;
};

std::shared_ptr<Transport> create_g4ros_transport(
    const std::string& arm_side, const std::string& node_name) {
    return std::make_shared<G4RosTransport>(arm_side, node_name);
}

} // namespace a1z

#else // !A1Z_HAS_ROS2

namespace a1z {

std::shared_ptr<Transport> create_g4ros_transport(
    const std::string& arm_side, const std::string& node_name) {
    (void)arm_side;
    (void)node_name;
    throw std::runtime_error("G4Ros transport not available: compiled without ROS2 support");
}

} // namespace a1z

#endif // A1Z_HAS_ROS2
