#include "a1z/transport.hpp"
#include "a1z/can_interface.hpp"

namespace a1z {

class SocketCanTransport : public Transport {
public:
    SocketCanTransport(const std::string& channel, int bitrate, const std::string& name)
        : can_(std::make_shared<CanInterface>(channel, bitrate, name)) {}

    bool send(const CanFrame& frame) override {
        return can_->send(frame);
    }

    std::optional<CanFrame> receive(int timeout_ms) override {
        return can_->receive(timeout_ms);
    }

    void start_buffered_reader(std::function<void(const CanFrame&)> callback) override {
        can_->start_buffered_reader(std::move(callback));
    }

    void stop_buffered_reader() override {
        can_->stop_buffered_reader();
    }

    void close() override {
        can_->close();
    }

    bool is_open() const override {
        return can_->is_open();
    }

    TransportType type() const override {
        return TransportType::SocketCAN;
    }

private:
    std::shared_ptr<CanInterface> can_;
};

std::shared_ptr<Transport> create_socketcan_transport(
    const std::string& channel, int bitrate, const std::string& name) {
    return std::make_shared<SocketCanTransport>(channel, bitrate, name);
}

} // namespace a1z
