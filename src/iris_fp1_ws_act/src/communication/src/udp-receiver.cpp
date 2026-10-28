#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <chrono>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

namespace {

constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
constexpr size_t kPacketSize = 20;

int32_t read_i32_le(const uint8_t *src) {
    uint32_t raw;
    std::memcpy(&raw, src, sizeof(raw));
    return static_cast<int32_t>(le32toh(raw));
}
}

class UdpReceiver : public rclcpp::Node {
    public:
     UdpReceiver() : Node("udp_receiver"){
        int port = declare_parameter("port", 12152);
        pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("feedback", 10);

        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            throw std::runtime_error("socket() failed");
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) <0) {
        close(sock_);
        throw std::runtime_error("bind() failed (port in use?)");
     }

     timer_ = create_wall_timer(2ms, [this]() { poll(); });
     RCLCPP_INFO(get_logger(), "Listening on UDP port %d", port);
}

~UdpReceiver() override {
    if (sock_ >= 0) close(sock_);
}

private:
 void poll() {
    uint8_t buf[64];
    while (true) {
        ssize_t n = recvfrom(sock_, buf, sizeof(buf), MSG_DONTWAIT, nullptr, nullptr);
        if (n < 0) return;
        if (static_cast<size_t>(n) < kPacketSize ||
            std::memcmp(buf, kHeader, sizeof(kHeader)) != 0) {
          RCLCPP_WARN(get_logger(), "Ignoring malformed packet (%zd bytes)", n);
          continue;    
        }
        std_msgs::msg::Int32MultiArray msg;
        msg.data = {read_i32_le(buf + 4), read_i32_le(buf + 8),
                    read_i32_le(buf + 12), read_i32_le(buf + 16)};
        pub_->publish(msg);
    }
 }

 int sock_{-1};
 rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
 rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpReceiver>());
    rclcpp::shutdown();
    return 0;
}