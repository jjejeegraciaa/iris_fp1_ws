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
#include <cstdint>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals

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

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        add.sin_port = htons(static_cast<uint16_t
     }
}