// #include <arpa/inet.h>
// #include <endian.h>
// #include <netinet/in.h>
// #include <sys/socket.h>
// #include <unistd.h>

// #include <algorithm>
// #include <chrono>
// #include <cstdint>
// #include <cstring>
// #include <stdexcept>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/int32_multi_array.hpp"

// using namespace std::chrono_literals;

// namespace {
// constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
// constexpr size_t kPacketSize = 12;
// }  // namespace

// class UdpSender : public rclcpp::Node {
//  public:
//   UdpSender() : Node("udp_sender") {
//     std::string host = declare_parameter("host", std::string("127.0.0.1"));
//     int port = declare_parameter("port", 12151);

//     sock_ = socket(AF_INET, SOCK_DGRAM, 0);
//     if (sock_ < 0) throw std::runtime_error("socket() failed");

//     addr_.sin_family = AF_INET;
//     addr_.sin_port = htons(static_cast<uint16_t>(port));
//     if (inet_pton(AF_INET, host.c_str(), &addr_.sin_addr) != 1) {
//       close(sock_);
//       throw std::runtime_error("invalid host");
//     }

//     sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
//         "cmd", 10, [this](std_msgs::msg::Int32MultiArray::SharedPtr msg) {
//           if (msg->data.size() < 2) {
//             RCLCPP_WARN(get_logger(), "cmd needs [pwm, steer]");
//             return;
//           }
//           pwm_ = std::clamp<int32_t>(msg->data[0], -255, 255);
//           steer_ = msg->data[1];
//           last_cmd_ = now();
//           has_cmd_ = true;
//         });

//     timer_ = create_wall_timer(50ms, [this]() { send(); });  // 20 Hz
//     RCLCPP_INFO(get_logger(), "Sending to %s:%d, subscribed to /cmd", host.c_str(), port);
//   }

//   ~UdpSender() override {
//     pwm_ = 0;
//     for (int i = 0; i < 3; ++i) send_packet();  // stop packet, repeated in case one is lost
//     if (sock_ >= 0) close(sock_);
//   }

//  private:
//   void send() {
//     // timeout: stop the robot if commands stop arriving
//     if (!has_cmd_ || (now() - last_cmd_).seconds() > 0.5) pwm_ = 0;
//     send_packet();
//   }

//   void send_packet() {
//     uint8_t buf[kPacketSize];
//     uint32_t pwm_le = htole32(static_cast<uint32_t>(pwm_));
//     uint32_t steer_le = htole32(static_cast<uint32_t>(steer_));
//     std::memcpy(buf, kHeader, sizeof(kHeader));
//     std::memcpy(buf + 4, &pwm_le, sizeof(pwm_le));
//     std::memcpy(buf + 8, &steer_le, sizeof(steer_le));
//     sendto(sock_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&addr_), sizeof(addr_));
//   }

//   int sock_{-1};
//   sockaddr_in addr_{};
//   int32_t pwm_{0};
//   int32_t steer_{0};
//   bool has_cmd_{false};
//   rclcpp::Time last_cmd_{0, 0, RCL_ROS_TIME};
//   rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
//   rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<UdpSender>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <arpa/inet.h>
#include <endian.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace {
constexpr char kHeader[4] = {'I', 'R', 'I', 'S'};
constexpr size_t kPacketSize = 12;
}  // namespace

class UdpSender : public rclcpp::Node {
 public:
  UdpSender() : Node("udp_sender") {
    std::string host = declare_parameter("host", std::string("127.0.0.1"));
    int port = declare_parameter("port", 12151);
    pwm_step_ = declare_parameter("pwm_step", 20);
    steer_step_ = declare_parameter("steer_step", 5);
    max_pwm_ = declare_parameter("max_pwm", 255);
    max_steer_ = declare_parameter("max_steer", 30);

    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) throw std::runtime_error("socket() failed");

    addr_.sin_family = AF_INET;
    addr_.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &addr_.sin_addr) != 1) {
      close(sock_);
      throw std::runtime_error("invalid host");
    }

    // baca tombol langsung, tanpa Enter dan tanpa echo
    tcgetattr(STDIN_FILENO, &old_);
    termios raw = old_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    std::printf("W/S: maju/mundur | A/D: belok | Q: lurus | Spasi: stop | Ctrl-C: keluar\n");
    timer_ = create_wall_timer(50ms, [this]() { tick(); });  // 20 Hz
  }

  ~UdpSender() override {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_);
    pwm_ = 0;
    for (int i = 0; i < 3; ++i) send_packet();  // paket stop, diulang kalau ada yang hilang
    if (sock_ >= 0) close(sock_);
    std::printf("\n");
  }

 private:
  void tick() {
    char c;
    while (read(STDIN_FILENO, &c, 1) == 1) {
      switch (std::tolower(static_cast<unsigned char>(c))) {
        case 'w': pwm_ += pwm_step_; break;
        case 's': pwm_ -= pwm_step_; break;
        case 'a': steer_ -= steer_step_; break;  // kalau arah terbalik, tukar a dan d
        case 'd': steer_ += steer_step_; break;
        case 'q': steer_ = 0; break;
        case ' ': pwm_ = 0; steer_ = 0; break;
        default: break;
      }
    }
    pwm_ = std::clamp(pwm_, -max_pwm_, max_pwm_);
    steer_ = std::clamp(steer_, -max_steer_, max_steer_);

    send_packet();
    std::printf("\rpwm=%4d  steer=%3d   ", pwm_, steer_);
    std::fflush(stdout);
  }

  void send_packet() {
    uint8_t buf[kPacketSize];
    uint32_t pwm_le = htole32(static_cast<uint32_t>(pwm_));
    uint32_t steer_le = htole32(static_cast<uint32_t>(steer_));
    std::memcpy(buf, kHeader, sizeof(kHeader));
    std::memcpy(buf + 4, &pwm_le, sizeof(pwm_le));
    std::memcpy(buf + 8, &steer_le, sizeof(steer_le));
    sendto(sock_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&addr_), sizeof(addr_));
  }

  int sock_{-1};
  sockaddr_in addr_{};
  int pwm_step_, steer_step_, max_pwm_, max_steer_;
  int pwm_{0}, steer_{0};
  termios old_{};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpSender>());
  rclcpp::shutdown();
  return 0;
}