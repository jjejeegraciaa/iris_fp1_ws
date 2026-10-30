#include <algorithm>
#include <cmath>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class MasterNode : public rclcpp::Node {
 public:
  MasterNode() : Node("master_node") {
    center_x_ = declare_parameter("center_x", 400.0);  
    lookahead_idx_ = declare_parameter("lookahead_idx", 2);  

    kp_ = declare_parameter("kp", 0.08);
    ki_ = declare_parameter("ki", 0.0);  
    kd_ = declare_parameter("kd", 0.03);
    i_limit_ = declare_parameter("i_limit", 10.0);  
    d_smooth_ = declare_parameter("d_smooth", 0.3); 
    max_steer_ = declare_parameter("max_steer", 30.0);
    steer_sign_ = declare_parameter("steer_sign", 1.0);  
    smooth_ = declare_parameter("smooth", 0.4);

    pwm_max_ = declare_parameter("pwm_max", 80);
    pwm_min_ = declare_parameter("pwm_min", 50);

    lost_timeout_ = declare_parameter("lost_timeout", 0.5);

    vision_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
        "vision_centers", 10,
        std::bind(&MasterNode::visionCallback, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("cmd", 10);
    RCLCPP_INFO(get_logger(), "Master Node (PID) berjalan");
  }

 private:
  void resetPid() {
    integral_ = 0.0;
    d_filtered_ = 0.0;
    have_last_ = false;
  }

  void visionCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    size_t n = msg->data.size() / 2;
    rclcpp::Time t = now();

    if (n == 0) {
      if ((t - last_seen_).seconds() > lost_timeout_) {
        resetPid();
        steer_f_ = 0.0;
        publishCommand(0, 0);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Jalur hilang, berhenti");
      } else {
        publishCommand(last_pwm_, static_cast<int>(std::lround(steer_f_)));
      }
      return;
    }
    last_seen_ = t;

    size_t idx = std::min<size_t>(static_cast<size_t>(lookahead_idx_), n - 1);
    double target_x = msg->data[idx * 2];
    double error = target_x - center_x_;

    double dt = have_last_ ? (t - last_time_).seconds() : 0.0;
    if (dt > 0.5) {  
      resetPid();
      dt = 0.0;
    }

    double p_term = kp_ * error;

    double i_term = 0.0;
    double d_term = 0.0;
    if (have_last_ && dt > 1e-3) {
    
      integral_ += error * dt;
      if (ki_ > 1e-9) {
        double lim = i_limit_ / ki_;
        integral_ = std::clamp(integral_, -lim, lim);
      }
      i_term = ki_ * integral_;

      double d_raw = (error - last_error_) / dt;
      d_filtered_ = d_smooth_ * d_raw + (1.0 - d_smooth_) * d_filtered_;
      d_term = kd_ * d_filtered_;
    }

    last_error_ = error;
    last_time_ = t;
    have_last_ = true;

    double raw = std::clamp((p_term + i_term + d_term) * steer_sign_,
                            -max_steer_, max_steer_);
    steer_f_ = smooth_ * raw + (1.0 - smooth_) * steer_f_;

    double ratio = std::min(std::abs(steer_f_) / max_steer_, 1.0);
    int pwm = static_cast<int>(pwm_max_ - ratio * (pwm_max_ - pwm_min_));

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "x=%.0f err=%.0f P=%.1f I=%.1f D=%.1f steer=%.1f pwm=%d",
                         target_x, error, p_term, i_term, d_term, steer_f_, pwm);
    publishCommand(pwm, static_cast<int>(std::lround(steer_f_)));
  }

  void publishCommand(int pwm, int steer) {
    last_pwm_ = pwm;
    std_msgs::msg::Int32MultiArray cmd;
    cmd.data = {pwm, steer};
    cmd_pub_->publish(cmd);
  }

  double center_x_, kp_, ki_, kd_, i_limit_, d_smooth_;
  double max_steer_, steer_sign_, smooth_, lost_timeout_;
  int pwm_max_, pwm_min_, lookahead_idx_;

  double integral_{0.0}, d_filtered_{0.0}, last_error_{0.0}, steer_f_{0.0};
  int last_pwm_{0};
  bool have_last_{false};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_seen_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr vision_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MasterNode>());
  rclcpp::shutdown();
  return 0;
}