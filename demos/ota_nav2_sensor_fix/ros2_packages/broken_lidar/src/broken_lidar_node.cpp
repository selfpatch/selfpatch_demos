// Copyright 2026 bburda. Apache-2.0.
#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::chrono_literals::operator""ms;

class BrokenLidarNode : public rclcpp::Node {
 public:
  BrokenLidarNode() : Node("scan_sensor_node") {
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    timer_ = create_wall_timer(100ms, [this]() { publish_scan(); });
  }

 private:
  void publish_scan() {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_scan";
    msg.angle_min = -static_cast<float>(M_PI);
    msg.angle_max = static_cast<float>(M_PI);
    msg.angle_increment = static_cast<float>(M_PI / 180.0);
    msg.range_min = 0.05f;
    msg.range_max = 10.0f;
    constexpr int kRays = 360;
    msg.ranges.assign(kRays, msg.range_max);
    // Inject a 1 m phantom return at angle 0 (straight ahead, ray index 180)
    msg.ranges[180] = 1.0f;
    pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrokenLidarNode>());
  rclcpp::shutdown();
  return 0;
}
