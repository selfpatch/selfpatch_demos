// Copyright 2026 bburda. Apache-2.0.
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

using std::chrono_literals::operator""s;

class LegacyNode : public rclcpp::Node {
 public:
  LegacyNode() : Node("broken_lidar_legacy") {
    timer_ = create_wall_timer(5s, []() {});
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegacyNode>());
  rclcpp::shutdown();
  return 0;
}
