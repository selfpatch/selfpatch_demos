// Copyright 2026 bburda. Apache-2.0.
//
// Post-OTA scan publisher: clean passthrough of the real gz front-laser
// scan (/scan_sim) onto /scan - no phantom, no fabricated data. The
// message is republished unchanged; frame, angles, ranges and increment
// all come from the incoming scan.
//
// Pure republisher - no fault reporting of any kind. Any fault raised
// while broken_lidar was applied comes from Nav2's own failure (via the
// log/action-status bridges), not from this node, and is a latched
// stored DTC: after the OTA swap the phantom disappears but the stored
// fault remains active on the Faults Dashboard until an operator clears
// it explicitly.

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class FixedLidarNode : public rclcpp::Node {
 public:
  FixedLidarNode() : Node("scan_sensor_node") {
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_sim", 10,
      [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
        pub_->publish(std::move(msg));
      });
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedLidarNode>());
  rclcpp::shutdown();
  return 0;
}
