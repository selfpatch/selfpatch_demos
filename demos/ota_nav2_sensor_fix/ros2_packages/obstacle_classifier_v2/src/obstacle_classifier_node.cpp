// Copyright 2026 bburda. Apache-2.0.
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ObstacleClassifierNode : public rclcpp::Node {
 public:
  ObstacleClassifierNode() : Node("obstacle_classifier") {
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("safety_overlay", 10);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { on_scan(*msg); });
  }

 private:
  void on_scan(const sensor_msgs::msg::LaserScan & scan) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker m;
    m.header = scan.header;
    m.ns = "safety_overlay";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.4;
    m.scale.y = 0.4;
    m.scale.z = 0.4;
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.4f;
    m.color.a = 0.6f;
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.2;
    m.pose.orientation.w = 1.0;
    markers.markers.push_back(m);
    pub_->publish(markers);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleClassifierNode>());
  rclcpp::shutdown();
  return 0;
}
