// Copyright 2026 bburda. Apache-2.0.
//
// Pre-OTA scan publisher: emits a 360-ray LaserScan at 10 Hz with a
// phantom 1 m return at index 180 (straight ahead). Nav2's local
// costmap traces it as an obstacle and the planner refuses to drive
// through.
//
// Reactive fault reporting (the demo narrative pivots on this beat):
// the node subscribes to /cmd_vel. While the controller is actively
// commanding non-zero motion - i.e. the operator just sent a goal and
// nav2 is trying to drive - we raise SCAN_PHANTOM_RETURN against
// scan_sensor_node every 5 s. After 10 s of idle /cmd_vel we emit
// report_passed and the fault clears from the dashboard.
//
// Result on screen: the Faults Dashboard is empty when the robot is
// idle, lights up the moment the operator publishes /goal_pose and
// the controller starts spinning, and clears once they stop driving
// or the OTA swap installs fixed_lidar (which doesn't make any of
// these reports).

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

class BrokenLidarNode : public rclcpp::Node {
 public:
  BrokenLidarNode() : Node("scan_sensor_node") {
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    timer_ = create_wall_timer(100ms, [this]() { publish_scan(); });

    // Nav2 Jazzy publishes /cmd_vel as TwistStamped; older stacks (and
    // teleop) still use plain Twist. Subscribe to both so the reactive
    // fault works regardless of which side is driving.
    constexpr double kThresh = 0.01;
    auto handle_motion = [this](double linear_x, double angular_z) {
      if (std::fabs(linear_x) > kThresh || std::fabs(angular_z) > kThresh) {
        last_motion_command_ = now();
      }
    };
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [handle_motion](const geometry_msgs::msg::Twist::SharedPtr msg) {
        handle_motion(msg->linear.x, msg->angular.z);
      });
    cmd_vel_stamped_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10,
      [handle_motion](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        handle_motion(msg->twist.linear.x, msg->twist.angular.z);
      });

    fault_client_ = create_client<ros2_medkit_msgs::srv::ReportFault>(
      "/fault_manager/report_fault");
    fault_timer_ = create_wall_timer(2s, [this]() { tick_fault(); });
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
    // Phantom obstacle: a 21-ray (~20 degree) wedge of 0.5 m returns
    // centered straight ahead. A single ray at 1.0 m the local costmap
    // happily plans around because nav2 marks one cell, raytracing
    // clears it on the next sweep, and the controller drives forward
    // anyway. A close wide wedge gets stamped into the local costmap
    // as a continuous wall the planner has to swerve to avoid - which
    // it can't reliably do because the wedge stays anchored to base_scan
    // (it rotates with the robot). End result: the controller stalls,
    // BT navigator times out, NavigateToPose returns ABORTED.
    constexpr int kPhantomCenter = 180;
    constexpr int kPhantomHalfWidth = 10;
    constexpr float kPhantomRange = 0.5f;
    for (int i = kPhantomCenter - kPhantomHalfWidth;
         i <= kPhantomCenter + kPhantomHalfWidth;
         ++i) {
      msg.ranges[i] = kPhantomRange;
    }
    pub_->publish(msg);
  }

  void tick_fault() {
    if (!fault_client_->service_is_ready()) return;

    const auto now_ts = now();
    const bool driving = last_motion_command_.nanoseconds() > 0
        && (now_ts - last_motion_command_).seconds() < kIdleTimeoutSec;

    if (driving) {
      send_report(false);  // EVENT_FAILED - keep fault active
      fault_active_ = true;
    } else if (fault_active_) {
      send_report(true);   // EVENT_PASSED - clear fault
      fault_active_ = false;
    }
  }

  void send_report(bool passed) {
    auto req = std::make_shared<ros2_medkit_msgs::srv::ReportFault::Request>();
    req->fault_code = "SCAN_PHANTOM_RETURN";
    req->event_type = passed
      ? ros2_medkit_msgs::srv::ReportFault::Request::EVENT_PASSED
      : ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED;
    req->severity = 3;  // ERROR
    req->description = "LaserScan ray index 180 reports a constant 1.0 m return "
                       "(straight ahead). Nav2 traces it as a phantom obstacle "
                       "and the controller cannot make progress.";
    req->source_id = "scan_sensor_node";

    auto cb = [this, passed](rclcpp::Client<ros2_medkit_msgs::srv::ReportFault>::SharedFuture fut) {
      try {
        auto result = fut.get();
        if (!result->accepted) {
          RCLCPP_DEBUG(get_logger(), "FaultManager rejected SCAN_PHANTOM_RETURN report (passed=%d)",
                       passed ? 1 : 0);
        }
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(get_logger(), "ReportFault call failed: %s", e.what());
      }
    };
    auto fut = fault_client_->async_send_request(req, cb);
    (void)fut;
  }

  static constexpr double kIdleTimeoutSec = 10.0;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_sub_;
  rclcpp::Client<ros2_medkit_msgs::srv::ReportFault>::SharedPtr fault_client_;
  rclcpp::TimerBase::SharedPtr fault_timer_;
  rclcpp::Time last_motion_command_{0, 0, RCL_ROS_TIME};
  bool fault_active_{false};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrokenLidarNode>());
  rclcpp::shutdown();
  return 0;
}
