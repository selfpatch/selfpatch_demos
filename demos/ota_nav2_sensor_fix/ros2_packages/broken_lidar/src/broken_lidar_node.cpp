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
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

class BrokenLidarNode : public rclcpp::Node {
 public:
  BrokenLidarNode() : Node("scan_sensor_node") {
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    timer_ = create_wall_timer(100ms, [this]() { publish_scan(); });

    // Nav2 Jazzy publishes /cmd_vel as TwistStamped. Subscribing to
    // both Twist and TwistStamped works at runtime, but Foxglove
    // (which inspects subscribers when listing schemas) complains:
    // "Multiple channels advertise the same topic /cmd_vel but the
    // schema, schema name or encodings do not match". Stick to the
    // Nav2 Jazzy default - TwistStamped only.
    constexpr double kThresh = 0.01;
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        if (std::fabs(msg->twist.linear.x) > kThresh ||
            std::fabs(msg->twist.angular.z) > kThresh) {
          last_motion_command_ = now();
        }
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
    // Phantom obstacle: a single 1.0 m return on ray index 180 (angle
    // 0, straight ahead in base_scan). The single ray is just enough to
    // jitter the local costmap inflation layer near the path - the
    // global planner still finds a route, the controller engages and
    // drives /cmd_vel, but the planner has to keep replanning as the
    // phantom rotates with the robot. Wider wedges trigger
    // NO_VIABLE_PATH at the planner level and the action ABORTS before
    // the controller ever spins, which kills the demo's reactive-fault
    // beat (broken_lidar only raises the fault while /cmd_vel is
    // commanded).
    msg.ranges[180] = 1.0f;
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
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
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
