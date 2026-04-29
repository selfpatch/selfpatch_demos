// Copyright 2026 bburda. Apache-2.0.
//
// Post-OTA scan publisher: clean 10 m returns on every ray, no phantom.
//
// On startup we also fire one EVENT_PASSED for SCAN_PHANTOM_RETURN to
// the fault_manager. broken_lidar reactively raises that fault while
// the controller is driving, but it cannot clear its own fault when the
// OTA process kills it - the report_passed call would never run. fixed
// _lidar takes over the scan_sensor_node identity, so it can credibly
// report "the fault is gone" on behalf of that source. Without this the
// Faults Dashboard stays red forever after the OTA swap, even though
// the underlying sensor is healthy.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

class FixedLidarNode : public rclcpp::Node {
 public:
  FixedLidarNode() : Node("scan_sensor_node") {
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    timer_ = create_wall_timer(100ms, [this]() { publish_scan(); });

    fault_client_ = create_client<ros2_medkit_msgs::srv::ReportFault>(
      "/fault_manager/report_fault");
    // Heartbeat at 500 ms (4x the broken_lidar FAILED tick rate) so the
    // healing path overtakes the FAILED counter quickly even if the
    // operator pushed multiple goals before the OTA. Stops being
    // load-bearing after the fault flips to HEALED - the storage just
    // no-ops further EVENT_PASSED reports - so the perpetual timer is
    // free.
    clear_timer_ = create_wall_timer(500ms, [this]() { try_clear_fault(); });
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
    pub_->publish(msg);
  }

  void try_clear_fault() {
    // FaultManager's healing path uses a signed debounce counter:
    // EVENT_FAILED decrements it, EVENT_PASSED increments it, and the
    // fault transitions to HEALED only when the counter rises to
    // `healing_threshold`. broken_lidar fires a FAILED tick every 2 s
    // while nav2 is driving, so by the time fixed_lidar takes over the
    // counter can be deeply negative (-15 to -30). We need to send
    // *more* PASSED events than broken_lidar sent FAILED events for
    // the counter to climb back above the heal threshold. The cheapest
    // path is to keep firing every 2 s for the lifetime of the node -
    // once the fault flips to HEALED the storage rejects further
    // PASSED events as no-ops, so we keep watching but it stops being
    // load-bearing.
    if (!fault_client_->service_is_ready()) return;

    auto req = std::make_shared<ros2_medkit_msgs::srv::ReportFault::Request>();
    req->fault_code = "SCAN_PHANTOM_RETURN";
    req->event_type = ros2_medkit_msgs::srv::ReportFault::Request::EVENT_PASSED;
    req->severity = 0;
    req->description = "fixed_lidar took over scan_sensor_node - phantom returns no longer published.";
    // Same FQN as broken_lidar so the EVENT_PASSED clears the fault on
    // the same reporting source - see broken_lidar_node.cpp for why a
    // bare name doesn't match the per-app/per-component aggregation.
    req->source_id = "/scan_sensor_node";

    auto cb = [this](rclcpp::Client<ros2_medkit_msgs::srv::ReportFault>::SharedFuture fut) {
      try {
        auto result = fut.get();
        if (result->accepted) {
          clear_call_count_++;
          if (clear_call_count_ == 1) {
            RCLCPP_INFO(get_logger(), "Sent first EVENT_PASSED for SCAN_PHANTOM_RETURN");
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(get_logger(), "ReportFault EVENT_PASSED call failed: %s", e.what());
      }
    };
    auto fut = fault_client_->async_send_request(req, cb);
    (void)fut;
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<ros2_medkit_msgs::srv::ReportFault>::SharedPtr fault_client_;
  rclcpp::TimerBase::SharedPtr clear_timer_;
  int clear_call_count_{0};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedLidarNode>());
  rclcpp::shutdown();
  return 0;
}
