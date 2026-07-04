// Copyright 2026 bburda. Apache-2.0.
//
// Post-OTA (regressed) scan publisher: republishes the REAL gz front-laser
// scan (/scan_sim) onto /scan. It starts CLEAN (a straight passthrough) and
// only overlays a blocking phantom sector once the robot has driven a short
// distance - so the demo shows the AMR set off, drive up the aisle, and then
// lose a lidar sector mid-mission, rather than a robot that is dead on arrival.
// The stuck sector is a contiguous band of rays forced to a constant close
// range regardless of the real geometry; every real obstacle elsewhere in the
// scan stays intact, so Nav2's costmap sees a genuine blocking phantom without
// the robot driving blind through the rest of the warehouse.
//
// Because the phantom is sensor-fixed (it moves + rotates with the robot) it
// sits permanently in front of the AMR once active, so the local planner cannot
// steer around it or rotate away - every forward trajectory runs into it, Nav2
// makes no progress, and navigate_to_pose aborts. Three ROS parameters, so the
// behaviour can be tuned without touching code:
//   phantom_range_m           - the constant range the stuck rays report (m).
//                               Close enough that DWB's forward rollouts hit it.
//   phantom_half_band_rays    - half-width of the stuck sector in rays either
//                               side of straight-ahead.
//   phantom_onset_distance_m  - the robot drives this far (from where this node
//                               started, tracked on /odom) with a clean scan
//                               before the sector goes stuck. 0 = phantom from
//                               the first scan.
//
// Straight-ahead index = round((0 - angle_min) / angle_increment). For the
// RB-Theron front_laser (-2.3..2.3 rad, 810 samples) this is ~405.
//
// Pure republisher - no fault reporting. Nav2's own failure (logged errors, an
// aborted navigate_to_pose) is what turns this into a SOVD fault, via the
// log/action-status bridges - not this node naming its own bug. The onset delay
// only choreographs WHEN the sensor degrades; the fault itself is still Nav2's.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

class BrokenLidarNode : public rclcpp::Node {
 public:
  BrokenLidarNode() : Node("scan_sensor_node") {
    // phantom_range_m 0.22 -> the range is measured at the laser, ~0.268 m ahead
    // of base_footprint, so 0.22 m places the sector ~0.49 m from centre, just
    // outside the 0.45 m footprint: the goal is accepted, but once active the
    // robot has no room to creep and stalls. phantom_half_band_rays 184 -> a
    // +/-60 deg sector, wide enough that the sector rotating with the robot
    // leaves no clear forward heading.
    phantom_range_ = static_cast<float>(declare_parameter<double>("phantom_range_m", 0.22));
    phantom_half_band_ = declare_parameter<int>("phantom_half_band_rays", 184);
    onset_distance_ = declare_parameter<double>("phantom_onset_distance_m", 1.5);
    if (onset_distance_ <= 0.0) {
      phantom_active_.store(true);
    }

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_sim", 10,
      [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
        if (phantom_active_.load()) {
          overlay_phantom(*msg);
        }
        pub_->publish(std::move(msg));
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      [this](const nav_msgs::msg::Odometry & msg) { track_distance(msg); });
  }

 private:
  void track_distance(const nav_msgs::msg::Odometry & msg) {
    if (phantom_active_.load()) return;
    const double x = msg.pose.pose.position.x;
    const double y = msg.pose.pose.position.y;
    if (!have_origin_) {
      origin_x_ = x;
      origin_y_ = y;
      have_origin_ = true;
      return;
    }
    if (std::hypot(x - origin_x_, y - origin_y_) >= onset_distance_) {
      phantom_active_.store(true);
      RCLCPP_INFO(get_logger(), "Lidar sector went stuck after ~%.1f m driven.", onset_distance_);
    }
  }

  void overlay_phantom(sensor_msgs::msg::LaserScan & msg) const {
    if (msg.ranges.empty() || msg.angle_increment == 0.0f) return;

    const auto n = static_cast<int>(msg.ranges.size());
    const double angle_min = static_cast<double>(msg.angle_min);
    const double angle_increment = static_cast<double>(msg.angle_increment);
    const int idx0 = static_cast<int>(std::lround((0.0 - angle_min) / angle_increment));

    const int lo = std::max(0, idx0 - phantom_half_band_);
    const int hi = std::min(n - 1, idx0 + phantom_half_band_);
    for (int i = lo; i <= hi; ++i) {
      msg.ranges[static_cast<size_t>(i)] = phantom_range_;
    }
  }

  float phantom_range_{0.22f};
  int phantom_half_band_{184};
  double onset_distance_{1.5};
  bool have_origin_{false};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::atomic<bool> phantom_active_{false};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrokenLidarNode>());
  rclcpp::shutdown();
  return 0;
}
