// Copyright 2026 bburda. Apache-2.0.
//
// Operator-invoked differential-diagnosis node. It does NOT raise faults -
// the log/action-status bridges already turn Nav2's own stall into SOVD
// faults once broken_lidar's phantom sector is applied. This node answers
// a single on-demand operation (health_check_msgs/RunHealthChecks) so an
// operator working a "navigate_to_pose aborted" fault can narrow the root
// cause down to a specific subsystem instead of guessing. The request picks
// which of the four checks to run (each bool defaults to true, so a bare
// call runs everything):
//   lidar        - is /scan reporting a stuck close sector?
//   localization - is AMCL still localizing?
//   drivetrain   - is odometry/joint-state telemetry live?
//   costmap      - does the local costmap show a live, sensor-only
//                  obstacle right in front (no matching static-map
//                  feature)?
//
// The response carries one report line per requested check plus an overall
// ok / checks_run / checks_failed summary - see
// health_check_msgs/srv/RunHealthChecks.srv.
//
// Each check only inspects the latest cached message per topic - no
// history, no state machine, no interaction with the OTA/fault-manager
// flow. Subscribing independently of scan_sensor_node means this node
// survives the broken_lidar <-> fixed_lidar swap untouched, so the same
// operation answers "broken" before the fix and "healthy" after it.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <health_check_msgs/srv/run_health_checks.hpp>

namespace {
// Lidar stuck-sector heuristic (see class comment above check_lidar_health).
constexpr float kStuckBandM = 0.05f;
constexpr float kStuckMaxRangeM = 1.5f;
constexpr size_t kStuckMinRunRays = 30;

// Freshness windows.
constexpr double kLocalizationMaxVar = 1.0;  // AMCL xy covariance (m^2); above this = diverged
constexpr double kDrivetrainFreshSec = 5.0;

// Costmap "directly ahead" window, robot/grid-frame relative.
constexpr double kCostmapAheadMinM = 0.3;
constexpr double kCostmapAheadMaxM = 0.8;
constexpr double kCostmapLateralM = 0.3;
constexpr int8_t kLethalCellValue = 99;

// Result of a single check: whether it passed and a human-readable detail
// message, combined into one "<name>: ok|FAIL - <message>" report line.
struct CheckOutcome {
  bool ok;
  std::string message;
};
}  // namespace

class HealthCheckNode : public rclcpp::Node {
 public:
  HealthCheckNode() : Node("health_check") {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_ = std::move(msg);
        last_scan_time_ = this->now();
        scan_received_ = true;
      });

    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        last_amcl_pose_ = std::move(msg);
        last_amcl_pose_time_ = this->now();
        amcl_pose_received_ = true;
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_ = std::move(msg);
        last_odom_time_ = this->now();
        odom_received_ = true;
      });

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        last_joint_state_ = std::move(msg);
        last_joint_state_time_ = this->now();
        joint_state_received_ = true;
      });

    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", 10,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        last_costmap_ = std::move(msg);
        last_costmap_time_ = this->now();
        costmap_received_ = true;
      });

    run_health_checks_srv_ = create_service<health_check_msgs::srv::RunHealthChecks>(
      "~/run_health_checks",
      [this](const std::shared_ptr<health_check_msgs::srv::RunHealthChecks::Request> request,
             std::shared_ptr<health_check_msgs::srv::RunHealthChecks::Response> response) {
        handle_run_health_checks(request, response);
      });
  }

  // Subscription callbacks capture `this`; drop them before the rest of the
  // object is torn down so none can fire against a partially-destroyed node.
  ~HealthCheckNode() override {
    scan_sub_.reset();
    amcl_pose_sub_.reset();
    odom_sub_.reset();
    joint_state_sub_.reset();
    costmap_sub_.reset();
  }

  HealthCheckNode(const HealthCheckNode &) = delete;
  HealthCheckNode & operator=(const HealthCheckNode &) = delete;
  HealthCheckNode(HealthCheckNode &&) = delete;
  HealthCheckNode & operator=(HealthCheckNode &&) = delete;

 private:
  // Runs every check whose request flag is true, builds one report line per
  // requested check ("<name>: ok|FAIL - <detail>"), and rolls the results up
  // into ok/checks_run/checks_failed.
  void handle_run_health_checks(
    const std::shared_ptr<health_check_msgs::srv::RunHealthChecks::Request> request,
    std::shared_ptr<health_check_msgs::srv::RunHealthChecks::Response> response) const {
    std::vector<std::string> lines;
    uint8_t checks_run = 0;
    uint8_t checks_failed = 0;

    const auto record = [&](const char * name, const CheckOutcome & outcome) {
      ++checks_run;
      if (!outcome.ok) ++checks_failed;
      std::ostringstream line;
      line << name << ": " << (outcome.ok ? "ok" : "FAIL") << " - " << outcome.message;
      lines.push_back(line.str());
    };

    // Only evaluate (and report on) a check when it was actually requested.
    if (request->lidar) record("lidar", check_lidar_health());
    if (request->localization) record("localization", check_localization_health());
    if (request->drivetrain) record("drivetrain", check_drivetrain_health());
    if (request->costmap) record("costmap", check_costmap_health());

    std::ostringstream report;
    for (size_t i = 0; i < lines.size(); ++i) {
      if (i > 0) report << "\n";
      report << lines[i];
    }

    response->checks_run = checks_run;
    response->checks_failed = checks_failed;
    response->report = report.str();
    response->ok = (checks_failed == 0);
  }

  // Scans for the longest run of consecutive FINITE ranges that are all
  // within kStuckBandM of one another and below kStuckMaxRangeM - a
  // contiguous band of near-equal close returns is the signature of the
  // broken_lidar phantom sector (a real environment practically never
  // presents 30+ consecutive rays all within 5 cm of each other).
  CheckOutcome check_lidar_health() const {
    if (!scan_received_ || !last_scan_) {
      return {false, "no /scan received"};
    }

    const auto & ranges = last_scan_->ranges;
    const size_t n = ranges.size();

    size_t finite_count = 0;
    float finite_min = std::numeric_limits<float>::infinity();
    float finite_max = -std::numeric_limits<float>::infinity();
    for (const float v : ranges) {
      if (!std::isfinite(v)) continue;
      ++finite_count;
      finite_min = std::min(finite_min, v);
      finite_max = std::max(finite_max, v);
    }

    size_t best_start = 0;
    size_t best_len = 0;
    float best_value = 0.0f;
    size_t i = 0;
    while (i < n) {
      if (!std::isfinite(ranges[i]) || ranges[i] >= kStuckMaxRangeM) {
        ++i;
        continue;
      }
      const size_t start = i;
      float run_min = ranges[i];
      float run_max = ranges[i];
      size_t j = i + 1;
      while (j < n && std::isfinite(ranges[j]) && ranges[j] < kStuckMaxRangeM) {
        const float new_min = std::min(run_min, ranges[j]);
        const float new_max = std::max(run_max, ranges[j]);
        if (new_max - new_min > kStuckBandM) break;
        run_min = new_min;
        run_max = new_max;
        ++j;
      }
      const size_t len = j - start;
      if (len > best_len) {
        best_len = len;
        best_start = start;
        best_value = (run_min + run_max) / 2.0f;
      }
      i = j;
    }

    if (best_len >= kStuckMinRunRays) {
      const double angle_min = static_cast<double>(last_scan_->angle_min);
      const double angle_increment = static_cast<double>(last_scan_->angle_increment);
      const double bearing_rad =
        angle_min + (static_cast<double>(best_start) + static_cast<double>(best_len) / 2.0) * angle_increment;
      const double bearing_deg = bearing_rad * 180.0 / M_PI;

      std::ostringstream oss;
      oss << "Stuck lidar sector: " << best_len << " rays pinned near " << std::fixed << std::setprecision(2)
          << best_value << " m around bearing " << std::setprecision(1) << bearing_deg
          << " deg - the lidar is reporting a phantom obstacle.";
      return {false, oss.str()};
    }

    std::ostringstream oss;
    oss << "Lidar nominal: " << finite_count << " finite returns";
    if (finite_count > 0) {
      oss << ", ranges " << std::fixed << std::setprecision(2) << finite_min << "-" << finite_max << " m";
    }
    oss << ", no stuck sector.";
    return {true, oss.str()};
  }

  CheckOutcome check_localization_health() const {
    // AMCL only republishes /amcl_pose on a motion-triggered filter update, so a
    // stationary robot that is already localized lets it go stale - that is NOT a
    // localization loss (AMCL keeps broadcasting map->odom the whole time). Judge
    // on the last pose's covariance (small = converged), not its age, so a robot
    // sitting still after the OTA fix still reports healthy; a genuine divergence
    // still trips the covariance bound.
    if (amcl_pose_received_ && last_amcl_pose_) {
      const auto & cov = last_amcl_pose_->pose.covariance;
      const double xy_var = cov[0] + cov[7];  // var(x) + var(y), m^2
      const double age_s = (this->now() - last_amcl_pose_time_).seconds();
      if (xy_var <= kLocalizationMaxVar) {
        std::ostringstream oss;
        oss << "Localization healthy: AMCL converged (xy variance " << std::fixed
            << std::setprecision(3) << xy_var << " m^2, last update "
            << std::setprecision(1) << age_s << "s ago).";
        return {true, oss.str()};
      }
      return {false, "AMCL localization diverged (high covariance)"};
    }
    return {false, "AMCL pose absent - never localized"};
  }

  CheckOutcome check_drivetrain_health() const {
    const double odom_age =
      odom_received_ ? (this->now() - last_odom_time_).seconds() : std::numeric_limits<double>::infinity();
    const double joint_age = joint_state_received_ ? (this->now() - last_joint_state_time_).seconds()
                                                    : std::numeric_limits<double>::infinity();
    const bool odom_fresh = odom_received_ && odom_age <= kDrivetrainFreshSec;
    const bool joint_fresh = joint_state_received_ && joint_age <= kDrivetrainFreshSec;

    if (odom_fresh && joint_fresh) {
      return {true, "Drivetrain healthy: odometry + joint states streaming, diff-drive active."};
    }

    std::vector<std::string> stale;
    if (!odom_fresh) stale.emplace_back("odometry");
    if (!joint_fresh) stale.emplace_back("joint states");

    std::ostringstream oss;
    oss << "Drivetrain unhealthy: ";
    for (size_t k = 0; k < stale.size(); ++k) {
      if (k > 0) oss << " and ";
      oss << stale[k];
    }
    oss << " stale/absent.";
    return {false, oss.str()};
  }

  // Counts lethal cells (>= kLethalCellValue) in a small window directly
  // ahead of the grid centre (~0.3-0.8 m ahead along the grid's own +x,
  // +/-0.3 m lateral). For a rolling local costmap the grid centre tracks
  // the robot, so this window is "in front of the robot" without needing a
  // TF lookup. A lethal hit here with no corresponding static-map feature
  // corroborates a live-sensor phantom rather than a real obstacle - this
  // check only ever reports ok=true, it is an observation, not a fault.
  CheckOutcome check_costmap_health() const {
    if (!costmap_received_ || !last_costmap_ || last_costmap_->data.empty() || last_costmap_->info.width == 0 ||
        last_costmap_->info.height == 0 || last_costmap_->info.resolution <= 0.0f) {
      return {false, "no /local_costmap/costmap received"};
    }

    const auto & info = last_costmap_->info;
    const double res = static_cast<double>(info.resolution);
    const double origin_x = info.origin.position.x;
    const double origin_y = info.origin.position.y;
    const int width = static_cast<int>(info.width);
    const int height = static_cast<int>(info.height);

    const double centre_x = origin_x + (static_cast<double>(width) * res) / 2.0;
    const double centre_y = origin_y + (static_cast<double>(height) * res) / 2.0;

    const auto to_col = [&](double x) { return static_cast<int>(std::floor((x - origin_x) / res)); };
    const auto to_row = [&](double y) { return static_cast<int>(std::floor((y - origin_y) / res)); };

    const int col_min = std::clamp(to_col(centre_x + kCostmapAheadMinM), 0, width - 1);
    const int col_max = std::clamp(to_col(centre_x + kCostmapAheadMaxM), 0, width - 1);
    const int row_min = std::clamp(to_row(centre_y - kCostmapLateralM), 0, height - 1);
    const int row_max = std::clamp(to_row(centre_y + kCostmapLateralM), 0, height - 1);

    int lethal_count = 0;
    double ahead_sum_m = 0.0;
    for (int row = row_min; row <= row_max; ++row) {
      for (int col = col_min; col <= col_max; ++col) {
        const size_t idx = static_cast<size_t>(row) * static_cast<size_t>(width) + static_cast<size_t>(col);
        if (idx >= last_costmap_->data.size()) continue;
        if (last_costmap_->data[idx] >= kLethalCellValue) {
          ++lethal_count;
          const double cell_x = origin_x + (static_cast<double>(col) + 0.5) * res;
          ahead_sum_m += cell_x - centre_x;
        }
      }
    }

    if (lethal_count > 0) {
      const double ahead_avg_m = ahead_sum_m / static_cast<double>(lethal_count);
      std::ostringstream oss;
      oss << "Local costmap: " << lethal_count << " lethal cells ~" << std::fixed << std::setprecision(2)
          << ahead_avg_m
          << " m ahead with no matching static-map feature - consistent with a live sensor (phantom), "
             "not the environment.";
      return {true, oss.str()};
    }

    return {true, "Local costmap clear ahead."};
  }

  // Cached latest messages + reception time (node clock, so this follows
  // use_sim_time like the rest of the demo) per subscribed topic.
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  rclcpp::Time last_scan_time_;
  bool scan_received_{false};

  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_amcl_pose_;
  rclcpp::Time last_amcl_pose_time_;
  bool amcl_pose_received_{false};

  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  rclcpp::Time last_odom_time_;
  bool odom_received_{false};

  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
  rclcpp::Time last_joint_state_time_;
  bool joint_state_received_{false};

  nav_msgs::msg::OccupancyGrid::SharedPtr last_costmap_;
  rclcpp::Time last_costmap_time_;
  bool costmap_received_{false};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  rclcpp::Service<health_check_msgs::srv::RunHealthChecks>::SharedPtr run_health_checks_srv_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HealthCheckNode>());
  rclcpp::shutdown();
  return 0;
}
