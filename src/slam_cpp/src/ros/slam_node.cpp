// Copyright 2026 Seif ElShafey
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// slam_node.cpp: ROS 2 node for real-time FastSLAM 2.0.
// Subscribes to /odometry/filtered and /scan.
// Publishes /slam/map, /slam/particles, /slam/best_pose, and map->odom TF.
// Frame names come from the map_frame and odom_frame parameters.
// Single-threaded executor; all callbacks are mutually exclusive.

/**
 * @file slam_node.cpp
 * @brief ROS 2 node that runs the particle-filter grid SLAM on odometry and laser scans.
 */

#include <algorithm>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "slam_cpp/check.hpp"
#include "slam_cpp/slam_config.hpp"
#include "slam_cpp/slam_types.hpp"
#include "slam_cpp/odometry.hpp"
#include "slam_cpp/particle_filter.hpp"
#include "slam_cpp/scan_matching.hpp"
#include "slam_cpp/sigmoid_lut.hpp"

namespace slam
{

// Largest distance between neighbouring kept scan points, in cells, before the node warns.
// Measured on simulated empty rectangular rooms: matching was reliable with kept points
// under 2 cells apart on the walls, and the start pose stuck from about 3 cells. A guide,
// hence a warning and not an error.
/// Largest spacing between kept scan points at the median range, in cells, before the node
/// warns that the scan is sparse for the cell size.
constexpr double kMaxPointSpacingCells = 2.0;

// Extract yaw from a quaternion (rotation about Z axis only).
static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

// ============================================================================
// SlamNode: orchestrates the FastSLAM 2.0 pipeline.
//
// Callback flow:
//   odom_callback (~50-100 Hz) → accumulate dx, dy, dth
//   scan_callback (~10 Hz)     → first valid scan: update_and_resample only;
//                                later scans, once the motion threshold is met:
//                                decompose motion, predict, update_and_resample
//                                then publish particles and best pose at the scan stamp
//   publish_callback (2 Hz)    → publish map and map->odom TF
// ============================================================================

/**
 * @brief ROS 2 node that builds an occupancy-grid map and estimates the robot pose
 *        from odometry and 2D laser scans.
 *
 * The node registers the first valid scan to define the map frame, then runs one
 * filter update each time the odometry has moved the robot past a motion threshold
 * since the last update. It publishes the particle poses and the pose of the best
 * particle once per update, and the map and the transform at a fixed rate. The node runs
 * on a single-threaded executor.
 *
 * Frames assumed: the lidar sits at the origin of the robot base frame with no yaw,
 * so scan points are used as given and no transform is looked up. Odometry poses are
 * read as (x, y, yaw) from the message pose and must be consistent with each other
 * over time. The map frame is the robot pose at the first update.
 *
 * @par Subscribed topics
 * | Topic | Type | QoS |
 * |---|---|---|
 * | `odom_topic` (default `/odometry/filtered`) | `nav_msgs/msg/Odometry` | reliable, depth 10 |
 * | `scan_topic` (default `/scan`) | `sensor_msgs/msg/LaserScan` | sensor data (best effort) |
 *
 * @par Published topics
 * | Topic | Type | QoS |
 * |---|---|---|
 * | `/slam/map` | `nav_msgs/msg/OccupancyGrid` | reliable, transient local, depth 1 |
 * | `/slam/particles` | `geometry_msgs/msg/PoseArray` | reliable, volatile, depth 1 |
 * | `/slam/best_pose` | `geometry_msgs/msg/PoseStamped` | reliable, volatile, depth 1 |
 *
 * `/slam/particles` and `/slam/best_pose` are published once per filter update and are
 * stamped with the stamp of the scan that update used. `/slam/map` is published at
 * `map_publish_rate` and stamped with the node clock. All carry `map_frame` as their
 * frame. Map cells are -1 for unknown, otherwise the occupancy probability in
 * percent (0 to 100). Poses are the particle poses in the map frame.
 *
 * @par Transform
 * Broadcasts the transform from `map_frame` to `odom_frame` at `map_publish_rate`,
 * stamped with the node clock. It is computed from the best particle pose and the
 * odometry pose recorded at the same update, so it stays constant between updates.
 * @note `/slam/best_pose` and `/slam/particles` carry the stamp of the scan, while the
 * transform carries the node clock at publish time. A consumer that looks up the
 * transform at the pose stamp can get an extrapolation error, because the transform is
 * published at `map_publish_rate` and may be older than the scan.
 *
 * At the first scan used, the node logs a warning once if the kept scan points are more
 * than 2 cells apart at the median range.
 * Scans received before the first odometry message are ignored. Nothing is published
 * before the first update. While the node clock runs backwards, the map and the
 * transform are not published until it passes the last publish time again.
 *
 * @par Parameters
 * Each parameter is declared with the default shown. Parameters that map to
 * `SlamConfig` are checked by `validate` when the node starts; an invalid value
 * makes construction throw. See `SlamConfig` for the meaning of each field.
 *
 * | Parameter | Default | Unit | Valid range |
 * |---|---|---|---|
 * | `num_particles` | 80 | count | >= 1 |
 * | `map_size` | 50.0 | m | > 0; `map_size / map_resolution` an even whole number of cells, 2 to 20000 |
 * | `map_resolution` | 0.1 | m per cell | > 0 |
 * | `log_odds_scale` | 20.0 | stored units per log-odds unit | 0 < value <= 127 |
 * | `lo_occupied` | 0.85 | log-odds | > 0 |
 * | `lo_free` | -0.4 | log-odds | < 0 |
 * | `lo_max` | 5.0 | log-odds | > 0 |
 * | `lo_min` | -5.0 | log-odds | < 0 |
 * | `alpha1` | 0.075 | unitless | >= 0 |
 * | `alpha2` | 0.03 | rad per m | >= 0 |
 * | `alpha3` | 0.075 | unitless | >= 0 |
 * | `alpha4` | 0.03 | m per rad | >= 0 |
 * | `sigma_hit` | 0.05 | unitless (occupancy probability) | > 0 |
 * | `score_stride` | 2 | scan points | >= 1 |
 * | `lm_max_iters` | 10 | iterations | >= 1 |
 * | `lm_max_sub_iters` | 3 | iterations | >= 1 |
 * | `lm_lambda_init` | 0.01 | unitless | >= `lm_lambda_min` |
 * | `lm_lambda_min` | 1e-7 | unitless | > 0 |
 * | `lm_lambda_max` | 1e4 | unitless | >= `lm_lambda_init` |
 * | `lm_lambda_up` | 10.0 | factor | > 1 |
 * | `lm_lambda_down` | 0.1 | factor | strictly between 0 and 1 |
 * | `motion_threshold_trans` | 0.05 | m | >= 0 |
 * | `motion_threshold_rot` | 0.05 | rad | >= 0 |
 * | `map_update_dist` | 0.20 | m | >= 0 |
 * | `map_update_angle` | 0.25 | rad | >= 0 |
 * | `obs_sigma_gain` | 3.0 | unitless | > 0 |
 * | `scan_stride` | 1 | beams | >= 1 |
 * | `quality_cache_interval` | 5 | updates | >= 1 |
 * | `odom_topic` | `/odometry/filtered` | topic name | none enforced |
 * | `scan_topic` | `/scan` | topic name | none enforced |
 * | `map_frame` | `map` | frame id | not empty, different from `odom_frame` |
 * | `odom_frame` | `odom` | frame id | not empty, different from `map_frame` |
 * | `map_publish_rate` | 2.0 | Hz | > 0 |
 *
 * Additionally, `log_odds_scale` times each of `lo_occupied`, `lo_free`, `lo_max`
 * and `lo_min` must round into [-127, 127], and `lo_occupied` and `lo_free` must not
 * round to 0. An empty or equal frame name, or a `map_publish_rate` not above 0, also
 * makes construction throw.
 */
class SlamNode : public rclcpp::Node
{
public:
  /**
   * @brief Declare the parameters, build the filter and create the subscriptions,
   *        publishers, transform broadcaster and publish timer.
   *
   * @throws std::invalid_argument if a parameter is outside its valid range.
   */
  SlamNode()
  : Node("slam_node")
  {
    SlamConfig config = load_config();

    pf_ = std::make_unique<ParticleFilter>(config);
    sigmoid_ = SigmoidLUT(config.log_odds_scale);
    RCLCPP_INFO(
      get_logger(), "ParticleFilter initialized: %d particles, %.1fm map, %.2fm/cell",
      config.num_particles, config.map_size, config.map_resolution);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&SlamNode::odom_callback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SlamNode::scan_callback, this, std::placeholders::_1));

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/slam/map", rclcpp::QoS(
        1).transient_local());
    particles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/slam/particles", 1);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/slam/best_pose", 1);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Use node clock (not wall clock) for sim time compatibility.
    double publish_rate = this->get_parameter("map_publish_rate").as_double();
    SLAM_CHECK(
      publish_rate > 0.0,
      "map_publish_rate must be > 0, got " + std::to_string(publish_rate));
    auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / publish_rate));
    publish_timer_ = rclcpp::create_timer(
      this, this->get_clock(), period,
      std::bind(&SlamNode::publish_callback, this));

    RCLCPP_INFO(
      get_logger(), "SlamNode ready. Subscribing to [%s] and [%s]",
      odom_topic_.c_str(), scan_topic_.c_str());
  }

private:
  // Load all ROS 2 parameters into SlamConfig.
  SlamConfig load_config()
  {
    SlamConfig c;
    c.num_particles = this->declare_parameter("num_particles", c.num_particles);
    c.map_size = this->declare_parameter("map_size", c.map_size);
    c.map_resolution = this->declare_parameter("map_resolution", c.map_resolution);
    c.log_odds_scale = this->declare_parameter("log_odds_scale", c.log_odds_scale);
    c.lo_occupied = this->declare_parameter("lo_occupied", c.lo_occupied);
    c.lo_free = this->declare_parameter("lo_free", c.lo_free);
    c.lo_max = this->declare_parameter("lo_max", c.lo_max);
    c.lo_min = this->declare_parameter("lo_min", c.lo_min);
    c.alpha1 = this->declare_parameter("alpha1", c.alpha1);
    c.alpha2 = this->declare_parameter("alpha2", c.alpha2);
    c.alpha3 = this->declare_parameter("alpha3", c.alpha3);
    c.alpha4 = this->declare_parameter("alpha4", c.alpha4);
    c.sigma_hit = this->declare_parameter("sigma_hit", c.sigma_hit);
    c.score_stride = this->declare_parameter("score_stride", c.score_stride);
    c.lm_max_iters = this->declare_parameter("lm_max_iters", c.lm_max_iters);
    c.lm_max_sub_iters = this->declare_parameter("lm_max_sub_iters", c.lm_max_sub_iters);
    c.lm_lambda_init = this->declare_parameter("lm_lambda_init", c.lm_lambda_init);
    c.lm_lambda_min = this->declare_parameter("lm_lambda_min", c.lm_lambda_min);
    c.lm_lambda_max = this->declare_parameter("lm_lambda_max", c.lm_lambda_max);
    c.lm_lambda_up = this->declare_parameter("lm_lambda_up", c.lm_lambda_up);
    c.lm_lambda_down = this->declare_parameter("lm_lambda_down", c.lm_lambda_down);
    c.motion_threshold_trans = this->declare_parameter(
      "motion_threshold_trans",
      c.motion_threshold_trans);
    c.motion_threshold_rot =
      this->declare_parameter("motion_threshold_rot", c.motion_threshold_rot);
    c.map_update_dist = this->declare_parameter("map_update_dist", c.map_update_dist);
    c.map_update_angle = this->declare_parameter("map_update_angle", c.map_update_angle);
    c.obs_sigma_gain = this->declare_parameter("obs_sigma_gain", c.obs_sigma_gain);
    c.scan_stride = this->declare_parameter("scan_stride", c.scan_stride);
    c.quality_cache_interval = this->declare_parameter(
      "quality_cache_interval",
      c.quality_cache_interval);

    odom_topic_ = this->declare_parameter("odom_topic", std::string("/odometry/filtered"));
    scan_topic_ = this->declare_parameter("scan_topic", std::string("/scan"));
    map_frame_ = this->declare_parameter("map_frame", std::string("map"));
    odom_frame_ = this->declare_parameter("odom_frame", std::string("odom"));
    SLAM_CHECK(!map_frame_.empty(), "map_frame must not be empty");
    SLAM_CHECK(!odom_frame_.empty(), "odom_frame must not be empty");
    SLAM_CHECK(
      map_frame_ != odom_frame_,
      "map_frame and odom_frame must differ, both are '" + map_frame_ + "'");
    this->declare_parameter("map_publish_rate", 2.0);

    return c;
  }

  // Accumulate odometry deltas between laser scans.
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double curr_x = msg->pose.pose.position.x;
    double curr_y = msg->pose.pose.position.y;
    double curr_th = yaw_from_quaternion(msg->pose.pose.orientation);

    if (!odom_initialized_) {
      prev_x_ = curr_x;
      prev_y_ = curr_y;
      prev_th_ = curr_th;
      start_seg_th_ = curr_th;
      odom_initialized_ = true;
      return;
    }

    double dx = curr_x - prev_x_;
    double dy = curr_y - prev_y_;
    double dth = normalize_angle(curr_th - prev_th_);

    accum_dx_ += dx;
    accum_dy_ += dy;
    accum_dth_ += dth;
    accum_dth_ = normalize_angle(accum_dth_);

    prev_x_ = curr_x;
    prev_y_ = curr_y;
    prev_th_ = curr_th;
  }

  // Register the first valid scan at once; afterwards run SLAM when the
  // accumulated motion exceeds the threshold.
  // A scan before the first odometry message, or with no valid beam, changes nothing.
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!odom_initialized_) {
      return;
    }

    // Motion decomposition: rotation-translation-rotation, with reverse detection.
    const MotionDelta motion =
      decompose_motion(accum_dx_, accum_dy_, accum_dth_, start_seg_th_);

    // After the first scan, run SLAM only when motion exceeds threshold.
    if (first_scan_registered_ &&
      !(motion.trans > pf_->config().motion_threshold_trans ||
      std::abs(accum_dth_) > pf_->config().motion_threshold_rot))
    {
      return;
    }

    auto local_scans = extract_local_scan(
      msg->ranges.data(),
      static_cast<int>(msg->ranges.size()),
      msg->angle_min,
      msg->angle_increment,
      msg->range_min,
      msg->range_max,
      pf_->config().scan_stride);

    // No valid beam: keep the accumulated motion for the next scan.
    if (local_scans.rows() == 0) {
      return;
    }

    // Once, at the scan that defines the map frame.
    if (!first_scan_registered_) {
      const double spacing =
        median_point_spacing(local_scans, msg->angle_increment * pf_->config().scan_stride);
      const double resolution = pf_->config().map_resolution;
      if (spacing > kMaxPointSpacingCells * resolution) {
        RCLCPP_WARN(
          get_logger(),
          "Scan points are %.3f m apart at the median range, which is %.1f cells of %.3f m. "
          "Above %.1f cells the map of the first scans can be too thin for matching and "
          "the pose can stay behind. Lower scan_stride or raise map_resolution.",
          spacing, spacing / resolution, resolution, kMaxPointSpacingCells);
      }
    }

    // The first scan defines the map frame: no motion is applied before it.
    if (first_scan_registered_) {
      pf_->predict(motion.trans, motion.rot1, motion.rot2, motion.direction);
    }
    pf_->update_and_resample(local_scans);
    first_scan_registered_ = true;
    slam_counter_++;

    // Reset accumulators for next segment, and keep the odometry pose
    // they were measured to: it belongs to the filter pose of this update.
    start_seg_th_ = prev_th_;
    accum_dx_ = 0.0;
    accum_dy_ = 0.0;
    accum_dth_ = 0.0;
    update_odom_ = Eigen::Vector3d(prev_x_, prev_y_, prev_th_);

    size_t best_idx = pf_->best_particle_index();
    publish_particles(msg->header.stamp);
    publish_best_pose(msg->header.stamp, best_idx);

    // Throttled status log (at most once every 2 seconds).
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "SLAM #%d | best=%zu | Neff=%.1f | mq=%.4f",
      slam_counter_, best_idx,
      1.0 / pf_->weights().squaredNorm(),
      pf_->grids()[best_idx].map_quality());
  }

  // Publish map and map->odom TF.
  void publish_callback()
  {
    if (slam_counter_ == 0) {
      return;
    }

    // Guard against sim time reset (Gazebo restart).
    rclcpp::Time now = this->get_clock()->now();
    if (last_publish_time_.nanoseconds() > 0 && now < last_publish_time_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Sim time went backward, skipping publish");
      return;
    }
    last_publish_time_ = now;

    size_t best_idx = pf_->best_particle_index();

    publish_map(now, best_idx);
    broadcast_tf(now, best_idx);
  }

  // Convert best particle's grid to nav_msgs::msg::OccupancyGrid.
  // Cell value 0 → -1 (unknown); any other value v → round(100 * p), with
  // p = sigmoid(v / log_odds_scale) the occupancy probability, in [0, 100].
  // Published via std::move for zero-copy transfer to middleware.
  void publish_map(const rclcpp::Time & stamp, size_t best_idx)
  {
    const auto & grid = pf_->grids()[best_idx];
    const auto & config = pf_->config();

    auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    msg->header.stamp = stamp;
    msg->header.frame_id = map_frame_;

    msg->info.resolution = static_cast<float>(config.map_resolution);
    msg->info.width = static_cast<uint32_t>(grid.cols());
    msg->info.height = static_cast<uint32_t>(grid.rows());

    double half = config.map_size / 2.0;
    msg->info.origin.position.x = -half;
    msg->info.origin.position.y = -half;
    msg->info.origin.position.z = 0.0;
    msg->info.origin.orientation.w = 1.0;

    int total_cells = grid.rows() * grid.cols();
    msg->data.resize(static_cast<size_t>(total_cells));
    const int8_t * raw = grid.data();

    for (int i = 0; i < total_cells; ++i) {
      int8_t value = raw[i];
      if (value == 0) {
        msg->data[static_cast<size_t>(i)] = -1;
      } else {
        int64_t occ = std::clamp<int64_t>(std::lround(100.0 * sigmoid_(value)), 0, 100);
        msg->data[static_cast<size_t>(i)] = static_cast<int8_t>(occ);
      }
    }

    map_pub_->publish(std::move(msg));
  }

  void publish_particles(const rclcpp::Time & stamp)
  {
    auto msg = std::make_unique<geometry_msgs::msg::PoseArray>();
    msg->header.stamp = stamp;
    msg->header.frame_id = map_frame_;

    const auto & poses = pf_->poses();
    int n = static_cast<int>(poses.cols());
    msg->poses.resize(static_cast<size_t>(n));

    for (int i = 0; i < n; ++i) {
      auto & p = msg->poses[static_cast<size_t>(i)];
      p.position.x = poses(0, i);
      p.position.y = poses(1, i);
      p.position.z = 0.0;
      double yaw = poses(2, i);
      p.orientation.w = std::cos(yaw / 2.0);
      p.orientation.z = std::sin(yaw / 2.0);
    }

    particles_pub_->publish(std::move(msg));
  }

  void publish_best_pose(const rclcpp::Time & stamp, size_t best_idx)
  {
    const auto & poses = pf_->poses();
    Eigen::Index idx = static_cast<Eigen::Index>(best_idx);

    auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    msg->header.stamp = stamp;
    msg->header.frame_id = map_frame_;

    msg->pose.position.x = poses(0, idx);
    msg->pose.position.y = poses(1, idx);
    double yaw = poses(2, idx);
    msg->pose.orientation.w = std::cos(yaw / 2.0);
    msg->pose.orientation.z = std::sin(yaw / 2.0);

    pose_pub_->publish(std::move(msg));
  }

  // Broadcast map->odom TF.
  // T(map->odom) = T(map->base_link) * T(odom->base_link)^-1
  // = slam_pose * inverse(odom_pose)
  // slam_pose is the best particle's pose after the last update; odom_pose is
  // the odometry pose stored at that same update. Both belong to the same
  // moment, so the transform is constant between updates.
  void broadcast_tf(const rclcpp::Time & stamp, size_t best_idx)
  {
    const Eigen::Vector3d slam_pose =
      pf_->poses().col(static_cast<Eigen::Index>(best_idx));
    const Eigen::Vector3d tf = map_to_odom(slam_pose, update_odom_);
    double tf_x = tf.x();
    double tf_y = tf.y();
    double tf_th = tf.z();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = map_frame_;
    t.child_frame_id = odom_frame_;

    t.transform.translation.x = tf_x;
    t.transform.translation.y = tf_y;
    t.transform.translation.z = 0.0;

    t.transform.rotation.w = std::cos(tf_th / 2.0);
    t.transform.rotation.z = std::sin(tf_th / 2.0);

    tf_broadcaster_->sendTransform(t);
  }

  // --- Members ---
  std::unique_ptr<ParticleFilter> pf_;
  SigmoidLUT sigmoid_;  // cell value → occupancy probability, for the published map

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string odom_topic_;
  std::string scan_topic_;
  std::string map_frame_;
  std::string odom_frame_;

  // Odometry accumulation (single-threaded: no synchronization needed).
  bool odom_initialized_ = false;
  double prev_x_ = 0.0, prev_y_ = 0.0, prev_th_ = 0.0;
  double start_seg_th_ = 0.0;
  double accum_dx_ = 0.0, accum_dy_ = 0.0, accum_dth_ = 0.0;
  bool first_scan_registered_ = false;
  // Odometry pose (x, y, θ) at the last update; pairs with the filter pose.
  Eigen::Vector3d update_odom_ = Eigen::Vector3d::Zero();

  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  int slam_counter_ = 0;
};

}  // namespace slam

/// Runs the SLAM node until shutdown.
///
/// @param argc  Argument count from the command line.
/// @param argv  Arguments; ROS arguments such as `--ros-args` are consumed.
/// @return 0 on normal shutdown.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam::SlamNode>());
  rclcpp::shutdown();
  return 0;
}
