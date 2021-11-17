// Copyright (c) 2021 Khaled SAAD, Jose M. TORRES-CAMARA and Marwan TAHER
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
// limitations under the License. Reserved.

#ifndef NAV2_LOCALIZATION__NAV2_LOCALIZATION_HPP_
#define NAV2_LOCALIZATION__NAV2_LOCALIZATION_HPP_

#include <string>
#include <vector>  // For vector<>
#include <memory>  // For shared_ptr<>

#include "nav2_util/lifecycle_node.hpp"
#include "nav_2d_utils/odom_subscriber.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_localization/plugins/sample_motion_models/sample_motion_model_base.hpp"
#include "nav2_localization/plugins/matchers/matcher2d_base.hpp"
#include "nav2_localization/plugins/solvers/solver_base.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "laser_geometry/laser_geometry.hpp"


namespace nav2_localization
{

class LocalizationServer : public nav2_util::LifecycleNode
{
public:
  /**
  * @brief Constructor for nav2_localization::LocalizationServer
  */
  LocalizationServer();

  /**
  * @brief Destructor for nav2_localization::LocalizationServer
  */
  ~LocalizationServer();

protected:
  /**
   * @brief Configures server parameters and member variables
   *
   * Configures motion model and matcher plugins; Initialize odom subscriber.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   * @throw pluginlib::PluginlibException When failed to initialize motion
   * model or matcher plugins
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activates member variables
   *
   * Activates motion model and matcher.
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Deactivates member variables
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Calls clean up states and resets member variables.
   *
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Called when in Shutdown state
   * @param state LifeCycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Initializes the publishers and subscribers
   */
  void initPubSub();

  /**
   * @brief Initializes the member variables required to transform between coordinate frames
   */
  void initTransforms();

  /**
   * @brief Initializes the scan message filter
   */
  void initMessageFilters();

  /**
   * @brief Initializes plugins
   */
  void initPlugins();

  /**
   * @brief Callback when the map is received
   * @param msg pointer to the received map message
   */
  void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief Callback when a LaserScan is received. It will convert it to a PC and use the callback for generic scans
   * @param scan pointer to the received LaserScan message
   */
  void laserAndOdomReceived(
    sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan,
    nav_msgs::msg::Odometry::ConstSharedPtr odom);

  /**
   * @brief Callback when the scan is received
   * @param scan pointer to the received PointCloud2 message
   * @param odom pointer to the received odometry message
   */
  void ponitCloudAndOdomReceived(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr scan,
    nav_msgs::msg::Odometry::ConstSharedPtr odom);

  /**
   * @brief Callback when the initial pose of the robot is received
   * @param init_pose pointer to the received pose
   */
  void initialPoseReceived(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr init_pose);

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr
    initial_pose_sub_;

  // Map
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr map_sub_;
  bool first_map_received_{false};

  // Parameters
  std::string scan_topic_;
  std::string odom_topic_;
  std::string pointcloud_topic_;

  // Transforms
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string map_frame_id_;
  tf2::Duration transform_tolerance_;

  // Message filters
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser_scan_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pointcloud_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan,
      nav_msgs::msg::Odometry> laser_odom_policy;
  std::shared_ptr<message_filters::Synchronizer<laser_odom_policy>> laser_odom_sync_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
      nav_msgs::msg::Odometry> pointcloud_odom_policy;
  std::shared_ptr<message_filters::Synchronizer<pointcloud_odom_policy>> pointcloud_odom_sync_;

  laser_geometry::LaserProjection laser_to_pc_projector_;

  // Sample Motion Model Plugin
  pluginlib::ClassLoader<nav2_localization::SampleMotionModel> sample_motion_model_loader_;
  nav2_localization::SampleMotionModel::Ptr sample_motion_model_;
  std::string default_sample_motion_model_id_;
  std::string sample_motion_model_id_;
  std::string sample_motion_model_type_;

  // Matcher Plugin
  pluginlib::ClassLoader<nav2_localization::Matcher2d> matcher2d_loader_;
  nav2_localization::Matcher2d::Ptr matcher2d_;
  std::string default_matcher2d_id_;
  std::string matcher2d_id_;
  std::string matcher2d_type_;

  // Solver Plugin
  pluginlib::ClassLoader<nav2_localization::Solver> solver_loader_;
  nav2_localization::Solver::Ptr solver_;
  std::string default_solver_id_;
  std::string solver_id_;
  std::string solver_type_;

  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> localization_ids_;

  // Initial pose
  bool initial_pose_set_;
  bool initial_odom_set_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    estimated_pose_pub_;
};

}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__NAV2_LOCALIZATION_HPP_
