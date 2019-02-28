/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef NAV2_AMCL__AMCL_NODE_HPP_
#define NAV2_AMCL__AMCL_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.h"
#include "nav2_lifecycle/lifecycle_node.hpp"
#include "nav2_tasks/map_service_client.hpp"
#include "nav2_util/motion_model/motion_model.hpp"
#include "nav2_util/sensors/laser/laser.hpp"
#include "nav_msgs/srv/set_map.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop

#define NEW_UNIFORM_SAMPLING 1

namespace nav2_amcl
{

class AmclNode : public nav2_lifecycle::LifecycleNode
{
public:
  AmclNode();
  ~AmclNode();

protected:
  // Implement the lifecycle interface
  nav2_lifecycle::CallbackReturn onConfigure(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onActivate(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onDeactivate(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn onCleanup(const rclcpp_lifecycle::State & state) override;

  // Since the sensor data from gazebo or the robot is not lifecycle enabled, we won't
  // respond until we're in the active state
  std::atomic<bool> active{false};

  // Map-related
  void initMap();
  nav2_tasks::MapServiceClient map_client_{"amcl"};
  map_t * map_{nullptr};
  map_t * convertMap(const nav_msgs::msg::OccupancyGrid & map_msg);
#if NEW_UNIFORM_SAMPLING
  static std::vector<std::pair<int, int>> free_space_indices;
#endif

  // Transforms
  void initTransforms();
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  bool sent_first_transform_{false};
  bool latest_tf_valid_{false};
  tf2::Transform latest_tf_;
  void waitForTransforms();

  // Message filters
  void initMessageFilters();
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser_scan_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> laser_scan_filter_;
  message_filters::Connection laser_scan_connection_;

  // Publishers and subscribers
  void initPubSub();
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr initial_pose_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;
  void initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan);

  // Services and service callbacks
  void initServices();
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;
  void globalLocalizationCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_update_srv_;  // to let amcl update samples without requiring motion
  void nomotionUpdateCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);
  bool force_update_{false}; // Nomotion update control. Used to temporarily let amcl update samples even when no motion occurs

  // Odometry
  void initOdometry();
  std::unique_ptr<nav2_util::MotionModel> motion_model_;
  geometry_msgs::msg::PoseStamped latest_odom_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped last_published_pose_;
  double init_pose_[3]; // Initial robot pose
  double init_cov_[3];
  bool getOdomPose(     // Helper to get odometric pose from transform system
    geometry_msgs::msg::PoseStamped & pose,
    double & x, double & y, double & yaw,
    const rclcpp::Time & t, const std::string & f);
  std::atomic<bool> first_pose_sent_;

  // Particle filter
  void initParticleFilter();
  static pf_vector_t uniformPoseGenerator(void * arg); // Pose-generating function used to uniformly distribute particles over the map
  pf_t * pf_{nullptr};
  bool pf_init_;
  pf_vector_t pf_odom_pose_;
  int resample_count_{0};

  // Laser scan related
  void initLaserScan();
  const char * scan_topic_{"scan"};
  nav2_util::Laser * createLaserObject();
  int scan_error_count_{0};
  std::vector<nav2_util::Laser *> lasers_;
  std::vector<bool> lasers_update_;
  std::map<std::string, int> frame_to_laser_;
  rclcpp::Time last_laser_received_ts_;
  void checkLaserReceived();
  std::chrono::seconds laser_check_interval_; // TODO: not initialized

  // Node parameters (initialized via initParameters)
  void initParameters();
  double        alpha1_;
  double        alpha2_;
  double        alpha3_;
  double        alpha4_;
  double        alpha5_;
  std::string   base_frame_id_;
  double        beam_skip_distance_;
  double        beam_skip_error_threshold_;
  double        beam_skip_threshold_;
  bool          do_beamskip_;
  std::string   global_frame_id_;
  double        lambda_short_;
  double        laser_likelihood_max_dist_;
  double        laser_max_range_;
  double        laser_min_range_;
  //TODO: need both?
    std::string laser_model_type_;
    std::string sensor_model_type_;
  int           max_beams_;
  int           max_particles_;
  int           min_particles_;
  std::string   odom_frame_id_;
  double        pf_err_;
  double        pf_z_;
  double        alpha_fast_;
  double        alpha_slow_;
  int           resample_interval_;
  std::string   robot_model_type_;
  tf2::Duration save_pose_period;
  double        sigma_hit_;
  bool          tf_broadcast_;
  tf2::Duration transform_tolerance_;
  double        a_thresh_;
  double        d_thresh_;
  double        z_hit_;
  double        z_max_;
  double        z_short_;
  double        z_rand_;
};

}  // namespace nav2_amcl

#endif  // NAV2_AMCL__AMCL_NODE_HPP_
